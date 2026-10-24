/* Copyright (c) 2020, Henk Stegeman and Edwin Freekenhorst

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
   HENK STEGEMAN AND EDWIN FREEKENHORST BE LIABLE FOR ANY CLAIM, DAMAGES OR
   OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
   ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
   DEALINGS IN THE SOFTWARE.
   ---------------------------------------------------------------------------

   3705_scan_T2.c: IBM 3705 Communication Scanner Type 2 simulator

   Some notes:
   1) The scanner has:
      - a ICW work register and is implemented in Ereg_Inp[0x44->0x47]
        Function PutICW will transfer ICW work reg to ICW storage[ABAR].
      - a ICW local storage register (See below)
        Function GetICW will transfer ICW storage[ABAR] to ICW input reg.
      - a ICW input register and is implemented in Ereg_Out[0x44/45/47]
   2) The 2 bits in SDF for Business Clock Osc selection bits are
      not implemented.  Reason: for programming simplicity.
   3) This code is pre-positined for multiple lines, but correctly only
      suitable for only ONE line.

   *** Input to CS2 (CCU output) Eregs ***
   Label      Ereg         Function
   --------------------------------------------------------------
   CMBAROUT   0x40         // ABAR Interface address
   CMADRSUB   0x41         // Scanner addr substitution.
   CMSCANLT   0x42         // Upper scan limit modification.
   CMCTL      0x43         // CA Address and ESC status.
   CMICWB0F   0x44         // ICW  0 THRU 15
   CMICWLP    0x45         // ICW 16 THRU 23
   CMICWS     0x46         // ICW 24 THRU 33
   CMICWB34   0x47         // ICW 34 THRU 43

   *** Output from CS2 (CCU input) Eregs ***
   Label      Ereg         Function
   --------------------------------------------------------------
   CMBARIN    0x40         // ABAR Interface address
              0x41         // Unused
              0x42         // Unused
   CMERREG    0x43         // Scan Error register
   CMICWB0F   0x44         // ICW  0 THRU 15
   CMICWLPS   0x45         // ICW 16 THRU 31
   CMICWDSP   0x46         // Display register
   CMICWB32   0x47         // ICW 32 THRU 45

                        PCF state
   +------------> +---->  [0] NO-OP
   |              |
   |              |
   |              L2 <--  [1] Set Mode - DTR on
   |              ^
   |              |
   |         +----<-----  [2] Monitor DSR
   |         |    |
   |         |    |
   |         L2   +-----  [3] Monitor DSR or RI on
   |         |
   |         |
   |         +--------->  [4] Monitor flag - Block DSR error
   |         +-----flag--/
   |         |
   |    +-------------->  [5] Monitor flag - Allow DSR error
   |    |    +-----flag--/
   |    |    v
   |    |    L2 ------->  [6] Receive Info - Block Data Interrupts
   |    |    ^    +------/
   |    |    |    L2
   |    |    |    +---->  [7] Receive Info - Allow Data Interrupts
   L2   |    +-----flag--/
   |    |
   |    L2   +-----CTS--  [8] Transmit Initial - turn RTS on
   |    |    |
   |    |    |
   |    |    +--------->  [9] Transmit Normal
   |    |
   |    |
   |    +-SDF is empty--  [C] Tx -> Rx turnaround RTS off
   |    |
   |    |
   |    +-SDF is empty--  [D] Tx -> Rx turnaround RTS on.
   |
   |
   +-- no DSR | no DCD--  [F] Disable

   L2 = Level 2 interrupt
   See IBM 3705 hardware documentation for more details.
*/

#include <pthread.h>
#include "sim_defs.h"
#include "i3705_defs.h"
#if defined(__GENODE__)
/* Genode has no SYS_gettid and the number is only ever printed. pthread_self
   is a pointer here, so it is cast rather than formatted as one. */
#include <pthread.h>
#define THREAD_ID()  ((long)(unsigned long)pthread_self())
#else
#include <sys/syscall.h>
#define THREAD_ID()  syscall(SYS_gettid)
#endif

#include "i3705_sdlc.h"

#if defined(__GENODE__)
#include "genode_trc.h"
#endif

/* SP_CR: the leading carriage return these instrument lines start with.
 *
 * It exists for a SIMH console, where the cursor sits part-way along a line
 * the simulator has just written and a report wants to begin at the left
 * margin. On Genode the same text goes to the system LOG, which is
 * line-based and prefixes every message with the component that wrote it --
 * and the capture pipeline ends a line on a carriage return as well. So the
 * return split each report in two: "[init -> 3705a]" alone on one line, and
 * the numbers on the next with nothing to say whose they were. With a trunk
 * PAIR that is not cosmetic: two controllers' reports interleave in one log
 * and 2,252 of them in cell trunktelem3 could only be attributed by pairing
 * each with the stranded prefix above it.
 *
 * Platform support, inert elsewhere: off Genode this is the "\r" it always
 * was.
 */
#if defined(__GENODE__)
#define SP_CR ""
#else
#define SP_CR "\r"
#endif

/* SCANPERF: is the 12 ms cycle wakeup latency, or work?
   Every previous measurement inferred it from the cycle RATE. This brackets
   the pacing sleep and the rest of the cycle separately with rdtsc -- a
   register read, because clock_gettime here cost more than the thing it was
   measuring and may go through the timer session. */
#include <stdint.h>
#include <time.h>
#include <unistd.h>
extern int16 ScanSpin;          /* deposit SCANSPIN 1 in the .cnf */
extern int16 ScanUs;            /* deposit SCANUS <us> in the .cnf   */
extern int16 L2Spin;            /* deposit L2SPIN 0 to restore sleeps */
/* PCF state occupancy, per scanner cycle, per line. Which state a line
   SITS in is the question a turnaround stall asks, and a full PCF trace
   is far too loud to answer it during a load. */
extern volatile unsigned long timer_ticks;   /* should be 10/s: the 3705's 100 ms timer */
volatile unsigned long pcf_hist[MAX_LINES][16];
/* Dwell: how long ONE stay in a state lasts, not how many cycles sit in it.
   A line parked for 1.5 s and a line flicking through the same state a
   thousand times look identical in the occupancy histogram above. */
volatile unsigned long long pcf_tsc[MAX_LINES][16];
volatile unsigned long      pcf_ent[MAX_LINES][16];
extern void genode_pace_us(unsigned us);  /* Timer session, microseconds */

/* TWCOUNT: how many TIMED WAITS each thread performs per second. Inspection
   has failed three times to say which thread degrades the scanner's sleep;
   this counts them instead. Index: 0 scanner, 1 panel, 2 cpu, 3 CA avail10,
   4 CA avail100, 5 LIB, 6 CA wait(), 7 idle-loader rests. */
volatile unsigned long tw_count[8];
extern volatile unsigned long lvl_hist[6];
extern volatile unsigned long long lvl_tsc[6];
extern volatile unsigned long l3_why[5];
extern volatile unsigned long x55_img[3];
/* Where the scanner BODY goes. SCANPERF says 3705b spends ~50% of each
   cycle in the body against 3705a's 1%, and the only blocking things in
   it are the two svc_req_L2 waits. [0] first loop, [1] second loop while
   RECEIVING (usleep(10), floored to 1 ms), [2] second loop otherwise
   (usleep(1000)). Each iteration of [1] or [2] costs about a millisecond,
   so these are milliseconds per second spent waiting for the CCU. */
volatile unsigned long l2wait[3];
static uint64_t sp_sleep, sp_body, sp_cycles, sp_last;
static inline uint64_t sp_tsc(void) { return __builtin_ia32_rdtsc(); }

/* SCANSPIN=1: pace with a bounded rdtsc spin instead of a timer sleep.
   usleep(500) measured 7-17 ms in this component -- 15-34x its argument --
   and giving the component three CPUs did not change it, so this is not
   contention. The spin costs CPU and keeps the emulation timing the scanner
   exists to model; that trade is the point of the experiment. */
static uint64_t sp_tsc_per_us;
static void sp_calibrate(void) {
   /* Calibrate against the CLOCK, not against the length of a sleep. The
      first attempt divided by the sleep's ARGUMENT and got 2298 tsc/us on
      one run and 4329 on the next, because usleep(50000) does not take
      50 ms here -- which is the whole problem being fixed. Measuring both
      ends makes the overrun irrelevant. */
   struct timespec c0, c1;
   uint64_t t0, t1, us;
   clock_gettime(CLOCK_MONOTONIC, &c0);
   t0 = sp_tsc();
   usleep(100000);
   t1 = sp_tsc();
   clock_gettime(CLOCK_MONOTONIC, &c1);
   us = (uint64_t)(c1.tv_sec - c0.tv_sec) * 1000000ULL
      + (uint64_t)(c1.tv_nsec / 1000) - (uint64_t)(c0.tv_nsec / 1000);
   sp_tsc_per_us = us ? (t1 - t0) / us : 0;
   if (sp_tsc_per_us < 100) sp_tsc_per_us = 100;
   printf(SP_CR "SCANPERF: calibrated %llu tsc/us over %llu us\n",
          (unsigned long long)sp_tsc_per_us, (unsigned long long)us);
}
/* The TSC rate in MHz, for every figure in this file that is quoted per
   second or per microsecond. Those divisors used to be literals: 1980 for
   the TWCOUNT/s rates and 2100 for the per-pass microseconds, the first of
   them labelled "genherc's i3-8145U" in i3705_cpu.c. On a host whose TSC
   runs at a different rate every one of them is wrong by that ratio -- on
   ms01a, 2995 MHz against 1980, every TWCOUNT/s read 1.51x high, and a
   figure labelled per second was per 0.66 second. sp_calibrate() has been
   in this file all along, measuring tsc/us against CLOCK_MONOTONIC; it was
   simply only called when SCANSPIN was on. The lazy call here is a
   backstop: the scanner calibrates at start-up when the TWCOUNT bit is
   set, so the 100 ms this costs lands before any interval is being
   measured rather than inside one. */
static double sp_tsc_mhz(void) {
   if (!sp_tsc_per_us) sp_calibrate();
   return (double)sp_tsc_per_us;
}
/* Characterise usleep: actual vs requested, measured on CLOCK_MONOTONIC so
   no tsc calibration is involved. The question is whether the overrun is a
   fixed offset (a coarse wakeup) or proportional (a slow clock). */
static void sp_sleep_curve(void) {
   static const unsigned req[] = { 100, 500, 1000, 5000, 10000, 50000 };
   unsigned i;
   printf(SP_CR "SLEEPCURVE: requested -> actual (us)\n");
   {  /* libc vs the Timer session, in THIS component, same thread */
      int q;
      for (q = 0; q < 3; q++) {
         struct timespec x, y;
         long long lus, tus;
         clock_gettime(CLOCK_MONOTONIC, &x);
         usleep(500);
         clock_gettime(CLOCK_MONOTONIC, &y);
         lus = (long long)(y.tv_sec-x.tv_sec)*1000000LL + (y.tv_nsec-x.tv_nsec)/1000;
         clock_gettime(CLOCK_MONOTONIC, &x);
         genode_pace_us(500);
         clock_gettime(CLOCK_MONOTONIC, &y);
         tus = (long long)(y.tv_sec-x.tv_sec)*1000000LL + (y.tv_nsec-x.tv_nsec)/1000;
         printf(SP_CR "PACECMP: libc %lld us   timer %lld us\n", lus, tus);
      }
   }
   for (i = 0; i < sizeof(req)/sizeof(req[0]); i++) {
      struct timespec a, b;
      long long us;
      clock_gettime(CLOCK_MONOTONIC, &a);
      usleep(req[i]);
      clock_gettime(CLOCK_MONOTONIC, &b);
      us = (long long)(b.tv_sec - a.tv_sec) * 1000000LL
         + (b.tv_nsec - a.tv_nsec) / 1000;
      printf(SP_CR "SLEEPCURVE: %6u -> %8lld   overrun %+lld\n",
             req[i], us, us - (long long)req[i]);
   }
}

/*
 * WAIT FOR THE CCU TO TAKE THE LEVEL 2, WITHOUT A SYSCALL.
 *
 * The two loops below sit on the PER-CHARACTER path: the scanner hands the
 * CPU one character, raises L2, and cannot go on until the CPU has taken it.
 * Upstream spins there with usleep(10) in PCF 7 and usleep(1000) otherwise.
 *
 * On Genode both of those are libc nanosleep, which divides tv_nsec by
 * 1000000 and clamps to 1 ms -- MEASURED at 1.7-1.9 ms a call here, against
 * 65 us and 1.06 ms natively. That is a 28x penalty on usleep(10), paid once
 * per character, and an NCP load frame is 548 of them: 548 x 1.8 ms = 0.99 s
 * of pure sleep in a leg that natively costs 0.63 s in total.
 *
 * The Timer session does not fix this one. It is right for the scanner's
 * pace and the CPU's WAIT, which are hundreds of microseconds, but a 10 us
 * wait cannot afford a round trip -- an earlier attempt to route this loop
 * through genode_pace_us(10) made it ~50 round trips per character and was
 * reverted, which is why this looked like "the floor is not the bottleneck".
 *
 * So spin on the TSC and do not sleep at all below the cap, then fall back to
 * the original sleep so a wait that is genuinely long still yields the CPU.
 * svc_req_L2 is read through a volatile cast: it is a plain int8 written by
 * another thread, and unlike usleep() a spin gives the compiler nothing to
 * stop it hoisting the load out of the loop.
 *
 * A PURE SPIN HERE IS WORSE THAN THE SLEEP IT REPLACES. Measured: the
 * remote load went from 404 s to 577 s. The thread this loop waits for is
 * the emulated CPU, and on a two-core machine also carrying Hercules, the
 * other 3705 and the relay, busy-waiting for it starves the very thread that
 * would end the wait. usleep()'s 1.8 ms was buying something real -- it
 * yielded.
 *
 * So yield instead of burning. The floor to aim at is the CPU's own wake
 * latency (~0.5 ms through its Timer session), not zero: below that there is
 * nothing to win, because the CPU cannot answer sooner.
 *
 * DEPOSIT L2SPIN 0 IN THE .cnf to get the original sleeps back.
 */
/*
 * WAIT FOR THE CCU TO TAKE THE LEVEL 2 -- ON A CONDITION VARIABLE.
 *
 * This loop is on the PER-CHARACTER path: the scanner hands the CPU one
 * character, raises L2, and cannot proceed until the CPU has taken it.
 * Upstream polls with usleep(10) in PCF 7 and usleep(1000) otherwise. On
 * Genode libc floors BOTH at about 1.8 ms (measured; natively 65 us and
 * 1.06 ms), and a frame is 548 characters.
 *
 * TWO BUSY-WAIT VERSIONS WERE TRIED AND BOTH WERE WORSE. Measured on the
 * remote NCP load, against 404 s for the usleep original:
 *
 *     bounded TSC spin      577 s   (+43%)
 *     sched_yield() loop    load never completed; the serial log went
 *                           silent for 34 minutes
 *
 * The reason is the same in both cases: the thread this loop waits for is
 * the emulated CPU, and on two cores also carrying Hercules, the second
 * 3705 and the relay, refusing to sleep starves the very thread that would
 * end the wait. usleep()'s 1.8 ms was buying something real.
 *
 * So do not poll at all. The CPU clears svc_req_L2 in one place
 * (i3705_cpu.c, "Reset L2 request flag") and signals here; this side blocks
 * until it does. The condition variable's own timeout granularity does not
 * matter, because the wake comes from the SIGNAL, not from the timeout --
 * which is exactly what neither sleeping nor spinning could give. The
 * timeout is only a safety net so a lost signal cannot wedge the scanner.
 *
 * DEPOSIT L2SPIN 0 IN THE .cnf to get the original usleep polling back.
 */
pthread_mutex_t l2_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  l2_cond = PTHREAD_COND_INITIALIZER;

static void sp_l2_wait(unsigned fallback_us)
{
   extern int8 svc_req_L2;

   if (!L2Spin) { usleep(fallback_us); return; }

   pthread_mutex_lock(&l2_lock);
   if (svc_req_L2 == ON) {
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      ts.tv_nsec += 5000000L;                 /* 5 ms safety net */
      if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
      pthread_cond_timedwait(&l2_cond, &l2_lock, &ts);
   }
   pthread_mutex_unlock(&l2_lock);
}

static void sp_pace(unsigned us) {
   if (!sp_tsc_per_us) sp_calibrate();
   {
      uint64_t const deadline = sp_tsc() + (uint64_t)us * sp_tsc_per_us;
      while (sp_tsc() < deadline) { }
   }
}
#include "i3705_scanner.h"
#include "i3705_Eregs.h"               /* External regs defs */
#include <signal.h>
#include <ctype.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/syscall.h>

#define BUFFER_SIZE   16384            /* Line Send/Receive buffer  */
                                       /* Make sure this matches the Buffer of the attached device */
extern int32 debug_reg;
extern int32 Eregs_Inp[];
extern int32 Eregs_Out[];
extern int8  svc_req_L2;               /* SVC L2 request flag       */
extern int8  timer_req_L3;             /* SVC L3 timer request flag */
extern int8  pci_req_L3;               /* SVC L3 pci request flag   */
extern FILE *trace;
extern int32 lvl;
extern int32 cc;
extern uint8_t LIBready;               /* LIB structures allocated and initialized */

extern int Ireg_bit(int reg, int bit_mask);
extern void wait();

int8 Eflg_rcvd[MAX_LINES];             /* Eflag received                            */
int8 FCS_rcvd[MAX_LINES][2];           /* Frame Check Sequence bytes Received       */

int abar;                              /* Attach Buffer Addr Reg (020-1FF) to CS2   */
int abar_int;                          /* ABAR of line interrupt (020-1FF) from CS2 */

/* ICW Local Store Registers */
uint8_t icw_scf[MAX_LINES];            /* ICW[ 0- 7] SCF - Secondary Control Field  */
uint8_t icw_pdf[MAX_LINES];            /* ICW[ 8-15] PDF - Parallel Data Field      */
uint8_t icw_lcd[MAX_LINES];            /* ICW[16-19] LCD - Line Code Definer        */
uint8_t icw_pcf[MAX_LINES];            /* ICW[20-23] PCF - Primary Control Field    */
uint8_t icw_sdf[MAX_LINES];            /* ICW[24-31] SDF - Serial Data Field        */
                                       /* ICW[32-33] Not implemented (OSC sel bits) */
uint16_t icw_Rflags[MAX_LINES];        /* ICW[34-47] flags                          */

/* Additional icw fields for emulator  */
uint8_t icw_pcf_prev[MAX_LINES];       /* Previous icw_pcf                          */
uint8_t icw_lne_stat[MAX_LINES];       /* Line state: RESET, TX or RX               */
uint8_t icw_pcf_nxt[MAX_LINES];        /* What will be the next pcf value           */
uint8_t icw_pdf_reg[MAX_LINES];        /* Status ICW PDF reg: FILLED or EMPTY       */
extern uint8_t RS232[MAX_LINES];       /* RS232 signals                             */

int8 CS2_req_L2_int = OFF;
pthread_mutex_t icw_lock;              // ICW lock (0 - 45)
extern pthread_mutex_t rs232_lock;            // RS232 signal lock
extern void LIB_signals(int k);               // grant CTS on demand (i3705_lib.c)

// Trace variables
uint16_t Sdbg_reg = 0x00;              // Bit flags for debug/trace
uint16_t Sdbg_flag = OFF;              // 1 when Strace.log open
FILE  *S_trace;                        // Scanner trace file fd

// This table contains the SMD area addresses of each scanner line.
int8 line_smd_addr[48];

int line;                              // ICW table index pointer

void Put_ICW(int i);
void Get_ICW(int i);
void Init_ICW(int max);
void proc_LIBdisbuf(int line);
void proc_LIBtdata(unsigned char transmitChar, uint8_t state, int line);
int  proc_LIBrdata(unsigned char *receivedChar, uint8_t state, int line);
static char * TimeStamp();

void *CS2_thread(void *arg) {
   int Bptr = 0;                       // Tx/Rx buffer index pointer
   int i, c;
   register char *s;
   unsigned char receivedChar[MAX_LINES], transmitChar;
   int ret;

   fprintf (stderr, "\rCS-T2: Thread %ld started succesfully...\n", THREAD_ID());

   // core_id = 1 (CPU), 2 (SCAN), 3 (SDLC)

   /****************
   int core_id = 2;
   int num_cores = sysconf(_SC_NPROCESSORS_ONLN);

   if ((core_id > 0) && (core_id <= num_cores)) {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(core_id, &cpuset);
      pthread_t current_thread = pthread_self();
      pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
      fprintf (stderr, "\rCS-T2: Thread assigned to core #%1d.\n", core_id);
   }
   ***************/

   Init_ICW(MAX_LINES);                // Initialize scanner & buffers
   fprintf (stderr, "\rCS-T2: Scanner initialized with %d lines...\n", MAX_LINES);

   // ********************************************************************
   //  Scanner debug trace facility
   // ********************************************************************
   if (Sdbg_flag == OFF) {
      S_trace = NULL;
#if defined(__GENODE__)
      /*
       * THE TELEMETRY BUS FIRST. This port has no writable filesystem
       * on the machine, so the fopen below cannot succeed, and the
       * fallbacks underneath it send the trace to the Genode LOG --
       * shared, synchronous and truncated at 267 characters -- or to
       * /dev/null. genode_trc_open returns a line-buffered stream whose
       * writes land in a bounded ring in this component and leave it on
       * a crosslink, so nothing the emulator does waits on a consumer.
       * NULL means no trace destination was configured, and the
       * fallbacks below still apply. See genode_trc.h and
       * docs/trace-facilities.md in the port.
       */
      S_trace = genode_trc_open("trace_S.log");
#endif
      if (S_trace == NULL)
         S_trace = fopen("trace_S.log", "w");
#if defined(__GENODE__)
      /*
       * The trace files are relative paths, so they land at the VFS root,
       * and a component whose root is a set of ROM nodes has nowhere to
       * put them. fopen then returns NULL and the very next line is an
       * unguarded fprintf to it -- which is an "illegal READ at address
       * 0xa0" from whichever thread got there first, and says nothing
       * about a log file.
       *
       * /dev/log rather than /dev/null, so a scenario that asks for the
       * trace can actually read it: the VOLUME is gated by Sdbg_reg, which
       * is 0x00 here and stays that way unless the 3705 config file says
       * otherwise --
       *
       *     d debugS 02      trace scanner activities
       *     d debugS 04      trace line I/O buffers
       *     d debugS 06      both
       *
       * -- so the default costs one banner at start-up and nothing after.
       * /dev/null would keep every fprintf valid too, and did, but a trace
       * that is discarded is no use for finding out why a line is silent.
       */
      if (S_trace == NULL)
         S_trace = fopen("/dev/log", "w");
      if (S_trace == NULL)
         S_trace = fopen("/dev/null", "w");
#endif
      time_t tme = time(NULL);
      struct tm *currentTime = localtime(&tme);
      fprintf(S_trace, "\r     ****** 3705 SCANNER log file ******\n"
                       "     ******   Date: %02d/%02d/%04d    ******\n"
                       "\r     sim> d debugS 01 - spare \n"
                       "                   02 - trace scanner activities [scan_T2.c]\n"
                       "                   04 - trace line I/O buffers [lib.c]\n\n"
                       "\r"
                       "     All trace lines are prefixed with: #xxLny \n"
                       "        xx = 02 scanner activity\n"
                       "             04 Line I/O buffer content\n"
                       "        Ln = Line number n = 0 -> 9 "
                       "        y  = '>' Tx/request; '<' Rx/response \n\n",
                       currentTime->tm_mday, currentTime->tm_mon + 1, currentTime->tm_year + 1900);
      /*
       * Unbuffered, or the trace is invisible. S_trace is its own FILE* and
       * nothing in this file flushes it, so with the default buffering the
       * banner and the first several KB sit in libc until something else
       * forces them out -- which, in a simulator that does not exit, is
       * never. The controllers' traces call fflush after every write; this
       * one does not, so the buffering is turned off once instead.
       */
#if !defined(__GENODE__)
      setvbuf(S_trace, NULL, _IONBF, 0);
#else
      /*
       * NOT ON GENODE, AND IT COSTS EIGHT WRITES A LINE THERE. On that
       * platform S_trace is not a file: genode_trc_open() returns a funopen
       * stream whose write callback copies into a bounded ring inside this
       * component, and a separate drain thread empties the ring onto a
       * crosslink. So the reason for the override does not apply -- nothing
       * is waiting for this process to exit before the text can be read,
       * and the ring IS the buffer -- while the override's cost is paid on
       * the scanner's own thread:
       *
       *   * FreeBSD's libc, which that platform uses, flushes vfprintf's
       *     accumulated iovec every NIOV entries, and NIOV is 8
       *     (lib/libc/stdio/printfcommon.h). One entry per literal or
       *     conversion segment of the format.
       *   * With __SNBF set, __sfvwrite() calls the stream's write function
       *     ONCE PER IOVEC ENTRY (lib/libc/stdio/fvwrite.c).
       *   * The fast path that would coalesce an unbuffered fprintf into a
       *     single write, __sbprintf(), is gated on fp->_file >= 0
       *     (lib/libc/stdio/vfprintf.c) -- and funopen() sets _file to -1,
       *     so a funopen stream never takes it.
       *
       * MEASURED on a remote network control program load across a pair of
       * 3705s, 2026-10-24: 335,855 scanner trace lines against 2,679,800
       * calls to the ring, which is 7.98 per line. The channel adapter's
       * trace, on the same emulator in the same boot and left
       * line-buffered, ran at 1.04. Each of those calls takes a mutex,
       * signals a condition variable and copies the bytes, and the trace
       * path as a whole cost 36 seconds of a 323 second load.
       *
       * genode_trc_open() has already set _IOLBF -- one call per line --
       * so the right thing here is to leave its choice alone.
       */
#endif
      Sdbg_flag = ON;
      /* Calibrate the TSC now if the TWCOUNT/s report is wanted, so the
         100 ms it costs is spent before the scanner loop starts rather
         than inside a measured interval. See sp_tsc_mhz(). */
      if (Sdbg_reg & 0x08) sp_calibrate();
   }
   /*
    * DO NOT CLOBBER A DEPOSIT FROM THE CONFIG FILE.
    *
    * Sdbg_reg is a global, zero at load, and this assignment ran
    * unconditionally -- after the simulator had processed the config file. So
    *
    *     d DEBUGS 06
    *
    * in 3705.cnf, which is the documented way to ask for the scanner and line
    * traces (and which the banner a few lines above prints as the usage), was
    * silently discarded every time, and the trace could only ever be turned
    * on by typing at a sim> prompt this port does not have.
    *
    * Initialising a global that is already zero buys nothing; leaving a
    * deposit alone buys the trace.
    */
   if (Sdbg_reg == 0xFFFF)
      Sdbg_reg = 0x00;
   while (LIBready == 0)                              // If LIB structures not allocated and initialized....
      wait();                                         // ... wait until they are ready

   // ********************************************************************
   // Scanner loop starts here...
   // ********************************************************************
   while(1) {
      for (line = 0; line < MAX_LINES; line++) {     // Scan all lines

         if (icw_pcf[line] != icw_pcf_nxt[line]) {   // pcf changed by NCP ?
            if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {  // Trace scanner activities ?
               fprintf(S_trace, "%s #02L%1d> CS2[%1X]: NCP changed PCF to %1X\n",
                                 TimeStamp(), line, icw_pcf[line], icw_pcf_nxt[line]);
             }
            pthread_mutex_lock(&icw_lock);
            if (icw_pcf_nxt[line] == 0x0)            // NCP changed PCF to 0 ?
               icw_lne_stat[line] = RESET;           // Line state = RESET
            icw_pcf_prev[line] = icw_pcf[line];      // Save current pcf and
            icw_pcf[line] = icw_pcf_nxt[line];       // Set new current pcf
            pthread_mutex_unlock(&icw_lock);
         }


         /*
          * EVERY CONFIGURED LINE, not the first two. These three arrays
          * and the print loop below were dimensioned [2][16] with a
          * `line < 2` guard, so the BSC line -- index 3, the one that
          * carries the cluster controller whose activation is worth 5.5%
          * of this scanner's rate -- reported nothing at all. Every PCF
          * figure in port/results/scanner-body-slowdown.txt is line 20's
          * for that reason, and the 30us per cycle that separates the two
          * line regimes had to be attributed to line 23 by subtraction.
          * MAX_LINES is 4 and a 3705-II has four line sets, so this is
          * the whole of what the hardware can have.
          */
         /*
          * LINES 2 AND UP ARE BEHIND DEBUGS 0x10, and that is not
          * caution, and the figure is smaller than the first version of
          * this comment claimed. Two more lines mean two more PCF and two
          * more DWELL lines in every 500-cycle report, about 100 bytes,
          * and on the Genode port those go to a LOG written synchronously
          * down a 115200 baud serial console.
          *
          * MEASURED, both ways round, on ms01a 2026-10-21/22: the same
          * line state reads body 132.2 and 132.3 us with four lines
          * reported and 130.4 us with two -- about 2 us a cycle, 0.15% of
          * the scanner rate. The first version of this comment attributed
          * a 6% drift against four-day-old figures to it, which was wrong:
          * a cell with the default report shows the same 1466/s, so the
          * drift is something else and is still open.
          *
          * The gate stays. 0.15% is cheap but it is not free, and an
          * instrument that costs anything should be asked for rather than
          * assumed -- which is also why the number is now in this comment.
          *
          * So the default is exactly what it was -- lines 0 and 1 -- and
          * a measurement that wants the BSC line asks for it with
          * `deposit DEBUGS 18`. An instrument that changes what it
          * measures has to be opt-in; this one was briefly not.
          */
         if (line < ((Sdbg_reg & 0x10) ? MAX_LINES : 2)) {
            unsigned const st = icw_pcf[line] & 0xF;
            static unsigned long long ent_t0[MAX_LINES];
            static unsigned          ent_st[MAX_LINES] = { 16, 16, 16, 16 };
            unsigned long long const nowp = sp_tsc();
            pcf_hist[line][st]++;
            if (ent_st[line] != st) {
               if (ent_st[line] < 16 && ent_t0[line]) {
                  pcf_tsc[line][ent_st[line]] += nowp - ent_t0[line];
                  pcf_ent[line][ent_st[line]]++;
               }
               ent_st[line] = st; ent_t0[line] = nowp;
            }
         }

         switch (icw_pcf[line]) {
            case 0x0:                                // NO-OP
               if (icw_pcf_prev[line] != icw_pcf[line]) {
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) { // Trace scanner activities ?
                     fprintf(S_trace, "%s #02L%1d> CS2[%1X]: PCF = 0 entered, scf = %02X, sdf = %02X. Next PCF will be set by NCP\n",
                                       TimeStamp(), line, icw_pcf[line], icw_scf[line], icw_sdf[line]);
                  }
               }
               break;

            case 0x1:                                // Set mode - DTR on.
               if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) { // Trace scanner activities ?
                     fprintf(S_trace, "%s #02L%1d> CS2[%1X]: PCF = 1 entered, scf = %02X, sdf = %02X. Next PCF will be 0\n",
                                       TimeStamp(), line, icw_pcf[line], icw_scf[line], icw_sdf[line]);
                  }
               }
               pthread_mutex_lock(&icw_lock);
               icw_scf[line] &= 0x4A;                // Reset all check cond. bits.
               icw_sdf[line] |= 0x08;                // Set DTR in ICW
               pthread_mutex_lock(&rs232_lock);
               RS232[line] |= DTR;                   // Set DTR on line
               pthread_mutex_unlock(&rs232_lock);
               icw_scf[line] |= 0x40;                // Set service request flag
               icw_pcf_nxt[line] = 0x0;              // Goto PCF = 0...
               pthread_mutex_unlock(&icw_lock);
               CS2_req_L2_int = ON;                  // ...and issue a L2 int
               proc_LIBdisbuf(line);                 // Discard line buffer content
               break;

            case 0x2:                                // Mon DSR on
               if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) { // Trace scanner activities ?
                     fprintf(S_trace, "%s #02L%1d> CS2[%1X]: PCF = 2 entered. scf = %02X, sdf = %02X,  RS232=%02X. Next PCF will be set by NCP\n",
                                       TimeStamp(), line, icw_pcf[line],icw_scf[line], icw_sdf[line], RS232[line]);
                  }
               }
               if (RS232[line] & DCD) {              // If DCD on...
                  pthread_mutex_lock(&icw_lock);
                  icw_scf[line] |= 0x08;             // ...flag DCD on
                  pthread_mutex_unlock(&icw_lock);
                  if (RS232[line] & DSR) {           // If DSR high...
                     pthread_mutex_lock(&icw_lock);
                     icw_scf[line] |= 0x40;          // Set service request flag
                     icw_pcf_nxt[line] = 0x4;        // Goto PCF = 4
                     pthread_mutex_unlock(&icw_lock);
                     CS2_req_L2_int = ON;            // ...and issue a L2 int
                  } else {
                     pthread_mutex_lock(&icw_lock);
                     icw_scf[line] |= 0x40;          // Set service request flag
                     icw_pcf_nxt[line] = 0x2;        // Stay in PCF = 2...
                     pthread_mutex_unlock(&icw_lock);
                     CS2_req_L2_int = ON;            // ...and issue a L2 int
                  }  // End  if (RS232[line] & DSR)
               } else {
                  pthread_mutex_lock(&icw_lock);
                  icw_pcf_nxt[line] = 0x2;           // Stay in PCF = 2
                  icw_scf[line] &= 0xF7;             // ...flag DCD  off
                  pthread_mutex_unlock(&icw_lock);
               }  // End  if (RS232[line] & DCD)
               break;

            case 0x3:                                // Mon RI or DSR on
               if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) { // Trace scanner activities ?
                     fprintf(S_trace, "%s #02L%1d> CS2[%1X]: PCF = 3 entered, next PCF will be 0\n",
                                       TimeStamp(), line, icw_pcf[line]);
                  }
               }
               if (RS232[line] & RI) {              // If Incomming Call ...
                  pthread_mutex_lock(&icw_lock);
                  icw_scf[line] |= 0x40;             // Set norm char serv flag
                  icw_pcf_nxt[line] = 0x0;           // Goto PCF = 0...
                  pthread_mutex_unlock(&icw_lock);
                  CS2_req_L2_int = ON;               // ...and issue a L2 int
               } else {
                  icw_pcf_nxt[line] = 0x3;           // Stay in PCF = 3
               }
               break;

            case 0x4:                                // Monitor - block DSR error
            case 0x5:                                // Monitor - allow DSR error
               if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {  // Trace scanner activities ?
                     fprintf(S_trace, "%s #02L%1d> CS2[%1X]: PCF = %d entered, LCD = %02X, RS232=%02X. next PCF will be 6 (SDLC) or 7 (BSC)\n",
                                       TimeStamp(), line, icw_pcf[line], icw_pcf[line],icw_lcd[line], RS232[line]);
                  }
               }
               if (!(RS232[line] & DSR)) {           // If DSR low...
                  pthread_mutex_lock(&icw_lock);
                  icw_scf[line] |= 0x40;             // Set norm char serv flag
                  icw_pcf_nxt[line] = 0x2;           // Go to PCF = 2...
                  pthread_mutex_unlock(&icw_lock);
                  CS2_req_L2_int = ON;               // ...and issue a L2 int
                  break;
               }

               //if (icw_lne_stat[line] == RESET)    // Line is silent. Wait for NCP time out.
               //   break;
               //if (icw_lne_stat[line] == TX)         // Line is silent. Wait for NCP action.
               //   break;

               if (icw_lcd[line] == 0xC) {           // BSC EBCDIC ?
                  //*******************************************************
                  ret = proc_LIBrdata(&receivedChar[line], icw_pcf[line], line);
                  //*******************************************************
                  if (ret != 1) break;               // If nothing received, skip over furhter processing
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {
                     fprintf (S_trace, "%s #02L%1d> CS2[%1X]: Read ret=%d, ch=%02X\n", TimeStamp(), line, icw_pcf[line], ret, receivedChar[line]);
                  }
                  if (receivedChar[line] == 0x32)  {    // Found a SYN flag
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {
                        fprintf(S_trace, "%s #02L%1d> CS2[%1X]: Received SYN! - goto state 7\n", TimeStamp(), line, icw_pcf[line]);
                     }
                     pthread_mutex_lock(&icw_lock);
                     icw_pdf[line] = receivedChar[line];
                     icw_pcf_nxt[line]  = 0x7;          // Goto PCF = 7
                     icw_sdf[line] |= 0x04;             // Set SYNC flag
                     pthread_mutex_unlock(&icw_lock);
                  }  // End if receivedChar[line]
               }  // end BSC

               if ((icw_lcd[line] == 0x8) || (icw_lcd[line] == 0x9)) {  // SDLC ?
                  Eflg_rcvd[line] = OFF;                // Reset Eflag
                  //*******************************************************
                  ret = proc_LIBrdata(&receivedChar[line], icw_pcf[line], line);
                  //*******************************************************
                  if (ret == 0) break;                  // Break if nothing received.
                  if (receivedChar[line] == 0x7E) {
                     pthread_mutex_lock(&icw_lock);
                     //icw_sdf[line] &= 0x00;           // Clear SDF
                     icw_scf[line] |= 0x04;             // Set 7E detected flag
                     icw_lcd[line]  = 0x9;              // LCD = 9 (SDLC 8-bit)
                     icw_pcf_nxt[line] = 0x6;           // Goto PCF = 6...
                     pthread_mutex_unlock(&icw_lock);
                     CS2_req_L2_int = ON;               // ...and issue a L2 int
                 } else {
                     proc_LIBdisbuf(line);              // Discard line buffer content
                     icw_pcf_nxt[line] = 0x5;           // Stay in PCF = 5...
                 } // End  if (receivedChar[line] == 0x7E)
               }  // End if (icw_lcd[line] == 0x8...
               break;

            case 0x6:                                   // Receive info-inhibit data interrupt
               if ((svc_req_L2 == ON) || (lvl == 2)) {  // Is L2 interrupt active ?
                  break;                                // Loop till inactive...
               }
                     //*******************************************************
                     ret = proc_LIBrdata(&receivedChar[line], icw_pcf[line], line);
                     //*******************************************************
                  if (ret == 0) break;                  // Break if nothing received.
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {     // Trace scanner activities ?
                        fprintf(S_trace, "%s #02L%1d< CS2[%1X]: proc_LBrdata rc=%d \r",
                                          TimeStamp(), line, icw_pcf[line],ret);
                     }
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {  // Trace scanner activities ?
                     fprintf(S_trace, "%s #02L%1d< CS2[%1X]: PCF = 6 (re-)entered\r",
                                       TimeStamp(), line, icw_pcf[line]);
                     fprintf(S_trace, "%s #02L%1d< CS2[%1X]: Received byte = *** %02X ***, Eflag=%d\r",
                                       TimeStamp(), line, icw_pcf[line], receivedChar[line], Eflg_rcvd[line]);
                  }

                  if (receivedChar[line] == 0x7E) {  // Flag ?
                     pthread_mutex_lock(&icw_lock);
                     icw_pcf_nxt[line] = 0x6;        // Stay in PCF = 6...
                     icw_scf[line] |= 0x04;          // Set flag det bit
                     pthread_mutex_unlock(&icw_lock);
                  } else {
                     pthread_mutex_lock(&icw_lock);
                     //icw_scf[line] &= ~0x04;         // Reset flag det bit
                     icw_pdf[line] = receivedChar[line];
                     icw_pdf_reg[line] = FILLED;     // Signal NCP to read pdf.
                     icw_scf[line] |= 0x40;          // Set norm char serv flag
                     icw_pcf_nxt[line] = 0x7;        // Go to PCF = 7...
                     pthread_mutex_unlock(&icw_lock);
                     CS2_req_L2_int = ON;            // Issue a L2 interrupt
                  }
               break;

            case 0x7:                                // Receive info-allow data interrupt
               if ((svc_req_L2 == ON) || (lvl == 2)) // If L2 interrupt active ?
                  break;                             // Loop till inactive...

               if (icw_lcd[line] == 0xC) {           // BSC ?
                  if ((icw_scf[line] & 0x40) == 0) { // NCP has read pdf ?
                     //*******************************************************
                     ret = proc_LIBrdata(&receivedChar[line], icw_pcf[line], line);
                     //*******************************************************
                     if (ret != 1) receivedChar[line] = 0xFF;
                     icw_pdf[line] = receivedChar[line];
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {  // Trace scanner activities ?
                        fprintf(S_trace, "%s #02L%1d< CS2[%1X]: State 7 ch = %02X\n", TimeStamp(), line, icw_pdf[line], receivedChar[line]);
                     }
                     pthread_mutex_lock(&icw_lock);
                     //icw_pdf_reg[line] = FILLED;   // Signal NCP to read pdf.
                     icw_scf[line] |= 0x40;          // Set norm char serv flag
                     icw_pcf_nxt[line] = 0x7;        // Stay in PCF = 7...
                     pthread_mutex_unlock(&icw_lock);
                     CS2_req_L2_int = ON;            // Issue a L2 interrupt
                  }  // End if icw_scf[line]
               }  // End BSC

               if ((icw_lcd[line] == 0x8) || (icw_lcd[line] == 0x9)) {  // SDLC ?
                  //*******************************************************
                  ret = proc_LIBrdata(&receivedChar[line], icw_pcf[line], line);
                  //*******************************************************
                  if (ret == 0) {                    // No characters received?
                     break;
                  }
                  // Check end of frame when a flag byte is detected. If the FCS matches, this is really the end of the frame.,
                  // else we bumped into a regular x7E character,
                  if ((receivedChar[line] == 0x7E) && (FCS_rcvd[line][0] == 0x47) && (FCS_rcvd[line][1] == 0x0F)) {
                     Eflg_rcvd[line] = ON;
                  } else {
                     FCS_rcvd[line][0] = FCS_rcvd[line][1];
                     FCS_rcvd[line][1] = receivedChar[line];
                     Eflg_rcvd[line] = OFF;                 // No Eflag
                  }

                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {     // Trace scanner activities ?
                     fprintf(S_trace, "%s #02L%1d< CS2[%1X]: PCF = 7 (re-)entered\n",
                                       TimeStamp(), line, icw_pcf[line]);
                     fprintf(S_trace, "%s #02L%1d< CS2[%1X]: Received byte = *** %02X ***, Eflag=%d\n",
                                       TimeStamp(), line, icw_pcf[line], receivedChar[line], Eflg_rcvd[line]);
                  } // End if ((receivedChar[line] == 0x7E)
                  if (Eflg_rcvd[line] == ON) {
                     pthread_mutex_lock(&icw_lock);
                     icw_lne_stat[line] = TX;           // Line turnaround to transmitting...
                     icw_scf[line] |= 0x44;             // Set 7E detected flag
                     //icw_lcd[line]  = 0x9;              // LCD = 9 (SDLC 8-bit)
                     icw_pcf_nxt[line] = 0x6;           // Goto PCF = 6...
                     pthread_mutex_unlock(&icw_lock);
                     CS2_req_L2_int = ON;               // ...and issue a L2 int
                     Eflg_rcvd[line] = OFF;             // Reset Eflag
                     FCS_rcvd[line][0] = 0x00;          // Clear FCS buffer
                     FCS_rcvd[line][1] = 0x00;          // Clear FCS buffer
                  } else {
                     pthread_mutex_lock(&icw_lock);
                     icw_pdf[line] = receivedChar[line];
                     icw_pdf_reg[line] = FILLED;        // Signal NCP to read pdf.
                     icw_scf[line] |= 0x40;             // Set norm char serv flag
                     icw_pcf_nxt[line] = 0x7;           // Stay in PCF = 7...
                     pthread_mutex_unlock(&icw_lock);
                     CS2_req_L2_int = ON;               // Issue a L2 interrupt
                  } // End if (Eflg_rcvd[line] == ON)
               }  // End if (icw_lcd[line] == 0x8...

               break;

            case 0x8:                                // Transmit initial turn RTS on
               if ((svc_req_L2 == ON) || (lvl == 2)) // If L2 interrupt active ?
                  break;

               if (!(RS232[line] & CTS)) {           // if not CTS, skip and try again later
                  RS232[line] |= RTS;                // Raise Request To Send
#if defined(__GENODE__)
                  /* Grant it NOW. Upstream runs the DCE signal rules from
                     its epoll loop, on every socket event; this port runs
                     them from a 20 Hz sweep, which put a 50 ms floor under
                     every turnaround -- native's median was 18 ms and this
                     port could not beat 50. Calling it here costs nothing
                     and needs no faster sweep. */
                  LIB_signals(line);
#endif
                  break;
               }

               if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) { // Trace scanner activities ?
                  fprintf(S_trace, "%s #02L%1d> CS2[%1X]: PCF = 8 entered, next PCF will be 9\n",
                                    TimeStamp(), line, icw_pcf[line]);
               }

               if (icw_lcd[line] == 0xC) {           // BSC EBCDIC ?
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {
                     fprintf(S_trace, "%s #02L%1d> CS2[%1X]: icw_pdf=%02X icw_scf=%02X icw_scf&0x40=%02X\n",
                                       TimeStamp(), line, icw_pcf[line], 0xff & icw_pdf[line], 0xff & icw_scf[line], icw_scf[line] & 0x40);
                     fprintf(S_trace, "%s #02L%1d> CS2[%1X]: 1. condition=%01X icw_pdf=%02X icw_scf=%02X\n",
                                       TimeStamp(), line, icw_pcf[line], (icw_scf[line] & 0x40) == 0, 0xff & icw_pdf[line], 0xff & icw_scf[line]);
                  }
                  if ((icw_scf[line] & 0x40) == 0) { // New char avail to xmit ?
                     transmitChar = icw_pdf[line];
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {
                        fprintf(S_trace, "%s #02L%1d> CS2[%1X]: 2. condition=%01X icw_pdf=%02X icw_scf=%02X\n",
                                          TimeStamp(), line, icw_pcf[line], (icw_scf[line] & 0x40) == 0, 0xff & icw_pdf[line], 0xff & icw_scf[line]);
                        fprintf(S_trace, "%s #02L%1d> CS2[%1X]: State 8 ch=%02X \n",TimeStamp(), line, icw_pcf[line], transmitChar);
                     }
                  }
                  //*******************************************************
                  proc_LIBtdata(transmitChar,icw_pcf[line], line);
                  //*******************************************************
                  // Next byte please...
                  pthread_mutex_lock(&icw_lock);
                  icw_pdf_reg[line] = EMPTY;         // Ask NCP for next byte
                  icw_scf[line] |= 0x40;             // Set norm char serv flag
                  icw_pcf_nxt[line] = 0x9;           // Go to PCF = 9...
                  pthread_mutex_unlock(&icw_lock);
                  CS2_req_L2_int = ON;               // Issue a L2 interrupt
               }  // End BSC

               if (icw_lcd[line] == 0x9) {           // SDLC ?
                  // Call LIBtdata to indicate start of a new frame. transmitChar is ignored.
                  //*******************************************************
                  proc_LIBtdata(transmitChar, icw_pcf[line], line);
                  //*******************************************************
                  pthread_mutex_lock(&icw_lock);
                  icw_scf[line] &= 0xFB;             // Reset flag detected flag
                  // CTS is now on.
                  icw_pcf_nxt[line] = 0x9;           // Goto PCF = 9
                  pthread_mutex_unlock(&icw_lock);
                  // NO CS2_req_L2_int !
               }  // End SDLC
               break;

            case 0x9:                                // Transmit normal
               if ((svc_req_L2 == ON) || (lvl == 2)) // If L2 interrupt active ?
                  break;

               if (icw_lcd[line] == 0xC) {           // BSC EBCDIC ?
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {
                     fprintf(S_trace, "%s #02L%1d> CS2[%1X]: icw_pdf=%02X icw_scf=%02X lvl=%d\n",
                                       TimeStamp(), line, icw_pcf[line], 0xff & icw_pdf[line], 0xff & icw_scf[line]);
                  }
                  if ((icw_scf[line] & 0x40) == 0) { // New char avail to xmit ?
                     transmitChar = icw_pdf[line];
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {   // Trace scanner activities ?
                        fprintf(S_trace, "%s #02L%1d> CS2[%1X]: State 9 ch=%02X \n", TimeStamp(), line, icw_pcf[line], transmitChar);
                     }
                     //*******************************************************
                     proc_LIBtdata(transmitChar,icw_pcf[line], line);
                     //*******************************************************

                     // Next byte please...
                     pthread_mutex_lock(&icw_lock);
                     icw_pdf_reg[line] = EMPTY;      // Ask NCP for next byte
                     icw_scf[line] |= 0x40;          // Set norm char serv flag
                     icw_pcf_nxt[line] = 0x9;        // Stay in PCF = 9...
                     pthread_mutex_unlock(&icw_lock);
                     CS2_req_L2_int = ON;            // Issue a L2 interrupt
                  }
               }

               if (icw_lcd[line] == 0x9) {           // SDLC ?
                  if (icw_pdf_reg[line] == FILLED) { // New char avail to xmit ?
                     transmitChar = icw_pdf[line];
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {  // Trace scanner activities ?
                        fprintf(S_trace, "%s #02L%1d> CS2[%1X]: PCF = 9 (re-)entered \n",
                                          TimeStamp(), line, icw_pcf[line]);
                        fprintf(S_trace, "%s #02L%1d> CS2[%1X]: Transmitting PDF = *** %02X ***\n",
                                          TimeStamp(), line, icw_pcf[line], icw_pdf[line]);
                     }
                     //*******************************************************
                     proc_LIBtdata(transmitChar, icw_pcf[line], line);
                     //*******************************************************

                     // Next char please...
                     pthread_mutex_lock(&icw_lock);
                     icw_pdf_reg[line] = EMPTY;      // Ask NCP for next byte
                     icw_scf[line] |= 0x40;          // Set norm char serv flag
                     icw_pcf_nxt[line] = 0x9;        // Stay in PCF = 9...
                     pthread_mutex_unlock(&icw_lock);
                     CS2_req_L2_int = ON;            // Issue a L2 interrupt
                  }
               }  // End SDLC
               break;

            case 0xA:                                // Transmit normal with new sync
               if ((svc_req_L2 == ON) || (lvl == 2)) // If L2 interrupt active ?
                  break;

               if (icw_lcd[line] == 0xC) {           // BSC EBCDIC
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {
                     fprintf(S_trace, "%s #02L%1d> CS2[%1X]: icw_pdf=%02X icw_scf=%02X lvl=%d\n",
                                       TimeStamp(), line, icw_pcf[line], 0xff & icw_pdf[line], 0xff & icw_scf[line]);
                  }
                  if ((icw_scf[line] & 0x40) == 0) { // New char avail to xmit ?
                     transmitChar = icw_pdf[line];
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {   // Trace scanner activities ?
                        fprintf(S_trace, "%s #02L%1d> CS2[%1X]: State 9 ch=%02X \n",
                                          TimeStamp(), line, icw_pcf[line], transmitChar);
                     }
                     //*******************************************************
                     proc_LIBtdata(transmitChar,icw_pcf[line], line);
                     //*******************************************************

                     // Next byte please...
                     pthread_mutex_lock(&icw_lock);
                     icw_pdf_reg[line] = EMPTY;      // Ask NCP for next byte
                     icw_scf[line] |= 0x40;          // Set norm char serv flag
                     icw_pcf_nxt[line] = 0xA;        // Stay in PCF = A...
                     pthread_mutex_unlock(&icw_lock);
                     CS2_req_L2_int = ON;            // Issue a L2 interrupt
                  }
               }  // End BSC
               break;

            case 0xB:                                // Not used
               break;

            case 0xC:                                // Transmit turnaround-turn RTS off
               if (icw_lcd[line] == 0xC) {           // BSC EBCDIC
                  if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {
                        fprintf(S_trace, "%s #02L%1d> CS2[%1X]: Now into state C.\n", TimeStamp(), line, icw_pcf[line]);
                     }
                     transmitChar = icw_pdf[line];   // Will not be used.
                     //*******************************************************
                     proc_LIBtdata(transmitChar, icw_pcf[line], line);  //Signal line we are done
                     //*******************************************************

                     pthread_mutex_lock(&icw_lock);
                     icw_lne_stat[line] = RX;        // Line turnaround to receiving...
                     icw_scf[line] |= 0x40;          // Set norm char serv flag
                     icw_pcf_nxt[line] = 0x5;        // Goto PCF = 5...
                     pthread_mutex_unlock(&icw_lock);
                     CS2_req_L2_int = ON;            // ...and issue a L2 int
                  }
               }  //end BSC

               if (icw_lcd[line] == 0x9) {           // SDLC ?
                  if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?

                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) { // Trace scanner activities ?
                        fprintf(S_trace, "%s #02L%1d> CS2[%1X]: PCF = C entered, next PCF will be set by NCP\n",
                                          TimeStamp(), line, icw_pcf[line]);
                     }
                     transmitChar = icw_pdf[line];   // Will not be used.
                     // ******************************************************************
                     // Signal SDLC that final character has been received.
                     proc_LIBtdata(transmitChar, icw_pcf[line], line);
                     // ******************************************************************
                     pthread_mutex_lock(&icw_lock);
                     icw_lne_stat[line] = RX;        // Line turnaround to receiving...
                     icw_scf[line] |= 0x40;          // Set norm char serv flag
                     icw_pcf_nxt[line] = 0x5;        // Goto PCF = 5...
                     pthread_mutex_unlock(&icw_lock);
                     CS2_req_L2_int = ON;            // ...and issue a L2 int
                  }
               }  // End SDLC
               RS232[line] &= ~RTS;                  // Drop RTS
               break;

            case 0xD:                                // Transmit turnaround-keep RTS on
               if (icw_lcd[line] == 0xC) {           // BSC EBCDIC ?
                  if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {    // Trace scanner activities ?
                        fprintf(S_trace, "%s #02L%1d> CS2[%1X]: PCF = D entered, next PCF will be set by NCP\n", TimeStamp(), line, icw_pcf[line]);
                     }
                  }
                     icw_pcf_nxt[line] = 0x5;        // Goto PCF = 5...
                     CS2_req_L2_int = ON;            // ...and issue a L2 int
                                                     // ==> ??? NO CS2_req_L2_int !
               }  // End BSC

               if (icw_lcd[line] == 0x9) {           // SDLC ?
                  if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {  // Trace scanner activities ?
                        fprintf(S_trace, "%s #02L%1d> CS2[%1X]: PCF = D entered, next PCF will be set by NCP\n",
                                          TimeStamp(), line, icw_pcf[line]);
                     }
                  }

                     transmitChar = icw_pdf[line];   // Will not be used.
                     // ******************************************************************
                     // Signal SDLC that final character has been received.
                     proc_LIBtdata(transmitChar, icw_pcf[line], line);
                     // ******************************************************************

               }  // End if (icw_lcd[line] == 0x9)

               if (icw_lcd[line] == 0x0) {                     // Duplex?
                  if (icw_pcf_prev[line] != icw_pcf[line]) {   // First entry ?
                     if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {    // Trace scanner activities ?
                        fprintf(S_trace, "%s #02L%1d> CS2[%1X]: PCF = D entered (lcd=0), next PCF will be set by NCP\n", TimeStamp(), line, icw_pcf[line]);
                     }
                  }
                     //icw_lne_stat[line] = RX;        // Line turnaround to receiving...
                     pthread_mutex_lock(&icw_lock);
                     icw_scf[line] |= 0x40;          // Set norm char serv flag
                     icw_pcf_nxt[line] = 0x0;        // Goto PCF = 0...
                     pthread_mutex_unlock(&icw_lock);
                     CS2_req_L2_int = ON;            // ...and issue a L2 int
               }  // End   if (icw_lcd[line] == 0x0)

               break;

            case 0xE:                                // Not used
               break;

            case 0xF:                                // Disable line
               if (icw_pcf_prev[line] != icw_pcf[line]) {
                  if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) { // Trace scanner activities ?
                     fprintf(S_trace, "%s #02L%1d> CS2[%1X]: PCF = F entered, next PCF will be set by NCP\n",
                                       TimeStamp(), line, icw_pcf[line]);
                  }
               }
               pthread_mutex_lock(&icw_lock);
               icw_scf[line] |= 0x40;                // Set norm char serv flag   <===!!!
               icw_sdf[line] &= ~0x08;               // reset DTR
               pthread_mutex_unlock(&icw_lock);
               RS232[line] &= ~DTR;                  // reset DTR on line
               icw_pcf_nxt[line] = 0x0;              // Goto PCF = 0...
               CS2_req_L2_int = ON;                  // ...and issue a L2 int
               break;

         }  // End of switch (icw_pcf[line])
         //pthread_mutex_unlock(&icw_lock);


         // ========  POST-PROCESSING SCAN A LINE CYCLE  ========

         if (CS2_req_L2_int) {                       // CS2 L2 interrupt requested ?
            if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {  // Trace scanner activities ?
               fprintf(S_trace, "%s #02L%1d> CS2[%1X]: SVCL2 interrupt issued for PCF = %1X\n",
                                 TimeStamp(), line, icw_pcf[line], icw_pcf[line]);
            }

            while (svc_req_L2 == ON) {               // Wait till CCU has finished L2 processing
                  l2wait[0]++;
                  sp_l2_wait(10);                    // wait until L2 is done.
            }

            abar_int = line + 0x020;                 // Set ABAR with line # that caused the L2 int.

            if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {  // Trace scanner activities ?
               fprintf(S_trace, "%s #02L%1d> CS2[%1X]: abar_int = %04X\n",
                                 TimeStamp(), line, icw_pcf[line], abar_int );
            }
            svc_req_L2 = ON;                         // Issue a level 2 interrrupt
            CS2_req_L2_int = OFF;                    // Reset int req flag
         }

         pthread_mutex_lock(&icw_lock);
         icw_pcf_prev[line] = icw_pcf[line];         // Save current pcf
         if (icw_pcf[line] != icw_pcf_nxt[line]) {   // pcf state changed ?
            icw_pcf[line]   = icw_pcf_nxt[line];     // Set new current pcf
         }
         pthread_mutex_unlock(&icw_lock);

         if (icw_pcf_prev[line] != icw_pcf[line]) {  // First entry ?
            if ((Sdbg_flag == ON) && (Sdbg_reg & 0x02)) {  // Trace scanner activities ?
               fprintf(S_trace, "%s #02L%1d> CS2[%1X]: Next PCF = %1X\n",
                                 TimeStamp(), line, icw_pcf_prev[line], icw_pcf[line] );
            }
         }

         while (svc_req_L2 == ON) {                  // Wait till CCU has finished L2 processing
              if (icw_pcf[line] == 0x7) {              // If we are receiving
                  l2wait[1]++;
                  sp_l2_wait(10);                    // wait until L2 is done.
               } else {                                  // If not receiving, wait a bit longer t give L3 a change
                  l2wait[2]++;
                  sp_l2_wait(1000);
               }
         }   // End   while (svc_req_L2 == ON)
      }  // End for line = 0 ---> MAX_LINES          // End of scanning one line, next please...
      {
         uint64_t const t0 = sp_tsc();
         /*
          * SCANUS IS THE LINE'S CHARACTER CLOCK, and 500 us was chosen for
          * the CPU it costs rather than for the rate it produces. One cycle
          * of this loop moves ONE character on each line, so the cycle rate
          * IS the line rate: at the 1225 cycles/s this settles to, a 542
          * byte frame takes 442 ms to clock out, and a block plus its reply
          * is most of a second before anything is processed. A real 56 kbps
          * line carries about 7000 characters/s.
          *
          * Measured on the two-3705 trunk remote load: 133 data blocks each
          * way, 268 turnarounds of over a second, ~91% of the elapsed time,
          * and that count did not move for ANY of the CPU-side fixes --
          * neither the idle throttle nor the WAIT state latency -- because
          * the line is character-clocked here and nowhere else.
          */
         if (ScanSpin) sp_pace(ScanUs ? ScanUs : 500);  // bounded spin, no timer
#if defined(__GENODE__)
         /* The Timer session at microsecond resolution: 577 us measured for
            a requested 500, against libc's 1.1 ms and native Linux's 565.
            libc's nanosleep divides tv_nsec by 1000000 and clamps to 1 ms,
            which is where the resolution went. */
         else          genode_pace_us(ScanUs ? ScanUs : 500);
#else
         else          usleep(ScanUs ? ScanUs : 500);  // ...give  L3 some time.
#endif
         {
            uint64_t const t1 = sp_tsc();
            sp_sleep += t1 - t0;
            if (sp_last) sp_body += t0 - sp_last;
            sp_last = t1;
            tw_count[0]++;
            if (++sp_cycles >= 500) {
               if ((Sdbg_flag == ON) && (Sdbg_reg & 0x08)) {
                  static uint64_t tw_last = 0;
                  uint64_t const nowt = sp_tsc();
                  if (tw_last) {
                     double const secs = (double)(nowt - tw_last) / (sp_tsc_mhz() * 1000000.0);
                     printf(SP_CR "TWCOUNT/s: scan=%.0f pnl=%.0f cpu=%.0f ca10=%.0f ca100=%.0f lib=%.0f cawait=%.0f idle=%.0f l2w=%.0f/%.0f/%.0f lvl=%.0f/%.0f/%.0f/%.0f/%.0f/%.0f\n",
                            tw_count[0]/secs, tw_count[1]/secs, tw_count[2]/secs,
                            tw_count[3]/secs, tw_count[4]/secs, tw_count[5]/secs,
                            tw_count[6]/secs, tw_count[7]/secs,
                            l2wait[0]/secs, l2wait[1]/secs, l2wait[2]/secs,
                            lvl_hist[0]/secs, lvl_hist[1]/secs, lvl_hist[2]/secs,
                            lvl_hist[3]/secs, lvl_hist[4]/secs, lvl_hist[5]/secs);
                  printf(SP_CR "TICKS: %.1f/s (100ms timer, want 10.0)\n", timer_ticks/secs);
                  timer_ticks = 0;
                  printf(SP_CR "L3WHY: inter=%.0f timer=%.0f pci=%.0f caDS=%.0f caIS=%.0f per s\n",
                         l3_why[0]/secs, l3_why[1]/secs, l3_why[2]/secs,
                         l3_why[3]/secs, l3_why[4]/secs);
                  for (int z = 0; z < 5; z++) l3_why[z] = 0;
                  /*
                   * Cumulative, NOT per window: these are totals for the run.
                   * PRINTED ONLY WHEN ONE OF THEM CHANGES. Printed every
                   * report instead, it added a log line to each of ~2400
                   * reports, and each one is a synchronous write out of the
                   * component -- which cost 2.6% of the scanner's rate beside
                   * a partition that never yields, while costing nothing at
                   * idle. The counters are free; the printing was not.
                   */
                  {
                     static unsigned long x55_seen[3];
                     if (x55_img[0] != x55_seen[0] ||
                         x55_img[1] != x55_seen[1] ||
                         x55_img[2] != x55_seen[2]) {
                        printf(SP_CR "X55IMG: stores=%lu saved_outcwar=%lu saved_incwar=%lu\n",
                               x55_img[0], x55_img[1], x55_img[2]);
                        for (int z = 0; z < 3; z++) x55_seen[z] = x55_img[z];
                     }
                  }
                  }
                  for (int ln = 0; ln < ((Sdbg_reg & 0x10) ? MAX_LINES : 2); ln++) {
                     unsigned long tot = 0;
                     for (int q = 0; q < 16; q++) tot += pcf_hist[ln][q];
                     if (!tot) continue;
                     printf(SP_CR "PCF%d:", ln);
                     for (int q = 0; q < 16; q++)
                        if (pcf_hist[ln][q])
                           printf(" %X=%.0f%%", q, 100.0*pcf_hist[ln][q]/tot);
                     printf("  (%lu cycles)\n", tot);
                     printf(SP_CR "DWELL%d:", ln);
                     for (int q = 0; q < 16; q++)
                        if (pcf_ent[ln][q])
                           printf(" %X=%.0fus/%lu", q,
                                  pcf_tsc[ln][q]/(double)pcf_ent[ln][q]/sp_tsc_mhz(),
                                  pcf_ent[ln][q]);
                     printf("\n");
                     for (int q = 0; q < 16; q++) { pcf_tsc[ln][q]=0; pcf_ent[ln][q]=0; }
                     for (int q = 0; q < 16; q++) pcf_hist[ln][q] = 0;
                  }
                  printf(SP_CR "LVLUS: w=%.0f L1=%.0f L2=%.0f L3=%.0f L4=%.0f L5=%.0f us/pass\n",
                         lvl_hist[0] ? lvl_tsc[0]/(double)lvl_hist[0]/sp_tsc_mhz() : 0.0,
                         lvl_hist[1] ? lvl_tsc[1]/(double)lvl_hist[1]/sp_tsc_mhz() : 0.0,
                         lvl_hist[2] ? lvl_tsc[2]/(double)lvl_hist[2]/sp_tsc_mhz() : 0.0,
                         lvl_hist[3] ? lvl_tsc[3]/(double)lvl_hist[3]/sp_tsc_mhz() : 0.0,
                         lvl_hist[4] ? lvl_tsc[4]/(double)lvl_hist[4]/sp_tsc_mhz() : 0.0,
                         lvl_hist[5] ? lvl_tsc[5]/(double)lvl_hist[5]/sp_tsc_mhz() : 0.0);
                  for (int z = 0; z < 6; z++) lvl_tsc[z] = 0;
                  tw_last = nowt;
                  for (int z = 0; z < 8; z++) tw_count[z] = 0;
                  for (int z = 0; z < 3; z++) l2wait[z]   = 0;
                  /* lvl_hist too -- it was cumulative while everything
                     beside it was per-interval, which read as millions
                     per second. */
                  for (int z = 0; z < 6; z++) lvl_hist[z] = 0;
               }
               if ((Sdbg_flag == ON) && (Sdbg_reg & 0x08))   /* SIM_TRACE=08 */
                  printf(SP_CR "SCANPERF: cycles=%llu sleep=%llu body=%llu tsc pct_sleep=%llu\n",
                         (unsigned long long)sp_cycles,
                         (unsigned long long)(sp_sleep / sp_cycles),
                         (unsigned long long)(sp_body  / sp_cycles),
                         (unsigned long long)(100 * sp_sleep / (sp_sleep + sp_body + 1)));
               sp_cycles = sp_sleep = sp_body = 0;
            }
         }
      }
   }  // End of while(1)...
   return (0);
}


// *******************************************************************
// Function to copy ICW[line] to input regs used in CCU coding.
// *******************************************************************
void Get_ICW(int line) {                  // See 3705 CE manauls for details.

   Eregs_Inp[0x44] = (icw_scf[line] << 8)  | icw_pdf[line];
   Eregs_Inp[0x45] = (icw_lcd[line] << 12) | (icw_pcf[line] << 8) | icw_sdf[line];
   Eregs_Inp[0x46] = ((RS232[line] & 0xF0) | 0x02) << 8;   // Display reg
   Eregs_Inp[0x47] =  icw_Rflags[line];   // ICW 32 - 47

   return;
}

// *******************************************************************
// Function to initialize the ICW and buffer of the scanner
// *******************************************************************
void Init_ICW(int max) {

   for (int j = 0; j < max; j++) {        // Initialize all lines
   // ICW Local Store Registers
      icw_scf[j] = 0;                     // ICW[ 0- 7] SCF - Secondary Control Field
      icw_pdf[j] = 0;                     // ICW[ 8-15] PDF - Parallel Data Field
      icw_lcd[j] = 0;                     // ICW[16-19] LCD - Line Code Definer
      icw_pcf[j] = 0xE;                   // ICW[20-23] PCF - Primary Control Field
      icw_sdf[j] = 0;                     // ICW[24-31] SDF - Serial Data Field
                                          // ICW[32-33] Not implemented (OSC sel bits)
      icw_Rflags[j] = 0;                  // ICW[34-47] flags
   // Additional icw fields for emulator
      icw_pcf_prev[j] = 0x0;              // Previous icw_pcf
      icw_lne_stat[j] = RESET;            // Line state: RESET, TX, RX
      icw_pcf_nxt[j] = 0x0;               // What will be the next pcf value
      icw_pdf_reg[j] = EMPTY;             // Status ICW PDF reg: NCP FILLED pdf for Tx
                                          //                     NCP EMPTY pdf during Rx
   }
   return;
}

/*-------------------------------------------------------------------*/
/* Return Timestamp hh.mm.ss.msec                                    */
/*-------------------------------------------------------------------*/
static char * TimeStamp() {
    static char Timebuff[17+1];
    struct timeval tv;
    struct timezone tz;
    struct tm *timestamp;
    gettimeofday(&tv,&tz);
    timestamp = localtime(&tv.tv_sec);
    time_t now = time (0);
    snprintf(Timebuff,18,"[%02d:%02d:%02d.%06d]", timestamp->tm_hour,timestamp->tm_min,timestamp->tm_sec,tv.tv_usec);
    return Timebuff;
}
