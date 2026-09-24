/* Copyright (c) 2024, Edwin Freekenhorst and Henk Stegeman

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

   i3705_LIB.c: This module contains the IBM 3705 Line Interface Base function
                It supports both BSC and SDLC lines.

   This LIB module emulates the 3705 hardware that provides RS232 connections
   for leased and switched lines.  The physical connections are emulated through
   TCP/IP connections.
   The TCP endpoints represent the RS232 DCEs (Data Communication Equipment).
   This LIB modules embeds the local DCE, providing RS232 signals to the scanner.
   Based on the signals, the scanner determines the appropriate course of action.
   Remote DCEs are embedded in other modules, like i3274, i3271 and DLSw.
   RS232 signal flow diagram:

         DTE         DCE                           DCE              DTE
        3705        3705                          Remote         Remote
         DTR --------------------------------+----> DCD ---------->
                                             +----> DSR ---------->
          <----------DCD <----+----------------------------------- DTR
          <----------DSR <----+
         RTS-------------------------------------------------------->
         <----------------------------------------------------------CTS

   The  DCE RS232 signals are represented through a mapping of the TCP/IP
   connection states as well as the actions of the NCP/scanner.
   The below mapping is from the viewpoint if the 3705.

   +------------------------------------------+-----------------------------+
   |                 Endpoint                 |      3705  RS232 signal     |
   +----------------------+-------------------+-----+-----+-----+-----+-----+-----+
   | 3705 DCE             | Remote DCE        | DCD | RI  | DSR | RTS | CTS | DTR |
   +----------------------+-------------------+-----+-----+-----+-----+-----+-----+
   | No Connection        | Not Connected     |  0  |  0  |  0  |  0  |  0  |  0  |
   +----------------------+-------------------+-----+-----+-----+-----+-----+-----+
   | Connection Accepted  | Connected         |  1  |  1  |  0  |  0  |  0  |  0  |
   +----------------------+-------------------+-----+-----+-----+-----+-----+-----+
   | NCP raises DTR       |                   |  1  |  1  |  1  |  0  |  0  |  1  |
   +----------------------+-------------------+-----+-----+-----+-----+-----+-----+
   | NCP raises RTS       |                   |  1  |  1  |  1  |  1  |  0  |  1  |
   +----------------------+-------------------+-----+-----+-----+-----+-----+-----+
   | DCE sends RTS        | DCE receives RTS  |  1  |  1  |  1  |  1  |  0  |  1  |
   +----------------------+-------------------+-----+-----+-----+-----+-----+-----+
   | DCE receives CTS     | Close Connection  |  1  |  1  |  1  |  1  |  1  |  1  |
   +----------------------+-------------------+-----+-----+-----+-----+-----+-----+
   | NCP drops RTS        |                   |  1  |  1  |  1  |  0  |  0  |  1  |
   +----------------------+-------------------+-----+-----+-----+-----+-----+-----+

   The RS232 signal handling for the remote DCE's (3274, 3271, DLSw) is straightforward:
   If RTS is received and the remote is ready to receive, CTS is returned.
   If the remote is not ready, nothing is returned.
   No other RS232 signals are being used by the remote DCE's (yet).

   ---------------------------------------------------------------------------

   BSC text layout:
   +----------+-----+-----+------//------+-----+---+---+-----+
   | AA | SYN | SYN.| SOT | ... Text ... | EOT |  CRC  | PAD |
   +----------+-----+-----+------//------+-----+---+---+-----+

   BSC long text layout:
   +----------+-----+-----+------//------+-----+-----+------//------+-----+---+---+-----+
   | AA | SYN | SYN.| SOT | ... Text ... | SYN | SYN | ... Text ... | EOT |  CRC  | PAD |
   +----------+-----+-----+------//------+-----+-----+------//------+-----+---+---+-----+

   SLDC frame
    <-------------------------------- BLU ----------------------------->
   layout:         |   FCntl   |
   +-------+-------+-----------+-------//-------+-------+-------+-------+
   | BFlag | FAddr |Nr|PF|Ns|Ft| ... Iframe ... | Hfcs  | Lfcs  | EFlag |
   +-------+-------+-----------+-------//-------+-------+-------+-------+

   ---------------------------------------------------------------------------
*/

#include <stdbool.h>
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

#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#if defined(__GENODE__)

/*
 * On Genode a line is a terminal_crosslink, not a listening socket.
 *
 * Upstream binds four listeners on 37520-37523 -- one per line -- and a
 * 3274, 3271, DLSw or Trunk connects to one of them. Here the crosslinks are
 * opened before the simulator starts and genode_ca.c holds them, so the
 * listen, the accept and the interface lookup that chose an address to bind
 * all go away together. See ../../COMM3705.md.
 *
 * Two redirections, for the same reasons as in i3705_chan_T2.c: a crosslink
 * write returns a short count rather than blocking, and its fd is not a
 * socket. The reads on a line are already "whatever arrived", which is what
 * genode_ca_read gives.
 *
 * ncurses is not here either. The LIB panel draws line status in a terminal
 * this component does not have, and the same information belongs in the
 * telemetry -- so the four drawing functions below are empty rather than
 * pulling a curses port in to render to nothing.
 */
#include "genode_ca.h"
#define send(fd, buf, len, flags)  genode_ca_write((fd), (buf), (int)(len))
#define read(fd, buf, len)         genode_ca_read ((fd), (buf), (int)(len))
#else
#include <ifaddrs.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ncurses.h>
#endif

#define RED_BLACK    1
#define GREEN_BLACK  2
#define YELLOW_BLACK 3
#define WHITE_BLACK  4
#define BLUE_BLACK   5
#define BLACK_RED    6
#define BLACK_GREEN  7
#define BLACK_YELLOW 8
#define BLACK_WHITE  9
#define BLACK_BLACK 10

#define BUFLEN_327x     16384          // 327x Send/Receive buffer
#define LIBLBASE        20             // LIB line ports start at 20

#define SYN 0x32

struct LIBLine {
   int      line_fd;
   int      linenum;
   int      d327x_fd;                  // Data lead connection
   int      s327x_fd;                  // RS232 signal lead connection
   int      epoll_fd;                  // Event polling file desciptor
   uint8_t  LIB_rbuf[BUFLEN_327x];     // Received data buffer
   uint8_t  LIB_tbuf[BUFLEN_327x];     // Transmit data buffer
   uint16_t LIBrlen;                   // Size of received data in buffer
   uint16_t LIBtlen;                   // Size of transmit data in buffer
   uint16_t LIBbptr;                   // Buffer position pointer
   int8     LIBsync;                   // Track receive progress
   int8     prevPCF;                   // Remember Previous PCF
} *LIBline[MAX_LINES];

#if !defined(__GENODE__)
struct epoll_event event, events[MAX_LINES];
#endif

pthread_mutex_t line_lock;             // Line lock
pthread_mutex_t rs232_lock;            // RS232 signal lock
uint8_t RS232[MAX_LINES] = {0};       // Local RS232 signals (current)
uint8_t RS232p[MAX_LINES] = {0};      // Local RS232 signals  (previous)
uint8_t RS232r[MAX_LINES] = {0};      // Remote RS232 signals
uint8_t RS232x[MAX_LINES] = {0};      // Transmt flag for RS232 signals to remote

// Trace variables
extern int32 debug_reg;                //Bit flags for instruction debug/trace
extern uint16_t Sdbg_reg;              // Bit flags for line & scanner debug/trace
extern uint16_t Sdbg_flag;             // 1 when S_trace.log open
extern FILE *S_trace;                  // Scanner trace file fd

extern int32 Eregs_Inp[];              // Input registers (only needed for the cycle counter)
int8 station;                          // Station #
uint8_t LIBready = 0;                  // LIB structures allocated and initialized.

void LIBpanel_Init();                                          /* Function to initialize the LIB  panel                                             */
static void stringAtXY(int x, int y, char *buf, int colour);   /* Write coloured text on the panel at row x, col y. Function defined in i3705_panel */
static void integerAtXY (int x, int y, int value, int colour); /* Write an integer on the panel at row x, col y. Function defined in i3705_panel    */
static char * TimeStamp();                                     /* Print timestamp in log file                                                       */
int8 shwlib = 0;                                               /* Dispaly LIB Panel, initially no                                                   */



// **********************************************************
// Function to build the LIB display.
// **********************************************************
void LIBpanel() {
#if defined(__GENODE__)
    return;
#else

    stringAtXY(0, 26, "IBM 3705 Line Interface Base", WHITE_BLACK);
    stringAtXY(1,  0, "----------------------------------------", GREEN_BLACK);
    stringAtXY(1, 40, "---------------------------------------", GREEN_BLACK);
    stringAtXY(3, 22, "CTS   RI  DSR  DCD  RTS  DTR", YELLOW_BLACK);
    for (int i=0; i < MAX_LINES; i++) {
       stringAtXY(4+i,5,"LINE ",GREEN_BLACK);
    }
    stringAtXY(22,70,  "HOME=exit", GREEN_BLACK);
    return;
#endif
}

// **********************************************************
// Function to update the LIB display.
// If the HOME key is presserd, the panlke is closed.
// **********************************************************
void LIBpanel_Updt() {
#if defined(__GENODE__)
   return;
#else
   int key, line;
    key = getch();
    if ((key != KEY_HOME) && (key != 0x007E)) {             /* 0x007E needed for putty  */
       for (int i=0; i < MAX_LINES; i++) {
          line = LIBLBASE + i;
          integerAtXY(4+i, 11, line, YELLOW_BLACK);

          for (int j=0; j < 6; j++) {                       /* check bits 0 through 5  (6 and 7 are not used) */
             if ((RS232[i] << j) & 0x80)                    /* If bit is on (= RS232 signal = high)           */
                stringAtXY(4+i,23+(j*5)," ",BLACK_GREEN);   /* show as a green blob                           */
             else
                stringAtXY(4+i,23+(j*5),"X",RED_BLACK);     /* if signal is low, show a red X                 */
          } // End for (int = j
       }  // End for (int = i
       refresh();
    } else {
       endwin();                         /* end window                      */
       shwlib = 0;                       /* do not show lib panel anymore */
       freopen("/dev/tty", "w", stdout); /* resume normal console output */
    }
    return;
#endif
}

//*********************************************************************
//  Function to discard line buffer conetent (set length to zero)     *
//*********************************************************************
void proc_LIBdisbuf(int k)  {
   if ( LIBline[k]->LIBrlen != 0) {
      pthread_mutex_lock(&line_lock);
      LIBline[k]->LIBrlen = 0;                              // set to no data in buffer.
      pthread_mutex_unlock(&line_lock);
      if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04)) {           // Trace line activities ?
         fprintf(S_trace, "%s #04L%1d< buffer content discarded\n", TimeStamp(), k);
      }
   }
}

//*********************************************************************
//  Function Shift all chacters in an array to the left.              *
//*********************************************************************
static int ShiftLeft(char *buffer, int len)  {
   int i;
   for (i = 1; i < len; i++) {
      buffer[i - 1] = buffer[i];
   }
   len--;
   return len;
}

//*********************************************************************
// Function to check if socket is (still) connected                   *
//*********************************************************************
static bool IsSocketConnected(int sockfd) {
#if defined(__GENODE__)
   /* A crosslink is connected for as long as it is open; see the same
      function in i3705_chan_T2.c. */
   return (sockfd > 0) ? true : false;
#else
   int rc;
   struct sockaddr_in peer_addr;
   int addrlen = sizeof(peer_addr);
   if (sockfd < 1) {
      return false;
   }
   rc = getpeername(sockfd, (struct sockaddr*)&peer_addr, &addrlen);
   if (rc != 0) {
      return false;
   }
   return true;
#endif
}

//*******************************************************************************************
// Check the RS232 signals from the DTE, Update DCE accordingly                             *
//*******************************************************************************************
static void ReadSig(int k) {
   int rc, pendingrcv;
   uint8_t sig;
   RS232x[k] = 0;                                        // Flag transmit off
   pthread_mutex_lock(&rs232_lock);

 if ((RS232[k] & RTS) && (RS232[k] & DSR) && (!(RS232[k] & CTS))) {   // If remote DCE has set DSR and CTS was not yet high....
     RS232[k] |= CTS;                                 // send request to remote DCE
      if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04)) {                // Trace line activities ?
         fprintf(S_trace, "%s #04L%1d> Raised CTS\n",TimeStamp(), k);
      }
 }
 if (!(RS232[k] & RTS) && (RS232[k] & CTS))  {          // If remote DCE drops RTS....
     RS232[k] &= ~CTS;                                  // ...pass it on the the remote DCE
      if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04)) {                  // Trace line activities ?
         fprintf(S_trace, "%s #04L%1d> Dropped CTS\n",TimeStamp(), k);
      }
 }
 if  ((RS232[k] & DTR) && (RS232[k] & DCD) && (!(RS232[k] & DSR))) {             // If DTE raise DTR and DCD is high...
     RS232[k] |= DSR;                                 // ...raise DSR
      if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04)) {                   // Trace line activities ?
         fprintf(S_trace, "%s #04L%1d> Raised DSR\n",TimeStamp(), k);
      }
 }

 if  ((!(RS232[k] & DTR)) && (RS232[k] & DSR)) {                                // If DTE sets DTR  high...
     RS232[k] &= ~DSR;                                 // ...Drop dsr
      if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04)) {                  // Trace line activities ?
         fprintf(S_trace, "%s #04L%1d> Dropped DSR\n",TimeStamp(), k);
      }
 }
   pthread_mutex_unlock(&rs232_lock);
   return;
}
//*********************************************************************
// Receive data from the line (SDLC or BSC frame)                     *
// If an error occurs, the connection will be closed                  *
//*********************************************************************
int ReadLIB(int k) {
   int rc, j, pendingrcv;
   LIBline[k]->LIBrlen = 0;                                         // Preset to no data received.
   LIBline[k]->LIBbptr = 0;                                         // Preset to beginning of buffer.
   rc = -1;                                                         // Preset return coe
   if (LIBline[k]->d327x_fd > 0) {                                  // If there is a connection...
      if (IsSocketConnected(LIBline[k]->d327x_fd)) {                // ...and if it is still alive...
         pendingrcv = 0;
#if defined(__GENODE__)
         /* TIMEOUT 0, NOT 10, AND THE DIFFERENCE IS THE WHOLE LINE.
          *
          * Upstream asks ioctl(FIONREAD) here, which answers instantly with a
          * byte count. This is the SCANNER asking for the next received
          * CHARACTER (proc_LIBrdata calls it whenever the line buffer is
          * empty), so it is on the hot path and is reached for every line on
          * every scan cycle. A 10 ms wait turns "is there a character?" into
          * a sleep, and with four lines that is up to 40 ms of dead time per
          * cycle. The receive side then cannot keep up with the line: bytes
          * all arrive correctly, but the far NCP's N(R) stops advancing, it
          * retransmits, and the remote load dies on FRMR.
          *
          * poll() with a zero timeout is what FIONREAD meant: ask, do not
          * wait. */
         pendingrcv = genode_ca_avail(LIBline[k]->d327x_fd, 0);
         rc = 0;
#else
         rc = ioctl(LIBline[k]->d327x_fd, FIONREAD, &pendingrcv);   // ...check for any data in the TCP buffer
#endif
         if (pendingrcv > 0) {
            pthread_mutex_lock(&line_lock);
            LIBline[k]->LIBrlen = read(LIBline[k]->d327x_fd, LIBline[k]->LIB_rbuf, BUFLEN_327x); // If data available, read it
            pthread_mutex_unlock(&line_lock);
         }  // End if (pendingrcv > 0)
         return 0;                                                  // return
      } else {
#if defined(__GENODE__)
         /* A crosslink does not disconnect, so this branch is unreachable;
            it is left rather than removed so the native build is unchanged. */
         return -1;
#else
         // Close the connection and add a new polling event, to allow for a re-connect.
         close (LIBline[k]->d327x_fd);
         LIBline[k]->d327x_fd = 0;
         RS232[k] &= ~(DCD | DSR | RI);                             // Set DCD, DSR and RI off;
         printf("\rLIB: 327x disconnected from line-%d\n", k+LIBLBASE);
         // Re-enable event polling for this line (i.e. poll for a new connection request).
         event.events = EPOLLIN;
         event.data.fd = LIBline[k]->line_fd;
         if (epoll_ctl(LIBline[k]->epoll_fd, EPOLL_CTL_MOD, LIBline[k]->line_fd, &event)) {
            printf("\rLIB: Modifying polling event error %d for line-%d\n", errno, k+LIBLBASE);
            close(LIBline[k]->epoll_fd);
            return -1;
         }
#endif
      }  // End if IsSocketConnected
      return -1;
   }  // End  if (LIBline[k]->d327x_fd > 0)
   return rc;
}


//*********************************************************************
//   Get transmitted Character from scanner                           *
//*********************************************************************
void proc_LIBtdata (unsigned char LIBtchar, uint8_t state, int line) {
uint8 rc;

   // Scanner state C or D  means end of transmission, send buffer to controller.
   if ((LIBline[line]->LIBsync == 1) && ((state == 0xC) || (state == 0xD))) {
      if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04)) {                  // Trace line activities ?
         fprintf(S_trace, "%s #04L%1d> Transmit Buffer (%d bytes): ", TimeStamp(), line, LIBline[line]->LIBtlen);
         for (int i = 0; i < LIBline[line]->LIBtlen; i ++) {
             fprintf(S_trace, "%02X ", LIBline[line]->LIB_tbuf[i]);
         }  // End for
         fprintf(S_trace, "\n");
      }  // End if Sdbg_reg

      LIBline[line]->LIBsync = 0;                                    // Reset SYNC.
      //**************************************************************
      rc = send(LIBline[line]->d327x_fd, LIBline[line]->LIB_tbuf, LIBline[line]->LIBtlen, 0); // Send frame
      //**************************************************************
      LIBline[line]->LIBtlen = 0;                                    // Reset transmitted data length.
   }  // End if state

   // If we are in receive mode, append the character to the buffer.
   if ((LIBline[line]->LIBsync == 1) && (state != 0x8)) {
      LIBline[line]->LIB_tbuf[LIBline[line]->LIBtlen] = LIBtchar;    // Add character to buffer
      LIBline[line]->LIBtlen++;                                      // Increment length
   }  // End if LIBline[line]->LIBsync

   // If we are in pcf 9, following a pcf D, we continue trnasmission with a new frame
   if ((LIBline[line]->LIBsync == 0) && (state == 0x9) && (LIBline[line]->prevPCF == 0x0D)) {
      LIBline[line]->LIBsync = 1;                                    // Indicate we are in receive mode
      LIBline[line]->LIBtlen = 0;                                    // Ensure length is set to zero
      LIBline[line]->LIB_tbuf[LIBline[line]->LIBtlen] = LIBtchar;    // Add character to buffer
      LIBline[line]->LIBtlen++;                                      // Increment length
   }  // End if LIBline[line]->LIBsync

   // Check if we are in pcf state 8. This indicates the start of a transmission.
   // However, if we are still in receiving mode, there is no reset of the buffer length and
   // we continue appending to the current buffer.
   if ((state == 0x8) && (LIBline[line]->LIBsync != 1)) {
      LIBline[line]->LIBsync = 1;                                    // Indicate we are in receive mode
      LIBline[line]->LIBtlen = 0;                                    // Ensure length is set to zero
   }  // End if state == 0x8

   // Check if we received two consecutive SYN characters in the text...
   // ...these are time-fill sync and must be removed
   //if ((LIBline[line]->LIBtlen > 3) && (LIBtchar == SYN) && (LIBline[line]->LIB_tbuf[LIBline[line]->LIBtlen-2] == SYN))  {
   //   LIBline[line]->LIBtlen = LIBline[line]->LIBtlen - 2;
   //   printf("\rLIB: Double SYNC detected, status = %01X, previous status = %01X\n", state, prev_state);
   //}  // End if BSCtlen
   LIBline[line]->prevPCF = state;                                   // Remember this state
   return;                                                           // back to schanner
}

//*********************************************************************
//   Send received Character to scanner                               *
//*********************************************************************
int  proc_LIBrdata (unsigned char *LIBrchar, uint8_t state, int line) {
uint8 rc;
   if (LIBline[line]->LIBrlen == 0) {                                // If line buffer empty...
      //*************************************************************
      rc = ReadLIB(line);                                            // ...receive new data if available
      if ((rc != 0) && (Sdbg_flag == ON) && (Sdbg_reg & 0x04)) {      // Trace line activities ?
         fprintf(S_trace, "%s #04L%1d> ReadLIB rc = %d\n", TimeStamp(), line, rc);
      }
      //*************************************************************
      if ((LIBline[line]->LIBrlen != 0) && (Sdbg_flag == ON) && (Sdbg_reg & 0x04)) {   // Trace line activities ?
         fprintf(S_trace, "%s #04L%1d> Receive Buffer (%d bytes): \n", TimeStamp(), line, LIBline[line]->LIBrlen);
         fprintf(S_trace, "%s #04L%1d> [%04x] ", TimeStamp(), line, 0);
         for (int i = 0; i < LIBline[line]->LIBrlen; i ++) {
            fprintf(S_trace, "%02X ", LIBline[line]->LIB_rbuf[i]);
            if ((i + 1) % 16 == 0)
               fprintf(S_trace, "\n%s #04L%1d> [%04x] ", TimeStamp(), line, i+1);
         }  // End for
         fprintf(S_trace, "\n");
         fflush(S_trace);
      }  // End if Sdbg_reg
   }
   rc = 0;                                                           // preset to no characters to transmit
   if (LIBline[line]->LIBrlen > 0) {                                 // If there is data in the buffer....
     // *LIBrchar = LIBline[line]->LIB_rbuf[0];                        // ...point to first character
      *LIBrchar = LIBline[line]->LIB_rbuf[LIBline[line]->LIBbptr];   // ...get character pointed to by buffer pointer
      if (!((state == 0x4) || (state == 0x5)))                       // If we are not in PCF 4 or 5...
         LIBline[line]->LIBbptr++;                                   // Increment buffer pointer
         //LIBline[line]->LIBrlen = ShiftLeft(LIBline[line]->LIB_rbuf, LIBline[line]->LIBrlen); // shift whole buffer to left.
      //if (LIBline[line]->LIBrlen == 0)                               // If buffer fully processed...
      if (LIBline[line]->LIBrlen == LIBline[line]->LIBbptr) {        // If buffer fully processed...
         LIBline[line]->LIBrlen = 0;                                 // Reset length
         rc = 2;                                                     // ...indicate last character, which also means end of frame
       } else {                                                      // Otherwise...
         rc = 1;                                                     // ...One character to transmit, there are more
       }
   }  // End if (LIBline[line]->LIBrlen > 0)
   return rc;                                                        // Back to scanner
}

//*********************************************************************
//   Thread to handle connections from the 327x cluster emulator      *
//*********************************************************************
void *LIB_thread(void *arg)
{
   int    devnum;                  /* device nr copy for convenience */
   int    sockopt;                 /* Used for setsocketoption       */
   int    pendingrcv;              /* pending data on the socket     */
   int    event_count;             /* # events received              */
   int    rc, rc1;                 /* return code from various rtns  */
   int    alive = 1;               /* Enable KEEP_ALIVE              */
   int    idle = 5;                /* First  probe after 5 seconds   */
   int    intvl = 3;               /* Subsequent probes after 3 sec  */
   int    cntpkt = 3;              /* Timeout after 3 failed probes  */
   int    timeout = 1000;
#if !defined(__GENODE__)
   struct sockaddr_in sin, *sin2;  /* bind socket address structure  */
   struct ifaddrs *nwaddr, *ifa;   /* interface address structure    */
   char   *ipaddr;
#endif

   printf("\rLIB: Thread %d started succesfully...\n", THREAD_ID());

   for (int j = 0; j < MAX_LINES; j++) {
      LIBline[j] =  malloc(sizeof(struct LIBLine));
      LIBline[j]->linenum = j;
      LIBline[j]->LIBrlen = 0;
      LIBline[j]->LIBtlen = 0;
      LIBline[j]->LIBsync = 0;
   }  // End for j = 0
   LIBready = 1;             /* LIB structures allocated and initialized */

#if defined(__GENODE__)
   /*
    * The lines, already open.
    *
    * Upstream this is a bind, a listen and an epoll registration per line,
    * and then an accept when a 3274, 3271, DLSw or Trunk connects. All of it
    * is replaced by taking the crosslink the component config named: a line
    * with a crosslink IS a line with something on the end of it, so DCD and
    * RI go up here rather than in an accept that will never happen.
    *
    * A line with no crosslink stays at fd 0, which is exactly the state an
    * unconnected line had before, and every caller already tests for it.
    */
   for (int j = 0; j < MAX_LINES; j++) {
      LIBline[j]->d327x_fd = genode_line_fd((unsigned)j);
      if (LIBline[j]->d327x_fd < 0) {
         LIBline[j]->d327x_fd = 0;
         continue;
      }
      RS232[j] |= (DCD | RI);                      // DCD and Ring on
      printf("\rLIB: Line-%d attached\n", j + LIBLBASE);
   }

   while (1) {
      for (int k = 0; k < MAX_LINES; k++)
         ReadSig(k);
      usleep(50000);
   }
   return NULL;
#else
   getifaddrs(&nwaddr);      /* Get TCP network address */
   for (ifa = nwaddr; ifa != NULL; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr->sa_family == AF_INET && strcmp(ifa->ifa_name, "lo")) {
         sin2 = (struct sockaddr_in *) ifa->ifa_addr;
         ipaddr = inet_ntoa((struct in_addr) sin2->sin_addr);
         if (strcmp(ifa->ifa_name, "eth")) break;
      }
   }
   printf("\rLIB: Using TCP network Address %s on %s for 327x connections\n", ipaddr, ifa->ifa_name);

   for (int j = 0; j < MAX_LINES; j++) {
      if ((LIBline[j]->line_fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)) == -1)
         printf("\rLIB: Endpoint creation for 327x failed with error %s \n", strerror(errno));

      // Reuse the address regardless of any spurious connection on that port
      sockopt = 1;
      setsockopt(LIBline[j]->line_fd, SOL_SOCKET, SO_REUSEADDR, (void*)&sockopt, sizeof(sockopt));

      // Bind the socket
      sin.sin_family = AF_INET;
      sin.sin_addr.s_addr = inet_addr(ipaddr);
      sin.sin_port = htons(37500 + LIBLBASE + j);        // <=== port related to line number
      if (bind(LIBline[j]->line_fd, (struct sockaddr *)&sin, sizeof(sin)) < 0) {
          printf("\rLIB: Bind line-2%d socket failed\n", j);
          free(LIBline[j]);
          exit(EXIT_FAILURE);
      }

      // Listen and verify
      if ((listen(LIBline[j]->line_fd, 10)) != 0) {
         printf("\rLIB: Line-%d Socket listen failed %s\n", j, strerror(errno));
          free(LIBline[j]);
          exit(-1);
      }

      // Add polling events for the port
      LIBline[j]->epoll_fd = epoll_create(1);
      if (LIBline[j]->epoll_fd == -1) {
         printf("\rLIB: Failed to created the line-%d epoll file descriptor\n", j);
         free(LIBline[j]);
         exit(-2);
      }
      event.events = EPOLLIN;
      event.data.fd = LIBline[j]->line_fd;
      if (epoll_ctl(LIBline[j]->epoll_fd, EPOLL_CTL_ADD, LIBline[j]->line_fd, &event) == -1) {
         printf("\rLIB: Add polling event failed for line-%d with error %s \n", j, strerror(errno));
         close(LIBline[j]->epoll_fd);
         free(LIBline[j]);
         exit(-3);
      }
      printf("\rLIB: Line-%d ready, waiting for connection on TCP port %d\n", j, 37500 + LIBLBASE + j );
   }
   //
   // Poll briefly for connect requests. If a connect request is received, proceed with connect/accept the request.
   // Next, check all active connection for input data.
   //
   while (1) {
      for (int k = 0; k < MAX_LINES; k++) {
         event_count = epoll_wait(LIBline[k]->epoll_fd, events, 1, 50);
         for (int i = 0; i < event_count; i++) {
            LIBline[k]->d327x_fd = accept(LIBline[k]->line_fd, NULL, 0);
            if (LIBline[k]->d327x_fd < 1) {
               printf("\rLIB: accept failed for connection on line-%d %s\n", k+LIBLBASE, strerror(errno));
            } else {
               if (setsockopt(LIBline[k]->d327x_fd, SOL_SOCKET, SO_KEEPALIVE, (void *)&alive, sizeof(alive))) {
                  perror("ERROR: setsockopt(), SO_KEEPALIVE");
                  return NULL;
               }
               if (setsockopt(LIBline[k]->d327x_fd, IPPROTO_TCP, TCP_KEEPIDLE, (void *)&idle, sizeof(idle))) {
                  perror("ERROR: setsockopt(), SO_KEEPIDLE");
                  return NULL;
               }
               if (setsockopt(LIBline[k]->d327x_fd, IPPROTO_TCP, TCP_KEEPINTVL, (void *)&intvl, sizeof(intvl))) {
                  perror("ERROR: setsockopt(), SO_KEEPINTVL");
                  return NULL;
               }
               if (setsockopt(LIBline[k]->d327x_fd, IPPROTO_TCP, TCP_KEEPCNT, (void *)&cntpkt, sizeof(cntpkt))) {
                  perror("ERROR: setsockopt(), SO_KEEPCNT");
                  return NULL;
               }
               RS232[k] |= (DCD | RI);        // DCD and Ring on
            }  // End if LIBline[k]->d327x_fd
         }  // End for int i
            ReadSig(k);
      }  // End for int k
     if (shwlib == 1) LIBpanel_Init();
     if (shwlib == 2) LIBpanel_Updt();
   }  // End while(1)

   return NULL;
#endif
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

void LIBpanel_Init() {
#if defined(__GENODE__)
   shwlib = 0;
   return;
#else
   // *****************************************************************************************
   // Because SIMH has been interrupted to set the swwlib value to 1 (= show panel), we need to
   // hold of showing the panel, so "c" (continue) can be entered.
   // We can check for processing to be started by looking at the cycle counter.
   // *****************************************************************************************
   uint32_t old_cucr;                         // To save cycle counter.
   old_cucr = Eregs_Inp[0x7A];                // save current cycle counter
   while (Eregs_Inp[0x7A] == old_cucr)        // wait until cycle counter changes (= processing continues)
      sleep(1);
   // *********************************************************************************
   // Build the lib panel.
   // If the panel is ready, set the shwlib value to 2, which enables the update cycle.
   // *********************************************************************************
   FILE *f = fopen("/dev/tty", "r+");
   freopen("/dev/null", "w", stdout);
   SCREEN *libwin = newterm(NULL, f, f);
   set_term(libwin);
   refresh();
   curs_set(0);

   if (has_colors() == FALSE) {
      endwin();                               // End window
      printf("\nLIB: No colour suipport for your terminal\r");
      shwlib = 0;                /* do not show front panel anymore */
      freopen("/dev/tty", "w", stdout);
      return;
   }  // End if (has_colors)
                                                       /* Define our colour patterns     */
   start_color();
   init_color(COLOR_YELLOW, 1000, 1000, 0);            /* the main colour defintions     */
   init_color(COLOR_RED, 1000, 0, 0);
   init_color(COLOR_BLUE, 0, 1000, 1000);
   init_color(COLOR_GREEN, 0, 1000, 0);
   init_pair(RED_BLACK, COLOR_RED, COLOR_BLACK);       /* Define the set of foreground with background colours */
   init_pair(GREEN_BLACK, COLOR_GREEN, COLOR_BLACK);
   init_pair(YELLOW_BLACK, COLOR_YELLOW, COLOR_BLACK);
   init_pair(WHITE_BLACK, COLOR_WHITE, COLOR_BLACK);
   init_pair(BLUE_BLACK, COLOR_BLUE, COLOR_BLACK);
   init_pair(BLACK_RED, COLOR_BLACK, COLOR_RED);
   init_pair(BLACK_GREEN, COLOR_BLACK, COLOR_GREEN);
   init_pair(BLACK_YELLOW, COLOR_BLACK, COLOR_YELLOW);
   init_pair(BLACK_WHITE, COLOR_BLACK, COLOR_WHITE);
   init_pair(BLACK_BLACK, COLOR_BLACK, COLOR_BLACK);
   noecho();                                           /* Do not echo any keyboard input */
   nodelay(stdscr,TRUE);                               /* do not wait for keyboard input */
   keypad(stdscr, TRUE);
   // Show the LIB panel
   LIBpanel();                                         /* build the panel                */
   refresh();                                          /* show it on the screen          */
   shwlib = 2;                                         /* swithc to update mode          */
#endif
 }

// ******************************************************************
// Function to write coloured text on the front panel at row x, col y
// ******************************************************************

static void stringAtXY (int x, int y, char *buf, int colour) {
#if defined(__GENODE__)
   (void)x; (void)y; (void)buf; (void)colour;
   return;
#else

   wmove(stdscr, x, y);
   attron(COLOR_PAIR(colour));
   //attron(A_BOLD);
   printw("%s",buf);
   attroff(COLOR_PAIR(colour));
   return;
#endif
}

// ******************************************************************
// Function to write coloured integer  on the front panel at row x, col y
// ******************************************************************

static void integerAtXY (int x, int y, int value, int colour) {
#if defined(__GENODE__)
   (void)x; (void)y; (void)value; (void)colour;
   return;
#else

   wmove(stdscr,x,y);
   attron(COLOR_PAIR(colour));
   printw("%d", value);
   attroff(COLOR_PAIR(colour));
   return;
#endif
}

