/* Copyright (c) 2023, Edwin Freekenhorst and Henk Stegeman

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

   i327x_3271.C   (c) Copyright  Edwin Freekenhost & Henk Stegeman

   This module appears to the 3705/3720 as a 3271 cluster controller
   Depending on the defintion, 1 or more 3271's are 'IML-ed', making
   the defined LU's availble for sign-on. A terminal may connect to
   the desired 3271 by selecting the approriate telnet port number.
   The TCP ports are defined as 32711 for the first IBM3271, 32712 for
   the 2nd, etc.
*/

#include <inttypes.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <ctype.h>
#include <pthread.h>
#include <string.h>
#include <ifaddrs.h>
#include "i327x_327x.h"
#include "ebcdic.h"
#include <errno.h>
#if defined(__GENODE__)
/* Same substitutions as the 3274's, and for the same reasons -- see
   0007-i3274-genode-ipc.patch. This is the BSC cluster where that is the
   SDLC controller; the seams are identical. */
#include <poll.h>
#include <errno.h>
#include "genode_line.h"
#include "genode_xlink.h"
#else
#include <sys/epoll.h>
#endif
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/time.h>
#include <time.h>


#if !defined(min)
#define min(a,b)  (((a) <= (b)) ? (a) : (b))
#endif

#define BUFPD 0x1C
#define GPOLL 1
#define SPOLL 2
#define SELECT 3

uint16_t Tdbg_flag = OFF;          /* 1 when Ttrace.log open            */
FILE *T_trace;

struct sockaddr_in servaddr;
struct sockaddr_in    sin1, *sin2; /* bind socket address structure     */
#if !defined(__GENODE__)
struct epoll_event event, events[MAXCLSTR*2];
#endif
struct ifaddrs *nwaddr, *ifa;      /* interface address structure       */
in_addr_t  lineip;                 /* BSC line listening address        */
uint16_t   lineport;               /* BSC line listening port           */
int        clubsc_fd;              /* BSC line data connection          */
int        sockopt;                /* Used for setsocketoption          */
int        pendingrcv;             /* pending data on the socket        */
int        event_count;            /* # events received                 */
char       *ipaddr;
uint8_t     bfr[256];

struct CB327x *clu[MAXCLSTR];          /* 3271 control block */
struct IO3270 *ioblk[MAXCLSTR][MAXLU]; /* 3270 data buffer   */

uint8_t CLSTR_config[MAXCLSTR][4] = {{0x20,0x40, 0x40,0x40}};

uint8_t BSC_rbuf[65536];
uint8_t BSC_tbuf[65536];

static char * TimeStamp();
void commadpt_read_tty(struct CB327x *i327x, struct IO3270 *ioblk, uint8_t * bfr, int lunum, int len);
int send_packet(int csock, uint8_t *buf, int len, char *caption);
int connect_client (int *csockp, BYTE i327xnump, BYTE *lunump, BYTE *lunumr);
int SocketReadAct (int fd);

uint8_t  ACKreq = 0;
uint8_t  NAKreq = 0;
uint8_t  EOTreq = 0;
uint8_t  lastACK = 1;                      // Value of last ACK
uint8_t  ENQ_type;                         // Type of ENQ.
                                           // 0 = Init xmit, 1 = GPOLL, 2 = Special POLL, 3 = Select
//uint8_t GPOLL[] = {
//      SYN, SYN, 0x40,0x40, 0x7F, 0x7F, ENQ, PAD}; // General POLL
//uint8_t SPOLL[] = {
//      SYN, SYN, 0x40,0x40, 0x40, 0x40, ENQ, PAD}; // Specific Poll
//uint8_t SELECT[] = {
//      SYN, SYN, 0x60,0x60, 0x40, 0x40, ENQ, PAD}; // Selection
uint8_t NAK_dlc[] = {
      SYN, SYN, NAK, PAD};                  // NAK
uint8_t EOT_dlc[] = {
      SYN, SYN, EOT, PAD};                  // EOT
uint8_t ENQ_dlc[] = {
      SYN, SYN, ENQ, PAD};                  // ENQ
uint8_t RVI_dlc[] = {
      SYN, SYN, DLE, 0x7C, PAD};            // RVI
uint8_t ACK0_dlc[] = {
      SYN, SYN, DLE, 0x70, PAD};            // ACK 0
uint8_t ACK1_dlc[] = {
      SYN, SYN, DLE, 0x61, PAD};            // ACK 1
uint8_t STX_dlc[] = {
      SYN, SYN, STX};                       // STX
uint8_t STX_addr[] = {
      SYN, SYN, STX, 0x40, 0x40};           // STX + CU addr
uint8_t SOH_stat[] = {
      SYN, SYN, SOH, 0x6C, 0xD9, STX, 0x40, 0x40}; // SOH (status) + STX
// Sense and Status uint8_ts
uint8_t SS_IR[] = { 0x40, 0x50 };        // Sense/Status : Intervention Required (Not Ready)
uint8_t SS_DE[] = { 0xC2, 0x40 };        // Sense/Status : Device End (Ready)
//int write_socket( int fd, const void *_ptr, int nuint8_ts );
//int send_packet (int csock, uint8_t *buf, int len, char *caption);

int double_up_iac (uint8_t *buf, int len);

/*-------------------------------------------------------------------*/
/* Function to calculate The Cyclic Redundancy Check                 */
/*-------------------------------------------------------------------*/
int crc16(unsigned char *ptr, int count)
{
   uint16_t crc;
   char i;

   crc = 0x0000;                         /* start value               */
   while (--count >= 0) {
      crc = crc ^ (unsigned int) *ptr++;
      i = 8;
      do {
         if (crc & 0x0001)
            crc = (crc >> 1) ^ CRC_POLY_16;
         else
            crc = (crc >> 1);
      } while(--i);
   }
   crc = (crc << 8) + (crc >> 8);        /* swap high and low bytes */
   return (crc);
}

/*-------------------------------------------------------------------*/
/* Process BSC message                                               */
/*-------------------------------------------------------------------*/
/*
   BSC text layout:
   +----------+-----+-----+-------//-------+-----+---+---+-----+
   | AA | SYN | SYN.| SOT |     Text       | EOT |  CRC  | PAD |
   +----------+-----+-----+-------//-------+-----+---+---+-----+

   BSC long text layout:
   +----------+-----+-----+-------//-----+-----+-----+-------//------+-----+----+---+-----+
   | AA | SYN | SYN.| SOT |     Text     | SYN | SYN |     Text      | EOT |  CRC   | PAD |
   +----------+-----+-----+-------//-----+-----+-----+-------//------+-----+----+---+-----+

   3271 General Poll:
   CU0:  40 40 7F 7F
   CU1:  C1 C1 7F 7F
   CU2:  C2 C2 7F 7F
   CU3:  C3 C3 7F 7F
   3271 Specific Poll:
   CU0 Terminal0: 40 40 40 40
   CU0 Terminal1: 40 40 C1 C1
   CU0 Terminal2: 40 40 C2 C2
   CU0 Terminal3: 40 40 C3 C3
   3271 Device Select:
   CU0 Terminal0: 60 60 40 40
   CU1 Terminal1: 61 61 C1 C1
   CU2 Terminal2: E2 E2 C2 C2
   CU3 Terminal3: E3 E3 C3 C3
*/
/*
 * BSC_tbuf IS uint8_t HERE, AND THAT IS THE WHOLE OF A REAL BUG.
 *
 * It used to be `char*`, while the caller passes &BSC_tbuf[Fptr] from a
 * `uint8_t BSC_tbuf[65536]` -- gcc warns "pointer targets ... differ in
 * signedness" and the warning is the bug, not noise. The record test below
 * is
 *
 *     if ((BSC_tbuf[BSCtlen-1] == PAD) && (BSC_tbuf[BSCtlen-4] == ETX))
 *
 * with PAD 0xff and ETX 0x03. Read through a SIGNED char the PAD byte is -1
 * and can never equal 255, so the test ALWAYS failed and every data block
 * was NAKed -- on every platform where char is signed, which includes
 * x86 Linux. ETX passed only because 0x03 is positive either way.
 *
 * Measured: a complete, well-formed 1332-byte block --
 *   CLU: BAD RECORD len=1332 last=FFFFFFFF etx_pos=03
 * -- rejected 12 times a boot. The 0xFFFFFFFF is the sign extension.
 *
 * The strncmp calls below now cast: they are comparing DLC byte patterns,
 * not text, so the signedness was never meaningful to them.
 */
int proc_BSC(uint8_t* BSC_tbuf, uint16_t BSCtlen) {
   uint8_t  Dbuf[BUFLEN_3270];            // Data buffer
   uint16_t BSCrlen, Dlen;
   int  i, rc, ckesc;
   uint16_t CRCds, CRCck;


   if ((BSC_tbuf[0] == SYN) && (BSC_tbuf[1] == SYN)) {
      if (strncmp((char*)BSC_tbuf, EOT_dlc, sizeof(EOT_dlc)) == 0)  {     // if EOT...
         BSCrlen = 0;
         if (Tdbg_flag == ON)
            fprintf(T_trace, "%s Received EOT...\n",TimeStamp());
      }
      if (strncmp((char*)BSC_tbuf, NAK_dlc, sizeof(NAK_dlc)) == 0)  {     // if NAK...
         BSCrlen = 0;
         if (Tdbg_flag == ON)
            fprintf(T_trace, "%s Received NAK...\n",TimeStamp());
      }
      if ((strncmp((char*)BSC_tbuf, ACK0_dlc, sizeof(ACK0_dlc)) == 0) ||  // if ACK0 or...
         (strncmp((char*)BSC_tbuf, ACK1_dlc, sizeof(ACK1_dlc)) == 0)) {   // if ACK1...
      if (Tdbg_flag == ON)
         fprintf(T_trace, "%s Received ACK0/ACK1...\n",TimeStamp());
      if (EOTreq == 1) {
         memcpy(&BSC_rbuf, EOT_dlc, sizeof(EOT_dlc));              // ...send EOT (nothing to send)
         BSCrlen = sizeof(EOT_dlc);
         EOTreq = 0;
         if (Tdbg_flag == ON)
            fprintf(T_trace, "%s Sending EOT...\n",TimeStamp());         // Nothing more to send
         }
      }
      //***********************************************************
      // Check for init transmission ENQ
      //***********************************************************
      if (strncmp((char*)BSC_tbuf, ENQ_dlc, sizeof(ENQ_dlc)) == 0) {      // if ENQ...
         if (Tdbg_flag == ON)
            fprintf(T_trace, "%s Received Init transmission ENQ...\n",TimeStamp());
         lastACK = 1;                                              // Respond with ACK...
         ACKreq = 1;                                               // ...send ACK
      }
      //***********************************************************
      // Check for  POLL ENQ and determine type
      //***********************************************************
      if ((BSCtlen == 8) && (BSC_tbuf[6] == ENQ)) {                // If POLL ENQ received or
         if  (BSC_tbuf[2]  & 0x20) ENQ_type = SELECT;
         else ENQ_type = SPOLL;
         if ((BSC_tbuf[4] == 0x7F) && (BSC_tbuf[5] == 0x7F)) ENQ_type = GPOLL;
         switch (ENQ_type) {
            //***********************************************************
            // Select type ENQ
            //***********************************************************
            case SELECT:
               if (Tdbg_flag == ON)
                  fprintf(T_trace, "%s Received Select...\n",TimeStamp());
               if (clu[0]->lu_fd[0] > 0) {                         // ...and if terminal connected
                  lastACK = 1;                                     // A select should always responds with ACK0
                  ACKreq = 1;                                      // ...send ACK
               } else {                                            // If terminal not connected...
                  memcpy(&BSC_rbuf, RVI_dlc, sizeof(RVI_dlc));     // ...send RVI (want to send status/sense)
                  BSCrlen = sizeof(RVI_dlc);
               }
            break;

            //***********************************************************
            // Specific poll type ENQ
            //***********************************************************
            case SPOLL:
               if (Tdbg_flag == ON)
                  fprintf(T_trace, "%s Received SPOLL...\n",TimeStamp());
               if (clu[0]->lu_fd[0] > 0) {                         // ...and if terminal connected
                  ACKreq = 1;                                      // ...send ACK
               } else {                                            // If terminal not connected...
                  memcpy(BSC_rbuf, SOH_stat, sizeof(SOH_stat));    // ...then begin with SYN SYN SOH
                  BSCrlen = sizeof(SOH_stat);
                  memcpy(&BSC_rbuf[BSCrlen], SS_IR, sizeof(SS_IR ));  // ...and add IR sense
                  BSCrlen = BSCrlen + sizeof(SS_IR);
                  BSC_rbuf[BSCrlen++] = ETX;                       // add ETX
                  //***********************************************************
                  //* Calculate CRC
                  //************************************************************
                  CRCck = crc16(&BSC_rbuf[3], BSCrlen-3);          // Calculate CRC, Exclude SYN SYN SOH
                  BSC_rbuf[BSCrlen++] = CRCck >> 8;                // First CRC 16 byte
                  BSC_rbuf[BSCrlen++] = CRCck & 0x00FF;            // Second CRC 16 byte
                  BSC_rbuf[BSCrlen++] = PAD;                       // Line turnaround
                  EOTreq = 1;                                      // Send EOT after receiving an ACK,
                  clu[0]->not_ready[0] = 1;
                  if (Tdbg_flag == ON)
                    fprintf(T_trace, "%s Sending Sense data %02X %02X...\n", TimeStamp(), BSC_rbuf[8], BSC_rbuf[9] );
               }  // End if (clu[0]->lu_fd[0] > 0)
            break;
            //***********************************************************
            // General poll type ENQ
            //***********************************************************
            case GPOLL:
               if (Tdbg_flag == ON)
                  fprintf(T_trace, "%s Received GPOLL...\n",TimeStamp());
               if (clu[0]->lu_fd[0] > 0) {                         // If terminal connected
                  if ((ioblk[0][0]->inpbufl > 0) && !(clu[0]->not_ready[0])) {  // Do we have data to transmit and terminal is ready... ?
                     memcpy(BSC_rbuf, STX_addr, sizeof(STX_addr)); // ...then begin with SYN SYN STX
                     BSCrlen = sizeof(STX_addr);
                     memcpy(&BSC_rbuf[BSCrlen], ioblk[0][0]->inpbuf, ioblk[0][0]->inpbufl);  // ...add the 3270 buffer content
                     BSCrlen = BSCrlen + ioblk[0][0]->inpbufl;
                     BSC_rbuf[BSCrlen++] = ETX;                    // add ETX
                     //***********************************************************
                     //* Calculate CRC
                     //************************************************************
                     CRCck = crc16(&BSC_rbuf[3], BSCrlen-3);       // Calculate CRC, Exclude SYN SYN STX
                     BSC_rbuf[BSCrlen++] = CRCck >> 8;             // First CRC 16 byte
                     BSC_rbuf[BSCrlen++] = CRCck & 0x00FF;         // Second CRC 6 byte
                     BSC_rbuf[BSCrlen++] = PAD;                    // Line turnaround
                     if (Tdbg_flag == ON) {
                        fprintf(T_trace, "%s 3270 Output Buffer: ",TimeStamp());
                        for (int i = 0; i < ioblk[0][0]->inpbufl; i++) {
                           fprintf(T_trace, "%02X ", ioblk[0][0]->inpbuf[i]);
                        }
                        fprintf(T_trace, "\n");
                     }  // End if Debug
                     ioblk[0][0]->inpbufl = 0;
                     EOTreq = 1;                                   // Send EOT after receiving an ACK,
                  } else {
                     memcpy(&BSC_rbuf, EOT_dlc, sizeof(EOT_dlc));  // ... send EOT (nothing to send)
                     BSCrlen = sizeof(EOT_dlc);
                     if (Tdbg_flag == ON)
                        fprintf(T_trace, "%s Returning EOT (a)...\n",TimeStamp());
                  }  // End if ioblk
                  if (clu[0]->not_ready[0]) {                      // If previous Not Ready state....
                     memcpy(BSC_rbuf, SOH_stat, sizeof(SOH_stat));   //...then begin with SYN SYN SOH
                     BSCrlen = sizeof(SOH_stat);
                     memcpy(&BSC_rbuf[BSCrlen], SS_DE, sizeof(SS_DE ));  //...and add DE sense
                     BSCrlen = BSCrlen + sizeof(SS_DE);
                     BSC_rbuf[BSCrlen++] = ETX;                    //  add ETX
                     //************************************************************
                     //* Calculate CRC
                     //************************************************************
                     CRCck = crc16(&BSC_rbuf[3], BSCrlen-3);       // Calculate CRC, Exclude SYN SYN STX
                     BSC_rbuf[BSCrlen++] = CRCck >> 8;             // First CRC 16 byte
                     BSC_rbuf[BSCrlen++] = CRCck & 0x00FF;         // Second CRC 16 byte
                     BSC_rbuf[BSCrlen++] = PAD;                    // Line turnaround
                     EOTreq = 1;                                   // Send EOT after receiving an ACK,
                     clu[0]->not_ready[0] = 0;                     // Reset not_ready state
                     if (Tdbg_flag == ON)
                        fprintf(T_trace, "%s Sending Sense data %02X %02X...\n", TimeStamp(),BSC_rbuf[8], BSC_rbuf[9] );
                  }  // End if (clu[0]->not_ready[0])
               } else {
                  memcpy(&BSC_rbuf, EOT_dlc, sizeof(EOT_dlc));     // ... send EOT (nothing to send)
                  BSCrlen = sizeof(EOT_dlc);
                  if (Tdbg_flag == ON)
                     fprintf(T_trace, "%s Returning EOT (b)...\n",TimeStamp());
               }  // End if clu[0]->lu_fd[0]
            break;
        } // End switch ENQ_type
     } // End if BSCtlen == 8
     //***********************************************************
     //* ???
     //***********************************************************
     if (strncmp((char*)BSC_tbuf, STX_dlc, sizeof(STX_dlc)) == 0) {       // if STX...
        if (Tdbg_flag == ON)
           fprintf(T_trace, "%s Receiving %d bytes of data...\n",TimeStamp(), BSCtlen);
        // Get Dlen by searching for x'FF'. (PAD)
        // and copy data to the Dbuf.
        if ((BSC_tbuf[BSCtlen - 1] == PAD) && (BSC_tbuf[BSCtlen - 4] == ETX)) {  // If we have a complete record
            CRCds = (BSC_tbuf[BSCtlen - 3] << 8) + BSC_tbuf[BSCtlen - 2];
            if (Tdbg_flag == ON)
               fprintf(T_trace, "%s CRC = %04X\n", TimeStamp(), CRCds);
            // memcpy(&Dbuf, BSC_tbuf+3, BSCtlen-6);               // copy all data between STX and ETX
            // Dlen = BSCtlen-7;
            if (Tdbg_flag == ON)
              fprintf(T_trace, "%s Dlen=%d Dbuf[0]=%02X, Dbuf[Dlen-1]=%02X, BSCtlen=%d\n", TimeStamp(), Dlen, Dbuf[0], Dbuf[Dlen-1], BSCtlen );
            //***********************************************************
            //* Check CRC
            //***********************************************************
            // CRCck = crc16(Dbuf, Dlen);                          // Calculate CRC, Exclude IAC
            CRCck = crc16(BSC_tbuf+3, BSCtlen-6);                  // Calculate CRC, Exclude SOT header
            // Dbuf[Dlen++] = IAC;
            // Dbuf[Dlen++] = 0xEF;
            BSC_tbuf[BSCtlen-4] = IAC;
            BSC_tbuf[BSCtlen-3] = 0xEF;

            ckesc = 0;                                             // preset to not preceeded by ESC
            if (BSC_tbuf[3+ckesc] == 0x27) ckesc=1;                // If data begins with ESC exclude it
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "%s 3270 Input Buffer: ", TimeStamp());
               for (int i = 0; i < BSCtlen-5-ckesc; i ++) {
                  fprintf(T_trace, "%02X ", BSC_tbuf[i+3+ckesc]);
               }
               fprintf(T_trace, "\n");
            }
            if ((CRCds ^ CRCck) == 0x0000) {
               //************************************************************
               rc = send_packet (clu[0]->lu_fd[0], (uint8_t *) BSC_tbuf+3+ckesc, BSCtlen-5-ckesc, "3270 BSC Data");
               //************************************************************
               if (rc == 0) ACKreq = 1;
                  else NAKreq = 1;
#if defined( __GENODE__ )
               /* A FAILED SEND IS THE ONLY WAY WE LEARN A CLIENT HAS GONE.
                *
                * Genode's libc poll() sets only POLLIN and POLLOUT -- there is
                * a literal "XXX POLLERR not supported" in vfs_plugin.cc -- so
                * the POLLERR/POLLNVAL/POLLHUP branch in the read loop below
                * can never fire here. read() and MSG_PEEK are no better: on
                * this lwIP an idle socket answers as though the peer had gone,
                * which is why that loop must not close on a quiet read.
                *
                * So the write path is all there is, and until now it only
                * NAKed: the slot stayed occupied by a client that no longer
                * existed, with not_ready still 0. VTAM then retried into the
                * void and gave up, and every later client on this boot found
                * terminal 0 taken. Observed on genherc 2026-09-26, where it
                * cost a boot.
                *
                * Releasing it here mirrors the disconnect path below, and
                * not_ready = 1 matters as much as the close: it is what lets
                * the Device End branch tell VTAM the terminal came ready when
                * a new client attaches, so the line recovers instead of
                * staying dark until reboot. */
               if (rc != 0 && clu[0]->lu_fd[0] > 0) {
                  printf("\rCLU: send to terminal 0 on 3271-0 failed (%s);"
                         " releasing it\n", strerror(errno));
                  clu[0]->not_ready[0] = 1;
                  if (ioblk[0][0] != NULL) { free(ioblk[0][0]); ioblk[0][0] = NULL; }
                  clu[0]->actlu[0] = 0;
                  close (clu[0]->lu_fd[0]);
                  clu[0]->lu_fd[0] = 0;
                  clu[0]->is_3270[0] = 0;
                  if (clu[0]->lunum > 0 || clu[0]->lunum == 0xFF)
                     clu[0]->lunum = 0;      /* terminal 0 is available again */
               }
#endif
            } else {
               if (Tdbg_flag == ON)
                  fprintf(T_trace, "%s CRC error: actual: %04X Received: %04X\n", TimeStamp(), CRCds, CRCck);
               printf("%s CLU: CRC error: Received: %04X Calculated: %04X\n", TimeStamp(), CRCds, CRCck);
               NAKreq = 1;
            }  // End if CRCds
         } else {                                                  // Somethings wrong with the input record
            /* VALUES, NOT A HEX DUMP. Genode's LOG truncates a message at
               ~267 characters, which is exactly a prefix plus 65 hex
               triplets -- so every long dump on this path was cut at 65
               bytes, identically, in every boot and in the native reference
               too. That artifact was mistaken for a 65-byte truncated frame
               and cost an evening. Print the three bytes the test actually
               reads, and the length, and nothing else. */
            printf("\rCLU: BAD RECORD len=%d last=%02X etx_pos=%02X pre=%02X\n",
                   (int)BSCtlen,
                   (unsigned)BSC_tbuf[BSCtlen-1],
                   (unsigned)BSC_tbuf[BSCtlen-4],
                   (unsigned)BSC_tbuf[BSCtlen-5]);
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "%s 3270 Input Buffer: ", TimeStamp());
               for (int i = 0; i < BSCtlen; i ++) {
                  fprintf(T_trace, "%02X ", BSC_tbuf[i]);
               }
               fprintf(T_trace, "\n");
            }
            NAKreq = 1;
         }  // End BSC_tbuf
      }  // End strncmp STX
      if (ACKreq == 1) {
         if (lastACK == 1) {
            memcpy(&BSC_rbuf, ACK0_dlc, sizeof(ACK0_dlc));         // send ACK
            BSCrlen = sizeof(ACK0_dlc);
            lastACK = 0;
            if (Tdbg_flag == ON)
               fprintf(T_trace, "%s Returning ACK0...\n",TimeStamp());
         } else {
            memcpy(&BSC_rbuf, ACK1_dlc, sizeof(ACK1_dlc));         // send ACK
            BSCrlen = sizeof(ACK1_dlc);
            lastACK = 1;
            if (Tdbg_flag == ON)
               fprintf(T_trace, "%s Returning ACK1...\n",TimeStamp());
         }  // End if lastACK
         ACKreq = 0;
      }  // End if ACKreq
      if (NAKreq == 1) {
         memcpy(&BSC_rbuf, NAK_dlc, sizeof(NAK_dlc));              // send NACK
         BSCrlen = sizeof(NAK_dlc);
         NAKreq = 0;
         if (Tdbg_flag == ON)
            fprintf(T_trace, "%s Returning NAK...\n",TimeStamp());
      }  // End if NAKreq
   }  // End if BSC_buf SYN
   return BSCrlen;
}
// #####################################################################


/**********************************************************************/
/* Procedure to 'iml' the 3271                                        */
/**********************************************************************/
int proc_CLUiml() {
   for (BYTE j = 0; j < MAXCLSTR; j++) {
      clu[j] =  malloc(sizeof(struct CB327x));
      // Init sockets for LU's
      for (BYTE i = 0; i < MAXLU; i++) {
         clu[j]->lu_fd[i] = 0;
         clu[j]->not_ready[i] = 1;
      }
      clu[j]->lunum = 0;
      clu[j]->last_lu = 0;
      clu[j]->punum = j;
   }  // End for j = 0

#if defined(__GENODE__)
   /* One interface, no getifaddrs in lwIP, and which address the listener is
      reachable at is decided by the run script's tcp-forward. */
   ipaddr = NULL;
   printf("\rCLU: listening on every address this component has, for 3270 connections\n");
#else
   getifaddrs(&nwaddr);      /* get network address */
   for (ifa = nwaddr; ifa != NULL; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr->sa_family == AF_INET && strcmp(ifa->ifa_name, "lo")) {
         sin2 = (struct sockaddr_in *) ifa->ifa_addr;
         ipaddr = inet_ntoa((struct in_addr) sin2->sin_addr);
         if (strcmp(ifa->ifa_name, "eth")) break;
      }
   }
   printf("\rCLU: Using network Address %s on %s for 3270 connections\n", ipaddr, ifa->ifa_name);
#endif

   for (BYTE j = 0; j < MAXCLSTR; j++) {
      if ((clu[j]->pu_fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)) == -1)
         printf("\rCLU: Endpoint creation for 3271 failed with error %s ", strerror(errno));
      /* Reuse the address regardless of any */
      /* spurious connection on that port    */
      sockopt=1;
      setsockopt(clu[j]->pu_fd, SOL_SOCKET, SO_REUSEADDR, (void*)&sockopt, sizeof(sockopt));
      /* Bind the socket */
      sin1.sin_family=AF_INET;
#if defined(__GENODE__)
      sin1.sin_addr.s_addr = htonl(INADDR_ANY);
#else
      sin1.sin_addr.s_addr = inet_addr(ipaddr);
#endif
      sin1.sin_port=htons(32711+j);
      if (bind(clu[j]->pu_fd, (struct sockaddr *)&sin1, sizeof(sin1)) < 0) {
          printf("\rCLU: Bind 3271-%01X socket failed\n\r", j);
          free(clu[j]);
          return -1;
      }
      /* Listen and verify */
      if ((listen(clu[j]->pu_fd, 10)) != 0) {
         printf("\rCLU: 3174-%01X socket listen failed %s\n\r", j, strerror(errno));
          free(clu[j]);
          return -1;
      }
      // Add polling events for the port
#if !defined(__GENODE__)
      clu[j]->epoll_fd = epoll_create(1);
      if (clu[j]->epoll_fd == -1) {
         printf("\rCLU: failed to created the 3271-%01X epoll file descriptor\n\r", j);
         free(clu[j]);
         return -3;
      }
      event.events = EPOLLIN;
      event.data.fd = clu[j]->pu_fd;
      if (epoll_ctl(clu[j]->epoll_fd, EPOLL_CTL_ADD, clu[j]->pu_fd, &event) == -1) {
         printf("\rCLU: Add polling event failed for 3271-%01X with error %s \n\r", j,  strerror(errno));
         close(clu[j]->epoll_fd);
         free(clu[j]);
         return -4;
      }
#endif
      printf("\rCLU: 3271-%01X IML ready. TN3270 can connect to port %d \n\r)", j,32711+j);
   }  // End for j=0
   return 0;
}
/********************************************************************/
/* Procedure to check for 3270 connection or data requests          */
/********************************************************************/
int proc_3270 () {

   int    rc;
   //
   // Poll briefly for connect requests. If a connect request is received,
   // proceed with connect/accept the request.
   // Next, check all active connection for input data.
   //
   for (BYTE k = 0; k < MAXCLSTR; k++) {
#if defined(__GENODE__)
      {
         struct pollfd pfd;
         pfd.fd = clu[k]->pu_fd; pfd.events = POLLIN; pfd.revents = 0;
         event_count = (poll(&pfd, 1, 50) > 0 && (pfd.revents & POLLIN)) ? 1 : 0;
      }
#else
      event_count = epoll_wait(clu[k]->epoll_fd, events, MAXLU, 50);
#endif
      for (int i = 0; i < event_count; i++) {
         if (clu[k]->lunum != 0xFF) {                                            /* if avail LU pool not exhausted      */
            clu[k]->lu_fd[clu[k]->lunum]=accept(clu[k]->pu_fd, NULL, 0);
            if (clu[k]->lu_fd[clu[k]->lunum] < 1) {
               printf("\rCLU: accept failed for 3171-%01X %s\n", k, strerror(errno));
            } else {
               if (connect_client(&clu[k]->lu_fd[clu[k]->lunum], clu[k]->punum, &clu[k]->lunum, &clu[k]->lunumr))  {
                  clu[k]->is_3270[clu[k]->lunum] = 1;
               } else {
                  clu[k]->is_3270[clu[k]->lunum] = 0;
               }  // End if connect_client

               if (clu[k]->lunumr != clu[k]->lunum)  {                                 /* Requested terminal number is not the proposed terminal number   */
                  if (clu[k]->lu_fd[clu[k]->lunumr] < 1) {
                     clu[k]->lu_fd[clu[k]->lunumr] = clu[k]->lu_fd[clu[k]->lunum];     /* Copy fd to request lu                               */
                     clu[k]->is_3270[clu[k]->lunumr] = clu[k]->is_3270[clu[k]->lunum]; /* copy 3270 indicator                                 */
                     clu[k]->lu_fd[clu[k]->lunum] = 0;                                 /* clear fd in proposed lu number                      */
                     clu[k]->is_3270[clu[k]->lunum] = 0;                               /* clear 3270 indicator for proposed lu number         */
                     clu[k]->lunum = clu[k]->lunumr;                                   /* replace proposed lu number with requested lu number */
                  } else {
                     printf("\rCLU: requested terminal port %02X is not available, request denied\n", clu[k]->lunumr);
                  }  // End  if (pu2[k]->lu_fd[pu2[k]->lunumr]
               }  // End if pu[k]->lunumr

               ioblk[k][clu[k]->lunum] =  malloc(sizeof(struct IO3270));
               ioblk[k][clu[k]->lunum]->inpbufl = 0;                             /* make sure the initial length is 0 */
               clu[k]->daf_addr1[clu[k]->lunum] = 0;                             /* make sure the initial value is 0 */
               clu[k]->bindflag[clu[k]->lunum] = 0;                              /* make sure the initial value is 0 */
               clu[k]->initselfflag[clu[k]->lunum] = 0;                          /* make sure the initial value is 0 */
               clu[k]->not_ready[clu[k]->lunum] = 0;                             /* Reset not-ready state            */
               printf("\rCLU: terminal %d connected to 3271-%01X\n", clu[k]->lunum, k);
               //  Next available terminal
               clu[k]->lunum = clu[k]->lunum + 1;
               //  Find first available LU
               clu[k]->lunum = 0xFF;                                              /* preset to no LU's availble         */
               for (BYTE j = 0; j < MAXLU; j++) {
                  if (clu[k]->lu_fd[j] < 1) clu[k]->lunum = j;
               } // end for BYTE j
               if (clu[k]->lunum == 0xFF) printf("\rCLU: No more terminal ports available. New connections rejected until a terminal port is released;\n");
            }  // End if clu[j]->lu_fd
         }  // End if (pu2[k] != 0xFF)
      }  // End for int i
      for (BYTE j = 0; j < MAXLU; j++) {
         if (clu[k]->lu_fd[j] > 0) {
#if defined(__GENODE__)
            /* No ioctl on this plugin, so poll -- and poll ONLY. Peeking
               to tell "no data" from "peer gone" does not work here: this
               lwIP returns 0 from recv(MSG_PEEK) on a live idle socket, and
               0 from read() on it too, which is what a closed peer returns.
               The 3274 carried the same peek and dropped every terminal 16 ms
               after accepting it; see 0007's preamble for the measurements.
               A quiet read is quiet, and a close comes from POLLHUP/POLLERR
               and from the write path. */
            {
               struct pollfd pfd;

               pfd.fd = clu[k]->lu_fd[j]; pfd.events = POLLIN; pfd.revents = 0;
               rc = 0; pendingrcv = 0;
               if (poll(&pfd, 1, 0) > 0) {
                  if (pfd.revents & (POLLERR | POLLNVAL))      rc = -1;
                  else if (pfd.revents & POLLIN)               pendingrcv = 1;
                  else if (pfd.revents & POLLHUP)              rc = -1;
               }
            }
#else
            rc = ioctl(clu[k]->lu_fd[j], FIONREAD, &pendingrcv);
            /* Outside the #else this ran on Genode too and undid the poll
               above: SocketReadAct is a select() for readability, which a
               quiet socket fails, so every idle terminal was disconnected
               regardless of what poll had just decided. */
            if ((pendingrcv < 1) && (SocketReadAct(clu[k]->lu_fd[j]))) rc = -1;
#endif
            if (rc < 0) {
               clu[k]->not_ready[j] = 1;
               free(ioblk[k][j]);
               clu[k]->actlu[j] = 0;
               close (clu[k]->lu_fd[j]);
               clu[k]->lu_fd[j] = 0;
               printf("\rCLU: terminal %d disconnected from 3271-%01X\n", j, k);
               if ((clu[k]->lunum > j) || (clu[k]->lunum == 0xFF))  /* If next available terminal greater or no terminals availble ... */
                   clu[k]->lunum = j;                               /* ...replace with the just released terminal number          */
            } else {
               if (pendingrcv > 0) {
                  rc=read(clu[k]->lu_fd[j], bfr, 256-BUFPD);
#if defined(__GENODE__)
                  if (rc <= 0) continue;   /* readable but empty: see above */
#endif
                  //******
                  if (Tdbg_flag == ON) {
                     fprintf(T_trace, "%s 3270 Read Buffer: ",TimeStamp());
                     for (int i = 0; i < rc; i ++) {
                        fprintf(T_trace, "%02X ", bfr[i]);
                     }
                     fprintf(T_trace, "\n");
                  } // End if debug
                  //******
                  commadpt_read_tty(clu[k], ioblk[k][j], bfr, j, rc);
               }  // End if pendingrcv
            }  // End if rc < 0
         }  // End if bsc-lu_fd
      }  // End for int j
   }  // End for int k

    return 0;
}

void main(int argc, char *argv[]) {
   unsigned long inaddr;
   struct hostent *lineent;
   uint16_t BSCrlen;               /* Buffer size of received data      */
   uint16_t BSCtlen;               /* Buffer size of transmitted data   */
   int    pendingrcv;              /* pending data on the socket        */
   int linenum = 20;               /* BSC line number (default 20)      */
   int Fptr, frame_len;            /* BSC frame pointers and lenght     */
   int    i, rc;
   char ipv4addr[sizeof(struct in_addr)];

   /* Read command line arguments */
   if (argc == 1) {
      printf("\rCLU: Error - Arguments missing\n");
      printf("\r   Valid arguments are:\n");
      printf("\r    -cchn {hostname}    : hostname of host running the 3705\n");
      printf("\r    -ccip {ipaddress}   : ipaddress of host running the 3705 \n");
      printf("\r    -line {line number} : BSC line number to connect to\n");
      printf("\r    -d : switch debug on  \n");
      return;
   }
   Tdbg_flag = OFF;
   i = 1;
   while (i < argc) {
      if (strcmp(argv[i], "-d") == 0) {
         Tdbg_flag = ON;
         printf("\rCLU: Debug on. Trace file is trace_3271.log\n");
         i++;
         continue;
      } else if (strcmp(argv[i], "-cchn") == 0) {
         if ( (lineent = gethostbyname(argv[i+1]) ) == NULL ) {
            printf("\rCLU: Cannot resolve hostname %s\n", argv[i+1]);
            return;                /* error */
         }  // End if lineent
         printf("\rCLU: Connection to be established with 3705 BSC line at host %s\n", argv[i+1]);
         i = i + 2;
         continue;
      } else if (strcmp(argv[i], "-ccip") == 0) {
         inet_pton(AF_INET, argv[i+1], ipv4addr);
         if ( (lineent = gethostbyaddr(&ipv4addr,sizeof(ipv4addr),AF_INET) ) == NULL ) {
            printf("\rCLU: Cannot resolve ip address %s\n", argv[i+1]);
            return; /* error */
         }  // End if lineent
         printf("\rCLU: Connection to be established with 3705 BSC line at ip address %s\n", argv[i+1]);
         i = i + 2;
         continue;
      } else if (strcmp(argv[i], "-line") == 0) {
         sscanf(argv[i+1], "%d", &linenum);
         printf("\rCLU: Connection to be established with BSC line %d\n", linenum);
         i = i + 2;
         continue;
      } else {
         printf("\rCLU: invalid argument %s\n", argv[i]);
         printf("\r   Valid arguments are:\n");
         printf("\r    -cchn {hostname}    : hostname of host running the 3705\n");
         printf("\r    -ccip {ipaddress}   : ipaddress of host running the 3705 \n");
         printf("\r    -line {line number} : BSC line number to connect to\n");
         printf("\r    -d : switch debug on  \n");
         return;
      }  // End else
   }  // End while

   // ********************************************************************
   // Terminal controller debug trace facility
   // ********************************************************************
   if (Tdbg_flag == ON) {
      time_t tme = time(NULL);
      struct tm *currentTime = localtime(&tme);
      T_trace = fopen("trace_3271.log", "w");
#if defined(__GENODE__)
      /* /dev/log, not /dev/null: a component with nowhere to put a file
         still has somewhere to put a line. See the 3274's patch. */
      if (T_trace == NULL) {
         T_trace = fopen("/dev/log", "w");
         if (T_trace != NULL) setvbuf(T_trace, NULL, _IONBF, 0);
      }
#endif
      fprintf(T_trace, "     ****** 3271 Terminal Controller log file ****** \n"
                       "     ******          Date: %02d/%02d/%04d         ****** \n\n"
                       "     i327x_3271 -d : trace all 3271 activities\n",
                       currentTime->tm_mday, currentTime->tm_mon + 1, currentTime->tm_year + 1900);
   }
#if defined(__GENODE__)
   /* No socket, no connect and no retry loop: the line is a crosslink that
      was open before either program started. genode_line.h has the rest. */
   linenum = (int) genode_line_number();
   if (genode_line_open() != 0)
      return;
   clubsc_fd = genode_line_fd();
   printf("\rCLU: BSC line %d is a crosslink; no connection to establish\n",
          linenum);
#else
   // BSC line socket creation
   clubsc_fd = socket(AF_INET, SOCK_STREAM, 0);
   if (clubsc_fd <= 0) {
      printf("\rCLU: Cannot create line socket\n");
      return;
   }
   // Assign IP addr and PORT number
   servaddr.sin_family = AF_INET;
   memcpy(&servaddr.sin_addr, lineent->h_addr_list[0], lineent->h_length);
   servaddr.sin_port = htons(BSCLBASE+linenum);
   // Connect to the BSC line socket
   printf("\rCLU: Waiting for BSC line connection to be established\n");
   while (connect(clubsc_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) != 0) {
      sleep(1);
   }
#endif
   printf("\rCLU: BSC line connection has been established\n");
   // Now 'IML' the 3271
   rc = proc_CLUiml();
   while (1) {
      rc = proc_3270();
#if defined(__GENODE__)
      pendingrcv = xlink_avail(clubsc_fd, 10) ? (int) sizeof(BSC_rbuf) : 0;
      rc = 0;
#else
      rc = ioctl(clubsc_fd, FIONREAD, &pendingrcv);
#endif
#if !defined(__GENODE__)
      if ((pendingrcv < 1) && (SocketReadAct(clubsc_fd))) rc = -1;
      if (rc < 0) {                      // Retry once to account for timing delays in TCP.
         if ((pendingrcv < 1) && (SocketReadAct(clubsc_fd))) rc = -1;
      }
#else
      /* A crosslink does not disconnect, and SocketReadAct is a select() on
         an fd that is not a socket. Letting it run took the "line dropped"
         path below, which close()s the crosslink and rebuilds it as a TCP
         socket -- unrecoverable here. Same reasoning as the unreachable
         disconnect branch in i3705_lib.c's ReadLIB. */
#endif
      if (rc < 0) {
         printf("\rCLU: BSC line dropped, trying to re-establish connection\n");
         // BSC line socket recreation
         close(clubsc_fd);
         clubsc_fd = socket(AF_INET, SOCK_STREAM, 0);
         if (clubsc_fd <= 0) {
            printf("\rCLU: Cannot create line socket\n");
            return;
         }
         while (connect(clubsc_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) != 0) {
            sleep(1);
         } // End while
         printf("\rCLU: BSC line connection has been re-established\n");
      } else {
         if (pendingrcv > 0) {
            /* One read is one or more whole frames here: the sender
               writes a complete frame and the crosslink delivers it intact.
               An accumulate-until-PAD loop was added on the theory that this
               read was coming back short -- it was not, and the loop body
               never executed once. What looked like a 65-byte truncation was
               Genode's LOG cutting the hex dump at ~267 characters, which is
               exactly 65 triplets after the timestamp prefix. The frame
               scanner below is bounded regardless, so a genuinely short or
               malformed frame cannot walk it off the end. */
            BSCtlen = read(clubsc_fd, BSC_tbuf, sizeof(BSC_tbuf));
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "%s 3271 Receive Buffer: ",TimeStamp());
               for (int i = 0; i < BSCtlen; i ++) {
                  fprintf(T_trace, "%02X ", BSC_tbuf[i]);
               }
               fprintf(T_trace, "\n");
            } // End if debug
   //****************************************************************************************************************************
   // Separate consecutive BSC frames .
   //****************************************************************************************************************************
            Fptr = 0;
            if ((BSC_tbuf[Fptr] == 0x00) || (BSC_tbuf[Fptr] == 0xAA)) Fptr = 1; // If modem clocking is used skip first char
            do {
               frame_len = 0;          // initial frame size
               // Find end of BSC frame...
#if defined(__GENODE__)
               /* Bounded: the unguarded form walks off the end of the bytes
                  actually read when the buffer holds a partial frame, which
                  is how a 65-byte read got reported as a 1332-byte frame.
                  Upstream issue 19. Guarded so the native reference binary,
                  which this tree also builds and which is the measurement
                  baseline, stays byte-for-byte unchanged. */
               while (Fptr + frame_len < BSCtlen &&
                      !(BSC_tbuf[Fptr + frame_len + 0] == 0xFF)) {
#else
               while (!(BSC_tbuf[Fptr + frame_len + 0] == 0xFF)) {
#endif
                  // Check if we received two consecutive SYN characters in the text...
                  // ...these are time-fill syncs and must be removed
                  if ((frame_len > 3) && (BSC_tbuf[Fptr + frame_len] == SYN) && (BSC_tbuf[Fptr + frame_len + 1] == SYN))  {
                     if (Tdbg_flag == ON)
                        fprintf(T_trace,"%s Double SYNC detected\n",TimeStamp());
                    //******************************************************************************
                    //* Shift all characters right of the two SYNC characters to places the left.  *
                    //******************************************************************************
                    for (int i = Fptr + frame_len; i < BSCtlen - 2; i++) {
                       BSC_tbuf[i] = BSC_tbuf[i + 2];
                      } // End  for (int i = frame_len + Fptr
                      BSCtlen = BSCtlen - 2;   // Length reduces due to  removal of the two SYNC characters
                  }  // End if ((frame_len > 3)
                  frame_len++;
               } // End while
               frame_len = frame_len + 1;     // Add pad character
               if (Tdbg_flag == ON) {
                  fprintf(T_trace, "%s BSC Frame found (%d): ", TimeStamp(),frame_len);
                  for (int i=0; i < frame_len; i ++) {
                     fprintf(T_trace, "%02X ", BSC_tbuf[Fptr+i]);
                  }
                  fprintf(T_trace, "\n");
                  fflush(T_trace);
               } // End if debug
      //****************************************************************************************************************************
      // Process BSC frame
      //****************************************************************************************************************************
               BSCrlen = proc_BSC(&BSC_tbuf[Fptr], frame_len);      // precess transmission
               if (BSCrlen != 0) {                                  // If response available, send it
#if defined(__GENODE__)
                  /* Whole message, and a crosslink is NOT a socket: send()
                     on a VFS terminal fd fails, rc is never looked at here,
                     and the trace below prints the buffer regardless -- so a
                     response that never left the component logged exactly
                     like one that did. The 3705 then polled an empty line
                     until REPLYTO expired. The 3274 got this substitution in
                     0007 and the 3271 did not. See ../xlink/genode_xlink.h. */
                  rc = xlink_write(clubsc_fd, BSC_rbuf, BSCrlen);
#else
                  rc = send(clubsc_fd, BSC_rbuf, BSCrlen, 0);
#endif
                  if (rc < 0)
                     printf("\rCLU: BSC line write failed\n");
                  if (Tdbg_flag == ON) {
                     fprintf(T_trace, "%s 3271 Send Buffer: ",TimeStamp());
                     for (int i = 0; i < BSCrlen; i ++) {
                        fprintf(T_trace, "%02X ", BSC_rbuf[i]);
                     }
                     fprintf(T_trace, "\n");
                     fflush(T_trace);
                  } // End if debug
               } // End  if (BSCrlen != 0)
               Fptr = Fptr + frame_len;
            } while (Fptr < BSCtlen); // end do
         }  // End if (pendingrcv > 0)
      }  // End if (rc < 0)
   }  // End while (1)
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
