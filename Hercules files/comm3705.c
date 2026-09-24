/*******************************************************************/
/* COMM3705.C (c) Copyright Edwin Freekenhorst and Henk Stegeman   */
/*                                                                 */
/* Hercules channel extender for the                               */
/*      3705 communications controller simulator                   */
/*                                                                 */
/* This version used the  COMM3705.C coding from Max H. Parke as   */
/* the starting point                                              */
/*                                                                 */
/* The purpose of this module is to extend the 37xx Channel Adapter*/
/* module of the 37xx simulator, into hercules.                    */
/* The core function is to maintain a dual TCP/IP connection with  */
/* the 37xx simulator, emulating a bus and tag channel connection. */
/*                                                                 */
/*  This coding is such, that it is (should be) independent from   */
/* the Hercules version (as well as Linux as Windows).             */
/*******************************************************************/
// Issue a warning if version hyperion 4.x is assumed.
#ifdef VERS_MAJ
 #ifdef WIN32
  #pragma message("Version major defined, assuming version 4")
 #else
  #warning "Version major defined, assuming version 4"
 #endif
#endif

#include "hstdinc.h"
#include "hercules.h"
#include "devtype.h"
#include "opcode.h"
#include "parser.h"


//#if !defined(WIN32)
//   #include "stdbool.h"
//#endif

#if !defined(true)
#define true 1
#endif

#if !defined(false)
#define false 0
#endif

#if defined(WIN32) && defined(OPTION_DYNAMIC_LOAD) && !defined(HDL_USE_LIBTOOL) && !defined(_MSVC_)
  SYSBLK *psysblk;
  #define sysblk (*psysblk)
#endif

#if !defined(min)
#define  min(a,b) (((a) <= (b)) ? (a) : (b))
#endif

#define BUFPD 0x1C

static BYTE commadpt_immed_command[256]=
{ 0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/*-----------------------------------------------------------------*/
/* PARSER TABLES                                                   */
/*-----------------------------------------------------------------*/
#if defined(__GENODE__)
/*-------------------------------------------------------------------*/
/* The channel adapter over Genode IPC rather than TCP                */
/*-------------------------------------------------------------------*/
/*                                                                   */
/* The 3705 end of this is i3705_chan_T2.c, patched the same way; the */
/* design and the reasoning are in port/COMM3705.md. Here the two     */
/* TCP connections a channel is made of become two terminal_crosslink */
/* sessions, and the device statement loses its address and its port  */
/* without gaining anything:                                         */
/*                                                                   */
/*   0660 3705 adaptip=127.0.0.1 port=37051      native, TCP          */
/*   0660 3705                                   Genode, IPC          */
/*                                                                   */
/* The two nodes are /dev/ca<devnum>_bus and /dev/ca<devnum>_tag, by  */
/* convention, exactly as genode_shrd.c opens /dev/arb<devnum>. This  */
/* file took two PATH arguments first, and that was wrong three times */
/* over: two arguments for a thing that is always one pair; a path in */
/* hercules.cnf for something the guest's configuration does not      */
/* decide and nothing checks; and a pairing stated rather than        */
/* structural, so crossing bus with another channel's tag, or naming  */
/* one and not the other, were both spellable.                       */
/*                                                                   */
/* Which crosslink this partition actually reaches is routing, and    */
/* Genode already says routing with a session label matched against a */
/* policy -- see "The path that went away" in port/SHARED-DASD.md,    */
/* which settled the same question for the arbitrated volume.        */
/*                                                                   */
/* The name is per-device because the SESSION is per-device, and it   */
/* is local to this partition: the 3705 calls the same crosslink CA1  */
/* channel A, because that is what it is at that end. Two names for   */
/* the two ends of one pipe is the shape shared DASD already has,     */
/* where one lock is arb0120 to one partition and arb0220 to another. */
/*                                                                   */
/* adaptip= and port= are still parsed, and are simply not used -- a  */
/* configuration carried over from a native run says what it meant    */
/* and is told what is wrong, rather than failing on an unrecognised  */
/* keyword.                                                          */
/*                                                                   */
/* The two-byte device address this end sends on the bus after        */
/* connecting is kept exactly: it is what makes the 3705 print        */
/* "CA1: Connected to device 0660", and a crosslink buffers it, so    */
/* the emulator no longer has to be started second.                   */
/*                                                                   */
/* send() and recv() are redirected rather than rewritten at each     */
/* call, for one reason that matters: terminal_crosslink's write      */
/* RETURNS A SHORT COUNT when its ring is full and never blocks, so a */
/* plain write drops the tail of a channel message with no error      */
/* anywhere. ipc_write loops. ipc_read returns what is there, which   */
/* is the semantic every recv() in this file already had.             */
/*-------------------------------------------------------------------*/

#include <poll.h>

static int ipc_write(int fd, const void *buf, int len)
{
    const char *p = (const char*) buf;
    int sent = 0;

    if (fd < 0) return -1;

    while (sent < len)
    {
        int const rc = (int) write( fd, p + sent, (size_t)(len - sent) );
        if (rc < 0) { if (errno == EINTR) continue; return -1; }
        if (rc == 0) return -1;
        sent += rc;
    }
    return sent;
}

static int ipc_read(int fd, void *buf, int len)
{
    int rc;

    if (fd < 0) return -1;

    do { rc = (int) read( fd, buf, (size_t)len ); }
    while (rc < 0 && errno == EINTR);

    return rc;
}

/*
 * Is there anything to read, without blocking to find out.
 *
 * The attention thread waits on the tag for the life of the device, and on
 * Genode a thread parked in a blocking VFS read is not free: the libc has one
 * context to service everybody, and the emulator's 3270 console devices reach
 * their clients through lwIP on that same libc. DASD does not -- it is a
 * Block session -- so an MVS IPL completes normally and only the console path
 * starves, which is why this surfaced as
 *
 *     IST093I  AJRP     ACTIVE
 *
 * and then silence: VTAM activating L3274, the local non-SNA major node for
 * the 3270s at 00C0-00C6, with nothing able to answer it.
 */
static int ipc_avail(int fd)
{
    struct pollfd p;

    if (fd < 0) return 0;

    p.fd = fd; p.events = POLLIN; p.revents = 0;

    if (poll( &p, 1, 0 ) <= 0) return 0;

    return (p.revents & POLLIN) ? 1 : 0;
}

static int ipc_read_exact(int fd, void *buf, int len)
{
    char *p = (char*) buf;
    int got = 0;

    if (fd < 0) return -1;

    while (got < len)
    {
        int const rc = (int) read( fd, p + got, (size_t)(len - got) );
        if (rc < 0) { if (errno == EINTR) continue; return -1; }
        if (rc == 0) return -1;
        got += rc;
    }
    return got;
}

#define send(fd, buf, len, flags)  ipc_write((fd), (buf), (int)(len))
#define recv(fd, buf, len, flags)  ipc_read ((fd), (buf), (int)(len))
#endif

static PARSER ptab[] = {
    {"port",    "%s"},
    {"adaptip", "%s"},
    {"debug",   "%s"},
    {"tracesna", "%s"},
    {NULL, NULL}
};


enum {
  COMMADPT_KW_PORT = 1,
    COMMADPT_KW_ADAPTIP,
    COMMADPT_KW_DEBUG,
    COMMADPT_KW_TRACESNA
} comm3705_kw;

struct COMMADPT
{
    DEVBLK *dev;                /* the devblk to which this CA is attched   */
    TID  tthread;               /* ATTN thread                              */
    BYTE *unitstat;             /* pointer to channel unitstat              */
    in_addr_t adaptip;          /* 3705 CA listening address                */
    U16   port;                 /* 3705 CA listening port                   */
    BYTE carnstat;              /* Channel Adaptor return status            */
    BYTE carnstat2;             /* Channel Adaptor return status (tag)      */
    BYTE ccwactive;             /* indicate an active CCW                   */
    BYTE cswpending;            /* indicate a CSW is pending                */
    int busfd;                  /* Communication socket for ccw/data        */
    int tagfd;                  /* Communication socket for attn            */
#if defined(__GENODE__)
    char buspath[32];           /* ...a crosslink node here, not a socket   */
    char tagpath[32];
#endif
    COND ipc;                   /* I/O <-> thread IPC condition EVB         */
    COND ipc_halt;              /* I/O <-> thread IPC HALT special EVB      */
    LOCK lock;                  /* COMMADPT lock                            */
    int pipe[2];                /* pipe used for I/O to thread signaling    */
    U16  devnum;                /* devnum copy from DEVBLK                  */
    struct sockaddr_in servaddr;

    U32 have_cthread:1;         /* the comm thread is running               */
    U32 attn_run;               /* the ATTN thread is running               */
    U32 attn_halt;              /* signal halt to the ATTN thread           */
    U32 debug;                  /* 1 = write general  debug messages        */
    U32 tracesna;               /* 1 = write sna request codes/Commands     */

    int read_ccw_count;
    int write_ccw_count;
    int unack_attn_count;

    BYTE inpbuf[65536];
    int inpbufl;

    void * freeq;
    void * sendq;
    BYTE * poolarea;

};

struct CSW {
   uint8_t  key;
   uint8_t  data_address[3];
   uint8_t  unit_conditions;
   uint8_t  channel_conditions;
   uint16_t count;
} csw;

/*-------------------------------------------------------------------*/
/* Internal macro definitions                                        */
/*-------------------------------------------------------------------*/

/* DEBUG_LVL: 0 = none
              1 = status
              2 = headers
              3 = buffers
*/
#define DEBUG_LVL        0

#if DEBUG_LVL == 0
  #define TNSDEBUG1      1 ? ((void)0) : logmsg
  #define TNSDEBUG2      1 ? ((void)0) : logmsg
  #define TNSDEBUG3      1 ? ((void)0) : logmsg
#endif
#if DEBUG_LVL == 1
  #define TNSDEBUG1      logmsg
  #define TNSDEBUG2      1 ? ((void)0) : logmsg
  #define TNSDEBUG3      1 ? ((void)0) : logmsg
#endif
#if DEBUG_LVL == 2
  #define TNSDEBUG1      logmsg
  #define TNSDEBUG2      logmsg
  #define TNSDEBUG3      1 ? ((void)0) : logmsg
#endif
#if DEBUG_LVL == 3
  #define TNSDEBUG1      logmsg
  #define TNSDEBUG2      logmsg
  #define TNSDEBUG3      logmsg
#endif

#define TNSERROR        logmsg

#define BUFLEN_3270     65536           /* 3270 Send/Receive buffer  */
#define BUFLEN_1052     150             /* 1052 Send/Receive buffer  */

#undef  FIX_QWS_BUG_FOR_MCS_CONSOLES

// ************************************************************
// This subroutine waits 1 usec
// ************************************************************
void wait1us() {
   /* NOT ONE MICROSECOND ON GENODE, and nothing here can make it one:
      nanosleep floors every non-zero request at 1 ms
      (repos/libports/src/lib/libc/sleep.cc). So the caller below --
      while (dev->busy || IOPENDING(dev)) wait1us(); -- polls at 1 kHz
      rather than 1 MHz and counts its "delay" in milliseconds. That is the
      right direction for a cooperative libc, where a microsecond spin would
      be a thousand monitor round trips per millisecond, but the name lies
      and the arithmetic downstream of it is out by 1000. Comment only; the
      code is upstream's. */
   usleep(1);
   return;
}

static int write_adpt(BYTE* bufferp, int len, COMMADPT* ca);

// ************************************************************
// Function to check if socket is (still) connected
// ************************************************************
static int IsSocketConnected(int sockfd, U16 ssid, U16 devnum)
{
#if defined(__GENODE__)
   /* A crosslink is connected for as long as it is open. The 3705 end of
      this says the same thing for the same reason; see COMM3705.md on what
      a restart mid-session would need and does not have. */
   (void)ssid; (void)devnum;
   return (sockfd > 0) ? 1 : 0;
#else
   int rc;
   struct sockaddr_in ccu_addr;
   socklen_t addrlen = sizeof(ccu_addr);
   if (sockfd < 1) {
      logmsg("CCBUS001E %1d:%04X: socket not connected\n", ssid, devnum);
      return false;
   }
   rc = getpeername(sockfd, (struct sockaddr*)&ccu_addr, &addrlen);
   if (rc != 0) {
      logmsg("CCBUS001E %1d:%04X: socket: %d rc: %d error: %s\n", ssid, devnum, sockfd, rc, strerror(errno));
      return false;
   }
   return true;
#endif
}

// ********************************************************************
// Function to enable TCP socket and connect to Remote channel adapter
// ********************************************************************
static int connect_adpt(COMMADPT *ca) {
#if defined(__GENODE__)
   char cua[2];
   int  rc;

   MSGBUF( ca->buspath, "/dev/ca%04X_bus", ca->dev->devnum );
   MSGBUF( ca->tagpath, "/dev/ca%04X_tag", ca->dev->devnum );

   /* Bus first, then tag. The ORDER is the protocol: the 3705 took the
      first of two connections on one port as the bus and the second as the
      tag, and although a crosslink pair has no accept to order, keeping the
      opens in that sequence keeps the two ends describing the same thing. */
   ca->busfd = open( ca->buspath, O_RDWR );
   if (ca->busfd < 0) {
      logmsg("CCBUS002E %1d:%04X: cannot open %s: %s\n",
             ca->dev->ssid, ca->dev->devnum, ca->buspath, strerror(errno));
      return(-1);
   }
   ca->tagfd = open( ca->tagpath, O_RDWR );
   if (ca->tagfd < 0) {
      logmsg("CCTAG002E %1d:%04X: cannot open %s: %s\n",
             ca->dev->ssid, ca->dev->devnum, ca->tagpath, strerror(errno));
      close( ca->busfd ); ca->busfd = -1;
      return(-1);
   }
   logmsg("CCTAG003I %1d:%04X: tag connection established on %s\n",
          ca->dev->ssid, ca->dev->devnum, ca->tagpath);

   cua[0] = (ca->devnum & 0x0000FF00) >> 8;
   cua[1] = (ca->devnum & 0x000000FF);

   rc = ipc_write( ca->busfd, cua, 2 );
   if (rc != 2) {
      logmsg("CCBUS004E %1d:%04X: device address not delivered\n",
             ca->dev->ssid, ca->dev->devnum);
      return(-1);
   }
   logmsg("CCBUS003I %1d:%04X: bus connection established on %s\n",
          ca->dev->ssid, ca->dev->devnum, ca->buspath);
   return(0);
#else

  int rc;
  char cua[2];
 #ifdef WIN32
     unsigned long bmode = 0;
  #endif
  // Bus socket creation
  ca->busfd = socket(AF_INET, SOCK_STREAM, 0);
   if (ca->busfd <= 0 ) {
     logmsg("CCBUS002E %1d:%04X: bus socket creation failed\n", ca->dev->ssid, ca->dev->devnum);
     return(-1);
  }

  // ATTN (Tag) socket creation
  ca->tagfd = socket(AF_INET, SOCK_STREAM, 0);
   if (ca->tagfd <= 0) {
     logmsg("CCTAG002E %1d:%04X: tag socket creation failed\n", ca->dev->ssid, ca->dev->devnum);
     return(-1);
  }

#ifdef WIN32
   rc = ioctlsocket(ca->busfd, FIONBIO, &bmode);
   if (rc != 0) {
      logmsg("CCBUS002E %1d:%04X: bus socket option failed\n", ca->dev->ssid, ca->dev->devnum);
      return(-1);
   }
   rc = ioctlsocket(ca->tagfd, FIONBIO, &bmode);
   if (rc != 0) {
      logmsg("CCTAG002E %1d:%04X: tag socket option failed\n", ca->dev->ssid, ca->dev->devnum);
      return(-1);
   }
#endif

   // Remove if below works on Linux: bzero(&ca->servaddr, sizeof(ca->servaddr));
   memset(&ca->servaddr, 0x00, sizeof(ca->servaddr));

   // Assign IP addr and PORT number
   ca->servaddr.sin_family = AF_INET;
   ca->servaddr.sin_addr.s_addr = ca->adaptip;
   ca->servaddr.sin_port = htons(ca->port);


   // Connect to the bus socket
   logmsg("CCBUS019I %1d:%04X: Waiting for bus(%d) connection to be established\n",  ca->dev->ssid, ca->dev->devnum, ca->busfd);
   while (connect(ca->busfd, (struct sockaddr*)&ca->servaddr, sizeof(ca->servaddr)) != 0) {
      if (ca->attn_halt)
         return(-1);
      sleep(1);
   }

   // Connect to the tag (ATTN) socket
    logmsg("CCTAG019I %1d:%04X: Waiting for tag(%d) connection to be established\n",  ca->dev->ssid, ca->dev->devnum, ca->tagfd);
   while (connect(ca->tagfd, (struct sockaddr*)&ca->servaddr, sizeof(ca->servaddr)) != 0) {
      if (ca->attn_halt)
         return(-1);
      sleep(1);
   }
   logmsg("CCTAG003I %1d:%04X: tag connection established on socket %d\n", ca->dev->ssid, ca->dev->devnum, ca->tagfd);

   cua[0] = (ca->devnum & 0x0000FF00) >> 8;
   cua[1] = (ca->devnum & 0x000000FF);
   if (ca->busfd  > 0) {
      rc = send (ca->busfd, cua, 2, 0);
      if (rc == 2)
          logmsg("CCBUS003I %1d:%04X: bus connection established on socket %d\n",  ca->dev->ssid, ca->dev->devnum, ca->busfd);
   } else {
      logmsg("CCBUS004E %1d:%04X: connect_adpt() %s\n",  ca->dev->ssid, ca->dev->devnum, strerror(HSO_errno));
      return(-1);
   }

   return(0);
#endif
}
// ********************************************************************
// Function to close the bus and tag TCP sockets
// ********************************************************************
static void close_adpt(COMMADPT *ca) {

   // Bus socket close
   if (ca->busfd > 0) {
#if defined(__GENODE__)
      close(ca->busfd);
#else
      shutdown(ca->busfd, SHUT_RDWR);
#endif
      ca->busfd = -1;
      logmsg("CCBUS0016I %1d:%04X: bus connection closed\n",  ca->dev->ssid, ca->dev->devnum);
   }
   // Tag socket close
   if (ca->tagfd > 0) {
#if defined(__GENODE__)
      close(ca->tagfd);
#else
      shutdown(ca->tagfd, SHUT_RDWR);
#endif
      ca->tagfd = -1;
      logmsg("CCTAG0016I %1d:%04X: tag connection closed\n",  ca->dev->ssid, ca->dev->devnum);
   }
   return;
}

/*-------------------------------------------------------------------*/
/* Subroutine to send buffer to remote channel adapter               */
/*-------------------------------------------------------------------*/
static int
write_adpt(BYTE *bufferp, int len, COMMADPT *ca) {
   int rc;                              /* Return code               */

   if (ca->busfd > 0) {
      rc = send (ca->busfd, bufferp, len, 0);
      if (ca->debug)
         logmsg("CCBUS004I %1d:%04X: write adpt(%d) completed. Bytes sent: %d \n", ca->dev->ssid, ca->dev->devnum, ca->busfd, rc);
  }
   else
      rc = -1;
   if (rc < 0) {
      logmsg("CCBUS004E %1d:%04X: write_adpt(%d) %s\n",  ca->dev->ssid, ca->dev->devnum, ca->busfd, strerror(HSO_errno));
      return -1;
   }
   return 0;
} /* End function write_adpt */

char EBCDIC2ASCII (char s) {
   static char etoa[] =
        "................................"
        "................................"
        " ...........<(+|&.........!$*); "  // first char here is real space !
        "-/.........,%_>?.........`:#@'=\""
        " abcdefghi.......jklmnopqr......"
        "  stuvwxyz......................"
        " ABCDEFGHI.......JKLMNOPQR......"
        "  STUVWXYZ......0123456789......";
   s = etoa[(unsigned char)s];
   return s;
}

static void logdump(char *txt, DEVBLK *dev, BYTE *bfr, size_t sz) {
   size_t i;
   if (!dev->ccwtrace) {
      return;
   }
   logmsg("CCBUS010D %1d:%04X:%s\n", dev->ssid, dev->devnum, txt);
   logmsg("CCBUS011D %1d:%04X:%s : Dump of %ld (%ld) byte(s)\n", dev->ssid, dev->devnum, txt, (long int)sz, (long int)sz);

   for (i = 0; i < sz; i++) {
      if (i%16 == 0) {
         if (i != 0) {
            logmsg("\n");
         }
         logmsg("CCBUS012D %1d:%04X:%s : %ld:", dev->ssid, dev->devnum, txt, (long int)i);
      }
      if (i%4 == 0) {
         logmsg(" ");
      }
      logmsg("%2.2X", bfr[i]);
   }
   logmsg("\nCCBUS013D ");
   for (i = 0; i <sz; i++) {
      if (i%16 == 0) {
         if (i != 0) {
            logmsg("\nCCBUS013D ");
         }
      }
      logmsg ("%c", EBCDIC2ASCII(bfr[i]));
   }
   logmsg("\n");
}

//***************************************************************************************
//* Function to "decode" the SNA traffic (tracesna=yes in hercules config file).
//* It is not entirely watertight, but sufficent for debugging purposes.
//***************************************************************************************
static void tracesna(DEVBLK *dev, BYTE *bfr, BYTE code) {
   int ru_len, fid4;
   uint8_t ru_cat;                                 // fmd = b00, nc = b01, dfc = b10, sc = b11
   char *ru_type = "";
   char *ru_code = "";

   if (!dev->commadpt->tracesna) {
      return;
   }
   if (bfr[0] == 0x41) return;                     // Skip if FID4 with priority network flow (FID1 will be empty)
   fid4 = 0;                                       // If no preceeding FID4, the offset to FID1 is 0 (earlier VTAM releases)
   ru_type = "Data";
   ru_code = "N/A";
   if (bfr[0] & 0x40) fid4 = 16;                   // If preceeded with a FID 4, then the FID1 is offset with 16 bytes
   else if (code == 0x02)  fid4 = 28;              // (MVS3.8 - VTAM L2) A read is preceeded with a FID1.
      if ((bfr[fid4+0] & 0x0C) == 0x0C) {          // If Only Segment
         ru_len = (bfr[fid4+8] << 8) + bfr[fid4+9];
         if ((ru_len > 3) && ((bfr[fid4+10] & 0x03) == 0x03)) {   // RU length must be > 3 and single chain only
            ru_cat = (bfr[fid4+10] & 0x60) >> 5;
            ru_code = "UNKNOWN";
            switch (ru_cat) {
               case 0x00: // FM Data
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x01", 3)) ru_code = "CONTACT";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x02", 3)) ru_code = "DISCONTACT";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x03", 3)) ru_code = "IPLINIT";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x04", 3)) ru_code = "IPLTEXT";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x05", 3)) ru_code = "IPLFINAL";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x06", 3)) ru_code = "DUMPOINIT";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x07", 3)) ru_code = "DUMPTEXT";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x08", 3)) ru_code = "DUMPFINAL";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x09", 3)) ru_code = "RPO";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x0A", 3)) ru_code = "ACTLINK";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x0B", 3)) ru_code = "DACTLINK";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x0E", 3)) ru_code = "CONNOUT";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x0F", 3)) ru_code = "ABCONN";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x11", 3)) ru_code = "SETCV NS(c)";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x14", 3)) ru_code = "ESLOW";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x15", 3)) ru_code = "EXSLOW";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x16", 3)) ru_code = "ACTCONNIN";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x17", 3)) ru_code = "DACTCONNIN";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x18", 3)) ru_code = "ABCONNOUT";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x19", 3)) ru_code = "ANA";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x1A", 3)) ru_code = "FNA";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x1B", 3)) ru_code = "REQDISCONT";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x80", 3)) ru_code = "CONTACTED";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x81", 3)) ru_code = "INOP";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x84", 3)) ru_code = "REQCONT";
                  if (!memcmp(&bfr[fid4+13], "\x01\x02\x85", 3)) ru_code = "NS-LSA";
                  if (!memcmp(&bfr[fid4+13], "\x01\x03\x01", 3)) ru_code = "EXECTEST";
                  if (!memcmp(&bfr[fid4+13], "\x01\x03\x02", 3)) ru_code = "ACTTRACE";
                  if (!memcmp(&bfr[fid4+13], "\x01\x03\x03", 3)) ru_code = "DACTTRACE";
                  if (!memcmp(&bfr[fid4+13], "\x01\x03\x11", 3)) ru_code = "SETCV NS(ma)";
                  if (!memcmp(&bfr[fid4+13], "\x41\x02\x10", 3)) ru_code = "RNAA";
                  if (!memcmp(&bfr[fid4+13], "\x81\x06\x88", 3)) ru_code = "SESSEND";
               break;
               case 0x01: // Network Control
                  if (bfr[fid4+13] == 0x02) ru_code = "NC-IPL-FINAL";
                  if (bfr[fid4+13] == 0x03) ru_code = "NC-IPL-INIT";
                  if (bfr[fid4+13] == 0x04) ru_code = "NC-IPL-TEXT";
                  if (bfr[fid4+13] == 0x05) ru_code = "LSA";
                  if (bfr[fid4+13] == 0x06) ru_code = "NC-ER-INOP";
                  if (bfr[fid4+13] == 0x09) ru_code = "NC-ER-TEST";
                  if (bfr[fid4+13] == 0x0A) ru_code = "NC-ER-TEST-REPLY";
                  if (bfr[fid4+13] == 0x0B) ru_code = "NC-ER-ACT";
                  if (bfr[fid4+13] == 0x0C) ru_code = "NC-ER-ACT-REPLY";
                  if (bfr[fid4+13] == 0x0D) ru_code = "NC-ACTVR";
                  if (bfr[fid4+13] == 0x0E) ru_code = "NC-DACTVR";
                  if (bfr[fid4+13] == 0x0F) ru_code = "NC-ER-OP";
                  if (bfr[fid4+13] == 0x46) ru_code = "NC-IPL-ABORT";
               break;
               case 0x02: // Data Flow Control
                  if (bfr[fid4+13] == 0x04) ru_code = "LUSTAT";
                  if (bfr[fid4+13] == 0x05) ru_code = "RTR";
                  if (bfr[fid4+13] == 0x07) ru_code = "ANSC";
                  if (bfr[fid4+13] == 0x70) ru_code = "BIS";
                  if (bfr[fid4+13] == 0x71) ru_code = "SBI";
                  if (bfr[fid4+13] == 0x80) ru_code = "QEC";
                  if (bfr[fid4+13] == 0x81) ru_code = "QC";
                  if (bfr[fid4+13] == 0x82) ru_code = "RELQ";
                  if (bfr[fid4+13] == 0x83) ru_code = "CANCEL";
                  if (bfr[fid4+13] == 0xC0) ru_code = "SHUTD";
                  if (bfr[fid4+13] == 0xC1) ru_code = "SHUTC";
                  if (bfr[fid4+13] == 0xC2) ru_code = "RSHUTD";
                  if (bfr[fid4+13] == 0xC8) ru_code = "BID";
                  if (bfr[fid4+13] == 0xC9) ru_code = "SIGNAL";
               break;
               case 0x03: // Session Control
                  if (bfr[fid4+13] == 0x0D) ru_code = "ACTLU";
                  if (bfr[fid4+13] == 0x0E) ru_code = "DACTLU";
                  if (bfr[fid4+13] == 0x11) ru_code = "ACTPU";
                  if (bfr[fid4+13] == 0x12) ru_code = "DACTPU";
                  if (bfr[fid4+13] == 0x14) ru_code = "ACTCDRM";
                  if (bfr[fid4+13] == 0x15) ru_code = "DACTCDRM";
                  if (bfr[fid4+13] == 0x31) ru_code = "BIND";
                  if (bfr[fid4+13] == 0x32) ru_code = "UNBIND";
                  if (bfr[fid4+13] == 0xA0) ru_code = "SDT";
                  if (bfr[fid4+13] == 0xA1) ru_code = "CLEAR";
                  if (bfr[fid4+13] == 0xC0) ru_code = "CRV";
               break;
            } // End switch ru_cat
            if (bfr[fid4+10] & 0x04) ru_code = "Sense Data";
            if (bfr[fid4+10] & 0x80)
              ru_type = "Response ";
            else ru_type = "Request";
         } //End if ru_len
      }   // End if Only_segment
      logmsg("CCBUS009T %1d:%04X: RU type=%s, RU code=%s\n",  dev->ssid, dev->devnum, ru_type,  ru_code);
}

static void put_bufpool(void ** anchor, BYTE * ele) {
   void ** elep = anchor;
   for (;;) {
      if (!*elep) break;
         elep = *elep;
   }
   *elep = ele;
   *(void**)ele = 0;
}


static void init_bufpool(COMMADPT *ca) {
   BYTE * areap;
   int i1;
   int numbufs = 64;
   int bufsize = 256 + 16 + 4;
   ca->poolarea = (BYTE*)calloc (numbufs, bufsize);
   if (!ca->poolarea) {
      return;
   }
   areap = ca->poolarea;
   for (i1 = 0; i1 < numbufs; i1++) {
      put_bufpool(&ca->freeq, areap);
      areap += (bufsize);
   }
}

static void free_bufpool(COMMADPT *ca) {
   ca->sendq = 0;
   ca->freeq = 0;
   if (ca->poolarea) {
      free(ca->poolarea);
      ca->poolarea = 0;
   }
}

/*-------------------------------------------------------------------*/
/* Free all private structures and buffers                           */
/*-------------------------------------------------------------------*/
static void commadpt_clean_device(DEVBLK *dev)
{
   if (dev->commadpt != NULL) {
      dev->commadpt->attn_halt =  1;
      close_adpt(dev->commadpt);
      while (dev->commadpt->attn_run == 1)
         usleep(20);

      free(dev->commadpt);
      dev->commadpt = NULL;
      logmsg("CCCOM013I %1d:%04X: clean : Control block freed\n", dev->ssid, dev->devnum);
   }
   else
   {
      logmsg("CCCOM014I %1d:%04X: clean : Control block not freed : not allocated\n", dev->ssid, dev->devnum);
   }
   return;
}

/*-------------------------------------------------------------------*/
/* Allocate initial private structures                               */
/*-------------------------------------------------------------------*/
static int commadpt_alloc_device(DEVBLK *dev) {
   dev->commadpt = malloc(sizeof(COMMADPT));
   if (dev->commadpt == NULL) {
      logmsg("CCCOM020E %1d:%04X:Memory allocation failure for main control block\n", dev->ssid, dev->devnum);
      return -1;
   }
   memset(dev->commadpt, 0, sizeof(COMMADPT));
   dev->commadpt->dev=dev;
   return 0;
}

/*-------------------------------------------------------------------*/
/* commadpt_getport : returns a port number or -1                    */
/*-------------------------------------------------------------------*/
static int commadpt_getport(char *txt) {
   int pno;
   struct servent *se;

   pno = atoi(txt);
   if (pno == 0) {
      se = getservbyname(txt, "tcp");
         if (se == NULL) {
            return -1;
         }
      pno = se->s_port;
   }
   return(pno);
}
/*-------------------------------------------------------------------*/
/* commadpt_getaddr : set an in_addr_t if ok, else return -1         */
/*-------------------------------------------------------------------*/
static int commadpt_getaddr(in_addr_t *ia, char *txt) {
   struct hostent *he;
   he = gethostbyname(txt);
   if (he == NULL) {
      return(-1);
   }
   memcpy(ia, he->h_addr_list[0], 4);
   return(0);
}


/*-------------------------------------------------------------------*/
/* CSW_adpt: Monitors sockets and handles Channel Status Words       */
/*            from the remote channel adapter                        */
/*-------------------------------------------------------------------*/
static void *CSW_adpt(void *vca) {
   COMMADPT *ca;

   int rc, rc_attn;           /* return code from various rtns       */
   int delay;

   ca = (COMMADPT*)vca;
   ca->dev->commadpt->attn_run = 1;

   while ((!IsSocketConnected(ca->busfd, ca->dev->ssid, ca->dev->devnum)) && !ca->dev->commadpt->attn_halt) {
      if (ca->dev->commadpt->debug)
         logmsg("CCTAG002D %1d:%04X: Preparing connection with remote channel adapter\n", ca->dev->ssid, ca->dev->devnum);

      rc = connect_adpt(ca);

      if ((rc == 0) && !ca->dev->commadpt->attn_halt) {
         if (ca->dev->commadpt->debug)
            logmsg("CCTAG008D %1d:%04X:connections on port %d; Bus socket: %d, Tag socket: %d\n", ca->dev->ssid, ca->dev->devnum, ca->port, ca->busfd, ca->tagfd);
      } else {
      if (!ca->dev->commadpt->attn_halt)
         *ca->unitstat |= CSW_UC;    // Signal unit check
      }

      while ((IsSocketConnected(ca->tagfd, ca->dev->ssid, ca->dev->devnum)) && !ca->dev->commadpt->attn_halt) {
#if defined(__GENODE__)
         /* Poll, then read. See ipc_avail: blocking here holds the libc
            context that the emulator's 3270 consoles need, and VTAM then
            never gets past activating them. 10 ms is far below any channel
            timing and costs nothing measurable against an idle adapter. */
         if (!ipc_avail(ca->tagfd)) {
            usleep(10000);
            continue;
         }
#endif
         // Wait for Channel Adapter status byte
         rc = recv(ca->tagfd, &ca->carnstat, 1, 0);

         if (rc > 0) {
            if (ca->dev->commadpt->debug)
               logmsg("CCTAG002D %1d:%04X: ATTN received %02X\n", ca->dev->ssid, ca->dev->devnum, ca->carnstat);

             if (ca->dev->commadpt->debug)
                logmsg("CCTAG011D %1d:%04X: Device busy: %02X CCW active %02X \n", ca->dev->ssid, ca->dev->devnum, ca->dev->busy, ca->ccwactive);
             /************************************************************************************************************/
             /* The code below injects the CSW into the channel subsys.                                            */
             /************************************************************************************************************/

            delay = 0;

             while (ca->dev->busy || IOPENDING(ca->dev)) {
                 wait1us();
                 delay++;
              }
                   obtain_lock(&ca->dev->commadpt->lock);
                   rc_attn = device_attention(ca->dev, CSW_ATTN);  // Inject Attention
                   if (ca->dev->commadpt->debug)
                      logmsg("CCTAG009D %1d:%04X: Attention raised after %d us delay, rc=%d\n", ca->dev->ssid, ca->dev->devnum, delay, rc_attn);
                   release_lock(&ca->dev->commadpt->lock);
                   if (ca->dev->commadpt->debug)
                      logmsg("CCTAG002D %1d:%04X: Release Lock OK\n", ca->dev->ssid, ca->dev->devnum);
             // } // End if ca->carnstat



         } else { // If Error during receive of CA status, close both sockets and try to re-open
            if (ca->dev->commadpt->debug)
               logmsg("CCTAG002D %1d:%04X: Closing sockets due to error\n", ca->dev->ssid, ca->dev->devnum);
            close_adpt(ca);
            if (!ca->dev->commadpt->attn_halt) {
               ca->dev->scsw.unitstat |= CSW_UC;       // Signal unit check
               sleep(1);      /* Wait 1 secs, then try to re-connect */
            }
         } // End if rc > 0

         if (ca->dev->commadpt->attn_halt == 1) {
            logmsg("CCTAG015I %1d:%04X: Closing sockets due to devinit or shutdown\n", ca->dev->ssid, ca->dev->devnum);
            close_adpt(ca);
         }
      }  // End while ca->tagfd
   }  // End while ca->busfd

   ca->dev->commadpt->attn_run = 0;
   return NULL;
}

/*-------------------------------------------------------------------*/
/* Halt currently executing I/O command                              */
/*-------------------------------------------------------------------*/
static void commadpt_halt(DEVBLK *dev) {
   if (!dev->busy) {
      return;
   }
   return;
}

/* The following 3 MSG functions ensure only 1 (one) hardcoded      */
/* instance exist for the same numbered msg that is issued on       */
/* multiple situations                                              */
static void msg013e(DEVBLK *dev, char *kw, char *kv)
{
   logmsg("CCCOM013E %1d:%04X:Incorrect %s specification %s\n", dev->ssid, dev->devnum, kw, kv);
}


/*-------------------------------------------------------------------*/
/* Device Initialisation                                             */
/*-------------------------------------------------------------------*/
static int commadpt_init_handler (DEVBLK *dev, int argc, char *argv[]) {
   char thread_name[32];
   int i;
   int rc;
   int pc; /* Parse code */
   int errcnt;
   union {
      int num;
      char text[80];  // Version 3
      // Version 4: char text[MAX_PARSER_STRLEN+1];
   } res;
   /* convert the device type string to hex */
   dev->devtype = (dev->typname[0] & 0x0F) << 12;
   dev->devtype = dev->devtype | ((dev->typname[1] & 0x0F) << 8);
   dev->devtype = dev->devtype | ((dev->typname[2] & 0x0F) << 4);
   dev->devtype = dev->devtype | (dev->typname[3] & 0x0F);
    logmsg("CCCOM001I %1d:%04X: Devicetype set to %04X\n", dev->ssid, dev->devnum, dev->devtype);

   /* For re-initialisation, close the existing file, if any */
   if (dev->fd >= 0)
      (dev->hnd->close)(dev);

   // Version 4: dev->excps = 0;
   logmsg("CCCOM002I %1d:%04X: initialization starting\n", dev->ssid, dev->devnum);

   /* Legacy sense-id not supported */
   dev->numdevid = 0;

   if (dev->commadpt != NULL) {
      commadpt_clean_device(dev);
   }
   rc = commadpt_alloc_device(dev);

   if (rc < 0) {
      logmsg("CCCOM002E %1d:%04X: initialization not performed\n", dev->ssid, dev->devnum);
      return(-1);
   }
   errcnt = 0;

   /* Initialise ports & hosts */
   dev->commadpt->tracesna=0;
   dev->commadpt->debug=0;
   dev->commadpt->busfd = -1;
   dev->commadpt->tagfd = -1;
   dev->commadpt->port = 0;

   for (i = 0; i < argc; i++) {
      pc = parser(ptab, argv[i], &res);

      if (pc < 0) {
         logmsg("CCCOM003E %1d:%04X: error parsing %s\n", dev->ssid, dev->devnum, argv[i]);
         errcnt++;
         continue;
      }
      if (pc == 0) {
         logmsg("CCCOM004E %1d:%04X:  unrecognized parameter %s\n", dev->ssid, dev->devnum, argv[i]);
         errcnt++;
         continue;
      }

      switch(pc) {
         case COMMADPT_KW_TRACESNA:
            if (res.text[0] == 'y' || res.text[0] == 'Y')  {
                  dev->commadpt->tracesna = 1;
                  logmsg("CCCOM005I %1d:%04X: found parameter %s\n", dev->ssid, dev->devnum, argv[i]);
               }
            else
               dev->commadpt->tracesna = 0;
            break;

         case COMMADPT_KW_DEBUG:
            if (res.text[0] == 'y' || res.text[0] == 'Y') {
                  dev->commadpt->debug = 1;
                  logmsg("CCCOM005I %1d:%04X: found parameter %s\n", dev->ssid, dev->devnum, argv[i]);
               }
            else
               dev->commadpt->debug = 0;
            break;

         case COMMADPT_KW_PORT:
            rc = commadpt_getport(res.text);
            if (rc < 0) {
               errcnt++;
               msg013e(dev, "PORT", res.text);
               break;
            }
            dev->commadpt->port = rc;
            break;

         case COMMADPT_KW_ADAPTIP:
#if defined(__GENODE__)
            /* Parsed and not used: a device statement carried over from a
               native run is told what is wrong rather than failing on an
               unrecognised keyword. bus= and tag= are what matter here. */
            break;
#else
            if (strcmp(res.text, "*") == 0) {
               dev->commadpt->adaptip = INADDR_ANY;
               break;
            }
            rc = commadpt_getaddr(&dev->commadpt->adaptip, res.text);
            if (rc != 0) {
               msg013e(dev, "ADAPTIP", res.text);
               errcnt++;
            }
            break;
#endif

         default:
            break;

      } // End of switch(pc)
   } // End of for stmt.

   if (errcnt > 0) {
      logmsg("CCCOM005E %1d:%04X: initialization failed due to previous errors", dev->ssid, dev->devnum);
      return -1;
   }

   dev->bufsize = 256;
   dev->numsense = 2;
   memset(dev->sense, 0, sizeof(dev->sense));

   init_bufpool(dev->commadpt);

   dev->commadpt->devnum = dev->devnum;

   /* Initialize the CA lock */
   initialize_lock(&dev->commadpt->lock);

   /* Initialise thread->I/O & halt initiation EVB */
   initialize_condition(&dev->commadpt->ipc);
   initialize_condition(&dev->commadpt->ipc_halt);

   /* Allocate I/O -> Thread signaling pipe */
   VERIFY(!create_pipe(dev->commadpt->pipe));

   create_pipe(dev->commadpt->pipe);

#ifndef VERS_MAJ
   /* Point to the halt routine for HDV/HIO/HSCH handling */
     dev->halt_device = commadpt_halt;
#endif
   /* Obtain the CA lock */
   obtain_lock(&dev->commadpt->lock);

   /* Start the thread to establish client connections    */
   /* with the remote channel adapter                     */
   strcpy(thread_name, "CSW_adpt");

   /* Set thread-name for debugging purposes */
   if (dev->commadpt->debug)
      logmsg("CCCOM012D %1d:%04X: Starting thread %s \n", dev->ssid, dev->devnum, thread_name);

   rc = create_thread(&dev->commadpt->tthread, &sysblk.detattr, CSW_adpt, dev->commadpt, thread_name);
   if (rc) {
      logmsg("CCCOM012E %1d:%04X: Error in function create_thread(): %s\n", dev->ssid, dev->devnum, strerror(rc));
      release_lock(&dev->commadpt->lock);
      return -1;
   }

   /* Release the CA lock */
   release_lock(&dev->commadpt->lock);
   /* Indicate succesfull completion */
   return 0;
}

/*-------------------------------------------------------------------*/
/* Query the device definition                                       */
/*-------------------------------------------------------------------*/
static void commadpt_query_device (DEVBLK *dev, char **class,
                int buflen, char *buffer) {
   *class = "LINE";
   snprintf(buffer, buflen, "Read count=%d, Write count=%d\n",
           dev->commadpt->read_ccw_count, dev->commadpt->write_ccw_count);
}

/*-------------------------------------------------------------------*/
/* Close the device                                                  */
/* Invoked by HERCULES shutdown & DEVINIT processing                 */
/*-------------------------------------------------------------------*/
static int commadpt_close_device (DEVBLK *dev) {
   if (dev->ccwtrace) {
      logmsg("CCCOM006D %1d:%04X:Closing down\n", dev->ssid, dev->devnum);
   }

   /* Obtain the CA lock */
   obtain_lock(&dev->commadpt->lock);

   /* Terminate current I/O thread if necessary */
   if (dev->busy) {
      commadpt_halt(dev);
   }
   free_bufpool(dev->commadpt);

   /* Release the CA lock   */
   release_lock(&dev->commadpt->lock);

   /* Free all work storage */
   commadpt_clean_device(dev);

   /* Indicate to hercules the device is no longer opened */
   dev->fd = -1;

   if (dev->ccwtrace) {
      logmsg("CCCOM006D %1d:%04X:Closed down\n", dev->ssid, dev->devnum);
   }
   return 0;
}

/*-------------------------------------------------------------------*/
/* xmit CCW to 3705/3720/3725 Channel Adapter for processing         */
/* Inject returned device status back into the channel               */
/*-------------------------------------------------------------------*/
#ifdef VERS_MAJ                // must be version 4,x or
#define Uxn uint32_t
#else                          // else version 3.x
#define Uxn uint16_t
#endif
static void commadpt_execute_ccw (DEVBLK *dev, BYTE code, BYTE flags,
        BYTE chained, Uxn count, BYTE prevcode, int ccwseq,
        BYTE *iobuf, BYTE *more, BYTE *unitstat, Uxn *residual)
   {
   U32 num;    /* Work : Actual CCW transfer count                   */
   char    buf[80];
   uint32_t     rc;

   struct CCW {
      BYTE code;
      BYTE data_address[3];
      BYTE flags;
      BYTE chain;
      BYTE count_ho;
      BYTE count_lo;
   } ccw;

   ccw.code = code;
   ccw.flags = flags;
   ccw.chain = chained;
   ccw.count_ho = (count & 0x0000FF00) >> 8 ;
   ccw.count_lo = (count & 0x000000FF);

   if (dev->commadpt->debug)
      logmsg("CCBUS005D %1d:%04X: CCW=%02X, CCW count=%04X\n", dev->ssid, dev->devnum, code, count);

   *residual = 0;

   if (IsSocketConnected(dev->commadpt->busfd, dev->ssid, dev->devnum)) {
      /* Obtain the COMMADPT lock */
      obtain_lock(&dev->commadpt->lock);
      dev->commadpt->unitstat = unitstat;

      if (dev->commadpt->debug)
         logmsg("CCBUS006D %1d:%04X: Sending CCW %02X\n", dev->ssid , dev->devnum, code);

      rc = write_adpt((void*)&ccw, sizeof(ccw), dev->commadpt);
      /* Wait for ACK */

      switch (code) {
         /*----------------------------------------------------------*/
         /* TEST I/O                                                 */
         /*----------------------------------------------------------*/
         case 0x00:
            *residual = 0;
            /* Get Channel Return Status */
            rc = recv(dev->commadpt->busfd, &dev->commadpt->carnstat, 1, 0);
            break;

         /*----------------------------------------------------------*/
         /* I/O NO-OP & Discontact                                                */
         /*----------------------------------------------------------*/
         case 0x03:     // No_Op
            *residual = 0;
            /* Get Channel Return Status */
            rc = recv(dev->commadpt->busfd, &dev->commadpt->carnstat, 1, 0);
            *unitstat = dev->commadpt->carnstat;
            break;

         /*----------------------------------------------------------*/
         /* BASIC SENSE                                              */
         /*----------------------------------------------------------*/
         case 0x04:
            /* Wait for the sense data -- ONE BYTE, and read exactly one.
             *
             * The 3705 sends a single sense byte and then, as a separate
             * write, the one-byte channel return status. Two one-byte writes
             * back to back on a loopback socket coalesce readily, and a read
             * of up to 256 then takes BOTH: numsense becomes 2, the status
             * byte is delivered as sense data, and the recv below waits
             * forever for a byte that has already been consumed.
             *
             * Nothing reports that. The adapter thread is blocked, the
             * simulator is idle, and three minutes later VTAM says
             *     IST291I 370X N16A LOAD FAILED- LOADER FAILURE IN 370X
             *
             * It is a race, so it is intermittent -- and the NCP load issues
             * exactly two SENSE commands, which is about the rate at which
             * the load failed. Anything that slows either side hides it,
             * which is why the load succeeded every time under a CCW trace.
             *
             * See i3705_chan_T2.c case 0x04: the send is send_socket(..., 1)
             * in both arms, so one is not a guess.
             */
            rc = recv(dev->commadpt->busfd, dev->sense, 1, 0);
            dev->numsense = rc;
            dev->commadpt->unack_attn_count = 0;
            *more = 0;
            if (dev->commadpt->debug)
               logmsg("CCBUS007D %1d:%04X: sense data %02X\n", dev->ssid, dev->devnum, dev->sense[0]);
            /* Copy device sense bytes to channel I/O buffer */
            memcpy (iobuf, dev->sense, rc);
            *residual = 0;

            /* Get Channel Return Status */
            rc = recv(dev->commadpt->busfd, &dev->commadpt->carnstat, 1, 0);
            *unitstat = dev->commadpt->carnstat;
            break;

         /*----------------------------------------------------------*/
         /* WRITE IPL                                                */
         /*----------------------------------------------------------*/
         case 0x05:
            dev->commadpt->unack_attn_count = 0;
            logdump("WRITE", dev, iobuf, count);
            tracesna(dev, iobuf, code);
            rc = write_adpt(iobuf, count, dev->commadpt);
            if (rc == 0) {
               *residual = 0;
               *more =  0;
               /* Get Channel Return Status */
               rc = recv(dev->commadpt->busfd, &dev->commadpt->carnstat, 1, 0);
               *unitstat = dev->commadpt->carnstat;
            } else {
               *unitstat |= CSW_ATTN;
               *unitstat |= CSW_UX | CSW_ATTN;
            }
            break;
         /*----------------------------------------------------------*/
         /* SENSE-ID  (not for 3705)                                 */
         /*----------------------------------------------------------*/
         case 0xE4:
            if ((dev->devtype & 0x00FF) == 0x05) {   /* If this is a 3705....  */
               ; //NOP
            } else {
               /* Wait for the sense-id data */
               rc = recv(dev->commadpt->busfd, dev->commadpt->inpbuf, count, 0);
               /* Calculate residual byte count */
               num = (count < rc) ? count:rc;
               *residual = count - num;
               *more = count < rc ? 1 : 0;
               /* Copy device sense-id bytes to channel I/O buffer */
               memcpy (iobuf, dev->commadpt->inpbuf, num);
               /* Initialize the device identifier bytes */
               memcpy (dev->devid, dev->commadpt->inpbuf, num);
               logdump("SENSE-ID", dev, iobuf, num);
               dev->numdevid = 4;
            } // End if dev->devtype

            /* Get Channel Return Status */
            rc = recv(dev->commadpt->busfd, &dev->commadpt->carnstat, 1, 0);
            *unitstat = dev->commadpt->carnstat;
            break;


         /*----------------------------------------------------------*/
         /* READ CCW                                                 */
         /*----------------------------------------------------------*/
         case 0x02:     /* READ */
#if defined(__GENODE__)
            /* Length first -- see i3705_chan_T2.c case 0x02. The device
               sends two bytes of count and then that many bytes, so this
               read and the status read after it are both of a known size
               and neither can swallow the other. */
            {
               unsigned char lenpfx[2];
               int want;
               rc = ipc_read_exact( dev->commadpt->busfd, lenpfx, 2 );
               if (rc != 2) { rc = -1; break; }
               want = (lenpfx[0] << 8) | lenpfx[1];
               if (want > (int)sizeof(dev->commadpt->inpbuf))
                  want = (int)sizeof(dev->commadpt->inpbuf);
               rc = want ? ipc_read_exact( dev->commadpt->busfd,
                                           dev->commadpt->inpbuf, want ) : 0;
            }
#else
            /* Wait for the remote channel adapter data */
            rc = recv(dev->commadpt->busfd, dev->commadpt->inpbuf, count, 0);
#endif

            dev->commadpt->read_ccw_count++;
            dev->commadpt->unack_attn_count = 0;
            *more = 0;
            /* Copy data to I/O buffer */
            memcpy (iobuf, dev->commadpt->inpbuf, rc);
            *residual = count - rc;
            logdump("READ", dev, iobuf, rc);
            tracesna(dev, iobuf, code);

            /* Get Channel Return Status */
            rc = recv(dev->commadpt->busfd, &dev->commadpt->carnstat, 1, 0);
            *unitstat = dev->commadpt->carnstat;
            break;

         /*----------------------------------------------------------*/
         /* WRITE CCW                                                */
         /*----------------------------------------------------------*/
         case 0x01:     /* WRITE */
         case 0x09:     /* WRITE BREAK */
            dev->commadpt->write_ccw_count++;
            dev->commadpt->unack_attn_count = 0;
            logdump("WRITE", dev, iobuf, count);
            tracesna(dev, iobuf, code);
            rc = write_adpt(iobuf, count, dev->commadpt);
            if (rc == 0) {
               *residual = 0;
               *more = 0;
               /* Get Channel Return Status */
               rc = recv(dev->commadpt->busfd, &dev->commadpt->carnstat, 1, 0);
               *unitstat = dev->commadpt->carnstat;
            } else {
               *unitstat |= CSW_ATTN;
               *unitstat |= CSW_UX | CSW_ATTN;
            }
            break;
         /*----------------------------------------------------------*/
         /* Below set of CCW's require initial select only           */
         /*----------------------------------------------------------*/
         case 0x31:    // Write start 0
         case 0x32:    // Read start 0
         case 0x51:    // Write start 1
         case 0x52:    // Read start 1
         case 0x61:    // Write XID
         case 0x62:    // Read XID
         case 0x93:    // Restart Reset
         case 0xA3:    // Discontact
         case 0xC3:    // Contact
            dev->commadpt->unack_attn_count = 0;
            *residual = count;
            *more = 0;
            /* Get Channel Return Status */
            rc = recv(dev->commadpt->busfd, &dev->commadpt->carnstat, 1, 0);
            *unitstat = dev->commadpt->carnstat;
            break;
         /*----------------------------------------------------------*/
         /*  Default: CCW not recognized, send unit check            */
         /*----------------------------------------------------------*/
         default:
            *unitstat = CSW_CE + CSW_DE + CSW_UC;
            dev->sense[0] = SENSE_CR;
            break;
      } // End of switch (code)

      // Remove if below works under Linux: bzero(buf, 80);
      memset(buf, 0x00, 80);
      release_lock(&dev->commadpt->lock);
      //dev->commadpt->ccwactive = 0x00;
      if (dev->commadpt->debug)
         logmsg("CCBUS002D %1d:%04X: Release Lock OK\n", dev->ssid, dev->devnum);
   } else {
      /*----------------------------------------------------------*/
      /*  37x5 not online, set command reject sense               */
      /*----------------------------------------------------------*/
      /* Set command reject sense byte, and unit check status     */
      *unitstat = CSW_CE + CSW_DE + CSW_UC;
      dev->sense[0] = SENSE_CR;

   } // End if IsSocketConnected...
}

/*-------------------------------------------------------------------*/
/* DEVICE FUNCTION POINTERS                                          */
/*-------------------------------------------------------------------*/
#if defined(OPTION_DYNAMIC_LOAD)
static
#endif

DEVHND com3705_device_hndinfo = {
        &commadpt_init_handler,        /* Device Initialisation      */
        &commadpt_execute_ccw,         /* Device CCW execute         */
        &commadpt_close_device,        /* Device Close               */
        &commadpt_query_device,        /* Device Query               */
#ifdef VERS_MAJ
        NULL,                          /* Device Extended Query      */
#endif
        NULL,                          /* Device Start channel pgm   */
        NULL,                          /* Device End channel pgm     */
        NULL,                          /* Device Resume channel pgm  */
        NULL,                          /* Device Suspend channel pgm */
#ifdef VERS_MAJ
        &commadpt_halt,                /* Device Halt channel pgm    */
#endif
        NULL,                          /* Device Read                */
        NULL,                          /* Device Write               */
        NULL,                          /* Device Query used          */
        NULL,                          /* Device Reserve             */
        NULL,                          /* Device Release             */
        NULL,                          /* Device Attention           */
        commadpt_immed_command,        /* Immediate CCW Codes        */
        NULL,                          /* Signal Adapter Input       */
        NULL,                          /* Signal Adapter Output      */
#ifdef VERS_MAJ
        NULL,                          /* Signal Adapter Sync        */
        NULL,                          /* Signal Adapter Output Mult */
        NULL,                          /* QDIO subsys desc           */
        NULL,                          /* QDIO set subchan ind       */
#endif
        NULL,                          /* Hercules suspend           */
        NULL                           /* Hercules resume            */
};

/* Libtool static name colision resolution                           */
/* note : lt_dlopen will look for symbol & modulename_LTX_symbol     */
#if !defined(HDL_BUILD_SHARED) && defined(HDL_USE_LIBTOOL)
#define hdl_ddev hdt3705_LTX_hdl_ddev
#define hdl_depc hdt3705_LTX_hdl_depc
#define hdl_reso hdt3705_LTX_hdl_reso
#define hdl_init hdt3705_LTX_hdl_init
#define hdl_fini hdt3705_LTX_hdl_fini
#endif

HDL_DEPENDENCY_SECTION;
{
   //HDL_DEPENDENCY(HERCULES);
   HDL_DEPENDENCY(DEVBLK);
   HDL_DEPENDENCY(SYSBLK);
}
END_DEPENDENCY_SECTION;


#if defined(WIN32) && !defined(HDL_USE_LIBTOOL) && !defined(_MSVC_)
   #undef sysblk
   HDL_RESOLVER_SECTION;
   {
      HDL_RESOLVE_PTRVAR( psysblk, sysblk );
   }
   END_RESOLVER_SECTION;
#endif

  /* NB: Make sure hdteq.c is updated to map 3720 and 3725 to 3705              */
HDL_DEVICE_SECTION;
{
   HDL_DEVICE(3705, com3705_device_hndinfo );
   HDL_DEVICE(3720, com3705_device_hndinfo );
   HDL_DEVICE(3725, com3705_device_hndinfo );
}
END_DEVICE_SECTION;

