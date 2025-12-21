/*
   OVERVIEW

   In order to interact with an SPI device on an ESP32, we create a dedicated
   thread which will manage the lifecycle of the SPI interaction. This thread
   is supplied the initial parameters for setting up SPI on the electrical
   layer (ie, speed, data order and data mode) as well as the GPIO pins used.
   It then opens a listening TCP socket, through which an external party may
   interact with the SPI device.

   Recall that, for the best performance, the ESP32 has 2x general purpose
   SPI interfaces (called HSPI and VSPI).

                VSPI    HSPI
     MOSI       23      13
     MISO       19      12
     SCK        18      14
     CS/SS      5       15

   DATA TRANSFER / PROTOCOL

   Note that data transfers on SPI always involve simultaneous reads after
   writes. Specifically, the SPI API functions,

     uint8_t c ;
     uint8_t buf[6] ;
     c = ... ;
     memcpy(buf, ...) ;
     SPI.transfer(c) ;      // writes single byte, reads single byte into "c"
     SPI.transfer(buf, 6) ; // writes 6 bytes, reads back 6 bytes into "buf"

   With this in mind, when a TCP client connects to this thread, the message
   format between client and server is always,

       <1-byte opcode><2-byte length><payload...><0x00, 0x00>

   The 2-byte length field is in network byte order (ie, big endian) and it
   informs the receiver to expect between 1 and 65535 bytes of payload. In
   reality, the maximum payload is capped at "SPI_MSG_BUFSIZE - 5". At the end
   of this payload is exactly 2x bytes of 0x00. This is a sanity check which
   allows both parties to discard unintentional/errornous traffic (eg, someone
   telnets by mistake).

   Thus upon connecting to the server, the client is expected to send a message
   which is at least 6 bytes (ie, 1 + 2 + 1 + 2). This thread delivers the
   payload to the SPI device while retrieving response bytes at the same time.
   Depending on the client's opcode, this thread may immediately send the reply
   to the client, again this is a miminum of 6 bytes.

   OPCODES

   All messages to/from a client must begin with an opcode, which identifies
   what the message is about. The following opcode IDs are implemented.

      1 - client sending SPI config (do this before sending SPI payload)
      2 - client sending echo request
      3 - server sending echo response
      4 - client sending payload, do not send response
      5 - client sending payload, send response immediately
      6 - server sending response (reply for opcodes 4 and 5)

   The client's SPI config message (ie, opcode 1) has the following format,

     <opcode><length><payload...><0x00,0x00>

   Where,
     opcode  - is always 1
     length  - is always 6
     payload - field format for SPI settings as follows

       <4-byte:SCK_freq_khz><1-byte:SPI_order><1-byte:SPI_mode>

     SPI_order,
       0 = LSBFIRST
       1 = MSBFIRST
     SPI_mode, for CPOL (Clock Polarity) and CPHA (Clock Phase)
       0 = CPOL 0 (clock idle at 0v), CPHA 0 (sample data on 1st edge of clock)
       1 = CPOL 0, CPHA 1 (sample data on 2nd edge of clock)
       2 = CPOL 1 (clock idle at 3.3v), CPHA 0
       3 = CPOL 1, CPHA 1

   If this thread receives an unsupported opcode from the client, the TCP
   session is closed immediately.

   IMPLEMENTATION

   When this thread starts up, it initializes the ESP32's hardware "VSPI"
   interface. This implies that only 1x instance of this thread should be
   allowed to run at any given time. The user specifies the 4x SPI pins (best
   to stick to the "recommended" pins). Thus this thread calls,

     spi = new SPIClass(VSPI) ;
     spi->begin(pins ...) ;

   When this thread is terminated (ie, told to shutdown), a proper cleanup is
   attempted, which involves,

     spi->end() ;
     delete spi ;

   Each time a client connects on this thread's listening TCP socket, the
   client MUST send an SPI config message, after which the thread performs,

     spi->beginTransaction(...) ;

   Over the life cycle of the TCP session, this thread handles messages
   described above in "DATA TRANSFER". Only 1x connected TCP client is
   supported at any one time. Once the TCP client disconnects, this thread
   performs,

     spi->endTransaction() ;

   In the event that the client is connected to this thread, but suddenly
   crashes, then this thread keeps the client's socket descriptor open
   indefinitely. After the client restarts, it will be unable to connect to
   this thread. For this reason, this thread monitors client idle time and
   will disconnect a connected client if this idle time is exceeded.

   USAGE

   This thread expects the following parameters,

     - TCP_listen_port
     - MOSI pin
     - MISO pin
     - SCK pin
     - Client max allowed idle time (secs)

   To send test message to this thread using "socat" (and read the reply),

     $ echo -ne "\x02\x00\x01\xff\x00\x00" | \
         socat - TCP4:192.168.6.50:9000 | xxd

   To setup the SPI device (5khz clock, order 1, mode 2) and hold the TCP
   session open for 5 seconds,

     $ (echo -ne "\x01\x00\x06\x00\x00\x00\x05\x01\x02\x00\x00" ; sleep 5) | \
         socat - TCP4:192.168.6.50:9000 | xxd

   The following example sets up SPI with SCK at 10khz, sends the WHO_AM_I
   request (ie, 0xf5) to an MPU-9250 and reads a 1-byte response (ie, 0x75).

     #!/bin/bash
     CTL_HOST="esp32.example.com"
     CTL_PORT=9000
     CS_PIN=5

     curl "http://$CTL_HOST/v1?cmd=hi+$CS_PIN" # pull CS high

     # SPI config - 10khz, MSBFIRST, SPI mode 0
     CMD_CONFIG="\x01\x00\x06\x00\x00\x00\xa0\x01\x00\x00\x00"
     # Send WHO_AM_I
     CMD_WHOAMI="\x04\x00\x01\xf5\x00\x00"
     # Get 1-byte response
     CMD_REPLY="\x05\x00\x01\x00\x00\x00"

     curl "http://$CTL_HOST/v1?cmd=lo+$CS_PIN" # pull CS low
     (
       echo -ne "$CMD_CONFIG"
       echo -ne "$CMD_WHOAMI"
       echo -ne "$CMD_REPLY"
     ) | socat - TCP4:$CTL_HOST:$CTL_PORT | xxd

     curl "http://$CTL_HOST/v1?cmd=hi+$CS_PIN" # pull CS back to high

   The output from the above script looks like this (note the \x75 reply),

     pin:5 HIGH
     pin:5 LOW
     00000000: 0600 0175 0000                           ...u..
     pin:5 HIGH

   NOTES

   This thread only manages the clocking in/out of data over the MOSI/MISO
   pins. This thread does NOT manage the CS pin, that would be the user's
   responsibility. In this way, this thread can interact with multiple SPI
   devices when the user manages the individual CS pins.
*/

#include <SPI.h>

/*
   maximum size of a message to/from client. This value was chosen so that
   a complete message fits within a single TCP packet (ie, IP + TCP + options)
   so as to avoid "fragmentation".
*/

#define SPI_MSG_BUFSIZE 1440

/* The various SPI opcodes used between us and a TCP client */

#define SPI_OP_SPI_CONFIG       1
#define SPI_OP_ECHO_REQ         2
#define SPI_OP_ECHO_RESP        3
#define SPI_OP_SEND_NO_RESP     4
#define SPI_OP_SEND_WITH_RESP   5
#define SPI_OP_DEVICE_RESP      6

/* SPI speed limits for SCK */

#define SPI_MAX_SCK_FREQ_KHZ 40000

/* Data structure in a SPI_OP_SPI_CONFIG message from the TCP client */

struct spi_config_msg
{
  unsigned long freq_khz ;
  unsigned char order ;
  unsigned char mode ;
} ;
typedef struct spi_config_msg S_spi_config_msg ;

/* Global variables */

int G_spi_transaction=0 ;       // spi->beginTransaction() has been called
SPIClass *G_spi_dev=NULL ;      // the SPI hardware device on the ESP32

/* ========================================================================= */

/*
   This function is called from ft_spi() just after it has performed a recv()
   with MSG_PEEK on "sd". Thus, "msg" contains a preview of "len" bytes. The
   incoming message may or may not be valid. A message is valid if,
     a) first byte is a valid opcode
     b) next 2x bytes is a valid 16-bit length
     c) the last 2x bytes are 0x00,0x00
   If valid, this function returns the length of the message. If invalid, we
   return -1.
*/

int f_spi_validate_msg (S_thread_entry *p, int sd, unsigned char *msg, int len)
{
  unsigned char opcode ;
  unsigned short msg_size ;

  if (len < 6)                                  // "msg" is impossibly small
    return (-1) ;

  opcode = msg[0] ;
  memcpy (&msg_size, msg+1, sizeof(msg_size)) ;
  msg_size = ntohs(msg_size) ;

  /* now we know "msg_size", see if "msg" is the expected "len" */

  int expected = 1 + 2 + msg_size + 2 ;
  if (len < expected)
    return (-1) ;                               // "msg" is too short

  /* make sure "msg" ends with \x00, \x00 */

  if ((msg[expected-1] != 0) || (msg[expected-2] != 0))
    return (-1) ;                               // missing \x00 \x00 at the end
  else
    return (expected) ;
}

/*
   This function is called from ft_spi() and supplied a complete "msg" of
   "len" bytes. Our job is to identify the opcode and act accordingly. For
   the most part, we return 1, or 0 if we received an invalid opcode.
   IMPORTANT !!! the contents of "msg" may be replaced in the course of our
   work.
*/

int f_spi_handle_msg (S_thread_entry *p, int sd, unsigned char *msg, int len)
{
  unsigned char opcode ;
  unsigned short msg_size ;
  unsigned char *payload ;
  S_spi_config_msg cfg ;
  SPISettings s_settings ;

  opcode = msg[0] ;
  memcpy (&msg_size, msg+1, sizeof(msg_size)) ;
  msg_size = ntohs(msg_size) ;
  payload = msg + 3 ;
  sprintf (p->msg, "opcode:%d size:%d", opcode, msg_size) ;

  switch (opcode)
  {
    case SPI_OP_SPI_CONFIG:                             // setup SPI settings
      memcpy (&cfg, payload, sizeof(S_spi_config_msg)) ;
      cfg.freq_khz = ntohl(cfg.freq_khz) ;

      /* sanity check all values before we continue */

      if ((cfg.freq_khz < 1) ||
          (cfg.freq_khz > SPI_MAX_SCK_FREQ_KHZ) ||
          (cfg.order > 1) ||
          (cfg.mode > 3))
        return (0) ;
      sprintf (p->msg, "freq:%dkhz order:%d mode:%d",
               cfg.freq_khz, cfg.order, cfg.mode) ;

      /* apply SPI settings and get hardware ready */

      G_spi_transaction = 1 ;
      G_spi_dev->beginTransaction (SPISettings(cfg.freq_khz * 1000,
                                               cfg.order, cfg.mode)) ;
      break ;

    case SPI_OP_ECHO_REQ:                               // echo request
      msg[0] = SPI_OP_ECHO_RESP ;
      write (sd, msg, len) ;
      break ;

    case SPI_OP_SEND_NO_RESP:                           // send bytes w/o reply
      if (G_spi_transaction == 0)
        return (0) ; // return failure
      G_spi_dev->transfer (payload, msg_size) ;
      p->results[3].i_value = p->results[3].i_value + msg_size ;
      break ;

    case SPI_OP_SEND_WITH_RESP:                         // send bytes /w reply
      if (G_spi_transaction == 0)
        return (0) ; // return failure
      G_spi_dev->transfer (payload, msg_size) ;
      p->results[3].i_value = p->results[3].i_value + msg_size ;
      msg[0] = SPI_OP_DEVICE_RESP ;
      write(sd, msg, len) ;
      break ;

    default:
      return (0) ;
  }

  p->results[1].i_value++ ;                             // message handled
  return (1) ;                                          // return success
}

void ft_spi (S_thread_entry *p)
{
  if (p->num_args != 5)
  {
    strcpy (p->msg, "FATAL! Expecting 5x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int listen_port = atoi (p->in_args[0]) ;
  int mosi_pin = atoi (p->in_args[1]) ;
  int miso_pin = atoi (p->in_args[2]) ;
  int sck_pin = atoi (p->in_args[3]) ;
  int max_idle_ms = atoi (p->in_args[4]) * 1000 ;

  /* try allocate the SPI message buffer from heap */

  unsigned char *spi_buf = (unsigned char*) malloc(SPI_MSG_BUFSIZE) ;
  if (spi_buf == NULL)
  {
    strcpy (p->msg, "FATAL! cannot allocate spi_buf") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  /* configure metrics that we'll expose */

  p->results[0].num_tags = 1 ;
  p->results[0].meta[0] = (char*) "client" ;
  p->results[0].data[0] = (char*) "\"connected\"" ;

  p->results[1].num_tags = 1 ;
  p->results[1].meta[0] = (char*) "msgs" ;
  p->results[1].data[0] = (char*) "\"handled\"" ;

  p->results[2].num_tags = 1 ;
  p->results[2].meta[0] = (char*) "msgs" ;
  p->results[2].data[0] = (char*) "\"invalid\"" ;

  p->results[3].num_tags = 1 ;
  p->results[3].meta[0] = (char*) "bytes" ;
  p->results[3].data[0] = (char*) "\"transferred\"" ;

  p->num_int_results = 4 ;

  /* try to setup a listening TCP port */

  struct sockaddr_in addr ;
  memset (&addr, 0, sizeof(addr)) ;
  addr.sin_family = AF_INET ;
  addr.sin_addr.s_addr = INADDR_ANY ;
  addr.sin_port = htons (listen_port) ;
  int listen_sd = socket (AF_INET, SOCK_STREAM, IPPROTO_IP) ;
  if (bind (listen_sd, (const struct sockaddr*) &addr, sizeof(addr)) < 0)
  {
    close (listen_sd) ;
    strcpy (p->msg, "FATAL! bind() failed") ;
    p->state = THREAD_STOPPED ;
    free (spi_buf) ;
    return ;
  }
  if (listen (listen_sd, 1) != 0)
  {
    close (listen_sd) ;
    strcpy (p->msg, "FATAL! listen() failed") ;
    p->state = THREAD_STOPPED ;
    free (spi_buf) ;
    return ;
  }
  sprintf (p->msg, "listening on %d", listen_port) ;

  /* bring SPI interface hardware online, note that CS is user controlled */

  if (G_spi_dev == NULL)
  {
    G_spi_dev = new SPIClass (VSPI) ;
    G_spi_dev->begin (sck_pin, miso_pin, mosi_pin, -1) ;
  }

  /*
     this is the main loop, wait for network activity, but check "p->state"
     periodically in case we need to shutdown.
  */

  int nap_ms = THREAD_SHUTDOWN_PERIOD / 5 ;
  int num_fds = 0 ;
  int client_sd = 0 ;
  int close_client = 0 ;
  int result, amt ;
  unsigned long last_activity_ms=0 ;
  fd_set rfds ;
  struct timeval tv ;

  while (p->state == THREAD_RUNNING)
  {
    close_client = 0 ;
    tv.tv_sec = 0 ;
    tv.tv_usec = nap_ms * 1000 ;
    FD_ZERO (&rfds) ;
    if (client_sd > 0)
    {
      FD_SET (client_sd, &rfds) ;       // only monitor TCP client
      num_fds = client_sd + 1 ;
    }
    else
    {
      FD_SET (listen_sd, &rfds) ;       // only monitor listening socket
      num_fds = listen_sd + 1 ;
    }
    result = select (num_fds, &rfds, NULL, NULL, &tv) ;
    if (result > 0)
    {
      if (FD_ISSET (listen_sd, &rfds))                  // new tcp client !!
      {
        int flag = 1 ;
        client_sd = accept (listen_sd, NULL, NULL) ;
        setsockopt (client_sd, IPPROTO_TCP, TCP_NODELAY,
                    (char*)&flag, sizeof(int)) ;
        p->results[0].i_value = 1 ;
        last_activity_ms = millis () ;
        sprintf (p->msg, "client on %d", listen_port) ;
      }
      if (FD_ISSET (client_sd, &rfds))
      {
        amt = recv (client_sd, spi_buf, SPI_MSG_BUFSIZE, MSG_PEEK) ;
        if (amt < 1)                                    // tcp client closed
          close_client = 1 ;
        else
        {
          amt = f_spi_validate_msg (p, client_sd, spi_buf, amt) ;
          if (amt < 0)                                  // invalid message
          {
            p->results[2].i_value++ ;
            close_client = 1 ;
          }
          else
          {
            last_activity_ms = millis () ;
            amt = read (client_sd, spi_buf, amt) ;      // pull one message
            if (f_spi_handle_msg (p, client_sd, spi_buf, amt) == 0)
              close_client = 1 ;
          }
        }
      }
    }

    /* check if TCP client is connected and has idled too long */

    if (client_sd > 0)
    {
      unsigned long now = millis () ;
      if (now - last_activity_ms > max_idle_ms)
        close_client = 1 ;
    }

    /* if the "close_client" flag is set, then disconnect TCP client now */

    if ((client_sd > 0) && (close_client))
    {
      shutdown (client_sd, SHUT_RDWR) ;
      close (client_sd) ;
      client_sd = 0 ;
      p->results[0].i_value = 0 ;
      if (G_spi_transaction)
      {
        G_spi_dev->endTransaction () ;
        G_spi_transaction = 0 ;
      }
      sprintf (p->msg, "listening on %d", listen_port) ;
    }
  }

  /* if we got here, that's because we've been told to shutdown */

  close (listen_sd) ;
  if (client_sd > 0)
    close(client_sd) ;
  p->state = THREAD_STOPPED ;
  free (spi_buf) ;

  G_spi_dev->end() ;
  delete (G_spi_dev) ;
  G_spi_dev = NULL ;
}

