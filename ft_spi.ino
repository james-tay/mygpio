/*
   OVERVIEW

   In order to interact with an SPI device on an ESP32, we create a dedicated
   thread which will manage the lifecycle of the SPI interaction. This thread
   is supplied the initial parameters for setting up SPI on the electrical
   layer (ie, speed, data order and data mode) as well as the GPIO pins used.
   It then opens a listening TCP socket, through which an external party may
   interact with the SPI device.

   References
     - https://www.e-tinkers.com/2020/03/do-you-know-arduino-spi-and-arduino-spi-library/

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
     SPI.transfer(c) ;   // writes single byte, reads single byte into "c"
     SPI.transfer(buf) ; // writes 6 bytes, reads back 6 bytes, byte by byte

   With this in mind, when a TCP client connects to this thread, the message
   format is,

     from client:
       <1-byte opcode><2-byte length><payload...><0x00, 0x00>

     from server:
       <1-byte opcode><2-byte length><payload...><0x00, 0x00>

   The 2-byte length field is in big endian and informs the receiver to expect
   between 1 and 65535 bytes of payload. In reality, the maximum payload is
   capped at "SPI_MSG_BUFSIZE - 5". At the end of this payload is exactly
   2x bytes of 0x00. This is a sanity check which allows both parties to
   discard unintentional/errornous traffic (eg, someone telnets by mistake).

   Thus upon connecting to the server, the client is expected to send a message
   which is at least 6 bytes (ie, 1 + 2 + 1 + 2). This thread delivers the
   payload to the SPI device while retrieving response bytes at the same time.
   Depending on the client's opcode, this thread may immediately send the reply
   to the client, again this is a miminum of 6 bytes.

   OPCODES

   All messages to/from a client must begin with an opcode, which identifies
   what the message is about. The following opcode IDs are implemented.

      1 - client sending echo request
      2 - server sending echo response
      3 - client sending payload, do not send response
      4 - client sending payload, send response immediately
      5 - server sending response

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
     spi_bus_free(VSPI_HOST) ;

   Each time a client connects on this thread's listening TCP socket, this
   thread performs,

     spi->beginTransaction(...) ;
     digitalWrite(CS_PIN, LOW) ;

   Over the life cycle of the TCP session, this thread handles messages
   described above in "DATA TRANSFER". Only 1x connected client is supported
   at any one time. Once the TCP client disconnects, this thread performs,

     spi->endTransaction() ;
     digitalWrite(CS_PIN, HIGH) ;

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
     - CS pin
     - Client max allowed idle time (secs)

   To send test message which includes 2x of SPI data to this thread,

     $ echo -ne "\x01\x00\x02\xbe\xef\x00\x00" >/dev/tcp/example.com/9000
*/

#include <SPI.h>

/*
   maximum size of a message to/from client. This value was chosen so that
   a complete message fits within a single TCP packet (ie, IP + TCP + options)
   so as to avoid "fragmentation".
*/

#define SPI_MSG_BUFSIZE 1440

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

void ft_spi (S_thread_entry *p)
{
  if (p->num_args != 6)
  {
    strcpy (p->msg, "FATAL! Expecting 5x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int listen_port = atoi (p->in_args[0]) ;
  int mosi_pin = atoi (p->in_args[1]) ;
  int miso_pin = atoi (p->in_args[2]) ;
  int sck_pin = atoi (p->in_args[3]) ;
  int cs_pin = atoi (p->in_args[4]) ;
  int max_idle_secs = atoi (p->in_args[5]) ;

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

  p->num_int_results = 1 ;

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

  /*
     this is the main loop, wait for network activity, but check "p->state"
     periodically in case we need to shutdown.
  */

  int nap_ms = THREAD_SHUTDOWN_PERIOD / 5 ;
  int num_fds = 0 ;
  int client_sd = 0 ;
  int result, amt ;
  fd_set rfds ;
  struct timeval tv ;

  while (p->state == THREAD_RUNNING)
  {
    tv.tv_sec = 0 ;
    tv.tv_usec = nap_ms * 1000 ;
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
        client_sd = accept (listen_sd, NULL, NULL) ;
        p->results[0].i_value = 1 ;
      }
      if (FD_ISSET (client_sd, &rfds))
      {
        amt = recv (client_sd, spi_buf, SPI_MSG_BUFSIZE, MSG_PEEK) ;
        if (amt < 1)                                    // tcp client closed
        {
          close (client_sd) ;
          client_sd = 0 ;
          p->results[0].i_value = 0 ;
        }
        else
        {
          amt = f_spi_validate_msg (p, client_sd, spi_buf, amt) ;
          if (amt < 0)                                  // invalid message
          {
            close (client_sd) ;
            client_sd = 0 ;
            p->results[0].i_value = 0 ;
          }
          else
          {
            amt = read (client_sd, spi_buf, amt) ;      // pull exact message




          }
        }
      }
    }
  }

  close (listen_sd) ;
  if (client_sd > 0)
    close(client_sd) ;
  p->state = THREAD_STOPPED ;
  free (spi_buf) ;
}

