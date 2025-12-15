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

   DATA TRANSFER

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
       <2-byte length><payload...><0x00, 0x00>

     from server:
       <2-byte length><payload...><0x00, 0x00>

   The 2-byte length field is in big endian and informs the receiver to expect
   between 1 and 65535 bytes of payload. At the end of this payload is exactly
   2x bytes of 0x00. This is a sanity check which allows both parties to
   discard unintentional/errornous traffic (eg, someone telnets by mistake).

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
   described above in "DATA TRANSFER". Once the TCP client disconnects, this
   thread performs,

     spi->endTransaction() ;
     digitalWrite(CS_PIN, HIGH) ;

   USAGE

   This thread expects the following parameters,

     - TCP_listen_port
     - MOSI pin
     - MISO pin
     - SCK pin
     - CS pin
*/

#include <SPI.h>

void ft_spi (S_thread_entry *p)
{
  if (p->num_args != 5)
  {
    strcpy (p->msg, "FATAL! Expecting 5x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }



}

