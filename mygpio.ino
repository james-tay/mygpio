/*
   Build/Upload

     % arduino-cli compile -b esp8266:esp8266:nodemcuv2 .
     % arduino-cli compile -b arduino:avr:uno .

     % arduino-cli upload -v -p /dev/ttyUSB0 -b esp8266:esp8266:nodemcuv2 .
     % arduino-cli upload -v -p /dev/ttyUSB0 -b arduino:avr:uno .

   Examples

     [ serial CLI ]

     test LED,
       hi 12
       lo 12

     test light sensor,
       hi 5
       aread 0
       lo 5

     test HC-SR04,
       hi 4
       hcsr04 2 3
       lo 4

     test DHT22,
       hi 7
       dht22 6
       lo 7

     test BMP180,       (on nodemcu, SCL is D1, SDA is D2)
       bmp180

     test build-in LED on NodeMCU
       lo 16
       hi 16

     test wifi client
       wifi ssid superman
       wifi pw changeme
       wifi connect

     test wifi boot up config
       fs write /wifi.ssid superman
       fs write /wifi.pw changeme
       fs write /udp.port 8266

     [ REST ]

     % curl 'http://esp32.example.com/v1?cmd=...'

     eg,

     % curl 'http://esp32.example.com/v1?cmd=wifi+status'

   Bugs

     - The "wifi {ssid|pw} <value>" command does not support arguments with
       spaces.
     - The "fs write <filename> <content>" command does not support content
       with spaces.
     - All filenames must begin with '/'.

   Notes

     - pin numbering goes by GPIO number, thus "hi 5" means GPIO pin 5, not
       the physical pin number.

     - LCD implementation reference :
       https://create.arduino.cc/projecthub/arduino_uno_guy/i2c-liquid-crystal-displays-5b806c

     - ESP32 hardware API reference
       https://github.com/espressif/arduino-esp32
*/

#include <Wire.h>
#include "LiquidCrystal_I2C.h"

/* ====== STUFF COMMON BETWEEN ESP32 and ESP8266 ====== */

#if defined ARDUINO_ESP8266_NODEMCU || ARDUINO_ESP32_DEV
  #include <FS.h>
  #include <WiFiUdp.h>

  #define MAX_SSID_LEN 32
  #define MAX_PASSWD_LEN 64             // maximum wifi password length
  #define MAX_WIFI_TIMEOUT 60           // wifi connect timeout (secs)
  #define WEB_PORT 80

  #define WIFI_SSID_FILE "/wifi.ssid"
  #define WIFI_PW_FILE "/wifi.pw"
  #define UDP_PORT_FILE "/udp.port"

  char cfg_wifi_ssid[MAX_SSID_LEN + 1] ;
  char cfg_wifi_pw[MAX_PASSWD_LEN + 1] ;
  int cfg_udp_port=0 ;
  WiFiUDP Udp ;                         // our UDP server socket
#endif

/* ====== STUFF SPECIFIC TO ESP32 and ESP8266 ====== */

#ifdef ARDUINO_ESP32_DEV
  #define LED_BUILTIN 2
  #define BLINK_ON HIGH
  #define BLINK_OFF LOW

  #include <SPIFFS.h>
  #include <WebServer.h>
  WebServer Webs(WEB_PORT) ;     // our built-in webserver
#endif

#ifdef ARDUINO_ESP8266_NODEMCU
  #define BLINK_ON LOW
  #define BLINK_OFF HIGH

  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  ESP8266WebServer Webs(WEB_PORT) ;     // our built-in webserver
#endif

/* ====== ALL OTHER GENERAL STUFF ====== */

#define DEF_BAUD 9600
#define SERIAL_TIMEOUT 1000     // serial timeout in milliseconds
#define BLINK_FREQ 5000         // blink to indicate we're alive (ms)
#define MAX_TOKENS 10
#define BUF_SIZE 80

#if defined ARDUINO_ESP8266_NODEMCU || ARDUINO_ESP32_DEV
  #define REPLY_SIZE 512
#else
  #define REPLY_SIZE 192
#endif

/* LCD definitions */

#define LCD_ADDR 0x27
#define LCD_WIDTH 16
#define LCD_ROWS 2

/* global variables */

char line[BUF_SIZE] ;           // general purpose string buffer
char input_buf[BUF_SIZE+1] ;    // bytes received on serial port
char reply_buf[REPLY_SIZE+1] ;  // accumulate our reply message here
int input_pos ;                 // index of bytes received on serial port
char *tokens[MAX_TOKENS+1] ;    // max command parameters we'll parse
unsigned long next_blink=BLINK_FREQ ; // "wall clock" time for next blink

LiquidCrystal_I2C lcd (LCD_ADDR, LCD_WIDTH, LCD_ROWS) ;

/* function prototypes */

void f_action (char **tokens) ;

/* ------------------------------------------------------------------------- */

/*
   Reads a 16-bit (signed) value from "device" at "address", saving the 16-bit
   value in "result". On success, it returns 1, otherwise 0.
*/

int f_i2c_readShort (int device, unsigned char addr, short *result)
{
  unsigned char data[2] ;
  Wire.beginTransmission (device) ;
  data[0] = addr ;
  data[1] = 0 ;
  if ((Wire.write (data[0]) != 1) ||
      (Wire.endTransmission() != 0) ||
      (Wire.requestFrom (device, 2) != 2))
    return (0) ;

  data[0] = Wire.read () ;
  data[1] = Wire.read () ;
  int v = data[0] ;
  v = (v << 8) + data[1] ;
  if (v > 32767)
    v = v - 65536 ;
  *result = (short) v ;
  return (1) ;
}

/*
   Reads a 16-bit (unsigned) value from "device" at "address", saving the
   value in "result". On success, it returns 1, otherwise 0.
*/

int f_i2c_readUShort (int device, unsigned char addr, unsigned short *result)
{
  unsigned char data[2] ;
  Wire.beginTransmission (device) ;
  data[0] = addr ;
  data[1] = 0 ;
  if ((Wire.write (data[0]) != 1) ||
      (Wire.endTransmission() != 0) ||
      (Wire.requestFrom (device, 2) != 2))
    return (0) ;

  data[0] = Wire.read () ;
  data[1] = Wire.read () ;
  int v = data[0] ;
  v = (v << 8) + data[1] ;
  *result = (unsigned short) v ;
  return (1) ;
}

/* ------------------------------------------------------------------------- */

/*
   Polls a BMP180 on the I2C bus. This should return temperature and pressure
   data which is then written into "temperature" and "pressure". On success
   we return 1, otherwise 0.
*/

int f_bmp180 (float *temperature, float *pressure)
{
  #define BMP180_ADDR 0x77      /* this came from the BMP180 data sheet */
  #define BMP180_MODE 3         /* the pressure oversampling */

  /* read the 11x 16-bit registers on the BMP180 for calibration data */

  short ac1, ac2, ac3, b1, b2, mb, mc, md ;
  unsigned short ac4, ac5, ac6 ;

  if ((f_i2c_readShort (BMP180_ADDR, 0xAA, &ac1) == 0) ||
      (f_i2c_readShort (BMP180_ADDR, 0xAC, &ac2) == 0) ||
      (f_i2c_readShort (BMP180_ADDR, 0xAE, &ac3) == 0) ||
      (f_i2c_readUShort (BMP180_ADDR, 0xB0, &ac4) == 0) ||
      (f_i2c_readUShort (BMP180_ADDR, 0xB2, &ac5) == 0) ||
      (f_i2c_readUShort (BMP180_ADDR, 0xB4, &ac6) == 0) ||
      (f_i2c_readShort (BMP180_ADDR, 0xB6, &b1) == 0) ||
      (f_i2c_readShort (BMP180_ADDR, 0xB8, &b2) == 0) ||
      (f_i2c_readShort (BMP180_ADDR, 0xBA, &mb) == 0) ||
      (f_i2c_readShort (BMP180_ADDR, 0xBC, &mc) == 0) ||
      (f_i2c_readShort (BMP180_ADDR, 0xBE, &md) == 0))
  {
    strcat (reply_buf, "FAULT: Cannot read data from BMP180.\r\n") ;
    return (0) ;
  }

  sprintf (line, "ac1: %d\r\nac2: %d\r\nac3: %d\r\n", ac1, ac2, ac3) ;
  strcat (reply_buf, line) ;
  sprintf (line, "ac4: %d\r\nac5: %d\r\nac6: %d\r\n", ac4, ac5, ac6) ;
  strcat (reply_buf, line) ;
  sprintf (line, "b1: %d\r\nb2: %d\r\nmb: %d\r\nmc: %d\r\n", b2, mb, mc) ;
  strcat (reply_buf, line) ;
  sprintf (line, "md: %d\r\n", md) ;
  strcat (reply_buf, line) ;

  /*
     read the raw temperature by writing 0x2E to address 0xF4, the result is
     2 bytes at 0xF6.
  */

  Wire.beginTransmission (BMP180_ADDR) ;
  Wire.write ((byte)(0xF4)) ;
  Wire.write ((byte)(0x2E)) ;
  Wire.endTransmission () ;
  delay (5) ;                           // datasheet says wait 4.5 ms
  short raw_t=0 ;
  f_i2c_readShort (BMP180_ADDR, 0xF6, &raw_t) ;

  sprintf (line, "raw_t: %d\r\n", raw_t) ;
  strcat (reply_buf, line) ;

  /* now calculate the true temperature */

  long x1 = (raw_t - ac6) * ac5 >> 15 ;
  long x2 = (mc << 11) / (x1 + md) ;
  long b5 = x1 + x2 ;
  long t = (b5 + 8) >> 4 ;
  *temperature = float(t) / 10.0 ;

  /*
     read raw pressure by writing "0x34+(mode<<6)" to address 0xF4, the
     result is 3 bytes (MSB, LSB, XLSB) starting at 0xF6.
  */

  long v = 0x34 + (BMP180_MODE << 6) ;
  Wire.beginTransmission (BMP180_ADDR) ;
  Wire.write ((byte)(0xF4)) ;
  Wire.write ((byte)(v)) ;
  Wire.endTransmission () ;
  delay (26) ;                          // datasheet says 25.5 ms for mode 3.

  Wire.beginTransmission (BMP180_ADDR) ;
  Wire.write ((byte)(0xF6)) ;
  Wire.endTransmission () ;
  Wire.requestFrom (BMP180_ADDR, 3) ;
  long msb = Wire.read () ;
  long lsb = Wire.read () ;
  long xlsb = Wire.read () ;
  long raw_p = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - BMP180_MODE) ;

  sprintf (line, "raw_p: %d\r\n", raw_p) ;
  strcat (reply_buf, line) ;

  long b6 = b5 - 4000 ;
  x1 = (b2 * (b6 * b6) >> 12) >> 11 ;
  x2 = (ac2 * b6) >> 11 ;
  long x3 = x1 + x2 ;
  long b3 = (((ac1 * 4 + x3) << BMP180_MODE) + 2) / 4 ;
  x1 = ac3 * b6 >> 13 ;
  x2 = ((b1 * (b6 * b6)) >> 12) >> 16 ;
  x3 = ((x1 + x2) + 2) >> 2 ;
  unsigned long b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15 ;
  unsigned long b7 = (raw_p - b3) * (50000 >> BMP180_MODE) ;
  long p ;
  if (b7 < (unsigned long) 0x80000000)
    p = (b7 * 2) / b4 ;
  else
    p = (b7 / b4) * 2 ;
  x1 = (p >> 8) * (p >> 8) ;
  x1 = (x1 * 3038) >> 16 ;
  x2 = (-7357 * p) >> 16 ;
  p = p + ((x1 + x2 + 3791) >> 4) ;
  *pressure = float(p) / 100.0 ;

  return (1) ;
}

/*
   Polls a DHT-22 and writes the current temperature and humidity in the
   supplied buffers. On success, it returns 1, otherwise 0.
*/

int f_dht22 (int dataPin, float *temperature, float *humidity)
{
  #define DHT22_TIMEOUT_USEC 500

  int cycles[40] ;
  unsigned char data[5] ;

  /* send the trigger to begin a poll cycle */

  pinMode (dataPin, OUTPUT) ;
  digitalWrite (dataPin, LOW) ;         // sensor reset
  delayMicroseconds (1200) ;
  digitalWrite (dataPin, HIGH) ;        // sensor trigger
  delayMicroseconds (20) ;
  pinMode (dataPin, INPUT) ;

  /* expect DHT22 to respond with a low and then a high */

  if (pulseIn (dataPin, HIGH) == 0)
  {
    strcat (reply_buf, "FAULT: f_dht22() no ACK, aborting.\r\n") ;
    return (0) ;
  }

  /* now read 40 bits of data, store pulse timings in an array */

  int i ;
  for (i=0 ; i<40 ; i++)
    cycles[i] = pulseIn (dataPin, HIGH, DHT22_TIMEOUT_USEC) ;

  /* convert pulse timings timings into humidity/temperature/checksum */

  memset (data, 0, 5) ;
  for (i=0 ; i<40 ; i++)
  {
    data[i/8] <<= 1 ;           // left shift bits, right most bit will be 0
    if (cycles[i] > 50)
      data[i/8] |= 1 ;          // set right most bit to 1
  }

  /* validate checksum */

  unsigned char c = data[0] + data[1] + data[2] + data[3] ;
  if ((c & 0xff) != data[4])
  {
    strcat (reply_buf, "FAULT: f_dht22() checksum failed.\r\n") ;
    return (0) ;
  }

  *humidity = float (((int) data[0] << 8 ) | data[1]) / 10.0 ;
  *temperature = float ((((int) data[2] & 0x7f ) << 8 ) | data[3]) / 10.0 ;
  return (1) ;
}

/*
   Returns the distance (in cm) measured by an HC-SR04 ultrasonic range sensor
   or -1.0 if it was unable to take a reading.
*/

float f_hcsr04 (int trigPin, int echoPin)
{
  #define HCSR04_TIMEOUT_USEC 60000

  pinMode (trigPin, OUTPUT) ;
  pinMode (echoPin, INPUT) ;

  /* set trigger pin low, then stay high for 10 usec */

  digitalWrite (trigPin, LOW) ;
  delayMicroseconds (1000) ;
  digitalWrite (trigPin, HIGH) ;
  delayMicroseconds (10) ;
  digitalWrite (trigPin, LOW) ;

  unsigned long echoUsecs = pulseIn (echoPin, HIGH, HCSR04_TIMEOUT_USEC) ;
  if (echoUsecs == 0)
  {
    strcat (reply_buf, "FAULT: f_hcsr04() no response.\r\n") ;
    return (-1.0) ;
  }
  else
    return (float(echoUsecs) / 58.0) ; // convert time to centimeters
}

/*
   Control an I2C LCD via a LCM1602 module.
*/

void f_lcd (char **tokens)
{
  if (tokens[1] == NULL)
  {
    strcat (reply_buf, "FAULT: No arguments.\r\n") ;
    return ;
  }

  if (strcmp(tokens[1], "init") == 0)                           // init
  {
    lcd.init () ;
    strcat (reply_buf, "LCD initialized.\r\n") ;
  }
  else
  if (strcmp(tokens[1], "backlight") == 0)                      // backlight
  {
    if (strcmp(tokens[2], "on") == 0)
    {
      lcd.backlight () ;
      strcat (reply_buf, "LCD backlight is on.\r\n") ;
    }
    else
    if (strcmp(tokens[2], "off") == 0)
    {
      lcd.noBacklight () ;
      strcat (reply_buf, "LCD backlight is off.\r\n") ;
    }
    else
    {
      strcat (reply_buf, "FAULT: Invalid argument.\r\n") ;
    }
  }
  else
  if (strcmp(tokens[1], "clear") == 0)                          // clear
  {
    lcd.clear () ;
    strcat (reply_buf, "LCD cleared.\r\n") ;
  }
  else
  if ((strcmp(tokens[1], "print") == 0) &&                      // print
      (tokens[2] != NULL) &&
      (tokens[3] != NULL) &&
      (tokens[4] != NULL))
  {
    int row = atoi(tokens[2]) ;
    int col = atoi(tokens[3]) ;

    line[0] = 0 ;
    for (int idx=4 ; idx < MAX_TOKENS ; idx++)
    {
      if (tokens[idx] == NULL)
        break ;
      if (strlen(line) > 0)
        strcat (line, " ") ;
      strcat (line, tokens[idx]) ;
    }

    lcd.setCursor (col, row) ;
    lcd.print (line) ;
    sprintf (line, "LCD write at row:%d col:%d.\r\n", row, col) ;
    strcat (reply_buf, line) ;
  }
  else
  {
    strcat (reply_buf, "FAULT: Invalid argument.\r\n") ;
  }
}

/* ------------------------------------------------------------------------- */
/* Features specific to both ESP8266 and ESP32                               */
/* ------------------------------------------------------------------------- */

#if defined ARDUINO_ESP8266_NODEMCU || ARDUINO_ESP32_DEV

int f_blink (int num)
{
  #define BLINK_DURATION 10     // just long enough to be noticed
  #define PAUSE_DURATION 300    // long enough for a human to count blinks

  int duration = 0 ;
  pinMode (LED_BUILTIN, OUTPUT) ;
  for (int i=0 ; i < num ; i++)
  {
    digitalWrite (LED_BUILTIN, BLINK_ON) ;
    delay (BLINK_DURATION) ;
    duration = duration + BLINK_DURATION ;
    digitalWrite (LED_BUILTIN, BLINK_OFF) ;
    if (i < num-1)
      delay (PAUSE_DURATION) ;
    duration = duration + PAUSE_DURATION ;
  }
  return (duration) ;
}

void f_fs (char **tokens)
{
  char msg[BUF_SIZE] ;

  if (strcmp(tokens[1], "info") == 0)                           // info
  {
    #ifdef ARDUINO_ESP8266_NODEMCU

    FSInfo fi ;
    if (SPIFFS.info (fi))
    {
      sprintf (line, "totalBytes: %d\r\nusedBytes: %d\r\nblockSize: %d\r\n",
               fi.totalBytes, fi.usedBytes, fi.blockSize) ;
      strcat (reply_buf, line) ;
      sprintf (line, "pageSize: %d\r\nmaxOpenFiles: %d\r\n",
               fi.pageSize, fi.maxOpenFiles) ;
      strcat (reply_buf, line) ;
      sprintf (line, "maxPathLength: %d\r\n", fi.maxPathLength) ;
    }
    else
    {
      strcat (reply_buf, "FAULT: Cannot obtain fs info.\r\n") ;
    }

    #endif
    #ifdef ARDUINO_ESP32_DEV

      sprintf (line, "totalBytes: %d\r\nusedBytes: %d\r\n",
               SPIFFS.totalBytes(), SPIFFS.usedBytes()) ;
      strcat (reply_buf, line) ;

    #endif
  }
  else
  if (strcmp(tokens[1], "format") == 0)                         // format
  {
    if (SPIFFS.format())
    {
      strcat (reply_buf, "Success.\r\n") ;
    }
    else
    {
      strcat (reply_buf, "FAULT: Formatting failed.\r\n") ;
    }
  }
  else
  if (strcmp(tokens[1], "ls") == 0)                             // ls
  {
    #ifdef ARDUINO_ESP8266_NODEMCU

    Dir dir = SPIFFS.openDir ("/") ;
    while (dir.next())
    {
      String s = dir.fileName () ;
      int len = s.length()+1 ;
      char filename[len+1] ;
      s.toCharArray (filename, len) ;
      File f = SPIFFS.open (s, "r") ;
      sprintf (line, "%-8d %s\r\n", f.size(), filename) ;
      f.close () ;
      strcat (reply_buf, line) ;
    }

    #endif
    #ifdef ARDUINO_ESP32_DEV

    File root = SPIFFS.open ("/") ;
    File f = root.openNextFile () ;
    while (f)
    {
      sprintf (line, "%-8d %s\r\n", f.size(), f.name()) ;
      strcat (reply_buf, line) ;
      f = root.openNextFile () ;
    }
    root.close () ;

    #endif
  }
  else
  if ((strcmp(tokens[1], "write") == 0) &&                      // write
      (tokens[2] != NULL) && (tokens[3] != NULL))
  {
    char *filename = tokens[2] ;
    char *content = tokens[3] ;

    /* force filenames to begin with '/' */

    if ((filename[0] != '/') || (strlen(filename) == 1))
    {
      strcat (reply_buf, "FAULT: Invalid filename.\r\n") ;
      return ;
    }

    File f = SPIFFS.open (filename, "w") ;
    if (f)
    {
      int amt = f.print (content) ;
      f.close () ;
      sprintf (msg, "Wrote %d bytes to '%s'.\r\n", amt, filename) ;
      strcat (reply_buf, msg) ;
    }
    else
    {
      sprintf (msg, "FAULT: Cannot write to '%s'.\r\n", filename) ;
      strcat (reply_buf, msg) ;
    }
  }
  else
  if ((strcmp(tokens[1], "read") == 0) && (tokens[2] != NULL))  // read
  {
    char *filename = tokens[2] ;
    File f = SPIFFS.open (filename, "r") ;
    int amt = f.readBytes (msg, BUF_SIZE-1) ;
    f.close () ;
    if (amt > 0)
    {
      msg[amt] = 0 ;
      strcat (reply_buf, msg) ;
      strcat (reply_buf, "\r\n") ;
    }
    else
    {
      strcat (reply_buf, "FAULT: Cannot read file.\r\n") ;
    }
  }
  else
  if ((strcmp(tokens[1], "rm") == 0) &&                         // rm
      (tokens[2] != NULL))
  {
    char *filename = tokens[2] ;
    if (SPIFFS.remove(filename))
      strcat (reply_buf, "File removed.\r\n") ;
    else
      strcat (reply_buf, "FAULT: Cannot remove file.\r\n") ;
  }
  else
  if ((strcmp(tokens[1], "rename") == 0) &&                     // rename
      (tokens[2] != NULL) && (tokens[3] != NULL))
  {
    char *old_name = tokens[2] ;
    char *new_name = tokens[3] ;

    if ((new_name[0] != '/') || (strlen(new_name) == 1))
    {
      strcat (reply_buf, "FAULT: Invalid filename.\r\n") ;
      return ;
    }
    if (SPIFFS.rename(old_name, new_name))
      strcat (reply_buf, "File renamed.\r\n") ;
    else
      strcat (reply_buf, "FAULT: Cannot rename file.\r\n") ;
  }
  else
  {
    strcat (reply_buf, "FAULT: Invalid argument.\r\n") ;
  }
}

void f_wifi (char **tokens)
{
  if (strcmp(tokens[1], "scan") == 0)                           // scan
  {
    int n = WiFi.scanNetworks() ;
    sprintf (line, "Found %d wifi networks.\r\n", n) ;
    strcat (reply_buf, line) ;
    for (int i=0 ; i<n ; i++)
    {
      char ssid[MAX_SSID_LEN+1] ;
      WiFi.SSID(i).toCharArray (ssid, MAX_SSID_LEN) ;
      sprintf (line, "%2d. ch %d, %d dBm [%s]\r\n",
               i+1, WiFi.channel(i), WiFi.RSSI(i), ssid) ;
      if (strlen(reply_buf) + strlen(line) < REPLY_SIZE)
        strcat (reply_buf, line) ;
    }
  }
  else
  if (strcmp(tokens[1], "status") == 0)                         // status
  {
    sprintf (line, "cfg_wifi_ssid: %s\r\n", cfg_wifi_ssid) ;
    strcat (reply_buf, line) ;
    if (strlen(cfg_wifi_pw) > 0)
      strcat (reply_buf, "cfg_wifi_pw: (set)\r\n") ;
    else
      strcat (reply_buf, "cfg_wifi_pw: (unset)\r\n") ;

    int status = WiFi.status() ;
    strcat (reply_buf, "status: ") ;
    switch (status)
    {
      case WL_CONNECTED:
        strcat (reply_buf, "WL_CONNECTED\r\n") ; break ;
      case WL_NO_SHIELD:
        strcat (reply_buf, "WL_NO_SHIELD\r\n") ; break ;
      case WL_IDLE_STATUS:
        strcat (reply_buf, "WL_IDLE_STATUS\r\n") ; break ;
      case WL_NO_SSID_AVAIL:
        strcat (reply_buf, "WL_NO_SSID_AVAIL\r\n") ; break ;
      case WL_SCAN_COMPLETED:
        strcat (reply_buf, "WL_SCAN_COMPLETED\r\n") ; break ;
      case WL_CONNECT_FAILED:
        strcat (reply_buf, "WL_CONNECT_FAILED\r\n") ; break ;
      case WL_CONNECTION_LOST:
        strcat (reply_buf, "WL_CONNECTION_LOST\r\n") ; break ;
      case WL_DISCONNECTED:
        strcat (reply_buf, "WL_DISCONNECTED\r\n") ; break ;
      default:
        strcat (reply_buf, "UNKNOWN\r\n") ; break ;
    }

    sprintf (line, "rssi: %d dBm\r\n", WiFi.RSSI()) ;
    strcat (reply_buf, line) ;
    sprintf (line, "udp_port: %d\r\n", cfg_udp_port) ;
    strcat (reply_buf, line) ;

    unsigned char mac[6] ;
    WiFi.macAddress(mac) ;
    sprintf (line, "wifi_mac: %x:%x:%x:%x:%x:%x\r\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]) ;
    strcat (reply_buf, line) ;
    sprintf (line, "wifi_ip: %s/%s\r\n",
             WiFi.localIP().toString().c_str(),
             WiFi.subnetMask().toString().c_str()) ;
    strcat (reply_buf, line) ;
  }
  else
  if (strcmp(tokens[1], "disconnect") == 0)                     // disconnect
  {
    WiFi.disconnect() ;
  }
  else
  if ((strcmp(tokens[1], "ssid") == 0) &&                       // set ssid
      (tokens[2] != NULL))
  {
    strlcpy (cfg_wifi_ssid, tokens[2], MAX_SSID_LEN) ;
  }
  else
  if (strcmp(tokens[1], "pw") == 0)                            // set pw
  {
    if (tokens[2] != NULL)
      strlcpy (cfg_wifi_pw, tokens[2], MAX_PASSWD_LEN) ;
    else
      cfg_wifi_pw[0] = 0 ;
  }
  else
  if (strcmp(tokens[1], "connect") == 0)                        // connect
  {
    WiFi.begin (cfg_wifi_ssid, cfg_wifi_pw) ;
    for (int retry=0 ; retry < MAX_WIFI_TIMEOUT ; retry++)
    {
      int status = WiFi.status() ;
      if (status == WL_CONNECTED)
      {
        WiFi.setAutoReconnect (true) ;
        sprintf (line, "Connected in %d seconds.\r\n", retry) ;
        strcat (reply_buf, line) ;
        return ;
      }
      else
      if (status == WL_DISCONNECTED)
        delay (1000) ;
      else
      {
        switch (status)
        {
          case WL_NO_SHIELD:
            strcat (reply_buf, "WL_NO_SHIELD\r\n") ; return ;
          case WL_IDLE_STATUS:
            strcat (reply_buf, "WL_IDLE_STATUS\r\n") ; return ;
          case WL_NO_SSID_AVAIL:
            strcat (reply_buf, "WL_NO_SSID_AVAIL\r\n") ; return ;
          case WL_SCAN_COMPLETED:
            strcat (reply_buf, "WL_SCAN_COMPLETE\r\n") ; return ;
          case WL_CONNECT_FAILED:
            strcat (reply_buf, "WL_CONNECT_FAILED\r\n") ; return ;
          case WL_CONNECTION_LOST:
            strcat (reply_buf, "WL_CONNECTION_LOST\r\n") ; return ;
          default:
            strcat (reply_buf, "UNKNOWN\r\n") ; return ;
        }
      }
    }
    strcat (reply_buf, "FAULT: Connection attempt timed out.\r\n") ;
  }
  else
  {
    strcat (reply_buf, "FAULT: Invalid argument.\r\n") ;
  }
}

/* callback functions for web server */

void f_handleWeb ()                             // for uri "/"
{
  Webs.send (200, "text/plain", "OK\n") ;
}

void f_v1api ()                                 // for uri "/v1"
{
  if ((Webs.args() != 1) ||                     // expect exactly 1 arg
      (Webs.argName(0) != "cmd"))
  {
    Webs.send (504, "text/plain", "Invalid usage\r\n") ;
    return ;
  }

  strncpy (input_buf, Webs.arg(0).c_str(), BUF_SIZE) ;
  int idx = 0 ;
  char *p = strtok (input_buf, " ") ;
  while ((p) && (idx < MAX_TOKENS))
  {
    tokens[idx] = p ;
    idx++ ;
    p = strtok (NULL, " ") ;
  }
  tokens[idx] = NULL ;
  reply_buf[0] = 0 ;
  if (idx > 0)
    f_action(tokens) ;
  if (strlen(reply_buf) > 0)
    Webs.send (200, "text/plain", reply_buf) ;
  else
    Webs.send (504, "text/plain", "No response\r\n") ;
}

void f_handleWebMetrics ()                      // for uri "/metrics"
{
  sprintf (line, "node_uptime_secs %ld\n", millis() / 1000) ;
  Webs.send (200, "text/plain", line) ;
}

#endif

/* ------------------------------------------------------------------------- */
/* Features specific to ESP32                                                */
/* ------------------------------------------------------------------------- */

#ifdef ARDUINO_ESP32_DEV

void f_esp32 (char **tokens)
{
  char msg[BUF_SIZE] ;

  if (strcmp(tokens[1], "hall") == 0)                           // hall
  {
    sprintf (msg, "%d\r\n", hallRead()) ;
    strcat (reply_buf, msg) ;
  }
  else
  {
    strcat (reply_buf, "FAULT: Invalid argument.\r\n") ;
  }
}

#endif

/* ------------------------------------------------------------------------- */

void f_action (char **tokens)
{
  if ((strcmp(tokens[0], "?") == 0) || (strcmp(tokens[0], "help") == 0))
  {
    strcat (reply_buf,
            "hi <pin 0-13>\r\n"
            "lo <pin 0-13>\r\n"
            "aread <pin 0-5> - analog read\r\n"
            "bmp180\r\n"
            "dht22 <dataPin> - DHT-22 temperature/humidity sensor\r\n"
            "hcsr04 <trigPin> <echoPin> - HC-SR04 ultrasonic ranger\r\n"
            "lcd backlight <on/off>\r\n"
            "lcd clear\r\n"
            "lcd init\r\n"
            "lcd print <row> <col> <message...>\r\n"
            "uptime\r\n"
            "version\r\n") ;

    #if defined ARDUINO_ESP8266_NODEMCU || ARDUINO_ESP32_DEV
      strcat (reply_buf,
              "fs format\r\n"
              "fs info\r\n"
              "fs ls\r\n"
              "fs read <filename>\r\n"
              "fs rm <filename>\r\n"
              "fs rename <old> <new>\r\n"
              "fs write <filename> <content>\r\n"
              "restart\r\n"
              "wifi connect\r\n"
              "wifi scan\r\n"
              "wifi status\r\n"
              "wifi ssid <ssid>\r\n"
              "wifi pw <password>\r\n"
              "wifi disconnect\r\n") ;
    #endif

    #ifdef ARDUINO_ESP32_DEV
      strcat (reply_buf,
              "esp32 hall\r\n") ;
    #endif
  }
  else
  if ((strcmp(tokens[0], "hi") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, OUTPUT) ;
    digitalWrite (pin, HIGH) ;
    sprintf (line, "pin:%d HIGH\r\n", pin) ;
    strcat (reply_buf, line) ;
  }
  else
  if ((strcmp(tokens[0], "lo") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, OUTPUT) ;
    digitalWrite (pin, LOW) ;
    sprintf (line, "pin:%d LOW\r\n", pin) ;
    strcat (reply_buf, line) ;
  }
  else
  if ((strcmp(tokens[0], "aread") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    int val = analogRead (pin) ;
    sprintf (line, "analogRead pin:%d - %d\r\n", pin, val) ;
    strcat (reply_buf, line) ;
  }
  else
  if (strcmp(tokens[0], "bmp180") == 0)
  {
    float t=0.0, p=0.0 ;
    if (f_bmp180 (&t, &p))
    {
      sprintf (line, "bmp180 - temperature:%d.%02d pressure:%d.%02d\r\n",
               int(t), (int)(t*100)%100, int(p), (int)(p*100)%100) ;
      strcat (reply_buf, line) ;
    }
  }
  else
  if ((strcmp(tokens[0], "dht22") == 0) && (tokens[1] != NULL))
  {
    float t=0.0, h=0.0 ;
    if (f_dht22 (atoi(tokens[1]), &t, &h))
    {
      sprintf (line, "dht22 - temperature:%d.%02d humidity:%d.%02d\r\n",
               int(t), (int)(t*100)%100, int(h), (int)(h*100)%100) ;
      strcat (reply_buf, line) ;
    }
  }
  else
  if ((strcmp(tokens[0], "hcsr04") == 0) && 
      (tokens[1] != NULL) && (tokens[2] != NULL))
  {
    float f = f_hcsr04 (atoi(tokens[1]), atoi(tokens[2])) ;
    if (f > 0.0)
    {
      sprintf (line, "hcsr04 - %d.%02d cm\r\n", int(f), (int)(f*100)%100) ;
      strcat (reply_buf, line) ;
    }
  }
  else
  if (strcmp(tokens[0], "uptime") == 0)
  {
    unsigned long now = millis() / 1000 ;
    sprintf (line, "uptime - %ld secs\r\n", now) ;
    strcat (reply_buf, line) ;
  }
  else
  if (strcmp(tokens[0], "version") == 0)
  {
    sprintf (line, "Built: %s, %s\r\n", __DATE__, __TIME__) ;
    strcat (reply_buf, line) ;
  }
  else
  if (strcmp(tokens[0], "lcd") == 0)
  {
    f_lcd (tokens) ;
  }

  #if defined ARDUINO_ESP8266_NODEMCU || ARDUINO_ESP32_DEV
  else
  if ((strcmp(tokens[0], "fs") == 0) && (tokens[1] != NULL))
  {
    f_fs (tokens) ;
  }
  else
  if ((strcmp(tokens[0], "wifi") == 0) && (tokens[1] != NULL))
  {
    f_wifi (tokens) ;
  }
  else
  if (strcmp(tokens[0], "restart") == 0)
  {
    Serial.println ("Restarting.") ;
    delay (1000) ;
    ESP.restart () ;
  }
  #endif
  #ifdef ARDUINO_ESP32_DEV
  else
  if ((strcmp(tokens[0], "esp32") == 0) && (tokens[1] != NULL))
  {
    f_esp32 (tokens) ;
  }
  #endif
  else
  {
    strcat (reply_buf, "FAULT: Enter 'help' for commands.\r\n") ;
  }
}

/* ------------------------------------------------------------------------- */

void setup ()
{
  Wire.begin () ;
  Serial.begin (DEF_BAUD) ;
  Serial.setTimeout (SERIAL_TIMEOUT) ;
  Serial.println ("\nNOTICE: System boot.") ;
  input_buf[0] = 0 ;
  input_pos = 0 ;

  #if defined ARDUINO_ESP8266_NODEMCU || ARDUINO_ESP32_DEV

    cfg_wifi_ssid[0] = 0 ;
    cfg_wifi_pw[0] = 0 ;
    pinMode (LED_BUILTIN, OUTPUT) ;

    /*
       if wifi ssid and password are available, try connect to wifi now.
       if udp server port is defined, configure it now.
    */

    if (SPIFFS.begin())
    {
      Serial.println ("NOTICE: Checking built-in configuration.") ;
      File f_ssid = SPIFFS.open (WIFI_SSID_FILE, "r") ;
      File f_pw = SPIFFS.open (WIFI_PW_FILE, "r") ;
      if ((f_ssid) && (f_pw) &&
          (f_ssid.size() > 0) && (f_pw.size() > 0) &&
          (f_ssid.size() <= MAX_SSID_LEN) && (f_pw.size() <= MAX_PASSWD_LEN))
      {
        int s_amt = f_ssid.readBytes (cfg_wifi_ssid, MAX_SSID_LEN) ;
        if (s_amt > 0)
          cfg_wifi_ssid[s_amt] = 0 ;
        int p_amt = f_pw.readBytes (cfg_wifi_pw, MAX_PASSWD_LEN) ;
        if (p_amt > 0)
          cfg_wifi_pw[p_amt] = 0 ;
        if ((s_amt > 0) && (p_amt > 0))
        {
          sprintf (line, "NOTICE: Wifi config loaded for %s.", cfg_wifi_ssid) ;
          Serial.println (line) ;
          WiFi.begin (cfg_wifi_ssid, cfg_wifi_pw) ;
        }
      }
      if (f_ssid)
        f_ssid.close () ;
      if (f_pw)
        f_pw.close () ;

      File f_udp = SPIFFS.open (UDP_PORT_FILE, "r") ;
      if ((f_udp) && (f_udp.size() > 0))
      {
        char msg[BUF_SIZE] ;
        int amt = f_udp.readBytes (msg, 6) ;
        if (amt > 0)
        {
          msg[amt] = 0 ;
          cfg_udp_port = atoi (msg) ;
          if ((cfg_udp_port < 65535) && (cfg_udp_port > 0))
          {
            sprintf (line, "NOTICE: UDP server on port %d.", cfg_udp_port) ;
            Serial.println (line) ;
            Udp.begin (cfg_udp_port) ;
          }
          else
          {
            sprintf (line, "WARNING: Invalid udp port '%s'.", msg) ;
            Serial.println (line) ;
            cfg_udp_port = 0 ;
          }
        }
      }
      if (f_udp)
        f_udp.close () ;
    }

    Webs.on ("/", f_handleWeb) ;
    Webs.on ("/v1", f_v1api) ;
    Webs.on ("/metrics", f_handleWebMetrics) ;
    Webs.begin () ;
    sprintf (line, "NOTICE: Web server started on port %d.\r\n", WEB_PORT) ;
    Serial.print (line) ;

    digitalWrite (LED_BUILTIN, LOW) ;           // blink to indicate boot.
    delay (1000) ;
    digitalWrite (LED_BUILTIN, HIGH) ;
  #endif

  Serial.println ("NOTICE: Ready.") ;
}

void loop ()
{
  /*
     see if serial data arrived, wait for '\r' (minicom sends this when user
     presses ENTER)
  */

  while ((Serial.available() > 0) && (input_pos < BUF_SIZE))
  {
    char c = Serial.read () ;
    input_buf[input_pos] = c ;
    if (input_buf[input_pos] == '\r')
    {
      input_buf[input_pos + 1] = 0 ;
      break ;
    }
    input_pos++ ;
  }

  if (input_buf[input_pos] == '\r')
  {
    input_buf[input_pos] = 0 ;
    int idx = 0 ;
    char *p = strtok (input_buf, " ") ;
    while ((p) && (idx < MAX_TOKENS))
    {
      tokens[idx] = p ;
      idx++ ;
      p = strtok (NULL, " ") ;
    }
    tokens[idx] = NULL ;

    /*
       now that we've tokenized "input_buf", have f_action() do something,
       which places the response in "reply_buf". Always print an "OK" on a
       new line to indicate that the previous command has completed.
    */

    reply_buf[0] = 0 ;
    if (tokens[0] != NULL)
      f_action (tokens) ;
    if (strlen(reply_buf) > 0)
    {
      Serial.print (reply_buf) ;
      if (reply_buf[strlen(reply_buf)-1] != '\n')
        Serial.print ("\r\n") ;                         // add CRNL if needed
    }
    Serial.println ("OK") ;

    input_buf[0] = 0 ;
    input_pos = 0 ;
  }

  #if defined ARDUINO_ESP8266_NODEMCU || ARDUINO_ESP32_DEV

  int pktsize = Udp.parsePacket () ;
  if (pktsize)
  {
    /* if a UDP packet arrived, parse the command and send a response */

    int amt = Udp.read (input_buf, BUF_SIZE) ;
    input_buf[amt] = 0 ;

    int idx = 0 ;
    char *p = strtok (input_buf, " ") ;
    while ((p) && (idx < MAX_TOKENS))
    {
      tokens[idx] = p ;
      idx++ ;
      p = strtok (NULL, " ") ;
    }
    tokens[idx] = NULL ;
    reply_buf[0] = 0 ;
    if (idx > 0)
      f_action (tokens) ;

    Udp.beginPacket (Udp.remoteIP(), Udp.remotePort()) ;
    Udp.write ((uint8_t*)reply_buf, strlen(reply_buf)) ;
    Udp.endPacket () ;
  }

  Webs.handleClient () ;

  /* one in a while, blink once if wifi is connected, twice otherwise. */

  if (millis() > next_blink)
  {
    if (WiFi.status() == WL_CONNECTED)
      f_blink (1) ;
    else
      f_blink (2) ;
    next_blink = next_blink + BLINK_FREQ ;
  }

  #endif

  delay (50) ; // mandatory sleep to prevent excessive power drain
}

