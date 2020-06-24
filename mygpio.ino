/*
   Examples

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

   Bugs

     - The "wifi {ssid|pw} <value>" command does not support arguments with
       spaces.
*/

#include <Wire.h>

#ifdef ARDUINO_ESP8266_NODEMCU
  #include <FS.h>
  #include <ESP8266WiFi.h>

  #define MAX_SSID_LEN 32
  #define MAX_PASSWD_LEN 64             // maximum wifi password length
  #define MAX_WIFI_TIMEOUT 60           // wifi connect timeout (secs)

  #define WIFI_SSID_FILE "/wifi.ssid"
  #define WIFI_PW_FILE "/wifi.pw"

  char cfg_wifi_ssid[MAX_SSID_LEN + 1] ;
  char cfg_wifi_pw[MAX_PASSWD_LEN + 1] ;
#endif

#define MAX_LONG 2147483647
#define MAX_TOKENS 10
#define BUF_SIZE 80

char line[BUF_SIZE] ;           // general purpose string buffer
char *tokens[MAX_TOKENS+1] ;    // max command parameters we'll parse

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
    Serial.println ("Cannot read data from BMP180.") ;
    return (0) ;
  }

  sprintf (line, "ac1: %d\nac2: %d\nac3: %d\nac4: %d", ac1, ac2, ac3, ac4) ;
  Serial.println (line) ;
  sprintf (line, "ac5: %d\nac6: %d\nb1: %d\nb2: %d", ac5, ac6, b1, b2) ;
  Serial.println (line) ;
  sprintf (line, "mb: %d\nmc: %d\nmd: %d", mb, mc, md) ;
  Serial.println (line) ;

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

  sprintf (line, "raw_t: %d", raw_t) ;
  Serial.println (line) ;

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

  sprintf (line, "raw_p: %d", raw_p) ;
  Serial.println (line) ;

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
    Serial.println ("FAULT: f_dht22() no ACK, aborting.") ;
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
    Serial.println ("FAULT: f_dht22() checksum failed.") ;
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
    Serial.println ("FAULT: f_hcsr04() no response.") ;
    return (-1.0) ;
  }
  else
    return (float(echoUsecs) / 58.0) ; // convert time to centimeters
}

/* ------------------------------------------------------------------------- */

#ifdef ARDUINO_ESP8266_NODEMCU

void f_fs (char **tokens)
{
  char msg[BUF_SIZE] ;

  if (strcmp(tokens[1], "info") == 0)                           // info
  {
    FSInfo fi ;
    if (SPIFFS.info (fi))
    {
      sprintf (line, "totalBytes: %d\nusedBytes: %d\nblockSize: %d",
               fi.totalBytes, fi.usedBytes, fi.blockSize) ;
      Serial.println (line) ;
      sprintf (line, "pageSize: %d\nmaxOpenFiles: %d\nmaxPathLength: %d",
               fi.pageSize, fi.maxOpenFiles, fi.maxPathLength) ;
      Serial.println (line) ;
    }
    else
    {
      Serial.println ("Cannot obtain fs info.") ;
    }
  }
  else
  if (strcmp(tokens[1], "format") == 0)                         // format
  {
    if (SPIFFS.format())
    {
      Serial.println ("Success.") ;
    }
    else
    {
      Serial.println ("Formatting failed.") ;
    }
  }
  else
  if (strcmp(tokens[1], "ls") == 0)                             // ls
  {
    Dir dir = SPIFFS.openDir ("/") ;
    while (dir.next())
    {
      String s = dir.fileName () ;
      Serial.println (s) ;
    }
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
      Serial.println ("Invalid filename.") ;
      return ;
    }

    File f = SPIFFS.open (filename, "w") ;
    if (f)
    {
      int amt = f.print (content) ;
      f.close () ;
      sprintf (msg, "Wrote %d bytes to '%s'.", amt, filename) ;
      Serial.println (msg) ;
    }
    else
    {
      sprintf (line, "Cannot write to '%s'.", filename) ;
      Serial.println (line) ;
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
      Serial.println (msg) ;
    }
    else
    {
      Serial.println ("Cannot read file.") ;
    }
  }
  else
  if ((strcmp(tokens[1], "remove") == 0) &&                     // remove
      (tokens[2] != NULL))
  {
    char *filename = tokens[2] ;
    if (SPIFFS.remove(filename))
      Serial.println ("File removed.") ;
    else
      Serial.println ("Cannot remove file.") ;
  }
  else
  if ((strcmp(tokens[1], "rename") == 0) &&                     // rename
      (tokens[2] != NULL) && (tokens[3] != NULL))
  {
    char *old_name = tokens[2] ;
    char *new_name = tokens[3] ;

    if ((new_name[0] != '/') || (strlen(new_name) == 1))
    {
      Serial.println ("Invalid filename.") ;
      return ;
    }
    if (SPIFFS.rename(old_name, new_name))
      Serial.println ("File renamed.") ;
    else
      Serial.println ("Cannot rename file.") ;
  }
  else
  {
    Serial.println ("Invalid argument.") ;
  }
}

void f_wifi (char **tokens)
{
  if (strcmp(tokens[1], "status") == 0)                         // status
  {
    sprintf (line, "cfg_wifi_ssid: %s", cfg_wifi_ssid) ;
    Serial.println (line) ;
    if (strlen(cfg_wifi_pw) > 0)
      Serial.println ("cfg_wifi_pw: (set)") ;
    else
      Serial.println ("cfg_wifi_pw: (unset)") ;

    int status = WiFi.status() ;
    strcpy (line, "status: ") ;
    switch (status)
    {
      case WL_CONNECTED:
        strcat (line, "WL_CONNECTED") ; break ;
      case WL_NO_SHIELD:
        strcat (line, "WL_NO_SHIELD") ; break ;
      case WL_IDLE_STATUS:
        strcat (line, "WL_IDLE_STATUS") ; break ;
      case WL_NO_SSID_AVAIL:
        strcat (line, "WL_NO_SSID_AVAIL") ; break ;
      case WL_SCAN_COMPLETED:
        strcat (line, "WL_SCAN_COMPLETED") ; break ;
      case WL_CONNECT_FAILED:
        strcat (line, "WL_CONNECT_FAILED") ; break ;
      case WL_CONNECTION_LOST:
        strcat (line, "WL_CONNECTION_LOST") ; break ;
      case WL_DISCONNECTED:
        strcat (line, "WL_DISCONNECTED") ; break ;
      default:
        strcat (line, "UNKNOWN") ; break ;
    }
    Serial.println (line) ;

    unsigned char mac[6] ;
    WiFi.macAddress(mac) ;
    sprintf (line, "wifi_mac: %x:%x:%x:%x:%x:%x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]) ;
    Serial.println (line) ;
    Serial.print ("wifi_ip: ") ;
    Serial.print (WiFi.localIP()) ;
    Serial.print ("/") ;
    Serial.println (WiFi.subnetMask()) ;
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
        sprintf (line, "Connected in %d seconds.", retry) ;
        Serial.println (line) ;
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
            Serial.println ("WL_NO_SHIELD") ; return ;
          case WL_IDLE_STATUS:
            Serial.println ("WL_IDLE_STATUS") ; return ;
          case WL_NO_SSID_AVAIL:
            Serial.println ("WL_NO_SSID_AVAIL") ; return ;
          case WL_SCAN_COMPLETED:
            Serial.println ("WL_SCAN_COMPLETE") ; return ;
          case WL_CONNECT_FAILED:
            Serial.println ("WL_CONNECT_FAILED") ; return ;
          case WL_CONNECTION_LOST:
            Serial.println ("WL_CONNECTION_LOST") ; return ;
          default:
            Serial.println ("UNKNOWN") ; return ;
        }
      }
    }
    Serial.println ("Connection attempt timed out.") ;
  }
  else
  {
    Serial.println ("Invalid argument.") ;
  }
}

#endif

/* ------------------------------------------------------------------------- */

void f_action (char **tokens)
{
  if ((strcmp(tokens[0], "?") == 0) || (strcmp(tokens[0], "help") == 0))
  {
    Serial.println ("hi <pin 0-13>") ;
    Serial.println ("lo <pin 0-13>") ;
    Serial.println ("aread <pin 0-5> - analog read") ;
    Serial.println ("bmp180") ;
    Serial.println ("dht22 <dataPin> - DHT-22 temperature/humidity sensor") ;
    Serial.println ("hcsr04 <trigPin> <echoPin> - HC-SR04 ultrasonic sensor") ;
    Serial.println ("uptime") ;
    #ifdef ARDUINO_ESP8266_NODEMCU
      Serial.println ("fs format") ;
      Serial.println ("fs info") ;
      Serial.println ("fs ls") ;
      Serial.println ("fs read <filename>") ;
      Serial.println ("fs remove <filename>") ;
      Serial.println ("fs rename <old> <new>") ;
      Serial.println ("fs write <filename> <content>") ;
      Serial.println ("wifi connect") ;
      Serial.println ("wifi status") ;
      Serial.println ("wifi ssid <ssid>") ;
      Serial.println ("wifi pw <password>") ;
      Serial.println ("wifi disconnect") ;
    #endif
  }
  else
  if ((strcmp(tokens[0], "hi") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, OUTPUT) ;
    digitalWrite (pin, HIGH) ;
    sprintf (line, "pin:%d HIGH", pin) ;
    Serial.println (line) ;
  }
  else
  if ((strcmp(tokens[0], "lo") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, OUTPUT) ;
    digitalWrite (pin, LOW) ;
    sprintf (line, "pin:%d LOW", pin) ;
    Serial.println (line) ;
  }
  else
  if ((strcmp(tokens[0], "aread") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    int val = analogRead (pin) ;
    sprintf (line, "analogRead pin:%d - %d", pin, val) ;
    Serial.println (line) ;
  }
  else
  if (strcmp(tokens[0], "bmp180") == 0)
  {
    float t=0.0, p=0.0 ;
    if (f_bmp180 (&t, &p))
    {
      sprintf (line, "bmp180 - temperature:%d.%02d pressure:%d.%02d",
               int(t), (int)(t*100)%100, int(p), (int)(p*100)%100) ;
      Serial.println (line) ;
    }
  }
  else
  if ((strcmp(tokens[0], "dht22") == 0) && (tokens[1] != NULL))
  {
    float t=0.0, h=0.0 ;
    if (f_dht22 (atoi(tokens[1]), &t, &h))
    {
      sprintf (line, "dht22 - temperature:%d.%02d humidity:%d.%02d",
               int(t), (int)(t*100)%100, int(h), (int)(h*100)%100) ;
      Serial.println (line) ;
    }
  }
  else
  if ((strcmp(tokens[0], "hi") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, OUTPUT) ;
    digitalWrite (pin, HIGH) ;
    sprintf (line, "pin:%d HIGH", pin) ;
    Serial.println (line) ;
  }
  else
  if ((strcmp(tokens[0], "lo") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, OUTPUT) ;
    digitalWrite (pin, LOW) ;
    sprintf (line, "pin:%d LOW", pin) ;
    Serial.println (line) ;
  }
  else
  if ((strcmp(tokens[0], "aread") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    int val = analogRead (pin) ;
    sprintf (line, "analogRead pin:%d - %d", pin, val) ;
    Serial.println (line) ;
  }
  else
  if (strcmp(tokens[0], "bmp180") == 0)
  {
    float t=0.0, p=0.0 ;
    if (f_bmp180 (&t, &p))
    {
      sprintf (line, "bmp180 - temperature:%d.%02d pressure:%d.%02d",
               int(t), (int)(t*100)%100, int(p), (int)(p*100)%100) ;
      Serial.println (line) ;
    }
  }
  else
  if ((strcmp(tokens[0], "dht22") == 0) && (tokens[1] != NULL))
  {
    float t=0.0, h=0.0 ;
    if (f_dht22 (atoi(tokens[1]), &t, &h))
    {
      sprintf (line, "dht22 - temperature:%d.%02d humidity:%d.%02d",
               int(t), (int)(t*100)%100, int(h), (int)(h*100)%100) ;
      Serial.println (line) ;
    }
  }
  else
  if ((strcmp(tokens[0], "hcsr04") == 0) && 
      (tokens[1] != NULL) && (tokens[2] != NULL))
  {
    float f = f_hcsr04 (atoi(tokens[1]), atoi(tokens[2])) ;
    if (f > 0.0)
    {
      sprintf (line, "hcsr04 - %d.%02d cm", int(f), (int)(f*100)%100) ;
      Serial.println (line) ;
    }
  }
  else
  if (strcmp(tokens[0], "uptime") == 0)
  {
    unsigned long now = millis() / 1000 ;
    sprintf (line, "uptime - %ld secs", now) ;
    Serial.println (line) ;
  }

  #ifdef ARDUINO_ESP8266_NODEMCU
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
  #endif

  else
  {
    Serial.println ("FAULT: Unknown command. Enter ? for help.") ;
  }
}

/* ------------------------------------------------------------------------- */

void setup ()
{
  Wire.begin () ;
  Serial.begin (9600) ;
  Serial.setTimeout (MAX_LONG) ; // Serial.read() to block as long as possible

  #ifdef ARDUINO_ESP8266_NODEMCU
    cfg_wifi_ssid[0] = 0 ;
    cfg_wifi_pw[0] = 0 ;
    SPIFFS.begin () ;
    pinMode (LED_BUILTIN, OUTPUT) ;

    /* if wifi ssid and password are available, try connect to wifi now */

    FSInfo fi ;
    if (SPIFFS.info(fi))
    {
      File f_ssid = SPIFFS.open (WIFI_SSID_FILE, "r") ;
      File f_pw = SPIFFS.open (WIFI_PW_FILE, "r") ;
      if ((f_ssid) && (f_pw) &&
          (f_ssid.size() <= MAX_SSID_LEN) && (f_pw.size() <= MAX_PASSWD_LEN))
      {
        int s_amt = f_ssid.readBytes (cfg_wifi_ssid, MAX_SSID_LEN) ;
        if (s_amt > 0)
          cfg_wifi_ssid[s_amt] = 0 ;
        int p_amt = f_pw.readBytes (cfg_wifi_pw, MAX_PASSWD_LEN) ;
        if (p_amt > 0)
          cfg_wifi_pw[p_amt] = 0 ;
        if ((s_amt > 0) && (p_amt > 0))
          WiFi.begin (cfg_wifi_ssid, cfg_wifi_pw) ;
      }
      if (f_ssid)
        f_ssid.close () ;
      if (f_pw)
        f_pw.close () ;
    }

    digitalWrite (LED_BUILTIN, LOW) ;           // blink to indicate boot.
    delay (1000) ;
    digitalWrite (LED_BUILTIN, HIGH) ;
  #endif

  Serial.println ("\nReady.") ;
}

void loop ()
{
  int idx=0 ;
  char *p ;

  /* At the start of our loop, print the prompt */

  Serial.println ("OK") ;

  /* wait for '\r' (minicom sends this when user presses ENTER) */

  int amt = Serial.readBytesUntil('\r', line, BUF_SIZE-1) ;
  line[amt] = 0 ;

  /* parse what we've received on the serial port */

  if (amt == 0)
  {
    Serial.print ("\r\n") ;
  }
  else
  {
    idx = 0 ;
    p = strtok (line, " ") ;
    while ((p) && (idx < MAX_TOKENS))
    {
      tokens[idx] = p ;
      idx++ ;
      p = strtok (NULL, " ") ;
    }
    tokens[idx] = NULL ;
    if (tokens[0] != NULL)
      f_action (tokens) ;
  }
}

