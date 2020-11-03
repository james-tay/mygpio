/*
   Build/Upload

     % arduino-cli lib install PubSubClient

     % arduino-cli compile -b esp32:esp32:esp32 .
     % arduino-cli compile -b esp8266:esp8266:nodemcuv2 .
     % arduino-cli compile -b arduino:avr:uno .

     % arduino-cli upload -v -p /dev/ttyUSB0 -b esp32:esp32:esp32 .
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
       hi 12
       dht22 13
       lo 12

     test BMP180,       (on nodemcu, SCL is D1, SDA is D2)
       bmp180

     test ADXL335       (on esp32)
       hi 23
       adxl335 36 39 34 1000 50
       lo 23

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

     test thread creation
       esp32 thread_start count1

     [ REST ]

     % curl 'http://esp32.example.com/metrics'
     % curl 'http://esp32.example.com/v1?cmd=...'

     eg,

     % curl 'http://esp32.example.com/v1?cmd=wifi+status'

   Bugs

     - The "wifi {ssid|pw} <value>" command does not support arguments with
       spaces.
     - The "fs write <filename> <content>" command does not support content
       with spaces.
     - All filenames must begin with '/'.
     - LCD row/columns should be configurable.

   Threads

     - Background threads can be dynamically started/stopped by the user. The
       expectation is that each thread runs some task at fixed cycle times,
       the cycle time is up to the thread author to implement.
     - Array of "G_thread_entry" tracks state, tid and parameters of all
       threads. We want to avoid using malloc().
     - Each thread must be given a unique user defined instance "name". This
       "name" is used to:
         a) identify which file stores the thread's configuration
         b) specify which thread to start/stop
     - Each thread may periodically save its output in the "results" array
       which can be viewed at the "/metrics" URI.
     - Each thread may write a status "msg", which can be viewed via the
       "esp32 thread_list" command.
     - Each thread could report multiple result values. Each value is reported
       with multiple (optional) tags, for example :
         <name>{[<meta1>=<data1>,...]} <value>
       eg:
         mycar_accel{axis="x",sample="ave"} 1791
     - Traditional task functions are named f_<task>, while thread safe tasks
       are named ft_<task>. This is a naming convention.
     - Thread life cycle is as follows :
         initialization by f_thread_create()
         `-> the tread function f_thread_lifecycle()
             `- main loop, call ft_<task>()
                |-> run user code, complete within 1 second (recommended)
                `-> update results in G_thread_entry structure
     - Each ft_<task> may return up to MAX_THREAD_RESULT_VALUES values, thus
       we need to label up to MAX_THREAD_RESULT_VALUES "meta" and "data"
       key/value pairs. This is the ft_<task>'s responsibility.
     - Arguments to ft_<task> must be stored in a file "/thread-<name>",
       which are read by f_thread_create(). This is the user's responsibility.
     - This file must contain 1 line with comma separated parameters :
         <ft_task>[,<arg>,...]
       Eg,
         ft_counter,200,1
     - The first argument is the ft_<task> that f_thread_lifecycle() will
       execute, the specified arguments are all passed to ft_<task> as a
       "num_args" array of (char*).
     - Thread management (ie, create/stop/list) is single threaded.

   Writing ft_<tasks>

     a) write function code : "void ft_<task> (S_thread_entry *p)"
       - ft_<task> will be called repeatedly by f_thread_lifecycle()
       - ft_<task> should introduce its own delay to limit cpu load
     b) document call and its arguments in "thread_help"
     c) update f_thread_create() to identify function address
     d) a thread may set its state to THREAD_STOPPED if it chooses to
        terminate early (eg, encountering a critical error state).

   Presenting Metrics For Scraping

     - Each thread performs its task and regularly updates its "results"
       array with data for prometheus to scrape. Consider a thread monitoring
       an analog input pin value, when prometheus scrapes us, we want to
       present an entry like :

         sensor_moisture{location="FlowerBed",model="resistance"} 568

     - To make this possible, we need to configure threads using 2x files :

         /thread-<name>
         /tags-<name>           (optional)

     - In the earlier example, we place the following in "/tags-<name>" :

         sensor_moisture,location=FlowerBed,model=resistance

     - Thus, the format for the "/tags-<name>" file is :

         <metric>[,<tagN>=<valueN>,...]

     - A thread may insert its own tags in its "results" structure. These
       custom tags will be merged with those in "/tags-<name>" provided they
       don't exceed MAX_THREAD_RESULT_TAGS. If the "/tags-<name>" file is
       absent, "<name>" will be used in place of "<metric>".


   Delivering events from threads (fix me)

     - A thread may deliver data when appropriate by calling f_delivery().
     - The f_delivery() function delivers "<name> <msg>" via MQTT.

   Notes

     - pin numbering goes by GPIO number, thus "hi 5" means GPIO pin 5, not
       the physical pin number.

     - LCD implementation reference :
       https://create.arduino.cc/projecthub/arduino_uno_guy/
         (navigate to "I2C Liquid Crystal Displays")

     - ESP32 hardware API reference
       https://github.com/espressif/arduino-esp32

     - PubSubClient reference
       https://pubsubclient.knolleary.net/

   Bugs

     - tons of buffer overrun opportunities due to static buffers with not
       much buffer length checking.

     - SPIFFS is not thread safe and can only be called safely by the main
       thread.
*/

#include <Wire.h>
#include "LiquidCrystal_I2C.h"

/* ====== STUFF COMMON BETWEEN ESP32 and ESP8266 ====== */

#if defined ARDUINO_ESP8266_NODEMCU || ARDUINO_ESP32_DEV
  #include <FS.h>
  #include <WiFiUdp.h>
  #include <PubSubClient.h>

  #define MAX_SSID_LEN 32
  #define MAX_PASSWD_LEN 64             // maximum wifi password length
  #define MAX_MQTT_LEN 80               // maximum mqtt message we receive
  #define MAX_WIFI_TIMEOUT 60           // wifi connect timeout (secs)
  #define WEB_PORT 80                   // web server listens on this port
  #define CRON_INTERVAL 60              // how often we run f_cron()

  #define WIFI_SSID_FILE "/wifi.ssid"
  #define WIFI_PW_FILE "/wifi.pw"
  #define UDP_PORT_FILE "/udp.port"
  #define MQTT_CFG_FILE "/mqtt.cfg"
  #define MQTT_SUB_FILE "/mqtt.sub"
  #define MQTT_PUB_FILE "/mqtt.pub"

  char cfg_wifi_ssid[MAX_SSID_LEN + 1] ;
  char cfg_wifi_pw[MAX_PASSWD_LEN + 1] ;
  int cfg_udp_port=0 ;
  WiFiUDP G_Udp ;                       // our UDP server socket

  char G_mqtt_pub[MAX_MQTT_LEN] ;       // mqtt topic we publish to
  char G_mqtt_sub[MAX_MQTT_LEN] ;       // mqtt topic we subscribe to
  unsigned long G_next_cron ;           // millis() time of next run

#endif

/* ====== STUFF SPECIFIC TO ESP32 only or ESP8266 only ====== */

#ifdef ARDUINO_ESP32_DEV        // ***** ESP32 Only ******
  #define LED_BUILTIN 2
  #define BLINK_ON HIGH
  #define BLINK_OFF LOW

  #include <pthread.h>
  #include <SPIFFS.h>
  #include <WebServer.h>

  WiFiClient G_wClient ;
  PubSubClient G_psClient (G_wClient) ;

  WebServer Webs(WEB_PORT) ;    // our built-in webserver
  int G_sleep = 0 ;             // a flag to tell us to enter sleep mode
  int G_req_connect = 0 ;       // threads requesting wifi/mqtt connection
  pthread_mutex_t G_delivery_lock ;     // lock for f_delivery()

  #define MAX_THREADS 16
  #define MAX_THREAD_NAME 40
  #define MAX_THREAD_ARGS 8             // number of input args
  #define MAX_THREAD_RESULT_TAGS 8      // meta data tags
  #define MAX_THREAD_RESULT_VALUES 16   // output values
  #define MAX_THREAD_CONF_BUF 80        // length of thread's "conf"
  #define MAX_THREAD_TAGS_BUF 80        // length of thread's tags
  #define MAX_THREAD_MSG_BUF 80         // length of thread's "msg"
  #define MAX_THREAD_TAGS 8             // tag pairs in "/tags-<name>"

  /* various states for S_thread_entry.state */

  #define THREAD_READY          0       // ready to be started
  #define THREAD_STARTING       1       // pthread_create() just got called
  #define THREAD_RUNNING        2       // thread in f_thread_lifecycle()
  #define THREAD_WRAPUP         3       // tell a thread to terminate
  #define THREAD_STOPPED        4       // awaiting pthread_join()

  /* Data structure of a single thread result value (with multiple tags) */

  struct thread_result_s
  {
    char *metric ;                      // the metric we're displaying
    int num_tags ;                      // number of meta data tags
    char *meta[MAX_THREAD_RESULT_TAGS] ;  // array of meta tags
    char *data[MAX_THREAD_RESULT_TAGS] ;  // array of data tags
    int i_value ;                       // this result's value
  } ;
  typedef struct thread_result_s S_thread_result ;

  /* Data structure to track a single thread, as well as its in/out data */

  struct thread_entry_s
  {
    char name[MAX_THREAD_NAME] ;
    unsigned char state ;
    pthread_t tid ;
    pthread_mutex_t lock ;              // lock before making changes here

    /* input arguments to <ft_task> */

    int num_args ;                      // number of input arguments
    char *in_args[MAX_THREAD_ARGS] ;    // array of pointers into "conf"
    char conf[MAX_THREAD_CONF_BUF] ;    // main config buffer
    char tags_buf[MAX_THREAD_TAGS_BUF] ; // metric and tags (optional)
    char *metric ;                      // pointer to metric name (optional)
    char *tags[MAX_THREAD_TAGS] ;       // pointers to "<key>=<value>" pairs
    unsigned long loops ;               // number of <ft_task> calls so far
    unsigned long ts_started ;        // millis() timestamp of pthread_create()
    void (*ft_addr)(struct thread_entry_s*) ; // the <ft_task> this thread runs

    /* IMPORTANT !!! <ft_task> may modify anything below this point */

    int num_int_results ;               // number of actual results returned
    S_thread_result results[MAX_THREAD_RESULT_VALUES] ;
    char msg[MAX_THREAD_MSG_BUF] ;      // provide some optional feedback
  } ;
  typedef struct thread_entry_s S_thread_entry ;
  S_thread_entry *G_thread_entry ;

#endif

#ifdef ARDUINO_ESP8266_NODEMCU  // ***** ESP8266 Only *****
  #define BLINK_ON LOW
  #define BLINK_OFF HIGH

  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  ESP8266WebServer Webs(WEB_PORT) ;     // our built-in webserver

  WiFiClient G_wClient ;
  PubSubClient G_psClient (G_wClient) ;
#endif

/* ====== ALL OTHER GENERAL STUFF ====== */

#define DEF_BAUD 9600
#define SERIAL_TIMEOUT 1000     // serial timeout in milliseconds
#define BLINK_FREQ 5000         // blink to indicate we're alive (ms)
#define MAX_TOKENS 10
#define BUF_SIZE 80

#if defined ARDUINO_ESP8266_NODEMCU || ARDUINO_ESP32_DEV
  #define REPLY_SIZE 1024
#else
  #define REPLY_SIZE 192        // arduino uno has only 2x of SRAM.
#endif

/* LCD definitions */

#define LCD_ADDR 0x27
#define LCD_WIDTH 16
#define LCD_ROWS 2

/* global variables */

char *G_reply_buf ;             // accumulate our reply message here
int G_serial_pos ;              // index of bytes received on serial port
char G_serial_buf[BUF_SIZE+1] ; // accumulate butes on our serial console
unsigned long G_next_blink=BLINK_FREQ ; // "wall clock" time for next blink

LiquidCrystal_I2C G_lcd (LCD_ADDR, LCD_WIDTH, LCD_ROWS) ;

/* internal performance metrics */

struct internal_metrics
{
  unsigned long serialInBytes ;
  unsigned long serialCmds ;
  unsigned long serialOverruns ;
  unsigned long udpInBytes ;
  unsigned long udpCmds ;
  unsigned long restInBytes ;
  unsigned long restCmds ;
  unsigned long mqttConnects ;
  unsigned long mqttPubs ;
  unsigned long mqttSubs ;
  unsigned long mqttOversize ;
} ;
typedef struct internal_metrics S_Metrics ;
S_Metrics G_Metrics ;

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
    strcat (G_reply_buf, "FAULT: Cannot read data from BMP180.\r\n") ;
    return (0) ;
  }

  char line[BUF_SIZE] ;

  sprintf (line, "ac1: %d\r\nac2: %d\r\nac3: %d\r\n", ac1, ac2, ac3) ;
  strcat (G_reply_buf, line) ;
  sprintf (line, "ac4: %d\r\nac5: %d\r\nac6: %d\r\n", ac4, ac5, ac6) ;
  strcat (G_reply_buf, line) ;
  sprintf (line, "b1: %d\r\nb2: %d\r\nmb: %d\r\nmc: %d\r\n", b2, mb, mc) ;
  strcat (G_reply_buf, line) ;
  sprintf (line, "md: %d\r\n", md) ;
  strcat (G_reply_buf, line) ;

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
  strcat (G_reply_buf, line) ;

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
  strcat (G_reply_buf, line) ;

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
    strcat (G_reply_buf, "FAULT: f_dht22() no ACK, aborting.\r\n") ;
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
    strcat (G_reply_buf, "FAULT: f_dht22() checksum failed.\r\n") ;
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
    strcat (G_reply_buf, "FAULT: f_hcsr04() no response.\r\n") ;
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
    strcat (G_reply_buf, "FAULT: No arguments.\r\n") ;
    return ;
  }

  if (strcmp(tokens[1], "init") == 0)                           // init
  {
    G_lcd.init () ;
    strcat (G_reply_buf, "LCD initialized.\r\n") ;
  }
  else
  if (strcmp(tokens[1], "backlight") == 0)                      // backlight
  {
    if (strcmp(tokens[2], "on") == 0)
    {
      G_lcd.backlight () ;
      strcat (G_reply_buf, "LCD backlight is on.\r\n") ;
    }
    else
    if (strcmp(tokens[2], "off") == 0)
    {
      G_lcd.noBacklight () ;
      strcat (G_reply_buf, "LCD backlight is off.\r\n") ;
    }
    else
    {
      strcat (G_reply_buf, "FAULT: Invalid argument.\r\n") ;
    }
  }
  else
  if (strcmp(tokens[1], "clear") == 0)                          // clear
  {
    G_lcd.clear () ;
    strcat (G_reply_buf, "LCD cleared.\r\n") ;
  }
  else
  if ((strcmp(tokens[1], "print") == 0) &&                      // print
      (tokens[2] != NULL) &&
      (tokens[3] != NULL) &&
      (tokens[4] != NULL))
  {
    int row = atoi(tokens[2]) ;
    int col = atoi(tokens[3]) ;

    char line[BUF_SIZE] ;
    line[0] = 0 ;
    for (int idx=4 ; idx < MAX_TOKENS ; idx++)
    {
      if (tokens[idx] == NULL)
        break ;
      if (strlen(line) > 0)
        strcat (line, " ") ;
      strcat (line, tokens[idx]) ;
    }

    G_lcd.setCursor (col, row) ;
    G_lcd.print (line) ;
    sprintf (line, "LCD write at row:%d col:%d.\r\n", row, col) ;
    strcat (G_reply_buf, line) ;
  }
  else
  {
    strcat (G_reply_buf, "FAULT: Invalid argument.\r\n") ;
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
  char msg[BUF_SIZE], line[BUF_SIZE] ;

  if (strcmp(tokens[1], "info") == 0)                           // info
  {
    #ifdef ARDUINO_ESP8266_NODEMCU

    FSInfo fi ;
    if (SPIFFS.info (fi))
    {
      sprintf (line, "totalBytes: %d\r\nusedBytes: %d\r\nblockSize: %d\r\n",
               fi.totalBytes, fi.usedBytes, fi.blockSize) ;
      strcat (G_reply_buf, line) ;
      sprintf (line, "pageSize: %d\r\nmaxOpenFiles: %d\r\n",
               fi.pageSize, fi.maxOpenFiles) ;
      strcat (G_reply_buf, line) ;
      sprintf (line, "maxPathLength: %d\r\n", fi.maxPathLength) ;
    }
    else
    {
      strcat (G_reply_buf, "FAULT: Cannot obtain fs info.\r\n") ;
    }

    #endif
    #ifdef ARDUINO_ESP32_DEV

      sprintf (line, "totalBytes: %d\r\nusedBytes: %d\r\n",
               SPIFFS.totalBytes(), SPIFFS.usedBytes()) ;
      strcat (G_reply_buf, line) ;

    #endif
  }
  else
  if (strcmp(tokens[1], "format") == 0)                         // format
  {
    if (SPIFFS.format())
    {
      strcat (G_reply_buf, "Success.\r\n") ;
    }
    else
    {
      strcat (G_reply_buf, "FAULT: Formatting failed.\r\n") ;
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
      strcat (G_reply_buf, line) ;
    }

    #endif
    #ifdef ARDUINO_ESP32_DEV

    File root = SPIFFS.open ("/", "r") ;
    File f = root.openNextFile () ;
    while (f)
    {
      sprintf (line, "%-8d %s\r\n", f.size(), f.name()) ;
      strcat (G_reply_buf, line) ;
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
      strcat (G_reply_buf, "FAULT: Invalid filename.\r\n") ;
      return ;
    }

    File f = SPIFFS.open (filename, "w") ;
    if (f)
    {
      int amt = f.print (content) ;
      f.close () ;
      sprintf (msg, "Wrote %d bytes to '%s'.\r\n", amt, filename) ;
      strcat (G_reply_buf, msg) ;
    }
    else
    {
      sprintf (msg, "FAULT: Cannot write to '%s'.\r\n", filename) ;
      strcat (G_reply_buf, msg) ;
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
      strcat (G_reply_buf, msg) ;
      strcat (G_reply_buf, "\r\n") ;
    }
    else
    {
      strcat (G_reply_buf, "FAULT: Cannot read file.\r\n") ;
    }
  }
  else
  if ((strcmp(tokens[1], "rm") == 0) &&                         // rm
      (tokens[2] != NULL))
  {
    char *filename = tokens[2] ;
    if (SPIFFS.remove(filename))
      strcat (G_reply_buf, "File removed.\r\n") ;
    else
      strcat (G_reply_buf, "FAULT: Cannot remove file.\r\n") ;
  }
  else
  if ((strcmp(tokens[1], "rename") == 0) &&                     // rename
      (tokens[2] != NULL) && (tokens[3] != NULL))
  {
    char *old_name = tokens[2] ;
    char *new_name = tokens[3] ;

    if ((new_name[0] != '/') || (strlen(new_name) == 1))
    {
      strcat (G_reply_buf, "FAULT: Invalid filename.\r\n") ;
      return ;
    }
    if (SPIFFS.rename(old_name, new_name))
      strcat (G_reply_buf, "File renamed.\r\n") ;
    else
      strcat (G_reply_buf, "FAULT: Cannot rename file.\r\n") ;
  }
  else
  {
    strcat (G_reply_buf, "FAULT: Invalid argument.\r\n") ;
  }
}

void f_wifi (char **tokens)
{
  char line[BUF_SIZE] ;

  if (strcmp(tokens[1], "scan") == 0)                           // scan
  {
    int n = WiFi.scanNetworks() ;
    sprintf (line, "Found %d wifi networks.\r\n", n) ;
    strcat (G_reply_buf, line) ;
    for (int i=0 ; i<n ; i++)
    {
      char ssid[MAX_SSID_LEN+1] ;
      WiFi.SSID(i).toCharArray (ssid, MAX_SSID_LEN) ;
      sprintf (line, "%2d. ch %d, %d dBm [%s]\r\n",
               i+1, WiFi.channel(i), WiFi.RSSI(i), ssid) ;
      if (strlen(G_reply_buf) + strlen(line) < REPLY_SIZE)
        strcat (G_reply_buf, line) ;
    }
  }
  else
  if (strcmp(tokens[1], "status") == 0)                         // status
  {
    sprintf (line, "cfg_wifi_ssid: %s\r\n", cfg_wifi_ssid) ;
    strcat (G_reply_buf, line) ;
    if (strlen(cfg_wifi_pw) > 0)
      strcat (G_reply_buf, "cfg_wifi_pw: (set)\r\n") ;
    else
      strcat (G_reply_buf, "cfg_wifi_pw: (unset)\r\n") ;

    int status = WiFi.status() ;
    strcat (G_reply_buf, "status: ") ;
    switch (status)
    {
      case WL_CONNECTED:
        strcat (G_reply_buf, "WL_CONNECTED\r\n") ; break ;
      case WL_NO_SHIELD:
        strcat (G_reply_buf, "WL_NO_SHIELD\r\n") ; break ;
      case WL_IDLE_STATUS:
        strcat (G_reply_buf, "WL_IDLE_STATUS\r\n") ; break ;
      case WL_NO_SSID_AVAIL:
        strcat (G_reply_buf, "WL_NO_SSID_AVAIL\r\n") ; break ;
      case WL_SCAN_COMPLETED:
        strcat (G_reply_buf, "WL_SCAN_COMPLETED\r\n") ; break ;
      case WL_CONNECT_FAILED:
        strcat (G_reply_buf, "WL_CONNECT_FAILED\r\n") ; break ;
      case WL_CONNECTION_LOST:
        strcat (G_reply_buf, "WL_CONNECTION_LOST\r\n") ; break ;
      case WL_DISCONNECTED:
        strcat (G_reply_buf, "WL_DISCONNECTED\r\n") ; break ;
      default:
        strcat (G_reply_buf, "UNKNOWN\r\n") ; break ;
    }

    sprintf (line, "rssi: %d dBm\r\n", WiFi.RSSI()) ;
    strcat (G_reply_buf, line) ;
    sprintf (line, "udp_port: %d\r\n", cfg_udp_port) ;
    strcat (G_reply_buf, line) ;

    unsigned char mac[6] ;
    WiFi.macAddress(mac) ;
    sprintf (line, "wifi_mac: %x:%x:%x:%x:%x:%x\r\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]) ;
    strcat (G_reply_buf, line) ;
    sprintf (line, "wifi_ip: %s/%s\r\n",
             WiFi.localIP().toString().c_str(),
             WiFi.subnetMask().toString().c_str()) ;
    strcat (G_reply_buf, line) ;
    sprintf (line, "mqtt_state: %d\r\n", G_psClient.state()) ;
    strcat (G_reply_buf, line) ;
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
        strcat (G_reply_buf, line) ;
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
            strcat (G_reply_buf, "WL_NO_SHIELD\r\n") ; return ;
          case WL_IDLE_STATUS:
            strcat (G_reply_buf, "WL_IDLE_STATUS\r\n") ; return ;
          case WL_NO_SSID_AVAIL:
            strcat (G_reply_buf, "WL_NO_SSID_AVAIL\r\n") ; return ;
          case WL_SCAN_COMPLETED:
            strcat (G_reply_buf, "WL_SCAN_COMPLETE\r\n") ; return ;
          case WL_CONNECT_FAILED:
            strcat (G_reply_buf, "WL_CONNECT_FAILED\r\n") ; return ;
          case WL_CONNECTION_LOST:
            strcat (G_reply_buf, "WL_CONNECTION_LOST\r\n") ; return ;
          default:
            strcat (G_reply_buf, "UNKNOWN\r\n") ; return ;
        }
      }
    }
    strcat (G_reply_buf, "FAULT: Connection attempt timed out.\r\n") ;
  }
  else
  {
    strcat (G_reply_buf, "FAULT: Invalid argument.\r\n") ;
  }
}

void f_mqtt_callback (char *topic, byte *payload, unsigned int length)
{
  char msg[MAX_MQTT_LEN + 1], buf[BUF_SIZE] ;

  if (length > MAX_MQTT_LEN)
  {
    G_Metrics.mqttOversize++ ;
    return ;
  }
  memcpy (msg, payload, length) ;
  msg[length] = 0 ;
  G_Metrics.mqttSubs++ ;

  snprintf (buf, BUF_SIZE, "[%s] %s", topic, msg) ;
  Serial.println (buf) ;
}

void f_mqtt_connect ()
{
  char buf[BUF_SIZE], line[BUF_SIZE] ;

  File f = SPIFFS.open (MQTT_SUB_FILE, "r") ; // subscribe file is optional
  if (f != NULL)
  {
    int amt = f.readBytes (buf, BUF_SIZE-1) ;
    f.close () ;
    if (amt > 0)
    {
      buf[amt] = 0 ;
      strcpy (G_mqtt_sub, buf) ;
    }
  }
  f = SPIFFS.open (MQTT_PUB_FILE, "r") ;      // publish file is mandatory
  if (f == NULL)
  {
    sprintf (line, "WARNING: Cannot read MQTT publish file '%s'.",
             MQTT_PUB_FILE) ;
    Serial.println (line) ;
    return ;
  }
  else
  {
    int amt = f.readBytes (buf, BUF_SIZE-1) ;
    f.close () ;
    if (amt > 0)
    {
      buf[amt] = 0 ;
      strcpy (G_mqtt_pub, buf) ;
    }
  }

  f = SPIFFS.open (MQTT_CFG_FILE, "r") ;      // broker config is mandatory
  if (f == NULL)
  {
    sprintf (line, "WARNING: Cannot read MQTT subscribe file '%s'.",
             MQTT_CFG_FILE) ;
    Serial.println (line) ;
  }
  else
  {
    int amt = f.readBytes (buf, BUF_SIZE-1) ;
    f.close () ;
    if (amt > 0)
    {
      buf[amt] = 0 ;
      char *mqtt_host, *mqtt_port, *user, *pw ;

      if (((mqtt_host = strtok (buf, ",")) == NULL) ||
          ((mqtt_port = strtok (NULL, ",")) == NULL) ||
          ((user = strtok (NULL, ",")) == NULL) ||
          ((pw = strtok (NULL, ",")) == NULL))
      {
        sprintf (line, "WARNING: Cannot parse %s.", MQTT_CFG_FILE) ;
        Serial.println (line) ;
      }
      else
      {
        char id[BUF_SIZE] ;
        unsigned char mac[6] ;
        WiFi.macAddress(mac) ;
        sprintf (id, "%x-%x-%x-%x-%x-%x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]) ;
        G_psClient.setServer (mqtt_host, atoi(mqtt_port)) ;
        if (G_psClient.connect (id, user, pw))
        {
          G_Metrics.mqttConnects++ ;
          if (strlen(G_mqtt_sub) > 0)
          {
            G_psClient.subscribe (G_mqtt_sub) ;
            G_psClient.setCallback (f_mqtt_callback) ;
          }
        }
        else
        {
          sprintf (line, "WARNING: Cannot connect to broker %s:%d.",
                   mqtt_host, atoi(mqtt_port)) ;
          Serial.println (line) ;
        }
      }
    }
  }
}

void f_delivery (char *name, char *payload)
{
  #ifdef ARDUINO_ESP32_DEV
  pthread_mutex_lock (&G_delivery_lock) ;
  #endif

  if (G_psClient.connected() == false)
  {
    #ifdef ARDUINO_ESP32_DEV
    pthread_mutex_unlock (&G_delivery_lock) ;
    #endif

    G_req_connect = 1 ;
    Serial.println ("WARNING: MQTT not connected.") ;
    return ;
  }

  if ((G_psClient.connected()) && (strlen(G_mqtt_pub) > 0) &&
      (name != NULL) && (payload != NULL))
  {
    char buf[BUF_SIZE] ;
    sprintf (buf, "%s %s", name, payload) ;
    G_psClient.publish (G_mqtt_pub, buf) ;
    G_Metrics.mqttPubs++ ;
  }

  #ifdef ARDUINO_ESP32_DEV
  pthread_mutex_unlock (&G_delivery_lock) ;
  #endif
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

  char url_buf[BUF_SIZE] ;
  strncpy (url_buf, Webs.arg(0).c_str(), BUF_SIZE) ;
  G_Metrics.restInBytes = G_Metrics.restInBytes + strlen(url_buf) ;
  G_Metrics.restCmds++ ;
  int idx = 0 ;
  char *tokens[MAX_TOKENS] ;
  char *p = strtok (url_buf, " ") ;
  while ((p) && (idx < MAX_TOKENS))
  {
    tokens[idx] = p ;
    idx++ ;
    p = strtok (NULL, " ") ;
  }
  tokens[idx] = NULL ;
  G_reply_buf[0] = 0 ;
  if (idx > 0)
    f_action(tokens) ;
  if (strlen(G_reply_buf) > 0)
    Webs.send (200, "text/plain", G_reply_buf) ;
  else
    Webs.send (504, "text/plain", "No response\r\n") ;
}

void f_handleWebMetrics ()                      // for uri "/metrics"
{
  sprintf (G_reply_buf,
           "node_uptime_secs %ld\n"
           "serial_in_bytes %ld\n"
           "serial_commands %ld\n"
           "serial_overruns %ld\n"
           "udp_in_bytes %ld\n"
           "udp_commands %ld\n"
           "rest_in_bytes %ld\n"
           "rest_commands %ld\n"
           "mqtt_connects %ld\n"
           "mqtt_pubs %ld\n"
           "mqtt_subs %ld\n"
           "mqtt_oversize %ld\n",
           millis() / 1000,
           G_Metrics.serialInBytes,
           G_Metrics.serialCmds,
           G_Metrics.serialOverruns,
           G_Metrics.udpInBytes,
           G_Metrics.udpCmds,
           G_Metrics.restInBytes,
           G_Metrics.restCmds,
           G_Metrics.mqttConnects,
           G_Metrics.mqttPubs,
           G_Metrics.mqttSubs,
           G_Metrics.mqttOversize
           ) ;

  #ifdef ARDUINO_ESP32_DEV
    int idx, threads=0 ;
    char one_tag[BUF_SIZE], all_tags[BUF_SIZE], line[BUF_SIZE] ;

    for (idx=0 ; idx < MAX_THREADS ; idx++)
      if (G_thread_entry[idx].state == THREAD_RUNNING)
      {
        /* this thread is running, print all result values, including tags */

        int r, t ;
        for (r=0 ; r < G_thread_entry[idx].num_int_results ; r++)
        {
          all_tags[0] = 0 ;
          for (t=0 ; t < G_thread_entry[idx].results[r].num_tags ; t++)
          {
            sprintf (one_tag, "%s=%s",
                     G_thread_entry[idx].results[r].meta[t],
                     G_thread_entry[idx].results[r].data[t]) ;
            if (strlen(all_tags) > 0)
              strcat (all_tags, ",") ;
            strcat (all_tags, one_tag) ;
          }
          if (strlen(all_tags) > 0)
            sprintf (line, "%s{%s} %d\r\n",
                     G_thread_entry[idx].name, all_tags,
                     G_thread_entry[idx].results[r].i_value) ;
          else
            sprintf (line, "%s %d\r\n",
                     G_thread_entry[idx].name,
                     G_thread_entry[idx].results[r].i_value) ;
          strcat (G_reply_buf, line) ;
        }
        threads++ ;
      }
    sprintf (line,
             "free_heap_bytes %ld\n"
             "threads_running %d\n",
             xPortGetFreeHeapSize(),
             threads) ;
    strcat (G_reply_buf, line) ;
  #endif

  Webs.send (200, "text/plain", G_reply_buf) ;
}

#endif

/* ------------------------------------------------------------------------- */
/* Features specific to ESP32                                                */
/* ------------------------------------------------------------------------- */

#ifdef ARDUINO_ESP32_DEV

/*
   "tokens" is a string array of 6x params :
     <don't_care> <x_pin> <y_pin> <z_pin> <total_dur> <interval>

   "i_results" is an array of 10x ints which store the number of samples
   examined, folled by min/ave/max values of the X/Y/Z axis respectively.
*/

void f_adxl335 (char **tokens, int *i_results)
{
  #define MAX_DURATION 60000 // take readings for a maximum of 60 secs

  int x_value, y_value, z_value ;
  int x_min, y_min, z_min ;
  int x_max, y_max, z_max ;

  int samples = 0 ;
  int x_total = 0, y_total = 0, z_total =0 ;

  int x_pin = atoi (tokens[1]) ;
  int y_pin = atoi (tokens[2]) ;
  int z_pin = atoi (tokens[3]) ;
  unsigned long total_duration = atoi (tokens[4]) ;
  unsigned long interval = atoi (tokens[5]) ;

  /* sanity check some parameters first */

  if (total_duration > MAX_DURATION) total_duration = MAX_DURATION ;
  if (total_duration < 1) total_duration = 1 ;
  if (interval < 1) interval = 1 ;

  pinMode (x_pin, INPUT) ;
  pinMode (y_pin, INPUT) ;
  pinMode (z_pin, INPUT) ;

  unsigned long start_time = millis () ;
  unsigned long end_time = start_time + total_duration ;

  while (start_time < end_time)
  {
    x_value = analogRead (x_pin) ;
    y_value = analogRead (y_pin) ;
    z_value = analogRead (z_pin) ;

    if (samples == 0)
    {
      x_min = x_max = x_value ;
      y_min = y_max = y_value ;
      z_min = z_max = z_value ;
    }
    else
    {
      if (x_value < x_min) x_min = x_value ;
      if (y_value < y_min) y_min = y_value ;
      if (z_value < z_min) z_min = z_value ;

      if (x_value > x_max) x_max = x_value ;
      if (y_value > y_max) y_max = y_value ;
      if (z_value > z_max) z_max = z_value ;
    }
    x_total = x_total + x_value ;
    y_total = y_total + y_value ;
    z_total = z_total + z_value ;

    if (interval == 0)
      start_time = millis () ;
    else
    {
      start_time = start_time + interval ;
      int nap = start_time - millis() ;
      if (nap > 0)
        delay (nap) ;
    }
    samples++ ;
  }

  /* now write our results into the int array */

  i_results[0] = samples ;
  i_results[1] = x_min ;
  i_results[2] = x_total / samples ;
  i_results[3] = x_max ;
  i_results[4] = y_min ;
  i_results[5] = y_total / samples ;
  i_results[6] = y_max ;
  i_results[7] = z_min ;
  i_results[8] = z_total / samples ;
  i_results[9] = z_max ;
}

/* ===================== */
/* thread safe functions */
/* ===================== */

void ft_counter (S_thread_entry *p)
{
  /* determine our parameters from "in_args" */

  if (p->num_args != 2)
  {
    strcpy (p->msg, "FATAL! Expecting 2x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }
  int delay_ms = atoi (p->in_args[0]) ;

  /* if "loops" is 0, this is our first call, initialize stuff */

  if (p->loops == 0)
  {
    p->num_int_results = 1 ;
    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = "myType" ;
    p->results[0].data[0] = "\"myCounter\"" ;
    p->results[0].i_value = atoi (p->in_args[1]) ;
    strcpy (p->msg, "ok") ;
  }

  /*
     We want to call f_delivery() every REPORT_INTERVAL millisecs. Calculate
     how often this will happen based on "delay_ms".
  */

  #define REPORT_INTERVAL 10 * 1000
  int num_cycles = REPORT_INTERVAL / delay_ms ;
  if (p->results[0].i_value % num_cycles == 0)
  {
    char s[BUF_SIZE] ;
    sprintf (s, "counter:%d", p->results[0].i_value) ;
    f_delivery (p->name, s) ;
  }

  /* at this point, just increment our counter and take a break. */

  p->results[0].i_value++ ;
  delay (delay_ms) ;
}

/*
   This function writes the analogRead value in p->results[0].i_value but
   also uses p->results[1].i_value to store our next f_delivery() time.
*/

void ft_aread (S_thread_entry *p)
{
  /* get ready our configuration */

  if (p->num_args != 3)
  {
    strcpy (p->msg, "FATAL! Expecting 3x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }
  int delay_ms = atoi (p->in_args[0]) ;
  int postInterval_ms = atoi (p->in_args[1]) ;
  int pin = atoi (p->in_args[2]) ;

  /* if "loops" is 0, this is our first call, initialize stuff */

  if (p->loops == 0)
  {
    p->num_int_results = 1 ;
    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = "pin" ;
    p->results[0].data[0] = p->in_args[2] ;
    p->results[1].i_value = millis () + postInterval_ms ;
    strcpy (p->msg, "ok") ;
  }

  pinMode (pin, INPUT) ;
  p->results[0].i_value = analogRead (pin) ;

  /* check if it's time to do f_delivery(), postInterval of "0" disables */

  if ((postInterval_ms > 0) && (millis() > p->results[1].i_value))
  {
    char s[BUF_SIZE] ;
    sprintf (s, "%d", p->results[0].i_value) ;
    f_delivery (p->name, s) ;
    p->results[1].i_value = p->results[1].i_value + postInterval_ms ;
  }

  delay (delay_ms) ;
}

void ft_adxl335 (S_thread_entry *p)
{
  /* get ready our configuration */

  if (p->num_args != 6)
  {
    strcpy (p->msg, "FATAL! Expecting 6x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }
  if (p->loops == 0)
  {
    int pwrPin = atoi (p->in_args[5]) ;
    pinMode (pwrPin, OUTPUT) ;
    digitalWrite (pwrPin, HIGH) ;
    delay (50) ;

    p->num_int_results = 9 ;

    p->results[0].num_tags = 2 ;
    p->results[0].meta[0] = "axis" ;
    p->results[0].data[0] = "\"x\"" ;
    p->results[0].meta[1] = "type" ;
    p->results[0].data[1] = "\"Min\"" ;

    p->results[1].num_tags = 2 ;
    p->results[1].meta[0] = "axis" ;
    p->results[1].data[0] = "\"x\"" ;
    p->results[1].meta[1] = "type" ;
    p->results[1].data[1] = "\"Ave\"" ;

    p->results[2].num_tags = 2 ;
    p->results[2].meta[0] = "axis" ;
    p->results[2].data[0] = "\"x\"" ;
    p->results[2].meta[1] = "type" ;
    p->results[2].data[1] = "\"Max\"" ;

    p->results[3].num_tags = 2 ;
    p->results[3].meta[0] = "axis" ;
    p->results[3].data[0] = "\"y\"" ;
    p->results[3].meta[1] = "type" ;
    p->results[3].data[1] = "\"Min\"" ;

    p->results[4].num_tags = 2 ;
    p->results[4].meta[0] = "axis" ;
    p->results[4].data[0] = "\"y\"" ;
    p->results[4].meta[1] = "type" ;
    p->results[4].data[1] = "\"Ave\"" ;

    p->results[5].num_tags = 2 ;
    p->results[5].meta[0] = "axis" ;
    p->results[5].data[0] = "\"y\"" ;
    p->results[5].meta[1] = "type" ;
    p->results[5].data[1] = "\"Max\"" ;

    p->results[6].num_tags = 2 ;
    p->results[6].meta[0] = "axis" ;
    p->results[6].data[0] = "\"z\"" ;
    p->results[6].meta[1] = "type" ;
    p->results[6].data[1] = "\"Min\"" ;

    p->results[7].num_tags = 2 ;
    p->results[7].meta[0] = "axis" ;
    p->results[7].data[0] = "\"z\"" ;
    p->results[7].meta[1] = "type" ;
    p->results[7].data[1] = "\"Ave\"" ;

    p->results[8].num_tags = 2 ;
    p->results[8].meta[0] = "axis" ;
    p->results[8].data[0] = "\"z\"" ;
    p->results[8].meta[1] = "type" ;
    p->results[8].data[1] = "\"Max\"" ;

    strcpy (p->msg, "ok") ;
  }

  /* parse config from "p->in_args", prepare them for f_adxl335() */

  char *t[6], s[BUF_SIZE] ;
  t[0] = NULL ;                 // don't care
  t[1] = p->in_args[2] ;        // x pin
  t[2] = p->in_args[3] ;        // y pin
  t[3] = p->in_args[4] ;        // z pin
  t[4] = p->in_args[1] ;        // total duration (ms)
  t[5] = p->in_args[0] ;        // interval (ms)

  int r[10] ;
  f_adxl335 (t, (int*) &r) ;

  /* place results from "r" into "results" array */

  p->results[0].i_value = r[1] ;
  p->results[1].i_value = r[2] ;
  p->results[2].i_value = r[3] ;
  p->results[3].i_value = r[4] ;
  p->results[4].i_value = r[5] ;
  p->results[5].i_value = r[6] ;
  p->results[6].i_value = r[7] ;
  p->results[7].i_value = r[8] ;
  p->results[8].i_value = r[9] ;

  if (strcmp(p->in_args[1], "0") != 0)  // if "postInterval" is non-zero
  {
    sprintf (s, "samples:%d x:%d/%d/%d y:%d/%d/%d z:%d/%d/%d",
             r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9]) ;
    f_delivery (p->name, s) ;
  }
}

/* =========================== */
/* thread management functions */
/* =========================== */

void *f_thread_lifecycle (void *p)
{
  int i ;
  S_thread_entry *entry = (S_thread_entry*) p ;

  /* initialize thread info, indicate thread is now running */

  pthread_mutex_lock (&entry->lock) ;
  entry->state = THREAD_RUNNING ;
  pthread_mutex_unlock (&entry->lock) ;

  while (entry->state == THREAD_RUNNING)
  {
    entry->ft_addr (entry) ;
    entry->loops++ ;
  }

  pthread_mutex_lock (&entry->lock) ;
  entry->state = THREAD_STOPPED ;
  pthread_mutex_unlock (&entry->lock) ;
}

void f_thread_create (char *name)
{
  int idx ;
  char line[BUF_SIZE] ;

  /* before we try creating threads, try reap dead ones (releases memory) */

  for (idx=0 ; idx < MAX_THREADS ; idx++)
    if (G_thread_entry[idx].state == THREAD_STOPPED)
    {
      pthread_mutex_lock (&G_thread_entry[idx].lock) ;
      pthread_join (G_thread_entry[idx].tid, NULL) ;
      G_thread_entry[idx].state = THREAD_READY;
      G_thread_entry[idx].name[0] = 0 ;
      pthread_mutex_unlock (&G_thread_entry[idx].lock) ;
    }

  /* do validations first */

  if (strlen(name) > MAX_THREAD_NAME)
  {
    sprintf (line, "FAULT: thread name is too long, %d bytes max.\r\n",
             MAX_THREAD_NAME) ;
    strcat (G_reply_buf, line) ;
    return ;
  }

  for (idx=0 ; idx < MAX_THREADS ; idx++)
    if (strcmp (G_thread_entry[idx].name, name) == 0)
    {
      strcat (G_reply_buf, "FAULT: Duplicate thread name.\r\n") ;
      return ;
    }

  /*
      try read configuration from file, hold it in "config" for now. we'll
      copy it into G_thread_entry[].conf after all validation is complete.
   */

  char filename[BUF_SIZE], config[MAX_THREAD_CONF_BUF] ;
  sprintf (filename, "/thread-%s", name) ;
  File f = SPIFFS.open (filename, "r") ;
  int amt = f.readBytes (config, MAX_THREAD_CONF_BUF-1) ;
  f.close () ;
  if (amt < 1)
  {
    sprintf (line, "FAULT: Cannot read config file '%s'.\r\n", filename) ;
    strcat (G_reply_buf, line) ;
    return ;
  }
  config[amt] = 0 ;

  /* start by looking for a free G_thread_entry */

  for (idx=0 ; idx < MAX_THREADS ; idx++)
    if (G_thread_entry[idx].state == THREAD_READY)
      break ;
  if (idx == MAX_THREADS)
  {
    strcat (G_reply_buf, "Maximum number of threads reached.\r\n") ;
    return ;
  }

  /* found an available G_thread_entry, lock it, update and fire it up */

  pthread_mutex_lock (&G_thread_entry[idx].lock) ; /* START CRITICAL SECTION */

  /*
     try read optional tags config file, parse it in G_thread_entry[].tags,
     and have various pointers point into this buffer.
  */

  G_thread_entry[idx].tags_buf[0] = 0 ;
  G_thread_entry[idx].metric = NULL ;
  memset (G_thread_entry[idx].tags, 0, sizeof(char*) * MAX_THREAD_TAGS) ;

  sprintf (filename, "/tags-%s", name) ;
  f = SPIFFS.open (filename, "r") ;
  if (f != NULL)
  {
    int amt = f.readBytes (G_thread_entry[idx].tags_buf,
                           MAX_THREAD_TAGS_BUF-1) ;
    if (amt > 0)
    {
      G_thread_entry[idx].tags_buf[amt] = 0 ;
      G_thread_entry[idx].metric = strtok (G_thread_entry[idx].tags_buf, ",") ;
      int i ;
      for (i=0 ; i < MAX_THREAD_TAGS-1 ; i++)
      {
        char *p = strtok (NULL, ",") ;
        if (p)
          G_thread_entry[idx].tags[i] = p ;
        else
          break ;
      }
    }
    f.close () ;
  }

  /* initialize data structure fields */

  strcpy (G_thread_entry[idx].name, name) ;
  strcpy (G_thread_entry[idx].conf, config) ;
  G_thread_entry[idx].num_args = 0 ;
  G_thread_entry[idx].num_int_results = 0 ;
  G_thread_entry[idx].loops = 0 ;
  G_thread_entry[idx].ts_started = millis () ;
  G_thread_entry[idx].msg[0] = 0 ;
  G_thread_entry[idx].ft_addr = NULL ;
  memset (&G_thread_entry[idx].in_args, 0, MAX_THREAD_ARGS * sizeof(char*)) ;
  memset (&G_thread_entry[idx].results, 0,
          MAX_THREAD_RESULT_VALUES * sizeof(S_thread_result)) ;

  /* tokenize thread's configuration */

  int num_args ;
  char *ft_taskname = strtok (G_thread_entry[idx].conf, ",") ;
  for (num_args=0 ; num_args < MAX_THREAD_ARGS ; num_args++)
  {
    G_thread_entry[idx].in_args[num_args] = strtok (NULL, ",") ;
    if (G_thread_entry[idx].in_args[num_args] == NULL)
      break ;
  }
  G_thread_entry[idx].num_args = num_args ;

  /* figure out the ft_<task> function address */

  if (strcmp (ft_taskname, "ft_counter") == 0)
    G_thread_entry[idx].ft_addr = ft_counter ;
  if (strcmp (ft_taskname, "ft_aread") == 0)
    G_thread_entry[idx].ft_addr = ft_aread ;
  if (strcmp (ft_taskname, "ft_adxl335") == 0)
    G_thread_entry[idx].ft_addr = ft_adxl335 ;

  if (G_thread_entry[idx].ft_addr == NULL)
  {
    G_thread_entry[idx].name[0] = 0 ;
    sprintf (line, "FAULT: no such ft_task '%s'.\r\n", ft_taskname) ;
    strcat (G_reply_buf, line) ;
    pthread_mutex_unlock (&G_thread_entry[idx].lock) ;
    return ;
  }

  /* (almost) everything prep'ed ... finally create the thread */

  pthread_t tid ;
  G_thread_entry[idx].state = THREAD_STARTING ;
  pthread_create (&tid, NULL, f_thread_lifecycle,
                  (void*) &G_thread_entry[idx]) ;
  G_thread_entry[idx].tid = tid ;

  pthread_mutex_unlock (&G_thread_entry[idx].lock) ; /* END CRITICAL SECTION */

  sprintf (line, "thread '%s' created with tid:%d in_args:%d\r\n",
           name, tid, num_args) ;
  strcat (G_reply_buf, line) ;
}

void f_thread_stop (char *name)
{
  int idx ;
  char line[BUF_SIZE] ;

  /* let's see if this thread exists */

  for (idx=0 ; idx < MAX_THREADS ; idx++)
    if ((G_thread_entry[idx].state == THREAD_RUNNING) &&
        (strcmp(G_thread_entry[idx].name, name) == 0))
    {
      G_thread_entry[idx].state = THREAD_WRAPUP ;
      pthread_join (G_thread_entry[idx].tid, NULL) ;
      sprintf (line, "tid:%d '%s' stopped.\r\n",
               G_thread_entry[idx].tid, name) ;
      strcat (G_reply_buf, line) ;
      return ;
    }

  sprintf (line, "Thread '%s' not currently running.\r\n", name) ;
  strcat (G_reply_buf, line) ;
}

void f_esp32 (char **tokens)
{
  char msg[BUF_SIZE] ;

  /* we definitely need at least 2x tokens, otherwise something is wrong */

  if (tokens[1] == NULL)
  {
    strcat (G_reply_buf, "FAULT: Invalid esp32 command.\r\n") ;
    return ;
  }

  if (strcmp(tokens[1], "hall") == 0)                           // hall
  {
    sprintf (msg, "%d\r\n", hallRead()) ;
    strcat (G_reply_buf, msg) ;
  }
  else
  if ((strcmp(tokens[1], "sleep") == 0) && (tokens[2] != NULL))
  {
    unsigned long duration = atoi (tokens[2]) ; // in seconds
    esp_sleep_enable_timer_wakeup (duration * 1000 * 1000) ;
    sprintf (msg, "Sleeping for %d secs.\r\n", duration) ;
    strcat (G_reply_buf, msg) ;
    G_sleep = 1 ;
  }
  else
  if (strcmp(tokens[1], "thread_help") == 0)                    // thread_help
  {
    strcat (G_reply_buf,
      "[Thread Config Files]\r\n"
      "/thread-<name> - (see below)\r\n"
      "/tags-<name>   - <metric>[,<tagN>=<valueN>,...]\r\n"
      "\r\n"
      "[Currently available <ft_tasks>]\r\n"
      "ft_adxl335,<delay>,<post>,<xPin>,<yPin>,<zPin>,<pwrPin>\r\n"
      "ft_aread,<delay>,<post>,<pin>\r\n"
      "ft_counter,<delay_ms>,<start_value>\r\n"
      "\r\n"
      "[Notes]\r\n"
      "<delay> - interval between sampling (millisecs)\r\n"
      "<post>  - interval between aggregating results (millisecs)\r\n") ;
  }
  else
  if (strcmp(tokens[1], "thread_list") == 0)                    // thread_list
  {
    int i, num=1 ;
    unsigned long now = millis () ;
    for (i=0 ; i < MAX_THREADS ; i++)
      if ((G_thread_entry[i].state == THREAD_RUNNING) ||
          (G_thread_entry[i].state == THREAD_STOPPED))
      {
        char *state = "running" ;
        char *metric = "(none)" ;
        if (G_thread_entry[i].state == THREAD_STOPPED)
          state = "stopped" ;
        if (G_thread_entry[i].metric != NULL)
          metric = G_thread_entry[i].metric ;

        sprintf (msg, "%d. %s tid:%d %s age:%ld metric:%s msg:%s\r\n",
                 num, state,
                 G_thread_entry[i].tid,
                 G_thread_entry[i].name,
                 (now - G_thread_entry[i].ts_started) / 1000,
                 metric,
                 G_thread_entry[i].msg) ;
        strcat (G_reply_buf, msg) ;
        num++ ;
      }
  }
  else
  if ((strcmp(tokens[1], "thread_start") == 0) && (tokens[2] != NULL)) // start
  {
    f_thread_create (tokens[2]) ;
  }
  else
  if ((strcmp(tokens[1], "thread_stop") == 0) && (tokens[2] != NULL))  // stop
  {
    f_thread_stop (tokens[2]) ;
  }
  else
  {
    strcat (G_reply_buf, "FAULT: Invalid argument.\r\n") ;
  }
}

#endif

/* ------------------------------------------------------------------------- */

void f_action (char **tokens)
{
  char line[BUF_SIZE] ;

  if ((strcmp(tokens[0], "?") == 0) || (strcmp(tokens[0], "help") == 0))
  {
    strcat (G_reply_buf,
            "[Common]\r\n"
            "hi <GPIO pin>\r\n"
            "lo <GPIO pin>\r\n"
            "aread <pin> - analog read\r\n"
            "dread <GPIO pin> - digital read\r\n"
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
      strcat (G_reply_buf,
              "\r\n[Config Files]\r\n"
              "/mqtt.cfg   <host>,<port>,<user>,<pw>\r\n"
              "/mqtt.pub   <topic to publish>\r\n"
              "/mqtt.sub   <topic to subscribe>\r\n"
              "/udp.port   <port>\r\n"
              "/wifi.ssid  <ssid>\r\n"
              "/wifi.pw    <pw>\r\n") ;

      strcat (G_reply_buf,
              "\r\n[ESP8266 or ESP32]\r\n"
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
      strcat (G_reply_buf,
              "\r\n[ESP32 only]\r\n"
              "adxl335 <Xpin> <Ypin> <Zpin> <Time(ms)> <Interval(ms)>\r\n"
              "esp32 hall\r\n"
              "esp32 sleep <secs>\r\n"
              "esp32 thread_help\r\n"
              "esp32 thread_list\r\n"
              "esp32 thread_start <name>\r\n"
              "esp32 thread_stop <name>\r\n") ;
    #endif
  }
  else
  if ((strcmp(tokens[0], "hi") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, OUTPUT) ;
    digitalWrite (pin, HIGH) ;
    sprintf (line, "pin:%d HIGH\r\n", pin) ;
    strcat (G_reply_buf, line) ;
  }
  else
  if ((strcmp(tokens[0], "lo") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, OUTPUT) ;
    digitalWrite (pin, LOW) ;
    sprintf (line, "pin:%d LOW\r\n", pin) ;
    strcat (G_reply_buf, line) ;
  }
  else
  if ((strcmp(tokens[0], "aread") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, INPUT) ;
    int val = analogRead (pin) ;
    sprintf (line, "analogRead pin:%d - %d\r\n", pin, val) ;
    strcat (G_reply_buf, line) ;
  }
  else
  if ((strcmp(tokens[0], "dread") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, INPUT) ;
    int val = digitalRead (pin) ;
    sprintf (line, "digitalRead pin:%d - %d\r\n", pin, val) ;
    strcat (G_reply_buf, line) ;
  }
  else
  if (strcmp(tokens[0], "bmp180") == 0)
  {
    float t=0.0, p=0.0 ;
    if (f_bmp180 (&t, &p))
    {
      sprintf (line, "bmp180 - temperature:%d.%02d pressure:%d.%02d\r\n",
               int(t), (int)(t*100)%100, int(p), (int)(p*100)%100) ;
      strcat (G_reply_buf, line) ;
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
      strcat (G_reply_buf, line) ;
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
      strcat (G_reply_buf, line) ;
    }
  }
  else
  if (strcmp(tokens[0], "uptime") == 0)
  {
    unsigned long now = millis() / 1000 ;
    sprintf (line, "uptime - %ld secs\r\n", now) ;
    strcat (G_reply_buf, line) ;
  }
  else
  if (strcmp(tokens[0], "version") == 0)
  {
    sprintf (line, "Built: %s, %s\r\n", __DATE__, __TIME__) ;
    strcat (G_reply_buf, line) ;
    sprintf (line, "Data sizes: ptr:%d char:%d "
                   "short:%d int:%d long:%d longlong:%d "
                   "float:%d double:%d\r\n",
             sizeof(void*), sizeof(char),
             sizeof(short), sizeof(int), sizeof(long), sizeof(long long),
             sizeof(float), sizeof(double)) ;
    strcat (G_reply_buf, line) ;
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
  if ((strcmp(tokens[0], "adxl335") == 0) && (tokens[1] != NULL) &&
      (tokens[2] != NULL) && (tokens[3] != NULL) &&
      (tokens[4] != NULL) && (tokens[5] != NULL))
  {
    int r[10] ;
    f_adxl335 (tokens, (int*) &r) ;
    sprintf (line, "samples:%d x:%d/%d/%d y:%d/%d/%d z:%d/%d/%d\r\n",
             r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9]) ;
    strcat (G_reply_buf, line) ;
  }
  else
  if (strcmp(tokens[0], "esp32") == 0)
  {
    f_esp32 (tokens) ;
  }
  #endif
  else
  {
    strcat (G_reply_buf, "FAULT: Enter 'help' for commands.\r\n") ;
  }
}

/* ------------------------------------------------------------------------- */

void setup ()
{
  Wire.begin () ;
  Serial.begin (DEF_BAUD) ;
  Serial.setTimeout (SERIAL_TIMEOUT) ;
  Serial.println ("\nNOTICE: System boot.") ;
  G_serial_buf[0] = 0 ;
  G_serial_pos = 0 ;
  G_reply_buf = (char*) malloc (REPLY_SIZE+1) ;
  G_reply_buf[0] = 0 ;

  #if defined ARDUINO_ESP8266_NODEMCU || ARDUINO_ESP32_DEV

    cfg_wifi_ssid[0] = 0 ;
    cfg_wifi_pw[0] = 0 ;
    G_mqtt_pub[0] = 0 ;
    G_mqtt_sub[0] = 0 ;
    G_next_cron = CRON_INTERVAL * 1000 ;
    pinMode (LED_BUILTIN, OUTPUT) ;
    G_thread_entry = (S_thread_entry*) malloc (sizeof(S_thread_entry) *
                                               MAX_THREADS) ;

    /*
       if wifi ssid and password are available, try connect to wifi now.
       if udp server port is defined, configure it now.
    */

    char line[BUF_SIZE] ;

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
            G_Udp.begin (cfg_udp_port) ;
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
    else
    {
      Serial.println ("WARNING: Could not initialize SPIFFS.") ;
    }

    Webs.on ("/", f_handleWeb) ;
    Webs.on ("/v1", f_v1api) ;
    Webs.on ("/metrics", f_handleWebMetrics) ;
    Webs.begin () ;
    sprintf (line, "NOTICE: Web server started on port %d.", WEB_PORT) ;
    Serial.println (line) ;

    #if defined ARDUINO_ESP32_DEV
     int i ;
     for (i=0 ; i < MAX_THREADS ; i++)
       memset (&G_thread_entry[i], 0, sizeof(S_thread_entry)) ;
       pthread_mutex_init (&G_thread_entry[i].lock, NULL) ;
     sprintf (line, "NOTICE: Max allowed threads %d.", MAX_THREADS) ;
     Serial.println (line) ;
     pthread_mutex_init (&G_delivery_lock, NULL) ;
    #endif

    digitalWrite (LED_BUILTIN, LOW) ;           // blink to indicate boot.
    delay (1000) ;
    digitalWrite (LED_BUILTIN, HIGH) ;
  #endif

  memset (&G_Metrics, 0, sizeof(G_Metrics)) ;
  Serial.println ("NOTICE: Ready.") ;
}

void loop ()
{
  /*
     see if serial data arrived, wait for '\r' (minicom sends this when user
     presses ENTER)
  */

  while ((Serial.available() > 0) && (G_serial_pos < BUF_SIZE - 1))
  {
    char c = (char) Serial.read () ;
    G_Metrics.serialInBytes++ ;
    if ((c == '\b') && (G_serial_pos > 0))              // delete previous char
    {
      Serial.print (c) ;
      Serial.print (" ") ;
      Serial.print (c) ;
      G_serial_pos-- ;
      G_serial_buf[G_serial_pos] = 0 ;
    }
    if (c != '\b')                                      // add non-BS char
    {
      G_serial_buf[G_serial_pos] = c ;
      G_serial_buf[G_serial_pos+1] = 0 ;
      Serial.print (c) ;
    }

    if (c == '\r')
    {
      G_Metrics.serialCmds++ ;
      break ;
    }

    if ((c != '\r') && (c != '\b'))
      G_serial_pos++ ;
  }
  if (G_serial_pos == BUF_SIZE-1)                        // buffer overrun
  {
    Serial.println ("FAULT: Input overrun.") ;
    G_Metrics.serialOverruns++ ;
    G_serial_pos = 0 ;
    G_serial_buf[0] = 0 ;
  }

  if (G_serial_buf[G_serial_pos] == '\r')
  {
    Serial.print ("\n") ;
    G_serial_buf[G_serial_pos] = 0 ;
    int idx = 0 ;
    char *tokens[MAX_TOKENS] ;
    char *p = strtok (G_serial_buf, " ") ;
    while ((p) && (idx < MAX_TOKENS))
    {
      tokens[idx] = p ;
      idx++ ;
      p = strtok (NULL, " ") ;
    }
    tokens[idx] = NULL ;

    /*
       now that we've tokenized "G_serial_buf", have f_action() do something,
       which places the response in "G_reply_buf". Always print an "OK" on a
       new line to indicate that the previous command has completed.
    */

    G_reply_buf[0] = 0 ;
    if (tokens[0] != NULL)
      f_action (tokens) ;
    if (strlen(G_reply_buf) > 0)
    {
      Serial.print (G_reply_buf) ;
      if (G_reply_buf[strlen(G_reply_buf)-1] != '\n')
        Serial.print ("\r\n") ;                         // add CRNL if needed
    }
    Serial.println ("OK") ;
    G_serial_buf[0] = 0 ;
    G_serial_pos = 0 ;
  }

  #if defined ARDUINO_ESP8266_NODEMCU || ARDUINO_ESP32_DEV

  int pktsize = G_Udp.parsePacket () ;
  if (pktsize)
  {
    /* if a UDP packet arrived, parse the command and send a response */

    char udp_buf[BUF_SIZE] ;
    int amt = G_Udp.read (udp_buf, BUF_SIZE) ;
    if (amt > 0)
    {
      udp_buf[amt] = 0 ;
      G_Metrics.udpInBytes = G_Metrics.udpInBytes + amt ;
      G_Metrics.udpCmds++ ;

      int idx = 0 ;
      char *tokens[MAX_TOKENS] ;
      char *p = strtok (udp_buf, " ") ;
      while ((p) && (idx < MAX_TOKENS))
      {
        tokens[idx] = p ;
        idx++ ;
        p = strtok (NULL, " ") ;
      }
      tokens[idx] = NULL ;
      G_reply_buf[0] = 0 ;
      if (idx > 0)
        f_action (tokens) ;

      G_Udp.beginPacket (G_Udp.remoteIP(), G_Udp.remotePort()) ;
      G_Udp.write ((uint8_t*)G_reply_buf, strlen(G_reply_buf)) ;
      G_Udp.endPacket () ;
    }
  }

  Webs.handleClient () ;        // handle http requests
  if (G_psClient.connected())   // handle incoming messages & keepalives
    G_psClient.loop () ;

  /* once in a while, blink once if wifi is connected, twice otherwise. */

  unsigned long now = millis() ;
  if (now > G_next_blink)
  {
    if (WiFi.status() == WL_CONNECTED)
      f_blink (1) ;
    else
      f_blink (2) ;
    G_next_blink = G_next_blink + BLINK_FREQ ;
  }

  /* once in a while, run "cron" jobs, or responds to connection request */

  if ((now > G_next_cron) || (G_req_connect))
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      char *args[3] ;
      args[0] = "wifi" ;
      args[1] = "connect" ;
      args[2] = NULL ;
      f_wifi (args) ;
    }
    if (G_psClient.connected() == false)
    {
      f_mqtt_connect () ;
    }
    G_req_connect = 0 ;
    G_next_cron = G_next_cron + (CRON_INTERVAL * 1000) ;
  }

  #endif

  delay (10) ; // mandatory sleep to prevent excessive power drain

  #ifdef ARDUINO_ESP32_DEV
    if (G_sleep)
    {
      G_sleep = 0 ;
      Serial.println ("NOTICE: entering sleep mode.") ;
      delay (100) ; // allow serial buffer to flush
      esp_light_sleep_start () ;
      Serial.println ("NOTICE: exiting sleep mode.") ;
    }
  #endif
}

