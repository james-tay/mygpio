/*
   Build/Upload

     % URL="https://dl.espressif.com/dl/package_esp32_index.json"
     % arduino-cli core update-index --additional-urls $URL
     % arduino-cli core install esp32:esp32 --additional-urls $URL

     % arduino-cli lib install ArduinoOTA               # version 1.0.5
     % arduino-cli lib install PubSubClient             # version 2.8
     % arduino-cli lib install DallasTemperature        # version 3.9.0
     % arduino-cli lib install "LiquidCrystal I2C"      # version 1.1.2

     % arduino-cli compile -b esp32:esp32:esp32 .
     % arduino-cli upload -v -p /dev/ttyUSB0 -b esp32:esp32:esp32 .

   Examples

     [ serial CLI ]

     test LED,
       hi 12
       lo 12

     test light sensor (on 3.3v),
       hi 25
       aread 33
       lo 5

     test capacitive touch pin T6 (ie, GPIO 14)
       tread 14

     test HC-SR04 (needs 5v),
       hi 1
       hcsr04 18 19
       lo 1

     test DHT22,
       hi 14
       dht22 12
       lo 14

     test BMP180,
       bmp180

     test ADXL335,
       hi 23
       adxl335 36 39 34 1000 50
       lo 23

     test wifi client
       wifi ssid superman
       wifi pw changeme
       wifi connect

     test wifi boot up config
       fs write /wifi.ssid superman
       fs write /wifi.pw changeme

     test thread creation
       fs write /thread-button1 ft_tread,50,14,50,60
       fs write /tags-button1 sensor_button,location="Porch"
       esp32 thread_start button1

     test PWM
       tone 15 2000 20

     [ REST ]

     % curl 'http://esp32.example.com/metrics'
     % curl 'http://esp32.example.com/v1?cmd=...'

     eg,

     % curl 'http://esp32.example.com/v1?cmd=wifi+status'

     % URL="http://vip-web.drowningfrog.homenet.org/mygpio.ino.bin"
     % curl "http://esp32-1/v1?cmd=ota+$URL"

   Bugs

     - The "wifi {ssid|pw} <value>" command does not support arguments with
       spaces.
     - The "fs write <filename> <content>" command does not support content
       with spaces.
     - All filenames must begin with '/'.
     - LCD row/columns should be configurable.
     - REST blocks during firmware OTA upgrades.
     - I2S DMA buffer size should be dynamic.

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
     - The number of thread results are stored in "num_int_results" or
       "num_float_results". One (or both) of these MUST be zero.
     - Arguments to ft_<task> must be stored in a file "/thread-<name>",
       which are read by f_thread_create(). This is the user's responsibility.
     - This file must contain 1 line with comma separated parameters :
         <ft_task>[,<arg>,...]
       Eg,
         ft_counter,200,1
     - The first argument is the ft_<task> that f_thread_lifecycle() will
       execute, the specified arguments are all passed to ft_<task> as a
       "num_args" array of (char*).
     - Thread management (ie, create/stop/list) is single threaded and happens
       in the main thread.
     - Threads can be started on the first f_cron() run by specifying their
       names (comma separated) in "/autoexec.cfg".

   Pin Flags

     - Typically, sensors polled by individual threads have a dedicated "power"
       pin and the sensor is momentarily powered on when needed. If 2 or more
       sensors have a shared "power" pin, then one thread may power off the
       power rail when another thread is about to make a reading. Using a
       shared lock on the power pin may not be good because a thread powering
       down and another thread immediately powering up the power rail may lead
       to sensor instability. Thus, it is best to leave the power pin high
       until the next reboot.

     - This functionality is automatically implemented with the global array
       called "G_pin_flags" which tracks pin usage. This array can be inspected
       from the "pinflags" command.

     - Any thread accessing "G_pin_flags" (read or write) in a non-atomic
       manner, must first acquire "G_pinflags_lock". This is especially
       important during concurrent ft_<task> creation.

     - Any ft_<task> controlling a power pin, must call f_register_power_pin()
       as early as possible, and must NOT power it off if the "is_shared_power"
       flag is set.

   Writing ft_<tasks>

     a) write function code : "void ft_<task> (S_thread_entry *p)"
       - ft_<task> will be called repeatedly by f_thread_lifecycle()
       - ft_<task> should introduce its own delay to limit cpu load
     b) document call and its arguments in "thread_help"
     c) update f_thread_create() to identify function address
     d) a thread may set its state to THREAD_STOPPED if it chooses to
        terminate early (eg, encountering a critical error state).
     e) the ft_<task> body generally consists of the sections:
       - check p->num_args and parse the string arguments
       - do one time initialization if p->loops is zero.
         - set p->results[].{num_tags,meta[],data[]} fields if needed
       - perform the task
         - call f_delivery() if needed
       - set p->num_{int|float}_results} to indicate success or failure
       - add an appropriate delay()
       - if its p->state is set to THREAD_WRAPUP, the thread has exactly 1
         second to exit cleanly before being terminated.

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

         sensor_moisture,location="FlowerBed",model="resistance"

     - Thus, the format for the "/tags-<name>" file is (note the doublequotes):

         <metric>[,<tagN>="<valueN>",...]

     - A thread may insert its own tags in its "results" structure. These
       custom tags will be merged with those in "/tags-<name>" provided they
       don't exceed MAX_THREAD_RESULT_TAGS. If the "/tags-<name>" file is
       absent, "<name>" will be used in place of "<metric>".

   Publishing/Subscribing events over MQTT

     - The global "G_psClient" object handles all MQTT work.
     - A thread may deliver event data by calling f_delivery(), which places
       the message into "G_pub_topic" and "G_pub_payload" respectively.
     - The f_delivery() function prepares "<metric>" for delivery via MQTT to
       a topic with the format: "<publish_prefix>/<hostname>/<thread_name>"
     - In order to present metrics published over MQTT in a manner that's
       similar to prometheus scrapes, we insert the contents of MQTT_TAGS_FILE
       in each metric. This file typically has the content :
         instance="foo:80",job="bar"
     - Each EC subscribes to a topic "<subscribe_prefix>/<hostname>" and awaits
       commands sent to it.
     - Command responses are published to "<publish_prefix>/<hostname>".
     - the main thread performs the actual MQTT publish since this library may
       not be thread safe. G_publish_lock controls access to G_pub_topic and
       G_pub_payload.
     - commands delivered to an EC via MQTT must be in the format :
         [<tag>|]<command...>
       where "<tag>" is optional (note the vertical bar separator). Eg,
         foo|hi 23
     - responses from commands are pubished with the format :
         [<tag>|]<output...>
       where "<tag>" was specified in the command. This allows confirmation
       of command execution from the command originator's prospective.
     - note that this "<tag>" only applies to MQTT commands and CANNOT be used
       in the REST interface.

   Wifi Connectivity

     - if we're connected to a particular wifi AP, and that wifi AP's radio
       goes down, our wifi status is immediately reported as :

         cfg_wifi_ssid: Cloud-A
         cfg_wifi_pw: (set)
         status: WL_CONNECTED
         rssi: 0 dBm
         wifi_mac: 24:62:ab:fc:3f:9c
         wifi_ip: 0.0.0.0/0.0.0.0
         bssid:
         mqtt_state: MQTT_CONNECTION_LOST
         hostname: esp32-1
         subscribed_topic: lawrence/ec/commands/esp32-1
         cmd_response_topic: lawrence/ec/response/esp32-1

       Note how the wifi status remains WL_CONNECTED. After about 100 secs,
       wifi automatically connects to the next AP with the same SSID.

   Hardware Serial

     - There are 3x hardware UARTs on an ESP32, but the first one cannot be
       used. Since the serial ports are not file descriptors, we cannot use
       select() or any other kind of multiplexing mechanism. We will be using
       Serial2() to interact with the 3rd hardware UART.

     - IO with the serial port is implemented by a dedicated thread. It
       creates a listening TCP socket but accepts a SINGLE client at any
       time. Data from the TCP client is sent to the UART and data from the
       UART is sent to the TCP client.

     - In order to coordinate with each other, all uart IO threads access a
       common data structure "G_hw_uart", which tracks usage of the serial
       port. This prevents multiple threads from unintentionally accessing
       the uart. The first thread to access the serial port MUST initialize
       the port (ie, set baud, pins, etc) and set "G_hw_uart->initialized".

     - Any thread that interacts with "G_hw_uart" (even reads) must first
       acquire G_hw_uart->lock, and also release it when no longer needed.

     - The "Serial2.begin()" function can only be called ONCE. Attempts to
       call it again results in a hard lock up. Thus any changes to serial
       port configuration (eg, pin changes, etc) must be followed by a system
       "reload".

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

     - Arduino OTA reference
       https://github.com/jandrassy/ArduinoOTA
       https://github.com/espressif/arduino-esp32/tree/master/libraries/Update/examples/AWS_S3_OTA_Update

   Bugs

     - tons of buffer overrun opportunities due to static buffers with not
       much buffer length checking.

     - SPIFFS is not thread safe and can only be called safely by the main
       thread.

     - the OTA implementation does not follow HTTP redirects.
*/

#include <soc/rtc.h>
#include <lwip/sockets.h>

#include <FS.h>
#include <Wire.h>                     // this is for I2C support
#include <WiFi.h>
#include <SPIFFS.h>
#include <Update.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <soc/i2s_reg.h>
#include <driver/i2s.h>

#include <OneWire.h>                  // 1-wire support, typically 16.3 kbps
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

#define MAX_SSID_LEN 32
#define MAX_PASSWD_LEN 32             // maximum wifi password length
#define MAX_MQTT_LEN 128              // maximum mqtt topic/message buffer
#define MAX_WIFI_TIMEOUT 60           // wifi connect timeout (secs)
#define MAX_SD_BACKLOG 4              // TCP listening socket backlog
#define WEB_PORT 80                   // web server listens on this port
#define CRON_INTERVAL 60              // how often we run f_cron()
#define RSSI_LOW_THRES -72            // less than this is consider low signal

#define WIFI_SSID_FILE "/wifi.ssid"
#define WIFI_PW_FILE "/wifi.pw"
#define MQTT_CFG_FILE "/mqtt.cfg"     // used in f_mqtt_connect()
#define MQTT_PUB_FILE "/mqtt.pub"     // loaded into "G_mqtt_pub"
#define MQTT_SUB_FILE "/mqtt.sub"     // loaded into "G_mqtt_sub"
#define MQTT_TAGS_FILE "/mqtt.tags"   // loaded into "G_mqtt_tags"
#define AUTOEXEC_FILE "/autoexec.cfg"
#define HOSTNAME_FILE "/hostname"

#define LED_BUILTIN 2                 // applies to KeeYees ESP32
#define BLINK_ON HIGH
#define BLINK_OFF LOW

#define MAX_THREADS 16
#define MAX_THREAD_STACK 4096
#define MAX_THREAD_NAME 40
#define MAX_THREAD_ARGS 8             // number of input args
#define MAX_THREAD_RESULT_TAGS 8      // meta data tags
#define MAX_THREAD_RESULT_VALUES 16   // output values
#define MAX_THREAD_CONF_BUF 80        // length of thread's "conf"
#define MAX_THREAD_TAGS_BUF 80        // length of thread's tags
#define MAX_THREAD_MSG_BUF 80         // length of thread's "msg"
#define MAX_THREAD_TAGS 8             // tag pairs in "/tags-<name>"
#define THREAD_SHUTDOWN_PERIOD 1000   // shutdown grace period (millisecs)

#define MAX_DS18B20_DEVICES 4         // maximum DS18B20 on a single pin

#define MAX_HTTP_REQUEST 1024
#define MAX_HTTP_CLIENTS 4            // esp32 supports 8x file descriptors
#define MAX_HTTP_RTIME 20             // seconds to receive an http request

/* my I2S settings */

#define MYI2S_BITS_PER_SAMPLE 32      // update i2s_config.bits_per_sample too
#define MYI2S_DMA_IN_BUFS 2           // number of DMA buffers sampling audio
#define MYI2S_DMA_OUT_BUFS 32         // number of DMA buffers outputing audio
#define MYI2S_DMA_SAMPLES 320         // samples per DMA buffer
#define MYI2S_DMA_WAITTICKS 100       // max ticks i2s_read() will wait
#define MYI2S_INPUT_PORT I2S_NUM_0    // audio input always uses port 0
#define MYI2S_OUTPUT_PORT I2S_NUM_1   // audio output always uses port 1

/* various states for S_thread_entry.state */

#define THREAD_READY          0       // ready to be started
#define THREAD_STARTING       1       // thread just got created
#define THREAD_RUNNING        2       // thread in f_thread_lifecycle()
#define THREAD_WRAPUP         3       // tell a thread to terminate
#define THREAD_STOPPED        4       // thread is as good as dead

/* ====== ALL OTHER GENERAL STUFF ====== */

#define FLOAT_DECIMAL_PLACES 6  // when printing float metrics
#define DEF_BAUD 9600           // USB serial port baud rate
#define SERIAL_TIMEOUT 1000     // serial timeout in milliseconds
#define BLINK_FREQ 5000         // blink to indicate we're alive (ms)
#define MAX_GPIO_PINS 40        // total number of GPIO pins in G_pin_flags[]
#define MAX_TOKENS 10
#define BUF_SIZE 80
#define BUF_MEDIUM 256
#define REPLY_SIZE 2048

/* LCD definitions */

#define LCD_ADDR 0x27
#define LCD_WIDTH 16
#define LCD_ROWS 2

/* ======= ALL GLOBAL VARIABLES ====== */

char cfg_wifi_ssid[MAX_SSID_LEN + 1] ;
char cfg_wifi_pw[MAX_PASSWD_LEN + 1] ;

int G_sleep = 0 ;               // a flag to tell us to enter sleep mode
int G_sd = 0 ;                  // web server's socket descriptor
char *G_mqtt_pub ;              // mqtt topic prefix we publish to
char *G_mqtt_sub ;              // mqtt topic prefix we subscribe to
char *G_mqtt_tags ;             // tags included in each publish
char *G_mqtt_stopic ;           // the (command) topic we subscribe to
char *G_mqtt_rtopic ;           // the (command) topic we respond to
char *G_pub_topic ;             // topic prefix we publish sensor data to
char *G_pub_payload ;           // current payload we will publish

char *G_reply_buf ;             // accumulate our reply message here
int G_serial_pos ;              // index of bytes received on serial port
int G_debug ;                   // global debug level
int G_reboot=0 ;                // setting this to 1 triggers a reboot
char *G_serial_buf ;            // accumulate butes on our serial console
char *G_hostname ;              // our hostname

unsigned long G_next_cron ;        // millis() time of next run
unsigned long G_next_blink=BLINK_FREQ ; // "wall clock" time for next blink
SemaphoreHandle_t G_publish_lock ;      // signal main thread to publish()

/* Data structure of a single thread result value (with multiple tags) */

struct thread_result_s
{
  int num_tags ;                      // number of meta data tags
  char *meta[MAX_THREAD_RESULT_TAGS] ;  // array of meta tags
  char *data[MAX_THREAD_RESULT_TAGS] ;  // array of data tags
  int i_value ;                       // this result's value
  double f_value ;                    // this result's value
} ;
typedef struct thread_result_s S_thread_result ;

/* Data structure to track a single thread, as well as its in/out data */

struct thread_entry_s
{
  TaskHandle_t tid ;
  char name[MAX_THREAD_NAME] ;
  unsigned char state ;
  SemaphoreHandle_t lock ;            // lock before making changes here

  /* input arguments to <ft_task> */

  int num_args ;                      // number of input arguments
  char *in_args[MAX_THREAD_ARGS] ;    // array of pointers into "conf"
  char conf[MAX_THREAD_CONF_BUF] ;    // main config buffer
  char tags_buf[MAX_THREAD_TAGS_BUF] ; // metric and tags (optional)
  char *metric ;                      // pointer into "tags_buf" (optional)
  char *tags[MAX_THREAD_TAGS+1] ;     // pointers into "tags_bufs" (optional)

  /* thread metadata */

  int core ;                          // cpu core ID thread runs on
  unsigned long loops ;               // number of <ft_task> calls so far
  unsigned long ts_started ;          // millis() timestamp of xTaskCreate()
  void (*ft_addr)(struct thread_entry_s*) ; // the <ft_task> this thread runs

  /* IMPORTANT !!! <ft_task> may modify anything below this point */

  int num_int_results ;               // number of "i_value" results returned
  int num_float_results ;             // number of "f_value" results returned
  S_thread_result results[MAX_THREAD_RESULT_VALUES] ;
  char msg[MAX_THREAD_MSG_BUF] ;      // provide some optional feedback
} ;
typedef struct thread_entry_s S_thread_entry ;

/* Data structure of a web client connection */

struct web_client
{
  int sd ;                              // the client's socket descriptor
  int req_pos ;                         // insertion point in "request"
  unsigned long connect_time ;          // millis() timestamp of connection
  char request[MAX_HTTP_REQUEST] ;      // http request from client
  char method[BUF_SIZE] ;               // http method from "request"
  char uri[BUF_MEDIUM] ;                // URI of http request
  char query[BUF_MEDIUM] ;              // arguments after "uri"
} ;
typedef struct web_client S_WebClient ;

/* internal performance metrics */

struct internal_metrics
{
  unsigned long cronRuns ;
  unsigned long serialInBytes ;
  unsigned long serialCmds ;
  unsigned long serialOverruns ;
  unsigned long restInBytes ;
  unsigned long restCmds ;
  unsigned long mqttConnects ;
  unsigned long mqttPubs ;
  unsigned long mqttSubs ;
  unsigned long mqttOversize ;
  unsigned long mqttPubWaits ;
  unsigned long wifiReconnects ;
} ;
typedef struct internal_metrics S_Metrics ;

/* data structure to track each GPIO pin's usage */

struct pin_flag
{
  unsigned char is_power_pin ;          // pin supplies power to a device
  unsigned char is_shared_power ;       // is a shared power pin
  S_thread_entry **users ;  // MAX_THREAD pointers to threads using this pin
} ;
typedef struct pin_flag S_pin_flag ;

/* data structure to track a hardware UART */

struct hw_uart
{
  int in_use ;                  // number of threads accessing UART
  int initialized ;             // track if "Serial2.begin()" has been called
  SemaphoreHandle_t lock ;      // lock before any access to this structure
} ;
typedef struct hw_uart S_hw_uart ;

S_pin_flag *G_pin_flags ;            // array of structs for each GPIO pin
SemaphoreHandle_t G_pinflags_lock ;  // lock before ANY access to "G_pin_flags"

S_Metrics *G_Metrics ;
S_thread_entry *G_thread_entry ;
S_WebClient *G_WebClient ;
S_hw_uart *G_hw_uart ;
LiquidCrystal_I2C G_lcd (LCD_ADDR, LCD_WIDTH, LCD_ROWS) ;

WiFiClient G_wClient ; // this is needed to instanciate "G_psClient"
PubSubClient G_psClient (G_wClient) ;

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

/*
   For some reason, the ESP32 does not always connect to the AP that is
   nearest (ie, strongest signal). This function performs a wifi scan and
   notes down the channel and BSSID of the nearest AP and then connects to
   it. On success, it returns 1, otherwise 0. It scan activity is written
   into "msg" as a 1-line string.
*/

int f_wifiConnect (char *ssid, char *pw, char *msg)
{
  int scan_chan=0, scan_rssi, scan_idx ;
  uint8_t *scan_bssid ;
  char scan_ssid[MAX_SSID_LEN+1] ;
  char line[BUF_SIZE] ;

  int n = WiFi.scanNetworks () ;
  for (int i=0 ; i < n ; i++)
  {
    WiFi.SSID(i).toCharArray(scan_ssid, MAX_SSID_LEN) ;
    if (strcmp(ssid, scan_ssid) == 0)
    {
      sprintf (line, "(%s ch:%d %ddBm) ",
               WiFi.BSSIDstr(i).c_str(), WiFi.channel(i), WiFi.RSSI(i)) ;
      strcat (msg, line) ;

      if (scan_chan == 0) /* this is our first matching AP, note it down */
      {
        scan_idx = i ;
        scan_rssi = WiFi.RSSI (i) ;
        scan_chan = WiFi.channel (i) ;
        scan_bssid = WiFi.BSSID (i) ;
      }
      else
      {
        if (WiFi.RSSI (i) > scan_rssi) /* check if this AP is nearer */
        {
          scan_idx = i ;
          scan_rssi = WiFi.RSSI (i) ;
          scan_chan = WiFi.channel (i) ;
          scan_bssid = WiFi.BSSID (i) ;
        }
      }
    }
  }

  if (scan_chan == 0) /* opsie, did not find requested AP */
  {
    strcpy (msg, "SSID not found") ;
    return (0) ;
  }

  sprintf (line, "[%s]", WiFi.BSSIDstr(scan_idx).c_str()) ;
  strcat (msg, line) ;
  WiFi.begin (ssid, pw, scan_chan, scan_bssid, true) ;
}

/*
   Sets the specified GPIO pin high. If an additional parameter is specified,
   then the pin stays high for that number of microseconds. Note that for 
   of "dur_usec" more than MAX_USEC, we'll use delay() instead. See :

  https://www.arduino.cc/reference/en/language/functions/time/delaymicroseconds/
*/

void f_hi (char **args)
{
  #define MAX_USEC 16000

  char line[BUF_SIZE] ;
  int pin = atoi (args[1]) ;
  int dur_usec = 0 ;
  if (args[2] != NULL)
    dur_usec = atoi (args[2]) ;

  pinMode (pin, OUTPUT) ;
  sprintf (line, "pin:%d HIGH\r\n", pin) ;
  digitalWrite (pin, HIGH) ;

  if (dur_usec > 0)
  {
    if (dur_usec < MAX_USEC)
      delayMicroseconds (dur_usec) ;
    else
      delay (dur_usec / 1000) ;

    digitalWrite (pin, LOW) ;
    sprintf (line, "pin:%d PULSED %d usecs\r\n", pin, dur_usec) ;
  }
  strcat (G_reply_buf, line) ;
}

/* ------------------------------------------------------------------------- */
/* "Drivers" for various sensor devices                                      */
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

  char buf[BUF_SIZE] ;
  memset (data, 0, 5) ;
  for (i=0 ; i<40 ; i++)
  {
    data[i/8] <<= 1 ;           // left shift bits, right most bit will be 0
    if (cycles[i] > 50)
    {
      if (G_debug)
      {
        snprintf (buf, BUF_SIZE, "DEBUG: bit:%d cycle:%d = 1", i, cycles[i]) ;
        Serial.println (buf) ;
      }
      data[i/8] |= 1 ;          // set right most bit to 1
    }
    else
    {
      if (G_debug)
      {
        snprintf (buf, BUF_SIZE, "DEBUG: bit:%d cycle:%d = 0", i, cycles[i]) ;
        Serial.println (buf) ;
      }
    }
  }

  /* validate checksum */

  unsigned char c = data[0] + data[1] + data[2] + data[3] ;
  if ((c & 0xff) != data[4])
  {
    if (G_debug)
    {
      snprintf (buf, BUF_SIZE, "DEBUG: bad checksum 0x%x, expecting 0x%x.",
                (int) c, (int) data[4]) ;
      Serial.println (buf) ;
    }
    strcat (G_reply_buf, "FAULT: f_dht22() checksum failed.\r\n") ;
    return (0) ;
  }

  float raw_temp = ((word)(data[2] & 0x7F)) << 8 | data[3] ;
  if (data[2] & 0x80)
    *temperature = raw_temp * -0.1 ;
  else
    *temperature = raw_temp * 0.1 ;

  *humidity = float (((int) data[0] << 8 ) | data[1]) / 10.0 ;
  return (1) ;
}

/*
   Polls DS18B20 devices on the specified 1-wire pin. Writes the current
   temperature in the supplied float buffer array and the device addresses in
   "addr" which must be (MAX_THREAD_RESULT_VALUES * 8) bytes long. Returns
   the number of DS18B20 devices successfully polled (ie, written into the
   supplied arrays)
*/

int f_ds18b20 (int dataPin, unsigned char *addr, float *temperature)
{
  #define DS18B20_RESOLUTION_BITS 12
  #define DS18B20_MAX_TEMP 85.0         // a reading this high is not normal
  #define DS18B20_MIN_TEMP -127.0       // a reading this low is not normal

  OneWire bus (dataPin) ;
  DallasTemperature sensor (&bus) ;

  sensor.begin () ;
  int devices = sensor.getDeviceCount() ;
  if (devices < 1)
  {
    strcat (G_reply_buf, "FAULT: no devices found.\r\n") ;
    return (0) ;
  }
  if (devices > MAX_DS18B20_DEVICES)
    devices = MAX_DS18B20_DEVICES ;

  int addr_offset = 0 ; // where in "addr" we're currently writing into
  int results = 0 ;     // number of successful addresses actually read
  unsigned char cur_addr[8] ;

  sensor.setResolution (DS18B20_RESOLUTION_BITS) ;
  sensor.requestTemperatures() ;
  for (int i=0 ; i < devices ; i++)
  {
    if (sensor.getAddress (cur_addr, i))
    {
      float f = sensor.getTempCByIndex (i) ;
      if ((f < DS18B20_MAX_TEMP) && (f > DS18B20_MIN_TEMP)) // sanity check
      {
        temperature[results] = f ;
        memcpy (addr+addr_offset, cur_addr, 8) ;
        addr_offset = addr_offset + 8 ;
        results++ ;
      }
    }
  }
  return (results) ;
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
    return (-1.0) ;
  else
  {
    float cm = float(echoUsecs) / 58.0 ;        // convert time to centimeters
    if (cm < 1.0)
      return (-1.0) ; // this is probably an invalid reading
    return (cm) ;
  }
}

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

/*
   look for "thread_name", set its "p->results[1].i_value" based on "state".
   Also, read its "p->results[0].i_value" to determine timeout, and then update
   its "p->results[2].i_value" to new shut off time.
*/

void f_relay (char *thread_name, char *state)
{
  int i ;

  if ((strcmp (state, "on") != 0) && (strcmp(state, "off") != 0))
  {
    strcpy (G_reply_buf, "FAULT: invalid state.\r\n") ;
    return ;
  }

  for (i=0 ; i < MAX_THREADS ; i++)
  {
    if ((G_thread_entry[i].state == THREAD_RUNNING) &&
        (strcmp (G_thread_entry[i].name, thread_name) == 0))
    {
      if (G_thread_entry[i].ft_addr != ft_relay)
      {
        strcpy (G_reply_buf, "FAULT: not an ft_relay() thread.\r\n") ;
        return ;
      }

      /* read the relay status: 0=off, 1=on, -1=fault */

      char line[BUF_SIZE] ;
      int cur_state = G_thread_entry[i].results[1].i_value ;
      if (cur_state < 0)
      {
        strcpy (G_reply_buf, "FAULT: relay in fault state.\r\n") ;
        return ;
      }

      if (strcmp(state, "on") == 0)
      {
        long now = millis () ;
        long expiry = (G_thread_entry[i].results[0].i_value * 1000) + now ;
        G_thread_entry[i].results[2].i_value = expiry ;
        G_thread_entry[i].results[1].i_value = 1 ; // request ON state
        sprintf (line, "Relay: %d->1\r\n", cur_state) ;

      }
      else
      {
        G_thread_entry[i].results[1].i_value = 0 ; // request OFF state
        sprintf (line, "Relay: %d->0\r\n", cur_state) ;
      }

      strcat (G_reply_buf, line) ;
      return ;
    }
  }

  /* if we're here, it means we didn't find "thread_name" */

  strcat (G_reply_buf, "FAULT: invalid thread name.\r\n") ;
}

/* ------------------------------------------------------------------------- */
/* System Management                                                         */
/* ------------------------------------------------------------------------- */

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
    sprintf (line, "totalBytes: %d\r\nusedBytes: %d\r\n",
             SPIFFS.totalBytes(), SPIFFS.usedBytes()) ;
    strcat (G_reply_buf, line) ;
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
    File root = SPIFFS.open ("/", "r") ;
    File f = root.openNextFile () ;
    while (f)
    {
      sprintf (line, "%-8d %s\r\n", f.size(), f.name()) ;
      strcat (G_reply_buf, line) ;
      f = root.openNextFile () ;
    }
    root.close () ;
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

void f_file (char **tokens)
{
  #define CLIENT_IO_TIMEOUT 10 // max idle(secs) for connect or data transfer

  if (((strcmp(tokens[1], "recv") != 0) && (strcmp(tokens[1], "send") != 0)) ||
      (tokens[2] == NULL) || (tokens[3] == NULL))
  {
    strcat (G_reply_buf, "FAULT: Invalid argument.\r\n") ;
    return ;
  }

  /* setup a listening TCP socket and expect a client to connect to it */

  int port = atoi (tokens[2]) ;
  char *path = tokens[3] ;
  int listen_sd = socket (AF_INET, SOCK_STREAM, IPPROTO_IP) ;
  if (listen_sd < 0)
  {
    strcat (G_reply_buf, "FATAL! socket() failed.\r\n") ;
    return ;
  }

  int reuse = 1 ; // allow quick re-bind()'ing of this port
  setsockopt (listen_sd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(int)) ;

  struct sockaddr_in addr ;
  memset (&addr, 0, sizeof(addr)) ;
  addr.sin_family = AF_INET ;
  addr.sin_addr.s_addr = INADDR_ANY ;
  addr.sin_port = htons (port) ;
  if (bind (listen_sd, (const struct sockaddr*) &addr, sizeof(addr)) < 0)
  {
    close (listen_sd) ;
    strcat (G_reply_buf, "FATAL! bind() failed.\r\n") ;
    return ;
  }
  if (listen (listen_sd, 1) != 0)
  {
    close (listen_sd) ;
    strcat (G_reply_buf, "FATAL! listen() failed.\r\n") ;
    return ;
  }

  struct timeval tv ;
  fd_set rfds ;
  tv.tv_sec = CLIENT_IO_TIMEOUT ;
  tv.tv_usec = 0 ;
  FD_ZERO (&rfds) ;
  FD_SET (listen_sd, &rfds) ;
  int result = select (listen_sd + 1, &rfds, NULL, NULL, &tv) ;
  if (result < 1)
  {
    close (listen_sd) ;
    strcat (G_reply_buf, "FATAL! No client connected.\r\n") ;
    return ;
  }
  int client_sd = accept (listen_sd, NULL, NULL) ;
  if (client_sd < 0)
  {
    close (listen_sd) ;
    strcat (G_reply_buf, "FATAL! accept() failed.\r\n") ;
    return ;
  }

  /*
     at this point, "client_sd" is connected, decide whether to send/recv,
     but *some* IO must occur within CLIENT_IO_TIMEOUT, or we'll close and
     abort the transfer.
  */

  int amt, total=0 ;
  char buf[BUF_MEDIUM] ;
  long start_time = millis () ;

  if (strcmp (tokens[1], "recv") == 0)                          // recv a file
  {
    File f = SPIFFS.open (path, "w") ;
    if (!f)
      strcat (G_reply_buf, "FATAL! Cannot write to file.\r\n") ;

    while (f)
    {
      tv.tv_sec = CLIENT_IO_TIMEOUT ;
      tv.tv_usec = 0 ;
      FD_ZERO (&rfds) ;
      FD_SET (client_sd, &rfds) ;
      result = select (client_sd + 1, &rfds, NULL, NULL, &tv) ;
      if (result < 1)
      {
        sprintf (buf, "FATAL! client timeout after %d bytes.\r\n", total) ;
        strcat (G_reply_buf, buf) ;
        break ;
      }
      amt = read (client_sd, buf, BUF_MEDIUM-1) ;
      if (amt < 1)
      {
        long duration_ms = millis() - start_time ;
        sprintf (buf, "Received %d bytes from client in %d ms.\r\n",
                total, duration_ms) ;
        strcat (G_reply_buf, buf) ;
        break ;
      }
      buf[amt] = 0 ;
      f.print (buf) ;
      total = total + amt ;
    }
    if (f)
      f.close () ;
  }

  if (strcmp (tokens[1], "send") == 0)                          // send file
  {
    File f = SPIFFS.open (path, "r") ;
    if (f)
    {
      while (total < f.size())
      {
        amt = f.readBytes (buf, BUF_SIZE) ; // this blocks if we past EOF !!
        if (amt > 0)
        {
          write (client_sd, buf, amt) ;
          total = total + amt ;
        }
        else
          break ;
      }
      long duration_ms = millis () - start_time ;
      sprintf (buf, "Sent %d bytes to client in %d ms.\r\n",
               total, duration_ms) ;
      strcat (G_reply_buf, buf) ;
      f.close () ;
    }
    else
      strcat (G_reply_buf, "FATAL! Cannot read from file.\r\n") ;
  }

  close (listen_sd) ;
  close (client_sd) ;
}

void f_wifi (char **tokens)
{
  char line[BUF_MEDIUM] ;

  if (strcmp(tokens[1], "scan") == 0)                           // scan
  {
    int n = WiFi.scanNetworks() ;
    sprintf (line, "Found %d wifi networks.\r\n", n) ;
    strcat (G_reply_buf, line) ;
    for (int i=0 ; i<n ; i++)
    {
      char ssid[MAX_SSID_LEN+1] ;
      WiFi.SSID(i).toCharArray (ssid, MAX_SSID_LEN) ;
      sprintf (line, "%2d. ch %d, %d dBm [%s] %s\r\n",
               i+1, WiFi.channel(i), WiFi.RSSI(i), ssid,
               WiFi.BSSIDstr(i).c_str()) ;
      if (strlen(G_reply_buf) + strlen(line) < REPLY_SIZE)
        strcat (G_reply_buf, line) ;
    }
  }
  else
  if (strcmp(tokens[1], "status") == 0)                         // status
  {
    snprintf (line, BUF_MEDIUM, "cfg_wifi_ssid: %s\r\n", cfg_wifi_ssid) ;
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
    snprintf (line, BUF_MEDIUM, "rssi: %d dBm\r\n", WiFi.RSSI()) ;
    strcat (G_reply_buf, line) ;

    unsigned char mac[6] ;
    WiFi.macAddress(mac) ;
    snprintf (line, BUF_MEDIUM, "wifi_mac: %x:%x:%x:%x:%x:%x\r\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]) ;
    strcat (G_reply_buf, line) ;
    snprintf (line, BUF_MEDIUM, "wifi_ip: %s/%s\r\n",
             WiFi.localIP().toString().c_str(),
             WiFi.subnetMask().toString().c_str()) ;
    strcat (G_reply_buf, line) ;
    snprintf (line, BUF_MEDIUM, "bssid: %s\r\n", WiFi.BSSIDstr().c_str()) ;
    strcat (G_reply_buf, line) ;

    strcat (G_reply_buf, "mqtt_state: ") ;
    switch (G_psClient.state())
    {
      case MQTT_CONNECTION_TIMEOUT:
        strcat (G_reply_buf, "MQTT_CONNECTION_TIMEOUT\r\n") ; break ;
      case MQTT_CONNECTION_LOST:
        strcat (G_reply_buf, "MQTT_CONNECTION_LOST\r\n") ; break ;
      case MQTT_CONNECT_FAILED:
        strcat (G_reply_buf, "MQTT_CONNECT_FAILED\r\n") ; break ;
      case MQTT_DISCONNECTED:
        strcat (G_reply_buf, "MQTT_DISCONNECTED\r\n") ; break ;
      case MQTT_CONNECTED:
        strcat (G_reply_buf, "MQTT_CONNECTED\r\n") ; break ;
      case MQTT_CONNECT_BAD_PROTOCOL:
        strcat (G_reply_buf, "MQTT_CONNECT_BAD_PROTOCOL\r\n") ; break ;
      case MQTT_CONNECT_BAD_CLIENT_ID:
        strcat (G_reply_buf, "MQTT_CONNECT_BAD_CLIENT_ID\r\n") ; break ;
      case MQTT_CONNECT_UNAVAILABLE:
        strcat (G_reply_buf, "MQTT_CONNECT_UNAVAILABLE\r\n") ; break ;
      case MQTT_CONNECT_BAD_CREDENTIALS:
        strcat (G_reply_buf, "MQTT_CONNECT_BAD_CREDENTIALS\r\n") ; break ;
      case MQTT_CONNECT_UNAUTHORIZED:
        strcat (G_reply_buf, "MQTT_CONNECT_UNAUTHORIZED\r\n") ; break ;
    }
    if (strlen(G_hostname) > 0)
    {
      snprintf (line, BUF_MEDIUM, "hostname: %s\r\n", G_hostname) ;
      strcat (G_reply_buf, line) ;
    }
    if (strlen(G_mqtt_stopic) > 0)
    {
      snprintf (line, BUF_MEDIUM, "subscribed_topic: %s\r\n", G_mqtt_stopic) ;
      strcat (G_reply_buf, line) ;
    }
    if (strlen(G_mqtt_rtopic) > 0)
    {
      snprintf (line, BUF_MEDIUM, "cmd_response_topic: %s\r\n", G_mqtt_rtopic) ;
      strcat (G_reply_buf, line) ;
    }
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
    f_wifiConnect (cfg_wifi_ssid, cfg_wifi_pw, line) ;
    strcat (G_reply_buf, line) ;
    strcat (G_reply_buf, " ") ;

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

/*
   This function is called by an ft_<task> when it has identified a power
   pin. Our job is to update the G_pin_flags array with this information.
   In particular, if another thread uses this pin for delivering power, we
   set the "is_power_pin" flag. We return the number of threads sharing this
   power pin (this is informational only).
*/

int f_register_power_pin (int powerPin, S_thread_entry *thread_p)
{
  xSemaphoreTake (G_pinflags_lock, portMAX_DELAY) ;
  S_pin_flag *pf = &G_pin_flags[powerPin] ;
  pf->is_power_pin = 1 ;

  /* count the number of active threads (excluding us) which own this pin */

  int idx, users=0, self=-1 ;
  for (idx=0 ; idx < MAX_THREADS ; idx++)
  {
    if (pf->users[idx] != NULL)
    {
      if (strcmp(pf->users[idx]->name, thread_p->name) == 0)
        self = idx ;                                    // our former index
      if ((pf->users[idx]->state == THREAD_RUNNING) &&
          (strcmp(pf->users[idx]->name, thread_p->name) != 0))
        users++ ;                                       // found active thread
    }
  }

  if (users > 0)
    pf->is_shared_power = 1 ;                           // tag this as shared

  /* if not already in the list, set us as one of the users of this pin */

  if (self < 0)
    for (idx=0 ; idx < MAX_THREADS ; idx++)
      if (pf->users[idx] == NULL)
      {
        pf->users[idx] = thread_p ;
        break ;
      }

  xSemaphoreGive (G_pinflags_lock) ;
  return (users) ;
}

/*
   This function is called from f_action(). Our job is to examine all GPIO
   pins and print any flags of interest.
*/

void f_pin_flags ()
{
  int idx ;
  char line[BUF_SIZE] ;

  xSemaphoreTake (G_pinflags_lock, portMAX_DELAY) ;
  for (idx=0 ; idx < MAX_GPIO_PINS ; idx++)
  {
    line[0] = 0 ;
    snprintf (line, BUF_SIZE, "GPIO%d: ", idx) ;
    strcat (G_reply_buf, line) ;
    line[0] = 0 ;

    if (G_pin_flags[idx].is_power_pin)
      strcat (G_reply_buf, "is_power_pin ") ;
    if (G_pin_flags[idx].is_shared_power)
      strcat (G_reply_buf, "is_shared_power ") ;

    int i ;
    for (i=0 ; i < MAX_THREADS ; i++)
      if ((G_pin_flags[idx].users[i] != NULL) &&
          (G_pin_flags[idx].users[i]->state == THREAD_RUNNING))
      {
        if (strlen(line) == 0)
          snprintf (line, BUF_SIZE, "users:%s",
                    G_pin_flags[idx].users[i]->name) ;
        else
        {
          strcat (line, ",") ;
          strcat (line, G_pin_flags[idx].users[i]->name) ;
        }
      }
    if (strlen(line) > 0)
      strcat (G_reply_buf, line) ;
    strcat (G_reply_buf, "\r\n") ;
  }
  xSemaphoreGive (G_pinflags_lock) ;
}

/* ------------------------------------------------------------------------- */
/* Network IO                                                                */
/* ------------------------------------------------------------------------- */

void f_mqtt_callback (char *topic, byte *payload, unsigned int length)
{
  char msg[MAX_MQTT_LEN + 1] ;
  char *cmd=msg, *tag=NULL ;

  if (length > MAX_MQTT_LEN)
  {
    G_Metrics->mqttOversize++ ;
    return ;
  }
  memcpy (msg, payload, length) ;
  msg[length] = 0 ;
  G_Metrics->mqttSubs++ ;

  /* attempt to identify optional "tag" from "cmd" */

  int idx ;
  for (idx=0 ; idx < strlen(msg) ; idx++)
  {
    if (msg[idx] == '|')
    {
      tag = msg ;
      tag[idx] = 0 ;            // found the optional "tag"
      cmd = msg + idx + 1 ;     // repoint to new "cmd" position
      break ;
    }
  }

  /* parse our "cmd" */

  idx = 0 ;
  char *tokens[MAX_TOKENS] ;
  char *p = strtok (cmd, " ") ;
  while ((p) && (idx < MAX_TOKENS))
  {
    tokens[idx] = p ;
    idx++ ;
    p = strtok (NULL, " ") ;
  }
  tokens[idx] = NULL ;

  /* prepend "G_reply_buf" with optional tag and execute the command */

  G_reply_buf[0] = 0 ;
  if (tag != NULL)
  {
    strcpy (G_reply_buf, tag) ;
    strcat (G_reply_buf, "|") ;
  }

  if (tokens[0] != NULL)
    f_action (tokens) ;

  /* if "G_reply_buf" has content, publish it, otherwise serial console */

  if ((strlen (G_reply_buf) > 0) &&
      (G_psClient.connected()) &&
      (strlen(G_mqtt_rtopic) > 0))
  {
    while ((strlen(G_reply_buf) > 0) &&
           ((G_reply_buf[strlen(G_reply_buf)-1] == '\r') ||
            (G_reply_buf[strlen(G_reply_buf)-1] == '\n')))
    {
      G_reply_buf[strlen(G_reply_buf)-1] = 0 ;
    }
    G_psClient.publish (G_mqtt_rtopic, G_reply_buf) ;
    G_Metrics->mqttPubs++ ;
  }
  else
  {
    Serial.println (G_reply_buf) ;
  }
  G_reply_buf[0] = 0 ;
}

/*
   This function reads the various MQTT config file and sets up an MQTT
   connection. It returns 1 on success, otherwise 0 with the error reported
   on the console. Note that topic subscription(s) are optional, this function
   still returns 1 if these are not configured.
*/

int f_mqtt_connect ()
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
    return (0) ;
  }
  else
  {
    int amt = f.readBytes (buf, BUF_SIZE-1) ;
    f.close () ;
    if (amt > 0)
    {
      char *idx=buf ;
      buf[amt] = 0 ;
      char *p = strtok_r (buf, ",", &idx) ;
      strcpy (G_mqtt_pub, p) ;
      p = strtok_r (NULL, ",", &idx) ;
      if ((p) && (strlen(G_hostname) > 0))
      {
        strcpy (G_mqtt_rtopic, p) ;
        strcat (G_mqtt_rtopic, "/") ;
        strcat (G_mqtt_rtopic, G_hostname) ;
      }
    }
  }

  f = SPIFFS.open (MQTT_CFG_FILE, "r") ;      // broker config is mandatory
  if (f == NULL)
  {
    sprintf (line, "WARNING: Cannot read MQTT subscribe file '%s'.",
             MQTT_CFG_FILE) ;
    Serial.println (line) ;
    return (0) ;
  }
  else
  {
    int amt = f.readBytes (buf, BUF_SIZE-1) ;
    f.close () ;
    if (amt < 1)
    {
      sprintf (line, "WARNING: %s is empty.", MQTT_CFG_FILE) ;
      Serial.println (line)  ;
      return (0) ;
    }
    else
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
        return (0) ;
      }
      else
      {
        char id[BUF_SIZE] ;
        unsigned char mac[6] ;
        WiFi.macAddress(mac) ;
        sprintf (id, "%x:%x:%x:%x:%x:%x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]) ;
        G_psClient.setServer (mqtt_host, atoi(mqtt_port)) ;
        if (G_psClient.connect (id, user, pw))
        {
          G_Metrics->mqttConnects++ ;
          if (strlen(G_mqtt_sub) > 0)
          {
            /* by default, subscribe to a topic "<subscribe_prefix>/<mac>" */

            char topic[MAX_MQTT_LEN] ;
            snprintf (topic, MAX_MQTT_LEN, "%s/%s", G_mqtt_sub, id) ;

            /*
               subscribe to "<subscribe_prefix>/<hostname>" if HOSTNAME_FILE
               exists
            */

            File f = SPIFFS.open (HOSTNAME_FILE, "r") ;
            if (f != NULL)
            {
              int amt = f.readBytes (G_hostname, BUF_SIZE) ;
              if (amt > 0)
              {
                G_hostname[amt] = 0 ;
                snprintf (topic, MAX_MQTT_LEN, "%s/%s",
                          G_mqtt_sub, G_hostname) ;
              }
              f.close () ;
            }

            strcpy (G_mqtt_stopic, topic) ;
            G_psClient.subscribe (topic) ;
            G_psClient.setCallback (f_mqtt_callback) ;
          }

          /* take this opportunity to read MQTT_TAGS_FILE */

          File f = SPIFFS.open (MQTT_TAGS_FILE, "r") ;
          if (f != NULL)
          {
            int amt = f.readBytes (G_mqtt_tags, MAX_MQTT_LEN-1) ;
            if (amt > 0)
              G_mqtt_tags[amt] = 0 ;
            f.close () ;
          }
        }
        else
        {
          sprintf (line, "WARNING: Cannot connect to broker %s:%d.",
                   mqtt_host, atoi(mqtt_port)) ;
          Serial.println (line) ;
          return (0) ;
        }
      }
    }
    return (1) ;
  }
}

/*
   This function is called from f_action(). It is mainly used for manual
   connection/disconnection of MQTT, most likely from the console.
*/

void f_mqtt_ctl (char *ctl)
{
  if (strcmp(ctl, "connect") == 0)
  {
    if (f_mqtt_connect())
      strcat (G_reply_buf, "MQTT connected\r\n") ;
    else
      strcat (G_reply_buf, "FAULT: check console for errors.\r\n") ;
  }
  else
  if (strcmp(ctl, "disconnect") == 0)
  {
    G_psClient.disconnect() ;
    strcat (G_reply_buf, "MQTT disconnected\r\n") ;
  }
  else
  {
    strcat (G_reply_buf, "FAULT: Invalid argument.\r\n") ;
  }
}

/*
   This function is supplied the S_thread_entry of the calling thread, and
   the result of interest. With some help from f_buildMetric(), our job is to
   format a string :

     <metric_name>{<result_tags>,<task_tags...>,<mqtt_tags...>} <result>

   which we publish to the topic :

     <publish_prefix>/<hostname>/<thread_name>
*/

void f_delivery (S_thread_entry *p, S_thread_result *r)
{
  char topic[MAX_MQTT_LEN] ;
  char payload[BUF_MEDIUM] ;

  snprintf (topic, MAX_MQTT_LEN, "%s/%s/%s", G_mqtt_pub, G_hostname, p->name) ;
  f_buildMetric (p, r, 1, payload) ;

  /* finally, append the value from "r" to "payload" as a string */

  char v[BUF_SIZE] ;
  if (p->num_int_results > 0)
    sprintf (v, " %d", r->i_value) ;
  else
    sprintf (v, " %d.%02d", int(r->f_value), (int)(r->f_value*100)%100) ;
  strcat (payload, v) ;

  /* write to G_pub_topic and G_pub_payload, only when they're empty */

  while (1)
  {
    xSemaphoreTake (G_publish_lock, portMAX_DELAY) ;
    if (strlen(G_pub_topic) == 0)
      break ;
    else
    {
      xSemaphoreGive (G_publish_lock) ;
      G_Metrics->mqttPubWaits++ ;
      delay (100) ;
    }
  }

  strcpy (G_pub_topic, topic) ;
  strcpy (G_pub_payload, payload) ;
  xSemaphoreGive (G_publish_lock) ;
}

void f_v1api (char *query)
{
  int i ;
  char url_buf[BUF_MEDIUM] ;

  for (i=0 ; query[i] != 0 ; i++)
  {
    if (query[i] == '+')
      url_buf[i] = ' ' ;
    else
      url_buf[i] = query[i] ;
  }
  url_buf[i] = 0 ;

  if (G_debug)
  {
    char line[BUF_MEDIUM] ;
    sprintf (line, "DEBUG: f_v1api() [%s]", url_buf) ;
    Serial.println (line) ;
  }

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
  int rlen = strlen (G_reply_buf) ;
  if (rlen >= REPLY_SIZE)
  {
    char line[BUF_SIZE] ;
    sprintf (line, "\r\nWARNING: G_reply_buf is %d bytes, max %d.\r\n",
             rlen, REPLY_SIZE) ;
    Serial.print (line) ;
  }
}

/*
   This is a convenience function called from f_handleWebMetrics() and also
   f_delivery(). Our job is to build up the metric entry with metadata tags
   for a given result "r", writing it into the supplied "metric" buffer. If
   "mqtt_tags" is non-zero, tags from MQTT_TAGS_FILE are added too. Note that
   the result value is not written into the output string "metric".
*/

void f_buildMetric (S_thread_entry *p, S_thread_result *r, int mqtt_tags,
                    char *metric)
{
  int i ;
  char all_tags[BUF_MEDIUM] ;

  char *label = p->name ;                       // use thread name or metric
  if (p->metric)
    label = p->metric ;
  strcpy (metric, label) ;

  all_tags[0] = 0 ;
  for (i=0 ; p->tags[i] != NULL ; i++)          // join up all task tags
  {
    if (strlen(all_tags) > 0)
      strcat (all_tags, ",") ;
    strcat (all_tags, p->tags[i]) ;
  }
  for (i=0 ; i < r->num_tags ; i++)             // join up thread's result tags
  {
    char one_tag[BUF_SIZE] ;
    snprintf (one_tag, BUF_SIZE, "%s=%s", r->meta[i], r->data[i]) ;
    if (strlen(all_tags) > 0)
      strcat (all_tags, ",") ;
    strcat (all_tags, one_tag) ;
  }
  if ((mqtt_tags) && (strlen(G_mqtt_tags) > 0))
  {
    if (strlen(all_tags) > 0)
      strcat (all_tags, ",") ;
    strcat (all_tags, G_mqtt_tags) ;
  }
  if (strlen(all_tags) > 0)                     // append all_tags to "metric"
  {
    strcat (metric, "{") ;
    strcat (metric, all_tags) ;
    strcat (metric, "}") ;
  }
}

void f_handleWebMetrics ()                      // for uri "/metrics"
{
  if (G_debug)
    Serial.println ("DEBUG: f_handleWebMetrics()") ;

  char line[BUF_MEDIUM] ; // <<-- also used to print our current stack addr
  sprintf (G_reply_buf,
           "ec_uptime_secs %ld\n"
           "ec_serial_in_bytes %ld\n"
           "ec_serial_commands %ld\n"
           "ec_serial_overruns %ld\n"
           "ec_rest_in_bytes %ld\n"
           "ec_rest_commands %ld\n"
           "ec_mqtt_connects %ld\n"
           "ec_mqtt_pubs %ld\n"
           "ec_mqtt_subs %ld\n"
           "ec_mqtt_oversize %ld\n"
           "ec_mqtt_pub_waits %ld\n"
           "ec_wifi_reconnects %ld\n"
           "ec_last_stack_addr %d\n"
           "ec_wifi_rssi %d\n",
           millis() / 1000,
           G_Metrics->serialInBytes,
           G_Metrics->serialCmds,
           G_Metrics->serialOverruns,
           G_Metrics->restInBytes,
           G_Metrics->restCmds,
           G_Metrics->mqttConnects,
           G_Metrics->mqttPubs,
           G_Metrics->mqttSubs,
           G_Metrics->mqttOversize,
           G_Metrics->mqttPubWaits,
           G_Metrics->wifiReconnects,
           line,
           WiFi.RSSI()) ;

  /* esp32 specific metrics */

  int idx, threads=0 ;
  for (idx=0 ; idx < MAX_THREADS ; idx++)
    if (G_thread_entry[idx].state == THREAD_RUNNING)
      threads++ ;

  sprintf (line,
           "ec_free_heap_bytes %ld\n"
           "ec_threads_running %d\n",
           xPortGetFreeHeapSize(),
           threads) ;
  strcat (G_reply_buf, line) ;

  /* individual thread metrics */

  char metric[BUF_MEDIUM] ;

  for (idx=0 ; idx < MAX_THREADS ; idx++)
    if (G_thread_entry[idx].state == THREAD_RUNNING)
    {
      /* this thread is running, print all result values, including tags */

      int r, t ;
      for (r=0 ; r < G_thread_entry[idx].num_int_results ; r++)
      {
        f_buildMetric (&G_thread_entry[idx],
                       &G_thread_entry[idx].results[r], 0, metric) ;
        sprintf (line, "%s %d\n", metric,
                 G_thread_entry[idx].results[r].i_value) ;
        strcat (G_reply_buf, line) ;
      }
      for (r=0 ; r < G_thread_entry[idx].num_float_results ; r++)
      {
        f_buildMetric (&G_thread_entry[idx],
                       &G_thread_entry[idx].results[r], 0, metric) ;
        String v = String (G_thread_entry[idx].results[r].f_value,
                           FLOAT_DECIMAL_PLACES) ;
        sprintf (line, "%s %s\n", metric, v.c_str()) ;
        strcat (G_reply_buf, line) ;
      }
    }
}

void f_handleWebRequest (S_WebClient *client)
{
  /* from the http "request", identify the Method, URI and query string */

  int idx, len=0 ;

  for (idx=0 ; idx < client->req_pos ; idx++)
  {
    if (isspace(client->request[idx]))
      break ;
    client->method[idx] = client->request[idx] ;
    client->method[idx+1] = 0 ;
    if (idx == BUF_SIZE - 1)
      break ;
  }
  idx++ ;
  while (idx < client->req_pos)
  {
    if ((client->request[idx] == '?') || (isspace(client->request[idx])) ||
        (client->request[idx] == '\n') || (client->request[idx] == '\r'))
      break ;
    client->uri[len] = client->request[idx] ;
    client->uri[len+1] = 0 ;
    idx++ ;
    len++ ;
  }
  if (client->request[idx] != '?')
    client->query[0] = 0 ;
  else
  {
    idx++ ;
    len = 0 ;
    while (idx < client->req_pos)
    {
      if ((client->request[idx] == '\n') || (client->request[idx] == '\r') ||
          (isspace(client->request[idx])))
        break ;
      client->query[len] = client->request[idx] ;
      client->query[len+1] = 0 ;
      idx++ ;
      len++ ;
    }
  }

  if (G_debug)
  {
    char line[BUF_MEDIUM] ;
    snprintf (line, BUF_MEDIUM, "DEBUG: method(%s) uri(%s) query(%s)",
              client->method, client->uri, client->query) ;
    Serial.println (line) ;
  }

  /* at this point, we have parsed enough to decide on what to do with it */

  G_reply_buf[0] = 0 ;
  G_Metrics->restCmds++ ;
  G_Metrics->restInBytes = G_Metrics->restInBytes +
                           strlen(client->uri) + strlen(client->query) ;

  if ((strcmp(client->method, "GET") == 0) &&
      (strcmp(client->uri, "/metrics") == 0))
  {
    f_handleWebMetrics () ;
  }
  if ((strcmp(client->method, "GET") == 0) &&
      (strcmp(client->uri, "/v1") == 0) &&
      (strncmp(client->query, "cmd=", 4) == 0))
  {
    f_v1api (client->query + 4) ;
  }

  char line[BUF_SIZE] ;
  strcpy (line, "HTTP/1.1 200 OK\n") ;
  write (client->sd, line, strlen(line)) ;
  strcpy (line, "Content-Type: text/plain\n") ;
  write (client->sd, line, strlen(line)) ;
  strcpy (line, "Connection: close\n") ;
  write (client->sd, line, strlen(line)) ;

  /* Now send G_reply_buf */

  write (client->sd, "\n", 1) ;
  write (client->sd, G_reply_buf, strlen(G_reply_buf)) ;
  close (client->sd) ;
  client->sd = 0 ;
  client->req_pos = 0 ;
}

void f_handleWebServer ()
{
  int i, max_fd ;
  struct fd_set fds ;
  struct timeval tv ;

  tv.tv_usec = 1000 ;
  tv.tv_sec = 0 ;
  FD_ZERO (&fds) ;
  FD_SET (G_sd, &fds) ;
  max_fd = G_sd ;
  for (i=0 ; i < MAX_HTTP_CLIENTS ; i++)
    if (G_WebClient[i].sd > 0)
    {
      FD_SET (G_WebClient[i].sd, &fds) ;
      if (G_WebClient[i].sd > max_fd)
        max_fd = G_WebClient[i].sd ;
    }

  if (select (max_fd+1, &fds, NULL, NULL, &tv) > 0)
  {
    if (FD_ISSET (G_sd, &fds))          // new client connection
    {
      unsigned int slen = sizeof(struct sockaddr_in) ;
      struct sockaddr_in saddr ;
      int sd = accept (G_sd, (struct sockaddr*) &saddr, &slen) ;
      if (sd > 0)
      {
        if (G_debug)
        {
          char line[BUF_SIZE] ;
          char ip[20] ;
          inet_ntop (AF_INET, &saddr.sin_addr, ip, sizeof(ip)) ;
          snprintf (line, BUF_SIZE, "DEBUG: New webclient from %s on sd %d.",
                    ip, sd) ;
          Serial.println (line) ;
        }

        /* find an available G_WebClient slot, otherwise close the socket */

        int idx ;
        for (idx=0 ; idx < MAX_HTTP_CLIENTS ; idx++)
          if (G_WebClient[idx].sd == 0)
          {
            G_WebClient[idx].sd = sd ;
            G_WebClient[idx].connect_time = millis () ;
            break ;
          }
        if (idx == MAX_HTTP_CLIENTS)
        {
          if (G_debug)
            Serial.println ("DEBUG: max http clients reached.") ;
          close (sd) ;
        }
      }
    }

    for (i=0 ; i < MAX_HTTP_CLIENTS ; i++)
      if ((G_WebClient[i].sd >0) && (FD_ISSET (G_WebClient[i].sd, &fds)))
      {
        /* activity on this client, could be more data, or a disconnect */

        if (G_debug)
        {
          char line[BUF_SIZE] ;
          snprintf (line, BUF_SIZE, "DEBUG: activity on sd:%d.",
                    G_WebClient[i].sd) ;
          Serial.println (line) ;
        }

        int buf_remaining = MAX_HTTP_REQUEST - G_WebClient[i].req_pos - 1 ;
        if (buf_remaining < 1)                  // client header too long
        {
          if (G_debug)
          {
            char line[BUF_SIZE] ;
            snprintf (line, BUF_SIZE, "DEBUG: client sd:%d exceeded %d bytes",
                      G_WebClient[i].sd, MAX_HTTP_REQUEST) ;
            Serial.println (line) ;
          }
          close (G_WebClient[i].sd) ;
          G_WebClient[i].sd = 0 ;
          G_WebClient[i].req_pos = 0 ;
        }
        else
        {
          int amt = read (G_WebClient[i].sd,
                          G_WebClient[i].request + G_WebClient[i].req_pos,
                          buf_remaining) ;
          if (amt < 1)                          // client closed connection
          {
            if (G_debug)
            {
              char line[BUF_SIZE] ;
              snprintf (line, BUF_SIZE, "DEBUG: client on sd:%d disconnected",
                        G_WebClient[i].sd) ;
              Serial.println (line) ;
            }
            close (G_WebClient[i].sd) ;
            G_WebClient[i].sd = 0 ;
            G_WebClient[i].req_pos = 0 ;
          }
          else                                  // client sent us data
          {
            char line[BUF_SIZE] ;
            if (G_debug)
            {
              snprintf (line, BUF_SIZE, "DEBUG: read %d bytes from sd:%d",
                        amt, G_WebClient[i].sd) ;
              Serial.println (line) ;
            }
            G_WebClient[i].req_pos = G_WebClient[i].req_pos + amt ;
            G_WebClient[i].request[G_WebClient[i].req_pos] = 0 ;

            /* if client's request is 2x empty lines, request is complete */

            if ((strstr(G_WebClient[i].request, "\n\n") != NULL) ||
                (strstr(G_WebClient[i].request, "\r\n\r\n") != NULL))
            {
              int idx = G_WebClient[i].req_pos ;
              while ((G_WebClient[i].request[idx-1] == '\n') ||
                     (G_WebClient[i].request[idx-1] == '\r'))
              {
                idx-- ;
                G_WebClient[i].request[idx] = 0 ;
              }
              G_WebClient[i].req_pos = idx ;
              f_handleWebRequest (&G_WebClient[i]) ;
            }
          }
        }
      }
  }

  /* inspect all connected clients, kick out idle ones */

  unsigned long now = millis () ;
  for (i=0 ; i < MAX_HTTP_CLIENTS ; i++)
    if ((G_WebClient[i].sd > 0) &&
        (G_WebClient[i].connect_time + (MAX_HTTP_RTIME*1000) < now ))
    {
      if (G_debug)
      {
        char line[BUF_SIZE] ;
        snprintf (line, BUF_SIZE, "DEBUG: disconnecting idle client sd:%d",
                  G_WebClient[i].sd) ;
        Serial.println (line) ;
      }
      close (G_WebClient[i].sd) ;
      G_WebClient[i].sd = 0 ;
      G_WebClient[i].req_pos = 0 ;
    }
}

/* ------------------------------------------------------------------------- */

void f_ota (char *url)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    strcat (G_reply_buf, "WARNING: WiFi not connected.\r\n") ;
    return ;
  }

  /* before we connected to the webserver, we need to parse our URL */

  #define READ_TIMEOUT 5
  #define PROTO "http://"
  #define CONTENT_TYPE "application/octet-stream"

  char line[BUF_SIZE] ;
  if (strncmp (PROTO, url, strlen(PROTO)) != 0)
  {
    sprintf (line, "WARNING: url does not start with '%s'.\r\n", PROTO) ;
    strcat (G_reply_buf, line) ;
    return ;
  }
  char *host = url + strlen(PROTO) ;
  char *uri = strstr (host, "/") ;
  if (uri != NULL)
  {
    int offset = uri - host ;
    host[offset] = 0 ;
    uri++ ;
  }
  if (uri == NULL)
    uri = "" ;
  if (strlen(host) < 1)
  {
    strcat (G_reply_buf, "WARNING: Invalid URL specified.\r\n") ;
    return ;
  }

  /* Now try make an HTTP GET */

  WiFiClient client ;
  if (client.connect (host, 80) == false)
  {
    sprintf (line, "WARNING: Cannot connect to %s.\r\n", host) ;
    strcat (G_reply_buf, line) ;
    return ;
  }

  char request[BUF_MEDIUM] ;
  sprintf (request, "GET /%s HTTP/1.1\r\n"
                    "Host: %s\r\n"
                    "Cache-Control: no-cache\r\n"
                    "Connection: close\r\n\r\n",
                    uri, host) ;
  client.print (request) ;

  /* read (and verify) HTTP headers from web server */

  long content_length = 0 ;
  char *content_type=NULL ;
  unsigned long last_act = millis () ;

  while (client.available () == 0)
  {
    if (millis() > last_act + (READ_TIMEOUT * 1000))
    {
      strcat (G_reply_buf, "WARNING: No response from web server.\r\n") ;
      client.stop () ;
      return ;
    }
    delay (10) ;
  }
  while (client.available () > 0)
  {
    String s = client.readStringUntil ('\n') ;
    s.trim () ;
    if (s.length() == 0)                // end of HTTP headers, ie blank line
      break ;

    if ((s.startsWith("HTTP/1.1")) && (s.indexOf("200") < 0))
    {
      sprintf (line, "WARNING: Received %s.\r\n", s.c_str()) ;
      strcat (G_reply_buf, line) ;
      client.stop () ;
      return ;
    }
    if (s.startsWith("Content-Length: "))
    {
      char *p = strstr (s.c_str(), " ") ;
      content_length = atoi (p+1) ;
    }
    if (s.startsWith("Content-Type: "))
    {
      char *p = strstr (s.c_str(), " ") ;
      content_type = p+1 ;
      if (strcmp (content_type, CONTENT_TYPE) != 0)
      {
        sprintf (line, "WARNING: Wrong Content-Type '%s'\r\n", content_type) ;
        strcat (G_reply_buf, line) ;
        client.stop () ;
        return ;
      }
    }
  }

  if ((content_length == 0) || (content_type == NULL))
  {
    strcat (G_reply_buf, "WARNING: Invalid HTTP response.\r\n") ;
    client.stop () ;
    return ;
  }

  if (Update.begin (content_length) == false)
  {
    sprintf (line, "WARNING: Insufficient space (%d bytes needed).\r\n",
             content_length) ;
    strcat (G_reply_buf, line) ;
    client.stop () ;
    return ;
  }

  /* Print message to the serial console, just in case an operator is there */

  sprintf (line, "NOTICE: Firmware download %s -> %s (%d bytes)",
           url, CONTENT_TYPE, content_length) ;
  Serial.println (line) ;

  /* Now we perform the actual firmware upgrade, no turning back ! */

  unsigned long tv_start = millis () ;
  unsigned long amt = Update.writeStream (client) ;
  if (amt != content_length)
  {
    sprintf (line, "WARNING: Only wrote %d out of %d bytes, aborting.\r\n",
             amt, content_length) ;
    strcat (G_reply_buf, line) ;
    client.stop () ;
    return ;
  }
  unsigned long tv_end = millis () ;
  if ((Update.isFinished()) && (Update.end()))
  {
    sprintf (line, "NOTICE: Flashed %d bytes in %d secs, please reboot.\r\n",
             amt, (tv_end - tv_start) / 1000) ;
    strcat (G_reply_buf, line) ;
  }
  else
  {
    sprintf (line, "WARNING: Firmware update failed error %d.\r\n",
             Update.getError()) ;
    Serial.print (line) ;
  }
  client.stop () ;
}

void f_cron ()
{
  if (G_debug)
    Serial.println ("DEBUG: f_cron()") ;

  /*
     Under certain circumstances, we want to try auto-reconnecting our wifi,
     even though our wifi state is WL_CONNECTED. In particular,
       a) if our rssi is 0 dBm (ie, not connected)
       b) our rssi is poorer than RSSI_LOW_THRES (eg, -72dBm)
       b) our mqtt state is MQTT_CONNECTION_LOST
  */

  int wifi_reconnect = 0 ;

  if (WiFi.status() == WL_CONNECTED)
  {
    if ((WiFi.RSSI() == 0) || (WiFi.RSSI() < RSSI_LOW_THRES))
      wifi_reconnect = 1 ;
    if (G_psClient.state() == MQTT_CONNECTION_LOST)
      wifi_reconnect = 1 ;
  }

  /* If wifi isn't connected, yet we have SSID & credentials, try reconnect */

  if ((WiFi.status() != WL_CONNECTED) &&
      (strlen(cfg_wifi_pw) > 0) &&
      (strlen(cfg_wifi_ssid) > 0))
    wifi_reconnect = 1 ;

  if (wifi_reconnect)
  {
    if (G_debug)
      Serial.println ("DEBUG: f_cron() calling f_wifi(connect)") ;

    char *args[3] ;
    args[0] = "wifi" ;
    args[1] = "connect" ;
    args[2] = NULL ;
    f_wifi (args) ;
    G_Metrics->wifiReconnects++ ;
  }

  /* If MQTT is not connected, try reconnect */

  if ((G_psClient.connected() == false) ||
      (G_psClient.state() != MQTT_CONNECTED))
  {
    if (G_debug)
      Serial.println ("DEBUG: f_cron() calling f_mqtt_connect()") ;
    f_mqtt_connect () ;
  }

  /* If this is our first run, check if AUTOEXEC_FILE exists */

  if (G_Metrics->cronRuns == 0)
  {
    File f = SPIFFS.open (AUTOEXEC_FILE, "r") ;
    if (f != NULL)
    {
      char buf[BUF_SIZE], *idx=buf ;
      int amt = f.readBytes (buf, BUF_SIZE-1) ;
      if (amt > 0)
      {
        buf[amt] = 0 ;
        char *p = strtok_r (buf, ",", &idx) ;
        while (p != NULL)
        {
          G_reply_buf[0] = 0 ;
          f_thread_create (p) ;
          delay (50) ;
          if (strlen(G_reply_buf) > 0)
          {
            Serial.print ("NOTICE: ") ;
            Serial.print (G_reply_buf) ;
          }
          p = strtok_r (NULL, ",", &idx) ;
        }
      }
      f.close () ;
    }
  }
  G_Metrics->cronRuns++ ;
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
     "delay_ms" will determine how often this function calls f_delivery().
     For example, call f_delivery() every 10 seconds if delay_ms is 100.
  */

  #define REPORT_INTERVAL 100

  if (p->results[0].i_value % REPORT_INTERVAL == 0)
  {
    f_delivery (p, &p->results[0]) ;
    p->results[0].i_value++ ;
  }

  /* at this point, just increment our counter and take a break. */

  p->results[0].i_value++ ;
  delay (delay_ms) ;
}

void ft_aread (S_thread_entry *p)
{
  /* get ready our configuration */

  if ((p->num_args < 2) || (p->num_args > 5))
  {
    strcpy (p->msg, "FATAL! Expecting 2-5x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int delay_ms = atoi (p->in_args[0]) ;
  int inPin = atoi (p->in_args[1]) ;
  int pwrPin = -1 ;

  char *loThres=NULL, *hiThres=NULL ;
  if (p->num_args > 2)
    pwrPin = atoi(p->in_args[2]) ;      // optional arg
  if (p->num_args > 3)
    loThres = p->in_args[3] ;           // optional arg
  if (p->num_args > 4)
    hiThres = p->in_args[4] ;           // optional arg

  /* if "loops" is 0, this is our first call, initialize stuff */

  if (p->loops == 0)
  {
    pinMode (inPin, INPUT) ;
    if (pwrPin >= 0)
    {
      pinMode (pwrPin, OUTPUT) ;
      f_register_power_pin (pwrPin, p) ;
    }
    p->num_int_results = 1 ;
    p->results[1].i_value = millis () ; // use this to store time of last run
    strcpy (p->msg, "ok") ;
  }

  /* power on device (if specified) and wait till half time before we poll */

  if (pwrPin >= 0)
    digitalWrite (pwrPin, HIGH) ;       // power on device

  #define MAX_POWER_UP_DELAY 50

  int nap = delay_ms / 2 ;
  if (nap > MAX_POWER_UP_DELAY)
    nap = MAX_POWER_UP_DELAY ;
  if (nap > 0)
    delay (nap) ;

  int cur_value = analogRead (inPin) ;          // read new value
  if ((pwrPin >= 0) && (G_pin_flags[pwrPin].is_shared_power == 0))
    digitalWrite (pwrPin, LOW) ;                // power off device

  /* if "loThres" or "hiThres" is defined, check for state change */

  if (p->loops > 0)
  {
    char *cur_state = "normal" ;
    char *prev_state = "normal" ;
    if ((loThres != NULL) && (strlen(loThres) > 0))
    {
      int loValue = atoi (loThres) ;
      if (cur_value < loValue)
        cur_state = "low" ;
      if (p->results[0].i_value < loValue)
        prev_state = "low" ;
    }
    if ((hiThres != NULL) && (strlen(hiThres) > 0))
    {
      int hiValue = atoi (hiThres) ;
      if (cur_value > hiValue)
        cur_state = "high" ;
      if (p->results[0].i_value > hiValue)
        prev_state = "high" ;
    }
    if (cur_state != prev_state)
    {
      p->results[0].i_value = cur_value ;
      f_delivery (p, &p->results[0]) ;
    }
  }

  p->results[0].i_value = cur_value ;

  /* we're done, figure out how long we have left to sleep */

  p->results[1].i_value = p->results[1].i_value + delay_ms ;
  nap = p->results[1].i_value - millis () ;
  if (nap > 0)
    delay (nap) ;
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
  t[4] = p->in_args[1] ;        // aggregation duration (ms)
  t[5] = p->in_args[0] ;        // interval between samples (ms)

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

  p->num_int_results = 9 ; // "announce" that we have results to view
}

void ft_dread (S_thread_entry *p)
{
  if ((p->num_args != 3) && (p->num_args != 4))
  {
    strcpy (p->msg, "FATAL! Expecting 3x or 4x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }
  int delay_ms = atoi (p->in_args[0]) ;         // how often we poll the pin
  int pin = atoi (p->in_args[1]) ;              // the pin we poll
  int mode = atoi (p->in_args[2]) ;             // whether to pull up to 3.3v
  int trig_ms = 0 ;                             // minimum pin high duration
  if (p->num_args == 4)
    trig_ms = atoi (p->in_args[3]) ;

  /* if "loops" is 0, this is our first call, initialize stuff */

  if (p->loops == 0)
  {
    if (mode == 0)
      pinMode (pin, INPUT) ;
    else
    if (mode == 1)
      pinMode (pin, INPUT_PULLUP) ;
    else
    {
      strcpy (p->msg, "FATAL! Invalid mode") ;
      p->state = THREAD_STOPPED ;
      return ;
    }

    p->results[0].meta[0] = "type" ;
    p->results[0].data[0] = "\"state\"" ;
    p->results[0].num_tags = 1 ;

    p->results[1].meta[0] = "type" ;
    p->results[1].data[0] = "\"short_triggers\"" ;
    p->results[1].num_tags = 1 ;

    p->num_int_results = 2 ;
    strcpy (p->msg, "ok") ;

    p->results[0].i_value = digitalRead (pin) ; // one time initialization
  }

  /*
     if "trig_ms" is zero, then report pin change immediately. Otherwise,
     wait for pin to go high, make sure it stays high long enough, then
     emit an event and wait for the pin to go low before we return.
  */

  if (trig_ms == 0)
  {
    int cur_value = digitalRead (pin) ;
    if ((p->loops > 1) && (cur_value != p->results[0].i_value))
    {
      p->results[0].i_value = cur_value ;
      f_delivery (p, &p->results[0]) ;
    }
    p->results[0].i_value = cur_value ;
    delay (delay_ms) ;
    return ;
  }
  else
  {
    int cur_value = digitalRead (pin) ;

    /* if we just went from high to low, announce it and return. */

    if ((p->results[0].i_value == 1) && (cur_value == 0))
    {
      p->results[0].i_value = 0 ;
      f_delivery (p, &p->results[0]) ;
      delay (delay_ms) ;
      return ;
    }

    /* if we're still low, do nothing and return */

    if ((p->results[0].i_value == 0) && (cur_value == 0))
    {
      delay (delay_ms) ;
      return ;
    }

    /* if we just went from low to high, monitor for "trig_ms" */

    if ((p->results[0].i_value == 0) && (cur_value == 1))
    {
      unsigned long now = millis () ;
      while (millis() < now + trig_ms)
      {
        delay (delay_ms) ;
        if (digitalRead (pin) == 0)     // pin did not stay high long enough
        {
          p->results[1].i_value++ ;
          return ;
        }
      }
      p->results[0].i_value = 1 ;
      f_delivery (p, &p->results[0]) ;  // announce that pin went high
      return ;
    }
  }
}

void ft_hcsr04 (S_thread_entry *p)
{
  if (p->num_args != 5)
  {
    strcpy (p->msg, "FATAL! Expecting 5x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  char s[BUF_SIZE] ;

  unsigned long delay_ms = atoi (p->in_args[0]) ;
  unsigned long aggr_ms = atoi (p->in_args[1]) ;
  int trigPin = atoi (p->in_args[2]) ;
  int echoPin = atoi (p->in_args[3]) ;
  int thres = atoi (p->in_args[4]) ;

  if (p->loops == 0)
  {
    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = "type" ;
    p->results[0].data[0] = "\"Min\"" ;

    p->results[1].num_tags = 1 ;
    p->results[1].meta[0] = "type" ;
    p->results[1].data[0] = "\"Ave\"" ;

    p->results[2].num_tags = 1 ;
    p->results[2].meta[0] = "type" ;
    p->results[2].data[0] = "\"Max\"" ;

    p->results[3].num_tags = 1 ;
    p->results[3].meta[0] = "type" ;
    p->results[3].data[0] = "\"Cur\"" ;

    p->results[0].i_value = millis () ; // use this to store time of last run
    strcpy (p->msg, "init") ;
  }

  /* probe our device until "aggr_ms" expires */

  int samples=0, faults=0 ;
  double v_total=0.0, v_min, v_max ;
  unsigned long job_start = p->results[0].i_value ;
  unsigned long job_end = p->results[0].i_value + aggr_ms ;

  while ((job_start < job_end) && (p->state == THREAD_RUNNING))
  {
    vTaskPrioritySet (p->tid, configMAX_PRIORITIES - 1) ;
    double f_value = f_hcsr04 (trigPin, echoPin) ;
    vTaskPrioritySet (p->tid, tskIDLE_PRIORITY) ;

    if (f_value > 0.0)                          // only process good data
    {
      v_total = v_total + f_value ;
      if (samples == 0)
        v_min = v_max = f_value ;
      else
      if (f_value > v_max)
        v_max = f_value ;
      else
      if (f_value < v_min)
        v_min = f_value ;
      samples++ ;

      /* check for state change */

      if (samples > 1)
      {
        int prev_state = 1 ;
        if (p->results[3].f_value < (double) thres)
          prev_state = 0 ;
        int cur_state = 1 ;
        if (f_value < (double) thres)
          cur_state = 0 ;

        if (prev_state != cur_state)
        {
          p->results[3].f_value = f_value ;
          f_delivery (p, &p->results[3]) ;
        }
      }
      p->results[3].f_value = f_value ; // use this to store previous value
    }
    else
      faults++ ;

    /* figure out how long to pause until our next job cycle */

    int nap = job_start + delay_ms - millis () ;
    if (nap > 0)
      delay (nap) ;
    job_start = job_start + delay_ms ;
  }

  sprintf (s, "[loop:%d samples:%d faults:%d]", p->loops, samples, faults) ;
  strcpy (p->msg, s) ;

  /* "assemble" our final results */

  p->results[0].f_value = v_min ;
  p->results[1].f_value = v_total / (double) samples ;
  p->results[2].f_value = v_max ;

  /* get ready for our next run */

  p->results[0].i_value = p->results[0].i_value + aggr_ms ;

  if (samples > 0)
    p->num_float_results = 4 ;                  // indicate results are good
  else
    p->num_float_results = 0 ;                  // indicate results are bad
}

void ft_dht22 (S_thread_entry *p)
{
  #define DHT22_POWER_ON_DELAY_MS 800
  #define DHT22_RETRY_DELAY_MS 50

  /* determine our parameters from "in_args" */

  if (p->num_args != 3)
  {
    strcpy (p->msg, "FATAL! Expecting 3x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }
  if (p->loops == 0)
  {
    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = "measurement" ;
    p->results[0].data[0] = "\"temperature\"" ;
    p->results[1].num_tags = 1 ;
    p->results[1].meta[0] = "measurement" ;
    p->results[1].data[0] = "\"humidity\"" ;
  }

  int delay_ms = atoi (p->in_args[0]) ;
  int dataPin = atoi (p->in_args[1]) ;
  int pwrPin = atoi (p->in_args[2]) ;
  unsigned long start_time = millis () ;
  unsigned long cutoff_time = start_time + delay_ms ;

  /* if pwrPin is > 0, that means it's a real pin, so power it on now */

  if (pwrPin >= 0)
  {
    pinMode (pwrPin, OUTPUT) ;
    digitalWrite (pwrPin, HIGH) ;
    delay (DHT22_POWER_ON_DELAY_MS) ;
  }

  int success=0 ;
  float temperature=0.0, humidity=0.0 ;

  while (millis() < cutoff_time)
  {
    /*
       reading DHT22 data is timing sensitive, set this thread priority high
       temporarily. If we don't do this, out thread WILL likely get context
       switched out in the middle of reading DHT22 data.
    */

    vTaskPrioritySet (p->tid, configMAX_PRIORITIES - 1) ;
    int result = f_dht22 (dataPin, &temperature, &humidity) ;
    vTaskPrioritySet (p->tid, tskIDLE_PRIORITY) ;

    if (result)
    {
      sprintf (p->msg, "polled in %dms", millis()-start_time) ;
      success = 1 ;
      break ;
    }
    delay (DHT22_RETRY_DELAY_MS) ;
  }

  /* if pwrPin is > 0, that means it's a real pin, so power it off now */

  if (pwrPin >= 0)
  {
    digitalWrite (pwrPin, LOW) ;
  }

  /* report results (or failure) */

  if (success)
  {
    p->results[0].f_value = temperature ;
    p->results[1].f_value = humidity ;
    p->num_float_results = 2 ;
  }
  else
  {
    snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
              "failed data:%d pwr:%d", dataPin, pwrPin) ;
    p->num_float_results = 0 ;
  }

  /* if we still have extra time, sleep a bit */

  unsigned long now = millis () ;
  if (now < cutoff_time)
    delay (cutoff_time - now) ;
}

void ft_ds18b20 (S_thread_entry *p)
{
  #define DS18B20_POWER_ON_DELAY_MS 500
  #define DS18B20_POWER_OFF_DELAY_MS 500
  #define DS18B20_RETRIES 10

  if (p->num_args != 3)
  {
    strcpy (p->msg, "FATAL! Expecting 3x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int delay_ms = atoi (p->in_args[0]) ;
  int dataPin = atoi (p->in_args[1]) ;
  int pwrPin = atoi (p->in_args[2]) ;
  unsigned long start_time = millis () ;

  /*
     Since this function calls f_ds18b20(), which returns multiple device
     addresses, we need a single static thread_local buffer which holds all
     null terminated strings of the device addresses in hex. To help us
     prepare this hex string, we use a temporary buffer "hex_buf" for this.
     Recall that a DS18B20's address is 8-bytes = 16+3 char string buffer,
     including double quotes required by prometheus's scrapes. To summarize :
       addr[] (binary) -> hex_buf -> addr_buf[] (text)
  */

  int results = 0 ;
  int retries = DS18B20_RETRIES ;
  int addr_size = (MAX_DS18B20_DEVICES * 8) + 1 ;
  float t[MAX_DS18B20_DEVICES] ;
  char hex_buf[16 + 3] ;
  unsigned char addr[addr_size] ;
  static thread_local char addr_buf[MAX_DS18B20_DEVICES * (16 + 3)] ;

  if (p->loops == 0)
  {
    memset (addr_buf, 0, MAX_DS18B20_DEVICES * (16 + 3)) ;

    /*
       turn on the power pin and leave it on in case we have other DS18B20s
       polled by different threads.
    */

    pinMode (pwrPin, OUTPUT) ;
    digitalWrite (pwrPin, HIGH) ;
    delay (DS18B20_POWER_ON_DELAY_MS) ;
  }

  while (retries > 0)
  {
    memset (addr, 0, addr_size) ;
    results = f_ds18b20 (dataPin, addr, t) ;
    if (results < 1)
    {
      p->num_float_results = 0 ;
      retries-- ;
      sprintf (p->msg, "Read failed, retries %d.", retries) ;
      pinMode (pwrPin, OUTPUT) ;
      digitalWrite (pwrPin, LOW) ;
      delay (DS18B20_POWER_OFF_DELAY_MS) ;      /* reboot the DS18B20 */
      digitalWrite (pwrPin, HIGH) ;
      delay (DS18B20_POWER_ON_DELAY_MS) ;
    }
    else
    {
      /*
         depending on the number of "results" we received, this will determine
         our "p->num_float_results", where each result's "data[0]" is the
         hardcoded string "temperature", but "data[1]" is the device's address.
      */

      int r_offset=0 ; // our current read offset in "addr"
      int w_offset=0 ; // our current write offset in "addr_buf"

      for (int idx=0 ; idx < results ; idx++)
      {
        sprintf (hex_buf, "\"%02x%02x%02x%02x%02x%02x%02x%02x\"",
                 addr[r_offset], addr[r_offset+1],
                 addr[r_offset+2], addr[r_offset+3],
                 addr[r_offset+4], addr[r_offset+5],
                 addr[r_offset+6], addr[r_offset+7]) ;
        memcpy (addr_buf+w_offset, hex_buf, 16 + 2) ;

        p->results[idx].num_tags = 2 ;
        p->results[idx].meta[0] = "measurement" ;
        p->results[idx].data[0] = "\"temperature\"" ;
        p->results[idx].meta[1] = "address" ;
        p->results[idx].data[1] = addr_buf + w_offset ;
        p->results[idx].f_value = t[idx] ;

        r_offset = r_offset + 8 ;       // move to next device address
        w_offset = w_offset + 16 + 3 ;  // move to next string
      }
      p->num_float_results = results ;
      break ;
    }
  }
  if (retries == 0)
    p->num_float_results = 0 ;

  /* now figure out how long to nap for */

  unsigned long end_time = millis () ;
  int duration_ms = end_time - start_time ;
  if (p->num_float_results)
    sprintf (p->msg, "polled %d in %dms", results, duration_ms) ;
  int nap = delay_ms - duration_ms ;
  if (nap > 0)
    delay (nap) ;
}

void ft_tread (S_thread_entry *p)
{
  #define TOUCH_SAMPLES 20 // calculate average value to suppress jitter

  if (p->num_args != 4)
  {
    strcpy (p->msg, "FATAL! Expecting 4x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  unsigned long start_time = millis () ;
  int delay_ms = atoi (p->in_args[0]) ;
  int pin = atoi (p->in_args[1]) ;
  int loThres = atoi (p->in_args[2]) ;
  int hiThres = atoi (p->in_args[3]) ;

  if (p->loops == 0)
  {
    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = "type" ;
    p->results[0].data[0] = "\"Cur\"" ;
    p->results[1].num_tags = 1 ;
    p->results[1].meta[0] = "type" ;
    p->results[1].data[0] = "\"Min\"" ;
    p->results[2].num_tags = 1 ;
    p->results[2].meta[0] = "type" ;
    p->results[2].data[0] = "\"Max\"" ;
    p->results[3].num_tags = 1 ;
    p->results[3].meta[0] = "type" ;
    p->results[3].data[0] = "\"State\"" ;
    strcpy (p->msg, "init") ;
  }

  /*
     Because the touch sensor gets lots of jitter, take multiple samples, find
     the standard deviation, then calculate the average from values within the
     standard deviation.
  */

  int i, val, samples=0 ;
  float raw[TOUCH_SAMPLES], total=0.0, mean, sd ;

  for (i=0 ; i < TOUCH_SAMPLES ; i++)
  {
    raw[i] = float (touchRead (pin)) ;
    total = total + raw[i] ;
  }
  mean = total / float(TOUCH_SAMPLES) ;

  /* sum up the square of each deviation */

  total = 0.0 ;
  for (i=0 ; i < TOUCH_SAMPLES ; i++)
    total = total + ((raw[i] - mean) * (raw[i] - mean)) ;

  /* standard deviation is the square root of average variance */

  sd = sqrt (total / float(TOUCH_SAMPLES)) ;

  /* calculate the average of datapoints within the standard deviation */

  total = 0.0 ;
  for (i=0 ; i < TOUCH_SAMPLES ; i++)
    if (fabsf (raw[i] - mean) <= sd)
    {
      total = total + raw[i] ;
      samples++ ;
    }
  val = int(total / float(samples)) ;

  if (p->loops == 0)
  {
    p->results[0].i_value = val ;       // current
    p->results[1].i_value = val ;       // minimum
    p->results[2].i_value = val ;       // maximum
    p->results[3].i_value = 0 ;         // state (0|1)
  }
  else
  if (samples > 1)
  {
    int prev = p->results[0].i_value ;
    p->results[0].i_value = val ;
    if (val < p->results[1].i_value)    // a lower minimum
      p->results[1].i_value = val ;
    if (val > p->results[2].i_value)    // a higher maximum
      p->results[2].i_value = val ;

    /*
       Check if we cross from a low to a high, or from high to low. Note
       that when touched (ie, state=1), "val" goes to a low value.
    */

    if ((p->results[3].i_value == 1) && (val > hiThres))        // touch off
    {
      p->results[3].i_value = 0 ;
      f_delivery (p, &p->results[3]) ;
    }
    if ((p->results[3].i_value == 0) && (val < loThres))        // touch on
    {
      p->results[3].i_value = 1 ;
      f_delivery (p, &p->results[3]) ;
    }
    p->num_int_results = 4 ;
  }

  /* see how long we get to sleep for */

  int nap = delay_ms - (millis() - start_time) ;
  sprintf (p->msg, "samples:%d sd:%d nap:%d", samples, (int)sd, nap) ;
  if (nap > 0)
    delay (nap) ;
}

/*
   This thread implements safe relay control. A relay is turned on by setting
   its control pin into INPUT mode (causing the pin to float high). The relay
   is turned off by setting the control pin to OUTPUT low. To command the relay
   on or off, an external command "relay <name> on|off" is invoked. Where
   "<name>" is our thread name. Once we are commanded on, we will stay on for
   another "dur" seconds before turning off automatically due to timeout (and
   entering a fault state). Once faulted, we will NOT turn on unless this
   thread is restarted.

   If we were in an ON state and the ESP32 reboots, the relay will remain in
   an ON state until this thread starts up (ie, "/autoexec.cfg").
*/

void ft_relay (S_thread_entry *p)
{
  #define RELAY_CHECK_INTERVAL_MS 80

  static thread_local int relay_state, fault_state ;

  if (p->num_args != 2)
  {
    strcpy (p->msg, "FATAL! Expecting 2x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int relay_pin = atoi (p->in_args[0]) ;
  int duration_secs = atoi (p->in_args[1]) ;

  if (p->loops == 0)
  {
    pinMode (relay_pin, INPUT) ;
    relay_state = 0 ;
    fault_state = 0 ;

    /* note: f_relay() reads this */

    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = "fault" ;
    p->results[0].data[0] = "\"timeout\"" ;
    p->results[0].i_value = duration_secs ;     // timeout to fault (secs)

    /* note: f_relay() reads and updates this */

    p->results[1].num_tags = 1 ;
    p->results[1].meta[0] = "relay" ;
    p->results[1].data[0] = "\"state\"" ;
    p->results[1].i_value = 0 ;                 // 0=off, 1=on, -1=fault

    /* note: f_relay() updates this */

    p->results[2].num_tags = 1 ;
    p->results[2].meta[0] = "autooff" ;
    p->results[2].data[0] = "\"time\"" ;
    p->results[2].i_value = 0 ;                 // millis() auto off time

    p->num_int_results = 3 ;
  }

  /* if we're in a fault state, turn OFF and do not continue */

  if (fault_state)
  {
    digitalWrite (relay_pin, HIGH) ;
    pinMode (relay_pin, INPUT) ;
    delay (RELAY_CHECK_INTERVAL_MS) ;
    return ;
  }

  /* check for state change request */

  if (relay_state != p->results[1].i_value)
  {
    relay_state = p->results[1].i_value ;
    if (relay_state == 0)                       // turn OFF
    {
      digitalWrite (relay_pin, HIGH) ;
      pinMode (relay_pin, INPUT) ;
    }
    if (relay_state == 1)                       // turn ON
    {
      pinMode (relay_pin, OUTPUT) ;
      digitalWrite (relay_pin, LOW) ;
    }
  }

  /* report on current state */

  long now = millis () ;
  if (relay_state == 0)
    strcpy (p->msg, "state:off") ;
  else
  {
    long time_left_ms = p->results[2].i_value - now ;
    sprintf (p->msg, "state:on timeout:%ds", time_left_ms / 1000) ;
  }

  /* if we're ON, check for timeout */

  if ((relay_state) && (now > p->results[2].i_value))
  {
    fault_state = 1 ;                   // set our internal fault flag
    p->results[1].i_value = -1 ;        // expose our fault state
    digitalWrite (relay_pin, HIGH) ;
    pinMode (relay_pin, INPUT) ;
    strcpy (p->msg, "state:off fault:on") ;
  }

  delay (RELAY_CHECK_INTERVAL_MS) ;
}

/*
   This function initializes an I2S audio source and then enters a main loop
   where it reads samples from a DMA buffer and sends them via UDP. This
   function lives in its main loop does not return unless "p->state" is set
   to THREAD_WRAPUP.
*/

void ft_i2sin (S_thread_entry *p)
{
  if ((p->num_args != 5) && (p->num_args != 6))
  {
    strcpy (p->msg, "FATAL! Expecting 5x or 6x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int sampleRate = atoi (p->in_args[0]) ;
  int SCKpin = atoi (p->in_args[1]) ;
  int WSpin = atoi (p->in_args[2]) ;
  int SDpin = atoi (p->in_args[3]) ;
  int dma_bufsize = MYI2S_BITS_PER_SAMPLE / 8 * MYI2S_DMA_SAMPLES ;

  int sd = -1 ;                         // xmit UDP socket descriptor
  double gain = 1.0 ;                   // software amplification (optional)
  double sample_ref = 0.0 ;             // the "zero" value for analog samples
  struct sockaddr_in dest_addr ;        // destination address (and port)
  size_t dma_read = 0 ;                 // number of bytes read from DMA
  unsigned long *dma_buf = NULL ;       // work buffer to copy DMA data to
  unsigned long sample_lo, sample_hi ;  // used to calculate dynamic range
  unsigned long long total = 0 ;        // used to calculate sample zero ref
  esp_err_t e ;                         // generic error return value

  /* parse "<ip>:<port>" */

  #define DEST_LEN 24 // buffer for storing "<ip>:<port>"
  char ip[DEST_LEN] ;
  if (strlen(p->in_args[4]) >= DEST_LEN-1)
  {
    strcpy (p->msg, "Invalid <ip>:<port>") ;
    p->state = THREAD_STOPPED ;
    return ;
  }
  strcpy (ip, p->in_args[4]) ;
  int idx, port=0 ;
  for (idx=1 ; idx < strlen(ip) ; idx++)
  {
    if (ip[idx] == ':')
    {
      ip[idx] = 0 ;
      port = atoi (ip + idx + 1) ;
    }
  }
  if ((port == 0) || (port > 65535))
  {
    strcpy (p->msg, "Invalid port number") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  /* software gain is optional */

  if (p->num_args == 6)
    gain = atof (p->in_args[5]) ;

  /* do one time I2S initialization (well, per thread instance) */

  if (p->loops == 0)
  {
    const i2s_config_t i2s_config = {
      .mode = i2s_mode_t (I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = sampleRate,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = i2s_comm_format_t (I2S_COMM_FORMAT_I2S |
                                                 I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = MYI2S_DMA_IN_BUFS,
      .dma_buf_len = MYI2S_DMA_SAMPLES
    } ;

    const i2s_pin_config_t pin_config = {
      .bck_io_num = SCKpin,                     // serial clock
      .ws_io_num = WSpin,                       // word select
      .data_out_num = I2S_PIN_NO_CHANGE,        // not used for audio in
      .data_in_num = SDpin                      // serial data
    } ;

    e = i2s_driver_install (MYI2S_INPUT_PORT, &i2s_config, 0, NULL) ;
    if (e != ESP_OK)
    {
      sprintf (p->msg, "i2s_driver_install() failed %d", e) ;
      p->state = THREAD_STOPPED ;
      return ;
    }

    /* the following 2x lines are hacks to make the SPH0645 work right */

    REG_SET_BIT (I2S_TIMING_REG(MYI2S_INPUT_PORT), BIT(9)) ;
    REG_SET_BIT (I2S_CONF_REG(MYI2S_INPUT_PORT), I2S_RX_MSB_SHIFT) ;

    e = i2s_set_pin (MYI2S_INPUT_PORT, &pin_config) ;
    if (e != ESP_OK)
    {
      sprintf (p->msg, "i2s_set_pin() failed %d", e) ;
      p->state = THREAD_STOPPED ;
      return ;
    }

    /* track i2s_read() success/failures, and timing info in our results[] */

    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = "dma_read" ;
    p->results[0].data[0] = "\"success\"" ;
    p->results[0].i_value = 0 ;                // i2s_read() bytes_read == size

    p->results[1].num_tags = 1 ;
    p->results[1].meta[0] = "dma_read" ;
    p->results[1].data[0] = "\"fails\"" ;
    p->results[1].i_value = 0 ;                // i2s_read() bytes_read < size

    p->results[2].num_tags = 1 ;
    p->results[2].meta[0] = "loop_work" ;
    p->results[2].data[0] = "\"ms\"" ;
    p->results[2].i_value = 0 ;                // millisecs last main work

    p->results[3].num_tags = 1 ;
    p->results[3].meta[0] = "loop_total" ;
    p->results[3].data[0] = "\"ms\"" ;
    p->results[3].i_value = 0 ;                // millisecs last main loop

    p->results[4].num_tags = 1 ;
    p->results[4].meta[0] = "signal" ;
    p->results[4].data[0] = "\"dynrange\"" ;
    p->results[4].i_value = 0 ;                // dynamic range (before gain)

    p->results[5].num_tags = 1 ;
    p->results[5].meta[0] = "signal" ;
    p->results[5].data[0] = "\"clipped\"" ;
    p->results[5].i_value = 0 ;                // sample sets clipped

    p->num_int_results = 6 ;

    /* prepare a UDP socket which we use to stream audio data */

    sd = socket (AF_INET, SOCK_DGRAM, IPPROTO_IP) ;
    if (sd < 0)
    {
      strcpy (p->msg, "socket(SOCK_DRAM) failed") ;
      p->state = THREAD_STOPPED ;
      i2s_stop (MYI2S_INPUT_PORT) ;
      i2s_driver_uninstall (MYI2S_INPUT_PORT) ;
      return ;
    }
    dest_addr.sin_addr.s_addr = inet_addr (ip) ;
    dest_addr.sin_family = AF_INET ;
    dest_addr.sin_port = htons (port) ;

    /* allocate a buffer to copy DMA contents to */

    dma_buf = (unsigned long*) malloc (dma_bufsize) ;
    if (dma_buf == NULL)
    {
      sprintf (p->msg, "dma_buf malloc(%d) failed", dma_bufsize) ;
      p->state = THREAD_STOPPED ;
      if (sd >= 0) close (sd) ;
      i2s_stop (MYI2S_INPUT_PORT) ;
      i2s_driver_uninstall (MYI2S_INPUT_PORT) ;
      return ;
    }

    /* do a test read */

    e = i2s_read (MYI2S_INPUT_PORT, dma_buf, dma_bufsize, &dma_read,
                  MYI2S_DMA_WAITTICKS) ;
    if ((e != ESP_OK) || (dma_read != dma_bufsize))
    {
      sprintf (p->msg, "i2s_read() failed %d", e) ;
      if (sd >= 0) close (sd) ;
      free (dma_buf) ;
      i2s_stop (MYI2S_INPUT_PORT) ;
      i2s_driver_uninstall (MYI2S_INPUT_PORT) ;
      return ;
    }
    sprintf (p->msg, "udp->%s:%d", ip, port) ;  // indicate we're all set !
  }

  unsigned int tv_loop=0, tv_work=0, tv_end ;
  while (p->state == THREAD_RUNNING)
  {
    tv_loop = millis () ;
    dma_read = 0 ;
    e = i2s_read (MYI2S_INPUT_PORT, dma_buf, dma_bufsize, &dma_read,
                  MYI2S_DMA_WAITTICKS) ;
    tv_work = millis () ;
    if ((e == ESP_OK) && (dma_read == dma_bufsize))
    {
      /* measure our samples to find dynamic range and zero reference value */

      sample_lo = sample_hi = total = dma_buf[0] ; // recalculate reference
      for (idx=1 ; idx < MYI2S_DMA_SAMPLES ; idx++)
      {
        if (dma_buf[idx] < sample_lo)
          sample_lo = dma_buf[idx] ;
        if (dma_buf[idx] > sample_hi)
          sample_hi = dma_buf[idx] ;
        total = total + dma_buf[idx] ;
      }
      long long ll = total / MYI2S_DMA_SAMPLES ;
      sample_ref = (double) (ll) ;                    // signal's "zero" level

      /*
         calculate dynamic range as a percentage against an unsigned 32-bit
         number (ie, save it as an integer result between 0 to 100)
      */

      long long drange = sample_hi - sample_lo ;
      p->results[4].i_value = (int) (drange >> 1) ;

      /* if optional gain is set, process it now */

      if (gain != 1.0)
      {
        /* figure out max gain without clipping */

        double apply_gain = gain ;
        double max_gain = (double) UINT_MAX / drange ;
        if (gain > max_gain)
        {
          apply_gain = max_gain ;               // limit the gain we'll apply
          p->results[5].i_value++ ;             // high gain clip
        }
        for (idx=0 ; idx < MYI2S_DMA_SAMPLES ; idx++) // apply software gain
        {
          double f = (double) dma_buf[idx] ;
          f = ((f - sample_ref) * apply_gain) + sample_ref ;
          dma_buf[idx] = (unsigned long) f ;
        }
      }

      sendto (sd, dma_buf, dma_bufsize, 0, (struct sockaddr*) &dest_addr,
              sizeof(dest_addr)) ;
      p->results[0].i_value++ ;
    }
    else
      p->results[1].i_value++ ;

    tv_end = millis() ;
    p->results[2].i_value = tv_end - tv_work ; // time for my logic in loop
    p->results[3].i_value = tv_end - tv_loop ; // time for entire loop
  }

  /* if we're here, we're shutting down, clean up resources */

  if (sd >= 0) close (sd) ;
  free (dma_buf) ;
  i2s_stop (MYI2S_INPUT_PORT) ;
  i2s_driver_uninstall (MYI2S_INPUT_PORT) ;

  strcpy (p->msg, "resources released") ; // indicate successful exit
}

/*
   This function initializes an I2S audio output and enters a main loop
   where it reads UDP packets and places them into a DMA buffer. This function
   lives in its main loop and does not return unless "p->state" is set to
   THREAD_WRAPUP.
*/

void ft_i2sout (S_thread_entry *p)
{
  if (p->num_args != 5)
  {
    strcpy (p->msg, "FATAL! Expecting 5x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int sampleRate = atoi (p->in_args[0]) ;
  int SCKpin = atoi (p->in_args[1]) ;
  int WSpin = atoi (p->in_args[2]) ;
  int SDpin = atoi (p->in_args[3]) ;
  int udpPort = atoi (p->in_args[4]) ;
  int dma_bufsize = MYI2S_BITS_PER_SAMPLE / 8 * MYI2S_DMA_SAMPLES ;

  int sd = -1 ;                         // recv UDP socket descriptor
  unsigned long *dma_buf = NULL ;       // work buffer to recv UDP data
  double sample_ref = 0.0 ;             // the "zero" value for analog values
  esp_err_t e ;                         // generic error return value

  /* do one time I2S initialization (well, per thread instance) */

  if (p->loops == 0)
  {
    const i2s_config_t i2s_config = {
      .mode = i2s_mode_t (I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = sampleRate,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = i2s_comm_format_t (I2S_COMM_FORMAT_I2S |
                                                 I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = 0,
      .dma_buf_count = MYI2S_DMA_OUT_BUFS,
      .dma_buf_len = MYI2S_DMA_SAMPLES
    } ;

    const i2s_pin_config_t pin_config = {
      .bck_io_num = SCKpin,
      .ws_io_num = WSpin,
      .data_out_num = SDpin,
      .data_in_num = I2S_PIN_NO_CHANGE
    } ;

    e = i2s_driver_install (MYI2S_OUTPUT_PORT, &i2s_config, 0, NULL) ;
    if (e != ESP_OK)
    {
      sprintf (p->msg, "i2s_driver_install() failed %d", e) ;
      p->state = THREAD_STOPPED ;
      return ;
    }

    e = i2s_set_pin (MYI2S_OUTPUT_PORT, &pin_config) ;
    if (e != ESP_OK)
    {
      sprintf (p->msg, "i2s_set_pin() failed %d", e) ;
      p->state = THREAD_STOPPED ;
      return ;
    }

    /* track UDP recv() metrics */

    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = "recv" ;
    p->results[0].data[0] = "\"ok\"" ;
    p->results[0].i_value = 0 ;         // recv() returns "dma_bufsize"

    p->results[1].num_tags = 1 ;
    p->results[1].meta[0] = "recv" ;
    p->results[1].data[0] = "\"short\"" ;
    p->results[1].i_value = 0 ;         // recv() returns < "dma_bufsize"

    p->results[2].num_tags = 1 ;
    p->results[2].meta[0] = "recv" ;
    p->results[2].data[0] = "\"idle\"" ;
    p->results[2].i_value = 0 ;         // select() timed out

    p->results[3].num_tags = 1 ;
    p->results[3].meta[0] = "dma_write" ;
    p->results[3].data[0] = "\"fails\"" ;
    p->results[3].i_value = 0 ;         // i2s_write() failed

    p->results[4].num_tags = 1 ;
    p->results[4].meta[0] = "signal" ;
    p->results[4].data[0] = "\"dynrange\"" ;
    p->results[4].i_value = 0 ;         // current "dynamic range" percentage

    p->num_int_results = 5 ;

    /* prepare a UDP socket and bind it to its listening port */

    sd = socket (AF_INET, SOCK_DGRAM, IPPROTO_IP) ;
    if (sd < 0)
    {
      strcpy (p->msg, "socket(SOCK_DRAM) failed") ;
      p->state = THREAD_STOPPED ;
      return ;
    }

    struct sockaddr_in addr ;
    memset (&addr, 0, sizeof(addr)) ;
    addr.sin_family = AF_INET ;
    addr.sin_addr.s_addr = INADDR_ANY ;
    addr.sin_port = htons (udpPort) ;
    if (bind (sd, (const struct sockaddr*) &addr, sizeof(addr)) < 0)
    {
      close (sd) ;
      i2s_stop (MYI2S_OUTPUT_PORT) ;
      i2s_driver_uninstall (MYI2S_OUTPUT_PORT) ;
      strcpy (p->msg, "bind() failed") ;
      p->state = THREAD_STOPPED ;
      return ;
    }

    /* allocate a buffer to copy UDP packet contents into */

    dma_buf = (unsigned long*) malloc (dma_bufsize) ;
    if (dma_buf == NULL)
    {
      sprintf (p->msg, "dma_buf malloc(%d) failed", dma_bufsize) ;
      close (sd) ;
      i2s_stop (MYI2S_OUTPUT_PORT) ;
      i2s_driver_uninstall (MYI2S_OUTPUT_PORT) ;
      p->state = THREAD_STOPPED ;
      return ;
    }
  }

  /*
     this is our main loop. it is very important that we do NOT block for
     too long, as we must check for a thread termination request. We want
     select() to only pause for a very short time. This is because during
     unusually long waits (ie, poor wifi), the I2S subsystem is still reading
     off our DMA buffers, and the audio playback will sound like noise. Thus
     select() waits for about half the time it takes to play through our DMA
     buffers

       packets_per_sec = sample_rate / samples_per_pkt
       usec_per_packet = 1000000 / packets_per_sec
  */

  int i, idx, usec_per_packet=0 ;
  unsigned long sample_lo, sample_hi ;
  unsigned long long total = 0 ;
  size_t written ;
  fd_set rfds ;
  struct timeval tv ;

  usec_per_packet = 1000000 / (sampleRate / MYI2S_DMA_SAMPLES) ;
  sprintf (p->msg, "%d usec/pkt", usec_per_packet) ;

  while (p->state == THREAD_RUNNING)
  {
    tv.tv_sec = 0 ;
    tv.tv_usec = usec_per_packet * (MYI2S_DMA_OUT_BUFS / 2) ;
    FD_ZERO (&rfds) ;
    FD_SET (sd, &rfds) ;
    int result = select (sd + 1, &rfds, NULL, NULL, &tv) ;
    if (result == 0)
    {
      /*
         If select() timed out, it means our audio stream got interrupted.
         It's very important that we zero out our DMA buffers otherwise the
         I2S subsystem continues to playback buffer contents, which for all
         practical purposes will sound like noise.
      */

      p->results[2].i_value++ ;
      for (i=0 ; i < MYI2S_DMA_OUT_BUFS ; i++)
      {
        memset (dma_buf, 0, dma_bufsize) ;
        i2s_write (MYI2S_OUTPUT_PORT, dma_buf, dma_bufsize, &written,
                   MYI2S_DMA_WAITTICKS) ;
      }
    }
    if ((result == 1) && (FD_ISSET (sd, &rfds)))
    {
      /*
         it's very important that we use the MSG_DONTWAIT flag here to
         prevent recv() from blocking. By right select() handles our blocking
         wait in a controlled manner. Not using MSG_DONTWAIT causes this
         thread to block excessively, which is bad.
      */

      size_t amt = recv (sd, dma_buf, dma_bufsize, MSG_DONTWAIT) ;
      if (amt < dma_bufsize)
        p->results[1].i_value++ ;       // recv() returned insufficient data
      if (amt == dma_bufsize)
      {
        /* inspect samples to find dynamic range and zero reference value */

        sample_lo = sample_hi = total = dma_buf[0] ;
        for (idx=1 ; idx < MYI2S_DMA_SAMPLES ; idx++)
        {
          if (dma_buf[idx] < sample_lo)
            sample_lo = dma_buf[idx] ;
          if (dma_buf[idx] > sample_hi)
            sample_hi = dma_buf[idx] ;
          total = total + dma_buf[idx] ;
        }
        long long ll = total / MYI2S_DMA_SAMPLES ;
        sample_ref = (double) (ll) ;                    // "zero" level

        /*
           dynamic range is an unsigned 32-bit number, but "i_value" is "int",
           so right shift bits by 1.
        */

        p->results[4].i_value = (int) ((sample_hi - sample_lo) >> 1) ;

        p->results[0].i_value++ ;       // recv() returned expected amount
        written = 0 ;
        e = i2s_write (MYI2S_OUTPUT_PORT, dma_buf, amt, &written,
                       MYI2S_DMA_WAITTICKS) ;
        if ((e != ESP_OK) || (written != amt))
          p->results[3].i_value++ ;
      }
    }
  }

  if (sd > 0) close (sd) ;
  free (dma_buf) ;
  i2s_stop (MYI2S_OUTPUT_PORT) ;
  i2s_driver_uninstall (MYI2S_OUTPUT_PORT) ;

  strcpy (p->msg, "resources released") ; // indicate successful exit
}

/*
   This function does not return until its "p->state" is set to THREAD_WRAPUP.
   Its job is to create and bind a TCP listening socket, initialize a hardware
   uart (if needed) and await bytes from either the TCP client or the uart.
   Note that only 1x connected TCP client may be serviced at a time.
*/

void ft_serial2tcp (S_thread_entry *p)
{
  if (p->num_args != 4)
  {
    strcpy (p->msg, "FATAL! Expecting 4x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int listen_sd = -1 ;
  int client_sd = -1 ;
  int port = atoi (p->in_args[0]) ;
  int baud = atoi (p->in_args[1]) ;
  int rx_pin = atoi (p->in_args[2]) ;
  int tx_pin = atoi (p->in_args[3]) ;

  /* configure metrics that we will expose */

  p->results[0].num_tags = 1 ;
  p->results[0].meta[0] = "client" ;
  p->results[0].data[0] = "\"connects\"" ;

  p->results[1].num_tags = 1 ;
  p->results[1].meta[0] = "client" ;
  p->results[1].data[0] = "\"connected\"" ;

  p->results[2].num_tags = 1 ;
  p->results[2].meta[0] = "uart_bytes" ;
  p->results[2].data[0] = "\"read\"" ;

  p->results[3].num_tags = 1 ;
  p->results[3].meta[0] = "uart_bytes" ;
  p->results[3].data[0] = "\"written\"" ;

  p->num_int_results = 4 ;

  /* create a TCP socket, bind to port and listen */

  listen_sd = socket (AF_INET, SOCK_STREAM, IPPROTO_IP) ;
  if (listen_sd < 0)
  {
    strcpy (p->msg, "FATAL! socket() failed") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  struct sockaddr_in addr ;
  memset (&addr, 0, sizeof(addr)) ;
  addr.sin_family = AF_INET ;
  addr.sin_addr.s_addr = INADDR_ANY ;
  addr.sin_port = htons (port) ;
  if (bind (listen_sd, (const struct sockaddr*) &addr, sizeof(addr)) < 0)
  {
    close (listen_sd) ;
    strcpy (p->msg, "FATAL! bind() failed") ;
    p->state = THREAD_STOPPED ;
    return ;
  }
  if (listen (listen_sd, 1) != 0)
  {
    close (listen_sd) ;
    strcpy (p->msg, "FATAL! listen() failed") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  /* if we're the first thread to access the hardware UART, initialize it */

  xSemaphoreTake (G_hw_uart->lock, portMAX_DELAY) ;
  G_hw_uart->in_use++ ;
  if (G_hw_uart->initialized == 0)
  {
    Serial2.begin (baud, SERIAL_8N1, rx_pin, tx_pin) ;
    G_hw_uart->initialized = 1 ;
  }
  xSemaphoreGive (G_hw_uart->lock) ;

  /* thread's main loop and variables to manage TCP sockets and UART data */

  int result, num_fds, amt ;
  char buf[BUF_SIZE] ;
  fd_set rfds ;
  struct timeval tv ;
  sprintf (p->msg, "rx:%d,tx:%d,port:%d", rx_pin, tx_pin, port) ;

  while (p->state == THREAD_RUNNING)
  {
    /*
       If we have no connected TCP client, check if "listen_sd" has a new
       client for us. Also, don't check for new clients if we already have
       a connected TCP client.
    */

    tv.tv_sec = 0 ;
    tv.tv_usec = 20 ;                   // stall select() for a short time
    FD_ZERO (&rfds) ;
    if (client_sd > 0)                  // check client for data
    {
      FD_SET (client_sd, &rfds) ;
      num_fds = client_sd + 1 ;
    }
    else                                // check for new client
    {
      FD_SET (listen_sd, &rfds) ;
      num_fds = listen_sd + 1 ;
    }
    result = select (num_fds, &rfds, NULL, NULL, &tv) ;

    if (result == 1)
    {
      if (FD_ISSET (listen_sd, &rfds))                  // new tcp client !!
      {
        client_sd = accept (listen_sd, NULL, NULL) ;
        p->results[0].i_value++ ;
        p->results[1].i_value = 1 ;
      }

      if (FD_ISSET (client_sd, &rfds))
      {
        amt = read (client_sd, buf, BUF_SIZE) ;
        if (amt < 1)                                    // client closed
        {
          close (client_sd) ;
          client_sd = -1 ;
          p->results[1].i_value = 0 ;
        }
        else                                            // send data to uart
        {
          Serial2.write (buf, amt) ;
          p->results[3].i_value = p->results[3].i_value + amt ;
          if (p->results[3].i_value < 0)
            p->results[3].i_value = 0 ; // fix rollover
        }
      }
    }

    /*
       check if data is available on the serial port, for some reason,
       "Serial2.read()" does not block.
    */

    amt = Serial2.read (buf, BUF_SIZE) ;
    if (amt > 0)
    {
      p->results[2].i_value = p->results[2].i_value + amt ;
      if (p->results[2].i_value < 0)
        p->results[2].i_value = 0 ; // fix rollover
      if (client_sd > 0)
        write (client_sd, buf, amt) ;
    }
  }

  /*
     if we're here, indicate we're not using the uart anymore and close TCP
     sockets in use.
  */

  xSemaphoreTake (G_hw_uart->lock, portMAX_DELAY) ;
  G_hw_uart->in_use-- ;
  xSemaphoreGive (G_hw_uart->lock) ;

  close (listen_sd) ;
  if (client_sd > 0)
    close (client_sd) ;

  p->state = THREAD_STOPPED ;
}

/*
   The role of this thread is to read messages from a hardware uart, parse
   them and extract data, publishing it as in the "p->results[]" array. The
   main loop only exits if "p->state" is set to THREAD_WRAPUP. Since our
   primary data is latitude/longitude/altitude, all results exposed by this
   thread are float.
*/

void ft_gpsmon (S_thread_entry *p)
{
  #define READ_BUF 32   // max bytes we read from the uart at a time
  #define GPS_BUF 128   // max length of a single GPS message
  #define GPS_FIELDS 32 // max comma separated fields in a GPS message

  if (p->num_args != 3)
  {
    strcpy (p->msg, "FATAL! Expecting 3x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int baud = atoi (p->in_args[0]) ;
  int rx_pin = atoi (p->in_args[1]) ;
  int tx_pin = atoi (p->in_args[2]) ;

  /* configure metrics that we will expose */

  p->results[0].num_tags = 1 ;
  p->results[0].meta[0] = "msg" ;
  p->results[0].data[0] = "\"cksumOk\"" ;

  p->results[1].num_tags = 1 ;
  p->results[1].meta[0] = "msg" ;
  p->results[1].data[0] = "\"cksumBad\"" ;

  p->results[2].num_tags = 1 ;
  p->results[2].meta[0] = "msg" ;
  p->results[2].data[0] = "\"overrun\"" ;

  p->results[3].num_tags = 1 ;
  p->results[3].meta[0] = "position" ;
  p->results[3].data[0] = "\"latitude\"" ;

  p->results[4].num_tags = 1 ;
  p->results[4].meta[0] = "position" ;
  p->results[4].data[0] = "\"longitude\"" ;

  p->results[5].num_tags = 1 ;
  p->results[5].meta[0] = "time" ;
  p->results[5].data[0] = "\"utc\"" ;

  p->results[6].num_tags = 1 ;
  p->results[6].meta[0] = "satellites" ;
  p->results[6].data[0] = "\"used\"" ;

  p->results[7].num_tags = 1 ;
  p->results[7].meta[0] = "elevation" ;
  p->results[7].data[0] = "\"sealevel\"" ;

  p->results[8].num_tags = 1 ;
  p->results[8].meta[0] = "elevation" ;
  p->results[8].data[0] = "\"geoid\"" ;

  p->results[9].num_tags = 1 ;
  p->results[9].meta[0] = "nav" ;
  p->results[9].data[0] = "\"mode\"" ; // 1=none, 2=2D fix, 3=3D fix

  p->results[10].num_tags = 1 ;
  p->results[10].meta[0] = "dilution" ;
  p->results[10].data[0] = "\"position\"" ;

  p->results[11].num_tags = 1 ;
  p->results[11].meta[0] = "dilution" ;
  p->results[11].data[0] = "\"horizontal\"" ;

  p->results[12].num_tags = 1 ;
  p->results[12].meta[0] = "dilution" ;
  p->results[12].data[0] = "\"vertical\"" ;

  p->results[13].num_tags = 1 ;
  p->results[13].meta[0] = "speed" ;
  p->results[13].data[0] = "\"kmh\"" ;

  p->num_float_results = 14 ;

  /* if we're the first thread to access the hardware UART, initialize it */

  xSemaphoreTake (G_hw_uart->lock, portMAX_DELAY) ;
  G_hw_uart->in_use++ ;
  if (G_hw_uart->initialized == 0)
  {
    Serial2.begin (baud, SERIAL_8N1, rx_pin, tx_pin) ;
    G_hw_uart->initialized = 1 ;
  }
  xSemaphoreGive (G_hw_uart->lock) ;
  strcpy (p->msg, "ok") ;

  int idx, amt ;
  int msg_len = 0 ;                     // length of current msg in "gps_buf"
  char read_buf[READ_BUF] ;             // buffer to read from uart
  char gps_buf[GPS_BUF] ;               // buffer to hold a complete GPS msg
  char *msg_tokens[GPS_FIELDS] ;        // pointers pointing into "gps_buf"

  while (p->state == THREAD_RUNNING)
  {
    /*
       keep reading bytes from UART until we have a complete message from
       the GPS beginning with '$' and ending with CR/NL.
    */

    amt = Serial2.read (read_buf, READ_BUF) ;
    if (amt > 0)
    {
      /*
         if "msg_len" == 0, that means we're hunting for the start of a msg.
         if "msg_len" > 0, that keep copying bytes until we see the msg end.
      */

      for (idx=0 ; idx < amt ; idx++)
      {
        if (read_buf[idx] == '$')       // found start of message
        {
          gps_buf[0] = read_buf[idx] ;
          msg_len = 1 ;
        }
        else
        if (read_buf[idx] == '\r')      // found end of message
        {
          gps_buf[msg_len] = 0 ;
          if (msg_len > 8)              // minimum msg_len for a sane msg
          {
            char checksum = 0 ;
            for (int i=1 ; i < msg_len ; i++)
            {
              if (gps_buf[i] == '*')
                break ;
              else
                checksum = checksum ^ gps_buf[i] ;
            }
            char verify[4] ; // just enough to store "*XX"
            sprintf (verify, "*%X", checksum) ;
            if (strcmp(gps_buf+msg_len-3, verify) == 0)         // checksum OK
            {
              p->results[0].f_value = p->results[0].f_value + 1 ;
              msg_len = msg_len - 3 ;
              gps_buf[msg_len] = 0 ; // remove checksum, don't need it anymore

              /*
                 tokenize "gps_buf" to "num_tokens" of "msg_tokens" pointers.
                 We can't use strtok() here because some fields may be zero
                 length and strtok() doesn't handle it correctly. This code
                 ensures zero length tokens point to a 0-length string.
              */

              int num_tokens = 0 ;
              int tok_start = 1 ; // move past the '$' character

              for (int tok_end=1 ; tok_end < msg_len ; tok_end++)
                if ((gps_buf[tok_end] == ',') || (tok_end == msg_len - 1))
                {
                  if (tok_end < msg_len - 1)
                    gps_buf[tok_end] = 0 ;
                  if (tok_end - tok_start > 0)
                    msg_tokens[num_tokens] = gps_buf + tok_start ;
                  else
                    msg_tokens[num_tokens] = "" ; // provide an 0-length string
                  num_tokens++ ;
                  tok_start = tok_end + 1 ;
                }

              if (G_debug > 1)
              {
                char line[BUF_SIZE] ;
                Serial.println ("DEBUG: ft_gpsmon() msg_tokens[]") ;
                for (int i=0 ; i < num_tokens ; i++)
                {
                  sprintf (line, "  %d:(%s)", i, msg_tokens[i]) ;
                  Serial.println (line) ;
                }
              }

              /* extract information from various GPS messages */

              if ((strcmp (msg_tokens[0], "GNRMC") == 0) && (num_tokens == 13))
              {
                double cur_deg, cur_mins ;
                sscanf (msg_tokens[3], "%2lf%lf", &cur_deg, &cur_mins) ;
                double latitude = cur_deg + (cur_mins / 60.0) ;
                if (msg_tokens[4][0] == 'S')
                  latitude = latitude * -1 ;
                sscanf (msg_tokens[5], "%3lf%lf", &cur_deg, &cur_mins) ;
                double longitude = cur_deg + (cur_mins / 60.0) ;
                if (msg_tokens[6][0] == 'W')
                  longitude = longitude * -1 ;

                p->results[3].f_value = latitude ;
                p->results[4].f_value = longitude ;

                /* parse the time and date */

                int hour, minute, seconds, day, month, year ;
                struct tm utc ;

                sscanf (msg_tokens[1], "%2d%2d%2d", &hour, &minute, &seconds) ;
                sscanf (msg_tokens[9], "%2d%2d%2d", &day, &month, &year) ;
                memset (&utc, 0, sizeof(utc)) ;
                utc.tm_sec = seconds ;
                utc.tm_min = minute ;
                utc.tm_hour = hour ;
                utc.tm_mday = day ;
                utc.tm_mon = month - 1 ; /* this needs to be 0 to 11 */
                utc.tm_year = year + 100 ; /* years since 1900 */
                time_t now = mktime (&utc) ;

                p->results[5].f_value = now ;
              }

              if (strcmp (msg_tokens[0], "GNGGA") == 0)
              {
                double satellites = atof (msg_tokens[7]) ; // satellites used
                double altitude = atof (msg_tokens[9]) ; // "m" above sea level
                double geoid = atof (msg_tokens[11]) ;  // geoid and sea level
                p->results[6].f_value = satellites ;
                p->results[7].f_value = altitude ;
                p->results[8].f_value = geoid ;
              }

              if ((strcmp (msg_tokens[0], "GNGSA") == 0) && (num_tokens == 18))
              {
                double mode = atof (msg_tokens[2]) ;
                double dilution_pos = atof(msg_tokens[15]) ;
                double dilution_hori = atof(msg_tokens[16]) ;
                double dilution_vert = atof(msg_tokens[17]) ;
                p->results[9].f_value = mode ;
                p->results[10].f_value = dilution_pos ;
                p->results[11].f_value = dilution_hori ;
                p->results[12].f_value = dilution_vert ;
              }

              if ((strcmp (msg_tokens[0], "GNVTG") == 0) && (num_tokens == 10))
              {
                double speed = atof (msg_tokens[7]) ;
                p->results[13].f_value = speed ;
              }
            }
            else                                                // checksum BAD
            {
              p->results[1].f_value = p->results[1].f_value + 1 ;
              if (G_debug)
              {
                char line[GPS_BUF+BUF_SIZE] ;
                snprintf (line, GPS_BUF+BUF_SIZE,
                          "DEBUG: ft_gpsmon() gps_buf<%s>[%X] Bad checksum",
                          gps_buf, checksum) ;
                Serial.println (line) ;
              }
            }
          }
          msg_len = 0 ;                 // "reset" gps_buf.
        }
        else
        if (msg_len > 0)                // in the middle of a message
        {
          gps_buf[msg_len] = read_buf[idx] ;
          msg_len++ ;
          if (msg_len == GPS_BUF)       // buffer overrun :(
          {
            msg_len = 0 ;
            p->results[2].f_value = p->results[2].f_value + 1 ;
          }
        }
      }
    }
  }

  xSemaphoreTake (G_hw_uart->lock, portMAX_DELAY) ;
  G_hw_uart->in_use-- ;
  xSemaphoreGive (G_hw_uart->lock) ;
}

/*
   This thread depends on metrics from ft_gpsmon(). Our job is to determine
   if we've moved and log our new location to a ring buffer. When the ring
   buffer fills up, flush it to a file. We must have moved a minimum distance
   in order to log an entry. If our minimum movement threahold is 2m, then

     arctan(2/6371001) = .0000003139 radians
                       = .0000003139 * 180 / PI
                       = .0000179864 degrees

   In actual fact, we add our elevation to the average earth radius for the
   best accuracy. Thus, we compute

     GPS elevation - GPS geoid + Ave Earth Radius

   This thread uses the following configuration determine that the GPS has
   achieved a reliable lock (pay attention to the various data types).

     cfg_gpsMode     - The GPS mode number which indicates 3D fix (def: 3)
     cfg_minSatsUsed - Minimum satellites used (def: 4)
     cfg_maxPosDil   - Maximum allowed position dilution (def: 3.5)

   Position data is written into a ring buffer which is flushed to a file as
   a CSV, with the first line containing the heading, followed by one or more
   data rows. The following configuration determines the data logging behavior.

     cfg_gpsThread     - the name of the ft_gpsmon() thread
     cfg_minDistMeters - minimum meters moved to trigger logging (def: 12.0)
     cfg_normLogSecs   - normal interval between log entries (def: 10)
     cfg_maxLogSecs    - max log interval when stationary (def: 60)
     cfg_fileName      - the CSV file we write to (def: "/gps.csv")
     cfg_fileMaxSize   - rotated if file exceeds this (def: 262144 bytes)

   Each entry written to "cfg_fileName" looks like,

     1638745487,43.723317,-79.409851,175.500000,2.480000

   Thus, at 52 bytes per line, a 256k file contains,

     262144 / 52 = 5041 entries
                 = 50410 seconds of entries (assuming cfg_normLogSecs = 10)
                 = 14 hours of position data (or more)

   Since there are many tunable parameters, this thread takes 1 argument on
   invocation, the path to its config file. Each line of the config file is
   expected to be in the format,

     <key> <value>
*/

void ft_gpslog (S_thread_entry *p)
{
  static thread_local int cfg_gpsMode = 3 ;
  static thread_local int cfg_minSatsUsed = 4 ;
  static thread_local double cfg_maxPosDil = 3.5 ;

  static thread_local char cfg_gpsThread[MAX_THREAD_NAME] ; // initialize later
  static thread_local double cfg_minDistMeters = 12.0 ;
  static thread_local int cfg_normLogSecs = 10 ;
  static thread_local int cfg_maxLogSecs = 60 ;
  static thread_local int cfg_fileMaxSize = 262144 ;
  static thread_local char cfg_fileName[BUF_SIZE] ;

  static thread_local double cur_ele = 0.0 ;
  static thread_local double cur_lat = 0.0 ;
  static thread_local double cur_long = 0.0 ;
  static thread_local double cur_epoch = 0.0 ;

  static thread_local int ring_pos = 0 ;     // next empty entry
  static thread_local int ring_entries = 0 ; // dynamically calculated
  static thread_local long last_run = 0 ;    // time this function last ran
  static thread_local long last_update = 0 ; // time of last ring buffer update

  /* our ringer buffer */

  struct ring_entry_t
  {
    time_t epoch ;
    double latitude ;
    double longitude ;
    double elevation ;
    double dilution ;
  } ;
  typedef struct ring_entry_t S_RingEntry ;
  static thread_local S_RingEntry *ring_buffer = NULL ; // the ring buffer

  #define MAX_RING_BUFFER_ELEMENTS 200 // protect against crazy big malloc()

  /* a macro to check if we're told to terminate, make sure we free(). */

  #define GPSLOG_CHECK_CLEANUP ({                 \
      if (p->state != THREAD_RUNNING)             \
      {                                           \
        if (ring_buffer != NULL)                  \
        {                                         \
          free (ring_buffer) ;                    \
          ring_buffer = NULL ;                    \
        }                                         \
        strcpy (p->msg, "ring_buffer released") ; \
        return ; \
      } \
    })

  long now = millis () ; // note down this function's wall clock time

  /*
     before we return() for whatever reason, check if we're being shutdown,
     and if "ring_buffer" is malloc()'ed, better free() it now.
  */

  GPSLOG_CHECK_CLEANUP ;

  if (p->num_args != 1)
  {
    strcpy (p->msg, "FATAL! Expecting 1x argument") ;
    p->state = THREAD_STOPPED ; return ;
  }

  if (p->loops == 0)
  {
    /* configure results (ie, internal metrics) we want to expose */

    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = "ringbuffer" ;
    p->results[0].data[0] = "\"size\"" ;

    p->results[1].num_tags = 1 ;
    p->results[1].meta[0] = "entries" ;
    p->results[1].data[0] = "\"written\"" ;

    p->results[2].num_tags = 1 ;
    p->results[2].meta[0] = "entries" ;
    p->results[2].data[0] = "\"moving\"" ;

    p->results[3].num_tags = 1 ;
    p->results[3].meta[0] = "entries" ;
    p->results[3].data[0] = "\"stationary\"" ;

    p->results[4].num_tags = 1 ;
    p->results[4].meta[0] = "files" ;
    p->results[4].data[0] = "\"rotated\"" ;

    p->num_int_results = 5 ;

    /* parse our config file, load configuration into thread_local vars */

    strcpy (cfg_gpsThread, "") ;
    strcpy (cfg_fileName, "/gps.csv") ;

    File f = SPIFFS.open (p->in_args[0], "r") ;
    if (f)
    {
      char line[BUF_SIZE], buf[BUF_SIZE] ;
      while (1)
      {
        int amt = f.readBytesUntil ('\n', line, BUF_SIZE-1) ;
        if (amt > 0)
        {
          line[amt] = 0 ;
          char *idx=NULL, *key=NULL, *value=NULL ;

          key = strtok_r (line, " ", &idx) ;
          if (key != NULL)
          {
            value = strtok_r (NULL, " ", &idx) ;
            if (value != NULL)
            {
              if (G_debug)
              {
                snprintf (buf, BUF_SIZE, "DEBUG: ft_gpslog() (%s)(%s)",
                          key, value) ;
                Serial.println (buf) ;
              }
              if (strcmp (key, "cfg_gpsMode") == 0)
                cfg_gpsMode = atoi (value) ;
              if (strcmp (key, "cfg_minSatsUsed") == 0)
                cfg_minSatsUsed = atoi (value) ;
              if (strcmp (key, "cfg_maxPosDil") == 0)
                cfg_maxPosDil = atof (value) ;

              if ((strcmp (key, "cfg_gpsThread") == 0) &&
                  (strlen (cfg_gpsThread) < MAX_THREAD_NAME))
                strncpy (cfg_gpsThread, value, MAX_THREAD_NAME) ;
              if (strcmp (key, "cfg_minDistMeters") == 0)
                cfg_minDistMeters = atof (value) ;
              if (strcmp (key, "cfg_normLogSecs") == 0)
                cfg_normLogSecs = atoi (value) ;
              if (strcmp (key, "cfg_maxLogSecs") == 0)
                cfg_maxLogSecs = atoi (value) ;
              if (strcmp (key, "cfg_fileMaxSize") == 0)
                cfg_fileMaxSize = atoi (value) ;
              if (strcmp (key, "cfg_fileName") == 0)
                strncpy (cfg_fileName, value, BUF_SIZE) ;
            }
            else
            {
              strcpy (p->msg, "FATAL! Missing value in config") ;
              p->state = THREAD_STOPPED ; return ;
            }
          }
          else
          {
            strcpy (p->msg, "FATAL! Invalid line in config") ;
            p->state = THREAD_STOPPED ; return ;
          }
        }
        else
          break ;
      }
      f.close () ;

      /* do some sanity check on our configuration to spot obvious mistakes */

      char *fault = NULL ;
      if (cfg_minSatsUsed < 1)
        fault = "cfg_minSatsUsed cannot be less than 1" ;
      if (strlen(cfg_gpsThread) < 1)
        fault = "cfg_gpsThread cannot be empty" ;
      if (cfg_normLogSecs < 1)
        fault = "cfg_normLogSecs cannot be less than 1" ;
      if (cfg_maxLogSecs < cfg_normLogSecs)
        fault = "cfg_maxLogSecs cannot be less than cfg_normLogSecs" ;
      if (strlen(cfg_fileName) < 1)
        fault = "cfg_fileName cannot be empty" ;

      if (fault)
      {
        strcpy (p->msg, fault) ;
        p->state = THREAD_STOPPED ; return ;
      }
    }
    else
    {
      strcpy (p->msg, "FATAL! Cannot read configuration") ;
      p->state = THREAD_STOPPED ; return ;
    }

    /* calculate the size of our ring buffer and allocate memory */

    ring_entries = (cfg_maxLogSecs / cfg_normLogSecs) + 1 ;
    if (ring_entries > MAX_RING_BUFFER_ELEMENTS)
    {
      snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
                "FATAL! ring_buffer sz %d exceeds %d",
                 ring_entries, MAX_RING_BUFFER_ELEMENTS) ;
      p->state = THREAD_STOPPED ; return ;
    }
    size_t sz = ring_entries * sizeof(S_RingEntry) ;
    ring_buffer = (S_RingEntry*) malloc (sz) ;
    if (ring_buffer == NULL)
    {
      snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
                "FATAL! ring_buffer malloc(%d) failed", sz) ;
      p->state = THREAD_STOPPED ; return ;
    }

    p->results[0].i_value = sz ;
    last_run = now ;
  }

  /*
     attempt to identify the "cfg_gpsThread", make sure it's running. If not
     found, sleep and try again (but check if we're told to terminate).
  */

  int gps_tid ;
  for (gps_tid=0 ; gps_tid < MAX_THREADS ; gps_tid++)
    if ((G_thread_entry[gps_tid].state == THREAD_RUNNING) &&
        (G_thread_entry[gps_tid].ft_addr == ft_gpsmon) &&
        (strcmp(G_thread_entry[gps_tid].name, cfg_gpsThread) == 0))
      break ;
  if (gps_tid == MAX_THREADS) /* can't locate "cfg_gpsThread", retry */
  {
    strcpy (p->msg, "WARNING: Cannot find cfg_gpsThread") ;
    last_run = last_run + (cfg_normLogSecs * 1000) ;
    while (millis() < last_run)
    {
      delay (THREAD_SHUTDOWN_PERIOD / 20) ;
      GPSLOG_CHECK_CLEANUP ;
    }
    return ;
  }

  /*
     check if GPS has a good lock. If not, sleep and try again (but check if
     we're told to terminate).
  */

  #define IDX_GPS_MODE 9
  #define IDX_SATS_USED 6
  #define IDX_POS_DIL 10

  S_thread_result *r_ptr = G_thread_entry[gps_tid].results ;
  if ((cfg_gpsMode != (int) r_ptr[IDX_GPS_MODE].f_value) ||
      (cfg_minSatsUsed > (int) r_ptr[IDX_SATS_USED].f_value) ||
      (cfg_maxPosDil < r_ptr[IDX_POS_DIL].f_value))
  {
    snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
              "WARNING: No lock, mode:%d sats:%d dil:%s",
              (int) r_ptr[IDX_GPS_MODE].f_value,
              (int) r_ptr[IDX_SATS_USED].f_value,
              String (r_ptr[IDX_POS_DIL].f_value, FLOAT_DECIMAL_PLACES)) ;

    last_run = last_run + (cfg_normLogSecs * 1000) ;
    while (millis() < last_run)
    {
      delay (THREAD_SHUTDOWN_PERIOD / 20) ;
      GPSLOG_CHECK_CLEANUP ;
    }
    return ;
  }

  /*
     if this is the first time initializing our position, don't proceed.
     Just sleep and run again (but check if we're told to terminate).
  */

  #define IDX_ELE 7
  #define IDX_UTC 5
  #define IDX_LAT 3
  #define IDX_LONG 4

  if ((cur_ele == 0.0) && (cur_lat == 0.0) && (cur_long == 0.0))
  {
    strcpy (p->msg, "INFO: initialized") ;
    cur_ele = r_ptr[IDX_ELE].f_value ;
    cur_lat = r_ptr[IDX_LAT].f_value ;
    cur_long = r_ptr[IDX_LONG].f_value ;
    cur_epoch = r_ptr[IDX_UTC].f_value ;
    last_update = now ;

    last_run = last_run + (cfg_normLogSecs * 1000) ;
    while (millis() < last_run)
    {
      delay (THREAD_SHUTDOWN_PERIOD / 20) ;
      GPSLOG_CHECK_CLEANUP ;
    }
    return ;
  }

  /*
     if we've moved, or if we've not added any entry in a long time, add our
     current position to our ring buffer.
  */

  #define AVE_EARTH_RADIUS 6371008.8 // meters

  double earthRadius = AVE_EARTH_RADIUS + r_ptr[IDX_ELE].f_value ;
  double minDegrees = atan (cfg_minDistMeters / earthRadius) * 180.0 / M_PI ;

  int we_moved = 0 ;
  if ((abs(r_ptr[IDX_LAT].f_value - cur_lat) > minDegrees) ||
      (abs(r_ptr[IDX_LONG].f_value - cur_long) > minDegrees))
  {
    we_moved = 1 ;
    p->results[2].i_value++ ;
  }

  if (((we_moved) || (now - last_update > cfg_maxLogSecs * 1000)) &&
      (ring_pos < ring_entries)) // make sure there's space available
  {
    ring_buffer[ring_pos].epoch = int(r_ptr[IDX_UTC].f_value) ;
    ring_buffer[ring_pos].latitude = r_ptr[IDX_LAT].f_value ;
    ring_buffer[ring_pos].longitude = r_ptr[IDX_LONG].f_value ;
    ring_buffer[ring_pos].elevation = r_ptr[IDX_ELE].f_value ;
    ring_buffer[ring_pos].dilution = r_ptr[IDX_POS_DIL].f_value ;
    ring_pos++ ;
    last_update = now ;

    if (we_moved == 0)
      p->results[3].i_value++ ;
  }

  /*
     if our ring buffer is full, or if we've not flushed it in a long time,
     write it to storage
  */

  if (ring_pos == ring_entries)
  {
    File f = SPIFFS.open (cfg_fileName, "a") ;
    if (!f)
    {
      snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
                "FATAL! Cannot open %s for writing", cfg_fileName) ;
      free (ring_buffer) ;
      ring_buffer = NULL ;
      p->state = THREAD_STOPPED ; return ;
    }
    if ((f.size() < 1) &&
        (f.print ("time,latitude,longitude,elevation,dilution\n") < 1))
    {
      snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
                "FATAL! Cannot write to %s", cfg_fileName) ;
      free (ring_buffer) ;
      ring_buffer = NULL ;
      f.close () ;
      p->state = THREAD_STOPPED ; return ;
    }

    char line[BUF_SIZE] ;
    for (int i=0 ; i < ring_pos ; i++)
    {
      String s_lat = String(ring_buffer[i].latitude, FLOAT_DECIMAL_PLACES) ;
      String s_long = String(ring_buffer[i].longitude, FLOAT_DECIMAL_PLACES) ;
      String s_elev = String(ring_buffer[i].elevation, FLOAT_DECIMAL_PLACES) ;
      String s_dilu = String(ring_buffer[i].dilution, FLOAT_DECIMAL_PLACES) ;
      snprintf (line, BUF_SIZE-1, "%ld,%s,%s,%s,%s\n",
                ring_buffer[i].epoch,
                s_lat.c_str(),
                s_long.c_str(),
                s_elev.c_str(),
                s_dilu.c_str()) ;
      f.print (line) ;
      p->results[1].i_value++ ;
    }
    ring_pos = 0 ; // reset ring buffer since it got flushed

    /* check if it's time to rotate cfg_fileName */

    int rotate = 0 ;
    if (f.size() > cfg_fileMaxSize)
      rotate = 1 ;
    f.close () ;

    if (rotate) // ... delete old file if present
    {
      char dst_name[BUF_SIZE] ;
      snprintf (dst_name, BUF_SIZE-1, "%s.old", cfg_fileName) ;
      File f = SPIFFS.open (dst_name) ;
      if (f)
      {
        f.close () ;
        SPIFFS.remove (dst_name) ;
      }
      SPIFFS.rename (cfg_fileName, dst_name) ;
      p->results[4].i_value++ ;
    }
  }

  /*
     figure out how long to sleep, but take short naps and check if we're
     being asked to shutdown.
  */

  long duration = now + (cfg_normLogSecs * 1000) - millis () ;
  if (duration > 0)
  {
    snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
              "ok, ring:%d/%d sleep:%ldms", ring_pos, ring_entries, duration) ;
    last_run = last_run + (cfg_normLogSecs * 1000) ;
    while (millis() < last_run)
    {
      delay (THREAD_SHUTDOWN_PERIOD / 20) ;
      GPSLOG_CHECK_CLEANUP ;
    }
  }
}

/* =========================== */
/* thread management functions */
/* =========================== */

void *f_thread_lifecycle (void *p)
{
  S_thread_entry *entry = (S_thread_entry*) p ;

  /* initialize thread info, indicate thread is now running */

  xSemaphoreTake (entry->lock, portMAX_DELAY) ;
  entry->state = THREAD_RUNNING ;
  entry->core = xPortGetCoreID () ;
  xSemaphoreGive (entry->lock) ;

  while (entry->state == THREAD_RUNNING)
  {
    entry->ft_addr (entry) ;
    entry->loops++ ;
  }

  xSemaphoreTake (entry->lock, portMAX_DELAY) ;
  entry->state = THREAD_STOPPED ;
  xSemaphoreGive (entry->lock) ;

  while (1) // await death, freeRTOS doesn't like thread functions to return
    delay (1000) ;
}

void f_thread_create (char *name)
{
  int idx ;
  char line[BUF_SIZE] ;

  /* before we try creating threads, try reap dead ones (releases memory) */

  for (idx=0 ; idx < MAX_THREADS ; idx++)
    if (G_thread_entry[idx].state == THREAD_STOPPED)
    {
      xSemaphoreTake (G_thread_entry[idx].lock, portMAX_DELAY) ;
      G_thread_entry[idx].state = THREAD_READY;
      G_thread_entry[idx].name[0] = 0 ;
      xSemaphoreGive (G_thread_entry[idx].lock) ;
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

  xSemaphoreTake (G_thread_entry[idx].lock, portMAX_DELAY) ;

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
      for (i=0 ; i < MAX_THREAD_TAGS ; i++)
      {
        char *p = strtok (NULL, ",") ;
        if (p)
        {
          G_thread_entry[idx].tags[i] = p ;
          G_thread_entry[idx].tags[i+1] = NULL ;
        }
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
  G_thread_entry[idx].num_float_results = 0 ;
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
  if (strcmp (ft_taskname, "ft_dht22") == 0)
    G_thread_entry[idx].ft_addr = ft_dht22 ;
  if (strcmp (ft_taskname, "ft_ds18b20") == 0)
    G_thread_entry[idx].ft_addr = ft_ds18b20 ;
  if (strcmp (ft_taskname, "ft_dread") == 0)
    G_thread_entry[idx].ft_addr = ft_dread ;
  if (strcmp (ft_taskname, "ft_gpsmon") == 0)
    G_thread_entry[idx].ft_addr = ft_gpsmon ;
  if (strcmp (ft_taskname, "ft_gpslog") == 0)
    G_thread_entry[idx].ft_addr = ft_gpslog ;
  if (strcmp (ft_taskname, "ft_hcsr04") == 0)
    G_thread_entry[idx].ft_addr = ft_hcsr04 ;
  if (strcmp (ft_taskname, "ft_tread") == 0)
    G_thread_entry[idx].ft_addr = ft_tread ;
  if (strcmp (ft_taskname, "ft_relay") == 0)
    G_thread_entry[idx].ft_addr = ft_relay ;
  if (strcmp (ft_taskname, "ft_i2sin") == 0)
    G_thread_entry[idx].ft_addr = ft_i2sin ;
  if (strcmp (ft_taskname, "ft_i2sout") == 0)
    G_thread_entry[idx].ft_addr = ft_i2sout ;
  if (strcmp (ft_taskname, "ft_serial2tcp") == 0)
    G_thread_entry[idx].ft_addr = ft_serial2tcp ;

  if (G_thread_entry[idx].ft_addr == NULL)
  {
    G_thread_entry[idx].name[0] = 0 ;
    sprintf (line, "FAULT: no such ft_task '%s'.\r\n", ft_taskname) ;
    strcat (G_reply_buf, line) ;
    xSemaphoreGive (G_thread_entry[idx].lock) ;
    return ;
  }

  /* (almost) everything prep'ed ... finally create the thread */

  G_thread_entry[idx].state = THREAD_STARTING ;
  xTaskCreate ((TaskFunction_t) f_thread_lifecycle,
               G_thread_entry[idx].name,
               MAX_THREAD_STACK,
               &G_thread_entry[idx],
               tskIDLE_PRIORITY,
               &G_thread_entry[idx].tid) ;

  xSemaphoreGive (G_thread_entry[idx].lock) ;

  sprintf (line, "thread '%s' created in_args:%d\r\n",
           name, num_args) ;
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
      G_thread_entry[idx].state = THREAD_WRAPUP ; // give it a chance to finish
      delay (THREAD_SHUTDOWN_PERIOD) ;
      vTaskDelete (G_thread_entry[idx].tid) ;
      G_thread_entry[idx].state = THREAD_STOPPED ;
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
  char msg[BUF_MEDIUM] ;

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
    strncat (G_reply_buf,
      "[Thread Config Files]\r\n"
      "/autoexec.cfg  - <name>[,<nameN>...]\r\n"
      "/thread-<name> - (see below)\r\n"
      "/tags-<name>   - <metric>[,<tagN>=\"<valueN>\",...]\r\n"
      "\r\n"
      "[Currently available <ft_tasks>]\r\n"
      "ft_adxl335,<delay>,<aggr>,<xPin>,<yPin>,<zPin>,<pwrPin>\r\n"
      "ft_aread,<delay>,<inPin>,[pwrPin],[loThres],[hiThres]\r\n"
      "ft_dht22,<delay>,<dataPin>,<pwrPin>\r\n"
      "ft_ds18b20,<delay>,<dataPin>,<pwrPin>\r\n"
      "ft_dread,<delay>,<pin>,<0=norm,1=pullup>[,<trig_ms>]\r\n"
      "ft_tread,<delay>,<pin>,<loThres>,<hiThres>\r\n"
      "ft_counter,<delay>,<start_value>\r\n"
      "ft_gpsmon,<baud>,<RXpin>,<TXpin>\r\n"
      "ft_gpslog,<config file>\r\n"
      "ft_hcsr04,<delay>,<aggr>,<trigPin>,<echoPin>,<thres(cm)>\r\n"
      "ft_i2sin,<sampleRate>,<SCKpin>,<WSpin>,<SDpin>,<ip:port>[,<gain>]\r\n"
      "ft_i2sout,<sampleRate>,<SCKpin>,<WSpin>,<SDpin>,<UDPport>\r\n"
      "ft_relay,<pin>,<dur(secs)>\r\n"
      "ft_serial2tcp,<port>,<baud>,<RXpin>,<TXpin>\r\n"
      "\r\n"
      "[Notes]\r\n"
      "<delay> - interval between sampling (millisecs)\r\n"
      "<aggr>  - interval between aggregating results (millisecs)\r\n",
      REPLY_SIZE) ;
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

        snprintf (msg, BUF_MEDIUM,
                  "%d. %s:%s loops:%d cpu:%d age:%ld metric:%s msg:%s\r\n",
                  num,
                  G_thread_entry[i].name,
                  state,
                  G_thread_entry[i].loops,
                  G_thread_entry[i].core,
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

/* ------------------------------------------------------------------------- */

void f_action (char **tokens)
{
  char line[BUF_SIZE] ;

  if ((strcmp(tokens[0], "?") == 0) || (strcmp(tokens[0], "help") == 0))
  {
    strcat (G_reply_buf,
            "[GPIO]\r\n"
            "hi <GPIO pin> [pulse (usecs)]\r\n"
            "lo <GPIO pin>\r\n"
            "aread <GPIO pin>  - analog read (always 0 on esp8266)\r\n"
            "dread <GPIO pin>  - digital read\r\n"
            "tread <GPIO pin>  - capacitive touch read\r\n"
            "bmp180            - barometric pressure (I2C)\r\n"
            "dht22 <dataPin>   - DHT-22 temperature/humidity sensor\r\n"
            "ds18b20 <dataPin> - DS18B20 temperature sensor\r\n"
            "hcsr04 <trigPin> <echoPin> - HC-SR04 ultrasonic ranger\r\n"
            "adxl335 <Xpin> <Ypin> <Zpin> <Time(ms)> <Interval(ms)>\r\n"
            "relay <thread name> on|off\r\n"
            "tone <GPIO pin> <freq> <dur(ms)>\r\n"
            "\r\n"
            "lcd init          - LCD on I2C\r\n"
            "lcd backlight <on/off>\r\n"
            "lcd clear\r\n"
            "lcd print <row> <col> <message...>\r\n"
            "\r\n"
            "[System]\r\n"
            "debug <num>\r\n"
            "file recv <port> <filename>\r\n"
            "file send <port> <filename>\r\n"
            "fs format\r\n"
            "fs info\r\n"
            "fs ls\r\n"
            "fs read <filename>\r\n"
            "fs rm <filename>\r\n"
            "fs rename <old> <new>\r\n"
            "fs write <filename> <content>\r\n"
            "mqtt connect\r\n"
            "mqtt disconnect\r\n"
            "ota <url>\r\n"
            "pinflags\r\n"
            "reload\r\n"
            "wifi connect\r\n"
            "wifi scan\r\n"
            "wifi status\r\n"
            "wifi ssid <ssid>\r\n"
            "wifi pw <password>\r\n"
            "wifi disconnect\r\n"
            "uptime\r\n"
            "version\r\n"

            "\r\n[ESP32 specific]\r\n"
            "esp32 hall\r\n"
            "esp32 sleep <secs>\r\n"
            "esp32 thread_help\r\n"
            "esp32 thread_list\r\n"
            "esp32 thread_start <name>\r\n"
            "esp32 thread_stop <name>\r\n"

            "\r\n[Config Files]\r\n"
            "/hostname   <hostname>\r\n"
            "/mqtt.cfg   <host>,<port>,<user>,<pw>\r\n"
            "/mqtt.pub   <sensor topic pub prefix>,<cmd topic pub prefix>\r\n"
            "/mqtt.sub   <cmd topic prefix>\r\n"
            "/mqtt.tags  <meta=\"value\",...>\r\n"
            "/wifi.ssid  <ssid>\r\n"
            "/wifi.pw    <pw>\r\n") ;
  }
  else
  if ((strcmp(tokens[0], "debug") == 0) && (tokens[1] != NULL))
  {
    G_debug = atoi(tokens[1]) ;
    sprintf (line, "Debug level set to %d.\r\n", G_debug) ;
    strcat (G_reply_buf, line) ;
  }
  else
  if ((strcmp(tokens[0], "hi") == 0) && (tokens[1] != NULL))
  {
    f_hi (tokens) ;
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
  if ((strcmp(tokens[0], "tread") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    int val = touchRead (pin) ;
    sprintf (line, "touchRead pin:%d - %d\r\n", pin, val) ;
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
               int(t), abs((int)(t*100)%100), int(h), (int)(h*100)%100) ;
      strcat (G_reply_buf, line) ;
    }
  }
  else
  if ((strcmp(tokens[0], "ds18b20") == 0) && (tokens[1] != NULL))
  {
    float t[MAX_DS18B20_DEVICES] ;
    unsigned char addr[(MAX_DS18B20_DEVICES * 8) + 1] ;
    memset (addr, 0, (MAX_DS18B20_DEVICES * 8) + 1) ;

    int results = f_ds18b20 (atoi(tokens[1]), addr, t) ;
    int offset = 0 ;
    for (int i=0 ; i < results ; i++)
    {
      sprintf (line, "ds18b20 - addr:%02x%02x%02x%02x%02x%02x%02x%02x "
               "temperature:%d.%02d\r\n",
               addr[offset], addr[offset+1], addr[offset+2], addr[offset+3],
               addr[offset+4], addr[offset+5], addr[offset+6], addr[offset+7],
               int(t[i]), abs((int)(t[i]*100)%100)) ;
      strcat (G_reply_buf, line) ;
      offset = offset + 8 ;
    }
  }
  else
  if ((strcmp(tokens[0], "hcsr04") == 0) && 
      (tokens[1] != NULL) && (tokens[2] != NULL))
  {
    float f = f_hcsr04 (atoi(tokens[1]), atoi(tokens[2])) ;
    if (f < 0.0)
    {
      strcat (G_reply_buf, "FAULT: f_hcsr04() no response.\r\n") ;
    }
    else
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
    /* attempt to measure tick period in milliseconds */

    #define TEST_TICKS 200
    int tv_start = millis () ;
    vTaskDelay (TEST_TICKS) ;
    int tv_end = millis () ;
    float tick_ms = (float) (tv_end - tv_start) / (float) TEST_TICKS ;
    char tick_str[8] ;
    dtostrf (tick_ms, 2, 4, tick_str) ;

    /* detect cpu clock frequency */

    rtc_cpu_freq_t cpu_freq = rtc_clk_cpu_freq_get() ;
    uint32_t hz = rtc_clk_cpu_freq_value (cpu_freq) ;

    /* now report all our findings */

    sprintf (line, "Running on cpu:%d\r\n", xPortGetCoreID()) ;
    strcat (G_reply_buf, line) ;
    sprintf (line, "CPU speed: %u hz\r\n", hz) ;
    strcat (G_reply_buf, line) ;
    sprintf (line, "Built: %s, %s\r\n", __DATE__, __TIME__) ;
    strcat (G_reply_buf, line) ;
    sprintf (line, "Thread priority: %d/%d\r\n",
             uxTaskPriorityGet(NULL), configMAX_PRIORITIES) ;
    strcat (G_reply_buf, line) ;
    sprintf (line, "Data sizes: ptr:%d char:%d "
                   "short:%d int:%d long:%d longlong:%d "
                   "float:%d double:%d\r\n",
             sizeof(void*), sizeof(char),
             sizeof(short), sizeof(int), sizeof(long), sizeof(long long),
             sizeof(float), sizeof(double)) ;
    strcat (G_reply_buf, line) ;
    sprintf (line, "Tick duration: %s ms\r\n", tick_str) ;
    strcat (G_reply_buf, line) ;
  }
  else
  if (strcmp(tokens[0], "lcd") == 0)
  {
    f_lcd (tokens) ;
  }
  else
  if ((strcmp(tokens[0], "fs") == 0) && (tokens[1] != NULL))
  {
    f_fs (tokens) ;
  }
  else
  if ((strcmp(tokens[0], "file") == 0) && (tokens[1] != NULL))
  {
    f_file (tokens) ;
  }
  else
  if ((strcmp(tokens[0], "wifi") == 0) && (tokens[1] != NULL))
  {
    f_wifi (tokens) ;
  }
  else
  if (strcmp(tokens[0], "pinflags") == 0)
  {
    f_pin_flags () ;
  }
  else
  if (strcmp(tokens[0], "reload") == 0)
  {
    Serial.println ("Rebooting.") ;
    strcat (G_reply_buf, "Rebooting.\r\n") ;
    G_reboot = 1 ;
  }
  else
  if ((strcmp(tokens[0], "mqtt") == 0) && (tokens[1] != NULL))
  {
    f_mqtt_ctl (tokens[1]) ;
  }
  else
  if ((strcmp(tokens[0], "ota") == 0) && (tokens[1] != NULL))
  {
    f_ota (tokens[1]) ;
  }
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
  else
  if ((strcmp(tokens[0], "relay") == 0) && (tokens[1] != NULL) &&
      (tokens[2] != NULL))
  {
    f_relay (tokens[1], tokens[2]) ;
  }
  else
  if ((strcmp(tokens[0], "tone") == 0) && (tokens[1] != NULL) &&
      (tokens[2] != NULL) && (tokens[3] != NULL))
  {
    #define CHANNEL 0                           // 16x PWM channels on an ESP32
    #define MAX_DUR 2000                        // don't block for too long

    int pin = atoi(tokens[1]) ;
    double freq = atoi(tokens[2]) ;
    unsigned long dur = atoi(tokens[3]) ;

    if (dur > MAX_DUR)
      dur = MAX_DUR ;
    if (freq < 1.0) // freq of 0.0 causes a panic
      freq = 1.0 ;

    double d = ledcWriteTone (CHANNEL, freq) ;
    ledcAttachPin (pin, CHANNEL) ;
    delay (dur) ;
    ledcDetachPin (pin) ;
    pinMode (pin, INPUT) ;              // this suppresses "buzzing" noises

    sprintf (line, "PWM %.2fhz\r\n", d) ;
    strcat (G_reply_buf, line) ;
  }
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
  G_serial_pos = 0 ;
  G_debug = 0 ;

  cfg_wifi_ssid[0] = 0 ;
  cfg_wifi_pw[0] = 0 ;
  G_next_cron = CRON_INTERVAL * 1000 ;
  pinMode (LED_BUILTIN, OUTPUT) ;

  /* early initialization for primary config global vars */

  G_hostname = (char*) malloc (BUF_SIZE+1) ;
  G_mqtt_pub = (char*) malloc (MAX_MQTT_LEN + 1) ;
  G_hostname[0] = 0 ;
  G_mqtt_pub[0] = 0 ;

  char line[BUF_MEDIUM] ;
  if (SPIFFS.begin())
  {
    int amt ;
    File f_cfg, f_pw ;

    Serial.println ("NOTICE: Checking built-in configuration.") ;

    /* if other config files are present, load them into memory now */

    f_cfg = SPIFFS.open (MQTT_PUB_FILE, "r") ;
    f_cfg.seek (0) ;
    amt = f_cfg.readBytes (G_mqtt_pub, BUF_MEDIUM) ;
    if (amt < 1)
    {
      snprintf (line, BUF_MEDIUM, "WARNING: %s read returned %d - %s",
                MQTT_PUB_FILE, amt, strerror(errno)) ;
      Serial.println (line) ;
    }
    else
    {
      G_mqtt_pub[amt] = 0 ;
      snprintf (line, BUF_MEDIUM, "NOTICE: publish prefix '%s'.", G_mqtt_pub) ;
      Serial.println (line) ;
    }
    f_cfg.close () ;

    f_cfg = SPIFFS.open (HOSTNAME_FILE, "r") ;
    amt = f_cfg.readBytes (G_hostname, BUF_MEDIUM) ;
    if (amt < 1)
    {
      snprintf (line, BUF_MEDIUM, "WARNING: %s read returned %d - %s",
                HOSTNAME_FILE, amt, strerror(errno)) ;
      Serial.println (line) ;
    }
    else
    {
      G_hostname[amt] = 0 ;
      snprintf (line, BUF_MEDIUM, "NOTICE: hostname is '%s'.", G_hostname) ;
      Serial.println (line) ;
    }
    f_cfg.close () ;


    /* if wifi ssid and password are available, try connect to wifi now.  */

    f_cfg = SPIFFS.open (WIFI_SSID_FILE) ;
    f_pw = SPIFFS.open (WIFI_PW_FILE) ;
    if ((f_cfg) && (f_pw) &&
        (f_cfg.size() > 0) && (f_pw.size() > 0) &&
        (f_cfg.size() <= MAX_SSID_LEN) && (f_pw.size() <= MAX_PASSWD_LEN))
    {
      int s_amt = f_cfg.readBytes (cfg_wifi_ssid, MAX_SSID_LEN) ;
      if (s_amt > 0)
        cfg_wifi_ssid[s_amt] = 0 ;
      int p_amt = f_pw.readBytes (cfg_wifi_pw, MAX_PASSWD_LEN) ;
      if (p_amt > 0)
        cfg_wifi_pw[p_amt] = 0 ;
      if ((s_amt > 0) && (p_amt > 0))
      {
        sprintf (line, "NOTICE: Wifi config loaded for %s.", cfg_wifi_ssid) ;
        Serial.println (line) ;
        strcpy (line, "NOTICE: ") ;
        f_wifiConnect (cfg_wifi_ssid, cfg_wifi_pw, line) ;
        Serial.println (line) ;

        /* fire up our web server socket */

        G_sd = socket (AF_INET, SOCK_STREAM, 0) ;
        if (G_sd > 0)
        {
          struct sockaddr_in saddr ;
          saddr.sin_family = AF_INET ;
          saddr.sin_addr.s_addr = htonl(INADDR_ANY) ;
          saddr.sin_port = htons (WEB_PORT) ;

          if ((bind (G_sd, (struct sockaddr*) &saddr, sizeof(saddr)) == 0) &&
              (listen (G_sd, MAX_SD_BACKLOG) == 0))
          {
            sprintf (line, "NOTICE: Webserver started on port %d.", WEB_PORT) ;
            Serial.println (line) ;
          }
        }
      }
    }
    if (f_cfg)
      f_cfg.close () ;
    if (f_pw)
      f_pw.close () ;
  }
  else
  {
    Serial.println ("WARNING: Could not initialize SPIFFS.") ;
  }

  /* for all large(r) string buffers or structures, do malloc() now */

  G_Metrics = (S_Metrics*) malloc (sizeof(S_Metrics)) ;
  memset (G_Metrics, 0, sizeof(S_Metrics)) ;
  G_hw_uart = (S_hw_uart*) malloc (sizeof(S_hw_uart)) ;
  memset (G_hw_uart, 0, sizeof(S_hw_uart)) ;
  G_hw_uart->lock = xSemaphoreCreateMutex () ;

  G_mqtt_sub = (char*) malloc (MAX_MQTT_LEN + 1) ;
  G_mqtt_stopic = (char*) malloc (MAX_MQTT_LEN + 1) ;
  G_mqtt_rtopic = (char*) malloc (MAX_MQTT_LEN + 1) ;
  G_mqtt_tags = (char*) malloc (MAX_MQTT_LEN + 1) ;
  G_pub_topic = (char*) malloc (MAX_MQTT_LEN + 1) ;
  G_pub_payload = (char*) malloc (MAX_MQTT_LEN + 1) ;
  G_reply_buf = (char*) malloc (REPLY_SIZE+1) ;
  G_serial_buf = (char*) malloc (BUF_SIZE+1) ;

  G_mqtt_sub[0] = 0 ;
  G_mqtt_stopic[0] = 0 ;
  G_mqtt_rtopic[0] = 0 ;
  G_mqtt_tags[0] = 0 ;
  G_pub_topic[0] = 0 ;
  G_pub_payload[0] = 0 ;
  G_reply_buf[0] = 0 ;
  G_serial_buf[0] = 0 ;

  G_WebClient = (S_WebClient*) malloc (sizeof(S_WebClient) *
                                       MAX_HTTP_CLIENTS) ;
  int i, j ;
  size_t sz ;
  for (i=0 ; i < MAX_HTTP_CLIENTS ; i++)
    memset (&G_WebClient[i], 0, sizeof(S_WebClient)) ;

  G_pin_flags = (S_pin_flag*) malloc (MAX_GPIO_PINS * sizeof(S_pin_flag)) ;
  for (i=0 ; i < MAX_GPIO_PINS ; i++)
  {
    memset (&G_pin_flags[i], 0, sizeof(S_pin_flag)) ;
    sz = MAX_THREADS * sizeof(S_thread_entry*) ;
    G_pin_flags[i].users = (S_thread_entry**) malloc (sz) ;
    for (j=0 ; j<MAX_THREADS ; j++)
      G_pin_flags[i].users[j] = NULL ;
  }
  G_pinflags_lock = xSemaphoreCreateMutex () ;

  sz = MAX_THREADS * sizeof(S_thread_entry) ;
  G_thread_entry = (S_thread_entry*) malloc (sz) ;
  for (i=0 ; i < MAX_THREADS ; i++)
  {
    memset (&G_thread_entry[i], 0, sizeof(S_thread_entry)) ;
    G_thread_entry[i].lock = xSemaphoreCreateMutex () ;
  }
  sprintf (line, "NOTICE: Max allowed threads %d.", MAX_THREADS) ;
  Serial.println (line) ;
  G_publish_lock = xSemaphoreCreateMutex () ;

  digitalWrite (LED_BUILTIN, LOW) ;           // 1-sec blink to indicate boot.
  delay (1000) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
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
    G_Metrics->serialInBytes++ ;
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
      G_Metrics->serialCmds++ ;
      break ;
    }

    if ((c != '\r') && (c != '\b'))
      G_serial_pos++ ;
  }
  if (G_serial_pos == BUF_SIZE-1)                        // buffer overrun
  {
    Serial.println ("FAULT: Input overrun.") ;
    G_Metrics->serialOverruns++ ;
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
    int rlen = strlen (G_reply_buf) ;
    if (rlen > 0)
    {
      if (rlen >= REPLY_SIZE)
      {
        char line[BUF_SIZE] ;
        sprintf (line, "\r\nWARNING: G_reply_buf is %d bytes, max %d.\r\n",
                 rlen, REPLY_SIZE) ;
        Serial.print (line) ;
      }

      Serial.print (G_reply_buf) ;
      if (G_reply_buf[strlen(G_reply_buf)-1] != '\n')
        Serial.print ("\r\n") ;                         // add CRNL if needed
    }
    Serial.println ("OK") ;
    G_serial_buf[0] = 0 ;
    G_serial_pos = 0 ;
  }

  if (G_psClient.connected())
  {
    xSemaphoreTake (G_publish_lock, portMAX_DELAY) ;
    if (strlen(G_pub_topic) > 0)
    {
      if (G_debug)
      {
        char line[BUF_SIZE] ;
        sprintf (line, "DEBUG: publishing %d bytes.", strlen(G_pub_payload)) ;
        Serial.println (line) ;
      }

      /* publish our "payload" to "topic" */

      G_psClient.publish (G_pub_topic, G_pub_payload) ;
      G_pub_topic[0] = 0 ;
      G_pub_payload[0] = 0 ;
      G_Metrics->mqttPubs++ ;
    }
    xSemaphoreGive (G_publish_lock) ;

    G_psClient.loop () ; // handle any incoming messages
  }

  /* handle web requests, only if "G_sd" was set */

  if (G_sd > 0)
    f_handleWebServer () ;

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

  /* once in a while, run "cron" jobs */

  if (now > G_next_cron)
  {
    f_cron () ;
    G_next_cron = G_next_cron + (CRON_INTERVAL * 1000) ;
  }

  delay (10) ; // mandatory sleep to prevent excessive power drain

  if (G_sleep)
  {
    G_sleep = 0 ;
    Serial.println ("NOTICE: entering sleep mode.") ;
    delay (100) ; // allow serial buffer to flush
    esp_light_sleep_start () ;
    Serial.println ("NOTICE: exiting sleep mode.") ;
  }

  if (G_reboot)
    ESP.restart () ;
}

