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

#include "mygpio.h"

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

/* =============================== */
/* esp32 platform native functions */
/* =============================== */

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
            "/autoexec.cfg <thread1>[,<threadN>,...]\r\n"
            "/hostname     <hostname>\r\n"
            "/mqtt.cfg     <host>,<port>,<user>,<pw>\r\n"
            "/mqtt.pub     <sensor topic pub prefix>,<cmd topic pub prefix>\r\n"
            "/mqtt.sub     <cmd topic prefix>\r\n"
            "/mqtt.tags    <meta=\"value\",...>\r\n"
            "/wifi.ssid    <ssid>\r\n"
            "/wifi.pw      <pw>\r\n") ;
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

