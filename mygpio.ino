/*
   Build/Upload

     Note that this code has been tested with
       - esp32 core v1.0.6
       - arduino-cli v0.29.0

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
         `-> the thread function f_thread_lifecycle()
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
     - Since the ESP32's (per thread) stack is fairly small, we should store
       variables in heap as much as possible. A thread is free to malloc()
       a SINGLE buffer and reference it at S_thread_entry->buf. This will be
       automatically free()'ed after the thread terminates.

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

char *cfg_wifi_ssid ;
char *cfg_wifi_pw ;

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
SemaphoreHandle_t G_hcsr04_lock ;    // serialize HC-SR04 sampling
SemaphoreHandle_t G_fast_aread_lock ; // serialize ft_fast_aread()

S_Metrics *G_Metrics ;
S_thread_entry *G_thread_entry ;
S_WebClient *G_WebClient ;
S_CamMgt *G_CamMgt=NULL ;
S_hw_uart *G_hw_uart ;
S_pwm_instance *G_pwm_instance ;
LiquidCrystal_I2C G_lcd (LCD_ADDR, LCD_WIDTH, LCD_ROWS) ;

WiFiClient G_wClient ; // this is needed to instanciate "G_psClient"
PubSubClient G_psClient (G_wClient) ;
camera_config_t *G_cam_config = NULL ;  // used on esp32-cam platforms

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

/* ================= */
/* System Management */
/* ================= */

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
      "ft_fast_aread,<delay>,<samples>,<gap_ms>,<inPin>[,<rolling_ms>]\r\n"
      "ft_dht22,<delay>,<dataPin>,<pwrPin>\r\n"
      "ft_ds18b20,<delay>,<dataPin>,<pwrPin>[,noaddr]\r\n"
      "ft_dread,<delay>,<pin>,<0=norm,1=pullup>[,<trig_ms>]\r\n"
      "ft_tread,<delay>,<pin>,<loThres>,<hiThres>\r\n"
      "ft_counter,<delay>,<start_value>\r\n"
      "ft_gpsmon,<baud>,<RXpin>,<TXpin>\r\n"
      "ft_gpslog,<config file>\r\n"
      "ft_hcsr04,<delay>,<aggr>,<trigPin>,<echoPin>,<thres(cm)>\r\n"
      "ft_i2sin,<sampleRate>,<SCKpin>,<WSpin>,<SDpin>,<ip:port>[,<gain>]\r\n"
      "ft_i2sout,<sampleRate>,<SCKpin>,<WSpin>,<SDpin>,<UDPport>\r\n"
      "ft_ntpclient,<ntp_server>,<sync_interval_secs>\r\n"
      "ft_relay,<pin>,<dur(secs)>\r\n"
      "ft_serial2tcp,<port>,<baud>,<RXpin>,<TXpin>\r\n"
      "ft_watchdog,<intervalSecs>,<minUptimeSecs>,<noActivitySecs>\r\n"
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
        char *state = (char*) "running" ;
        char *metric = (char*) "(none)" ;
        if (G_thread_entry[i].state == THREAD_STOPPED)
          state = (char*) "stopped" ;
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

void setup ()
{
  Serial.begin (DEF_BAUD) ;
  Serial.setTimeout (SERIAL_TIMEOUT) ;

  unsigned char mac[6] ;
  char line[BUF_MEDIUM] ;

  WiFi.macAddress(mac) ;
  sprintf (line, "\nNOTICE: System boot with wifi mac %x:%x:%x:%x:%x:%x.",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]) ;
  Serial.println (line) ;
  sprintf (line, "NOTICE: Built on %s %s", __DATE__, __TIME__) ;
  Serial.println (line) ;

  G_serial_pos = 0 ;
  G_debug = 0 ;
  G_next_cron = CRON_INTERVAL * 1000 ;
  pinMode (LED_BUILTIN, OUTPUT) ;

  /* early initialization for primary config global vars */

  cfg_wifi_ssid = (char*) malloc (MAX_SSID_LEN + 1) ;
  cfg_wifi_pw = (char*) malloc (MAX_PASSWD_LEN + 1) ;
  cfg_wifi_ssid[0] = 0 ;
  cfg_wifi_pw[0] = 0 ;
  G_hostname = (char*) malloc (BUF_SIZE+1) ;
  G_mqtt_pub = (char*) malloc (MAX_MQTT_LEN + 1) ;
  G_hostname[0] = 0 ;
  G_mqtt_pub[0] = 0 ;
  G_pwm_instance = NULL ;

  if (SPIFFS.begin())
  {
    int amt ;
    File f_cfg, f_pw ;

    /* if other config files are present, load them into memory now */

    f_cfg = SPIFFS.open (MQTT_PUB_FILE, "r") ;
    f_cfg.seek (0) ;
    amt = f_cfg.readBytes (G_mqtt_pub, BUF_MEDIUM) ;
    if (amt > 0)
    {
      G_mqtt_pub[amt] = 0 ;
      snprintf (line, BUF_MEDIUM, "NOTICE: publish prefix '%s'.", G_mqtt_pub) ;
      Serial.println (line) ;
    }
    f_cfg.close () ;

    f_cfg = SPIFFS.open (HOSTNAME_FILE, "r") ;
    amt = f_cfg.readBytes (G_hostname, BUF_MEDIUM) ;
    if (amt > 0)
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
        sprintf (line,
                 "NOTICE: Connecting to %s, disconnect serial terminal now.",
                 cfg_wifi_ssid) ;
        Serial.println (line) ;
        WiFi.begin (cfg_wifi_ssid, cfg_wifi_pw) ;

        /* fire up our web server socket */

        G_sd = socket (AF_INET, SOCK_STREAM, 0) ;
        if (G_sd > 0)
        {
          struct sockaddr_in saddr ;
          saddr.sin_family = AF_INET ;
          saddr.sin_addr.s_addr = htonl(INADDR_ANY) ;
          saddr.sin_port = htons (WEB_PORT) ;
          bind (G_sd, (struct sockaddr*) &saddr, sizeof(saddr)) ;
          listen (G_sd, MAX_SD_BACKLOG) ;
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
  G_hcsr04_lock = xSemaphoreCreateMutex () ;
  G_fast_aread_lock = xSemaphoreCreateMutex () ;

  sz = MAX_THREADS * sizeof(S_thread_entry) ;
  G_thread_entry = (S_thread_entry*) malloc (sz) ;
  for (i=0 ; i < MAX_THREADS ; i++)
  {
    memset (&G_thread_entry[i], 0, sizeof(S_thread_entry)) ;
    G_thread_entry[i].lock = xSemaphoreCreateMutex () ;
  }
  G_publish_lock = xSemaphoreCreateMutex () ;

  Wire.begin () ;

  digitalWrite (LED_BUILTIN, LOW) ;           // 1-sec blink to indicate boot.
  delay (1000) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
  Serial.println ("NOTICE: Ready. Type 'help' for commands.") ;
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

