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

/*
   this is our cron-like function whose role is to make sure we stay connected
   to wifi and mqtt (optional)
*/

void f_cron ()
{
  char msg[BUF_SIZE] ;

  if (G_debug)
    Serial.println ("DEBUG: f_cron()") ;

  /*
     Under certain circumstances, we want to try auto-reconnecting our wifi,
     even though our wifi state is WL_CONNECTED. In particular,
       a) if our rssi is 0 dBm (ie, not connected)
       b) our rssi is poorer than the value in WIFI_RSSI_FILE, eg "-72" (dBm)
       b) our mqtt state is MQTT_CONNECTION_LOST
  */

  int wifi_reconnect = 0 ;

  if (WiFi.status() == WL_CONNECTED)
  {
    File f = SPIFFS.open (WIFI_RSSI_FILE, "r") ;
    int amt = f.readBytes (msg, BUF_SIZE-1) ;
    f.close () ;
    if (amt > 0)
    {
      msg[amt] = 0 ;
      if ((WiFi.RSSI() == 0) || (WiFi.RSSI() < atoi(msg)))
        wifi_reconnect = 1 ;
    }
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
    args[0] = (char*) "wifi" ;
    args[1] = (char*) "connect" ;
    args[2] = NULL ;
    f_wifi (args) ;
    G_Metrics->wifiReconnects++ ;
  }

  /* If MQTT is configured, but not connected, try reconnect */

  File f = SPIFFS.open (MQTT_CFG_FILE, "r") ;
  int amt = f.readBytes (msg, BUF_SIZE-1) ;
  f.close () ;
  if ((amt > 0) &&
      ((G_psClient.connected() == false) ||
       (G_psClient.state() != MQTT_CONNECTED)))
  {
    if (G_debug)
      Serial.println ("DEBUG: f_cron() calling f_mqtt_connect()") ;
    f_mqtt_connect () ;
  }

  /* If this is our first run, check if AUTOEXEC_FILE exists */

  if (G_Metrics->cronRuns == 0)
  {
    File f = SPIFFS.open (AUTOEXEC_FILE, "r") ;
    if (f)
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

/*
   this function rapidly reads analog input(s), saving the requested number of
   samples in to memory before writing it to a file (which can be transferred
   later). Since we have a variable number of arguments, this function is
   supplied a NULL terminated array of string pointers, in the following
   sequence:

     num    - total number of samples we'll be observing
     gap_ms - interval between samples, all inputs are sampled together
     file   - the output file we'll save results into
     ...    - a variable list of GPIO inputs

   During operation, we need to ensure other threads don't affect our
   analogRead(). Thus we acquire "G_fast_aread_lock" during our busy loop.

   Example usage:

     sampler 5000 1 /myfile.json 36 39 34 35

   In the above example, a total of 5000 samples will be taken 1ms apart.
   Before performing the sampling, we'll spend 200ms trying to tune the
   "ets_delay_us()" gap between samples. We will then sample the values on
   GPIO pins 36, 39, 34 and 35. We write to the results to "/myfile.json".

     sampler 600 100 /over1min.json 36

   In the above example, a total of 600 samples are taken in 100ms intervals,
   thus spanning 1 minute. Note that the ESP32 will be UNRESPONSIVE during
   this time because it is busy with this command. Thus the user should always
   consider overall command timings before running this. To prevent the ESP32
   from locking up too long, this function will abort the command if it
   calculates we'll be locked up for longer than SAMPLER_MAX_TIME_MS.

   Samples are stored in a single buffer made up of an array of int for
   simplicity.

     [ <32-bit_ts><gpio1_value>{<gpioN_value>...}, ... ]

   An ESP32 (with an 80MHz cpu) typically does 1000x analogRead() calls in
   86 ms, or about 86 usec per call. Since we have "cal_ms" to figure out our
   optimal "gap_usec" value for ets_delay_us(), we'll do 2x test runs. The
   first uses our theoretical timings and then second run uses our adjusted
   timings.

   When writing to the output file, it follows the following JSON format,

     {
       "samples" : [
         [ <ts1>, <gpio1>, ... ],
         ...
       ]
     }
*/

void f_sampler (char **args)
{
  #define SAMPLER_MAX_GPIO_INPUTS 4     // limited by MAX_TOKENS - 1
  #define SAMPLER_MAX_SAMPLES 5000      // note, each sample is 4 bytes
  #define SAMPLER_MAX_GAP_MS 1000       // maximum allowed sample gap in msecs
  #define SAMPLER_MAX_TIME_MS 20000     // don't run for more than 20 secs
  #define DEF_ANALOGREAD_TIME_USEC 86   // time for a single analogRead()

  int i, count, total_args=0, num_samples, gap_ms, num_gpio ;
  int len=0, offset, sbuf_len, gap_usec ;
  int gpio_list[SAMPLER_MAX_GPIO_INPUTS] ;
  unsigned long *sample_buf=NULL ;
  char *outfile, line[BUF_SIZE] ;
  unsigned long start_t, end_t, dur ;

  /* attempt to parse our arguments and sanity check them */

  while (total_args < SAMPLER_MAX_GPIO_INPUTS + 5)
    if (args[total_args] == NULL) break ;
      else total_args++ ;

  if ((total_args == (SAMPLER_MAX_GPIO_INPUTS + 5)) &&
      (args[total_args-1] != NULL))
  {
    strcpy (G_reply_buf, "FAULT: Too many arguments\r\n") ;
    return ;
  }
  num_gpio = total_args - 4 ;
  num_samples = atoi (args[1]) ;
  gap_ms = atoi (args[2]) ;
  outfile = args[3] ;
  for (i=0 ; i < num_gpio ; i++)
    gpio_list[i] = atoi (args[i+4]) ;

  if ((gap_ms < 1) || (gap_ms > SAMPLER_MAX_GAP_MS))
  {
    sprintf (G_reply_buf, "FAULT: Invalid gap_ms (1-%d).\r\n",
             SAMPLER_MAX_GAP_MS) ;
    return ;
  }
  if (outfile[0] != '/')
  {
    strcpy (G_reply_buf, "FAULT: Invalid filename.\r\n") ;
    return ;
  }
  if ((num_samples < 1) || (num_samples > SAMPLER_MAX_SAMPLES))
  {
    sprintf (G_reply_buf, "FAULT: Invalid number of samples (1-%d).\r\n",
             SAMPLER_MAX_SAMPLES) ;
    return ;
  }
  dur = num_samples * gap_ms ;
  if (dur > SAMPLER_MAX_TIME_MS)
  {
    sprintf (G_reply_buf, "FAULT: total duration %dms exceeds %dms.\r\n",
             dur, SAMPLER_MAX_TIME_MS) ;
    return ;
  }

  /* prepare the sample buffer */

  sbuf_len = num_samples * (sizeof(unsigned long) * (num_gpio + 1)) ;
  sample_buf = (unsigned long*) malloc (sbuf_len) ;
  if (sample_buf == NULL)
  {
    sprintf (G_reply_buf, "FAULT: could not malloc sample_buf, %d bytes.\r\n",
            sbuf_len) ;
    return ;
  }

  /* calculate "gap_usec" based on theoretical timing of analogRead() */

  gap_usec = (gap_ms * 1000) - (DEF_ANALOGREAD_TIME_USEC * num_gpio) ;

  xSemaphoreTake (G_fast_aread_lock, portMAX_DELAY) ;
  start_t = millis () ;
  for (count=0 ; count < num_samples ; count++)
  {
    offset = count * (num_gpio + 1) ;
    sample_buf[offset] = millis () ;
    for (i=0 ; i < num_gpio ; i++)
    {
      offset = (count * (num_gpio + 1)) + 1 + i ;
      sample_buf[offset] = analogRead (gpio_list[i]) ;
    }
    ets_delay_us (gap_usec) ;
  }
  end_t = millis () ;
  xSemaphoreGive (G_fast_aread_lock) ;
  dur = end_t - start_t ;

  /* now save "sample_buf" into "outfile" */

  File f = SPIFFS.open (outfile, "w") ;
  if (f)
  {
    char s[16] ; // a temporary buffer for converting int to strings

    strcpy (line, "{\r\n\"samples\":[\r\n") ;
    len = f.print (line) ;
    for (count=0 ; count < num_samples ; count++)
    {
      offset = count * (num_gpio + 1) ;
      sprintf (s, "%ld", sample_buf[offset]) ;
      sprintf (line, "[%s,", s) ;
      for (i=0 ; i < num_gpio ; i++)
      {
        offset = (count * (num_gpio + 1)) + 1 + i ;
        sprintf (s, "%d", sample_buf[offset]) ;
        strcat (line, s) ;
        if (i < num_gpio - 1)
          strcat (line, ",") ;
      }
      if (count < num_samples - 1)
        strcat (line, "],\r\n") ;
      else
        strcat (line, "]\r\n") ;
      len += f.print (line) ;
    }
    strcpy (line, "]\r\n}\r\n") ;
    len += f.print (line) ;
    f.close () ;
  }
  else
  {
    sprintf (G_reply_buf, "FAULT: Cannot write to '%s'.\r\n", outfile) ;
  }
  free (sample_buf) ;

  sprintf(G_reply_buf, "sbuf_len:%d gap_usec:%ld dur:%ld len:%d\r\n",
          sbuf_len, gap_usec, dur, len) ;
}

