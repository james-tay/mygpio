/*
   This function implements our main "UI". Our job is to parse the command
   and take action.
*/

void f_action (char **tokens)
{
  char line[BUF_SIZE] ;

  if ((strcmp(tokens[0], "?") == 0) || (strcmp(tokens[0], "help") == 0))
  {
    snprintf (G_reply_buf, REPLY_SIZE,
      "[GPIO]\r\n"
      "hi <GPIO pin> [pulse (usecs)]\r\n"
      "lo <GPIO pin>\r\n"
      "aread <GPIO pin>  - analog read\r\n"
      "dread <GPIO pin>  - digital read\r\n"
      "tread <GPIO pin>  - capacitive touch read\r\n"
      "bmp180            - barometric pressure (I2C)\r\n"
      "dht22 <dataPin>   - DHT-22 temperature/humidity sensor\r\n"
      "ds18b20 <dataPin> - DS18B20 temperature sensor\r\n"
      "hcsr04 <trigPin> <echoPin> - HC-SR04 ultrasonic ranger\r\n"
      "adxl335 <Xpin> <Ypin> <Zpin> <Time(ms)> <Interval(ms)>\r\n"
      "relay <thread name> on|off\r\n"
      "sampler <num> <gap_ms> <outfile> <gpioN> [gpioN ...]\r\n"
      "tone <GPIO pin> <freq> <dur(ms)>\r\n"
      "ping <count 1-%d> <dest_ip>\r\n"
      "pwm_on <GPIO pin> <freq> <duty> <res[1-20]>\r\n"
      "pwm_duty <GPIO pin> <duty>\r\n"
      "pwm_off <GPIO pin>\r\n"
      "\r\n"
      "lcd init          - LCD on I2C\r\n"
      "lcd backlight <on/off>\r\n"
      "lcd clear\r\n"
      "lcd print <row> <col> <message...>\r\n"
      "\r\n"
      "[System]\r\n"
      "cam init [xclk Mhz (6-24, def 20)]\r\n"
      "cam help\r\n"
      "cam show\r\n"
      "cam set <param> <value>\r\n"
      "cam reg <addr> <mask> <value>\r\n"
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
      "/tags-<t>     <metric>[,<label>=\"<value>\",...]\r\n"
      "/thread-<t>   <function>,...\r\n"
      "/wifi.rssi    <dBm>\r\n"
      "/wifi.ssid    <ssid>\r\n"
      "/wifi.pw      <pw>\r\n",

      MAX_PING_PKTS) ;
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

    uint32_t cpu_freq = rtc_clk_apb_freq_get() ;

    /* now report all our findings */

    sprintf (line, "Running on cpu:%d\r\n", xPortGetCoreID()) ;
    strcat (G_reply_buf, line) ;
    sprintf (line, "CPU speed: %u hz\r\n", cpu_freq) ;
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
  if ((strcmp(tokens[0], "cam") == 0) && (tokens[1] != NULL))
  {
    f_cam_cmd (tokens) ;
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
  if (strcmp(tokens[0], "sampler") == 0)
  {
    f_sampler (tokens) ;
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
  if ((strcmp(tokens[0], "ping") == 0) && (tokens[1] != NULL) &&
      (tokens[2] != NULL))
  {
    f_ping (tokens) ;
  }
  else
  if ((strcmp(tokens[0], "pwm_on") == 0) && (tokens[1] != NULL) &&
      (tokens[2] != NULL) && (tokens[3] != NULL) && (tokens[4] != NULL))
  {
    f_pwm_on (tokens) ;
  }
  else
  if ((strcmp(tokens[0], "pwm_duty") == 0) && (tokens[1] != NULL) &&
      (tokens[2] != NULL))
  {
    f_pwm_duty (tokens) ;
  }
  else
  if ((strcmp(tokens[0], "pwm_off") == 0) && (tokens[1] != NULL))
  {
    f_pwm_off (tokens) ;
  }
  else
  {
    strcat (G_reply_buf, "FAULT: Enter 'help' for commands.\r\n") ;
  }
}

