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

