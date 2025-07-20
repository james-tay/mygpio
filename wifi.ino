/*
   For some reason, the ESP32 does not always connect to the AP that is
   nearest (ie, strongest signal). This function performs a wifi scan and
   notes down the channel and BSSID of the nearest AP and then connects to
   it. On success, it returns 1, otherwise 0. It scan activity is written
   into "msg" as a 1-line string.
*/

void f_wifiConnect (char *ssid, char *pw, char *msg)
{
  int scan_chan=0, scan_rssi, scan_idx ;
  uint8_t scan_bssid[6] ;
  char scan_ssid[MAX_SSID_LEN+1] ;
  char line[BUF_SIZE] ;

  WiFi.mode (WIFI_STA) ;
  int n = WiFi.scanNetworks () ;
  for (int i=0 ; i < n ; i++)
  {
    WiFi.SSID(i).toCharArray(scan_ssid, MAX_SSID_LEN) ;
    if (strcmp(ssid, scan_ssid) == 0)
    {
      uint8_t *ptr = WiFi.BSSID(i) ;
      sprintf (line, "found(%x:%x:%x:%x:%x:%x ch:%d %ddBm) ",
               ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5],
               WiFi.channel(i), WiFi.RSSI(i)) ;
      strcat (msg, line) ;

      if (scan_chan == 0) /* this is our first matching AP, note it down */
      {
        scan_idx = i ;
        scan_rssi = WiFi.RSSI (i) ;
        scan_chan = WiFi.channel (i) ;
        memcpy (scan_bssid, WiFi.BSSID(i), 6) ;
      }
      else
      {
        if (WiFi.RSSI (i) > scan_rssi) /* check if this AP is nearer */
        {
          scan_idx = i ;
          scan_rssi = WiFi.RSSI (i) ;
          scan_chan = WiFi.channel (i) ;
          memcpy (scan_bssid, WiFi.BSSID(i), 6) ;
        }
      }
    }
  }

  if (scan_chan == 0) /* opsie, did not find requested AP */
  {
    strcpy (msg, "SSID not found") ;
    return ;
  }

  sprintf (line, "using(%x:%x:%x:%x:%x:%x ch:%d)",
           scan_bssid[0], scan_bssid[1], scan_bssid[2],
           scan_bssid[3], scan_bssid[4], scan_bssid[5], scan_chan) ;
  strcat (msg, line) ;
  WiFi.begin (ssid, pw, scan_chan, scan_bssid, true) ;
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
    strcat (G_reply_buf, "FAULT: Wifi connection timed out. Retrying.\r\n") ;

    /* if connection failed, use the normal "WiFi.begin()" to try connect */

    WiFi.begin (cfg_wifi_ssid, cfg_wifi_pw) ;
  }
  else
  {
    strcat (G_reply_buf, "FAULT: Invalid argument.\r\n") ;
  }
}

/*
   The following functions handle a ping request. Recall that this is blocking
   in that we cannot service other requests in the meantime. Thus we will send
   out ICMP echo requests in 1 second intervals, up to 5 of them, and we'll
   wait up to 1 second for replies. Now the way ping is implemented is that we
   need to setup a ping session which will execute callback functions as
   follows,

     - f_ping_success()
     - f_ping_timeout()
     - f_ping_end()

   Now we're able to send a single argument to each of our callbacks. This is
   our opportunity to pass in a data structure which captures the state of the
   ping session. This is the purpose of the struct "S_ping_data".

   The ping subsystem is made available for both user (ie, calling our REST)
   and internal systems. Thus we need the functions,

     - f_ping()         # this is called when the user calls our REST command
     - f_ping_run()     # this actually sets up the ping session

   References
     - https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/protocols/icmp_echo.html
*/

#define PING_ERROR_MSG_MAX_LEN 128

struct ping_data_s
{
  unsigned char running ;               // 1=running 0=stopped
  unsigned char num_ping_success ;      // number of ICMP echo responses
  unsigned char num_ping_timeout ;      // number of ping timeouts
  unsigned int elapsed_ms[MAX_PING_PKTS] ; // individual roundtrip responses
} ;
typedef struct ping_data_s S_ping_data ;

void f_cb_ping_success (esp_ping_handle_t handle, void *args)
{
  S_ping_data *ping_data = (S_ping_data*) args ;
  ping_data->num_ping_success++ ;

  unsigned int elapsed_ms ;
  unsigned short seq_num ;

  esp_ping_get_profile (handle, ESP_PING_PROF_TIMEGAP,
                        &elapsed_ms, sizeof(elapsed_ms)) ;
  esp_ping_get_profile (handle, ESP_PING_PROF_SEQNO,
                        &seq_num, sizeof(seq_num)) ;
  if (seq_num < MAX_PING_PKTS)
    ping_data->elapsed_ms[seq_num] = elapsed_ms ;
}

void f_cb_ping_timeout (esp_ping_handle_t handle, void *args)
{
  S_ping_data *ping_data = (S_ping_data*) args ;
  ping_data->num_ping_timeout++ ;
}

void f_cb_ping_end (esp_ping_handle_t handle, void *args)
{
  S_ping_data *ping_data = (S_ping_data*) args ;
  ping_data->running = 0 ;
}

void f_ping_run (int pkts, char *dest, char *error_msg,
                 struct ping_data_s *ping_data)
{
  /*
     resolve "dest" host into an IPv4 address "target_addr" we can use in
     "esp_ping_config_t".
  */

  ip_addr_t target_addr ;
  target_addr.u_addr.ip4.addr = ESP_IP4TOUINT32 (8,8,8,8) ;
  target_addr.type = ESP_IPADDR_TYPE_V4 ;






  /* now we configure the ping session */

  esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG () ;
  ping_config.target_addr = target_addr ;
  ping_config.count = pkts ;
  ping_config.interval_ms = 1000 ;              // default 1 sec interval
  ping_config.timeout_ms = 1000 ;               // default 1 sec timeout

  esp_ping_callbacks_t cbs ;                    // configure our callbacks
  cbs.on_ping_success = f_cb_ping_success ;
  cbs.on_ping_timeout = f_cb_ping_timeout ;
  cbs.on_ping_end = f_cb_ping_end ;
  cbs.cb_args = ping_data ;

  /*
     create the ping session, fire it off and enter a semi-busy wait until
     it's all done.
  */

  esp_ping_handle_t ping_session ;
  esp_err_t result = esp_ping_new_session (&ping_config, &cbs, &ping_session) ;
  if (result != ESP_OK)
  {
    snprintf (error_msg, PING_ERROR_MSG_MAX_LEN-1,
              "FAULT: Cannot start ping session: %s\r\n",
              esp_err_to_name (result)) ;
    return ;
  }

  unsigned long cutoff = millis() + ((MAX_PING_PKTS+2) * 1000) ;
  ping_data->running = 1 ;
  esp_ping_start (ping_session) ;

  while ((ping_data->running) && (millis() < cutoff)) // don't wait forever
    delay (100) ;

  esp_ping_stop (ping_session) ;
  esp_ping_delete_session (ping_session) ;
}

/*
   This function is called by the user, it's mainly a wrapper to f_ping_run().
*/

void f_ping (char **tokens)
{
  static char error_msg[PING_ERROR_MSG_MAX_LEN] ;
  error_msg[0] = 0 ;

  int count = atoi (tokens[1]) ;
  char *dest = tokens[2] ;

  if (count > MAX_PING_PKTS)
  {
    sprintf(G_reply_buf, "FAULT: maximum count is %d.\r\n", MAX_PING_PKTS) ;
    return ;
  }

  S_ping_data ping_data ;
  memset (&ping_data, 0, sizeof(S_ping_data)) ;

  f_ping_run (count, dest, error_msg, &ping_data) ;
  if (error_msg[0] != 0)
  {
    strcpy (G_reply_buf, error_msg) ;
    return ;                            // uh oh, something went wrong
  }

  sprintf(G_reply_buf, "success:%d timed_out:%d\r\n",
          ping_data.num_ping_success, ping_data.num_ping_timeout) ;
}

