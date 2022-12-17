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

  WiFi.mode (WIFI_STA) ;
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

