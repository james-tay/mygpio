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

