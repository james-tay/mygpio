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

  char line[BUF_MEDIUM] ; // <<-- also used to identify our current stack addr
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

  /* if we have a camera, and it's initialized */

  if (G_cam_config != NULL)
  {
    sprintf (line,
             "ec_cam_faults %ld\n"
             "ec_cam_frames %ld\n"
             "ec_cam_last_frame_size %ld\n"
             "ec_cam_last_capture_dur_ms %ld\n"
             "ec_cam_last_capture_time_ms %ld\n"
             "ec_cam_last_xmit_time_ms %ld\n"
             "ec_cam_client_faults %ld\n"
             "ec_cam_client_closed %ld\n"
             "ec_cam_streamed %ld\n",
             G_Metrics->camFaults,
             G_Metrics->camFrames,
             G_Metrics->camLastFrameSize,
             G_Metrics->camLastCaptureDurMs,
             G_Metrics->camLastCaptureTimeMs,
             G_Metrics->camLastXmitDurMs,
             G_Metrics->camClientFaults,
             G_Metrics->camClientClosed,
             G_Metrics->camStreamed) ;
    strcat (G_reply_buf, line) ;
  }

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

  /*
     at this point, we have parsed enough to decide on what to do with it.
     Save our HTTP response code in "response_code", which typically should
     be 200. If we set this to -1, it means some function is handling the
     HTTP response itself.
  */

  int response_code = 200 ;
  G_reply_buf[0] = 0 ;
  G_Metrics->restCmds++ ;
  G_Metrics->restInBytes = G_Metrics->restInBytes +
                           strlen(client->uri) + strlen(client->query) ;

  if ((strcmp(client->method, "GET") == 0) &&
      (strcmp(client->uri, "/metrics") == 0))
  {
    f_handleWebMetrics () ;
  }
  else
  if ((strcmp(client->method, "GET") == 0) &&
      (strcmp(client->uri, "/v1") == 0) &&
      (strncmp(client->query, "cmd=", 4) == 0))
  {
    f_v1api (client->query + 4) ;
  }
  else
  if ((strcmp(client->method, "GET") == 0) &&
      (strcmp(client->uri, "/cam") == 0))
  {
    response_code = -1 ;        // we'll handle the HTTP response ourselves
    f_cam_img (client) ;
  }
  else
  if ((strcmp(client->method, "GET") == 0) &&
      (strcmp(client->uri, "/") == 0))
  {
    strcpy (G_reply_buf, "OK\r\n") ;
  }
  else
  {
    strcpy (G_reply_buf, "Invalid request\r\n") ;
    response_code = 404 ;
  }

  if (response_code > 0)
  {
    char line[BUF_SIZE] ;
    sprintf (line, "HTTP/1.1 %d OK\n", response_code) ;
    write (client->sd, line, strlen(line)) ;
    strcpy (line, "Content-Type: text/plain\n") ;
    write (client->sd, line, strlen(line)) ;
    strcpy (line, "Connection: close\n") ;
    write (client->sd, line, strlen(line)) ;

    /* Now send G_reply_buf */

    write (client->sd, "\n", 1) ;
    write (client->sd, G_reply_buf, strlen(G_reply_buf)) ;
  }

  if (client->hold_open == 0)
  {
    close (client->sd) ;
    client->sd = 0 ;
    client->req_pos = 0 ;
  }
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
    if ((G_WebClient[i].sd > 0) && (G_WebClient[i].hold_open == 0))
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
          if ((G_WebClient[idx].sd == 0) && (G_WebClient[idx].hold_open == 0))
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
      if ((G_WebClient[i].sd > 0) &&
          (G_WebClient[i].hold_open == 0) &&
          (FD_ISSET (G_WebClient[i].sd, &fds)))
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
        (G_WebClient[i].hold_open == 0) &&
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

