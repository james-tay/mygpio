/*
   This is the callback function that is called whenever an MQTT message
   arrives at a topic that we have subscribed to. This is typically the
   command topic.
*/

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
  if (f)
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
  if (!f)
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
  if (!f)
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
            if (f)
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
          if (f)
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

