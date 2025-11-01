/* =========================== */
/* thread management functions */
/* =========================== */

void *f_thread_lifecycle (void *p)
{
  S_thread_entry *entry = (S_thread_entry*) p ;

  /* initialize thread info, indicate thread is now running */

  xSemaphoreTake (entry->lock, portMAX_DELAY) ;
  entry->state = THREAD_RUNNING ;
  entry->core = xPortGetCoreID () ;
  xSemaphoreGive (entry->lock) ;

  while (entry->state == THREAD_RUNNING)
  {
    entry->ft_addr (entry) ;
    entry->loops++ ;
  }

  xSemaphoreTake (entry->lock, portMAX_DELAY) ;
  entry->state = THREAD_STOPPED ;
  xSemaphoreGive (entry->lock) ;

  while (1) // await death, freeRTOS doesn't like thread functions to return
    delay (1000) ;
}

void f_thread_create (char *name)
{
  int idx ;
  char line[BUF_SIZE] ;

  /* before we try creating threads, try reap dead ones (releases memory) */

  for (idx=0 ; idx < MAX_THREADS ; idx++)
    if (G_thread_entry[idx].state == THREAD_STOPPED)
    {
      xSemaphoreTake (G_thread_entry[idx].lock, portMAX_DELAY) ;
      G_thread_entry[idx].state = THREAD_READY;
      G_thread_entry[idx].name[0] = 0 ;
      xSemaphoreGive (G_thread_entry[idx].lock) ;
    }

  /* do validations first */

  if (strlen(name) > MAX_THREAD_NAME)
  {
    sprintf (line, "FAULT: thread name is too long, %d bytes max.\r\n",
             MAX_THREAD_NAME) ;
    strcat (G_reply_buf, line) ;
    return ;
  }

  for (idx=0 ; idx < MAX_THREADS ; idx++)
    if (strcmp (G_thread_entry[idx].name, name) == 0)
    {
      strcat (G_reply_buf, "FAULT: Duplicate thread name.\r\n") ;
      return ;
    }

  /*
      try read configuration from file, hold it in "config" for now. we'll
      copy it into G_thread_entry[].conf after all validation is complete.
   */

  char filename[BUF_SIZE], config[MAX_THREAD_CONF_BUF] ;
  sprintf (filename, "/thread-%s", name) ;
  File f = SPIFFS.open (filename, "r") ;
  int amt = f.readBytes (config, MAX_THREAD_CONF_BUF-1) ;
  f.close () ;
  if (amt < 1)
  {
    sprintf (line, "FAULT: Cannot read config file '%s'.\r\n", filename) ;
    strcat (G_reply_buf, line) ;
    return ;
  }
  config[amt] = 0 ;

  /* start by looking for a free G_thread_entry */

  for (idx=0 ; idx < MAX_THREADS ; idx++)
    if (G_thread_entry[idx].state == THREAD_READY)
      break ;
  if (idx == MAX_THREADS)
  {
    strcat (G_reply_buf, "Maximum number of threads reached.\r\n") ;
    return ;
  }

  /* found an available G_thread_entry, lock it, update and fire it up */

  xSemaphoreTake (G_thread_entry[idx].lock, portMAX_DELAY) ;

  /*
     try read optional tags config file, parse it in G_thread_entry[].tags,
     and have various pointers point into this buffer.
  */

  G_thread_entry[idx].tags_buf[0] = 0 ;
  G_thread_entry[idx].metric = NULL ;
  memset (G_thread_entry[idx].tags, 0, sizeof(char*) * MAX_THREAD_TAGS) ;

  sprintf (filename, "/tags-%s", name) ;
  f = SPIFFS.open (filename, "r") ;
  if (f)
  {
    int amt = f.readBytes (G_thread_entry[idx].tags_buf,
                           MAX_THREAD_TAGS_BUF-1) ;
    if (amt > 0)
    {
      G_thread_entry[idx].tags_buf[amt] = 0 ;
      G_thread_entry[idx].metric = strtok (G_thread_entry[idx].tags_buf, ",") ;
      int i ;
      for (i=0 ; i < MAX_THREAD_TAGS ; i++)
      {
        char *p = strtok (NULL, ",") ;
        if (p)
        {
          G_thread_entry[idx].tags[i] = p ;
          G_thread_entry[idx].tags[i+1] = NULL ;
        }
        else
          break ;
      }
    }
    f.close () ;
  }

  /* initialize data structure fields */

  strcpy (G_thread_entry[idx].name, name) ;
  strcpy (G_thread_entry[idx].conf, config) ;
  G_thread_entry[idx].num_args = 0 ;
  G_thread_entry[idx].num_int_results = 0 ;
  G_thread_entry[idx].num_float_results = 0 ;
  G_thread_entry[idx].loops = 0 ;
  G_thread_entry[idx].ts_started = millis () ;
  G_thread_entry[idx].msg[0] = 0 ;
  G_thread_entry[idx].ft_addr = NULL ;
  G_thread_entry[idx].buf = NULL ;
  memset (&G_thread_entry[idx].in_args, 0, MAX_THREAD_ARGS * sizeof(char*)) ;
  memset (&G_thread_entry[idx].results, 0,
          MAX_THREAD_RESULT_VALUES * sizeof(S_thread_result)) ;

  /* tokenize thread's configuration */

  int num_args ;
  char *ft_taskname = strtok (G_thread_entry[idx].conf, ",") ;
  for (num_args=0 ; num_args < MAX_THREAD_ARGS ; num_args++)
  {
    G_thread_entry[idx].in_args[num_args] = strtok (NULL, ",") ;
    if (G_thread_entry[idx].in_args[num_args] == NULL)
      break ;
  }
  G_thread_entry[idx].num_args = num_args ;

  /* figure out the ft_<task> function address */

  if (strcmp (ft_taskname, "ft_counter") == 0)
    G_thread_entry[idx].ft_addr = ft_counter ;
  if (strcmp (ft_taskname, "ft_aread") == 0)
    G_thread_entry[idx].ft_addr = ft_aread ;
  if (strcmp (ft_taskname, "ft_fast_aread") == 0)
    G_thread_entry[idx].ft_addr = ft_fast_aread ;
  if (strcmp (ft_taskname, "ft_adxl335") == 0)
    G_thread_entry[idx].ft_addr = ft_adxl335 ;
  if (strcmp (ft_taskname, "ft_dht22") == 0)
    G_thread_entry[idx].ft_addr = ft_dht22 ;
  if (strcmp (ft_taskname, "ft_ds18b20") == 0)
    G_thread_entry[idx].ft_addr = ft_ds18b20 ;
  if (strcmp (ft_taskname, "ft_dread") == 0)
    G_thread_entry[idx].ft_addr = ft_dread ;
  if (strcmp (ft_taskname, "ft_gpsmon") == 0)
    G_thread_entry[idx].ft_addr = ft_gpsmon ;
  if (strcmp (ft_taskname, "ft_gpslog") == 0)
    G_thread_entry[idx].ft_addr = ft_gpslog ;
  if (strcmp (ft_taskname, "ft_hcsr04") == 0)
    G_thread_entry[idx].ft_addr = ft_hcsr04 ;
  if (strcmp (ft_taskname, "ft_tread") == 0)
    G_thread_entry[idx].ft_addr = ft_tread ;
  if (strcmp (ft_taskname, "ft_ntpclient") == 0)
    G_thread_entry[idx].ft_addr = ft_ntpclient ;
  if (strcmp (ft_taskname, "ft_relay") == 0)
    G_thread_entry[idx].ft_addr = ft_relay ;
  if (strcmp (ft_taskname, "ft_i2sin") == 0)
    G_thread_entry[idx].ft_addr = ft_i2sin ;
  if (strcmp (ft_taskname, "ft_i2sout") == 0)
    G_thread_entry[idx].ft_addr = ft_i2sout ;
  if (strcmp (ft_taskname, "ft_serial2tcp") == 0)
    G_thread_entry[idx].ft_addr = ft_serial2tcp ;
  if (strcmp (ft_taskname, "ft_tasks") == 0)
    G_thread_entry[idx].ft_addr = ft_tasks ;
  if (strcmp (ft_taskname, "ft_watchdog") == 0)
    G_thread_entry[idx].ft_addr = ft_watchdog ;

  if (G_thread_entry[idx].ft_addr == NULL)
  {
    G_thread_entry[idx].name[0] = 0 ;
    sprintf (line, "FAULT: no such ft_task '%s'.\r\n", ft_taskname) ;
    strcat (G_reply_buf, line) ;
    xSemaphoreGive (G_thread_entry[idx].lock) ;
    return ;
  }

  /* (almost) everything prep'ed ... finally create the thread */

  G_thread_entry[idx].state = THREAD_STARTING ;
  xTaskCreate ((TaskFunction_t) f_thread_lifecycle,
               G_thread_entry[idx].name,
               MAX_THREAD_STACK,
               &G_thread_entry[idx],
               tskIDLE_PRIORITY,
               &G_thread_entry[idx].tid) ;

  xSemaphoreGive (G_thread_entry[idx].lock) ;

  sprintf (line, "thread '%s' created in_args:%d\r\n",
           name, num_args) ;
  strcat (G_reply_buf, line) ;
}

void f_thread_stop (char *name)
{
  int idx ;
  char line[BUF_SIZE] ;

  /* let's see if this thread exists */

  for (idx=0 ; idx < MAX_THREADS ; idx++)
    if ((G_thread_entry[idx].state == THREAD_RUNNING) &&
        (strcmp(G_thread_entry[idx].name, name) == 0))
    {
      G_thread_entry[idx].state = THREAD_WRAPUP ; // give it a chance to finish
      delay (THREAD_SHUTDOWN_PERIOD) ;
      vTaskDelete (G_thread_entry[idx].tid) ;

      /* if this thread allocated memory for itself, free it now */

      if (G_thread_entry[idx].buf != NULL)
      {
        free (G_thread_entry[idx].buf) ;
        G_thread_entry[idx].buf = NULL ;
      }

      G_thread_entry[idx].state = THREAD_STOPPED ;
      sprintf (line, "tid:%d '%s' stopped.\r\n",
               G_thread_entry[idx].tid, name) ;
      strcat (G_reply_buf, line) ;
      return ;
    }

  sprintf (line, "Thread '%s' not currently running.\r\n", name) ;
  strcat (G_reply_buf, line) ;
}

