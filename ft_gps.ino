/*
   The role of this thread is to read messages from a hardware uart, parse
   them and extract data, publishing it as in the "p->results[]" array. The
   main loop only exits if "p->state" is set to THREAD_WRAPUP. Since our
   primary data is latitude/longitude/altitude, all results exposed by this
   thread are float.
*/

void ft_gpsmon (S_thread_entry *p)
{
  #define READ_BUF 32   // max bytes we read from the uart at a time
  #define GPS_BUF 128   // max length of a single GPS message
  #define GPS_FIELDS 32 // max comma separated fields in a GPS message

  if (p->num_args != 3)
  {
    strcpy (p->msg, "FATAL! Expecting 3x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int baud = atoi (p->in_args[0]) ;
  int rx_pin = atoi (p->in_args[1]) ;
  int tx_pin = atoi (p->in_args[2]) ;

  /* configure metrics that we will expose */

  p->results[0].num_tags = 1 ;
  p->results[0].meta[0] = "msg" ;
  p->results[0].data[0] = "\"cksumOk\"" ;

  p->results[1].num_tags = 1 ;
  p->results[1].meta[0] = "msg" ;
  p->results[1].data[0] = "\"cksumBad\"" ;

  p->results[2].num_tags = 1 ;
  p->results[2].meta[0] = "msg" ;
  p->results[2].data[0] = "\"overrun\"" ;

  p->results[3].num_tags = 1 ;
  p->results[3].meta[0] = "position" ;
  p->results[3].data[0] = "\"latitude\"" ;

  p->results[4].num_tags = 1 ;
  p->results[4].meta[0] = "position" ;
  p->results[4].data[0] = "\"longitude\"" ;

  p->results[5].num_tags = 1 ;
  p->results[5].meta[0] = "time" ;
  p->results[5].data[0] = "\"utc\"" ;

  p->results[6].num_tags = 1 ;
  p->results[6].meta[0] = "satellites" ;
  p->results[6].data[0] = "\"used\"" ;

  p->results[7].num_tags = 1 ;
  p->results[7].meta[0] = "elevation" ;
  p->results[7].data[0] = "\"sealevel\"" ;

  p->results[8].num_tags = 1 ;
  p->results[8].meta[0] = "elevation" ;
  p->results[8].data[0] = "\"geoid\"" ;

  p->results[9].num_tags = 1 ;
  p->results[9].meta[0] = "nav" ;
  p->results[9].data[0] = "\"mode\"" ; // 1=none, 2=2D fix, 3=3D fix

  p->results[10].num_tags = 1 ;
  p->results[10].meta[0] = "dilution" ;
  p->results[10].data[0] = "\"position\"" ;

  p->results[11].num_tags = 1 ;
  p->results[11].meta[0] = "dilution" ;
  p->results[11].data[0] = "\"horizontal\"" ;

  p->results[12].num_tags = 1 ;
  p->results[12].meta[0] = "dilution" ;
  p->results[12].data[0] = "\"vertical\"" ;

  p->results[13].num_tags = 1 ;
  p->results[13].meta[0] = "speed" ;
  p->results[13].data[0] = "\"kmh\"" ;

  p->num_float_results = 14 ;

  /* if we're the first thread to access the hardware UART, initialize it */

  xSemaphoreTake (G_hw_uart->lock, portMAX_DELAY) ;
  G_hw_uart->in_use++ ;
  if (G_hw_uart->initialized == 0)
  {
    Serial2.begin (baud, SERIAL_8N1, rx_pin, tx_pin) ;
    G_hw_uart->initialized = 1 ;
  }
  xSemaphoreGive (G_hw_uart->lock) ;
  strcpy (p->msg, "ok") ;

  int idx, amt ;
  int msg_len = 0 ;                     // length of current msg in "gps_buf"
  char read_buf[READ_BUF] ;             // buffer to read from uart
  char gps_buf[GPS_BUF] ;               // buffer to hold a complete GPS msg
  char *msg_tokens[GPS_FIELDS] ;        // pointers pointing into "gps_buf"

  while (p->state == THREAD_RUNNING)
  {
    /*
       keep reading bytes from UART until we have a complete message from
       the GPS beginning with '$' and ending with CR/NL.
    */

    amt = Serial2.read (read_buf, READ_BUF) ;
    if (amt > 0)
    {
      /*
         if "msg_len" == 0, that means we're hunting for the start of a msg.
         if "msg_len" > 0, that keep copying bytes until we see the msg end.
      */

      for (idx=0 ; idx < amt ; idx++)
      {
        if (read_buf[idx] == '$')       // found start of message
        {
          gps_buf[0] = read_buf[idx] ;
          msg_len = 1 ;
        }
        else
        if (read_buf[idx] == '\r')      // found end of message
        {
          gps_buf[msg_len] = 0 ;
          if (msg_len > 8)              // minimum msg_len for a sane msg
          {
            char checksum = 0 ;
            for (int i=1 ; i < msg_len ; i++)
            {
              if (gps_buf[i] == '*')
                break ;
              else
                checksum = checksum ^ gps_buf[i] ;
            }
            char verify[4] ; // just enough to store "*XX"
            sprintf (verify, "*%X", checksum) ;
            if (strcmp(gps_buf+msg_len-3, verify) == 0)         // checksum OK
            {
              p->results[0].f_value = p->results[0].f_value + 1 ;
              msg_len = msg_len - 3 ;
              gps_buf[msg_len] = 0 ; // remove checksum, don't need it anymore


              /*
                 tokenize "gps_buf" to "num_tokens" of "msg_tokens" pointers.
                 We can't use strtok() here because some fields may be zero
                 length and strtok() doesn't handle it correctly. This code
                 ensures zero length tokens point to a 0-length string.
              */

              int num_tokens = 0 ;
              int tok_start = 1 ; // move past the '$' character

              for (int tok_end=1 ; tok_end < msg_len ; tok_end++)
                if ((gps_buf[tok_end] == ',') || (tok_end == msg_len - 1))
                {
                  if (tok_end < msg_len - 1)
                    gps_buf[tok_end] = 0 ;
                  if (tok_end - tok_start > 0)
                    msg_tokens[num_tokens] = gps_buf + tok_start ;
                  else
                    msg_tokens[num_tokens] = "" ; // provide an 0-length string
                  num_tokens++ ;
                  tok_start = tok_end + 1 ;
                }

              if (G_debug > 1)
              {
                char line[BUF_SIZE] ;
                Serial.println ("DEBUG: ft_gpsmon() msg_tokens[]") ;
                for (int i=0 ; i < num_tokens ; i++)
                {
                  sprintf (line, "  %d:(%s)", i, msg_tokens[i]) ;
                  Serial.println (line) ;
                }
              }

              /* extract information from various GPS messages */

              if ((strcmp (msg_tokens[0], "GNRMC") == 0) && (num_tokens == 13))
              {
                double cur_deg, cur_mins ;
                sscanf (msg_tokens[3], "%2lf%lf", &cur_deg, &cur_mins) ;
                double latitude = cur_deg + (cur_mins / 60.0) ;
                if (msg_tokens[4][0] == 'S')
                  latitude = latitude * -1 ;
                sscanf (msg_tokens[5], "%3lf%lf", &cur_deg, &cur_mins) ;
                double longitude = cur_deg + (cur_mins / 60.0) ;
                if (msg_tokens[6][0] == 'W')
                  longitude = longitude * -1 ;

                p->results[3].f_value = latitude ;
                p->results[4].f_value = longitude ;

                /* parse the time and date */

                int hour, minute, seconds, day, month, year ;
                struct tm utc ;

                sscanf (msg_tokens[1], "%2d%2d%2d", &hour, &minute, &seconds) ;
                sscanf (msg_tokens[9], "%2d%2d%2d", &day, &month, &year) ;
                memset (&utc, 0, sizeof(utc)) ;
                utc.tm_sec = seconds ;
                utc.tm_min = minute ;
                utc.tm_hour = hour ;
                utc.tm_mday = day ;
                utc.tm_mon = month - 1 ; /* this needs to be 0 to 11 */
                utc.tm_year = year + 100 ; /* years since 1900 */
                time_t now = mktime (&utc) ;

                p->results[5].f_value = now ;
              }

              if (strcmp (msg_tokens[0], "GNGGA") == 0)
              {
                double satellites = atof (msg_tokens[7]) ; // satellites used
                double altitude = atof (msg_tokens[9]) ; // "m" above sea level
                double geoid = atof (msg_tokens[11]) ;  // geoid and sea level
                p->results[6].f_value = satellites ;
                p->results[7].f_value = altitude ;
                p->results[8].f_value = geoid ;
              }

              if ((strcmp (msg_tokens[0], "GNGSA") == 0) && (num_tokens == 18))
              {
                double mode = atof (msg_tokens[2]) ;
                double dilution_pos = atof(msg_tokens[15]) ;
                double dilution_hori = atof(msg_tokens[16]) ;
                double dilution_vert = atof(msg_tokens[17]) ;
                p->results[9].f_value = mode ;
                p->results[10].f_value = dilution_pos ;
                p->results[11].f_value = dilution_hori ;
                p->results[12].f_value = dilution_vert ;
              }

              if ((strcmp (msg_tokens[0], "GNVTG") == 0) && (num_tokens == 10))
              {
                double speed = atof (msg_tokens[7]) ;
                p->results[13].f_value = speed ;
              }
            }
            else                                                // checksum BAD
            {
              p->results[1].f_value = p->results[1].f_value + 1 ;
              if (G_debug)
              {
                char line[GPS_BUF+BUF_SIZE] ;
                snprintf (line, GPS_BUF+BUF_SIZE,
                          "DEBUG: ft_gpsmon() gps_buf<%s>[%X] Bad checksum",
                          gps_buf, checksum) ;
                Serial.println (line) ;
              }
            }
          }
          msg_len = 0 ;                 // "reset" gps_buf.
        }
        else
        if (msg_len > 0)                // in the middle of a message
        {
          gps_buf[msg_len] = read_buf[idx] ;
          msg_len++ ;
          if (msg_len == GPS_BUF)       // buffer overrun :(
          {
            msg_len = 0 ;
            p->results[2].f_value = p->results[2].f_value + 1 ;
          }
        }
      }
    }
  }

  xSemaphoreTake (G_hw_uart->lock, portMAX_DELAY) ;
  G_hw_uart->in_use-- ;
  xSemaphoreGive (G_hw_uart->lock) ;
}

/*
   This thread depends on metrics from ft_gpsmon(). Our job is to determine
   if we've moved and log our new location to a ring buffer. When the ring
   buffer fills up, flush it to a file. We must have moved a minimum distance
   in order to log an entry. If our minimum movement threahold is 2m, then

     arctan(2/6371001) = .0000003139 radians
                       = .0000003139 * 180 / PI
                       = .0000179864 degrees

   In actual fact, we add our elevation to the average earth radius for the
   best accuracy. Thus, we compute

     GPS elevation - GPS geoid + Ave Earth Radius

   This thread uses the following configuration determine that the GPS has
   achieved a reliable lock (pay attention to the various data types).

     cfg_gpsMode     - The GPS mode number which indicates 3D fix (def: 3)
     cfg_minSatsUsed - Minimum satellites used (def: 4)
     cfg_maxPosDil   - Maximum allowed position dilution (def: 3.5)

   Position data is written into a ring buffer which is flushed to a file as
   a CSV, with the first line containing the heading, followed by one or more
   data rows. The following configuration determines the data logging behavior.

     cfg_gpsThread     - the name of the ft_gpsmon() thread (MANDATORY)
     cfg_minDistMeters - minimum meters moved to trigger logging (def: 12.0)
     cfg_normLogSecs   - normal interval between log entries (def: 10)
     cfg_maxLogSecs    - max log interval when stationary (def: 60)
     cfg_fileName      - the CSV file we write to (def: "/gpslog.csv")
     cfg_fileMaxSize   - rotated if file exceeds this (def: 262144 bytes)

   Each entry written to "cfg_fileName" looks like,

     1638745487,43.723317,-79.409851,175.500000,2.480000

   Thus, at 52 bytes per line, a 256k file contains,

     262144 / 52 = 5041 entries
                 = 50410 seconds of entries (assuming cfg_normLogSecs = 10)
                 = 14 hours of position data (or more)

   Since there are many tunable parameters, this thread takes 1 argument on
   invocation, the path to its config file. Each line of the config file is
   expected to be in the format,

     <key> <value>

   Additional sensor data can optionally be captured together with each GPS
   position entry. This is done by searching for <threadName> and then
   acquiring the value of <resultIdx>. As an example, a light sensor accessed
   by an ft_aread() thread is called "light1". This thread exposes 1x metric.
   At the same time, we have an ft_hcsr04() thread called "range1" and we want
   to capture its max distance metric. We specify,

     cfg_extraMetrics light1:0,range1:2

   Thus, we can acquire additional sensor metrics by specifying,

     cfg_extraMetrics <threadName>:<resultIdx>[,<threadNameN>:<resultIdxN>...]

   In order for each ring buffer element to accomodate extra metrics, the
   struct{} features a Flexible Array Member (FAM), since this can only be
   determined after parsing cfg_extraMetrics. Recall that thread results may
   be either int or double, thus each FAM element is declared as a union.

   Column names of extra metrics are generated in the format :

     <threadName>_<resultIdx>

   Thus in our above example, the column names will be written as,

     light1_0,range1_2
*/

void ft_gpslog (S_thread_entry *p)
{
  static thread_local int cfg_gpsMode = 3 ;
  static thread_local int cfg_minSatsUsed = 4 ;
  static thread_local double cfg_maxPosDil = 3.5 ;

  static thread_local char cfg_gpsThread[MAX_THREAD_NAME] ; // initialize later
  static thread_local double cfg_minDistMeters = 12.0 ;
  static thread_local int cfg_normLogSecs = 10 ;
  static thread_local int cfg_maxLogSecs = 60 ;
  static thread_local int cfg_fileMaxSize = 262144 ;
  static thread_local char cfg_fileName[BUF_SIZE] ;
  static thread_local char cfg_extraMetrics[BUF_SIZE] ;

  static thread_local double cur_ele = 0.0 ;
  static thread_local double cur_lat = 0.0 ;
  static thread_local double cur_long = 0.0 ;
  static thread_local double cur_epoch = 0.0 ;

  static thread_local int ring_pos = 0 ;     // next empty entry
  static thread_local int ring_entries = 0 ; // dynamically calculated
  static thread_local long last_run = 0 ;    // time this function last ran
  static thread_local long last_update = 0 ; // time of last ring buffer update

  /* variables for tracking (optional) extra metrics */

  #define MAX_EXTRA_METRICS 8
  static thread_local int rt_xm_total = 0 ; // number of extra metrics
  static thread_local int rt_xm_resultidx[MAX_EXTRA_METRICS] ;
  static thread_local char *rt_xm_threadname[MAX_EXTRA_METRICS] ;

  /* extra metrics, only used if user specifies "cfg_extraMetrics") */

  union extra_metrics_u
  {
    int i_value ;
    double f_value ;
  } ;

  /* our ringer buffer */

  struct ring_entry_t
  {
    time_t epoch ;
    double latitude ;
    double longitude ;
    double elevation ;
    double dilution ;
    extra_metrics_u xmetrics[] ; // must be the last element
  } ;
  typedef struct ring_entry_t S_RingEntry ;
  static thread_local S_RingEntry *ring_buffer = NULL ; // the ring buffer

  #define MAX_RING_BUFFER_ELEMENTS 200 // protect against crazy big malloc()
  static thread_local size_t rt_rbuf_element_sz = 0 ; // including xmetrics[]

  /* a macro to check if we're told to terminate, make sure we free(). */

  #define GPSLOG_CHECK_CLEANUP ({                 \
      if (p->state != THREAD_RUNNING)             \
      {                                           \
        if (ring_buffer != NULL)                  \
        {                                         \
          free (ring_buffer) ;                    \
          ring_buffer = NULL ;                    \
        }                                         \
        strcpy (p->msg, "ring_buffer released") ; \
        return ; \
      } \
    })

  long now = millis () ; // note down this function's wall clock time

  /*
     before we return() for whatever reason, check if we're being shutdown,
     and if "ring_buffer" is malloc()'ed, better free() it now.
  */

  GPSLOG_CHECK_CLEANUP ;

  if (p->num_args != 1)
  {
    strcpy (p->msg, "FATAL! Expecting 1x argument") ;
    p->state = THREAD_STOPPED ; return ;
  }

  if (p->loops == 0)
  {
    /* configure results (ie, internal metrics) we want to expose */

    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = "ringbuffer" ;
    p->results[0].data[0] = "\"size\"" ;

    p->results[1].num_tags = 1 ;
    p->results[1].meta[0] = "entries" ;
    p->results[1].data[0] = "\"written\"" ;

    p->results[2].num_tags = 1 ;
    p->results[2].meta[0] = "entries" ;
    p->results[2].data[0] = "\"moving\"" ;

    p->results[3].num_tags = 1 ;
    p->results[3].meta[0] = "entries" ;
    p->results[3].data[0] = "\"stationary\"" ;

    p->results[4].num_tags = 1 ;
    p->results[4].meta[0] = "files" ;
    p->results[4].data[0] = "\"rotated\"" ;

    p->num_int_results = 5 ;

    /* parse our config file, load configuration into thread_local vars */

    strcpy (cfg_gpsThread, "") ;
    strcpy (cfg_extraMetrics, "") ;
    strcpy (cfg_fileName, "/gpslog.csv") ;

    File f = SPIFFS.open (p->in_args[0], "r") ;
    if (f)
    {
      char line[BUF_SIZE], buf[BUF_SIZE] ;
      while (1)
      {
        int amt = f.readBytesUntil ('\n', line, BUF_SIZE-1) ;
        if (amt > 0)
        {
          line[amt] = 0 ;
          char *idx=NULL, *key=NULL, *value=NULL ;

          key = strtok_r (line, " ", &idx) ;
          if (key != NULL)
          {
            value = strtok_r (NULL, " ", &idx) ;
            if (value != NULL)
            {
              if (G_debug)
              {
                snprintf (buf, BUF_SIZE, "DEBUG: ft_gpslog() (%s)(%s)",
                          key, value) ;
                Serial.println (buf) ;
              }
              if (strcmp (key, "cfg_gpsMode") == 0)
                cfg_gpsMode = atoi (value) ;
              if (strcmp (key, "cfg_minSatsUsed") == 0)
                cfg_minSatsUsed = atoi (value) ;
              if (strcmp (key, "cfg_maxPosDil") == 0)
                cfg_maxPosDil = atof (value) ;

              if ((strcmp (key, "cfg_gpsThread") == 0) &&
                  (strlen (cfg_gpsThread) < MAX_THREAD_NAME))
                strncpy (cfg_gpsThread, value, MAX_THREAD_NAME) ;
              if (strcmp (key, "cfg_minDistMeters") == 0)
                cfg_minDistMeters = atof (value) ;
              if (strcmp (key, "cfg_normLogSecs") == 0)
                cfg_normLogSecs = atoi (value) ;
              if (strcmp (key, "cfg_maxLogSecs") == 0)
                cfg_maxLogSecs = atoi (value) ;
              if (strcmp (key, "cfg_fileMaxSize") == 0)
                cfg_fileMaxSize = atoi (value) ;
              if (strcmp (key, "cfg_fileName") == 0)
                strncpy (cfg_fileName, value, BUF_SIZE) ;
              if (strcmp (key, "cfg_extraMetrics") == 0)
                strncpy (cfg_extraMetrics, value, BUF_SIZE) ;
            }
            else
            {
              strcpy (p->msg, "FATAL! Missing value in config") ;
              p->state = THREAD_STOPPED ; return ;
            }
          }
          else
          {
            strcpy (p->msg, "FATAL! Invalid line in config") ;
            p->state = THREAD_STOPPED ; return ;
          }
        }
        else
          break ;
      }
      f.close () ;

      /* do some sanity check on our configuration to spot obvious mistakes */

      char *fault = NULL ;
      if (cfg_minSatsUsed < 1)
        fault = "cfg_minSatsUsed cannot be less than 1" ;
      if (strlen(cfg_gpsThread) < 1)
        fault = "cfg_gpsThread cannot be empty" ;
      if (cfg_normLogSecs < 1)
        fault = "cfg_normLogSecs cannot be less than 1" ;
      if (cfg_maxLogSecs < cfg_normLogSecs)
        fault = "cfg_maxLogSecs cannot be less than cfg_normLogSecs" ;
      if (strlen(cfg_fileName) < 1)
        fault = "cfg_fileName cannot be empty" ;

      if (fault)
      {
        strcpy (p->msg, fault) ;
        p->state = THREAD_STOPPED ; return ;
      }
    }
    else
    {
      strcpy (p->msg, "FATAL! Cannot read configuration") ;
      p->state = THREAD_STOPPED ; return ;
    }

    /* if cfg_extraMetrics is defined, parse it now */

    if (strlen(cfg_extraMetrics) > 0)
    {
      char *idx=NULL, *key=NULL ;
      key = strtok_r (cfg_extraMetrics, ",", &idx) ; // key is "<thr>:<i>"
      while (key)
      {
        rt_xm_threadname[rt_xm_total] = key ;
        rt_xm_resultidx[rt_xm_total] = -1 ;
        for (int i=0 ; i < strlen(key) ; i++)
          if (key[i] == ':')
          {
            key[i] = 0 ;
            rt_xm_resultidx[rt_xm_total] = atoi(key+i+1) ;
          }

        /* sanity check thread name and result index */

        if ((strlen(rt_xm_threadname[rt_xm_total]) < 1) ||
            (strlen(rt_xm_threadname[rt_xm_total]) > MAX_THREAD_NAME))
        {
          strcpy (p->msg, "FATAL! Invalid thread name") ;
          p->state = THREAD_STOPPED ; return ;
        }
        if ((rt_xm_resultidx[rt_xm_total] < 0) ||
            (rt_xm_resultidx[rt_xm_total] >= MAX_THREAD_RESULT_VALUES))
        {
          strcpy (p->msg, "FATAL! Invalid result index") ;
          p->state = THREAD_STOPPED ; return ;
        }

        rt_xm_total++ ;
        key = strtok_r (NULL, ",", &idx) ; // move on to next token
      }
    }

    /* calculate the size of our ring buffer and allocate memory */

    ring_entries = (cfg_maxLogSecs / cfg_normLogSecs) + 1 ;
    if (ring_entries > MAX_RING_BUFFER_ELEMENTS)
    {
      snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
                "FATAL! ring_buffer sz %d exceeds %d",
                 ring_entries, MAX_RING_BUFFER_ELEMENTS) ;
      p->state = THREAD_STOPPED ; return ;
    }
    rt_rbuf_element_sz = sizeof (S_RingEntry) +
                         (rt_xm_total * sizeof(extra_metrics_u)) ;
    size_t sz = ring_entries * rt_rbuf_element_sz ;
    ring_buffer = (S_RingEntry*) malloc (sz) ;
    if (ring_buffer == NULL)
    {
      snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
                "FATAL! ring_buffer malloc(%d) failed", sz) ;
      p->state = THREAD_STOPPED ; return ;
    }

    p->results[0].i_value = sz ;
    last_run = now ;
  }

  /*
     attempt to identify the "cfg_gpsThread", make sure it's running. If not
     found, sleep and try again (but check if we're told to terminate).
  */

  int gps_tid ;
  for (gps_tid=0 ; gps_tid < MAX_THREADS ; gps_tid++)
    if ((G_thread_entry[gps_tid].state == THREAD_RUNNING) &&
        (G_thread_entry[gps_tid].ft_addr == ft_gpsmon) &&
        (strcmp(G_thread_entry[gps_tid].name, cfg_gpsThread) == 0))
      break ;
  if (gps_tid == MAX_THREADS) /* can't locate "cfg_gpsThread", retry */
  {
    strcpy (p->msg, "WARNING: Cannot find cfg_gpsThread") ;
    last_run = last_run + (cfg_normLogSecs * 1000) ;
    while (millis() < last_run)
    {
      delay (THREAD_SHUTDOWN_PERIOD / 20) ;
      GPSLOG_CHECK_CLEANUP ;
    }
    return ;
  }

  /*
     check if GPS has a good lock. If not, sleep and try again (but check if
     we're told to terminate).
  */

  #define IDX_GPS_MODE 9
  #define IDX_SATS_USED 6
  #define IDX_POS_DIL 10

  S_thread_result *r_ptr = G_thread_entry[gps_tid].results ;
  if ((cfg_gpsMode != (int) r_ptr[IDX_GPS_MODE].f_value) ||
      (cfg_minSatsUsed > (int) r_ptr[IDX_SATS_USED].f_value) ||
      (cfg_maxPosDil < r_ptr[IDX_POS_DIL].f_value))
  {
    snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
              "WARNING: No lock, mode:%d sats:%d dil:%s",
              (int) r_ptr[IDX_GPS_MODE].f_value,
              (int) r_ptr[IDX_SATS_USED].f_value,
              String (r_ptr[IDX_POS_DIL].f_value, FLOAT_DECIMAL_PLACES)) ;

    last_run = last_run + (cfg_normLogSecs * 1000) ;
    while (millis() < last_run)
    {
      delay (THREAD_SHUTDOWN_PERIOD / 20) ;
      GPSLOG_CHECK_CLEANUP ;
    }
    return ;
  }

  /*
     if this is the first time initializing our position, don't proceed.
     Just sleep and run again (but check if we're told to terminate).
  */

  #define IDX_ELE 7
  #define IDX_UTC 5
  #define IDX_LAT 3
  #define IDX_LONG 4

  if ((cur_ele == 0.0) && (cur_lat == 0.0) && (cur_long == 0.0))
  {
    strcpy (p->msg, "INFO: initialized") ;
    cur_ele = r_ptr[IDX_ELE].f_value ;
    cur_lat = r_ptr[IDX_LAT].f_value ;
    cur_long = r_ptr[IDX_LONG].f_value ;
    cur_epoch = r_ptr[IDX_UTC].f_value ;
    last_update = now ;

    last_run = last_run + (cfg_normLogSecs * 1000) ;
    while (millis() < last_run)
    {
      delay (THREAD_SHUTDOWN_PERIOD / 20) ;
      GPSLOG_CHECK_CLEANUP ;
    }
    return ;
  }

  /*
     if we've moved, or if we've not added any entry in a long time, add our
     current position to our ring buffer.
  */

  #define AVE_EARTH_RADIUS 6371008.8 // meters

  double earthRadius = AVE_EARTH_RADIUS + r_ptr[IDX_ELE].f_value ;
  double minDegrees = atan (cfg_minDistMeters / earthRadius) * 180.0 / M_PI ;

  int we_moved = 0 ;
  if ((abs(r_ptr[IDX_LAT].f_value - cur_lat) > minDegrees) ||
      (abs(r_ptr[IDX_LONG].f_value - cur_long) > minDegrees))
  {
    we_moved = 1 ;
    p->results[2].i_value++ ;
  }

  if (((we_moved) || (now - last_update > cfg_maxLogSecs * 1000)) &&
      (ring_pos < ring_entries)) // make sure there's space available
  {
    ring_buffer[ring_pos].epoch = int(r_ptr[IDX_UTC].f_value) ;
    ring_buffer[ring_pos].latitude = r_ptr[IDX_LAT].f_value ;
    ring_buffer[ring_pos].longitude = r_ptr[IDX_LONG].f_value ;
    ring_buffer[ring_pos].elevation = r_ptr[IDX_ELE].f_value ;
    ring_buffer[ring_pos].dilution = r_ptr[IDX_POS_DIL].f_value ;
    ring_pos++ ;
    last_update = now ;

    if (we_moved == 0)
      p->results[3].i_value++ ;
  }

  /*
     if our ring buffer is full, or if we've not flushed it in a long time,
     write it to storage
  */

  if (ring_pos == ring_entries)
  {
    File f = SPIFFS.open (cfg_fileName, "a") ;
    if (!f)
    {
      snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
                "FATAL! Cannot open %s for writing", cfg_fileName) ;
      free (ring_buffer) ;
      ring_buffer = NULL ;
      p->state = THREAD_STOPPED ; return ;
    }
    if ((f.size() < 1) &&
        (f.print ("time,latitude,longitude,elevation,dilution\n") < 1))
    {
      snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
                "FATAL! Cannot write to %s", cfg_fileName) ;
      free (ring_buffer) ;
      ring_buffer = NULL ;
      f.close () ;
      p->state = THREAD_STOPPED ; return ;
    }

    char line[BUF_SIZE] ;
    for (int i=0 ; i < ring_pos ; i++)
    {
      String s_lat = String(ring_buffer[i].latitude, FLOAT_DECIMAL_PLACES) ;
      String s_long = String(ring_buffer[i].longitude, FLOAT_DECIMAL_PLACES) ;
      String s_elev = String(ring_buffer[i].elevation, FLOAT_DECIMAL_PLACES) ;
      String s_dilu = String(ring_buffer[i].dilution, FLOAT_DECIMAL_PLACES) ;
      snprintf (line, BUF_SIZE-1, "%ld,%s,%s,%s,%s\n",
                ring_buffer[i].epoch,
                s_lat.c_str(),
                s_long.c_str(),
                s_elev.c_str(),
                s_dilu.c_str()) ;
      f.print (line) ;
      p->results[1].i_value++ ;
    }
    ring_pos = 0 ; // reset ring buffer since it got flushed

    /* check if it's time to rotate cfg_fileName */

    int rotate = 0 ;
    if (f.size() > cfg_fileMaxSize)
      rotate = 1 ;
    f.close () ;

    if (rotate) // ... delete old file if present
    {
      char dst_name[BUF_SIZE] ;
      snprintf (dst_name, BUF_SIZE-1, "%s.old", cfg_fileName) ;
      File f = SPIFFS.open (dst_name) ;
      if (f)
      {
        f.close () ;
        SPIFFS.remove (dst_name) ;
      }
      SPIFFS.rename (cfg_fileName, dst_name) ;
      p->results[4].i_value++ ;
    }
  }

  /*
     figure out how long to sleep, but take short naps and check if we're
     being asked to shutdown.
  */

  long duration = now + (cfg_normLogSecs * 1000) - millis () ;
  if (duration > 0)
  {
    snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
              "ele_sz:%d %d ring:%d/%d sleep:%ldms",
              rt_rbuf_element_sz, ring_pos, ring_entries, duration) ;
    last_run = last_run + (cfg_normLogSecs * 1000) ;
    while (millis() < last_run)
    {
      delay (THREAD_SHUTDOWN_PERIOD / 20) ;
      GPSLOG_CHECK_CLEANUP ;
    }
  }
}

