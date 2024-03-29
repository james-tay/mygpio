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
  p->results[0].meta[0] = (char*) "msg" ;
  p->results[0].data[0] = (char*) "\"cksumOk\"" ;

  p->results[1].num_tags = 1 ;
  p->results[1].meta[0] = (char*) "msg" ;
  p->results[1].data[0] = (char*) "\"cksumBad\"" ;

  p->results[2].num_tags = 1 ;
  p->results[2].meta[0] = (char*) "msg" ;
  p->results[2].data[0] = (char*) "\"overrun\"" ;

  p->results[3].num_tags = 1 ;
  p->results[3].meta[0] = (char*) "position" ;
  p->results[3].data[0] = (char*) "\"latitude\"" ;

  p->results[4].num_tags = 1 ;
  p->results[4].meta[0] = (char*) "position" ;
  p->results[4].data[0] = (char*) "\"longitude\"" ;

  p->results[5].num_tags = 1 ;
  p->results[5].meta[0] = (char*) "time" ;
  p->results[5].data[0] = (char*) "\"utc\"" ;

  p->results[6].num_tags = 1 ;
  p->results[6].meta[0] = (char*) "satellites" ;
  p->results[6].data[0] = (char*) "\"used\"" ;

  p->results[7].num_tags = 1 ;
  p->results[7].meta[0] = (char*) "elevation" ;
  p->results[7].data[0] = (char*) "\"sealevel\"" ;

  p->results[8].num_tags = 1 ;
  p->results[8].meta[0] = (char*) "elevation" ;
  p->results[8].data[0] = (char*) "\"geoid\"" ;

  p->results[9].num_tags = 1 ;
  p->results[9].meta[0] = (char*) "nav" ;
  p->results[9].data[0] = (char*) "\"mode\"" ; // 1=none, 2=2D fix, 3=3D fix

  p->results[10].num_tags = 1 ;
  p->results[10].meta[0] = (char*) "dilution" ;
  p->results[10].data[0] = (char*) "\"position\"" ;

  p->results[11].num_tags = 1 ;
  p->results[11].meta[0] = (char*) "dilution" ;
  p->results[11].data[0] = (char*) "\"horizontal\"" ;

  p->results[12].num_tags = 1 ;
  p->results[12].meta[0] = (char*) "dilution" ;
  p->results[12].data[0] = (char*) "\"vertical\"" ;

  p->results[13].num_tags = 1 ;
  p->results[13].meta[0] = (char*) "speed" ;
  p->results[13].data[0] = (char*) "\"kmh\"" ;

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
                    msg_tokens[num_tokens] = (char*) "" ; // empty string
                  num_tokens++ ;
                  tok_start = tok_end + 1 ;
                }

              if (G_debug)
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

   Column names of extra metrics are generated in the format :

     <threadName>_<resultIdx>

   Thus in our above example, the column names will be written as,

     light1_0,range1_2
*/

void ft_gpslog (S_thread_entry *p)
{
  #define MAX_EXTRA_METRICS 8
  #define MAX_PARM_LEN 80

  /* our configuration */

  struct logger_configuration
  {
    int    gpsMode ;            // mode 3 means 3D fix
    int    minSatsUsed ;        // minimum satellites used
    int    normLogSecs ;        // normal logging interval (ie, moving)
    int    maxLogSecs ;         // maximum logging interval (ie, stationary)
    int    fileMaxSize ;        // log file (ie, CSV) max size
    int    ring_pos ;           // current insertion point in ring buffer
    int    ring_entries ;       // total elements in the ring buffer
    long   last_run ;           // time this function last ran
    long   last_update ;        // time of last ring buffer update
    double maxPosDil ;          // position dilution limit (ie, too inaccurate)
    double minDistMeters ;      // minimum delta to be considered movement
    double cur_ele ;            // current elevation (meters)
    double cur_lat ;            // current lattitude (degrees)
    double cur_long ;           // current longitude (degrees)
    double cur_epoch ;          // walllock time from satellite

    char   gpsThread[MAX_THREAD_NAME] ;         // name of ft_gpsmon()
    char   fileName[MAX_FILENAME] ;             // CSV log file
    char   extraMetrics[MAX_PARM_LEN] ;         // optional metrics

    int    xm_total ;                           // number of extra metrics
    int    xm_resultidx[MAX_EXTRA_METRICS] ;    // metrics result indexes
    char   *xm_threadname[MAX_EXTRA_METRICS] ;  // metrics thread names
  } ;
  typedef struct logger_configuration S_LoggerConf ;

  S_LoggerConf *cfg = NULL ;

  /* extra metrics, only used if user specifies "cfg_extraMetrics") */

  struct extra_metrics_s
  {
    int is_int ;        // a flag to determine if i_value or f_value is used
    int i_value ;
    double f_value ;
  } ;
  typedef struct extra_metrics_s S_ExtraMetrics ;

  /* our ringer buffer */

  #define MAX_RING_BUFFER_ELEMENTS 200 // protect against crazy big malloc()
  struct ring_entry_t
  {
    time_t epoch ;
    double latitude ;
    double longitude ;
    double elevation ;
    double dilution ;
    S_ExtraMetrics xmetrics[MAX_EXTRA_METRICS] ;
  } ;
  typedef struct ring_entry_t S_RingEntry ;

  static thread_local S_RingEntry *ring_buffer = NULL ; // the ring buffer

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
    p->results[0].meta[0] = (char*) "ringbuffer" ;
    p->results[0].data[0] = (char*) "\"size\"" ;

    p->results[1].num_tags = 1 ;
    p->results[1].meta[0] = (char*) "entries" ;
    p->results[1].data[0] = (char*) "\"written\"" ;

    p->results[2].num_tags = 1 ;
    p->results[2].meta[0] = (char*) "entries" ;
    p->results[2].data[0] = (char*) "\"moving\"" ;

    p->results[3].num_tags = 1 ;
    p->results[3].meta[0] = (char*) "entries" ;
    p->results[3].data[0] = (char*) "\"stationary\"" ;

    p->results[4].num_tags = 1 ;
    p->results[4].meta[0] = (char*) "files" ;
    p->results[4].data[0] = (char*) "\"rotated\"" ;

    p->num_int_results = 5 ;

    /* allocate memory for our configuration and initialize defaults */

    p->buf = malloc (sizeof(S_LoggerConf)) ;
    cfg = (S_LoggerConf*) p->buf ;
    memset (cfg, 0, sizeof(S_LoggerConf)) ;

    cfg->gpsMode = 3 ;
    cfg->minSatsUsed = 4 ;
    cfg->normLogSecs = 10 ;
    cfg->maxLogSecs = 60 ;
    cfg->fileMaxSize = 262144 ;

    cfg->maxPosDil = 3.5 ;
    cfg->minDistMeters = 12.0 ;

    strcpy (cfg->fileName, "/gpslog.csv") ;

    /* parse our config file, load configuration into thread_local vars */

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
                cfg->gpsMode = atoi (value) ;
              if (strcmp (key, "cfg_minSatsUsed") == 0)
                cfg->minSatsUsed = atoi (value) ;
              if (strcmp (key, "cfg_maxPosDil") == 0)
                cfg->maxPosDil = atof (value) ;
              if (strcmp (key, "cfg_minDistMeters") == 0)
                cfg->minDistMeters = atof (value) ;
              if (strcmp (key, "cfg_normLogSecs") == 0)
                cfg->normLogSecs = atoi (value) ;
              if (strcmp (key, "cfg_maxLogSecs") == 0)
                cfg->maxLogSecs = atoi (value) ;
              if (strcmp (key, "cfg_fileMaxSize") == 0)
                cfg->fileMaxSize = atoi (value) ;

              if ((strcmp (key, "cfg_gpsThread") == 0) &&
                  (strlen (value) < MAX_THREAD_NAME))
                strncpy (cfg->gpsThread, value, MAX_THREAD_NAME) ;

              if ((strcmp (key, "cfg_fileName") == 0) &&
                  (strlen (value) < MAX_FILENAME))
                strncpy (cfg->fileName, value, MAX_FILENAME) ;

              if ((strcmp (key, "cfg_extraMetrics") == 0) &&
                  (strlen (value) < MAX_PARM_LEN))
                strncpy (cfg->extraMetrics, value, MAX_PARM_LEN) ;
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
      if (cfg->minSatsUsed < 1)
        fault = (char*) "cfg_minSatsUsed cannot be less than 1" ;
      if (strlen(cfg->gpsThread) < 1)
        fault = (char*) "cfg_gpsThread cannot be empty" ;
      if (cfg->normLogSecs < 1)
        fault = (char*) "cfg_normLogSecs cannot be less than 1" ;
      if (cfg->maxLogSecs < cfg->normLogSecs)
        fault = (char*) "cfg_maxLogSecs cannot be less than cfg_normLogSecs" ;
      if (strlen(cfg->fileName) < 1)
        fault = (char*) "cfg_fileName cannot be empty" ;

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

    if (strlen(cfg->extraMetrics) > 0)
    {
      char *idx=NULL, *key=NULL ;
      key = strtok_r (cfg->extraMetrics, ",", &idx) ; // key is "<thr>:<i>"
      while (key)
      {
        cfg->xm_threadname[cfg->xm_total] = key ;
        cfg->xm_resultidx[cfg->xm_total] = -1 ;
        for (int i=0 ; i < strlen(key) ; i++)
          if (key[i] == ':')
          {
            key[i] = 0 ;
            cfg->xm_resultidx[cfg->xm_total] = atoi(key+i+1) ;
          }

        /* sanity check thread name and result index */

        if ((strlen(cfg->xm_threadname[cfg->xm_total]) < 1) ||
            (strlen(cfg->xm_threadname[cfg->xm_total]) > MAX_THREAD_NAME))
        {
          strcpy (p->msg, "FATAL! Invalid thread name") ;
          p->state = THREAD_STOPPED ; return ;
        }
        if ((cfg->xm_resultidx[cfg->xm_total] < 0) ||
            (cfg->xm_resultidx[cfg->xm_total] >= MAX_THREAD_RESULT_VALUES))
        {
          strcpy (p->msg, "FATAL! Invalid result index") ;
          p->state = THREAD_STOPPED ; return ;
        }

        cfg->xm_total++ ;
        if (cfg->xm_total > MAX_EXTRA_METRICS)
        {
          strcpy (p->msg, "FATAL! too many extra metrics") ;
          p->state = THREAD_STOPPED ; return ;
        }
        key = strtok_r (NULL, ",", &idx) ; // move on to next token
      }
    }

    /* calculate the size of our ring buffer and allocate memory */

    cfg->ring_entries = (cfg->maxLogSecs / cfg->normLogSecs) + 1 ;
    if (cfg->ring_entries > MAX_RING_BUFFER_ELEMENTS)
    {
      snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
                "FATAL! ring_buffer sz %d exceeds %d",
                 cfg->ring_entries, MAX_RING_BUFFER_ELEMENTS) ;
      p->state = THREAD_STOPPED ; return ;
    }
    size_t sz = cfg->ring_entries * sizeof (S_RingEntry) ;
    ring_buffer = (S_RingEntry*) malloc (sz) ;
    if (ring_buffer == NULL)
    {
      snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
                "FATAL! ring_buffer malloc(%d) failed", sz) ;
      p->state = THREAD_STOPPED ; return ;
    }
    memset (ring_buffer, 0, sz) ;

    p->results[0].i_value = sz ; /* ring buffer size */
    cfg->last_run = now ;
  }

  cfg = (S_LoggerConf*) p->buf ;

  /*
     attempt to identify the "cfg_gpsThread", make sure it's running. If not
     found, sleep and try again (but check if we're told to terminate).
  */

  int gps_tid ;
  for (gps_tid=0 ; gps_tid < MAX_THREADS ; gps_tid++)
    if ((G_thread_entry[gps_tid].state == THREAD_RUNNING) &&
        (G_thread_entry[gps_tid].ft_addr == ft_gpsmon) &&
        (strcmp(G_thread_entry[gps_tid].name, cfg->gpsThread) == 0))
      break ;
  if (gps_tid == MAX_THREADS) /* can't locate "cfg_gpsThread", retry */
  {
    strcpy (p->msg, "WARNING: Cannot find cfg_gpsThread") ;
    cfg->last_run = cfg->last_run + (cfg->normLogSecs * 1000) ;
    while (millis() < cfg->last_run)
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
  if ((cfg->gpsMode != (int) r_ptr[IDX_GPS_MODE].f_value) ||
      (cfg->minSatsUsed > (int) r_ptr[IDX_SATS_USED].f_value) ||
      (cfg->maxPosDil < r_ptr[IDX_POS_DIL].f_value))
  {
    snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
              "WARNING: No lock, mode:%d sats:%d dil:%s",
              (int) r_ptr[IDX_GPS_MODE].f_value,
              (int) r_ptr[IDX_SATS_USED].f_value,
              String (r_ptr[IDX_POS_DIL].f_value, FLOAT_DECIMAL_PLACES)) ;

    cfg->last_run = cfg->last_run + (cfg->normLogSecs * 1000) ;
    while (millis() < cfg->last_run)
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

  if ((cfg->cur_ele == 0.0) && (cfg->cur_lat == 0.0) && (cfg->cur_long == 0.0))
  {
    strcpy (p->msg, "INFO: initialized") ;
    cfg->cur_ele = r_ptr[IDX_ELE].f_value ;
    cfg->cur_lat = r_ptr[IDX_LAT].f_value ;
    cfg->cur_long = r_ptr[IDX_LONG].f_value ;
    cfg->cur_epoch = r_ptr[IDX_UTC].f_value ;
    cfg->last_update = now ;

    cfg->last_run = cfg->last_run + (cfg->normLogSecs * 1000) ;
    while (millis() < cfg->last_run)
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
  double minDegrees = atan (cfg->minDistMeters / earthRadius) * 180.0 / M_PI ;

  int we_moved = 0 ;
  if ((abs(r_ptr[IDX_LAT].f_value - cfg->cur_lat) > minDegrees) ||
      (abs(r_ptr[IDX_LONG].f_value - cfg->cur_long) > minDegrees))
  {
    we_moved = 1 ;
    p->results[2].i_value++ ; /* "moving" counter */
  }

  if (((we_moved) || (now - cfg->last_update > cfg->maxLogSecs * 1000)) &&
      (cfg->ring_pos < cfg->ring_entries)) // make sure there's space available
  {
    ring_buffer[cfg->ring_pos].epoch = int(r_ptr[IDX_UTC].f_value) ;
    ring_buffer[cfg->ring_pos].latitude = r_ptr[IDX_LAT].f_value ;
    ring_buffer[cfg->ring_pos].longitude = r_ptr[IDX_LONG].f_value ;
    ring_buffer[cfg->ring_pos].elevation = r_ptr[IDX_ELE].f_value ;
    ring_buffer[cfg->ring_pos].dilution = r_ptr[IDX_POS_DIL].f_value ;

    /*
       if there are extra metrics for us to add into ring_buffer, attempt
       to do so now. If something went wrong, don't increment "ring_pos" or
       "last_update", so that we retry.
    */

    if (cfg->xm_total > 0)
    {
      int metrics_obtained = 0 ;
      for (int xm_idx=0 ; xm_idx < cfg->xm_total ; xm_idx++)
        for (int t_idx=0 ; t_idx < MAX_THREADS ; t_idx++)
          if ((G_thread_entry[t_idx].state == THREAD_RUNNING) &&
               (strcmp(cfg->xm_threadname[xm_idx],
                G_thread_entry[t_idx].name) == 0))
          {
            int res_idx = cfg->xm_resultidx[xm_idx] ;

            if (G_thread_entry[t_idx].num_int_results > res_idx)  // is int
            {
              ring_buffer[cfg->ring_pos].xmetrics[xm_idx].is_int = 1 ;
              ring_buffer[cfg->ring_pos].xmetrics[xm_idx].i_value =
                G_thread_entry[t_idx].results[res_idx].i_value ;
              metrics_obtained++ ;

              if (G_debug) // validate what went into ring_buffer
              {
                char buf[BUF_SIZE] ;
                String s =
                  String(ring_buffer[cfg->ring_pos].xmetrics[xm_idx].i_value) ;
                snprintf (
                  buf, BUF_SIZE,
                  "DEBUG: ft_gpslog() ring_pos:%d %s_%d i:%d",
                  cfg->ring_pos,
                  cfg->xm_threadname[xm_idx],
                  cfg->xm_resultidx[xm_idx],
                  s.c_str()) ;
                Serial.println (buf) ;
              }
            }
            if (G_thread_entry[t_idx].num_float_results > res_idx) // is float
            {
              ring_buffer[cfg->ring_pos].xmetrics[xm_idx].is_int = 0 ;
              ring_buffer[cfg->ring_pos].xmetrics[xm_idx].f_value =
                G_thread_entry[t_idx].results[res_idx].f_value ;
              metrics_obtained++ ;

              if (G_debug) // validate what went into ring_buffer
              {
                char buf[BUF_SIZE] ;
                String s =
                  String(ring_buffer[cfg->ring_pos].xmetrics[xm_idx].f_value,
                         FLOAT_DECIMAL_PLACES) ;
                snprintf (
                  buf, BUF_SIZE,
                  "DEBUG: ft_gpslog() ring_pos:%d %s_%d f:%s",
                  cfg->ring_pos,
                  cfg->xm_threadname[xm_idx],
                  cfg->xm_resultidx[xm_idx],
                  s.c_str()) ;
                Serial.println (buf) ;
              }
            }
          }

      if (metrics_obtained == cfg->xm_total) // yay ! we got all extra metrics
      {
        cfg->ring_pos++ ;
        cfg->last_update = now ;
      }
    }
    else
    {
      cfg->ring_pos++ ;
      cfg->last_update = now ;
    }

    if (we_moved == 0)
      p->results[3].i_value++ ; /* "stationary" counter */
  }

  /*
     if our ring buffer is full, or if we've not flushed it in a long time,
     write it to storage
  */

  if (cfg->ring_pos == cfg->ring_entries)
  {
    char line[BUF_SIZE] ;

    File f = SPIFFS.open (cfg->fileName, "a") ;
    if (!f)
    {
      snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
                "FATAL! Cannot open %s for writing", cfg->fileName) ;
      free (ring_buffer) ;
      ring_buffer = NULL ;
      p->state = THREAD_STOPPED ; return ;
    }
    if (f.size() < 1)
    {
      strcpy (line, "time,latitude,longitude,elevation,dilution") ;
      for (int i=0 ; i < cfg->xm_total ; i++)
      {
        char label[MAX_THREAD_NAME+4] ;
        sprintf (label, ",%s_%d", cfg->xm_threadname[i], cfg->xm_resultidx[i]) ;
        strcat (line, label) ;
      }
      strcat (line, "\n") ;
      if (f.print (line) < 1)
      {
        snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
                  "FATAL! Cannot write to %s", cfg->fileName) ;
        free (ring_buffer) ;
        ring_buffer = NULL ;
        f.close () ;
        p->state = THREAD_STOPPED ; return ;
      }
    }

    for (int i=0 ; i < cfg->ring_pos ; i++)
    {
      String s_lat = String(ring_buffer[i].latitude, FLOAT_DECIMAL_PLACES) ;
      String s_long = String(ring_buffer[i].longitude, FLOAT_DECIMAL_PLACES) ;
      String s_elev = String(ring_buffer[i].elevation, FLOAT_DECIMAL_PLACES) ;
      String s_dilu = String(ring_buffer[i].dilution, FLOAT_DECIMAL_PLACES) ;
      snprintf (line, BUF_SIZE-1, "%ld,%s,%s,%s,%s",
                ring_buffer[i].epoch,
                s_lat.c_str(),
                s_long.c_str(),
                s_elev.c_str(),
                s_dilu.c_str()) ;

      /* if we've requested for extra metrics, append them to "line". */

      if (cfg->xm_total > 0)
      {
        for (int xm_idx=0 ; xm_idx < cfg->xm_total ; xm_idx++)
        {
          String s ;
          if (ring_buffer[i].xmetrics[xm_idx].is_int)
            s = String (ring_buffer[i].xmetrics[xm_idx].i_value) ;
          else
            s = String (ring_buffer[i].xmetrics[xm_idx].f_value,
                        FLOAT_DECIMAL_PLACES) ;
          strcat (line, ",") ;
          strcat (line, s.c_str()) ;
        }
      }

      strcat (line, "\n") ;
      f.print (line) ;
      p->results[1].i_value++ ; /* entries written counter */
    }
    cfg->ring_pos = 0 ; // reset ring buffer since it got flushed

    /* check if it's time to rotate cfg_fileName */

    int rotate = 0 ;
    if (f.size() > cfg->fileMaxSize)
      rotate = 1 ;
    f.close () ;

    if (rotate) // ... delete old file if present
    {
      char dst_name[BUF_SIZE] ;
      snprintf (dst_name, BUF_SIZE-1, "%s.old", cfg->fileName) ;
      File f = SPIFFS.open (dst_name) ;
      if (f)
      {
        f.close () ;
        SPIFFS.remove (dst_name) ;
      }
      SPIFFS.rename (cfg->fileName, dst_name) ;
      p->results[4].i_value++ ; /* files rotated counter */
    }
  }

  /*
     figure out how long to sleep, but take short naps and check if we're
     being asked to shutdown.
  */

  long duration = now + (cfg->normLogSecs * 1000) - millis () ;
  if (duration > 0)
  {
    snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
              "ring:%d/%d sleep:%ldms",
              cfg->ring_pos, cfg->ring_entries, duration) ;
    cfg->last_run = cfg->last_run + (cfg->normLogSecs * 1000) ;
    while (millis() < cfg->last_run)
    {
      delay (THREAD_SHUTDOWN_PERIOD / 20) ;
      GPSLOG_CHECK_CLEANUP ;
    }
  }
}

