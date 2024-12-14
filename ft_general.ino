void ft_counter (S_thread_entry *p)
{
  /* determine our parameters from "in_args" */

  if (p->num_args != 2)
  {
    strcpy (p->msg, "FATAL! Expecting 2x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }
  int delay_ms = atoi (p->in_args[0]) ;

  /* if "loops" is 0, this is our first call, initialize stuff */

  if (p->loops == 0)
  {
    p->num_int_results = 1 ;
    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = (char*) "myType" ;
    p->results[0].data[0] = (char*) "\"myCounter\"" ;
    p->results[0].i_value = atoi (p->in_args[1]) ;
    strcpy (p->msg, "ok") ;
  }

  /*
     "delay_ms" will determine how often this function calls f_delivery().
     For example, call f_delivery() every 10 seconds if delay_ms is 100.
  */

  #define REPORT_INTERVAL 100

  if (p->results[0].i_value % REPORT_INTERVAL == 0)
  {
    f_delivery (p, &p->results[0]) ;
    p->results[0].i_value++ ;
  }

  /* at this point, just increment our counter and take a break. */

  p->results[0].i_value++ ;
  delay (delay_ms) ;
}

void ft_aread (S_thread_entry *p)
{
  /* get ready our configuration */

  if ((p->num_args < 2) || (p->num_args > 5))
  {
    strcpy (p->msg, "FATAL! Expecting 2-5x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int delay_ms = atoi (p->in_args[0]) ;
  int inPin = atoi (p->in_args[1]) ;
  int pwrPin = -1 ;

  char *loThres=NULL, *hiThres=NULL ;
  if (p->num_args > 2)
    pwrPin = atoi(p->in_args[2]) ;      // optional arg
  if (p->num_args > 3)
    loThres = p->in_args[3] ;           // optional arg
  if (p->num_args > 4)
    hiThres = p->in_args[4] ;           // optional arg

  /* if "loops" is 0, this is our first call, initialize stuff */

  if (p->loops == 0)
  {
    pinMode (inPin, INPUT) ;
    if (pwrPin >= 0)
    {
      pinMode (pwrPin, OUTPUT) ;
      f_register_power_pin (pwrPin, p) ;
    }
    p->num_int_results = 1 ;
    p->results[1].i_value = millis () ; // use this to store time of last run
    strcpy (p->msg, "ok") ;
  }

  /* power on device (if specified) and wait till half time before we poll */

  if (pwrPin >= 0)
    digitalWrite (pwrPin, HIGH) ;       // power on device

  #define MAX_POWER_UP_DELAY 50

  int nap = delay_ms / 2 ;
  if (nap > MAX_POWER_UP_DELAY)
    nap = MAX_POWER_UP_DELAY ;
  if (nap > 0)
    delay (nap) ;

  int cur_value = analogRead (inPin) ;          // read new value
  if ((pwrPin >= 0) && (G_pin_flags[pwrPin].is_shared_power == 0))
    digitalWrite (pwrPin, LOW) ;                // power off device

  /* if "loThres" or "hiThres" is defined, check for state change */

  if (p->loops > 0)
  {
    char *cur_state = (char*) "normal" ;
    char *prev_state = (char*) "normal" ;
    if ((loThres != NULL) && (strlen(loThres) > 0))
    {
      int loValue = atoi (loThres) ;
      if (cur_value < loValue)
        cur_state = (char*) "low" ;
      if (p->results[0].i_value < loValue)
        prev_state = (char*) "low" ;
    }
    if ((hiThres != NULL) && (strlen(hiThres) > 0))
    {
      int hiValue = atoi (hiThres) ;
      if (cur_value > hiValue)
        cur_state = (char*) "high" ;
      if (p->results[0].i_value > hiValue)
        prev_state = (char*) "high" ;
    }
    if (cur_state != prev_state)
    {
      p->results[0].i_value = cur_value ;
      f_delivery (p, &p->results[0]) ;
    }
  }

  p->results[0].i_value = cur_value ;

  /* we're done, figure out how long we have left to sleep */

  p->results[1].i_value = p->results[1].i_value + delay_ms ;
  nap = p->results[1].i_value - millis () ;
  if (nap > 0)
    delay (nap) ;
}

/*
   This thread calls analogRead() as fast as possible to collect the specified
   number of samples and then sleeps until it's time to do work again. This
   is useful for sampling rapidly changing inputs and capturing its min, max
   and ave values.

   In order to hold a large number of samples, we need to allocate an array
   from heap. This means that when it's time to die, we need to immediately
   release this memory. For this reason, this thread does NOT return until
   it's time to die.

   Since taking samples is paced using "ets_delay_us()", which implements a
   busy wait, it is very important to prevent multiple ft_fast_aread() threads
   from entering their fast sampling loop concurrently. To prevent this, we
   acquire "G_fast_aread_lock" first.

   If the optional 5th argument "rolling_ms" is specified, then this function
   exposes an additional 3x metrics, ie, "rMin", "rAve" and "rMax". These
   represent rolling values observered over "rolling_ms". For example, if
   "delay_ms" is 2000 (ie, 2 sec) and "rolling_ms" is 60000 (ie, 60 secs),
   then this function allocates a 3 sets of 60000 / 2000 = 30x element arrays
   which store Min/Ave/Max values encountered over the past 60 secs. This is
   useful for capturing transient events which may span just a few seconds
   despite prometheus scraping us every minute.
*/

void ft_fast_aread (S_thread_entry *p)
{
  struct st_sample_set
  {
    unsigned short min ;
    unsigned short ave ;
    unsigned short max ;
  } ;
  typedef struct st_sample_set S_SampleSet ;

  int delay_ms, num_samples, gap_ms, in_pin, rolling_ms=0 ;
  int rolling_samples=0, rolling_offset=0 ;
  unsigned long last_run_ms, tv_start, tv_end ;
  S_SampleSet *sample_set=NULL ;

  /* sanity check our inputs and allocate memory first */

  if ((p->num_args != 4) && (p->num_args != 5))
  {
    strcpy (p->msg, "FATAL! Expecting 4x or 5x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  delay_ms = atoi (p->in_args[0]) ;             // duration of entire cycle
  num_samples = atoi (p->in_args[1]) ;          // number of analogRead() calls
  gap_ms = atoi (p->in_args[2]) ;               // timing gap between samples
  in_pin = atoi (p->in_args[3]) ;               // GPIO pin to read from
  if (p->num_args == 5)
  {
    rolling_ms = atoi (p->in_args[4]) ;
    if (rolling_ms < 2 * delay_ms)
    {
      strcpy (p->msg, "FATAL! rolling_ms must be at least 2x delay") ;
      p->state = THREAD_STOPPED ;
      return ;
    }
    rolling_samples = rolling_ms / delay_ms ;
    sample_set = (S_SampleSet*) malloc (sizeof(S_SampleSet) *
                                        rolling_samples) ;
    if (sample_set == NULL)
    {
      sprintf (p->msg, "FATAL! Cannot malloc() %dx sample_set",
               rolling_samples) ;
      p->state = THREAD_STOPPED ;
      return ;
    }
    memset (sample_set, 0, sizeof(S_SampleSet) * rolling_samples) ;
  }

  if (gap_ms * num_samples >= delay_ms)
  {
    strcpy (p->msg, "FATAL! samples * gap_ms >= delay") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  if (gap_ms >= THREAD_SHUTDOWN_PERIOD / 4)
  {
    sprintf (p->msg, "FATAL! gap_ms exceeds %d", THREAD_SHUTDOWN_PERIOD / 4) ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  unsigned short *samples = (unsigned short*) malloc (sizeof(unsigned short) *
                                                      num_samples) ;
  if (samples == NULL)
  {
    sprintf (p->msg, "FATAL! Cannot malloc() %d samples", num_samples) ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  /* set input pin mode and then setup results fields */

  pinMode (in_pin, INPUT) ;
  p->results[0].num_tags = 1 ;
  p->results[0].meta[0] = (char*) "type" ;
  p->results[0].data[0] = (char*) "\"Min\"" ;
  p->results[1].num_tags = 1 ;
  p->results[1].meta[0] = (char*) "type" ;
  p->results[1].data[0] = (char*) "\"Ave\"" ;
  p->results[2].num_tags = 1 ;
  p->results[2].meta[0] = (char*) "type" ;
  p->results[2].data[0] = (char*) "\"Max\"" ;
  if (rolling_samples > 0)
  {
    p->results[3].num_tags = 1 ;
    p->results[3].meta[0] = (char*) "type" ;
    p->results[3].data[0] = (char*) "\"rMin\"" ;
    p->results[4].num_tags = 1 ;
    p->results[4].meta[0] = (char*) "type" ;
    p->results[4].data[0] = (char*) "\"rAve\"" ;
    p->results[5].num_tags = 1 ;
    p->results[5].meta[0] = (char*) "type" ;
    p->results[5].data[0] = (char*) "\"rMax\"" ;
  }

  /* thread's main loop */

  int gap_usec = gap_ms * 1000 ;
  last_run_ms = millis () ;

  while (p->state == THREAD_RUNNING)
  {
    p->loops++ ; // fake our loop counter

    int idx ;

    xSemaphoreTake (G_fast_aread_lock, portMAX_DELAY) ;
    tv_start = millis() ;
    for (idx=0 ; idx <num_samples ; idx++)
    {
      samples[idx] = analogRead (in_pin) ;
      ets_delay_us (gap_usec) ; // NOTE !! this is a busy wait !!
    }
    tv_end = millis() ;
    xSemaphoreGive (G_fast_aread_lock) ;

    /*
       tune "gap_usec" so that the actual sampling interval time matches what
       was asked for. For example, if "gap_ms" is 2 and "samples" is 300, then
       we should be completing in 600 ms.
    */

    int actual_ms = tv_end - tv_start ;
    int intended_ms = num_samples * gap_ms ;
    int offset = abs(intended_ms - actual_ms) / 2 ; // apply small adjustment
    float adj_factor ;
    if (actual_ms > intended_ms)
      adj_factor = (float) (intended_ms - offset) / (float) intended_ms ;
    else
      adj_factor = (float) (intended_ms + offset) / (float) intended_ms ;
    gap_usec = (int) ((float) gap_usec * adj_factor) ;

    /* now calculate the min/max/ave values */

    unsigned short min_value, ave_value, max_value ;
    unsigned int  total=0 ;

    for (idx=0 ; idx < num_samples ; idx++)
    {
      total = total + samples[idx] ;
      if (idx == 0)
      {
        min_value = samples[idx] ;
        max_value = samples[idx] ;
      }
      else
      {
        if (samples[idx] < min_value)
          min_value = samples[idx] ;
        if (samples[idx] > max_value)
          max_value = samples[idx] ;
      }
    }
    ave_value = total / num_samples ;
    p->results[0].i_value = min_value ;
    p->results[1].i_value = ave_value ;
    p->results[2].i_value = max_value ;

    /* handle rolling samples, if requested */

    if (rolling_ms > 0)
    {
      sample_set[rolling_offset].min = min_value ;
      sample_set[rolling_offset].ave = ave_value ;
      sample_set[rolling_offset].max = max_value ;
      rolling_offset++ ;
      if (rolling_offset == rolling_samples)
        rolling_offset = 0 ;

      /* now calculate and expose rolling values */

      int elements = rolling_samples ;  // elements in "sample_set" to examine
      if (p->loops < rolling_samples)
        elements = p->loops ;

      total = 0 ;
      min_value = sample_set[0].min ;
      max_value = sample_set[0].max ;
      for (idx=1 ; idx < elements ; idx++)
      {
        total = total + sample_set[idx].ave ;
        if (sample_set[idx].min < min_value)
          min_value = sample_set[idx].min ;
        if (sample_set[idx].max > max_value)
          max_value = sample_set[idx].max ;
      }
      p->results[3].i_value = min_value ;
      p->results[4].i_value = total / elements ;
      p->results[5].i_value = max_value ;
      p->num_int_results = 6 ;
    }

    if (p->num_int_results == 0)
      p->num_int_results = 3 ;

    /* take short naps so we don't sleep past THREAD_SHUTDOWN_PERIOD */

    int max_nap = THREAD_SHUTDOWN_PERIOD / 5 ; // short naps
    int nap_ms = last_run_ms + delay_ms - millis() ;
    int total_napped=0 ;
    sprintf (p->msg, "loops:%d nap_ms:%d gap_usec:%d",
             p->loops, nap_ms, gap_usec) ;

    if (nap_ms > 0)
    {
      while ((total_napped < nap_ms) && (p->state == THREAD_RUNNING))
      {
        if (nap_ms - total_napped > max_nap)
        {
          delay (max_nap) ;
          total_napped = total_napped + max_nap ;
        }
        else
        {
          delay (nap_ms - total_napped) ;
          total_napped = nap_ms ;
        }
      }
      last_run_ms = last_run_ms + delay_ms ;
    }
  }
  free (samples) ;
  if (sample_set != NULL)
    free (sample_set) ;
}

void ft_dread (S_thread_entry *p)
{
  if ((p->num_args != 3) && (p->num_args != 4))
  {
    strcpy (p->msg, "FATAL! Expecting 3x or 4x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }
  int delay_ms = atoi (p->in_args[0]) ;         // how often we poll the pin
  int pin = atoi (p->in_args[1]) ;              // the pin we poll
  int mode = atoi (p->in_args[2]) ;             // whether to pull up to 3.3v
  int trig_ms = 0 ;                             // minimum pin high duration
  if (p->num_args == 4)
    trig_ms = atoi (p->in_args[3]) ;

  /* if "loops" is 0, this is our first call, initialize stuff */

  if (p->loops == 0)
  {
    if (mode == 0)
      pinMode (pin, INPUT) ;            // set mode to input (floating)
    else
    if (mode == 1)
      pinMode (pin, INPUT_PULLUP) ;     // use built-in pull up resistor
    else
    {
      strcpy (p->msg, "FATAL! Invalid mode") ;
      p->state = THREAD_STOPPED ;
      return ;
    }

    /* the current high or low state of the pin */

    p->results[0].meta[0] = (char*) "type" ;
    p->results[0].data[0] = (char*) "\"state\"" ;
    p->results[0].num_tags = 1 ;

    /* number of times pin was high for less than "trig_ms" */

    p->results[1].meta[0] = (char*) "type" ;
    p->results[1].data[0] = (char*) "\"short_triggers\"" ;
    p->results[1].num_tags = 1 ;

    /* number of times the pin was high for longer than "trig_ms" */

    p->results[2].meta[0] = (char*) "type" ;
    p->results[2].data[0] = (char*) "\"triggers\"" ;
    p->results[2].num_tags = 1 ;

    p->num_int_results = 3 ;
    strcpy (p->msg, "ok") ;

    p->results[0].i_value = digitalRead (pin) ; // one time initialization
  }

  /*
     if "trig_ms" is zero, then report pin change immediately. Otherwise,
     wait for pin to go high, make sure it stays high long enough, then
     emit an event and wait for the pin to go low before we return.
  */

  if (trig_ms == 0)
  {
    int cur_value = digitalRead (pin) ;
    if ((p->loops > 1) && (cur_value != p->results[0].i_value))
    {
      if (cur_value)                    // count a high state immediately
        p->results[2].i_value++ ;
      p->results[0].i_value = cur_value ;
      f_delivery (p, &p->results[0]) ;
    }
    p->results[0].i_value = cur_value ;
    delay (delay_ms) ;
    return ;
  }
  else
  {
    int cur_value = digitalRead (pin) ;

    /* if we just went from high to low, announce it and return. */

    if ((p->results[0].i_value == 1) && (cur_value == 0))
    {
      p->results[0].i_value = 0 ;
      f_delivery (p, &p->results[0]) ;
      delay (delay_ms) ;
      return ;
    }

    /* if we're still low, do nothing and return */

    if ((p->results[0].i_value == 0) && (cur_value == 0))
    {
      delay (delay_ms) ;
      return ;
    }

    /* if we just went from low to high, monitor for "trig_ms" */

    if ((p->results[0].i_value == 0) && (cur_value == 1))
    {
      unsigned long now = millis () ;
      while (millis() < now + trig_ms)
      {
        delay (delay_ms) ;
        if (digitalRead (pin) == 0)     // pin did not stay high long enough
        {
          p->results[1].i_value++ ;
          return ;
        }
      }
      p->results[0].i_value = 1 ;
      p->results[2].i_value++ ;
      f_delivery (p, &p->results[0]) ;  // announce that pin went high
      return ;
    }
  }
}

void ft_tread (S_thread_entry *p)
{
  #define TOUCH_SAMPLES 20 // calculate average value to suppress jitter

  if (p->num_args != 4)
  {
    strcpy (p->msg, "FATAL! Expecting 4x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  unsigned long start_time = millis () ;
  int delay_ms = atoi (p->in_args[0]) ;
  int pin = atoi (p->in_args[1]) ;
  int loThres = atoi (p->in_args[2]) ;
  int hiThres = atoi (p->in_args[3]) ;

  if (p->loops == 0)
  {
    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = (char*) "type" ;
    p->results[0].data[0] = (char*) "\"Cur\"" ;
    p->results[1].num_tags = 1 ;
    p->results[1].meta[0] = (char*) "type" ;
    p->results[1].data[0] = (char*) "\"Min\"" ;
    p->results[2].num_tags = 1 ;
    p->results[2].meta[0] = (char*) "type" ;
    p->results[2].data[0] = (char*) "\"Max\"" ;
    p->results[3].num_tags = 1 ;
    p->results[3].meta[0] = (char*) "type" ;
    p->results[3].data[0] = (char*) "\"State\"" ;
    strcpy (p->msg, "init") ;
  }

  /*
     Because the touch sensor gets lots of jitter, take multiple samples, find
     the standard deviation, then calculate the average from values within the
     standard deviation.
  */

  int i, val, samples=0 ;
  float raw[TOUCH_SAMPLES], total=0.0, mean, sd ;

  for (i=0 ; i < TOUCH_SAMPLES ; i++)
  {
    raw[i] = float (touchRead (pin)) ;
    total = total + raw[i] ;
  }
  mean = total / float(TOUCH_SAMPLES) ;

  /* sum up the square of each deviation */

  total = 0.0 ;
  for (i=0 ; i < TOUCH_SAMPLES ; i++)
    total = total + ((raw[i] - mean) * (raw[i] - mean)) ;

  /* standard deviation is the square root of average variance */

  sd = sqrt (total / float(TOUCH_SAMPLES)) ;

  /* calculate the average of datapoints within the standard deviation */

  total = 0.0 ;
  for (i=0 ; i < TOUCH_SAMPLES ; i++)
    if (fabsf (raw[i] - mean) <= sd)
    {
      total = total + raw[i] ;
      samples++ ;
    }
  val = int(total / float(samples)) ;

  if (p->loops == 0)
  {
    p->results[0].i_value = val ;       // current
    p->results[1].i_value = val ;       // minimum
    p->results[2].i_value = val ;       // maximum
    p->results[3].i_value = 0 ;         // state (0|1)
  }
  else
  if (samples > 1)
  {
    int prev = p->results[0].i_value ;
    p->results[0].i_value = val ;
    if (val < p->results[1].i_value)    // a lower minimum
      p->results[1].i_value = val ;
    if (val > p->results[2].i_value)    // a higher maximum
      p->results[2].i_value = val ;

    /*
       Check if we cross from a low to a high, or from high to low. Note
       that when touched (ie, state=1), "val" goes to a low value.
    */

    if ((p->results[3].i_value == 1) && (val > hiThres))        // touch off
    {
      p->results[3].i_value = 0 ;
      f_delivery (p, &p->results[3]) ;
    }
    if ((p->results[3].i_value == 0) && (val < loThres))        // touch on
    {
      p->results[3].i_value = 1 ;
      f_delivery (p, &p->results[3]) ;
    }
    p->num_int_results = 4 ;
  }

  /* see how long we get to sleep for */

  int nap = delay_ms - (millis() - start_time) ;
  sprintf (p->msg, "samples:%d sd:%d nap:%d", samples, (int)sd, nap) ;
  if (nap > 0)
    delay (nap) ;
}

/*
   This thread implements safe relay control. A relay is turned on by setting
   its control pin into INPUT mode (causing the pin to float high). The relay
   is turned off by setting the control pin to OUTPUT low. To command the relay
   on or off, an external command "relay <name> on|off" is invoked. Where
   "<name>" is our thread name. Once we are commanded on, we will stay on for
   another "dur" seconds before turning off automatically due to timeout (and
   entering a fault state). Once faulted, we will NOT turn on unless this
   thread is restarted.

   If we were in an ON state and the ESP32 reboots, the relay will remain in
   an ON state until this thread starts up (ie, "/autoexec.cfg").
*/

void ft_relay (S_thread_entry *p)
{
  #define RELAY_CHECK_INTERVAL_MS 80

  static thread_local int relay_state, fault_state ;

  if (p->num_args != 2)
  {
    strcpy (p->msg, "FATAL! Expecting 2x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int relay_pin = atoi (p->in_args[0]) ;
  int duration_secs = atoi (p->in_args[1]) ;

  if (p->loops == 0)
  {
    pinMode (relay_pin, INPUT) ;
    relay_state = 0 ;
    fault_state = 0 ;

    /* note: f_relay() reads this */

    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = (char*) "fault" ;
    p->results[0].data[0] = (char*) "\"timeout\"" ;
    p->results[0].i_value = duration_secs ;     // timeout to fault (secs)

    /* note: f_relay() reads and updates this */

    p->results[1].num_tags = 1 ;
    p->results[1].meta[0] = (char*) "relay" ;
    p->results[1].data[0] = (char*) "\"state\"" ;
    p->results[1].i_value = 0 ;                 // 0=off, 1=on, -1=fault

    /* note: f_relay() updates this */

    p->results[2].num_tags = 1 ;
    p->results[2].meta[0] = (char*) "autooff" ;
    p->results[2].data[0] = (char*) "\"time\"" ;
    p->results[2].i_value = 0 ;                 // millis() auto off time

    p->num_int_results = 3 ;
  }

  /* if we're in a fault state, turn OFF and do not continue */

  if (fault_state)
  {
    digitalWrite (relay_pin, HIGH) ;
    pinMode (relay_pin, INPUT) ;
    delay (RELAY_CHECK_INTERVAL_MS) ;
    return ;
  }

  /* check for state change request */

  if (relay_state != p->results[1].i_value)
  {
    relay_state = p->results[1].i_value ;
    if (relay_state == 0)                       // turn OFF
    {
      digitalWrite (relay_pin, HIGH) ;
      pinMode (relay_pin, INPUT) ;
    }
    if (relay_state == 1)                       // turn ON
    {
      pinMode (relay_pin, OUTPUT) ;
      digitalWrite (relay_pin, LOW) ;
    }
  }

  /* report on current state */

  long now = millis () ;
  if (relay_state == 0)
    strcpy (p->msg, "state:off") ;
  else
  {
    long time_left_ms = p->results[2].i_value - now ;
    sprintf (p->msg, "state:on timeout:%ds", time_left_ms / 1000) ;
  }

  /* if we're ON, check for timeout */

  if ((relay_state) && (now > p->results[2].i_value))
  {
    fault_state = 1 ;                   // set our internal fault flag
    p->results[1].i_value = -1 ;        // expose our fault state
    digitalWrite (relay_pin, HIGH) ;
    pinMode (relay_pin, INPUT) ;
    strcpy (p->msg, "state:off fault:on") ;
  }

  delay (RELAY_CHECK_INTERVAL_MS) ;
}

