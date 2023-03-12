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

