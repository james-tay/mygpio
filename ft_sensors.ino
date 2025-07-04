void ft_adxl335 (S_thread_entry *p)
{
  /* get ready our configuration */

  if (p->num_args != 6)
  {
    strcpy (p->msg, "FATAL! Expecting 6x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }
  if (p->loops == 0)
  {
    int pwrPin = atoi (p->in_args[5]) ;
    pinMode (pwrPin, OUTPUT) ;
    digitalWrite (pwrPin, HIGH) ;
    delay (50) ;

    p->results[0].num_tags = 2 ;
    p->results[0].meta[0] = (char*) "axis" ;
    p->results[0].data[0] = (char*) "\"x\"" ;
    p->results[0].meta[1] = (char*) "type" ;
    p->results[0].data[1] = (char*) "\"Min\"" ;

    p->results[1].num_tags = 2 ;
    p->results[1].meta[0] = (char*) "axis" ;
    p->results[1].data[0] = (char*) "\"x\"" ;
    p->results[1].meta[1] = (char*) "type" ;
    p->results[1].data[1] = (char*) "\"Ave\"" ;

    p->results[2].num_tags = 2 ;
    p->results[2].meta[0] = (char*) "axis" ;
    p->results[2].data[0] = (char*) "\"x\"" ;
    p->results[2].meta[1] = (char*) "type" ;
    p->results[2].data[1] = (char*) "\"Max\"" ;

    p->results[3].num_tags = 2 ;
    p->results[3].meta[0] = (char*) "axis" ;
    p->results[3].data[0] = (char*) "\"y\"" ;
    p->results[3].meta[1] = (char*) "type" ;
    p->results[3].data[1] = (char*) "\"Min\"" ;

    p->results[4].num_tags = 2 ;
    p->results[4].meta[0] = (char*) "axis" ;
    p->results[4].data[0] = (char*) "\"y\"" ;
    p->results[4].meta[1] = (char*) "type" ;
    p->results[4].data[1] = (char*) "\"Ave\"" ;

    p->results[5].num_tags = 2 ;
    p->results[5].meta[0] = (char*) "axis" ;
    p->results[5].data[0] = (char*) "\"y\"" ;
    p->results[5].meta[1] = (char*) "type" ;
    p->results[5].data[1] = (char*) "\"Max\"" ;

    p->results[6].num_tags = 2 ;
    p->results[6].meta[0] = (char*) "axis" ;
    p->results[6].data[0] = (char*) "\"z\"" ;
    p->results[6].meta[1] = (char*) "type" ;
    p->results[6].data[1] = (char*) "\"Min\"" ;

    p->results[7].num_tags = 2 ;
    p->results[7].meta[0] = (char*) "axis" ;
    p->results[7].data[0] = (char*) "\"z\"" ;
    p->results[7].meta[1] = (char*) "type" ;
    p->results[7].data[1] = (char*) "\"Ave\"" ;

    p->results[8].num_tags = 2 ;
    p->results[8].meta[0] = (char*) "axis" ;
    p->results[8].data[0] = (char*) "\"z\"" ;
    p->results[8].meta[1] = (char*) "type" ;
    p->results[8].data[1] = (char*) "\"Max\"" ;

    strcpy (p->msg, "ok") ;
  }

  /* parse config from "p->in_args", prepare them for f_adxl335() */

  char *t[6], s[BUF_SIZE] ;
  t[0] = NULL ;                 // don't care
  t[1] = p->in_args[2] ;        // x pin
  t[2] = p->in_args[3] ;        // y pin
  t[3] = p->in_args[4] ;        // z pin
  t[4] = p->in_args[1] ;        // aggregation duration (ms)
  t[5] = p->in_args[0] ;        // interval between samples (ms)

  int r[10] ;
  f_adxl335 (t, (int*) &r) ;

  /* place results from "r" into "results" array */

  p->results[0].i_value = r[1] ;
  p->results[1].i_value = r[2] ;
  p->results[2].i_value = r[3] ;
  p->results[3].i_value = r[4] ;
  p->results[4].i_value = r[5] ;
  p->results[5].i_value = r[6] ;
  p->results[6].i_value = r[7] ;
  p->results[7].i_value = r[8] ;
  p->results[8].i_value = r[9] ;

  p->num_int_results = 9 ; // "announce" that we have results to view
}

/*
   This thread grabs multiple samples from an HC-SR04, for up to "aggr_ms",
   before aggregating the results (ie, exposing min/ave/max/cur).  It then
   sleeps until its next measurement cycle, determined by "delay_ms". In
   general, a sample takes the longest to complete when measuring a distant
   object (eg, 4m), in which case 14 samples can be typically measured in about
   500ms (ie, "aggr_ms"). By contrast, with a near object (eg, 18cm), about 38
   measurements can be done in 500ms.

   If we have multiple HC-SR04 sensors attached to us, then running multiple
   instances of this thread can lead to multiple sensors firing concurrently
   and interfering with each other. To prevent this, we must acquire the mutex
   lock "G_hcsr04_lock" before we command our sensor to take a sample.
*/

void ft_hcsr04 (S_thread_entry *p)
{
  static thread_local int prev_state ;  // 0 when < thres, 1 when > thres

  if (p->num_args != 5)
  {
    strcpy (p->msg, "FATAL! Expecting 5x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  char s[BUF_SIZE] ;

  unsigned long delay_ms = atoi (p->in_args[0]) ;       // an entire cycle
  unsigned long aggr_ms = atoi (p->in_args[1]) ;        // repeatedly sample
  int trigPin = atoi (p->in_args[2]) ;                  // trigger GPIO pin
  int echoPin = atoi (p->in_args[3]) ;                  // response GPIO pin
  int thres = atoi (p->in_args[4]) ;                    // send an MQTT event
  int nap_ms = 0 ;

  if (delay_ms < aggr_ms)
  {
    strcpy (p->msg, "FATAL! 'delay' cannot be less than 'aggr'.") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  if (p->loops == 0)
  {
    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = (char*) "type" ;
    p->results[0].data[0] = (char*) "\"Min\"" ;

    p->results[1].num_tags = 1 ;
    p->results[1].meta[0] = (char*) "type" ;
    p->results[1].data[0] = (char*) "\"Ave\"" ;

    p->results[2].num_tags = 1 ;
    p->results[2].meta[0] = (char*) "type" ;
    p->results[2].data[0] = (char*) "\"Max\"" ;

    p->results[3].num_tags = 1 ;
    p->results[3].meta[0] = (char*) "type" ;
    p->results[3].data[0] = (char*) "\"Cur\"" ;

    p->results[0].i_value = millis () ; // use this to store time of last run
    prev_state = -1 ;
    strcpy (p->msg, "init") ;
  }
  else
  {
    /*
       "p->results[0].i_value" stores our last cycle start time. Sit here and
       wait until it's time to run again. We don't actually expose any integer
       results.
    */

    unsigned long next_run = p->results[0].i_value + delay_ms ;
    nap_ms = next_run - millis () ;
    if (nap_ms > 0)
      delay (nap_ms) ;

    p->results[0].i_value = p->results[0].i_value + delay_ms ;
  }

  /* quickly probe our device as fast as possible until "aggr_ms" passes */

  int samples=0, faults=0 ;
  double v_total=0.0, v_min, v_max ;
  unsigned long job_start = p->results[0].i_value ;
  unsigned long job_end = p->results[0].i_value + aggr_ms ;

  xSemaphoreTake (G_hcsr04_lock, portMAX_DELAY) ; // only 1x sensor polling

  while ((millis() < job_end) && (p->state == THREAD_RUNNING))
  {
    vTaskPrioritySet (p->tid, configMAX_PRIORITIES - 1) ;
    double f_value = f_hcsr04 (trigPin, echoPin) ;
    vTaskPrioritySet (p->tid, tskIDLE_PRIORITY) ;

    if (f_value > 0.0)                          // only process good data
    {
      v_total = v_total + f_value ;
      if (samples == 0)
        v_min = v_max = f_value ;
      else
      if (f_value > v_max)
        v_max = f_value ;
      else
      if (f_value < v_min)
        v_min = f_value ;
      samples++ ;
      p->results[3].f_value = f_value ;         // update current value
    }
    else
      faults++ ;

    /* show what's going on internally */

    sprintf (s, "[busy nap_ms:%d samples:%d faults:%d]",
             nap_ms, samples, faults) ;
    strcpy (p->msg, s) ;

    delay (10) ; // mandatory sleep (ms) inbetween samples
  }
  xSemaphoreGive (G_hcsr04_lock) ;

  /* "assemble" our final results */

  p->results[0].f_value = v_min ;
  p->results[1].f_value = v_total / (double) samples ;
  p->results[2].f_value = v_max ;

  if (samples > 0)
  {
    p->num_float_results = 4 ;                  // indicate results are good

    /* check if we had a state change, ie, crossed "thres" */

    int cur_state = 0 ;
    if (p->results[1].f_value > (double) thres)
      cur_state = 1 ;
    if ((prev_state > -1) && (prev_state != cur_state)) // skip init cycle
      f_delivery (p, &p->results[1]) ;                  // send ave value
    prev_state = cur_state ;
  }
  else
    p->num_float_results = 0 ;                  // indicate results are bad

  sprintf (s, "[idle nap_ms:%d samples:%d faults:%d]",
           nap_ms, samples, faults) ;
  strcpy (p->msg, s) ;
}

void ft_dht22 (S_thread_entry *p)
{
  #define DHT22_POWER_ON_DELAY_MS 800
  #define DHT22_RETRY_DELAY_MS 50
  #define DHT22_MAX_TEMPERATURE 140.0     // reject readings above this
  #define DHT22_MIN_TEMPERATURE -48.0     // reject readings under this

  /*
     keep track of previous temperature & humidity. If readings change too
     drastically, re-read to confirm them.
  */

  static thread_local float prev_t=0.0, prev_h=0.0 ;

  #define DHT22_MAX_TEMPERATURE_DELTA 5.0
  #define DHT22_MAX_HUMIDITY_DELTA 20.0

  /* determine our parameters from "in_args" */

  if (p->num_args != 3)
  {
    strcpy (p->msg, "FATAL! Expecting 3x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int delay_ms = atoi (p->in_args[0]) ;
  int dataPin = atoi (p->in_args[1]) ;
  int pwrPin = atoi (p->in_args[2]) ;
  unsigned long start_time = millis () ;
  unsigned long cutoff_time = start_time + delay_ms ;

  if (p->loops == 0)
  {
    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = (char*) "measurement" ;
    p->results[0].data[0] = (char*) "\"temperature\"" ;
    p->results[1].num_tags = 1 ;
    p->results[1].meta[0] = (char*) "measurement" ;
    p->results[1].data[0] = (char*) "\"humidity\"" ;
    p->results[2].num_tags = 1 ;
    p->results[2].meta[0] = (char*) "readings" ;
    p->results[2].data[0] = (char*) "\"abnormal\"" ;

    f_register_power_pin (pwrPin, p) ;
  }

  /* if pwrPin is > 0, that means it's a real pin, so power it on now */

  if (pwrPin >= 0)
  {
    pinMode (pwrPin, OUTPUT) ;
    digitalWrite (pwrPin, HIGH) ;
    delay (DHT22_POWER_ON_DELAY_MS) ;
  }

  int success=0 ;
  float temperature=0.0, humidity=0.0 ;

  while (millis() < cutoff_time)
  {
    /*
       reading DHT22 data is timing sensitive, set this thread priority high
       temporarily. If we don't do this, out thread WILL likely get context
       switched out in the middle of reading DHT22 data.
    */

    vTaskPrioritySet (p->tid, configMAX_PRIORITIES - 1) ;
    int result = f_dht22 (dataPin, &temperature, &humidity) ;
    vTaskPrioritySet (p->tid, tskIDLE_PRIORITY) ;

    if (result)
    {
      if (p->loops == 0)
      {
        prev_t = temperature ;
        prev_h = humidity ;
        sprintf (p->msg, "polled in %dms", millis()-start_time) ;
        success = 1 ;
        break ;
      }
      else
      {
        /*
           check if temperature & humidity readings are
             1. within an acceptable delta
             2. not unreasonably high/low
             3. are not both exactly 0.0
        */

        if ((fabsf(temperature - prev_t) < DHT22_MAX_TEMPERATURE_DELTA) &&
            (fabsf(humidity - prev_h) < DHT22_MAX_HUMIDITY_DELTA) &&
            (temperature < DHT22_MAX_TEMPERATURE) &&
            (temperature > DHT22_MIN_TEMPERATURE) &&
            ((temperature != 0.0) || (humidity != 0.0)))
        {
          success = 1 ;
          prev_t = temperature ;
          prev_h = humidity ;
        }
        else
          p->results[2].f_value = p->results[2].f_value + 1 ; // abnormal

        if (success)
        {
          sprintf (p->msg, "polled in %dms", millis()-start_time) ;
          break ;
        }
      }
    }
    delay (DHT22_RETRY_DELAY_MS) ;
  }

  /* if pwrPin is > 0, that means it's a real pin, so power it off now */

  if ((pwrPin >= 0) && (G_pin_flags[pwrPin].is_shared_power == 0))
    digitalWrite (pwrPin, LOW) ;

  /* report results (or failure) */

  if (success)
  {
    p->results[0].f_value = temperature ;
    p->results[1].f_value = humidity ;
    p->num_float_results = 3 ;
  }
  else
  {
    snprintf (p->msg, MAX_THREAD_MSG_BUF-1,
              "failed data:%d pwr:%d", dataPin, pwrPin) ;
    p->num_float_results = 0 ;
  }

  /* if we still have extra time, sleep a bit */

  unsigned long now = millis () ;
  if (now < cutoff_time)
    delay (cutoff_time - now) ;
}

void ft_ds18b20 (S_thread_entry *p)
{
  #define DS18B20_POWER_ON_DELAY_MS 500
  #define DS18B20_POWER_OFF_DELAY_MS 500
  #define DS18B20_RETRIES 10

  if ((p->num_args != 3) && (p->num_args != 4))
  {
    strcpy (p->msg, "FATAL! Expecting 3x or 4x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int delay_ms = atoi (p->in_args[0]) ;
  int dataPin = atoi (p->in_args[1]) ;
  int pwrPin = atoi (p->in_args[2]) ;
  int noAddr = 0 ;
  if ((p->num_args == 4) and (strcmp(p->in_args[3], "noaddr") == 0))
    noAddr = 1 ;
  unsigned long start_time = millis () ;

  /*
     Since this function calls f_ds18b20(), which returns multiple device
     addresses, we need a single static thread_local buffer which holds all
     null terminated strings of the device addresses in hex. To help us
     prepare this hex string, we use a temporary buffer "hex_buf" for this.
     Recall that a DS18B20's address is 8-bytes = 16+3 char string buffer,
     including double quotes required by prometheus's scrapes. To summarize :
       addr[] (binary) -> hex_buf -> addr_buf[] (text)
  */

  int results = 0 ;
  int retries = DS18B20_RETRIES ;
  int addr_size = (MAX_DS18B20_DEVICES * 8) + 1 ;
  float t[MAX_DS18B20_DEVICES] ;
  char hex_buf[16 + 3] ;
  unsigned char addr[addr_size] ;
  static thread_local char addr_buf[MAX_DS18B20_DEVICES * (16 + 3)] ;

  if (p->loops == 0)
  {
    memset (addr_buf, 0, MAX_DS18B20_DEVICES * (16 + 3)) ;

    /*
       turn on the power pin and leave it on in case we have other DS18B20s
       polled by different threads.
    */

    f_register_power_pin (pwrPin, p) ;
    pinMode (pwrPin, OUTPUT) ;
    digitalWrite (pwrPin, HIGH) ;
    delay (DS18B20_POWER_ON_DELAY_MS) ;
  }

  while (retries > 0)
  {
    memset (addr, 0, addr_size) ;
    vTaskPrioritySet (p->tid, configMAX_PRIORITIES - 1) ;
    results = f_ds18b20 (dataPin, addr, t) ;
    vTaskPrioritySet (p->tid, tskIDLE_PRIORITY) ;
    if (results < 1)
    {
      p->num_float_results = 0 ;
      retries-- ;
      sprintf (p->msg, "Read failed, retries %d.", retries) ;

      if (G_pin_flags[pwrPin].is_shared_power == 0)
      {
        pinMode (pwrPin, OUTPUT) ;
        digitalWrite (pwrPin, LOW) ;
        delay (DS18B20_POWER_OFF_DELAY_MS) ;      /* reboot the DS18B20 */
      }
      digitalWrite (pwrPin, HIGH) ;
      delay (DS18B20_POWER_ON_DELAY_MS) ;
    }
    else
    {
      /*
         depending on the number of "results" we received, this will determine
         our "p->num_float_results", where each result's "data[0]" is the
         hardcoded string "temperature", but "data[1]" is the device's address.
      */

      int r_offset=0 ; // our current read offset in "addr"
      int w_offset=0 ; // our current write offset in "addr_buf"

      for (int idx=0 ; idx < results ; idx++)
      {
        p->results[idx].f_value = t[idx] ;
        p->results[idx].num_tags = 1 ;
        p->results[idx].meta[0] = (char*) "measurement" ;
        p->results[idx].data[0] = (char*) "\"temperature\"" ;

        if (noAddr == 0)
        {
          sprintf (hex_buf, "\"%02x%02x%02x%02x%02x%02x%02x%02x\"",
                   addr[r_offset], addr[r_offset+1],
                   addr[r_offset+2], addr[r_offset+3],
                   addr[r_offset+4], addr[r_offset+5],
                   addr[r_offset+6], addr[r_offset+7]) ;
          memcpy (addr_buf+w_offset, hex_buf, 16 + 2) ;

          p->results[idx].num_tags = 2 ;
          p->results[idx].meta[0] = (char*) "measurement" ;
          p->results[idx].data[0] = (char*) "\"temperature\"" ;
          p->results[idx].meta[1] = (char*) "address" ;
          p->results[idx].data[1] = addr_buf + w_offset ;
        }

        r_offset = r_offset + 8 ;       // move to next device address
        w_offset = w_offset + 16 + 3 ;  // move to next string
      }
      p->num_float_results = results ;
      break ;
    }
  }
  if (retries == 0)
    p->num_float_results = 0 ;

  /* now figure out how long to nap for */

  unsigned long end_time = millis () ;
  int duration_ms = end_time - start_time ;
  if (p->num_float_results)
    sprintf (p->msg, "polled %d in %dms", results, duration_ms) ;
  int nap = delay_ms - duration_ms ;
  if (nap > 0)
    delay (nap) ;
}

