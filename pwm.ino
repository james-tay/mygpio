/*
   References

   - https://github.com/espressif/arduino-esp32/blob/2.0.14/docs/source/api/ledc.rst

   USAGE

   On first use, the "G_pwm_instance" pointer (which is initially NULL), points
   to a buffer which holds an array of "S_pwm_instance" structures. The array
   size depends on the number of PWM_NUM_CHANNELS the device supports. Thus,
   the array index references a specific PWM channel assigned to drive a
   particular GPIO pin.

   The user should only be concerned with managing PWM on GPIO pins. We track
   the PWM channels used silently.

   When managing PWM on a particular GPIO pin with f_pwm_on(), scan through the
   G_pwm_instance array to determine whether we're creating or updating a PWM
   instance. A PWM instance is released in f_pwm_off(). After a PWM channel is
   activated, its duty cycle can be adjusted with f_pwm_duty().

   Incidentally, to calculate a PWM's pulse duration,
     - a cycle lasts "1 / freq" seconds
     - one cycle is 2^res units long
     - thus pulse duration (secs) = ( duty / 2^res ) * ( 1 / freq )

   For example, controlling a MG90S servo on GPIO5, using 8-bit resolution,

     % curl http://esp32/v1?cmd=pwm_on+5+50+20+8        # centered
     % curl http://esp32/v1?cmd=pwm_duty+5+33           # full anti-clockwise
     % curl http://esp32/v1?cmd=pwm_duty+5+7            # full clockwise

   When using 16-bit resolution,

     % curl http://esp32/v1?cmd=pwm_on+5+50+5100+16   # center, 1.556ms pulse
     % curl http://esp32/v1?cmd=pwm_duty+5+8500         # full anti-clockwise
     % curl http://esp32/v1?cmd=pwm_duty+5+1700         # full clockwise

   To "disengage" the servo (ie, no more hold-in-place),

     % curl http://esp32/v1?cmd=pwm_off+5
*/

void f_pwm_on (char **tokens)
{
  int gpio = atoi (tokens[1]) ;         // don't use input-only GPIO pins
  int freq = atoi (tokens[2]) ;         // in hertz
  int duty = atoi (tokens[3]) ;         // if "res" is 8-bit, duty is 1-255
  int res = atoi (tokens[4]) ;          // 1-20 bits for esp32

  /* initialize the "G_pwm_instance" on first use */

  if (G_pwm_instance == NULL)
  {
    int bufsize = PWM_NUM_CHANNELS * sizeof(S_pwm_instance) ;
    G_pwm_instance = (S_pwm_instance*) malloc (bufsize) ;
    if (G_pwm_instance == NULL)
    {
      sprintf (G_reply_buf, "FAULT: Cannot allocate %d bytes.\r\n", bufsize) ;
      return ;
    }
    memset (G_pwm_instance, 0, bufsize) ;
  }

  /* check if pin is already initialized, and if we have available channels */

  int i ;
  int first_available=-1 ;

  for (i=0 ; i < PWM_NUM_CHANNELS ; i++)
  {
    if (G_pwm_instance[i].gpio == gpio)
    {
      sprintf (G_reply_buf, "FAULT: GPIO%d already initialized.\r\n", gpio) ;
      return ;
    }
    if ((G_pwm_instance[i].gpio == 0) && (first_available < 0))
      first_available = i ; // note the first available PWM channel
  }
  if (first_available < 0)
  {
    sprintf (G_reply_buf, "FAULT: No PWM channels available, %d used.\r\n",
             PWM_NUM_CHANNELS) ;
    return ;
  }

  /* if we made it this far, initialize "first_available" instance. */

  if (ledcSetup (first_available, freq, res) == 0)
  {
    sprintf (G_reply_buf, "FAULT: ledcSetup() failed, freq:%d res:%d.\r\n",
             freq, res) ;
    return ;
  }
  ledcAttachPin (gpio, first_available) ;
  ledcWrite (first_available, duty) ;
  G_pwm_instance[first_available].gpio = gpio ;
  G_pwm_instance[first_available].res = res ;
  G_pwm_instance[first_available].freq = freq ;
  G_pwm_instance[first_available].duty = duty ;

  /* calculate pulse duration for fun */

  float duty_time = 1.0 / (float) freq ;
  float duty_fraction = (float) duty / powf (2.0, (float) res) ;
  float duty_pulse_ms = duty_fraction * duty_time * 1000.0 ;

  sprintf (G_reply_buf, "using chan:%d on gpio:%d for %.3fms.\r\n",
           first_available, gpio, duty_pulse_ms) ;
}

void f_pwm_duty (char **tokens)
{
  int gpio = atoi (tokens[1]) ;
  int duty = atoi (tokens[2]) ;

  if (G_pwm_instance == NULL)
  {
    sprintf (G_reply_buf, "FAULT: No PWM instances.\r\n") ;
    return ;
  }

  /* find the PWM instance we're talking about */

  int i ;
  for (i=0 ; i < PWM_NUM_CHANNELS ; i++)
    if (G_pwm_instance[i].gpio == gpio)
    {
      ledcWrite (i, duty) ;
      float duty_time = 1.0 / (float) G_pwm_instance[i].freq ;
      float duty_fraction = (float) duty /
                            powf (2.0, (float) G_pwm_instance[i].res) ;
      float duty_pulse_ms = duty_fraction * duty_time * 1000.0 ;
      sprintf (G_reply_buf, "updated chan:%d on gpio:%d to %.3fms.\r\n",
               i, gpio, duty_pulse_ms) ;
      return ;
    }

  strcpy (G_reply_buf, "FAULT: No PWM channel found.\r\n") ;
}

void f_pwm_off (char **tokens)
{
  int gpio = atoi (tokens[1]) ;

  if (G_pwm_instance == NULL)
  {
    sprintf (G_reply_buf, "FAULT: No PWM instances.\r\n") ;
    return ;
  }

  /* find the PWM instance we're talking about */

  int i ;
  for (i=0 ; i < PWM_NUM_CHANNELS ; i++)
    if (G_pwm_instance[i].gpio == gpio)
    {
      ledcDetachPin (gpio) ;
      memset (&G_pwm_instance[i], 0, sizeof(S_pwm_instance)) ;
      sprintf (G_reply_buf, "PWM channel %d released.\r\n", i) ;
      return ;
    }

  strcpy (G_reply_buf, "FAULT: No PWM channel found.\r\n") ;
}

