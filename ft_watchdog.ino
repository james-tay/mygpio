/*
   This function reboots the device if there might be something wrong with
   our network. Unfortunately, ping'ing a device is fairly complicated.
   Testing the network by reaching out by TCP (or UDP) requires some kind
   of external server and protocol. Thus, the easiest means of determining
   if we're reachable on the network, is to monitor for,
     - we received a REST command (including "/metrics")
     - we received an MQTT command
     - we received a serial command
   We will trigger a reboot if,
     - our uptime is at least N seconds
     - no activity occured for N seconds
*/

void ft_watchdog (S_thread_entry *p)
{
  #define WG_MIN_UPTIME_SECS 60 /* don't allow reboot too soon */
  #define WG_MIN_NO_ACTIVITY_SECS 10 /* min value for "no_activity_secs */

  /* items we track in S_Metrics */

  static thread_local unsigned long serialCmds ;
  static thread_local unsigned long restCmds ;
  static thread_local unsigned long mqttSubs ;
  static thread_local unsigned long last_activity ;

  static thread_local int interval_secs ;
  static thread_local int min_uptime_secs ;
  static thread_local int no_activity_secs ;

  unsigned long now_secs = millis () / 1000 ;

  /* if "loops" is 0, this is our first call, initialize stuff */

  if (p->loops == 0)
  {
    serialCmds = G_Metrics->serialCmds ;
    restCmds = G_Metrics->restCmds ;
    mqttSubs = G_Metrics->mqttSubs ;
    last_activity = now_secs ;

    /* parse (and sanitize) our arguments */

    if (p->num_args != 3)
    {
      strcpy (p->msg, "FATAL! Expecting 3x arguments") ;
      p->state = THREAD_STOPPED ;
      return ;
    }

    interval_secs = atoi(p->in_args[0]) ;
    min_uptime_secs = atoi(p->in_args[1]) ;
    no_activity_secs = atoi(p->in_args[2]) ;

    if (interval_secs < 1)
      interval_secs = 1 ;
    if (min_uptime_secs < WG_MIN_UPTIME_SECS)
      min_uptime_secs = WG_MIN_UPTIME_SECS ;
    if (no_activity_secs < WG_MIN_NO_ACTIVITY_SECS)
      no_activity_secs = WG_MIN_NO_ACTIVITY_SECS ;
  }

  /* now inspect "G_Metrics" and see if we received activity recently */

  if (G_Metrics->serialCmds > serialCmds)
  {
    serialCmds = G_Metrics->serialCmds ;
    last_activity = now_secs ;
  }
  if (G_Metrics->restCmds > restCmds)
  {
    restCmds = G_Metrics->restCmds ;
    last_activity = now_secs ;
  }
  if (G_Metrics->mqttSubs > mqttSubs)
  {
    mqttSubs = G_Metrics->mqttSubs ;
    last_activity = now_secs ;
  }





  delay (interval_secs * 1000) ;
}

