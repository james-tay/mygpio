/*
   References
   - https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/system_time.html
   - https://github.com/espressif/esp-idf/blob/v5.4.2/components/lwip/include/apps/esp_sntp.h
*/

#include "esp_sntp.h"

/*
   The following global variable is shared by ft_ntp_sync_callback() and
   ft_ntpclient().
*/

S_thread_entry *G_ntpclient_p = NULL ;

/*
   This callback is invoked whenever we receive a time sync event from the
   NTP server. Our job is to update ft_ntpclient() thread's p->results[0],
   but there's no way to pass that in as this function's arguments. So we
   must use a global variable "G_ntpclient_p" to reference it.
*/

void ft_ntp_sync_callback (struct timeval *tv)
{
  S_thread_entry *p = G_ntpclient_p ;

  if ((p != NULL) &&
      (p->ft_addr == ft_ntpclient) &&
      (p->state == THREAD_RUNNING) &&
      (p->num_int_results == 1))
    p->results[0].i_value++ ;
}

/*
   This thread configures an NTP client. Once configured, it does not actually
   do anything, apart from periodically updating its "p->msg", which can be
   observed via the "thread_list" command. Its main purpose is to allow other
   threads/systems to obtain the wallclock time to be accessed via the
   "getLocalTime()" function.
*/

void ft_ntpclient (S_thread_entry *p)
{
  if (p->num_args != 2)
  {
    strcpy (p->msg, "FATAL! Expecting 2x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  /* one time initialization, when this thread starts up */

  if (p->loops == 0)
  {
    G_ntpclient_p = p ; // this allows ft_ntp_sync_callback() to access us.

    /* setup our "results", which consist of number of NTP updates received */

    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = (char*) "event" ;
    p->results[0].data[0] = (char*) "\"updates\"" ;
    p->num_int_results = 1 ;

    sntp_stop () ; // make sure this is stopped (or we'll crash if we proceed).

    int sync_interval_secs = atoi (p->in_args[1]) ;
    if (sync_interval_secs < 15) // RFC 4330 says this cannot be less than 15s
      sync_interval_secs = 15 ;

    sntp_setoperatingmode (SNTP_OPMODE_POLL) ;
    sntp_set_sync_mode (SNTP_SYNC_MODE_IMMED) ;
    sntp_setservername (0, p->in_args[0]) ;
    sntp_set_sync_interval (sync_interval_secs * 1000) ;
    sntp_set_time_sync_notification_cb (ft_ntp_sync_callback) ;

    /*
       Try "sntp_restart()" first and if it fails, that means we never called
       "sntp_init()".
    */

    if (!sntp_restart())
      sntp_init () ;
    strcpy (p->msg, "sntp client running") ;
  }

  /* if we've been told to terminate (recall we have 1 sec for this) */

  if ((p->loops > 0) && (p->state == THREAD_WRAPUP))
  {
    sntp_stop () ;
    strcpy (p->msg, "sntp client stopped") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  /* update "msg" with the current timestamp to prove we're good */

  if (p->state == THREAD_RUNNING)
  {
    struct tm tm_now;
    if (getLocalTime(&tm_now))
      strftime (p->msg, MAX_THREAD_MSG_BUF-1, "%Y%m%d-%H%M%S UTC", &tm_now) ;
    else
      strcpy (p->msg, "sntp not in sync") ;
  }
  delay (500) ;
}

