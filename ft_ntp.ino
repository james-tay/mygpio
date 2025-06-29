/*
   References
   - https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/system_time.html
   - https://github.com/espressif/esp-idf/blob/v5.4.2/components/lwip/include/apps/esp_sntp.h
*/

#include "esp_sntp.h"

/*
   This thread configures an NTP client. Once configured, it does not actually
   do anything, apart from periodically updating its "p->msg", which can be
   observed via the "thread_list" command. Its main purpose is to allow other
   threads/systems to obtain the wallclock time to be accessed via the
   "getLocalTime()" function.
*/

void ft_ntpclient (S_thread_entry *p)
{
  if (p->num_args != 1)
  {
    strcpy (p->msg, "FATAL! Expecting 1x argument") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  /* one time initialization, when this thread starts up */

  if (p->loops == 0)
  {
    sntp_stop () ; // make sure this is stopped (or we'll crash if we proceed).

    sntp_setoperatingmode (SNTP_OPMODE_POLL) ;
    sntp_set_sync_mode (SNTP_SYNC_MODE_IMMED) ;
    sntp_setservername (0, p->in_args[0]) ;

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

