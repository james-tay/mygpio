/*
   This function does not return until its "p->state" is set to THREAD_WRAPUP.
   Its job is to create and bind a TCP listening socket, initialize a hardware
   uart (if needed) and await bytes from either the TCP client or the uart.
   Note that only 1x connected TCP client may be serviced at a time.
*/

void ft_serial2tcp (S_thread_entry *p)
{
  if (p->num_args != 4)
  {
    strcpy (p->msg, "FATAL! Expecting 4x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int listen_sd = -1 ;
  int client_sd = -1 ;
  int port = atoi (p->in_args[0]) ;
  int baud = atoi (p->in_args[1]) ;
  int rx_pin = atoi (p->in_args[2]) ;
  int tx_pin = atoi (p->in_args[3]) ;

  /* configure metrics that we will expose */

  p->results[0].num_tags = 1 ;
  p->results[0].meta[0] = "client" ;
  p->results[0].data[0] = "\"connects\"" ;

  p->results[1].num_tags = 1 ;
  p->results[1].meta[0] = "client" ;
  p->results[1].data[0] = "\"connected\"" ;

  p->results[2].num_tags = 1 ;
  p->results[2].meta[0] = "uart_bytes" ;
  p->results[2].data[0] = "\"read\"" ;

  p->results[3].num_tags = 1 ;
  p->results[3].meta[0] = "uart_bytes" ;
  p->results[3].data[0] = "\"written\"" ;

  p->num_int_results = 4 ;

  /* create a TCP socket, bind to port and listen */

  listen_sd = socket (AF_INET, SOCK_STREAM, IPPROTO_IP) ;
  if (listen_sd < 0)
  {
    strcpy (p->msg, "FATAL! socket() failed") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  struct sockaddr_in addr ;
  memset (&addr, 0, sizeof(addr)) ;
  addr.sin_family = AF_INET ;
  addr.sin_addr.s_addr = INADDR_ANY ;
  addr.sin_port = htons (port) ;
  if (bind (listen_sd, (const struct sockaddr*) &addr, sizeof(addr)) < 0)
  {
    close (listen_sd) ;
    strcpy (p->msg, "FATAL! bind() failed") ;
    p->state = THREAD_STOPPED ;
    return ;
  }
  if (listen (listen_sd, 1) != 0)
  {
    close (listen_sd) ;
    strcpy (p->msg, "FATAL! listen() failed") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  /* if we're the first thread to access the hardware UART, initialize it */

  xSemaphoreTake (G_hw_uart->lock, portMAX_DELAY) ;
  G_hw_uart->in_use++ ;
  if (G_hw_uart->initialized == 0)
  {
    Serial2.begin (baud, SERIAL_8N1, rx_pin, tx_pin) ;
    G_hw_uart->initialized = 1 ;
  }
  xSemaphoreGive (G_hw_uart->lock) ;

  /* thread's main loop and variables to manage TCP sockets and UART data */

  int result, num_fds, amt ;
  char buf[BUF_SIZE] ;
  fd_set rfds ;
  struct timeval tv ;
  sprintf (p->msg, "rx:%d,tx:%d,port:%d", rx_pin, tx_pin, port) ;

  while (p->state == THREAD_RUNNING)
  {
    /*
       If we have no connected TCP client, check if "listen_sd" has a new
       client for us. Also, don't check for new clients if we already have
       a connected TCP client.
    */

    tv.tv_sec = 0 ;
    tv.tv_usec = 20 ;                   // stall select() for a short time
    FD_ZERO (&rfds) ;
    if (client_sd > 0)                  // check client for data
    {
      FD_SET (client_sd, &rfds) ;
      num_fds = client_sd + 1 ;
    }
    else                                // check for new client
    {
      FD_SET (listen_sd, &rfds) ;
      num_fds = listen_sd + 1 ;
    }
    result = select (num_fds, &rfds, NULL, NULL, &tv) ;

    if (result == 1)
    {
      if (FD_ISSET (listen_sd, &rfds))                  // new tcp client !!
      {
        client_sd = accept (listen_sd, NULL, NULL) ;
        p->results[0].i_value++ ;
        p->results[1].i_value = 1 ;
      }

      if (FD_ISSET (client_sd, &rfds))
      {
        amt = read (client_sd, buf, BUF_SIZE) ;
        if (amt < 1)                                    // client closed
        {
          close (client_sd) ;
          client_sd = -1 ;
          p->results[1].i_value = 0 ;
        }
        else                                            // send data to uart
        {
          Serial2.write (buf, amt) ;
          p->results[3].i_value = p->results[3].i_value + amt ;
          if (p->results[3].i_value < 0)
            p->results[3].i_value = 0 ; // fix rollover
        }
      }
    }

    /*
       check if data is available on the serial port, for some reason,
       "Serial2.read()" does not block.
    */

    amt = Serial2.read (buf, BUF_SIZE) ;
    if (amt > 0)
    {
      p->results[2].i_value = p->results[2].i_value + amt ;
      if (p->results[2].i_value < 0)
        p->results[2].i_value = 0 ; // fix rollover
      if (client_sd > 0)
        write (client_sd, buf, amt) ;
    }
  }

  /*
     if we're here, indicate we're not using the uart anymore and close TCP
     sockets in use.
  */

  xSemaphoreTake (G_hw_uart->lock, portMAX_DELAY) ;
  G_hw_uart->in_use-- ;
  xSemaphoreGive (G_hw_uart->lock) ;

  close (listen_sd) ;
  if (client_sd > 0)
    close (client_sd) ;

  p->state = THREAD_STOPPED ;
}

