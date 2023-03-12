/*
   This function initializes an I2S audio source and then enters a main loop
   where it reads samples from a DMA buffer and sends them via UDP. This
   function lives in its main loop does not return unless "p->state" is set
   to THREAD_WRAPUP.
*/

void ft_i2sin (S_thread_entry *p)
{
  if ((p->num_args != 5) && (p->num_args != 6))
  {
    strcpy (p->msg, "FATAL! Expecting 5x or 6x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int sampleRate = atoi (p->in_args[0]) ;
  int SCKpin = atoi (p->in_args[1]) ;
  int WSpin = atoi (p->in_args[2]) ;
  int SDpin = atoi (p->in_args[3]) ;
  int dma_bufsize = MYI2S_BITS_PER_SAMPLE / 8 * MYI2S_DMA_SAMPLES ;

  int sd = -1 ;                         // xmit UDP socket descriptor
  double gain = 1.0 ;                   // software amplification (optional)
  double sample_ref = 0.0 ;             // the "zero" value for analog samples
  struct sockaddr_in dest_addr ;        // destination address (and port)
  size_t dma_read = 0 ;                 // number of bytes read from DMA
  unsigned long *dma_buf = NULL ;       // work buffer to copy DMA data to
  unsigned long sample_lo, sample_hi ;  // used to calculate dynamic range
  unsigned long long total = 0 ;        // used to calculate sample zero ref
  esp_err_t e ;                         // generic error return value

  /* parse "<ip>:<port>" */

  #define DEST_LEN 24 // buffer for storing "<ip>:<port>"
  char ip[DEST_LEN] ;
  if (strlen(p->in_args[4]) >= DEST_LEN-1)
  {
    strcpy (p->msg, "Invalid <ip>:<port>") ;
    p->state = THREAD_STOPPED ;
    return ;
  }
  strcpy (ip, p->in_args[4]) ;
  int idx, port=0 ;
  for (idx=1 ; idx < strlen(ip) ; idx++)
  {
    if (ip[idx] == ':')
    {
      ip[idx] = 0 ;
      port = atoi (ip + idx + 1) ;
    }
  }
  if ((port == 0) || (port > 65535))
  {
    strcpy (p->msg, "Invalid port number") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  /* software gain is optional */

  if (p->num_args == 6)
    gain = atof (p->in_args[5]) ;

  /* do one time I2S initialization (well, per thread instance) */

  if (p->loops == 0)
  {
    const i2s_config_t i2s_config = {
      .mode = i2s_mode_t (I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = (uint32_t) sampleRate,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = i2s_comm_format_t (I2S_COMM_FORMAT_I2S |
                                                 I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = MYI2S_DMA_IN_BUFS,
      .dma_buf_len = MYI2S_DMA_SAMPLES
    } ;

    const i2s_pin_config_t pin_config = {
      .bck_io_num = SCKpin,                     // serial clock
      .ws_io_num = WSpin,                       // word select
      .data_out_num = I2S_PIN_NO_CHANGE,        // not used for audio in
      .data_in_num = SDpin                      // serial data
    } ;

    e = i2s_driver_install (MYI2S_INPUT_PORT, &i2s_config, 0, NULL) ;
    if (e != ESP_OK)
    {
      sprintf (p->msg, "i2s_driver_install() failed %d", e) ;
      p->state = THREAD_STOPPED ;
      return ;
    }

    /* the following 2x lines are hacks to make the SPH0645 work right */

    REG_SET_BIT (I2S_TIMING_REG(MYI2S_INPUT_PORT), BIT(9)) ;
    REG_SET_BIT (I2S_CONF_REG(MYI2S_INPUT_PORT), I2S_RX_MSB_SHIFT) ;

    e = i2s_set_pin (MYI2S_INPUT_PORT, &pin_config) ;
    if (e != ESP_OK)
    {
      sprintf (p->msg, "i2s_set_pin() failed %d", e) ;
      p->state = THREAD_STOPPED ;
      return ;
    }

    /* track i2s_read() success/failures, and timing info in our results[] */

    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = (char*) "dma_read" ;
    p->results[0].data[0] = (char*) "\"success\"" ;
    p->results[0].i_value = 0 ;                // i2s_read() bytes_read == size

    p->results[1].num_tags = 1 ;
    p->results[1].meta[0] = (char*) "dma_read" ;
    p->results[1].data[0] = (char*) "\"fails\"" ;
    p->results[1].i_value = 0 ;                // i2s_read() bytes_read < size

    p->results[2].num_tags = 1 ;
    p->results[2].meta[0] = (char*) "loop_work" ;
    p->results[2].data[0] = (char*) "\"ms\"" ;
    p->results[2].i_value = 0 ;                // millisecs last main work

    p->results[3].num_tags = 1 ;
    p->results[3].meta[0] = (char*) "loop_total" ;
    p->results[3].data[0] = (char*) "\"ms\"" ;
    p->results[3].i_value = 0 ;                // millisecs last main loop

    p->results[4].num_tags = 1 ;
    p->results[4].meta[0] = (char*) "signal" ;
    p->results[4].data[0] = (char*) "\"dynrange\"" ;
    p->results[4].i_value = 0 ;                // dynamic range (before gain)

    p->results[5].num_tags = 1 ;
    p->results[5].meta[0] = (char*) "signal" ;
    p->results[5].data[0] = (char*) "\"clipped\"" ;
    p->results[5].i_value = 0 ;                // sample sets clipped

    p->num_int_results = 6 ;

    /* prepare a UDP socket which we use to stream audio data */

    sd = socket (AF_INET, SOCK_DGRAM, IPPROTO_IP) ;
    if (sd < 0)
    {
      strcpy (p->msg, "socket(SOCK_DRAM) failed") ;
      p->state = THREAD_STOPPED ;
      i2s_stop (MYI2S_INPUT_PORT) ;
      i2s_driver_uninstall (MYI2S_INPUT_PORT) ;
      return ;
    }
    dest_addr.sin_addr.s_addr = inet_addr (ip) ;
    dest_addr.sin_family = AF_INET ;
    dest_addr.sin_port = htons (port) ;

    /* allocate a buffer to copy DMA contents to */

    dma_buf = (unsigned long*) malloc (dma_bufsize) ;
    if (dma_buf == NULL)
    {
      sprintf (p->msg, "dma_buf malloc(%d) failed", dma_bufsize) ;
      p->state = THREAD_STOPPED ;
      if (sd >= 0) close (sd) ;
      i2s_stop (MYI2S_INPUT_PORT) ;
      i2s_driver_uninstall (MYI2S_INPUT_PORT) ;
      return ;
    }

    /* do a test read */

    e = i2s_read (MYI2S_INPUT_PORT, dma_buf, dma_bufsize, &dma_read,
                  MYI2S_DMA_WAITTICKS) ;
    if ((e != ESP_OK) || (dma_read != dma_bufsize))
    {
      sprintf (p->msg, "i2s_read() failed %d", e) ;
      if (sd >= 0) close (sd) ;
      free (dma_buf) ;
      i2s_stop (MYI2S_INPUT_PORT) ;
      i2s_driver_uninstall (MYI2S_INPUT_PORT) ;
      return ;
    }
    sprintf (p->msg, "udp->%s:%d", ip, port) ;  // indicate we're all set !
  }

  unsigned int tv_loop=0, tv_work=0, tv_end ;
  while (p->state == THREAD_RUNNING)
  {
    tv_loop = millis () ;
    dma_read = 0 ;
    e = i2s_read (MYI2S_INPUT_PORT, dma_buf, dma_bufsize, &dma_read,
                  MYI2S_DMA_WAITTICKS) ;
    tv_work = millis () ;
    if ((e == ESP_OK) && (dma_read == dma_bufsize))
    {
      /* measure our samples to find dynamic range and zero reference value */

      sample_lo = sample_hi = total = dma_buf[0] ; // recalculate reference
      for (idx=1 ; idx < MYI2S_DMA_SAMPLES ; idx++)
      {
        if (dma_buf[idx] < sample_lo)
          sample_lo = dma_buf[idx] ;
        if (dma_buf[idx] > sample_hi)
          sample_hi = dma_buf[idx] ;
        total = total + dma_buf[idx] ;
      }
      long long ll = total / MYI2S_DMA_SAMPLES ;
      sample_ref = (double) (ll) ;                    // signal's "zero" level

      /*
         calculate dynamic range as a percentage against an unsigned 32-bit
         number (ie, save it as an integer result between 0 to 100)
      */

      long long drange = sample_hi - sample_lo ;
      p->results[4].i_value = (int) (drange >> 1) ;

      /* if optional gain is set, process it now */

      if (gain != 1.0)
      {
        /* figure out max gain without clipping */

        double apply_gain = gain ;
        double max_gain = (double) UINT_MAX / drange ;
        if (gain > max_gain)
        {
          apply_gain = max_gain ;               // limit the gain we'll apply
          p->results[5].i_value++ ;             // high gain clip
        }
        for (idx=0 ; idx < MYI2S_DMA_SAMPLES ; idx++) // apply software gain
        {
          double f = (double) dma_buf[idx] ;
          f = ((f - sample_ref) * apply_gain) + sample_ref ;
          dma_buf[idx] = (unsigned long) f ;
        }
      }

      sendto (sd, dma_buf, dma_bufsize, 0, (struct sockaddr*) &dest_addr,
              sizeof(dest_addr)) ;
      p->results[0].i_value++ ;
    }
    else
      p->results[1].i_value++ ;

    tv_end = millis() ;
    p->results[2].i_value = tv_end - tv_work ; // time for my logic in loop
    p->results[3].i_value = tv_end - tv_loop ; // time for entire loop
  }

  /* if we're here, we're shutting down, clean up resources */

  if (sd >= 0) close (sd) ;
  free (dma_buf) ;
  i2s_stop (MYI2S_INPUT_PORT) ;
  i2s_driver_uninstall (MYI2S_INPUT_PORT) ;

  strcpy (p->msg, "resources released") ; // indicate successful exit
}

/*
   This function initializes an I2S audio output and enters a main loop
   where it reads UDP packets and places them into a DMA buffer. This function
   lives in its main loop and does not return unless "p->state" is set to
   THREAD_WRAPUP.
*/

void ft_i2sout (S_thread_entry *p)
{
  if (p->num_args != 5)
  {
    strcpy (p->msg, "FATAL! Expecting 5x arguments") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  int sampleRate = atoi (p->in_args[0]) ;
  int SCKpin = atoi (p->in_args[1]) ;
  int WSpin = atoi (p->in_args[2]) ;
  int SDpin = atoi (p->in_args[3]) ;
  int udpPort = atoi (p->in_args[4]) ;
  int dma_bufsize = MYI2S_BITS_PER_SAMPLE / 8 * MYI2S_DMA_SAMPLES ;

  int sd = -1 ;                         // recv UDP socket descriptor
  unsigned long *dma_buf = NULL ;       // work buffer to recv UDP data
  double sample_ref = 0.0 ;             // the "zero" value for analog values
  esp_err_t e ;                         // generic error return value

  /* do one time I2S initialization (well, per thread instance) */

  if (p->loops == 0)
  {
    const i2s_config_t i2s_config = {
      .mode = i2s_mode_t (I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = (uint32_t) sampleRate,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = i2s_comm_format_t (I2S_COMM_FORMAT_I2S |
                                                 I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = 0,
      .dma_buf_count = MYI2S_DMA_OUT_BUFS,
      .dma_buf_len = MYI2S_DMA_SAMPLES
    } ;

    const i2s_pin_config_t pin_config = {
      .bck_io_num = SCKpin,
      .ws_io_num = WSpin,
      .data_out_num = SDpin,
      .data_in_num = I2S_PIN_NO_CHANGE
    } ;

    e = i2s_driver_install (MYI2S_OUTPUT_PORT, &i2s_config, 0, NULL) ;
    if (e != ESP_OK)
    {
      sprintf (p->msg, "i2s_driver_install() failed %d", e) ;
      p->state = THREAD_STOPPED ;
      return ;
    }

    e = i2s_set_pin (MYI2S_OUTPUT_PORT, &pin_config) ;
    if (e != ESP_OK)
    {
      sprintf (p->msg, "i2s_set_pin() failed %d", e) ;
      p->state = THREAD_STOPPED ;
      return ;
    }

    /* track UDP recv() metrics */

    p->results[0].num_tags = 1 ;
    p->results[0].meta[0] = (char*) "recv" ;
    p->results[0].data[0] = (char*) "\"ok\"" ;
    p->results[0].i_value = 0 ;         // recv() returns "dma_bufsize"

    p->results[1].num_tags = 1 ;
    p->results[1].meta[0] = (char*) "recv" ;
    p->results[1].data[0] = (char*) "\"short\"" ;
    p->results[1].i_value = 0 ;         // recv() returns < "dma_bufsize"

    p->results[2].num_tags = 1 ;
    p->results[2].meta[0] = (char*) "recv" ;
    p->results[2].data[0] = (char*) "\"idle\"" ;
    p->results[2].i_value = 0 ;         // select() timed out

    p->results[3].num_tags = 1 ;
    p->results[3].meta[0] = (char*) "dma_write" ;
    p->results[3].data[0] = (char*) "\"fails\"" ;
    p->results[3].i_value = 0 ;         // i2s_write() failed

    p->results[4].num_tags = 1 ;
    p->results[4].meta[0] = (char*) "signal" ;
    p->results[4].data[0] = (char*) "\"dynrange\"" ;
    p->results[4].i_value = 0 ;         // current "dynamic range" percentage

    p->num_int_results = 5 ;

    /* prepare a UDP socket and bind it to its listening port */

    sd = socket (AF_INET, SOCK_DGRAM, IPPROTO_IP) ;
    if (sd < 0)
    {
      strcpy (p->msg, "socket(SOCK_DRAM) failed") ;
      p->state = THREAD_STOPPED ;
      return ;
    }

    struct sockaddr_in addr ;
    memset (&addr, 0, sizeof(addr)) ;
    addr.sin_family = AF_INET ;
    addr.sin_addr.s_addr = INADDR_ANY ;
    addr.sin_port = htons (udpPort) ;
    if (bind (sd, (const struct sockaddr*) &addr, sizeof(addr)) < 0)
    {
      close (sd) ;
      i2s_stop (MYI2S_OUTPUT_PORT) ;
      i2s_driver_uninstall (MYI2S_OUTPUT_PORT) ;
      strcpy (p->msg, "bind() failed") ;
      p->state = THREAD_STOPPED ;
      return ;
    }

    /* allocate a buffer to copy UDP packet contents into */

    dma_buf = (unsigned long*) malloc (dma_bufsize) ;
    if (dma_buf == NULL)
    {
      sprintf (p->msg, "dma_buf malloc(%d) failed", dma_bufsize) ;
      close (sd) ;
      i2s_stop (MYI2S_OUTPUT_PORT) ;
      i2s_driver_uninstall (MYI2S_OUTPUT_PORT) ;
      p->state = THREAD_STOPPED ;
      return ;
    }
  }

  /*
     this is our main loop. it is very important that we do NOT block for
     too long, as we must check for a thread termination request. We want
     select() to only pause for a very short time. This is because during
     unusually long waits (ie, poor wifi), the I2S subsystem is still reading
     off our DMA buffers, and the audio playback will sound like noise. Thus
     select() waits for about half the time it takes to play through our DMA
     buffers

       packets_per_sec = sample_rate / samples_per_pkt
       usec_per_packet = 1000000 / packets_per_sec
  */

  int i, idx, usec_per_packet=0 ;
  unsigned long sample_lo, sample_hi ;
  unsigned long long total = 0 ;
  size_t written ;
  fd_set rfds ;
  struct timeval tv ;

  usec_per_packet = 1000000 / (sampleRate / MYI2S_DMA_SAMPLES) ;
  sprintf (p->msg, "%d usec/pkt", usec_per_packet) ;

  while (p->state == THREAD_RUNNING)
  {
    tv.tv_sec = 0 ;
    tv.tv_usec = usec_per_packet * (MYI2S_DMA_OUT_BUFS / 2) ;
    FD_ZERO (&rfds) ;
    FD_SET (sd, &rfds) ;
    int result = select (sd + 1, &rfds, NULL, NULL, &tv) ;
    if (result == 0)
    {
      /*
         If select() timed out, it means our audio stream got interrupted.
         It's very important that we zero out our DMA buffers otherwise the
         I2S subsystem continues to playback buffer contents, which for all
         practical purposes will sound like noise.
      */

      p->results[2].i_value++ ;
      for (i=0 ; i < MYI2S_DMA_OUT_BUFS ; i++)
      {
        memset (dma_buf, 0, dma_bufsize) ;
        i2s_write (MYI2S_OUTPUT_PORT, dma_buf, dma_bufsize, &written,
                   MYI2S_DMA_WAITTICKS) ;
      }
    }
    if ((result == 1) && (FD_ISSET (sd, &rfds)))
    {
      /*
         it's very important that we use the MSG_DONTWAIT flag here to
         prevent recv() from blocking. By right select() handles our blocking
         wait in a controlled manner. Not using MSG_DONTWAIT causes this
         thread to block excessively, which is bad.
      */

      size_t amt = recv (sd, dma_buf, dma_bufsize, MSG_DONTWAIT) ;
      if (amt < dma_bufsize)
        p->results[1].i_value++ ;       // recv() returned insufficient data
      if (amt == dma_bufsize)
      {
        /* inspect samples to find dynamic range and zero reference value */

        sample_lo = sample_hi = total = dma_buf[0] ;
        for (idx=1 ; idx < MYI2S_DMA_SAMPLES ; idx++)
        {
          if (dma_buf[idx] < sample_lo)
            sample_lo = dma_buf[idx] ;
          if (dma_buf[idx] > sample_hi)
            sample_hi = dma_buf[idx] ;
          total = total + dma_buf[idx] ;
        }
        long long ll = total / MYI2S_DMA_SAMPLES ;
        sample_ref = (double) (ll) ;                    // "zero" level

        /*
           dynamic range is an unsigned 32-bit number, but "i_value" is "int",
           so right shift bits by 1.
        */

        p->results[4].i_value = (int) ((sample_hi - sample_lo) >> 1) ;

        p->results[0].i_value++ ;       // recv() returned expected amount
        written = 0 ;
        e = i2s_write (MYI2S_OUTPUT_PORT, dma_buf, amt, &written,
                       MYI2S_DMA_WAITTICKS) ;
        if ((e != ESP_OK) || (written != amt))
          p->results[3].i_value++ ;
      }
    }
  }

  if (sd > 0) close (sd) ;
  free (dma_buf) ;
  i2s_stop (MYI2S_OUTPUT_PORT) ;
  i2s_driver_uninstall (MYI2S_OUTPUT_PORT) ;

  strcpy (p->msg, "resources released") ; // indicate successful exit
}

