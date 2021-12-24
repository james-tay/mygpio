void f_fs (char **tokens)
{
  char msg[BUF_SIZE], line[BUF_SIZE] ;

  if (strcmp(tokens[1], "info") == 0)                           // info
  {
    sprintf (line, "totalBytes: %d\r\nusedBytes: %d\r\n",
             SPIFFS.totalBytes(), SPIFFS.usedBytes()) ;
    strcat (G_reply_buf, line) ;
  }
  else
  if (strcmp(tokens[1], "format") == 0)                         // format
  {
    if (SPIFFS.format())
    {
      strcat (G_reply_buf, "Success.\r\n") ;
    }
    else
    {
      strcat (G_reply_buf, "FAULT: Formatting failed.\r\n") ;
    }
  }
  else
  if (strcmp(tokens[1], "ls") == 0)                             // ls
  {
    File root = SPIFFS.open ("/", "r") ;
    File f = root.openNextFile () ;
    while (f)
    {
      sprintf (line, "%-8d %s\r\n", f.size(), f.name()) ;
      strcat (G_reply_buf, line) ;
      f = root.openNextFile () ;
    }
    root.close () ;
  }
  else
  if ((strcmp(tokens[1], "write") == 0) &&                      // write
      (tokens[2] != NULL) && (tokens[3] != NULL))
  {
    char *filename = tokens[2] ;
    char *content = tokens[3] ;

    /* force filenames to begin with '/' */

    if ((filename[0] != '/') || (strlen(filename) == 1))
    {
      strcat (G_reply_buf, "FAULT: Invalid filename.\r\n") ;
      return ;
    }

    File f = SPIFFS.open (filename, "w") ;
    if (f)
    {
      int amt = f.print (content) ;
      f.close () ;
      sprintf (msg, "Wrote %d bytes to '%s'.\r\n", amt, filename) ;
      strcat (G_reply_buf, msg) ;
    }
    else
    {
      sprintf (msg, "FAULT: Cannot write to '%s'.\r\n", filename) ;
      strcat (G_reply_buf, msg) ;
    }
  }
  else
  if ((strcmp(tokens[1], "read") == 0) && (tokens[2] != NULL))  // read
  {
    char *filename = tokens[2] ;
    File f = SPIFFS.open (filename, "r") ;
    int amt = f.readBytes (msg, BUF_SIZE-1) ;
    f.close () ;
    if (amt > 0)
    {
      msg[amt] = 0 ;
      strcat (G_reply_buf, msg) ;
      strcat (G_reply_buf, "\r\n") ;
    }
    else
    {
      strcat (G_reply_buf, "FAULT: Cannot read file.\r\n") ;
    }
  }
  else
  if ((strcmp(tokens[1], "rm") == 0) &&                         // rm
      (tokens[2] != NULL))
  {
    char *filename = tokens[2] ;
    if (SPIFFS.remove(filename))
      strcat (G_reply_buf, "File removed.\r\n") ;
    else
      strcat (G_reply_buf, "FAULT: Cannot remove file.\r\n") ;
  }
  else
  if ((strcmp(tokens[1], "rename") == 0) &&                     // rename
      (tokens[2] != NULL) && (tokens[3] != NULL))
  {
    char *old_name = tokens[2] ;
    char *new_name = tokens[3] ;

    if ((new_name[0] != '/') || (strlen(new_name) == 1))
    {
      strcat (G_reply_buf, "FAULT: Invalid filename.\r\n") ;
      return ;
    }
    if (SPIFFS.rename(old_name, new_name))
      strcat (G_reply_buf, "File renamed.\r\n") ;
    else
      strcat (G_reply_buf, "FAULT: Cannot rename file.\r\n") ;
  }
  else
  {
    strcat (G_reply_buf, "FAULT: Invalid argument.\r\n") ;
  }
}

void f_file (char **tokens)
{
  #define CLIENT_IO_TIMEOUT 10 // max idle(secs) for connect or data transfer

  if (((strcmp(tokens[1], "recv") != 0) && (strcmp(tokens[1], "send") != 0)) ||
      (tokens[2] == NULL) || (tokens[3] == NULL))
  {
    strcat (G_reply_buf, "FAULT: Invalid argument.\r\n") ;
    return ;
  }

  /* setup a listening TCP socket and expect a client to connect to it */

  int port = atoi (tokens[2]) ;
  char *path = tokens[3] ;
  int listen_sd = socket (AF_INET, SOCK_STREAM, IPPROTO_IP) ;
  if (listen_sd < 0)
  {
    strcat (G_reply_buf, "FATAL! socket() failed.\r\n") ;
    return ;
  }

  int reuse = 1 ; // allow quick re-bind()'ing of this port
  setsockopt (listen_sd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(int)) ;

  struct sockaddr_in addr ;
  memset (&addr, 0, sizeof(addr)) ;
  addr.sin_family = AF_INET ;
  addr.sin_addr.s_addr = INADDR_ANY ;
  addr.sin_port = htons (port) ;
  if (bind (listen_sd, (const struct sockaddr*) &addr, sizeof(addr)) < 0)
  {
    close (listen_sd) ;
    strcat (G_reply_buf, "FATAL! bind() failed.\r\n") ;
    return ;
  }
  if (listen (listen_sd, 1) != 0)
  {
    close (listen_sd) ;
    strcat (G_reply_buf, "FATAL! listen() failed.\r\n") ;
    return ;
  }

  struct timeval tv ;
  fd_set rfds ;
  tv.tv_sec = CLIENT_IO_TIMEOUT ;
  tv.tv_usec = 0 ;
  FD_ZERO (&rfds) ;
  FD_SET (listen_sd, &rfds) ;
  int result = select (listen_sd + 1, &rfds, NULL, NULL, &tv) ;
  if (result < 1)
  {
    close (listen_sd) ;
    strcat (G_reply_buf, "FATAL! No client connected.\r\n") ;
    return ;
  }
  int client_sd = accept (listen_sd, NULL, NULL) ;
  if (client_sd < 0)
  {
    close (listen_sd) ;
    strcat (G_reply_buf, "FATAL! accept() failed.\r\n") ;
    return ;
  }

  /*
     at this point, "client_sd" is connected, decide whether to send/recv,
     but *some* IO must occur within CLIENT_IO_TIMEOUT, or we'll close and
     abort the transfer.
  */

  int amt, total=0 ;
  char buf[BUF_MEDIUM] ;
  long start_time = millis () ;

  if (strcmp (tokens[1], "recv") == 0)                          // recv a file
  {
    File f = SPIFFS.open (path, "w") ;
    if (!f)
      strcat (G_reply_buf, "FATAL! Cannot write to file.\r\n") ;

    while (f)
    {
      tv.tv_sec = CLIENT_IO_TIMEOUT ;
      tv.tv_usec = 0 ;
      FD_ZERO (&rfds) ;
      FD_SET (client_sd, &rfds) ;
      result = select (client_sd + 1, &rfds, NULL, NULL, &tv) ;
      if (result < 1)
      {
        sprintf (buf, "FATAL! client timeout after %d bytes.\r\n", total) ;
        strcat (G_reply_buf, buf) ;
        break ;
      }
      amt = read (client_sd, buf, BUF_MEDIUM-1) ;
      if (amt < 1)
      {
        long duration_ms = millis() - start_time ;
        sprintf (buf, "Received %d bytes from client in %d ms.\r\n",
                total, duration_ms) ;
        strcat (G_reply_buf, buf) ;
        break ;
      }
      buf[amt] = 0 ;
      f.print (buf) ;
      total = total + amt ;
    }
    if (f)
      f.close () ;
  }

  if (strcmp (tokens[1], "send") == 0)                          // send file
  {
    File f = SPIFFS.open (path, "r") ;
    if (f)
    {
      while (total < f.size())
      {
        amt = f.readBytes (buf, BUF_SIZE) ; // this blocks if we past EOF !!
        if (amt > 0)
        {
          write (client_sd, buf, amt) ;
          total = total + amt ;
        }
        else
          break ;
      }
      long duration_ms = millis () - start_time ;
      sprintf (buf, "Sent %d bytes to client in %d ms.\r\n",
               total, duration_ms) ;
      strcat (G_reply_buf, buf) ;
      f.close () ;
    }
    else
      strcat (G_reply_buf, "FATAL! Cannot read from file.\r\n") ;
  }

  close (listen_sd) ;
  close (client_sd) ;
}

