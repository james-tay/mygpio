#define MAX_TASK_FILE_SIZE 4096 // maximum allowed file size

/*
   This thread executes various tasks from the user specified file. Each
   line in the file is a command to be executed. Note that this thread will
   execute each line in the file once. Once complete, this thread will
   terminate. This thread can be used to perform custom startup initialization
   by executing commands and then starting up other threads.

   Normally, commands are executed by "f_action()". However, the following
   built-in commands are supported,

     delay_ms <milliseconds>
*/

void ft_tasks (S_thread_entry *p)
{
  if (p->num_args != 1)
  {
    strcpy (p->msg, "FATAL! Expecting 1x argument") ;
    p->state = THREAD_STOPPED ;
    return ;
  }

  char *filename = p->in_args[0] ;
  File f = SPIFFS.open (filename, "r") ;
  if (f.size() >= MAX_TASK_FILE_SIZE)
    strcpy (p->msg, "FATAL! Task file is too large") ;
  else
  {
    char *buf = (char*) malloc (f.size() + 1) ;
    if (buf == NULL)
      strcpy (p->msg, "FATAL! Could not allocate buf for task file") ;
    else
    {
      int amt = f.readBytes(buf, MAX_TASK_FILE_SIZE) ;
      if (amt < 1)
      {
        strcpy (p->msg, "FATAL! No data read from task file") ;
      }
      else
      {
        /* At this point, try to split the file into individual lines */

        int total_commands = 0 ;
        char *c_idx = NULL ;
        char *cmd = strtok_r (buf, "\n", &c_idx) ;
        while (cmd)
        {
          total_commands++ ;

          /*
             at this point, we need to prepare an array of strings because
             "f_action(char **tokens)".
          */

          int i ;
          char *tokens[MAX_TOKENS] ;
          for (i=0 ; i < MAX_TOKENS ; i++)
            tokens[i] = NULL ;                  // initialize tokens array

          int num = 0 ;
          char *t_idx = NULL ;
          tokens[num] = strtok_r (cmd, " ", &t_idx) ;
          for (num=1 ; num < MAX_TOKENS ; num++)
          {
            tokens[num] = strtok_r (NULL, " ", &t_idx) ;
            if (tokens[num] == NULL)
              break ;
          }

          /* Check if user specified a built-in command */

          if (strcmp(tokens[0], "delay_ms") == 0)
            delay (atoi(tokens[1])) ;
          else
            f_action (tokens) ;

          /* move on to the next command */

          cmd = strtok_r (NULL, "\n", &c_idx) ;
        }
        sprintf (p->msg, "Executed %d commands", total_commands) ;
      }
      free (buf) ;
    }
  }
  f.close() ;

  p->state = THREAD_STOPPED ;
}

