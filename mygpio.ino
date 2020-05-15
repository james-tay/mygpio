#define MAX_LONG 2147483647
#define MAX_TOKENS 10
#define BUF_SIZE 80

char line[BUF_SIZE] ;
char *tokens[MAX_TOKENS] ;

/* ------------------------------------------------------------------------- */

void f_action (char **tokens)
{
  if ((strcmp(tokens[0], "hi") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, OUTPUT) ;
    digitalWrite (pin, HIGH) ;
    Serial.print ("f_action() pin HIGH ") ;
    Serial.print (pin) ;
    Serial.print ("\r\n") ;
  }
  if ((strcmp(tokens[0], "lo") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, OUTPUT) ;
    digitalWrite (pin, LOW) ;
    Serial.print ("f_action() pin LOW ") ;
    Serial.print (pin) ;
    Serial.print ("\r\n") ;
  }

}

/* ------------------------------------------------------------------------- */

void setup ()
{
  Serial.begin (9600) ;
  Serial.setTimeout (MAX_LONG) ;
  Serial.println ("Ready.") ;
}

void loop ()
{
  int idx=0 ;
  char *p ;

  /* At the start of our loop, print the prompt */

  unsigned long now = millis() / 1000 ;
  Serial.print (now) ;
  Serial.print ("> ") ;
  int amt = Serial.readBytesUntil('\r', line, BUF_SIZE-1) ;
  line[amt] = 0 ;
  Serial.print ("\r\n") ;
  Serial.print ("Received ") ;
  Serial.print (amt) ;
  Serial.print (" bytes.\r\n") ;

  /* parse what we've received on the serial port */

  if (amt > 0)
  {
    idx = 0 ;
    p = strtok (line, " ") ;
    while (p)
    {
      tokens[idx] = p ;
      idx++ ;
      p = strtok (NULL, " ") ;
    }
    tokens[idx] = NULL ;

    /* print what we've parsed */

    idx = 0 ;
    while (tokens[idx] != NULL)
    {
      Serial.print ("[") ;
      Serial.print (tokens[idx]) ;
      Serial.print ("]") ;
      idx++ ;
    }
    Serial.print ("\r\n") ;

    if (tokens[0] != NULL)
      f_action (tokens) ;
  }



}
