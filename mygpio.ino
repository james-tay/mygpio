#define MAX_LONG 2147483647
#define MAX_TOKENS 10
#define BUF_SIZE 80

char line[BUF_SIZE] ;
char *tokens[MAX_TOKENS+1] ;

/* ------------------------------------------------------------------------- */

/*
   Returns the distance (in cm) measured by an HC-SR04 ultrasonic range sensor
   or -1.0 if it was unable to take a reading.
*/

float f_hcsr04 (int trigPin, int echoPin)
{
  #define TIMEOUT_USEC 60000

  pinMode (trigPin, OUTPUT) ;
  pinMode (echoPin, INPUT) ;

  /* set trigger pin low, then stay high for 10 usec */

  digitalWrite (trigPin, LOW) ;
  delayMicroseconds (1000) ;
  digitalWrite (trigPin, HIGH) ;
  delayMicroseconds (10) ;
  digitalWrite (trigPin, LOW) ;

  unsigned long echoUsecs = pulseIn (echoPin, HIGH, TIMEOUT_USEC) ;
  if (echoUsecs == 0)
    return (-1.0) ;
  else
    return (float(echoUsecs) / 58.0) ;
}

/* ------------------------------------------------------------------------- */

void f_action (char **tokens)
{
  if ((strcmp(tokens[0], "?") == 0) || (strcmp(tokens[0], "help") == 0))
  {
    Serial.println ("hi <pin 0-13>") ;
    Serial.println ("lo <pin 0-13>") ;
    Serial.println ("aread <pin 0-5> - analog read") ;
    Serial.println ("hcsr04 <trigPin> <echoPin> - HC-SR04 ultrasonic sensor") ;
  }

  if ((strcmp(tokens[0], "hi") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, OUTPUT) ;
    digitalWrite (pin, HIGH) ;
    Serial.print ("f_action() pin HIGH ") ;
    Serial.println (pin) ;
  }

  if ((strcmp(tokens[0], "lo") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, OUTPUT) ;
    digitalWrite (pin, LOW) ;
    Serial.print ("f_action() pin LOW ") ;
    Serial.println (pin) ;
  }

  if ((strcmp(tokens[0], "aread") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    int val = analogRead (pin) ;
    Serial.print ("f_action() analogRead pin:") ;
    Serial.print (pin) ;
    Serial.print (" value:") ;
    Serial.println (val) ;
  }

  if ((strcmp(tokens[0], "hcsr04") == 0) && 
      (tokens[1] != NULL) && (tokens[2] != NULL))
  {
    float rangeCm = f_hcsr04 (atoi(tokens[1]), atoi(tokens[2])) ;
    Serial.print ("f_action() hcsr04 ") ;
    Serial.print (rangeCm) ;
    Serial.println (" cm") ;
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
    while ((p) && (idx < MAX_TOKENS))
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
