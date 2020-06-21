/*
   Examples

     test LED,
       hi 12
       lo 12

     test light sensor,
       hi 5
       aread 0
       lo 5

     test HC-SR04,
       hi 4
       hcsr04 2 3
       lo 4

     test DHT22,
       hi 7
       dht22 6
       lo 7

     test BMP180,
       bmp180

     test build-in LED on NodeMCU
       lo 16
       hi 16
*/

#include <Wire.h>

#define MAX_LONG 2147483647
#define MAX_TOKENS 10
#define BUF_SIZE 80

char line[BUF_SIZE] ;           // general purpose string buffer
char *tokens[MAX_TOKENS+1] ;    // max command parameters we'll parse

/* ------------------------------------------------------------------------- */

/*
   Polls a BMP180 on the I2C bus. This should return temperature and pressure
   data which is then written into "temperature" and "pressure". On success
   we return 1, otherwise 0.
*/

int f_bmp180 (float *temperature, float *pressure)
{
  #define BMP180_ADDR 0x77 /* this came from the BMP180 data sheet */

  unsigned char data[2] ;

  Wire.beginTransmission (BMP180_ADDR) ;

  /* read the 11x 16-bit registers on the BMP180 for calibration data */

  short AC1, AC2, AC3, VB1, VB2, MB, MC, MD ;
  unsigned short AC4, AC5, AC6 ;

  Wire.beginTransmission (BMP180_ADDR) ;
  data[0] = 0xAA ;
  data[1] = 0 ;
  Wire.write(data[0]) ;
  int error = Wire.endTransmission() ;
  if (error == 0)
  {
    Wire.requestFrom (BMP180_ADDR, 2) ;
    data[0] = Wire.read() ;
    data[1] = Wire.read() ;

    sprintf (line, "AC1: %d %d.", data[0], data[1]) ;
    Serial.println (line) ;
  }
  else
  {
    sprintf (line, "FAULT: f_bmp180() xmit error %d.", error) ;
    Serial.println (line) ;
    return (0) ;
  }

  return (1) ;
}

/*
   Polls a DHT-22 and writes the current temperature and humidity in the
   supplied buffers. On success, it returns 1, otherwise 0.
*/

int f_dht22 (int dataPin, float *temperature, float *humidity)
{
  #define DHT22_TIMEOUT_USEC 500

  int cycles[40] ;
  unsigned char data[5] ;

  /* send the trigger to begin a poll cycle */

  pinMode (dataPin, OUTPUT) ;
  digitalWrite (dataPin, LOW) ;         // sensor reset
  delayMicroseconds (1200) ;
  digitalWrite (dataPin, HIGH) ;        // sensor trigger
  delayMicroseconds (20) ;
  pinMode (dataPin, INPUT) ;

  /* expect DHT22 to respond with a low and then a high */

  if (pulseIn (dataPin, HIGH) == 0)
  {
    Serial.println ("FAULT: f_dht22() no ACK, aborting.") ;
    return (0) ;
  }

  /* now read 40 bits of data, store pulse timings in an array */

  int i ;
  for (i=0 ; i<40 ; i++)
    cycles[i] = pulseIn (dataPin, HIGH, DHT22_TIMEOUT_USEC) ;

  /* convert pulse timings timings into humidity/temperature/checksum */

  memset (data, 0, 5) ;
  for (i=0 ; i<40 ; i++)
  {
    data[i/8] <<= 1 ;           // left shift bits, right most bit will be 0
    if (cycles[i] > 50)
      data[i/8] |= 1 ;          // set right most bit to 1
  }

  /* validate checksum */

  unsigned char c = data[0] + data[1] + data[2] + data[3] ;
  if ((c & 0xff) != data[4])
  {
    Serial.println ("FAULT: f_dht22() checksum failed.") ;
    return (0) ;
  }

  *humidity = float (((int) data[0] << 8 ) | data[1]) / 10.0 ;
  *temperature = float ((((int) data[2] & 0x7f ) << 8 ) | data[3]) / 10.0 ;
  return (1) ;
}

/*
   Returns the distance (in cm) measured by an HC-SR04 ultrasonic range sensor
   or -1.0 if it was unable to take a reading.
*/

float f_hcsr04 (int trigPin, int echoPin)
{
  #define HCSR04_TIMEOUT_USEC 60000

  pinMode (trigPin, OUTPUT) ;
  pinMode (echoPin, INPUT) ;

  /* set trigger pin low, then stay high for 10 usec */

  digitalWrite (trigPin, LOW) ;
  delayMicroseconds (1000) ;
  digitalWrite (trigPin, HIGH) ;
  delayMicroseconds (10) ;
  digitalWrite (trigPin, LOW) ;

  unsigned long echoUsecs = pulseIn (echoPin, HIGH, HCSR04_TIMEOUT_USEC) ;
  if (echoUsecs == 0)
  {
    Serial.println ("FAULT: f_hcsr04() no response.") ;
    return (-1.0) ;
  }
  else
    return (float(echoUsecs) / 58.0) ; // convert time to centimeters
}

/* ------------------------------------------------------------------------- */

void f_action (char **tokens)
{
  if ((strcmp(tokens[0], "?") == 0) || (strcmp(tokens[0], "help") == 0))
  {
    Serial.println ("hi <pin 0-13>") ;
    Serial.println ("lo <pin 0-13>") ;
    Serial.println ("aread <pin 0-5> - analog read") ;
    Serial.println ("bmp180") ;
    Serial.println ("dht22 <dataPin> - DHT-22 temperature/humidity sensor") ;
    Serial.println ("hcsr04 <trigPin> <echoPin> - HC-SR04 ultrasonic sensor") ;
    Serial.println ("uptime") ;
  }
  else
  if ((strcmp(tokens[0], "hi") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, OUTPUT) ;
    digitalWrite (pin, HIGH) ;
    sprintf (line, "pin:%d HIGH", pin) ;
    Serial.println (line) ;
  }
  else
  if ((strcmp(tokens[0], "lo") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    pinMode (pin, OUTPUT) ;
    digitalWrite (pin, LOW) ;
    sprintf (line, "pin:%d LOW", pin) ;
    Serial.println (line) ;
  }
  else
  if ((strcmp(tokens[0], "aread") == 0) && (tokens[1] != NULL))
  {
    int pin = atoi(tokens[1]) ;
    int val = analogRead (pin) ;
    sprintf (line, "analogRead pin:%d - %d", pin, val) ;
    Serial.println (line) ;
  }
  else
  if (strcmp(tokens[0], "bmp180") == 0)
  {
    float t=0.0, p=0.0 ;
    if (f_bmp180 (&t, &p))
    {
      sprintf (line, "bmp180 - temperature:%d.%02d pressure:%d.%02d",
               int(t), (int)(t*100)%100, int(p), (int)(p*100)%100) ;
      Serial.println (line) ;
    }
  }
  else
  if ((strcmp(tokens[0], "dht22") == 0) && (tokens[1] != NULL))
  {
    float t=0.0, h=0.0 ;
    if (f_dht22 (atoi(tokens[1]), &t, &h))
    {
      sprintf (line, "dht22 - temperature:%d.%02d humidity:%d.%02d",
               int(t), (int)(t*100)%100, int(h), (int)(h*100)%100) ;
      Serial.println (line) ;
    }
  }
  else
  if ((strcmp(tokens[0], "hcsr04") == 0) && 
      (tokens[1] != NULL) && (tokens[2] != NULL))
  {
    float f = f_hcsr04 (atoi(tokens[1]), atoi(tokens[2])) ;
    if (f > 0.0)
    {
      sprintf (line, "hcsr04 - %d.%02d cm", int(f), (int)(f*100)%100) ;
      Serial.println (line) ;
    }
  }
  else
  if (strcmp(tokens[0], "uptime") == 0)
  {
    unsigned long now = millis() / 1000 ;
    sprintf (line, "uptime - %ld secs", now) ;
    Serial.println (line) ;
  }
  else
  {
    Serial.println ("FAULT: Unknown command. Enter ? for help.") ;
  }
}

/* ------------------------------------------------------------------------- */

void setup ()
{
  Wire.begin () ;
  Serial.begin (9600) ;
  Serial.setTimeout (MAX_LONG) ; // Serial.read() to block as long as possible
  Serial.println ("Ready.") ;
}

void loop ()
{
  int idx=0 ;
  char *p ;

  /* At the start of our loop, print the prompt */

  Serial.println ("OK") ;

  /* wait for '\r' (minicom sends this when user presses ENTER) */

  int amt = Serial.readBytesUntil('\r', line, BUF_SIZE-1) ;
  line[amt] = 0 ;

  /* parse what we've received on the serial port */

  if (amt == 0)
  {
    Serial.print ("\r\n") ;
  }
  else
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
    if (tokens[0] != NULL)
      f_action (tokens) ;
  }
}

