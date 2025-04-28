/* ------------------------------------------------------------------------- */
/* "Drivers" for various sensor devices                                      */
/* ------------------------------------------------------------------------- */

/*
   Polls a BMP180 on the I2C bus. This should return temperature and pressure
   data which is then written into "temperature" and "pressure". On success
   we return 1, otherwise 0.
*/

int f_bmp180 (float *temperature, float *pressure)
{
  #define BMP180_ADDR 0x77      /* this came from the BMP180 data sheet */
  #define BMP180_MODE 3         /* the pressure oversampling */

  /* read the 11x 16-bit registers on the BMP180 for calibration data */

  short ac1, ac2, ac3, b1, b2, mb, mc, md ;
  unsigned short ac4, ac5, ac6 ;

  if ((f_i2c_readShort (BMP180_ADDR, 0xAA, &ac1) == 0) ||
      (f_i2c_readShort (BMP180_ADDR, 0xAC, &ac2) == 0) ||
      (f_i2c_readShort (BMP180_ADDR, 0xAE, &ac3) == 0) ||
      (f_i2c_readUShort (BMP180_ADDR, 0xB0, &ac4) == 0) ||
      (f_i2c_readUShort (BMP180_ADDR, 0xB2, &ac5) == 0) ||
      (f_i2c_readUShort (BMP180_ADDR, 0xB4, &ac6) == 0) ||
      (f_i2c_readShort (BMP180_ADDR, 0xB6, &b1) == 0) ||
      (f_i2c_readShort (BMP180_ADDR, 0xB8, &b2) == 0) ||
      (f_i2c_readShort (BMP180_ADDR, 0xBA, &mb) == 0) ||
      (f_i2c_readShort (BMP180_ADDR, 0xBC, &mc) == 0) ||
      (f_i2c_readShort (BMP180_ADDR, 0xBE, &md) == 0))
  {
    strcat (G_reply_buf, "FAULT: Cannot read data from BMP180.\r\n") ;
    return (0) ;
  }

  char line[BUF_SIZE] ;

  sprintf (line, "ac1: %d\r\nac2: %d\r\nac3: %d\r\n", ac1, ac2, ac3) ;
  strcat (G_reply_buf, line) ;
  sprintf (line, "ac4: %d\r\nac5: %d\r\nac6: %d\r\n", ac4, ac5, ac6) ;
  strcat (G_reply_buf, line) ;
  sprintf (line, "b1: %d\r\nb2: %d\r\nmb: %d\r\nmc: %d\r\n", b2, mb, mc) ;
  strcat (G_reply_buf, line) ;
  sprintf (line, "md: %d\r\n", md) ;
  strcat (G_reply_buf, line) ;

  /*
     read the raw temperature by writing 0x2E to address 0xF4, the result is
     2 bytes at 0xF6.
  */

  Wire.beginTransmission (BMP180_ADDR) ;
  Wire.write ((byte)(0xF4)) ;
  Wire.write ((byte)(0x2E)) ;
  Wire.endTransmission () ;
  delay (5) ;                           // datasheet says wait 4.5 ms
  short raw_t=0 ;
  f_i2c_readShort (BMP180_ADDR, 0xF6, &raw_t) ;

  sprintf (line, "raw_t: %d\r\n", raw_t) ;
  strcat (G_reply_buf, line) ;

  /* now calculate the true temperature */

  long x1 = (raw_t - ac6) * ac5 >> 15 ;
  long x2 = (mc << 11) / (x1 + md) ;
  long b5 = x1 + x2 ;
  long t = (b5 + 8) >> 4 ;
  *temperature = float(t) / 10.0 ;

  /*
     read raw pressure by writing "0x34+(mode<<6)" to address 0xF4, the
     result is 3 bytes (MSB, LSB, XLSB) starting at 0xF6.
  */

  long v = 0x34 + (BMP180_MODE << 6) ;
  Wire.beginTransmission (BMP180_ADDR) ;
  Wire.write ((byte)(0xF4)) ;
  Wire.write ((byte)(v)) ;
  Wire.endTransmission () ;
  delay (26) ;                          // datasheet says 25.5 ms for mode 3.

  Wire.beginTransmission (BMP180_ADDR) ;
  Wire.write ((byte)(0xF6)) ;
  Wire.endTransmission () ;
  Wire.requestFrom (BMP180_ADDR, 3) ;
  long msb = Wire.read () ;
  long lsb = Wire.read () ;
  long xlsb = Wire.read () ;
  long raw_p = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - BMP180_MODE) ;

  sprintf (line, "raw_p: %d\r\n", raw_p) ;
  strcat (G_reply_buf, line) ;

  long b6 = b5 - 4000 ;
  x1 = (b2 * (b6 * b6) >> 12) >> 11 ;
  x2 = (ac2 * b6) >> 11 ;
  long x3 = x1 + x2 ;
  long b3 = (((ac1 * 4 + x3) << BMP180_MODE) + 2) / 4 ;
  x1 = ac3 * b6 >> 13 ;
  x2 = ((b1 * (b6 * b6)) >> 12) >> 16 ;
  x3 = ((x1 + x2) + 2) >> 2 ;
  unsigned long b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15 ;
  unsigned long b7 = (raw_p - b3) * (50000 >> BMP180_MODE) ;
  long p ;
  if (b7 < (unsigned long) 0x80000000)
    p = (b7 * 2) / b4 ;
  else
    p = (b7 / b4) * 2 ;
  x1 = (p >> 8) * (p >> 8) ;
  x1 = (x1 * 3038) >> 16 ;
  x2 = (-7357 * p) >> 16 ;
  p = p + ((x1 + x2 + 3791) >> 4) ;
  *pressure = float(p) / 100.0 ;

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
  digitalWrite (dataPin, LOW) ;         // set low before becoming an input pin
  pinMode (dataPin, INPUT) ;

  /* expect DHT22 to respond with a low and then a high */

  if (pulseIn (dataPin, HIGH) == 0)
  {
    strcat (G_reply_buf, "FAULT: f_dht22() no ACK, aborting.\r\n") ;
    return (0) ;
  }

  /* now read 40 bits of data, store pulse timings in an array */

  int i ;
  for (i=0 ; i<40 ; i++)
    cycles[i] = pulseIn (dataPin, HIGH, DHT22_TIMEOUT_USEC) ;

  /* convert pulse timings timings into humidity/temperature/checksum */

  char buf[BUF_SIZE] ;
  memset (data, 0, 5) ;
  for (i=0 ; i<40 ; i++)
  {
    data[i/8] <<= 1 ;           // left shift bits, right most bit will be 0
    if (cycles[i] > 50)
    {
      if (G_debug)
      {
        snprintf (buf, BUF_SIZE, "DEBUG: bit:%d cycle:%d = 1", i, cycles[i]) ;
        Serial.println (buf) ;
      }
      data[i/8] |= 1 ;          // set right most bit to 1
    }
    else
    {
      if (G_debug)
      {
        snprintf (buf, BUF_SIZE, "DEBUG: bit:%d cycle:%d = 0", i, cycles[i]) ;
        Serial.println (buf) ;
      }
    }
  }

  /* validate checksum */

  unsigned char c = data[0] + data[1] + data[2] + data[3] ;
  if ((c & 0xff) != data[4])
  {
    if (G_debug)
    {
      snprintf (buf, BUF_SIZE, "DEBUG: bad checksum 0x%x, expecting 0x%x.",
                (int) c, (int) data[4]) ;
      Serial.println (buf) ;
    }
    strcat (G_reply_buf, "FAULT: f_dht22() checksum failed.\r\n") ;
    return (0) ;
  }

  float raw_temp = ((word)(data[2] & 0x7F)) << 8 | data[3] ;
  if (data[2] & 0x80)
    *temperature = raw_temp * -0.1 ;
  else
    *temperature = raw_temp * 0.1 ;

  *humidity = float (((int) data[0] << 8 ) | data[1]) / 10.0 ;
  return (1) ;
}

/*
   Polls DS18B20 devices on the specified 1-wire pin. Writes the current
   temperature in the supplied float buffer array and the device addresses in
   "addr" which must be (MAX_THREAD_RESULT_VALUES * 8) bytes long. Returns
   the number of DS18B20 devices successfully polled (ie, written into the
   supplied arrays)
*/

int f_ds18b20 (int dataPin, unsigned char *addr, float *temperature)
{
  #define DS18B20_RESOLUTION_BITS 12
  #define DS18B20_MAX_TEMP 85.0         // a reading this high is not normal
  #define DS18B20_MIN_TEMP -127.0       // a reading this low is not normal

  OneWire bus (dataPin) ;
  DallasTemperature sensor (&bus) ;

  sensor.begin () ;
  int devices = sensor.getDeviceCount() ;
  if (devices < 1)
  {
    strcat (G_reply_buf, "FAULT: no devices found.\r\n") ;
    return (0) ;
  }
  if (devices > MAX_DS18B20_DEVICES)
    devices = MAX_DS18B20_DEVICES ;

  int addr_offset = 0 ; // where in "addr" we're currently writing into
  int results = 0 ;     // number of successful addresses actually read
  unsigned char cur_addr[8] ;

  sensor.setResolution (DS18B20_RESOLUTION_BITS) ;
  sensor.requestTemperatures() ;
  for (int i=0 ; i < devices ; i++)
  {
    if (sensor.getAddress (cur_addr, i))
    {
      float f = sensor.getTempCByIndex (i) ;
      if ((f < DS18B20_MAX_TEMP) && (f > DS18B20_MIN_TEMP)) // sanity check
      {
        temperature[results] = f ;
        memcpy (addr+addr_offset, cur_addr, 8) ;
        addr_offset = addr_offset + 8 ;
        results++ ;
      }
    }
  }
  return (results) ;
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
    return (-1.0) ;
  else
  {
    float cm = float(echoUsecs) / 58.0 ;        // convert time to centimeters
    if (cm < 1.0)
      return (-1.0) ; // this is probably an invalid reading
    return (cm) ;
  }
}

/*
   "tokens" is a string array of 6x params :
     <don't_care> <x_pin> <y_pin> <z_pin> <total_dur> <interval>

   "i_results" is an array of 10x ints which store the number of samples
   examined, folled by min/ave/max values of the X/Y/Z axis respectively.
*/

void f_adxl335 (char **tokens, int *i_results)
{
  #define MAX_DURATION 60000 // take readings for a maximum of 60 secs

  int x_value, y_value, z_value ;
  int x_min, y_min, z_min ;
  int x_max, y_max, z_max ;

  int samples = 0 ;
  int x_total = 0, y_total = 0, z_total =0 ;

  int x_pin = atoi (tokens[1]) ;
  int y_pin = atoi (tokens[2]) ;
  int z_pin = atoi (tokens[3]) ;
  unsigned long total_duration = atoi (tokens[4]) ;
  unsigned long interval = atoi (tokens[5]) ;

  /* sanity check some parameters first */

  if (total_duration > MAX_DURATION) total_duration = MAX_DURATION ;
  if (total_duration < 1) total_duration = 1 ;
  if (interval < 1) interval = 1 ;

  pinMode (x_pin, INPUT) ;
  pinMode (y_pin, INPUT) ;
  pinMode (z_pin, INPUT) ;

  unsigned long start_time = millis () ;
  unsigned long end_time = start_time + total_duration ;

  while (start_time < end_time)
  {
    x_value = analogRead (x_pin) ;
    y_value = analogRead (y_pin) ;
    z_value = analogRead (z_pin) ;

    if (samples == 0)
    {
      x_min = x_max = x_value ;
      y_min = y_max = y_value ;
      z_min = z_max = z_value ;
    }
    else
    {
      if (x_value < x_min) x_min = x_value ;
      if (y_value < y_min) y_min = y_value ;
      if (z_value < z_min) z_min = z_value ;

      if (x_value > x_max) x_max = x_value ;
      if (y_value > y_max) y_max = y_value ;
      if (z_value > z_max) z_max = z_value ;
    }
    x_total = x_total + x_value ;
    y_total = y_total + y_value ;
    z_total = z_total + z_value ;

    if (interval == 0)
      start_time = millis () ;
    else
    {
      start_time = start_time + interval ;
      int nap = start_time - millis() ;
      if (nap > 0)
        delay (nap) ;
    }
    samples++ ;
  }

  /* now write our results into the int array */

  i_results[0] = samples ;
  i_results[1] = x_min ;
  i_results[2] = x_total / samples ;
  i_results[3] = x_max ;
  i_results[4] = y_min ;
  i_results[5] = y_total / samples ;
  i_results[6] = y_max ;
  i_results[7] = z_min ;
  i_results[8] = z_total / samples ;
  i_results[9] = z_max ;
}

