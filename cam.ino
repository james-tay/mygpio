/*
   References
     - https://github.com/espressif/esp32-camera/blob/master/conversions/to_jpg.cpp
     - https://github.com/espressif/esp32-camera/blob/master/driver/include/sensor.h
     - https://github.com/espressif/esp32-camera/blob/master/driver/include/esp_camera.h
*/

/* camera pin configuration */

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27
#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

/* jpeg quality */
#define CAM_JPEG_QUALITY 90

/*
   This function is called from f_action(). Our job is to parse the "cam"
   command's sub-commands and act on them.
*/

void f_cam_cmd (char **tokens)
{
  char line[BUF_SIZE] ;

  if (strcmp(tokens[1], "init") == 0)                   // init
  {
    if (G_cam_config != NULL)
    {
      strcat (G_reply_buf, "FAULT: camera already initialized.\r\n") ;
      return ;
    }

    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

    G_cam_config = (camera_config_t*) malloc (sizeof(camera_config_t)) ;
    memset (G_cam_config, 0, sizeof(camera_config_t)) ;

    G_cam_config->pin_pwdn = CAM_PIN_PWDN ;
    G_cam_config->pin_reset = CAM_PIN_RESET ;
    G_cam_config->pin_xclk = CAM_PIN_XCLK ;
    G_cam_config->pin_d7 = CAM_PIN_D7 ;
    G_cam_config->pin_d6 = CAM_PIN_D6 ;
    G_cam_config->pin_d5 = CAM_PIN_D5 ;
    G_cam_config->pin_d4 = CAM_PIN_D4 ;
    G_cam_config->pin_d3 = CAM_PIN_D3 ;
    G_cam_config->pin_d2 = CAM_PIN_D2 ;
    G_cam_config->pin_d1 = CAM_PIN_D1 ;
    G_cam_config->pin_d0 = CAM_PIN_D0 ;
    G_cam_config->pin_vsync = CAM_PIN_VSYNC ;
    G_cam_config->pin_href = CAM_PIN_HREF ;
    G_cam_config->pin_sscb_sda = CAM_PIN_SIOD ;
    G_cam_config->pin_sscb_scl = CAM_PIN_SIOC ;
    G_cam_config->pin_pclk = CAM_PIN_PCLK ;

    G_cam_config->xclk_freq_hz = 20000000 ;
    G_cam_config->ledc_timer = LEDC_TIMER_0 ;
    G_cam_config->ledc_channel = LEDC_CHANNEL_0 ;

    /* various other formats : YUV422,GRAYSCALE,RGB565,JPEG */

    G_cam_config->pixel_format = PIXFORMAT_JPEG ;
    G_cam_config->grab_mode = CAMERA_GRAB_WHEN_EMPTY ;
    G_cam_config->fb_location = CAMERA_FB_IN_DRAM ;

    // UXGA  1600x1200
    // SXGA  1280x1024
    // HD    1280x720
    // XGA   1024x768
    // SVGA  800x600
    // VGA   640x480

    G_cam_config->frame_size = FRAMESIZE_SVGA ;
    G_cam_config->jpeg_quality = 10;
    G_cam_config->fb_count = 1;

    if (psramFound())
    {
      G_cam_config->frame_size = FRAMESIZE_SXGA ;
      G_cam_config->fb_count = 1 ;
      G_cam_config->fb_location = CAMERA_FB_IN_PSRAM ;
    }
    else
    {
      strcat (G_reply_buf, "FAULT: No PSRAM found.\r\n") ;
      return ;
    }

    esp_err_t err = esp_camera_init (G_cam_config) ;
    if (err)
      sprintf (line, "FAULT: esp_camera_init() failed 0x%x.\r\n", err) ;
    else
      sprintf (line, "initialized psram free:%d size:%d bytes.\r\n",
               ESP.getFreePsram(), ESP.getPsramSize()) ;
    strcat (G_reply_buf, line) ;
    return ;
  }

  if (strcmp(tokens[1], "set") == 0)                    // set <param> <value>
  {
    if ((tokens[2] == NULL) || (tokens[3] == NULL))
    {
      strcat (G_reply_buf, "FAULT: missing arguments.\r\n") ;
      return ;
    }
    char *key = tokens[2] ;
    int v = atoi(tokens[3]) ;
    sensor_t *s = esp_camera_sensor_get () ;

    if (strcmp(key, "contrast") == 0)      s->set_contrast(s, v) ;
    if (strcmp(key, "brightness") == 0)    s->set_brightness(s, v) ;
    if (strcmp(key, "saturation") == 0)    s->set_saturation(s, v) ;
    if (strcmp(key, "sharpness") == 0)     s->set_sharpness(s, v) ;
    if (strcmp(key, "denoise") == 0)       s->set_denoise(s, v) ;
    if (strcmp(key, "quality") == 0)       s->set_quality(s, v) ;
    if (strcmp(key, "colorbar") == 0)      s->set_colorbar(s, v) ;
    if (strcmp(key, "whitebal") == 0)      s->set_whitebal(s, v) ;
    if (strcmp(key, "gain_ctrl") == 0)     s->set_gain_ctrl(s, v) ;
    if (strcmp(key, "exposure_ctrl") == 0) s->set_exposure_ctrl(s, v) ;
    if (strcmp(key, "hmirror") == 0)       s->set_hmirror(s, v) ;
    if (strcmp(key, "vflip") == 0)         s->set_vflip(s, v) ;
    if (strcmp(key, "aec2") == 0)          s->set_aec2(s, v) ;
    if (strcmp(key, "awb_gain") == 0)      s->set_awb_gain(s, v) ;
    if (strcmp(key, "agc_gain") == 0)      s->set_agc_gain(s, v) ;
    if (strcmp(key, "aec_value") == 0)     s->set_aec_value(s, v) ;

    /* framesize is specified as a string */

    if (strcmp(key, "framesize") == 0)
    {
      if (strcmp(tokens[3], "uxga") == 0)
        s->set_framesize(s, FRAMESIZE_UXGA) ;   // 1600x1200
      if (strcmp(tokens[3], "sxga") == 0)
        s->set_framesize(s, FRAMESIZE_SXGA) ;   // 1280x1024
      if (strcmp(tokens[3], "hd") == 0)
        s->set_framesize(s, FRAMESIZE_HD) ;     // 1280x720
      if (strcmp(tokens[3], "xga") == 0)
        s->set_framesize(s, FRAMESIZE_XGA) ;    // 1024x768
      if (strcmp(tokens[3], "svga") == 0)
        s->set_framesize(s, FRAMESIZE_SVGA) ;   // 800x600
      if (strcmp(tokens[3], "vga") == 0)
        s->set_framesize(s, FRAMESIZE_VGA) ;    // 640x480
    }

    sprintf (G_reply_buf, "setting %s -> %s.\r\n", key, tokens[3]) ;
  }
}

/*
   This function is called when a web client has called us with the "/cam"
   URI. Our job is to attempt to capture a frame and send the jpeg data over.
*/

void f_cam_img (S_WebClient *client)
{
  char line[BUF_SIZE] ;

  /* try grab a single camera frame first */

  camera_fb_t *fb = esp_camera_fb_get () ;
  if (fb == NULL)
  {
    strcpy (line, "HTTP/1.1 503 Unavailable\n") ;
    write (client->sd, line, strlen(line)) ;
    strcpy (line, "Connection: close\n\n") ;
    write (client->sd, line, strlen(line)) ;
    strcpy (line, "FAULT: esp_camera_fb_get() failed.\n") ;
    write (client->sd, line, strlen(line)) ;
    return ;
  }

  /* make sure the frame we just grabbed is a jpeg */

  size_t jpg_len = fb->len ;
  uint8_t *jpg_buf = fb->buf ;

  if ((fb->format != PIXFORMAT_JPEG) &&
      (frame2jpg (fb, CAM_JPEG_QUALITY, &jpg_buf, &jpg_len) == false))
  {
    strcpy (line, "HTTP/1.1 503 Unavailable\n") ;
    write (client->sd, line, strlen(line)) ;
    strcpy (line, "Connection: close\n\n") ;
    write (client->sd, line, strlen(line)) ;
    strcpy (line, "FAULT: frame2jpg() failed.\n") ;
    write (client->sd, line, strlen(line)) ;
    esp_camera_fb_return (fb) ;
    return ;
  }

  /* set the HTTP header and send the jpeg data */

  strcpy (line, "HTTP/1.1 200 OK\n") ;
  write (client->sd, line, strlen(line)) ;
  strcpy (line, "Accept-Ranges: bytes\n") ;
  write (client->sd, line, strlen(line)) ;
  strcpy (line, "Cache-Control: no-cache\n") ;
  write (client->sd, line, strlen(line)) ;
  strcpy (line, "Content-Type: image/jpeg\n") ;
  write (client->sd, line, strlen(line)) ;
  sprintf (line, "Content-Length: %d\n\n", jpg_len) ;
  write (client->sd, line, strlen(line)) ;

  int written = 0 ;
  while (written != jpg_len)
  {
    int remainder = jpg_len - written ;
    int amt = write (client->sd, jpg_buf + written, remainder) ;
    if (amt < 1)
      break ;
    else
      written = written + amt ;
  }
  esp_camera_fb_return (fb) ;
}

