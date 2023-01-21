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

void f_cam_cmd (char **tokens)
{
  char line[BUF_SIZE] ;

  if (strcmp(tokens[1], "init") == 0)                           // init
  {
    if (G_cam_config != NULL)
    {
      strcat (G_reply_buf, "FAULT: camera already initialized.\r\n") ;
      return ;
    }
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
      G_cam_config->fb_count = 2 ;
      G_cam_config->fb_location = CAMERA_FB_IN_PSRAM ;
    }
    else
    {
      strcat (G_reply_buf, "FAULT: No PSRAM found.\r\n") ;
      return ;
    }

    esp_err_t err = esp_camera_init (G_cam_config) ;
    if (err)
      sprintf (line, "FAULT: esp_camera_init() failed 0x%x", err) ;
    else
      sprintf (line, "initialized psram free:%d size:%d bytes.\r\n",
               ESP.getFreePsram(), ESP.getPsramSize()) ;
    strcat (G_reply_buf, line) ;
    return ;
  }
}

void f_cam_img (S_WebClient *client)
{


}
