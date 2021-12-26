/* mygpio.h */

#include <soc/rtc.h>
#include <lwip/sockets.h>

#include <FS.h>
#include <Wire.h>                     // this is for I2C support
#include <WiFi.h>
#include <SPIFFS.h>
#include <Update.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <soc/i2s_reg.h>
#include <driver/i2s.h>

#include <OneWire.h>                  // 1-wire support, typically 16.3 kbps
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

#define MAX_SSID_LEN 32
#define MAX_PASSWD_LEN 32             // maximum wifi password length
#define MAX_MQTT_LEN 128              // maximum mqtt topic/message buffer
#define MAX_WIFI_TIMEOUT 60           // wifi connect timeout (secs)
#define MAX_SD_BACKLOG 4              // TCP listening socket backlog
#define MAX_FILENAME 16               // maximum filename length
#define WEB_PORT 80                   // web server listens on this port
#define CRON_INTERVAL 60              // how often we run f_cron()
#define RSSI_LOW_THRES -72            // less than this is consider low signal

#define WIFI_SSID_FILE "/wifi.ssid"
#define WIFI_PW_FILE "/wifi.pw"
#define MQTT_CFG_FILE "/mqtt.cfg"     // used in f_mqtt_connect()
#define MQTT_PUB_FILE "/mqtt.pub"     // loaded into "G_mqtt_pub"
#define MQTT_SUB_FILE "/mqtt.sub"     // loaded into "G_mqtt_sub"
#define MQTT_TAGS_FILE "/mqtt.tags"   // loaded into "G_mqtt_tags"
#define AUTOEXEC_FILE "/autoexec.cfg"
#define HOSTNAME_FILE "/hostname"

#define LED_BUILTIN 2                 // applies to KeeYees ESP32
#define BLINK_ON HIGH
#define BLINK_OFF LOW

#define MAX_THREADS 16
#define MAX_THREAD_STACK 4096
#define MAX_THREAD_NAME 40
#define MAX_THREAD_ARGS 8             // number of input args
#define MAX_THREAD_RESULT_TAGS 8      // meta data tags
#define MAX_THREAD_RESULT_VALUES 16   // output values
#define MAX_THREAD_CONF_BUF 80        // length of thread's "conf"
#define MAX_THREAD_TAGS_BUF 80        // length of thread's tags
#define MAX_THREAD_MSG_BUF 80         // length of thread's "msg"
#define MAX_THREAD_TAGS 8             // tag pairs in "/tags-<name>"
#define THREAD_SHUTDOWN_PERIOD 1000   // shutdown grace period (millisecs)

#define MAX_DS18B20_DEVICES 4         // maximum DS18B20 on a single pin

#define MAX_HTTP_REQUEST 1024
#define MAX_HTTP_CLIENTS 4            // esp32 supports 8x file descriptors
#define MAX_HTTP_RTIME 20             // seconds to receive an http request

/* my I2S settings */

#define MYI2S_BITS_PER_SAMPLE 32      // update i2s_config.bits_per_sample too
#define MYI2S_DMA_IN_BUFS 2           // number of DMA buffers sampling audio
#define MYI2S_DMA_OUT_BUFS 32         // number of DMA buffers outputing audio
#define MYI2S_DMA_SAMPLES 320         // samples per DMA buffer
#define MYI2S_DMA_WAITTICKS 100       // max ticks i2s_read() will wait
#define MYI2S_INPUT_PORT I2S_NUM_0    // audio input always uses port 0
#define MYI2S_OUTPUT_PORT I2S_NUM_1   // audio output always uses port 1

/* various states for S_thread_entry.state */

#define THREAD_READY          0       // ready to be started
#define THREAD_STARTING       1       // thread just got created
#define THREAD_RUNNING        2       // thread in f_thread_lifecycle()
#define THREAD_WRAPUP         3       // tell a thread to terminate
#define THREAD_STOPPED        4       // thread is as good as dead

/* ====== ALL OTHER GENERAL STUFF ====== */

#define FLOAT_DECIMAL_PLACES 6  // when printing float metrics
#define DEF_BAUD 9600           // USB serial port baud rate
#define SERIAL_TIMEOUT 1000     // serial timeout in milliseconds
#define BLINK_FREQ 5000         // blink to indicate we're alive (ms)
#define MAX_GPIO_PINS 40        // total number of GPIO pins in G_pin_flags[]
#define MAX_TOKENS 10
#define BUF_SIZE 80
#define BUF_MEDIUM 256
#define REPLY_SIZE 2048

/* LCD definitions */

#define LCD_ADDR 0x27
#define LCD_WIDTH 16
#define LCD_ROWS 2

/* Data structure of a single thread result value (with multiple tags) */

struct thread_result_s
{
  int num_tags ;                      // number of meta data tags
  char *meta[MAX_THREAD_RESULT_TAGS] ;  // array of meta tags
  char *data[MAX_THREAD_RESULT_TAGS] ;  // array of data tags
  int i_value ;                       // this result's value
  double f_value ;                    // this result's value
} ;
typedef struct thread_result_s S_thread_result ;

/* Data structure to track a single thread, as well as its in/out data */

struct thread_entry_s
{
  TaskHandle_t tid ;
  char name[MAX_THREAD_NAME] ;
  unsigned char state ;
  SemaphoreHandle_t lock ;            // lock before making changes here

  /* input arguments to <ft_task> */

  int num_args ;                      // number of input arguments
  char *in_args[MAX_THREAD_ARGS] ;    // array of pointers into "conf"
  char conf[MAX_THREAD_CONF_BUF] ;    // main config buffer
  char tags_buf[MAX_THREAD_TAGS_BUF] ; // metric and tags (optional)
  char *metric ;                      // pointer into "tags_buf" (optional)
  char *tags[MAX_THREAD_TAGS+1] ;     // pointers into "tags_bufs" (optional)

  /* thread metadata */

  int core ;                          // cpu core ID thread runs on
  unsigned long loops ;               // number of <ft_task> calls so far
  unsigned long ts_started ;          // millis() timestamp of xTaskCreate()
  void (*ft_addr)(struct thread_entry_s*) ; // the <ft_task> this thread runs

  /* IMPORTANT !!! <ft_task> may modify anything below this point */

  int num_int_results ;               // number of "i_value" results returned
  int num_float_results ;             // number of "f_value" results returned
  S_thread_result results[MAX_THREAD_RESULT_VALUES] ;
  char msg[MAX_THREAD_MSG_BUF] ;      // provide some optional feedback
} ;
typedef struct thread_entry_s S_thread_entry ;

/* Data structure of a web client connection */

struct web_client
{
  int sd ;                              // the client's socket descriptor
  int req_pos ;                         // insertion point in "request"
  unsigned long connect_time ;          // millis() timestamp of connection
  char request[MAX_HTTP_REQUEST] ;      // http request from client
  char method[BUF_SIZE] ;               // http method from "request"
  char uri[BUF_MEDIUM] ;                // URI of http request
  char query[BUF_MEDIUM] ;              // arguments after "uri"
} ;
typedef struct web_client S_WebClient ;

/* internal performance metrics */

struct internal_metrics
{
  unsigned long cronRuns ;
  unsigned long serialInBytes ;
  unsigned long serialCmds ;
  unsigned long serialOverruns ;
  unsigned long restInBytes ;
  unsigned long restCmds ;
  unsigned long mqttConnects ;
  unsigned long mqttPubs ;
  unsigned long mqttSubs ;
  unsigned long mqttOversize ;
  unsigned long mqttPubWaits ;
  unsigned long wifiReconnects ;
} ;
typedef struct internal_metrics S_Metrics ;

/* data structure to track each GPIO pin's usage */

struct pin_flag
{
  unsigned char is_power_pin ;          // pin supplies power to a device
  unsigned char is_shared_power ;       // is a shared power pin
  S_thread_entry **users ;  // MAX_THREAD pointers to threads using this pin
} ;
typedef struct pin_flag S_pin_flag ;

/* data structure to track a hardware UART */

struct hw_uart
{
  int in_use ;                  // number of threads accessing UART
  int initialized ;             // track if "Serial2.begin()" has been called
  SemaphoreHandle_t lock ;      // lock before any access to this structure
} ;
typedef struct hw_uart S_hw_uart ;

/* Function prototypes in "misc.c" */

int f_i2c_readShort (int device, unsigned char addr, short *result) ;
int f_i2c_readUShort (int device, unsigned char addr, unsigned short *result) ;

