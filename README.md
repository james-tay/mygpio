# mygpio

## What is mygpio

Once upon a time, in the world of the raspberry pi, there exists a utility
called "gpio" which lets you set or get GPIO pin hi/lo states from the
commandline. "mygpio" started life on an ESP8266 as a similar tool to
easily set or get GPIO pin hi/lo states, eventually moving up to the ESP32
and eventually becoming an ESP32-only tool.

The software has since evolved to doing much more than setting GPIO pins
high and low. Here is a summary of what it can do nowadays.

- read analog values (from 0v to 3.3v)
- read capacitive touch pins
- read commonly used sensors like,
  - accelerometer (ADXL335)
  - barometric pressure (BMP180)
  - temperature (DS18B20)
  - humidity (DHT22)
  - ultrasonic range finder (HC-SR04)
- play PWM tones on a speaker
- display text on an I2C LCD

In addition, "mygpio" can schedule multiple threads to run simultaneously,
including,

- read a GPS receiver (BN-220)
- log GPS position to a file
- read from an I2S microphone (SPH0645) and stream via UDP
- receive raw audio via UDP and play it on an I2S audio amplifier (MAX98357)
- activate relays for a specific number of seconds
- connect TTL serial devices to a TCP port

Other features include,

- a REST interface (just use curl)
- expose internal runtime metrics as well sensor metrics to prometheus
- publish events to an MQTT topic (eg, button press on a GPIO pin)
- receive commands by subscribing to an MQTT topic (eg, set GPIO5 high, etc)
- files on flash can be uploaded/downloaded over a TCP socket
- Over-The-Air (OTA) firmware upgrades off an HTTP webserver (won't affect
  existing config files)
- automatically (re)connects to the wifi AP with the strongest signal
- hibernate in a low power mode for a certain duration

## Build and Compile

Make sure you have the [arduino-cli](https://www.arduino.cc/pro/cli) installed.
Next, we need to be able to build for the ESP32 platform, thus,

```
% URL="https://dl.espressif.com/dl/package_esp32_index.json"
% arduino-cli core update-index --additional-urls $URL
% arduino-cli core install esp32:esp32 --additional-urls $URL
```

For reference,

```
% arduino-cli core list
ID          Installed Latest Name
esp32:esp32 2.0.1     2.0.11 esp32
```

Install the external libraries we depend on,

```
% arduino-cli lib install ArduinoOTA
% arduino-cli lib install PubSubClient
% arduino-cli lib install DallasTemperature
% arduino-cli lib install "LiquidCrystal I2C"
```

For reference,

```
% arduino-cli lib list
Name              Installed Available    Location              Description
ArduinoOTA        1.0.9     1.0.10       LIBRARY_LOCATION_USER ...
DallasTemperature 3.9.0     -            LIBRARY_LOCATION_USER -
LiquidCrystal I2C 1.1.2     -            LIBRARY_LOCATION_USER -
OneWire           2.3.7     -            LIBRARY_LOCATION_USER -
PubSubClient      2.8       -            LIBRARY_LOCATION_USER -
```

At this point we should be ready to compile and flash an ESP32,

```
% arduino-cli compile -b esp32:esp32:esp32 .
% arduino-cli upload -v -p /dev/ttyUSB0 -b esp32:esp32:esp32 .
```

To build for the ESP32-CAM platform,

```
% arduino-cli compile -b esp32:esp32:esp32cam .
% arduino-cli upload -v -p /dev/ttyUSB1 -b esp32:esp32:esp32cam .
```


## Initial Setup

"mygpio" listens on the serial port (ie, USB serial port), and waits for
user commands. This is similar to how you perform initial configuration with
network devices (eg, Cisco, Arista, Juniper switches, etc). Simply run your
favorite terminal emulator, setting the serial line to 115200 baud, N-8-1. On
a linux machine, this is typically,

```
# minicom -D /dev/ttyUSB0
```

Once connected, press ENTER, and "mygpio" should respond with the "OK"
prompt. You can type `help` and press ENTER, for a list of all commands
available.

Since the ESP32 supports a simplistic filesystem, "mygpio" uses files to
store its configuration. Typically, the first thing you want to do is get
your ESP32 connected to your wifi network. To do this, you need to first
format the file system (this may take a minute, please be patient) and then
reboot. On your terminal emulator connected to the ESP32's USB port, type the
commands,

```
fs format
reload
````

Next, create files which hold your wifi's SSID and password. In the following
example, our wifi SSID is "sunshine" and our wifi password is "moonlight".
Once these files are present, reboot the ESP32 and it should automatically
connect to your wifi network.

```
fs write /wifi.ssid sunshine
fs write /wifi.pw moonlight
reload
```

Once the ESP32 reboots and connects to your wifi network, you can access all
the commands (on the serial console) over HTTP in the format,

```
% curl http://<esp32_address>/v1?cmd=<args...>
```

For example,

```
% curl http://esp32.example.com/v1?cmd=uptime
```

Some commands may have multiple arguments, separate arguments with a `+`,
for example,

```
% curl http://esp32.example.com/v1?cmd=wifi+status
```

## MQTT Setup (optional)

While much of normal operation can occur over the REST interface, "mygpio" can
also interact over MQTT. This is especially useful to capture short and
transient events, like button presses or analog values crossing a threshold.
Like any MQTT client, we need to specify some configuration for this to work.
The following configuration files are used.

| Filename  | Description  |
| --- | --- |
| /hostname | The name of our ESP32 (short name, not FQDN) |
| /mqtt.cfg | MQTT server hostname, port, username and password |
| /mqtt.sub | Topic prefix for commands meant for us to execute |
| /mqtt.pub | Topic prefix to publish sensor events, and topic prefix for command response |

The hostname of our ESP32 is always appended at the end of topics configured
for publish and subscribe. For example, let's say our MQTT setup is as follows,

- our ESP32's FQDN is "porch.example.com", thus its shortname is "porch"
- we have an MQTT server called "mqtt.example.com", listening on port 1883
- to connect to the MQTT server, our username is "bob", password is "alice"
- we send commands to our ESP32 by publishing to "myhome/command/porch"
- the ESP32 publishes its responses to the topic "myhome/response/porch"
- sensor events (eg, thread named "button1") on our ESP32 are published to the topic "myhome/sensors/porch/button1"

The following commands create the config files for the above setup.

```
% curl http://porch.example.com/v1?cmd=fs+write+/hostname+porch
% curl http://porch.example.com/v1?cmd=fs+write+/mqtt.cfg+mqtt.example.com,1883,bob,alice
% curl http://porch.example.com/v1?cmd=fs+write+/mqtt.sub+myhome/command
% curl http://porch.example.com/v1?cmd=fs+write+/mqtt.pub+myhome/sensors,myhome/response
```

When the above configuration is completed, reboot the ESP32,

```
% curl http://porch.example.com/v1?cmd=reload
```

## Boot up sequence

After a reboot, "mygpio" will attempt to load basic configuration into memory.
If present, this includes,

- our hostname (ie, `/hostname`)
- MQTT configuration (ie, `/mqtt.cfg`, `/mqtt.pub` and `/mqtt.sub`)
- wifi SSID and password (ie, `/wifi.ssid` and `/wifi.pw`)

It will then scan for its wifi SSID and connect to the AP with the strongest
signal. Note that MQTT (even if configured) will be in a disconnected state.
After running for 60 seconds (and every 60 seconds after that), it runs 

- if wifi is disconnected, scan for APs and try to reconnect
- if MQTT is disconnected, try to connect
- if we're up for exactly 1 minute, check if the file `/autoexec.cfg`
  exists, and start background threads listed in this file.
- if `/wifi.rssi` exists, and our wifi RSSI is lower than the value in this
  file, re-scan wifi and search for other base stations (same SSID) with a
  stronger RSSI. Eg, "-72" (dBm).

A bad configuration or misbehaving thread may lead to a crash. To reduce the
effects of crash looping, we delay all thread creation for 1 minute so that
we have a sufficient window to update or correct configuration files.

ESP32s typically have a built-in user controlled LED. Every 5 seconds, a
single blink indicates wifi is connected, while 2x blinks indicate wifi is
disconnected.

## Setting up Threads

"mygpio" can run multiple threads in the background. The most common use case
is to monitor sensors (analog or digital), stream audio, etc. Typically, we
want to expose sensor metrics for prometheus to scrape. Thus, a thread needs
to be started up with some configuration and optionally metadata tags that
are presented along with metrics for prometheus. To obtain a list of the
various threads "mygpio" can run, together with their arguments,

```
% curl http://porch.example.com/v1?cmd=esp32+thread_help
```

Each thread we run must have a unique name. In the following example, we
have a DHT22 (temperature and humidity) sensor with its data pin connected to
the ESP32's GPIO17, and its power supply pin connected to GPIO16. We want
our thread to poll the sensor every 60000 milliseconds. Thus we create a
config file in the format `/thread-<name>`. In this example, our thread
will be named "env1",

```
% curl http://porch.example.com/v1?cmd=fs+write+/thread-env1+ft_dht22,60000,17,16
```

Once this thread config file is created, we can start the thread manually,

```
% curl http://porch.example.com/v1?cmd=esp32+thread_start+env1
```

We can now check that the thread is running, eg

```
% curl http://porch.example.com/v1?cmd=esp32+thread_list
```

If all goes well, we can scrape our ESP32, which should include metrics from
our DHT22,

```
% curl http://porch.example.com/metrics
...
env1{measurement="temperature"} 22.100000
env1{measurement="humidity"} 49.000000
env1{readings="abnormal"} 0.000000
```

Notice that the name of the metric is our thread name. While this is sufficient
to expose simple time series data to prometheus, we may have a scenario where
many sensors are connected to a single ESP32, or we may have multiple ESP32s
with many DHT22 sensors attached. In this scenario, we want to expose all
the sensor metrics with some addition metadata tags to help us identify what
the sensor is reading. To do this, each thread has a tags config file in the
format `/tags-<name>`. In the following example, we want our DHT22's metrics
exposed as "sensor_environment". We also tag the location and model of the
sensor (notice that we escape the double-quotes).

```
% curl http://porch.example.com/v1?cmd=fs+write+/tags-env1+sensor_environment,location=\"Porch\",model=\"dht22\"
```

Next, restart the "env1" thread,

```
% curl http://porch.example.com/v1?cmd=esp32+thread_stop+env1
% curl http://porch.example.com/v1?cmd=esp32+thread_start+env1
```

Now, when we try to scrape, we see the thread's metrics updated with the
additional metadata tags we've configured,

```
% curl http://porch.example.com/metrics
...
sensor_environment{location="Office",model="dht22",measurement="temperature"} 22.299999
sensor_environment{location="Office",model="dht22",measurement="humidity"} 47.099998
sensor_environment{location="Office",model="dht22",readings="abnormal"} 0.000000
```

Finally, to have this thread automatically start on boot (technically 1 minute
after boot),

```
% curl http://porch.example.com/v1?cmd=fs+write+/autoexec.cfg+env1
```

Multiple threads can be automatically started on boot using this method.
Simply provide the thread names as a comma separated list in the 
`/autoexec.cfg` file.

## Support for ESP32-CAM

The same source code can be compiled to support camera functions for the
esp32-cam platform. To compile and upload,

```
% arduino-cli compile -b esp32:esp32:esp32cam .
% arduino-cli upload -v -p /dev/ttyUSB0 -b esp32:esp32:esp32cam .
```

Due to the way the esp32 camera module uses serial signalling, it may be
necessary to perform initial configuration (ie, wifi setup) using the arduino
IDE's Serial Monitor. To use the camera, first initialize it, optionally
configure it, and then start capturing jpeg frames. For example,

```
% curl http://esp32-cam.example.com/v1?cmd=cam+init
% curl http://esp32-cam.example.com/v1?cmd=cam+set+framesize+xga
% curl http://esp32-cam.example.com/v1?cmd=cam+set+vflip+1
% curl http://esp32-cam.example.com/v1?cmd=cam+set+hmirror+1
% curl http://esp32-cam.example.com/cam >image.jpg
```

To view have the image auto-reload in your browser, specify the refresh time
in milli-seconds. Eg,

```
http://esp32-cam.example.com/cam?refresh=5000
```

To stream images from the camera, specify the desired frame rate (eg, 5 fps),

```
http://esp32-cam.example.com/cam?stream=5
```

Idle power draw from 5v USB power is about 80ma and up to 150ma when camera
is capturing and transmitting over wifi.

To enable long exposure times on the esp32-cam module,

```
% curl http://esp32-cam.example.com/v1?cmd=cam+reg+255+255+1
% curl http://esp32-cam.example.com/v1?cmd=cam+reg+17+255+1
```

The esp32-cam's OTA may fail because it uses the "huge_app" partition scheme
by default (which only has 1x partition for user application code). This
partition scheme is defined in file,

`.arduino15/packages/esp32/hardware/esp32/2.0.1/tools/partitions/huge_app.csv`

We can modify,

`.arduino15/packages/esp32/hardware/esp32/2.0.1/boards.txt`

changing the lines,

```
esp32cam.upload.maximum_size=3145728
esp32cam.build.partitions=huge_app
```

to

```
esp32cam.upload.maximum_size=1966080
esp32cam.build.partitions=min_spiffs
```

This in turn uses the partition scheme defined in the file,

`.arduino15/packages/esp32/hardware/esp32/2.0.1/tools/partitions/min_spiffs.csv`

After making the above changes, be sure to recompile, re-upload the binary
and run an "fs format".

### Notes

- Running the camera's XCLK at 20mhz (default) may result in wifi interference,
  as a workaround run it at a different frequency. The following example sets
  XCLK to 6mhz when initializing the camera,
  `curl http://esp32-cam.example.com/v1?cmd=cam+init+6`

- Due to high cpu consumption at high(er) resolutions and/or frame rates, it
  may be possible to overheat the unit, which results in unstable behavior. To
  workaround this, either reduce the frame rate, reduce the XCLK frequency or
  do this only for short periods (eg, a few minutes).

- The "DallasTemperature" library is not thread safe. When multiple DS18B20
  sensors are accessed by different GPIO pins, `ft_ds18b20()` does not correctly
  print their addresses. For this reason, printing of addresses can be
  suppressed with the "noaddr" flag.

