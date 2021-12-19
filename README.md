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

## Initial Setup

"mygpio" listens on the serial port (ie, USB serial port), and waits for
user commands. This is similar to how you perform initial configuration with
network devices (eg, Cisco, Arista, Juniper switches, etc). Simply run your
favorite terminal emulator, setting the serial line to 9600 baud, N-8-1. On
a linux machine, this is typically,

```
# minicom -D /dev/ttyUSB0
```

Once connected, press ENTER, and "mygpio" should respond with the "OK"
prompt. You can type ``help`` and press ENTER, for a list of all commands
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

Some commands may have multiple arguments, separate arguments with a ``+``,
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

- our hostname (ie, ``/hostname``)
- MQTT configuration (ie, ``/mqtt.cfg``, ``/mqtt.pub`` and ``/mqtt.sub``)
- wifi SSID and password (ie, ``/wifi.ssid`` and ``/wifi.pw``)

It will then scan for its wifi SSID and connect to the AP with the strongest
signal. Note that MQTT (even if configured) will be in a disconnected state.
After running for 60 seconds (and every 60 seconds after that), it runs 

- if wifi is disconnected, scan for APs and try to reconnect
- if MQTT is disconnected, try to connect
- if we're up for exactly 1 minute, check if the file ``/autoexec.cfg``
  exists, and start background threads listed in this file.

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
config file in the format ``/thread-<name>``. In this example, our thread
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
format ``/tags-<name>``. In the following example, we want our DHT22's metrics
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
``/autoexec.cfg`` file.

