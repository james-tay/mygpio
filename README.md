# mygpio

## What is mygpio

Once upon a time, in the world of the raspberry pi, there exists a utility
called "gpio" which lets you set or get GPIO pin hi/lo states from the
commandline. "mygpio" started life on an ESP8266 as a similar tool to
easily set or get GPIO pin hi/lo states, eventually moving up to the ESP32
and eventually becoming an ESP32-only tool.

The software has since evolved to doing much more than setting GPIO pins
high and low. Here is a summary of what it can do nowadays.

- read analog values
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
- Over-The-Air (OTA) firmware upgrades from an HTTP webserver
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
Before configuring MQTT, prepare the following information :

| Filename | Description |
| --- | --- |
| /hostname | The name of our ESP32 (short name, not FQDN) |
| /mqtt.cfg | MQTT server hostname, port, username and password |
| /mqtt.sub | Listen for commands on this topic |
| /mqtt.pub | Topic to publish sensor events, and topic for command response |

The hostname of our ESP32 is always appended at the end of topics configured
for publish and subscribe. For example, let's say our MQTT setup is as follows,

- our ESP32's FQDN is "porch.example.com", thus its shortname is "porch"
- we have an MQTT server called "mqtt.example.com", listening on port 1883
- to connect to the MQTT server, our username is "bob", password is "alice"
- we send commands to our ESP32 by publishing to "myhome/command/porch"
- the ESP32 publishes its responses to the topic "myhome/response/porch"
- sensor events on our ESP32 are published to the topic "myhome/sensor/porch"

The following commands create the config files for the above setup.

```
curl http://porch.example.com/v1?cmd=fs+write+/hostname+porch
curl http://porch.example.com/v1?cmd=fs+write+/mqtt.cfg+mqtt.example.com,1883,bob,alice
curl http://porch.example.com/v1?cmd=fs+write+/mqtt.sub+myhome/command
curl http://porch.example.com/v1?cmd=fs+write+/mqtt.pub+myhome/sensors,myhome/response
```

