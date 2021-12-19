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

