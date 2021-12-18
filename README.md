# mygpio

## What is mygpio

Once upon a time, in the world of the raspberry pi, there exists a utility
called "gpio" which lets you set or get GPIO pin hi/lo states from the
commandline. "mygpio" started life on an ESP8266 as a similar tool to
easily set or get GPIO pin hi/lo states, eventually moving up to the ESP32
and eventually becoming an ESP32-only tool.

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

