#!/usr/bin/env python
#
# This script connects to an arduino, at its virtual serial port, sends the
# user specified command, and prints whatever response received from the
# arduino.

import os
import sys
import termios

import pprint

cfg_serial_port = "/dev/ttyUSB0"
cfg_serial_baud = termios.B9600

# Concat our commandline arguments, that's what we'll send to the arduino.

del (sys.argv[0])
cmd = " ".join(sys.argv)

# Now open the serial port.

com_fd = os.open (cfg_serial_port, os.O_RDWR | os.O_NOCTTY)
if (com_fd is None):
  print ("FATAL! Cannot open '%s'." % cfg_serial_port)
  sys.exit (1)

# get the current line attributes, which is an array. Note:
#  - array elements : [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
#
# References:
#  https://docs.python.org/2/library/termios.html
#  https://github.com/vsergeev/python-periphery/blob/master/periphery/serial.py

(iflag, oflag, cflag, lflag, ispeed, ospeed, cc) = termios.tcgetattr (com_fd)

# enable receiver "CREAD" in "cflag"
# ignore modem control lines "CLOCAL" in "cflag"
# set baud rate in "cflag"
# set data bit 8 "CS8" in "cflag"
# set stop bit 1 in "cflag"

cflag = ( termios.CREAD | termios.CLOCAL | termios.CS8 | cfg_serial_baud )
cflag &= ~termios.CSTOPB

# ignore break characters in "iflag"
# set parity none in "iflag"

iflag = ( termios.IGNBRK | termios.INPCK | termios.ISTRIP )

# set baud rate in "ispeed" and "ospeed"

ispeed = cfg_serial_baud
ospeed = cfg_serial_baud

# now set line characteristics

termios.tcsetattr (com_fd, termios.TCSANOW,
                   [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])

# Send a newline and keep reading until we get "OK".

os.write (com_fd, cmd + "\r")
buf = ""
while (buf.endswith("OK\r\n") == False):
  s = os.read (com_fd, 80)
  buf = buf + s

buf = buf.rstrip ("OK\r\n")
print (buf)
os.close (com_fd)

