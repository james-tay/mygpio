#!/usr/bin/env python
#
# This script connects to an arduino, at its virtual serial port, sends the
# user specified command, and prints whatever response received from the
# arduino.

import os
import sys
import time
import select
import termios

cfg_serial_port = "/dev/ttyUSB0"
cfg_serial_baud = termios.B9600
cfg_discard_wait_sec = 0.2              # wait for bytes before first xmit
cfg_read_timeout = 20.0                 # secs to wait for (text) response
cfg_debug = 0

if (os.getenv("DEBUG") is not None):            # optional debug flag
  cfg_debug = 1

if (os.getenv("SERIAL") is not None):           # specify serial port
  cfg_serial_port = os.getenv("SERIAL")

# Concat our commandline arguments, that's what we'll send to the arduino.

del (sys.argv[0])
cmd = " ".join(sys.argv)

# Now open the serial port.

com_fd = os.open (cfg_serial_port, os.O_RDWR | os.O_NOCTTY)
if (com_fd is None):
  print ("FAULT: Cannot open '%s'." % cfg_serial_port)
  sys.exit (1)

# If there are unexpected bytes coming from the arduino, read and discard
# them now. Make sure we sit here for a while, this makes sure we read out
# any buffered bytes.

end_tm = time.time () + cfg_discard_wait_sec
while (1):
  remainder = end_tm - time.time()
  if (remainder < 0):
    break
  rfds, wfds, xfds = select.select ([com_fd], [], [], remainder)
  if (com_fd in rfds):
    discard = os.read(com_fd, 80) ;
    if (cfg_debug):
      print ("DEBUG: discard(%s)" % discard.rstrip("\r\n")) ;

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
cflag &= ~( termios.CSTOPB )

#  turn off echo'ing of input characters

lflag &= ~( termios.ECHO )

# ignore break characters in "iflag"
# set parity none in "iflag"

iflag = ( termios.IGNBRK | termios.INPCK | termios.ISTRIP )

# set baud rate in "ispeed" and "ospeed"

ispeed = cfg_serial_baud
ospeed = cfg_serial_baud

# now set line characteristics

termios.tcsetattr (com_fd, termios.TCSANOW,
                   [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])

# Send a newline and keep reading until we get "OK", but enforce a timeout.

amt = os.write (com_fd, cmd + "\r")
if (cfg_debug):
  print ("DEBUG: wrote %d bytes." % amt)

buf = ""
while (buf.endswith("OK\r\n") == False):
  rfds, wfds, xfds = select.select ([com_fd], [], [], cfg_read_timeout)
  if (com_fd in rfds):
    s = os.read (com_fd, 80)
    buf = buf + s
    if (cfg_debug):
      print ("DEBUG: s(%s)" % s.rstrip("\r\n"))
  else:
    print ("FAULT: Timed out on read.")
    sys.exit (1)
    break

buf = buf.rstrip ("OK\r\n")
print (buf)
os.close (com_fd)

