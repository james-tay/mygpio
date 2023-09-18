#!/usr/bin/env python3
#
# REQUIREMENTS
#
#  # apt install python3-yaml python3-requests
#
# USAGE
#
#  % ./dev-init.py <config.yaml>
#
# DESCRIPTION
#
# This utility keeps an eye on ESP32 units and detects if any of them have
# rebooted. If an ESP32 rebooted, commands are sent to it for one time
# initialization. This utility saves the uptime of ESP32 unit periodically
# in case it crashes. This way, after starting back up, it is able to
# determine if any ESP32 units rebooted and require initialization. The uptime
# of each ESP32 is determined by a query to prometheus (ie, using the
# "ec_uptime_secs" metric).
#
# The following sample illustrates this utility's config file,
#
#   dev_state: /srv/devices.json
#   prom_server: http://prometheus.example.com:9090
#   prom_query: ec_uptime_secs{instance=~"iot[0-9].example.com"}
#   poll_freq: 60
#   devices:
#     - instance: "garden:80"           # the "instance" name in prometheus
#       addr: planter.iot.example.com   # how we'll contact this device
#       init:
#         - "hi+15"
#         - "lo+16"
#     - instance: "back-cam:80"
#       addr: 192.168.1.250
#       init:
#         - "cam+init+6"
#         - "cam+set+framesize+sxga"
#     ...
#
# IMPORTANT
#
# We expect our "poll_freq" to be equal or greater than the frequency at
# which prometheus scrapes the ESP32 devices.

import os
import sys
import time
import json
import yaml
import urllib
import requests
import datetime

# -----------------------------------------------------------------------------

# This function executes "query" on a prometheus "server". It returns a dict
# of ESP32 devices, where the key is the instance name of the ESP32 device and
# the value is a dict, including its "uptime".

def f_get_uptime(server, query):

  query_url = "%s/api/v1/query" % server
  payload = "query=%s" % urllib.parse.quote(query)
  headers = { "Content-Type" : "application/x-www-form-urlencoded" }

  try:
    req = requests.post(query_url, headers=headers, data=payload)
  except:
    e = sys.exc_info()
    print("WARNING: Cannot query %s - %s" % (server, e[1]))
    return(None)

  if (req.status_code != 200):
    print("WARNING: POST to %s returned %d." % (query_url, req.status_code))
    return(None)

  # at this point, "req.text" should contain JSON, which looks like,
  #   {
  #     "data": {
  #       "result": [
  #         {
  #           "metric": {
  #             "__name__": "<esp32_name>",
  #             "instance": "<name>:<port>"
  #             "job": "<job_name>",
  #           },
  #           "value": [
  #             <timestamp>, "<value>"
  #           ]
  #         },
  #         ...
  #       ],
  #       ...
  #     }
  #   }

  resp = json.loads(req.text)
  if (resp is None) or \
     ("data" not in resp) or \
     ("result" not in resp["data"]) or \
     (len(resp["data"]["result"]) < 1):
    print("WARNING: No metrics returned for '%s'." % query)
    return(None)

  devices = {}
  for idx in range(0, len(resp["data"]["result"])):
    device = resp["data"]["result"][idx]

    # make sure we can identify "instance" and "value" from "device".

    if ("metric" in device) and ("instance" in device["metric"]) and \
       ("value" in device) and (len(device["value"]) > 0):
      instance = device["metric"]["instance"]
      x = {}
      x["uptime"] = int(device["value"][1])     # device's uptime
      devices[instance] = x

  return(devices)

# This function is called when it's time to initialize a device. It is given
# the config block from the "devices" config array. Our job is to execute the
# "init" commands on the device at "addr".

def f_init_device(dev_config):
  print("NOTICE: initializing %s" % dev_config["addr"])

  tv_start = time.time()
  for idx in range (0, len(dev_config["init"])):
    url = "http://%s/v1?cmd=%s" % (dev_config["addr"], dev_config["init"][idx])
    try:
      result = requests.get(url)
    except:
      e = sys.exc_info()
      print("WARNING: '%s' failed - %s" % (url, e[1]))
      return

    msg = result.content.decode("utf-8").rstrip("\r\n")
    print("  %s returns %d - %s" % (url, result.status_code, msg))
  tv_end = time.time()
  print("NOTICE: completed in %.3f secs." % (tv_end - tv_start))

# -----------------------------------------------------------------------------

# Parse our command line and load up our "cfg".

if (len(sys.argv) != 2):
  print("Usage: %s <config.yaml>" % sys.argv[0])
  sys.exit(1)

try:
  fd = open(sys.argv[1])
except:
  e = sys.exc_info()
  print("FATAL! Cannot read %s - %s." % (sys.argv[1], e[1]))
  sys.exit(1)

try:
  cfg = yaml.safe_load(fd)
except:
  e = sys.exc_info()
  print("FATAL! Cannot parse %s - %s." % (sys.argv[1], e[1]))
  sys.exit(1)
fd.close()

# Sanity check our configuration.

if ("prom_server" not in cfg):
  print("FATAL! 'prom_server' not declared.")
  sys.exit(1)
if ("prom_query" not in cfg):
  print("FATAL! 'prom_query' not declared.")
  sys.exit(1)
if ("devices" not in cfg):
  print("FATAL! 'devices' not declared.")
  sys.exit(1)
if (type(cfg["devices"]) != list):
  print("FATAL! 'devices' is not a list.")
  sys.exit(1)

for idx in range(0, len(cfg["devices"])):
  if ("instance" not in cfg["devices"][idx]):
    print("FATAL! device 'instance' not declared.")
    sys.exit(1)
  if ("addr" not in cfg["devices"][idx]):
    print("FATAL! device 'addr' not declared.")
    sys.exit(1)
  if ("init" not in cfg["devices"][idx]) or \
      (type(cfg["devices"][idx]["init"]) != list):
    print("FATAL! device 'init' is missing or is not a list.")
    sys.exit(1)

# Load an initial list of devices. Note that "prev_list" may be None (eg,
# prometheus is temporarily offline, etc). If the "dev_state" file is supplied,
# use that instead.

prev_list = None
if ("dev_state" in cfg):
  fd = None
  try:
    fd = open(cfg["dev_state"])
  except:
    e = sys.exc_info()
    print("WARNING: Cannot open %s - %s" % (cfg["dev_state"], e[1]))
  if (fd is not None):
    try:
      prev_list = json.load(fd)
    except:
      e = sys.exc_info()
      print("FATAL! Cannot parse JSON in %s - %s" % (cfg["dev_state"], e[1]))
      sys.exit(1)
    fd.close()
    print("NOTICE: loaded %d devices from %s." % \
          (len(prev_list), cfg["dev_state"]))

if (prev_list is None):
  prev_list = f_get_uptime(cfg["prom_server"], cfg["prom_query"])
  if (prev_list is not None):
    print("NOTICE: received %d devices from %s." % \
          (len(prev_list), cfg["prom_server"]))

last_run = time.time()
next_run = last_run + cfg["poll_freq"]

while(1):

  # figure out how long we'll sleep before pulling device uptime metrics

  sleep_time = next_run - time.time()
  if (sleep_time > 0):
    print("NOTICE: time %s, %d devices, sleeping %.3f secs." % \
          (datetime.datetime.now().strftime("%Y%m%d-%H%M%S"),
           len(prev_list), sleep_time))
    time.sleep(sleep_time)
  next_run = next_run + cfg["poll_freq"]
  sys.stdout.flush()

  cur_list = f_get_uptime(cfg["prom_server"], cfg["prom_query"])

  # check if any devices rebooted

  if (cur_list is not None):
    for instance in cur_list.keys():
      if (prev_list is not None) and (instance in prev_list):
        jump_time = cur_list[instance]["uptime"] - prev_list[instance]["uptime"]

        # If any devices have a lower uptime than previously recorded, then
        # we consider them to have been rebooted.

        if (jump_time < 0):
          dev_idx = -1
          for idx in range(0, len(cfg["devices"])):
            if (cfg["devices"][idx]["instance"] == instance):
              dev_idx = idx

          if (dev_idx < 0):
            print("WARNING: %s is not in my configuration." % instance)
          else:
            f_init_device(cfg["devices"][dev_idx])

    prev_list = cur_list

  # if "dev_state" was specified, try to save "prev_list" there.

  if "dev_state" in cfg:
    fd = None
    try:
      fd = open ("%s.new" % cfg["dev_state"], "w")
    except:
      e = sys.exc_info()
      print("WARNING: Cannot open %s.new - %s" % (cfg["dev_state"], e[1]))

    if (fd is not None):
      fault = 0
      try:
        fd.write(json.dumps(prev_list))
      except:
        e = sys.exc_info()
        print("WARNING: Cannot write to %s.new - %s" % \
              (cfg["dev_state"], e[1]))
        fault = 1
      fd.close()
      if (fault):
        os.unlink("%s.new" % cfg["dev_state"])
      else:
        try:
          os.rename("%s.new" % cfg["dev_state"], cfg["dev_state"])
        except:
          e = sys.exc_info()
          print("WARNING: Cannot rename %s.new to %s - %s" % \
                (cfg["dev_state"], cfg["dev_state"], e[1]))

  sys.stdout.flush()

