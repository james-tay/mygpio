#!/usr/bin/env python3
#
# OVERVIEW
#
# This utility inspects a directory containin ESP32 configuration files,
# then it checks if these files are present on each of the ESP32s. The
# directory names are the hostnames of the ESP32s that will be checked.
#
# On the other hand, this utility can be pointed to a directory (with empty
# subdirectories bearing the names of ESP32s), It then downloads all files
# from ESP32s and writes them into these directories.

import os
import sys
import urllib.request

import pprint

# This dict contains the local file contents for all ESP32 devices. The key is
# the hostname of the ESP32 and the value is another dict, where its key is
# the filename and the value is the contents.

G_conf = {}

# This dict contains the running configuration on all ESP32 devices. The key
# is the hostname of the ESP32 and the value is another dict, where its key
# is the filename and the value is the contents.

G_devices = {}

# -----------------------------------------------------------------------------

# This function is given the path to a directory. The files in this directory
# are presumed to be config files. This function returns a dict where the
# key is the filename and the value is the file's contents. If something went
# wrong, we return None.

def f_read_local_conf(dirname):
  conf = {}
  for (dirpath, dirnames, filenames) in os.walk(dirname):
    for f in filenames:
      filepath = "%s/%s" % (dirname, f)
      if (dirname.endswith("/")):
        filepath = "%s%s" % (dirname, f)
      try:
        fd = open("%s" % filepath, "r")
      except:
        e = sys.exc_info()
        print("WARNING: Cannot read %s - %s" % (filepath, e[1]))
        return(None) 
      conf[f] = fd.read().rstrip("\r\n")
      fd.close()
  return(conf)

# This function is given the hostname of an ESP32. Its job is to read all
# configuration files on the device and return a dict where the key is the
# filename and the value is the content. If something goes wrong, we return
# None.

def f_read_dev_conf(host):
  url = "http://%s/v1?cmd=fs+ls" % host
  try:
    resp = urllib.request.urlopen(url, timeout=5).read().decode("utf-8")
  except:
    e = sys.exc_info()
    print("WARNING: Cannot query %s - %s" % (host, e[1]))
    return(None)

  # at this point, "resp" would look like,
  #
  #   10       /hostname
  #   55       /mqtt.cfg
  #   40       /mqtt.pub
  #   20       /mqtt.sub
  #
  # identify all the various file names on this ESP32, save it into "conf".

  conf = {}
  for line in resp.split("\n"):
    line = line.rstrip("\r\n")
    if (len(line) > 1):
      tokens = line.split()
      if (len(tokens) != 2):
        print("WARNING: Cannot parse line from %s [%s]" % (host, line))
        return(None)
      conf[tokens[1]] = ""

  # now download all files in this ESP32. Note that for some reason, the
  # ESP32-cam devices return filenames without a "/" in front, thus we need
  # to manually stick it in.

  for filename in conf.keys():
    if filename.startswith("/"):
      url = "http://%s/v1?cmd=fs+read+%s" % (host, filename)
    else:
      url = "http://%s/v1?cmd=fs+read+/%s" % (host, filename)

    try:
      resp = urllib.request.urlopen(url, timeout=5).read().decode("utf-8")
    except:
      e = sys.exc_info()
      print("WARNING: Cannot read %s on %s - %s" % (filename, host, e[1]))
      return(None)
    if resp.startswith("FAULT"):
      print("WARNING: '%s' returned '%s'." % (url, resp.rstrip("\r\n")))
      return(None)

    conf[filename] = resp.rstrip("\r\n")
  return(conf)

# This function is given "file_path". Our job is to check if its "contents"
# match, and thus return True if contents match or False if contents do not
# match. If "file_path" does not exist, it returns False.

def f_diff_file (file_path, contents):

  if (os.path.isfile(file_path) == False):
    return(False) # because it essentially doesn't match "contents"
  try:
    fd = open(file_path, "r")
  except:
    e = sys.exc_info()
    print("WARNING: Cannot open %s - %s" % (file_path, e[1]))
    return(False)
  try:
    buf = fd.read()
  except:
    e = sys.exc_info()
    print("WARNING: Cannot read %s - %s" % (file_path, e[1]))
    fd.close()
    return(False)
  fd.close()
  buf = buf.rstrip("\n")

  if (buf == contents):
    return(True)
  else:
    return(False)

# This function writes "contents" to "file_path" on "dev". On success it
# returns True, otherwise False if something goes wrong.

def f_write_dev_file(dev, file, contents):
  if (file.startswith("/")):
    file_path = file
  else:
    file_path = "/%s" % file
  url = "http://%s/v1?cmd=fs+write+%s+%s" % (dev, file_path, contents)

  try:
    resp = urllib.request.urlopen(url, timeout=5).read().decode("utf-8")
  except:
    e = sys.exc_info()
    print("WARNING: Failed to write %s on %s - %s" % (file_path, dev, e[1]))
    return(False)

  if (resp.startswith("FAULT")):
    print("WARNING: Error writing %s on %s - %s" % (file_path, dev, resp))
    return(False)

  return(True)

# -----------------------------------------------------------------------------

if (len(sys.argv) != 3):
  print("Usage: %s <config dir> { to_local | from_local }" % sys.argv[0])
  sys.exit(1)

# load local ESP32 config files into memory. Even if sub-dirs are empty,
# this gives us the names of our ESP32s.

for (dirpath, dirnames, filenames) in os.walk(sys.argv[1]):
  for d in dirnames:
    if (dirpath.endswith("/")):
      full_path = "%s%s" % (dirpath, d)
    else:
      full_path = "%s/%s" % (dirpath, d)
    print("NOTICE: Reading local %s." % full_path)
    x = f_read_local_conf(full_path)
    if (x is not None):
      G_conf[d] = x

# now that we know our ESP32's hostnames, load their config files into memory.

for dev in G_conf.keys():
  print("NOTICE: Reading config from %s" % dev)
  x = f_read_dev_conf(dev)
  if (x is not None):
    G_devices[dev] = x

# depending on how we were called on the commandline ...

if (sys.argv[2] == "to_local"):
  for dev in G_devices.keys():
    for file in G_devices[dev].keys():
      file_path = "%s/%s/%s" % (sys.argv[1], dev, file.lstrip("/"))

      # check if file contents match before (over)writing the file.

      if (f_diff_file(file_path, G_devices[dev][file]) == False):
        print("NOTICE: Writing %s." % file_path)
        try:
          fd = open(file_path, "w")
        except:
          e = sys.exc_info()
          print("FATAL! Cannot write to %s - %s" % (file_path, e[1]))
          sys.exit(1)
        if (G_devices[dev][file].endswith("\n")):
          fd.write(G_devices[dev][file])
        else:
          fd.write("%s\n" % G_devices[dev][file])
        fd.close()
      else:
        print("NOTICE: No changes to %s." % file_path)
  sys.exit(0)

if (sys.argv[2] == "from_local"):
  for dev in G_devices.keys():          # loop through all devices
    for filename in G_conf[dev].keys(): # go through all files on this device
      write_this_file = False           # whether we'll call f_write_dev_File()

      if (filename not in G_devices[dev]):
        print("NOTICE: File %s not present on %s." % (filename, dev))
        write_this_file = True
      else:
        if (G_devices[dev][filename] != G_conf[dev][filename]):
          print("NOTICE: File %s does not match on %s." % (filename, dev))
          write_this_file = True
      if (write_this_file):
        f_write_dev_file(dev, filename, G_conf[dev][filename])
      else:
        print("NOTICE: No change to %s on %s." % (filename, dev))

