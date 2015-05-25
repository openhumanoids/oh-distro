#!/usr/bin/python
import os,sys
import lcm
import time

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from kinect.map_command_t import map_command_t

def timestamp_now (): return int (time.time () * 1000000)

if len(sys.argv)>1:
  val = sys.argv[1].lower()
  # currently crashes otherwise
else:
  print 'No mode specified: start or finish, some finish'
  val = 'finish'




msg = map_command_t()
msg.timestamp = timestamp_now()
if (val is 'start'):
  msg.command = 0
if (val == 'finish'):
  msg.command = 1


lc = lcm.LCM()
lc.publish("KINECT_MAP_COMMAND", msg.encode())
print "Commanding: " , val

