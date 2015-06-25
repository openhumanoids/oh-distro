#!/usr/bin/python
import os,sys
import lcm
import time

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from ipab.pause_command_message_t import pause_command_message_t

def timestamp_now (): return int (time.time () * 1000000)

if len(sys.argv)>1:
  val = sys.argv[1].lower()
  if (val[0] == 'f'):
    pause = False
  if (val[0] == 't'):
    pause = True
  # currently crashes otherwise

else:
  print 'No mode specified!'
  pause= True

msg = pause_command_message_t()
msg.utime = timestamp_now()
msg.pause = pause

lc = lcm.LCM()
lc.publish("VAL_COMMAND_PAUSE", msg.encode())
print "Commanding pause: " , pause

