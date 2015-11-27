#!/usr/bin/python
import os,sys
import lcm
import time

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from ipab.com_height_packet_message_t import com_height_packet_message_t

def timestamp_now (): return int (time.time () * 1000000)

if len(sys.argv)>1:
  height = float(sys.argv[1])
else:
  print 'No mode specified!'
  height= 0.85

if (height > 0.12):
  height=0.12
  print "too high, capping to 0.12"

if (height < -0.3):
  height=-0.3
  print "too low, capping to -0.3"

msg = com_height_packet_message_t()
msg.utime = timestamp_now()
msg.min_com_height = height - 0.02
msg.max_com_height = height + 0.02
msg.height_offset = height

lc = lcm.LCM()
lc.publish("VAL_COMMAND_COM_HEIGHT", msg.encode())
print "Commanding height to" , height

