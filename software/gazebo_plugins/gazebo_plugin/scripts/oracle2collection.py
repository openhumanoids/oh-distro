#!/usr/bin/python
import os,sys
import lcm
from lcm import LCM

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")
from drc.affordance_collection_t import affordance_collection_t
from drc.affordance_t import affordance_t

def on_affs(channel, data):
  global counter
  m = affordance_collection_t.decode(data)
  if (counter%30 ==0):
    print "%d affordances from oracle plugin republished [%d]" % (m.naffs, counter)
  counter=counter+1
  lc.publish("AFFORDANCE_COLLECTION", m.encode())

#################################################################################
print "started"
global counter
counter =0;
lc = lcm.LCM()
sub1 = lc.subscribe("AFFORDANCE_ORACLE", on_affs)
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub1)
