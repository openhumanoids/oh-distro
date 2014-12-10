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
from drc.affordance_plus_collection_t import affordance_plus_collection_t
from drc.affordance_plus_t import affordance_plus_t

def on_aff_fit(channel, data):
  global counter
  m = affordance_plus_t.decode(data)
  if (counter%100 ==0):
    print "FIT %d id from oracle republished [counter %d]" % (m.aff.uid, counter)
  counter=counter+1
  lc.publish("AFFORDANCE_FIT", m.encode())

global init_list
init_list =[]

def on_aff_track(channel, data):
  m = affordance_plus_t.decode(data)
  global counter, init_list
  if m.aff.uid in init_list:
    #print "saw %d before" %(m.uid)
    return
  else:
    init_list.append(m.aff.uid)
  print "TRK %d id from oracle republished [counter %d]" % (m.aff.uid, counter)
  counter=counter+1
  m.aff.aff_store_control = 0
  m.aff.uid=-1
  lc.publish("AFFORDANCE_FIT", m.encode())

#################################################################################
print "started"
global counter
counter =0;
lc = lcm.LCM()
sub1 = lc.subscribe("AFFORDANCE_FIT_ORACLE", on_aff_fit)
sub2 = lc.subscribe("AFFORDANCE_TRACK_ORACLE", on_aff_track)
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub1)
