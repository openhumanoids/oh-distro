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
from vs.obj_collection_t import obj_collection_t
from vs.obj_t import obj_t
from vs.text_collection_t import text_collection_t
from vs.text_t import text_t

def on_aff(channel, data):
  global counter
  m = affordance_collection_t.decode(data)
  print str(m.naffs)+ " affs"
  ####### Create and Output OBJ_COLLECTION
  ocol = obj_collection_t()
  ocol.id =569999
  ocol.type =5
  ocol.reset=True
  ocol.name="AFF2TEXT Pose"
  ocol.nobjs=m.naffs
  for i in range(m.naffs):
    aff = m.affs[i]
    o = obj_t()
    o.id= aff.uid
    for j in range(aff.nparams):
      if (aff.param_names[j] =='x'):
        o.x =aff.params[j]
      if (aff.param_names[j] =='y'):
        o.y =aff.params[j]
      if (aff.param_names[j] =='z'):
        o.z =aff.params[j]
      if (aff.param_names[j] =='yaw'):
        o.yaw =aff.params[j]
      if (aff.param_names[j] =='pitch'):
        o.pitch =aff.params[j]
      if (aff.param_names[j] =='roll'):
        o.roll =aff.params[j]
    ocol.objs.append(o)
  lc.publish("OBJ_COLLECTION", ocol.encode())
  ####### Create and Output OBJ_COLLECTION
  vcol = text_collection_t()
  vcol.id = 569998
  vcol.name="AFF2TEXT Labels"
  vcol.reset=True
  vcol.n = m.naffs
  for i in range(m.naffs):
    aff = m.affs[i]
    t = text_t()
    t.id=aff.uid
    t.collection_id=569999
    t.object_id=aff.uid
    t.text = str(aff.uid)
    vcol.texts.append(t)
  lc.publish("TEXT_COLLECTION",vcol.encode() )

#################################################################################
print "started"
global counter
counter =0;
lc = lcm.LCM()
sub1 = lc.subscribe("AFFORDANCE_COLLECTION", on_aff)
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub1)
