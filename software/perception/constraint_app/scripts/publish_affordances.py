import sys
import os
# for bottime:
import time

myhome = os.environ.get("HOME")

path1 = myhome + "/drc/software/build/lib/python2.7/site-packages"
path2 = myhome + "/drc/software/build/lib/python2.7/dist-packages"
print path1
print path2
sys.path.append(path1)
sys.path.append(path2)
import lcm
from drc.affordance_collection_t import affordance_collection_t
from drc.affordance_t import affordance_t

def timestamp_now (): return int (time.time () * 1000000)

lc = lcm.LCM()

print 'Number of arguments:', len(sys.argv), 'arguments.'
print 'Argument List:', str(sys.argv)
# wheelchair robot assumes 40Hz

print "send a single message"

affc = affordance_collection_t() 

    #pose.orientation[0] = 1.000000
affc.name ="homers_desk"
affc.utime = timestamp_now()
affc.map_id = 1
#affc.naffs = 2
affc.naffs = 1

aff1 = affordance_t() 
aff1.utime =affc.utime
aff1.otdf_type = "gate_valve"
aff1.uid = 0
aff1.nparams     = 11
aff1.params      = [0.0, 0.0, 0.0,    0.0,     0.0,   0.0,            1.0,           0.2,  4.0,  4.0,  4.0]
aff1.param_names = ["x", "y", "z", "roll", "pitch", "yaw", "valve_radius", "tube_radius", "lX", "lY", "lZ"]
'''
aff2 = affordance_t() 
aff2.utime =affc.utime
aff2.otdf_type = "savereactor"
aff2.uid = 0 
aff2.nparams =23
aff2.params = [0, 1, 1, 1,1, 3, 3, 3, 1, 1.5, 0.2, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1, -0.67, 0.0]
aff2.param_names = ["Density", "base_density", "base_h", "base_l", "base_w", "lX", "lY", "lZ", "lever_density", "lever_length","lever_thickness", "lever_turn", "lever_width", "lever_y_offset", "lever_z_offset", "pitch", "roll", "tube_radius", "wheel_radius", "x", "y", "yaw", "z"]
'''
for i in range(2):
  print i
  #aff2.params[11] =i/10.0 
  #affc.affs = [aff1,aff2]
  affc.affs = [aff1]
  
  lc.publish("AFFORDANCE_COLLECTION", affc.encode())
  time.sleep(0.1)
 


