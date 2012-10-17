import sys
import lcm
# for bottime:
import time


sys.path.append("/home/mfallon/drc/software/build/lib/python2.7/site-packages")
sys.path.append("/home/mfallon/drc/software/build/lib/python2.7/dist-packages")
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
affc.map_utime = timestamp_now()
affc.map_id = 1
affc.naffs = 2

aff1 = affordance_t() 
aff1.map_utime =affc.map_utime
aff1.name = "uraniumrod"
aff1.object_id = 0
aff1.otdf_id = 0 
aff1.nparams =9
aff1.params = [0.41,1.0,0.0,0.16,0.0,0.66,-0.31,0.0,0.45]
aff1.param_names = ["length","mass","pitch", "radius","roll","x", "y","yaw","z"]

aff2 = affordance_t() 
aff2.map_utime =affc.map_utime
aff2.name = "savereactor"
aff2.object_id = 0
aff2.otdf_id = 0 
aff2.nparams =23
aff2.params = [0, 1, 1, 1,1, 3, 3, 3, 1, 1.5, 0.2, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1, -0.67, 0.0]
aff2.param_names = ["Density", "base_density", "base_h", "base_l", "base_w", "lX", "lY", "lZ", "lever_density", "lever_length","lever_thickness", "lever_turn", "lever_width", "lever_y_offset", "lever_z_offset", "pitch", "roll", "tube_radius", "wheel_radius", "x", "y", "yaw", "z"]

for i in range(100):
  print i
  aff2.params[11] =i/10.0 
  affc.affs = [aff1,aff2]
  
  lc.publish("AFFORDANCE_COLLECTION", affc.encode())
  time.sleep(0.1)
 

