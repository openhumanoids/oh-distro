import sys
import lcm
# for bottime:
import time

sys.path.append("/home/mfallon/projects/kmcl/build/lib/python2.7/site-packages")
sys.path.append("/home/mfallon/projects/kmcl/build/lib/python2.7/dist-packages")
from bot_core.pose_t import pose_t

def timestamp_now (): return int (time.time () * 1000000)

lc = lcm.LCM()



# wheelchair robot assumes 40Hz

for i in range(1, 2):
    print "send a single pose message"
    time.sleep(0.025)
    pose = pose_t() 
    # example pose in 8th floor map
    pose.pos[0] = 43.66
    pose.pos[1] = 11.33
    pose.pos[1] = 0.0


    pose.orientation[0] = 1.000000
    pose.utime =timestamp_now()
    lc.publish("POSE", pose.encode())
