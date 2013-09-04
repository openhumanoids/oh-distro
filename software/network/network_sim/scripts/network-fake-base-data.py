import sys
import lcm
# for bottime:
import time

sys.path.append("/home/mfallon/projects/kmcl/build/lib/python2.7/site-packages")
sys.path.append("/home/mfallon/projects/kmcl/build/lib/python2.7/dist-packages")
from drc.gaze_command_t import gaze_command_t
from drc.recovery_t import recovery_t

def timestamp_now (): return int (time.time () * 1000000)

lc = lcm.LCM()

while (1):
    time.sleep(1)
    gaze = gaze_command_t()
    gaze.utime =timestamp_now()
    gaze.affordance_id = -1
    lc.publish("GAZE_COMMAND", gaze.encode())

    rec = recovery_t()
    rec.utime = timestamp_now()
    rec.mode =1;
    rec.controller =0;
    lc.publish("RECOVERY_CMD", rec.encode())
