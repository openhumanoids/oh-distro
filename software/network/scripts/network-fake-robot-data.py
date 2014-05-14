import sys
import lcm
# for bottime:
import time

sys.path.append("/home/mfallon/projects/kmcl/build/lib/python2.7/site-packages")
sys.path.append("/home/mfallon/projects/kmcl/build/lib/python2.7/dist-packages")
from drc.estimated_biases_t import estimated_biases_t
from drc.system_status_t import system_status_t

def timestamp_now (): return int (time.time () * 1000000)

lc = lcm.LCM()

while (1):
    time.sleep(1)
    bias = estimated_biases_t()
    bias.utime =timestamp_now()
    lc.publish("ESTIMATED_ACCEL_BIASES", bias.encode())

    system = system_status_t()
    system.utime = timestamp_now()
    system.system =1;
    system.importance =1;
    system.frequency=1;
    system.value = "Data message"
    lc.publish("SYSTEM_STATUS", system.encode())

