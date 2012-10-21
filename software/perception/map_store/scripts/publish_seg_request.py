import sys
import time
import os
# for bottime:

myhome = os.environ.get("HOME")
path1 = myhome + "/drc/software/build/lib/python2.7/site-packages"
path2 = myhome + "/drc/software/build/lib/python2.7/dist-packages"
print path1
print path2
sys.path.append(path1)
sys.path.append(path2)

import lcm
from drc.localize_reinitialize_cmd_t import localize_reinitialize_cmd_t

def timestamp_now (): return int (time.time () * 1000000)

lc = lcm.LCM()

print 'Number of arguments:', len(sys.argv), 'arguments.'
print 'Argument List:', str(sys.argv)
print sys.argv[1]
# wheelchair robot assumes 40Hz

print "send a single message"

pose = localize_reinitialize_cmd_t() 

    #pose.orientation[0] = 1.000000
pose.utime =timestamp_now()
lc.publish(sys.argv[1], pose.encode())
