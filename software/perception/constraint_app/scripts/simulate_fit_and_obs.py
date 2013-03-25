import sys
import os
# for bottime:
import time
import random
import numpy
import termios, atexit
from select import select

def kbhit():
    dr,dw,de = select([sys.stdin], [], [], 0)
    return dr <> []

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
from drc.affordance_track_collection_t import affordance_track_collection_t
from drc.affordance_track_t import affordance_track_t
from drc.vector_3d_t import vector_3d_t

def timestamp_now (): return int (time.time () * 1000000)

lc = lcm.LCM()

print 'Number of arguments:', len(sys.argv), 'arguments.'
print 'Argument List:', str(sys.argv)

# 1. Send a fitted affordance to the module
aff_fit = affordance_t() 
aff_fit.utime = timestamp_now()
aff_fit.otdf_type = "gate_valve"
aff_fit.uid = 0
aff_fit.nparams     = 11
aff_fit.params      = [1.0, 1.0, 1.0,    0.0,     0.0,   0.0,            1.0,           0.2,  4.0,  4.0,  4.0]
aff_fit.param_names = ["x", "y", "z", "roll", "pitch", "yaw", "valve_radius", "tube_radius", "lX", "lY", "lZ"]

print 'Sending fit message'
lc.publish("AFFORDANCE_FIT", aff_fit.encode())

# 2. Send some sample "initial points" (i.e., the fit)
track_ids = [10, 20, 30]
track_means = numpy.matrix([[ 1.2, 1.2, 1.2 ],
                            [ 0.9, 0.8, 0.9 ],
                            [ 1.5, 1.0, 1.1 ]])

atc = affordance_track_collection_t()
atc.utime = timestamp_now()
atc.uid = 0
atc.ntracks = 2

for i in range(atc.ntracks):
    track = affordance_track_t()
    track.segment = str(track_ids[i])
    v = vector_3d_t()
    v.x = track_means[i,0]
    v.y = track_means[i,1]
    v.z = track_means[i,2]
    track.position = v
    atc.tracks.append(track)

print 'Sending a bunch of initial points'
lc.publish("AFFORDANCE_TRACK_COLLECTION", atc.encode())

# 3. Send some noisy observations
for j in range(3):
    atc = affordance_track_collection_t()
    atc.utime = timestamp_now()
    atc.uid = 0
    count = 0
    atc.ntracks = 0

    for i in range(len(track_ids)):
        if j == 2 and i == 1:
            continue
        track = affordance_track_t()
        track.segment = str(track_ids[i])
        v = vector_3d_t()
        var = 0.01*0.01;
        v.x = track_means[i,0] + random.gauss(0.0,var)
        v.y = track_means[i,1] + random.gauss(0.0,var)
        v.z = track_means[i,2] + random.gauss(0.0,var)
        track.position = v
        atc.tracks.append(track)
        atc.ntracks +=1 
        print 'queueing observation for segment', track.segment

    print 'Sending a bunch of noisy observations'
    lc.publish("AFFORDANCE_TRACK_COLLECTION", atc.encode())

    print 'waiting...'
    while ( not kbhit() ):
        pass
    sys.stdin.read(1)

