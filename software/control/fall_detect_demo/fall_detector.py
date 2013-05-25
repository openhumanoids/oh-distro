from __future__ import division
import lcm
import os
import sys
sys.path.append(os.path.expanduser('~/drc/software/build/lib/python2.7/dist-packages'))
import drc

m = lcm.LCM()

class FallDetector:
    def __init__(self):
        self.decay_rate = 0.2
        self.zmp_error = 0
        self.last_t = None

    def handle(self, channel, data):
        msg = drc.controller_zmp_status_t.decode(data)
        t = msg.utime / 1e9
        if self.last_t is None:
            self.last_t = t
        else:
            self.zmp_error = max(0, self.zmp_error + (t - self.last_t) * (int(not msg.zmp_ok) - self.decay_rate))
            self.last_t = t
        # print "{:.2f} {:b} {:.3f}".format(t, msg.zmp_ok, self.zmp_error)
        if self.zmp_error > 0.1:
            print "WARNING: FALL DETECTED"

f = FallDetector()

m.subscribe("ZMP_STATUS", f.handle)
while True:
    m.handle()
