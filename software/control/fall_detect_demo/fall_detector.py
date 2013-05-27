from __future__ import division
import lcm
import os
import sys
sys.path.append(os.path.expanduser('~/drc/software/build/lib/python2.7/dist-packages'))
import drc
from collections import deque

"""
A very simple fall detector for the Atlas. Requires that the walking controller be running in matlab (use_mex=false). Listens to CONTROLLER_STATUS and prints a warning if a fall is detected.

Usage:
    python fall_detector.py
"""

vdot_threshold = 0.002
vdot_count_threshold = 2
vdot_memory = 10

m = lcm.LCM()

class FallDetector:
    def __init__(self):
        self.decay_rate = 0.2
        self.vdot_errror_count = 0
        self.vdot_history = deque()
        self.last_t = None

    def handle(self, channel, data):
        msg = drc.controller_status_t.decode(data)
        t = msg.utime / 1e6
        if self.last_t is None or t - self.last_t > 1 or t < 0.001:
            self.last_t = t
            self.vdot_errror_count = 0
            self.vdot_history = deque()
        else:
            if len(self.vdot_history) >= vdot_memory:
                self.vdot_errror_count -= int(self.vdot_history.popleft())
            self.vdot_history.append(msg.Vdot > vdot_threshold)
            self.vdot_errror_count += int(msg.Vdot > vdot_threshold)
            print self.vdot_history
            if self.vdot_errror_count > vdot_count_threshold:
                print "WARNING: POSSIBLE FALL at time: {:.3f}".format(t)
            self.last_t = t

f = FallDetector()

m.subscribe("CONTROLLER_STATUS", f.handle)
while True:
    m.handle()
