from __future__ import division
import time
import lcm
import os
import sys
import drc
import argparse
from collections import deque

"""
A very simple fall detector for the Atlas. Requires that the walking controller be running in matlab (use_mex=false). Listens to CONTROLLER_STATUS and attempts to set the robot to its brace position if a fall is detected.

Usage:
    python fall_detector.py
"""

DEFAULTS = {'vdot_threshold': 0.5,
            'vdot_count_threshold': 5,
            'vdot_memory': 10,
            'retransmit_delay_s': 5}

m = lcm.LCM()

class FallDetector:
    def __init__(self, opts):
        self.opts = opts
        self.vdot_errror_count = 0
        self.vdot_history = deque()
        self.last_t = None
        self.last_publish_time = 0

    def handle(self, channel, data):
        msg = drc.controller_status_t.decode(data)
        t = msg.utime / 1e6
        if self.last_t is None or t - self.last_t > 1 or t < 0.001:
            self.last_t = t
            self.vdot_errror_count = 0
            self.vdot_history = deque()
        else:
            if len(self.vdot_history) >= self.opts.memory:
                self.vdot_errror_count -= int(self.vdot_history.popleft())
            err = msg.Vdot > self.opts.vdot_threshold
            # print msg.Vdot
            if err:
                print "Positive Vdot ({:.5f}) at time: {:.3f}".format(msg.Vdot, t)
            self.vdot_history.append(err)
            self.vdot_errror_count += int(err)
            if self.vdot_errror_count > self.opts.count_threshold:
                print "WARNING: Fall detected at time: {:.3f}".format(t)
                print "Bracing NOW"
                if time.time() - self.last_publish_time > self.opts.retransmit_delay:
                    brace_msg = drc.utime_t()
                    brace_msg.utime = 0
                    m.publish("BRACE_FOR_FALL", brace_msg.encode())
                    status_msg = drc.system_status_t()
                    status_msg.utime = msg.utime
                    status_msg.system = 8
                    status_msg.importance = 0
                    status_msg.frequency = 0
                    status_msg.value = 'Possible fall detected. Bracing now!'
                    m.publish('SYSTEM_STATUS', status_msg.encode())
                    self.last_publish_time = time.time()
            self.last_t = t

def main():
    parser = argparse.ArgumentParser(description='Try to detect falls from controller status and trigger switch to bracing controller. Fall detection will be triggered if more than C values of vdot are above V for the last M controller cycles.')
    parser.add_argument('--vdot_threshold', '-v', metavar='V', action='store', type=float, default=DEFAULTS['vdot_threshold'])
    parser.add_argument('--count_threshold', '-c', metavar='C', action='store', type=float, default=DEFAULTS['vdot_count_threshold'], help='')
    parser.add_argument('--memory', '-m', metavar='M', action='store', type=int, default=DEFAULTS['vdot_memory'])
    parser.add_argument('--retransmit_delay', '-r', metavar='R', action='store', type=float, default=DEFAULTS['retransmit_delay_s'], help='Time (s) before sending another BRACE_FOR_FALL message')
    opts = parser.parse_args()

    f = FallDetector(opts)

    m.subscribe("CONTROLLER_STATUS", f.handle)
    print "Watching for falls..."
    while True:
        m.handle()

if __name__ == '__main__':
    main()
