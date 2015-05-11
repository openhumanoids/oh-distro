#!/usr/bin/python

import lcm
import drc
from drake import lcmt_force_torque
import sys
import time
from drc.utime_t import utime_t

if len(sys.argv) != 5:
	print "Usage: publish_disturbance_....py <x> <y> <z> <duration>"
	print "   If duration 0 or negative, does not publish zero force"
	print "   afterwards."
	exit(0)

x = float(sys.argv[1])
y = float(sys.argv[2])
z = float(sys.argv[3])
dur = float(sys.argv[4])

lc = lcm.LCM()

msg = lcmt_force_torque();
msg.timestamp = 0;
msg.fx = x;
msg.fy = y;
msg.fz = z;
msg.tx = 0;
msg.ty = 0;
msg.tz = 0;

starttime = time.time();

if (dur <= 0):
	lc.publish("SIM_WRENCH_INPUT", msg.encode())
	exit(0);

while (time.time() - starttime < dur):
	lc.publish("SIM_WRENCH_INPUT", msg.encode())
	time.sleep(0.05)

msg.fx = 0;
msg.fy = 0;
msg.fz = 0;
lc.publish("SIM_WRENCH_INPUT", msg.encode())

msg = recovery_trigger_t()
msg.activate = True
msg.override = True
msg.utime = 0
lc = lcm.LCM()
lc.publish("RECOVERY_TRIGGER", msg.encode())