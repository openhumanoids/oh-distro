#!/usr/bin/python

import lcm
import drc
from drc.utime_t import utime_t

msg = recovery_trigger_t()
msg.activate = True
msg.override = True
msg.utime = 0
lc = lcm.LCM()
lc.publish("RECOVERY_TRIGGER", msg.encode())