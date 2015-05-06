#!/usr/bin/python

import lcm
import drc
from drc.utime_t import utime_t

msg = boolean_t()
msg.utime = 0;
msg.data = True
lc = lcm.LCM()
lc.publish("RECOVERY_TRIGGER", msg.encode())