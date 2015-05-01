#!/usr/bin/python

import lcm
import drc
from drc.utime_t import utime_t

msg = utime_t()
msg.utime = 0;
lc = lcm.LCM()
lc.publish("RECOVERY_TRIGGER_OFF", msg.encode())