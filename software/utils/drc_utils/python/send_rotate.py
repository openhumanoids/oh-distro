#!/usr/bin/python
import os,sys
import time
import datetime
import subprocess

import lcm
import multisense as lcmmultisense

print "drc-send-rotate: "

print "Waiting 5 seconds..."
lc = lcm.LCM()
time.sleep(5)
msg = lcmmultisense.command_t()
msg.utime = time.time () * 1000000
msg.fps = 15
msg.gain = -1
msg.exposure_us = 10000
msg.agc = -1
msg.rpm = 5
msg.leds_flash = False
msg.leds_duty_cycle = 0
lc.publish("MULTISENSE_COMMAND", msg.encode())
print "Publishing Multisense command to spin at 5rpm!"

