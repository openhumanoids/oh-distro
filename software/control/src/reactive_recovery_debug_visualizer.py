#!/usr/bin/python

import lcm
import drc
from drc import reactive_recovery_debug_t
import sys
import time
from bot_lcmgl import lcmgl, GL_LINES
import numpy as np

def ppval(coefs, ts, t):
  ti = np.where(ts < t)[-1]
  if (len(ti) >= 1):
    ti = ti[-1]
  elif (len(ti) == 0):
    ti = 0;
  t_off = (t - ts[ti]);
  out = np.array([0.0]*6)
  for j in range(0, 6):
    out[j] = coefs[j, ti, 0]*(t_off**3.0) + coefs[j, ti, 1]*(t_off**2.0) + coefs[j, ti, 2]*t_off + coefs[j, ti, 3]
  return out

def handle_debug_msg(channel, data):
  msg = reactive_recovery_debug_t.decode(data)
  print("received")
  com = msg.com;
  icp = msg.icp;
  # draw com, icp
  gl.glColor3f(0.2,0.2,1.0);
  gl.sphere(com[0], com[1], 0, 0.01, 20, 20);
  gl.glColor3f(0.9,0.2,0.2);
  gl.sphere(icp[0], icp[1], 0, 0.01, 20, 20);

  coefs = np.array(msg.coefs)
  ts = np.array(msg.ts)
  if ts.shape[0] == 0:
    return

  # draw spline segments
  tsdense = np.linspace(ts[0], ts[-1], 30)
  gl.glColor3f(1.0, 0.2, 0.2);
  gl.glLineWidth(1);
  gl.glBegin(GL_LINES);
  ps = np.array([ppval(coefs, msg.ts, t) for t in tsdense]);
  for j in range(0,tsdense.size-1):
    gl.glVertex3f(ps[j,0], ps[j,1], ps[j,2]);
    gl.glVertex3f(ps[j+1,0], ps[j+1,1], ps[j+1,2]);
  gl.glEnd();

  gl.switch_buffer()


lc = lcm.LCM()
gl = lcmgl('reactive recovery debug', lc);
subscription = lc.subscribe("REACTIVE_RECOVERY_DEBUG", handle_debug_msg)
try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass