#!/usr/bin/python

''' Listens to QP Controller Inputs and draws, in different but
order-consistent colors, the cubic splines being followed by each
body motion block. '''

import lcm
import drc
from drake import lcmt_qp_controller_input, lcmt_body_motion_data
import sys
import time
from bot_lcmgl import lcmgl, GL_LINES
import numpy as np
 
color_order = [[1.0, 0.1, 0.1], [0.1, 1.0, 0.1], [0.1, 0.1, 1.0], [1.0, 1.0, 0.1], [1.0, 0.1, 1.0], [0.1, 1.0, 1.0]];

def pval(coefs, t_off):
  out = np.array([0.0]*6)
  for j in range(0, 6):
    out[j] = coefs[j, 0]*(t_off**3.0) + coefs[j, 1]*(t_off**2.0) + coefs[j, 2]*t_off + coefs[j, 3]
  return out

def handle_qp_controller_input_msg(channel, data):
  msg = lcmt_qp_controller_input.decode(data)
  #print("received")
  # draw spline segment for each tracked body
  for i in range(0, msg.num_tracked_bodies):
    bmd = msg.body_motion_data[i]
    ts = bmd.ts;
    tsdense = np.linspace(ts[0], ts[-1], 20);
    coefs = np.array(bmd.coefs);
    color = color_order[i%len(color_order)];
    gl.glColor3f(color[0], color[1], color[2]);
    gl.glLineWidth(5);
    gl.glBegin(GL_LINES);
    ps = np.array([pval(coefs, t-ts[0]) for t in tsdense]);
    for j in range(0,tsdense.size-1):
      gl.glVertex3f(ps[j,0], ps[j,1], ps[j,2]);
      gl.glVertex3f(ps[j+1,0], ps[j+1,1], ps[j+1,2]);
    gl.glEnd();
  gl.switch_buffer()
  


lc = lcm.LCM()
gl = lcmgl('qp input bmd snoop', lc);
subscription = lc.subscribe("QP_CONTROLLER_INPUT", handle_qp_controller_input_msg)
subscription.set_queue_capacity(1);

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass