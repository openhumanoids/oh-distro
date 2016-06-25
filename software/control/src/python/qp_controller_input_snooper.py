#!/usr/bin/python

''' Listens to QP Controller Inputs and draws, in different but
order-consistent colors, the cubic splines being followed by each
body motion block. '''

import lcm
import drc
from drake import lcmt_qp_controller_input, lcmt_body_motion_data
from bot_core import robot_state_t
import sys
import time
from bot_lcmgl import lcmgl, GL_LINES
import numpy as np
import math

lc = lcm.LCM()
gl = lcmgl('qp input bmd snoop', lc);

color_order = [[1.0, 0.1, 0.1], [0.1, 1.0, 0.1], [0.1, 0.1, 1.0], [1.0, 1.0, 0.1], [1.0, 0.1, 1.0], [0.1, 1.0, 1.0]];
t_global = 0;

def pval(coefs, t_off):
  out = 0.;
  for j in range(0, coefs.size):
    out += coefs[j]*(t_off**j)
  return out

def handle_robot_state_msg(channel, data):
  global t_global
  msg = robot_state_t.decode(data)
  t_global = msg.utime/1E6;

def handle_qp_controller_input_msg(channel, data):
  msg = lcmt_qp_controller_input.decode(data)
  #print("received")
  # draw spline segment for each tracked body
  min_body_z = np.inf
  for i in range(0, msg.num_tracked_bodies):
    bmd = msg.body_motion_data[i]
    ts = bmd.spline.breaks;
    t_clamped = max(min(t_global, ts[-1]), ts[0]);
    color = color_order[i%len(color_order)];
    for j in range(0, bmd.spline.num_segments):
      if (math.isinf(ts[j+1])):
        continue
      tsdense = np.linspace(ts[j], ts[j+1], 20);
      ps = np.zeros((tsdense.size,
                      bmd.spline.polynomial_matrices[j].rows,
                      bmd.spline.polynomial_matrices[j].cols))
      for k in range(0, bmd.spline.polynomial_matrices[j].rows):
        for l in range(0, bmd.spline.polynomial_matrices[j].cols):
          coefs = np.array(bmd.spline.polynomial_matrices[j].polynomials[k][l].coefficients);
          ps[:, k, l] = [pval(coefs, t-ts[j]) for t in tsdense]

      gl.glColor3f(color[0], color[1], color[2]);
      gl.glLineWidth(5);
      gl.glBegin(GL_LINES);
      for k in range(0,tsdense.size-1):
        gl.glVertex3f(ps[k,0], ps[k,1], ps[k,2]);
        gl.glVertex3f(ps[k+1,0], ps[k+1,1], ps[k+1,2]);
      gl.glEnd();

      if (t_clamped >= tsdense[0] and t_clamped <= tsdense[-1]):
        # make marker indicating current tracking position
        gl.glColor3f(0.9,0.2,0.9);
        ctp = np.zeros((bmd.spline.polynomial_matrices[j].rows,
                      bmd.spline.polynomial_matrices[j].cols))
        for k in range(0, bmd.spline.polynomial_matrices[j].rows):
          for l in range(0, bmd.spline.polynomial_matrices[j].cols):
            coefs = np.array(bmd.spline.polynomial_matrices[j].polynomials[k][l].coefficients);
            ctp[k, l] = pval(coefs, t_clamped-ts[j])
        gl.sphere(ctp[0], ctp[1], ctp[2], 0.005, 20, 20);
        min_body_z = min(min_body_z, ctp[2])

  gl.glColor3f(0, 1, 0)
  gl.sphere(msg.zmp_data.y0[0][0], msg.zmp_data.y0[1][0], min_body_z - 0.1, 0.01, 20, 20)

  gl.switch_buffer()


def main():
  subscription = lc.subscribe("QP_CONTROLLER_INPUT", handle_qp_controller_input_msg)
  subscriptionstate = lc.subscribe("EST_ROBOT_STATE", handle_robot_state_msg)
  subscription.set_queue_capacity(1);
  subscriptionstate.set_queue_capacity(1);
  print "snooper ready"

  try:
    while True:
      lc.handle()
  except KeyboardInterrupt:
    pass

if __name__ == '__main__':
  main()
