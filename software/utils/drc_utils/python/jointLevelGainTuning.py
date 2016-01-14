#!/usr/bin/python
import os,sys
import time
import lcm
from drc.robot_command_t import robot_command_t
from drc.joint_command_t import joint_command_t
import time
import numpy as np

'''

Utility for tuning control gains.

For a given joint, offers utilities for:
- Issuing a foh or chirp position command to that joint
- Issuing a foh or chirp force command to that joint

'''

joint = 'leftKneePitch'
mode = 'position'
signal = 'foh'

T = 30. # duration, s
dt = 0.05

# chirp specific params
amp = 0.15 # Nm or radians
chirp_f0 = 0.1 # starting freq, hz
chirp_fT = 0.5 # ending freq, hz
chirp_sign = 0 # 1: below offset, 1: above offset, 0: centered on offset

# zoh/foh
foh_vals = [0., 1., -1., 0.] # Nm or radians

# gains
k_q_p = 100.
k_q_i = 0.
k_qd_p = 10.
k_f_p = 0.
ff_qd = 0.
ff_qd_d = 0.
ff_f_d = 0.
ff_const = 0.

ts = np.arange(0, T, dt)
vals = 0. * ts
if signal == 'foh':
  per_step = ts.shape[0] / len(foh_vals)
  for val in range(len(foh_vals)-1):
    start = foh_vals[val]
    end = foh_vals[val+1]
    startstep = int(per_step*val)
    endstep = startstep + per_step
    for j in range(startstep, endstep):
      vals[j] = (end - start)*float(j - startstep) / float(endstep - startstep) + start
elif signal == 'chirp':
  print "NOT IMPLEMENTED YET"
  exit(0)

#import matplotlib.pyplot as plt
#plt.scatter(ts, vals)
#plt.show()

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

lc = lcm.LCM()

starttime = time.time()

for i in range(ts.shape[0]):
  t = ts[i]
  val = vals[i]
  print t, val

  while (time.time() - starttime < t):
    time.sleep(0.001)

  msg = robot_command_t();
  msg.num_joints = 1

  command = joint_command_t()
  command.joint_name = joint
  command.k_q_p = k_q_p
  command.k_q_i = k_q_i
  command.k_qd_p = k_qd_p
  command.k_f_p = k_f_p
  command.ff_qd = ff_qd
  command.ff_qd_d = ff_qd_d
  command.ff_f_d = ff_f_d
  command.ff_const = ff_const

  command.position = 0
  command.velocity = 0
  command.effort = 0

  if mode == 'force':
    command.effort = val
  elif mode == 'position':
    command.position = val

  msg.joint_commands.append(command)
  lc.publish("NASA_COMMAND", msg.encode())