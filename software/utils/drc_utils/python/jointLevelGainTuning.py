#!/usr/bin/python
import os,sys
import time
import lcm
from bot_core.atlas_command_t import atlas_command_t
import time
import numpy as np
import math

'''

Utility for tuning control gains.

For a given joint, offers utilities for:
- Issuing a foh or chirp position command to that joint
- Issuing a foh or chirp force command to that joint

'''

targetjoint = 'leftAnkleRoll'
mode = 'position'
signal = 'chirp'

# joint, joint position, K gain multiplier, D gain multiplier
val_gains = {
  'torsoYaw': (100.0, 1.0),
  'torsoPitch': (800.0, 1.0),
  'torsoRoll': (800.0, 1.0),
  'lowerNeckPitch': (30.0, 1.0),
  'neckYaw': (30.0, 1.0),
  'upperNeckPitch': (30.0, 1.0),
  'rightShoulderPitch': (100.0, 1.0),
  'rightShoulderRoll': (100.0, 1.0),
  'rightShoulderYaw': (50.0, 1.0),
  'rightElbowPitch': (50.0, 1.0),
  'rightForearmYaw': (30.0, 1.0),
  'rightWristRoll': (30.0, 1.0),
  'rightWristPitch': (30.0, 1.0),
  'leftShoulderPitch': (100.0, 1.0),
  'leftShoulderRoll': ( 100.0, 1.0),
  'leftShoulderYaw': (50.0, 1.0),
  'leftElbowPitch': (30.0, 1.0),
  'leftForearmYaw': (30.0, 1.0),
  'leftWristRoll': (30.0, 1.0),
  'leftWristPitch': (30.0, 1.0),
  'rightHipYaw': (50.0, 1.0),
  'rightHipRoll': (100.0, 1.0),
  'rightHipPitch': ( 200.0, 1.0),
  'rightKneePitch': (50.0, 1.0),
  'rightAnklePitch': ( 30.0, 1.0),
  'rightAnkleRoll': ( 30.0, 1.0),
  'leftHipYaw': (50.0, 1.0),
  'leftHipRoll': (100.0, 1.0),
  'leftHipPitch': ( 200.0, 1.0),
  'leftKneePitch': (50.0, 1.0),
  'leftAnklePitch': (30.0, 1.0),
  'leftAnkleRoll': (30.0, 1.0)
}
val_default_pose = {
  'torsoYaw': 0.0,
  'torsoPitch': 0.0,
  'torsoRoll': 0.0,
  'lowerNeckPitch': 0.0,
  'neckYaw': 0.0,
  'upperNeckPitch': 0.0,
  'rightShoulderPitch': 0.300196631343,
  'rightShoulderRoll': 1.25,
  'rightShoulderYaw': 0.0,
  'rightElbowPitch': 0.785398163397,
  'rightForearmYaw': 1.571,
  'rightWristRoll': 0.0,
  'rightWristPitch': 0.0,
  'leftShoulderPitch': 0.300196631343,
  'leftShoulderRoll': -1.25,
  'leftShoulderYaw': 0.0,
  'leftElbowPitch': -0.785398163397,
  'leftForearmYaw': 1.571,
  'leftWristRoll': 0.0,
  'leftWristPitch': 0.0,
  'rightHipYaw': 0.0,
  'rightHipRoll': 0.0,
  'rightHipPitch': -0.49,
  'rightKneePitch': 1.205,
  'rightAnklePitch': -0.71,
  'rightAnkleRoll': 0.0,
  'leftHipYaw': 0.2,
  'leftHipRoll': 1.0,
  'leftHipPitch': -0.49,
  'leftKneePitch': 1.205,
  'leftAnklePitch': -0.71,
  'leftAnkleRoll': 0.0
}

T = 25. # duration, s
dt = 0.05

# chirp specific params
amp = 0.3 # Nm or radians
chirp_f0 = 0.1 # starting freq, hz
chirp_fT = 0.5 # ending freq, hz
chirp_sign = 0 # 1: below offset, 1: above offset, 0: centered on offset
chirp_offset = 0.0

# zoh/foh
foh_vals = [0.0, 0.0, 0.0, 0.0] # Nm or radians

# gains for the joint of interest
k_q_p = 1
k_q_i = 0.
k_qd_p = 1.
k_f_p = 0.
ff_qd = 0.
ff_qd_d = 0.
ff_f_d = 0.
ff_const = 0.

# base gains for the other joints
other_k_q_p = 1
other_k_q_i = 0.
other_k_qd_p = 1
other_k_f_p = 0.
other_ff_qd = 0.
other_ff_qd_d = 0.
other_ff_f_d = 0.
other_ff_const = 0.


msg = atlas_command_t();  
msg.num_joints = len(val_default_pose.keys())
command_i = -1
for i, joint in enumerate(val_default_pose.keys()):
  if joint == targetjoint:
    command_i = i
  msg.joint_names.append(joint)
  msg.k_q_p.append(other_k_q_p*val_gains[joint][0])
  msg.k_q_i.append(other_k_q_i)
  msg.k_qd_p.append(other_k_qd_p*val_gains[joint][1])
  msg.k_f_p.append(other_k_f_p)
  msg.ff_qd.append(other_ff_qd)
  msg.ff_qd_d.append(other_ff_qd_d)
  msg.ff_f_d.append(other_ff_f_d)
  msg.ff_const.append(other_ff_const)
  msg.position.append(val_default_pose[joint])
  msg.velocity.append(0)
  msg.effort.append(0)

  msg.k_effort += ' '

if command_i == -1:
  print "Couldn't find the desired joint!"
  exit(0)

ts = np.arange(0, T, dt)
vals = 0. * ts
if signal == 'foh':
  per_step = ts.shape[0] / (len(foh_vals)-1)
  for val in range(len(foh_vals)-1):
    start = foh_vals[val]
    end = foh_vals[val+1]
    startstep = int(per_step*val)
    endstep = startstep + per_step+1
    for j in range(startstep, endstep):
      vals[j] = (end - start)*float(j - startstep) / float(endstep - startstep) + start
elif signal == 'chirp':
  freq = np.linspace(chirp_f0, chirp_fT, ts.shape[0])
  if chirp_sign == 0:
    vals = chirp_offset + amp * np.sin(ts * freq * 2 * math.pi)
  else:
    vals = chirp_offset + chirp_sign * (0.5*amp - 0.5 * amp * np.cos(ts * freq * 2 * math.pi))

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

  msg.joint_names[command_i] = targetjoint
  msg.k_q_p[command_i] = k_q_p*val_gains[targetjoint][0]
  msg.k_q_i[command_i] = k_q_i
  msg.k_qd_p[command_i] = k_qd_p*val_gains[targetjoint][1]
  msg.k_f_p[command_i] = k_f_p
  msg.ff_qd[command_i] = ff_qd
  msg.ff_qd_d[command_i] = ff_qd_d
  msg.ff_f_d[command_i] = ff_f_d
  msg.ff_const[command_i] = ff_const

  msg.position[command_i] = 0
  msg.velocity[command_i] = 0
  msg.effort[command_i] = 0

  if mode == 'force':
    msg.effort[command_i] = val
    msg.position[command_i] = 0
  elif mode == 'position':
    msg.position[command_i] = val
    msg.effort[command_i] = 0

  lc.publish("ROBOT_COMMAND", msg.encode())