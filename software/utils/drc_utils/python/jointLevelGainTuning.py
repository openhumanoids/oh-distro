#!/usr/bin/python
import os,sys
import time
import lcm
from drc.robot_command_t import robot_command_t
from drc.joint_command_t import joint_command_t
import time
import numpy as np
import math

'''

Utility for tuning control gains.

For a given joint, offers utilities for:
- Issuing a foh or chirp position command to that joint
- Issuing a foh or chirp force command to that joint

'''

joint = 'leftAnkleRoll'
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
val_default_pose = (
  ('torsoYaw', 0.0, 30.0, 1.0),
  ('torsoPitch', 0.0, 30.0, 1.0),
  ('torsoRoll', 0.0, 30.0, 1.0),
  ('lowerNeckPitch', 0.0, 30.0, 1.0),
  ('neckYaw', 0.0, 30.0, 1.0),
  ('upperNeckPitch', 0.0, 30.0, 1.0),
  ('rightShoulderPitch', 0.300196631343, 30.0, 1.0),
  ('rightShoulderRoll', 1.25, 30.0, 1.0),
  ('rightShoulderYaw', 0.0, 30.0, 1.0),
  ('rightElbowPitch', 0.785398163397, 30.0, 1.0),
  ('rightForearmYaw', 1.571, 30.0, 1.0),
  ('rightWristRoll', 0.0, 30.0, 1.0),
  ('rightWristPitch', 0.0, 30.0, 1.0),
  ('leftShoulderPitch', 0.300196631343, 30.0, 1.0),
  ('leftShoulderRoll', -1.25, 30.0, 1.0),
  ('leftShoulderYaw', 0.0, 30.0, 1.0),
  ('leftElbowPitch', -0.785398163397, 30.0, 1.0),
  ('leftForearmYaw', 1.571, 30.0, 1.0),
  ('leftWristRoll', 0.0, 30.0, 1.0),
  ('leftWristPitch', 0.0, 30.0, 1.0),
  ('rightHipYaw', 0.0, 30.0, 1.0),
  ('rightHipRoll', 0.0, 30.0, 1.0),
  ('rightHipPitch', -0.49, 30.0, 1.0),
  ('rightKneePitch', 1.205, 30.0, 1.0),
  ('rightAnklePitch', -0.71, 30.0, 1.0),
  ('rightAnkleRoll', 0.0, 30.0, 1.0),
  ('leftHipYaw', 0.2, 30.0, 1.0),
  ('leftHipRoll', 1.0, 30.0, 1.0),
  ('leftHipPitch', -0.49, 30.0, 1.0),
  ('leftKneePitch', 1.205, 30.0, 1.0),
  ('leftAnklePitch', -0.71, 30.0, 1.0),
  ('leftAnkleRoll', 0.0, 30.0, 1.0)
)

T = 25. # duration, s
dt = 0.05

# chirp specific params
amp = 0.3 # Nm or radians
chirp_f0 = 0.1 # starting freq, hz
chirp_fT = 1.0 # ending freq, hz
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


msg = robot_command_t();  
msg.num_joints = len(val_default_pose)
command_i = -1
for i in range(msg.num_joints):
  if val_default_pose[i][0] == joint:
    command_i = i
  command = joint_command_t()
  command.joint_name = val_default_pose[i][0]
  command.k_q_p = other_k_q_p*val_default_pose[i][2]
  command.k_q_i = other_k_q_i
  command.k_qd_p = other_k_qd_p*val_default_pose[i][3]
  command.k_f_p = other_k_f_p
  command.ff_qd = other_ff_qd
  command.ff_qd_d = other_ff_qd_d
  command.ff_f_d = other_ff_f_d
  command.ff_const = other_ff_const
  command.position = val_default_pose[i][1]
  command.velocity = 0
  command.effort = 0
  msg.joint_commands.append(command)

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

  command = joint_command_t()
  command.joint_name = joint
  command.k_q_p = k_q_p*val_gains[joint][0]
  command.k_q_i = k_q_i
  command.k_qd_p = k_qd_p*val_gains[joint][1]
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

  msg.joint_commands[command_i] = command
  lc.publish("NASA_COMMAND", msg.encode())