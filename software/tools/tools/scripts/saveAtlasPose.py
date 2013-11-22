#!/usr/bin/env python
import lcm
import sys
import json
import time
import os.path
from drc.robot_state_t import robot_state_t
from drc.joint_angles_t import joint_angles_t

def on_est_robot_state(channel, data):
  global filename, msgtype,shortdesc,longdesc,key,jointlist
  m = robot_state_t.decode(data)
  if not os.path.exists(filename):
    print('No database exists.  Creating a new one.')
    posedb = dict()
  else:
    posedb = json.load(open(filename, 'r'))

  posture = dict()
  posture['short'] = shortdesc
  posture['long'] = longdesc
  joints = dict();
  for i in range(len(m.joint_name)):
    joint = m.joint_name[i]
    if joint in jointlist:
      joints[joint] = m.joint_position[i]
  posture['joints'] = joints

  posedb[key] = posture

  with open(filename, 'w') as outfile:
    json.dump(posedb, outfile)
    print('Wrote pose to database')
    print json.dumps(posedb, indent=2)
  quit()

def on_posture_goal(channel, data):
  global filename, msgtype,shortdesc,longdesc,key,jointlist
  m = joint_angles_t.decode(data)
  if not os.path.exists(filename):
    print('No database exists.  Creating a new one.')
    posedb = dict()
  else:
    posedb = json.load(open(filename, 'r'))

  posture = dict()
  posture['short'] = shortdesc
  posture['long'] = longdesc
  joints = dict();
  for i in range(len(m.joint_name)):
    joint = m.joint_name[i]
    if joint in jointlist:
      joints[joint] = m.joint_position[i]
  posture['joints'] = joints

  posedb[key] = posture

  with open(filename, 'w') as outfile:
    json.dump(posedb, outfile)
    print('Wrote pose to database')
    print json.dumps(posedb, indent=2)
  quit()


def main():
  global filename, msgtype,shortdesc,longdesc,key,jointlist
  #usage saveAtlasPose.py filename msg-type joint-types name long key
  if len(sys.argv) == 2 and sys.argv[1] == '-h':
    print('saveAtlasPose.py')
    print('A simple script for saving the robot pose to a json file')
    print('usage: python saveAtlasPose.py filename msg-type joint-types name long key')
    print('  filename: append to this file if it exists, or create a new one')
    print('  msg-type: 1 for EST_ROBOT_STATE and 2 for POSTURE_GOAL. Determines what LCM message contains the state')
    print('  joint-types: specify which joints are to be saved.  (r)ight arm, (l)eft arm, and (b)ack joints are allowed, or some combination')
    print('  name: the short name of the pose')
    print('  long: the long description of the pose')
    print('  key: they key-press used to index this key. will overwrite anything else in the database')
    quit()
  if len(sys.argv) != 7:
    raise SyntaxError('Require 7 arguments.  Usage saveAtlasPose.py filename msg-type joint-types name long key')

  lc = lcm.LCM();

  filename = sys.argv[1]
  msgtype = int(sys.argv[2])
  jointtypes = sys.argv[3]  
  shortdesc = sys.argv[4]
  longdesc = sys.argv[5]
  key = sys.argv[6]

  jointlist = [];
  for c in jointtypes:
    if c=='l':
      jointlist.append('l_arm_usy');
      jointlist.append('l_arm_shx');
      jointlist.append('l_arm_ely');
      jointlist.append('l_arm_elx');
      jointlist.append('l_arm_uwy');
      jointlist.append('l_arm_mwx');
    elif c=='r':
      jointlist.append('r_arm_usy');
      jointlist.append('r_arm_shx');
      jointlist.append('r_arm_ely');
      jointlist.append('r_arm_elx');
      jointlist.append('r_arm_uwy');
      jointlist.append('r_arm_mwx');
    elif c=='b':
      jointlist.append('back_bkz');
      jointlist.append('back_bky');
      jointlist.append('back_bkx');
    else:
      raise SyntaxError('Invalid joint type character')

  if msgtype == 1:
    print('Listening for an estimated robot-state');
    lc.subscribe("EST_ROBOT_STATE", on_est_robot_state)
  elif msgtype == 2:
    print('Listening for a robot pose');
    lc.subscribe("POSTURE_GOAL", on_posture_goal)
  else:
    raise SyntaxError('Msg-type must be 1 or 2')


  while(1==1):
    lc.handle()
    time.sleep(1)
if __name__ == '__main__':
  main()
