#!/usr/bin/python
import roslib; roslib.load_manifest('tutorial_atlas_control')
import rospy, yaml, sys
import csv_parse
import carry_out_sequence 

if __name__ == '__main__':
  if len(sys.argv) < 2:
    print "usage: traj_yaml.py crawling_gains.csv integer"
    print "  where crawling_gains.csv is the gains and positions file"
    sys.exit(1)

  filename = sys.argv[1]
  which_steps=0 # 0 means all. positive number means a specific row. -1 means all in reverse
  if len(sys.argv) == 3:
    which_steps = sys.argv[2] 
    
  carry_out_sequence.carry_out_sequence(filename, which_steps, 1) # real time range hack
