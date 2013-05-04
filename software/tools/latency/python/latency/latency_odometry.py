import os,sys
import lcm
import time
from lcm import LCM
from drc_latency import Latency

home_dir =os.getenv("HOME")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")
from drc.robot_state_t import robot_state_t
from drc.joint_command_t import joint_command_t

def on_true_robot_state(channel, data):
  m = robot_state_t.decode(data)
  js_time = int (m.utime*1E3)
  #print js_time
  s.add_from(js_time, s.timestamp_now_sec())

def on_joint_commands(channel, data):
  m = joint_command_t.decode(data)
  jc_time = int (m.utime*1E3)
  s.add_to(jc_time, s.timestamp_now_sec(),"ODO")

s = Latency(0, 0)
lc = lcm.LCM()
lc.subscribe("EST_ROBOT_STATE", on_true_robot_state)
lc.subscribe("JOINT_COMMANDS", on_joint_commands) 
while True:
  lc.handle()

