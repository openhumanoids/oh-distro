#!/usr/bin/python
# todo: add the pelvis pitch to the yaw from the gui

import os,sys
import lcm
import time
from lcm import LCM
import math
from threading import Thread

home_dir =os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")
from drc.robot_state_t import robot_state_t
from drc.position_3d_t import position_3d_t
from drc.vector_3d_t import vector_3d_t
from drc.quaternion_t import quaternion_t
from drc.twist_t import twist_t
from drc.force_torque_t import force_torque_t
from drc.walking_goal_t import walking_goal_t
from drc.atlas_behavior_manipulate_params_t import atlas_behavior_manipulate_params_t
########################################################################################

def timestamp_now (): return int (time.time () * 1000000)

#full height, 0.86
def setStateAtHeight86(msg):
  #msg.pose.translation.x = 0.610854804516
  #msg.pose.translation.y = -0.277687519789
  msg.pose.translation.z = 0.858534753323
  # orientation, 0.00342719585229, 0.00504788873763, 0.103853363782,

  msg.joint_position = [0.00432538986206, 0.000792384147644, -0.0010027885437, 1.14905309677, 0.000830829143524, 0.0304319858551, -0.495565026999, \
                        1.01119089127, -0.54089897871, -0.0268067382276, -0.00381422042847, -0.0367293357849, -0.494196861982, 0.994021117687, \
                        -0.524335980415, 0.0539663769305, 0.280060052872, -1.30356013775, 2.00733232498, 0.483818888664, 0.0282346289605, \
                        -0.00199056160636, 0.309303581715, 1.35600960255, 2.05246806145, -0.442733466625, 0.00692522525787, 0.145126670599 ]
  return msg

#0.80 back turned
def setStateAtHeight80(msg):
  #0.790696501732, -0.275361269712, 
  msg.pose.translation.z = 0.794386198521
  # -0.00225271177662, 0.00527569419577, 0.094212812089, 

  msg.joint_position = [0.004677157402, 0.00342667102814, -0.00474905967712, 1.14958667755, 0.00408941507339, 0.0368480682373, \
                        -0.681962370872, 1.34392225742, -0.680823981762, -0.0329969972372, -0.00800085067749, -0.0247997045517, \
                        -0.650174021721, 1.32475161552, -0.687993586063, 0.0449704453349, 0.256810456514, -1.30653166771, 2.03570413589, \
                        0.458308488131, 0.0489153526723, -0.0120663093403, 0.30805721879, 1.36914455891, 2.04527759552, -0.403616964817, 0.00682938098907, -1.05751430988]
  return msg

#0.75 back turned
def setStateAtHeight75(msg):
  #0.778320133686, -0.284947812557, 
  msg.pose.translation.z = 0.732506911755
  #, -0.000808249484968, -0.00102079036455, 0.0933028354976, 

  msg.joint_position = [0.004677157402, 0.00316667556763, -0.00521075725555, 1.14958667755, 0.00421768426895, 0.0511291027069, \
                        -0.804708003998, 1.56003177166, -0.770609498024, -0.0488710962236, -0.0078706741333, -0.0163278579712, \
                        -0.775656223297, 1.54737138748, -0.783533096313, 0.0399675630033, 0.255834490061, -1.30554103851, 2.036921978, \
                        0.455057352781, 0.053745046258, -0.0125637603924, 0.30805721879, 1.37250006199, 2.04537343979, -0.403616964817, 0.00682938098907, -1.05751430988]
  return msg

#0.70 back turned
def setStateAtHeight70(msg):
  #0.779415488243, -0.284410715103, 
  msg.pose.translation.z = 0.700085043907
  #-0.00262847501533, 0.00501577123118, 0.0927118574385,

  msg.joint_position = [0.004677157402, 0.00349199771881, -0.00572419166565, 1.14974021912, 0.00549167394638, 0.0557951927185, \
                        -0.906586408615, 1.75183320045, -0.869903743267, -0.0556065551937, -0.00715255737305, -0.0153464078903, \
                        -0.873906850815, 1.73374772072, -0.883218050003, 0.0490799583495, 0.255567997694, -1.30838859081, 2.03509545326, \
                        0.451305687428, 0.050525251776, -0.0110711865127, 0.307961344719, 1.37230837345, 2.04546928406, -0.404479831457, 0.00682938098907, -1.05751430988]
  return msg

#0.66 back turned
def setStateAtHeight66(msg):
  #0.780104577541, -0.28855150938, 
  msg.pose.translation.z = 0.665467739105
  # -0.00185047180292, 0.00574357871503, 0.0934642327473,

  msg.joint_position = [0.004677157402, 0.00485932826996, -0.00669956207275, 1.14996910095, 0.00459951162338, 0.0668022632599, \
                        -0.965375542641, 1.86640143394, -0.928911805153, -0.0676346570253, -0.00748014450073, -0.0108672380447, \
                        -0.932114124298, 1.85122585297, -0.945250093937, 0.0485419183969, 0.248025298119, -1.30591237545, 2.04970741272, \
                        0.446678996086, 0.0600606650114, -0.0167930833995, 0.30805721879, 1.37700617313, 2.04546928406, -0.389811128378, 0.00673353672028, -1.03239548206]
  return msg

def quat_to_euler(q) :
  
  roll_a = 2.0 * (q[0]*q[1] + q[2]*q[3]);
  roll_b = 1.0 - 2.0 * (q[1]*q[1] + q[2]*q[2]);
  roll = math.atan2 (roll_a, roll_b);

  pitch_sin = 2.0 * (q[0]*q[2] - q[3]*q[1]);
  pitch = math.asin (pitch_sin);

  yaw_a = 2.0 * (q[0]*q[3] + q[1]*q[2]);
  yaw_b = 1.0 - 2.0 * (q[2]*q[2] + q[3]*q[3]);  
  yaw = math.atan2 (yaw_a, yaw_b);
  
  #a = q[0]
  #b = q[1]
  #c = q[2]  
  #d = q[3]  
  #roll = math.atan2(2.0*(c*d + b*a),a*a-b*b-c*c+d*d);
  #pitch = math.asin(-2.0*(b*d - a*c));
  #yaw = math.atan2(2.0*(b*c + d*a),a*a+b*b-c*c-d*d);
  
  #roll = math.atan2( 2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
  #pitch = math.asin(2*(q[0]*q[2]-q[3]*q[1]));
  #yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
  return [roll,pitch,yaw]

def getPoseMsg():
  pose = position_3d_t()
  translation = vector_3d_t()
  rotation = quaternion_t()
  rotation.w = 1
  pose.translation = translation
  pose.rotation = rotation
  return pose

def getTwistMsg():
  twist = twist_t()
  linv = vector_3d_t()
  angv = vector_3d_t()
  twist.linear_velocity = linv
  twist.angular_velocity = angv
  return twist

def getRobotStateMsg():
  msg = robot_state_t()
  msg.utime = timestamp_now ()
  
  ft = force_torque_t()
  msg.pose = getPoseMsg()
  msg.twist = getTwistMsg()
  msg.force_torque = ft

  msg.num_joints = 28
  msg.joint_position = [0]*28
  msg.joint_velocity = [0]*28
  msg.joint_effort = [0]*28

  msg.joint_name = ['back_bkz', 'back_bky', 'back_bkx', 'neck_ay', 'l_leg_hpz', 'l_leg_hpx',\
                    'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx', 'r_leg_hpz', 'r_leg_hpx',\
                    'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx', 'l_arm_usy', 'l_arm_shx',\
                    'l_arm_ely',      'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx', 'r_arm_usy', 'r_arm_shx',\
                    'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx']
  return msg  
  
def sendRobotStateMsg():
  global goal_pelvis_height, goal_pos
  msg = getRobotStateMsg()
  msg.pose = goal_pose
  print "Pelvis Height: " , goal_pelvis_height
  if (goal_pelvis_height >  0.83 ):
    msg = setStateAtHeight86(msg)
  elif (goal_pelvis_height >  0.775 ):
    msg = setStateAtHeight80(msg)
  elif (goal_pelvis_height >  0.725 ):
    msg = setStateAtHeight75(msg)
  elif (goal_pelvis_height >  0.68 ):
    msg = setStateAtHeight70(msg)
  else:
    msg = setStateAtHeight66(msg)

  lc.publish("EST_ROBOT_STATE", msg.encode())


def on_manip_params(channel, data):
  global goal_pelvis_height, goal_pose
  m = atlas_behavior_manipulate_params_t.decode(data)
  goal_pelvis_height = m.desired.pelvis_height
  print "Changing height"
  sendRobotStateMsg()
  
def on_walking_goal(channel, data):
  global goal_pelvis_height, goal_pose
  m = walking_goal_t.decode(data)
  goal_pose = m.goal_pos
  sendRobotStateMsg()

  

#################################################################################

lc = lcm.LCM()
print "started"

goal_pose = getPoseMsg()
goal_pelvis_height = 0
#msg.pos = ( val, 0, 0)
#msg.orientation = (1, 0, 0, 0)




def lcm_thread():
  sub1 = lc.subscribe("ATLAS_MANIPULATE_PARAMS", on_manip_params) 
  sub2 = lc.subscribe("WALKING_GOAL", on_walking_goal) 
  while True:
    ## Handle LCM if new messages have arrived.
    lc.handle()

  lc.unsubscribe(sub1)
  lc.unsubscribe(sub2)

t2 = Thread(target=lcm_thread)
t2.start()

sleep_timing=0.5 # time between updates of the plots - in wall time
while (1==1):
  time.sleep(sleep_timing)
  sendRobotStateMsg()

