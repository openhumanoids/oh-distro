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
from drc.robot_urdf_t import robot_urdf_t
########################################################################################

class JointNames:
    def __init__(self):
        self.left_sandia =( ["left_f0_0","left_f0_1","left_f0_2",   "left_f1_0","left_f1_1","left_f1_2",\
                            "left_f2_0","left_f2_1","left_f2_2",   "left_f3_0","left_f3_1","left_f3_2" ] )
        self.right_sandia=( ["right_f0_0","right_f0_1","right_f0_2",  "right_f1_0","right_f1_1","right_f1_2", \
                            "right_f2_0","right_f2_1","right_f2_2",  "right_f3_0","right_f3_1","right_f3_2" ] )
        self.left_irobot =( ["left_finger[0]/joint_base_rotation", "left_finger[0]/joint_base", "left_finger[0]/joint_flex", \
                            "left_finger[1]/joint_base_rotation", "left_finger[1]/joint_base", "left_finger[1]/joint_flex", \
                            "left_finger[2]/joint_base", "left_finger[2]/joint_flex" ] )
        self.right_irobot=( ["right_finger[0]/joint_base_rotation", "right_finger[0]/joint_base", "right_finger[0]/joint_flex",\
                            "right_finger[1]/joint_base_rotation", "right_finger[1]/joint_base", "right_finger[1]/joint_flex",\
                            "right_finger[2]/joint_base", "right_finger[2]/joint_flex" ] )
        self.head = ( ["pre_spindle_cal_x_joint", "pre_spindle_cal_y_joint", "pre_spindle_cal_z_joint", \
                            "pre_spindle_cal_roll_joint", "pre_spindle_cal_pitch_joint", "pre_spindle_cal_yaw_joint", \
                            "hokuyo_joint", \
                            "post_spindle_cal_x_joint", "post_spindle_cal_y_joint", "post_spindle_cal_z_joint", \
                            "post_spindle_cal_roll_joint", "post_spindle_cal_pitch_joint", "post_spindle_cal_yaw_joint" ])

def timestamp_now (): return int (time.time () * 1000000)

def quat_to_euler(q) :
  roll_a = 2.0 * (q[0]*q[1] + q[2]*q[3]);
  roll_b = 1.0 - 2.0 * (q[1]*q[1] + q[2]*q[2]);
  roll = math.atan2 (roll_a, roll_b);

  pitch_sin = 2.0 * (q[0]*q[2] - q[3]*q[1]);
  pitch = math.asin (pitch_sin);

  yaw_a = 2.0 * (q[0]*q[3] + q[1]*q[2]);
  yaw_b = 1.0 - 2.0 * (q[2]*q[2] + q[3]*q[3]);  
  yaw = math.atan2 (yaw_a, yaw_b);
  return [roll,pitch,yaw]

def euler_to_quat(roll, pitch, yaw):
  sy = math.sin(yaw*0.5);
  cy = math.cos(yaw*0.5);
  sp = math.sin(pitch*0.5);
  cp = math.cos(pitch*0.5);
  sr = math.sin(roll*0.5);
  cr = math.cos(roll*0.5);
  w = cr*cp*cy + sr*sp*sy;
  x = sr*cp*cy - cr*sp*sy;
  y = cr*sp*cy + sr*cp*sy;
  z = cr*cp*sy - sr*sp*cy;
  return [w,x,y,z]


def setStateAtHeight85BackAndPelvisPitched(msg):
  msg.pose.translation.z = 0.855
  # convert input rotation to euler
  quat_in= [msg.pose.rotation.w, msg.pose.rotation.x, msg.pose.rotation.y, msg.pose.rotation.z]
  [roll,pitch,yaw]=quat_to_euler(quat_in)
  nominal_rpy = [0.01269, 0.2714, yaw] # combine yaw with nominal leaning of robot:
  quat_out = euler_to_quat(nominal_rpy[0], nominal_rpy[1], nominal_rpy[2])
  msg.pose.rotation.w = quat_out[0]
  msg.pose.rotation.x = quat_out[1]
  msg.pose.rotation.y = quat_out[2]
  msg.pose.rotation.z = quat_out[3]
  msg.joint_position = [0.0255525112152, 0.441202163696, 0.00035834312439, 0.0319782495499, 0.0157958865166, 0.0383446216583, -0.91215968132, \
                        0.951368033886, -0.315932631493, -0.0551896877587, -0.0183086395264, -0.0554901361465, -0.92648422718, 0.953379929066, \
                        -0.280878514051, 0.0627506822348, 0.27154108882, -1.361130476, 2.00148749352, 0.515956759453, 0.0401229038835, \
                        0.000248298572842, 0.269899457693, 1.37451326847, 2.04671573639, -0.454142481089, 0.00692522525787, 0.00274819415063]
  return msg


def setStateAtHeight65BackAndPelvisPitched(msg):
  msg.pose.translation.z = 0.66
  # convert input rotation to euler
  quat_in= [msg.pose.rotation.w, msg.pose.rotation.x, msg.pose.rotation.y, msg.pose.rotation.z]
  [roll,pitch,yaw]=quat_to_euler(quat_in)
  nominal_rpy = [0.0, 0.263, yaw] # combine yaw with nominal leaning of robot:
  quat_out = euler_to_quat(nominal_rpy[0], nominal_rpy[1], nominal_rpy[2])
  msg.pose.rotation.w = quat_out[0]
  msg.pose.rotation.x = quat_out[1]
  msg.pose.rotation.y = quat_out[2]
  msg.pose.rotation.z = quat_out[3]
  msg.joint_position = [0.0102956295013, 0.447079539299, -0.00384950637817, 0.596411466599, 0.012946665287, 0.0286318063736, \
                        -1.47610342503, 1.81983685493, -0.648966550827, -0.0258723720908, -0.0262627601624, -0.0393126010895,\
                        -1.45850729942, 1.81290996075, -0.61605745554, 0.063511967659, 0.253438413143, -1.32485473156, 2.00952410698, \
                        0.458183526993, 0.0392559692264, -0.003483354114, 0.326177358627, 1.38256680965, 2.02945828438, -0.434584200382, 0.0155538320541, 0.0141570083797]
  return msg

# ... arm also extended
def setStateAtHeight65BackAndPelvisPitchedAndArmExtended(msg):
  msg.pose.translation.z = 0.67
  # convert input rotation to euler
  quat_in= [msg.pose.rotation.w, msg.pose.rotation.x, msg.pose.rotation.y, msg.pose.rotation.z]
  [roll,pitch,yaw]=quat_to_euler(quat_in)
  nominal_rpy = [0.0, 0.278, yaw] # combine yaw with nominal leaning of robot:
  quat_out = euler_to_quat(nominal_rpy[0], nominal_rpy[1], nominal_rpy[2])
  msg.pose.rotation.w = quat_out[0]
  msg.pose.rotation.x = quat_out[1]
  msg.pose.rotation.y = quat_out[2]
  msg.pose.rotation.z = quat_out[3]
  msg.joint_position = [0.0150854587555, 0.481599569321, -0.00544047355652, 0.596459150314, 0.0206943154335, 0.00824320316315, -1.51817989349, \
                        1.72192263603, -0.49567347765, -0.00896684732288, -0.0317978858948, -0.0276851654053, -1.50690329075, 1.71808087826, \
                        -0.464701503515, 0.050920818001, 0.25139734149, -1.3262168169, 2.01269006729, 0.464060992002, 0.0487913787365, -0.00783698540181, \
                        -1.36388576031, 0.381931900978, 2.0297460556, -0.429119378328, 0.0155538320541, 0.0141570083797]
  return msg

def setStateAtHeight86(msg):
  msg.pose.translation.z = 0.858534753323
  # orientation, 0.00342719585229, 0.00504788873763, 0.103853363782,
  msg.joint_position = [0.00432538986206, 0.000792384147644, -0.0010027885437, 1.14905309677, 0.000830829143524, 0.0304319858551, -0.495565026999, \
                        1.01119089127, -0.54089897871, -0.0268067382276, -0.00381422042847, -0.0367293357849, -0.494196861982, 0.994021117687, \
                        -0.524335980415, 0.0539663769305, 0.280060052872, -1.30356013775, 2.00733232498, 0.483818888664, 0.0282346289605, \
                        -0.00199056160636, 0.309303581715, 1.35600960255, 2.05246806145, -0.442733466625, 0.00692522525787, 0.145126670599 ]
  return msg

def setStateAtHeight80(msg):
  msg.pose.translation.z = 0.794386198521
  # -0.00225271177662, 0.00527569419577, 0.094212812089, 

  msg.joint_position = [0.004677157402, 0.00342667102814, -0.00474905967712, 1.14958667755, 0.00408941507339, 0.0368480682373, \
                        -0.681962370872, 1.34392225742, -0.680823981762, -0.0329969972372, -0.00800085067749, -0.0247997045517, \
                        -0.650174021721, 1.32475161552, -0.687993586063, 0.0449704453349, 0.256810456514, -1.30653166771, 2.03570413589, \
                        0.458308488131, 0.0489153526723, -0.0120663093403, 0.30805721879, 1.36914455891, 2.0452775, -0.40361696, 0.0068293809, -1.0575143]
  return msg

def setStateAtHeight75(msg):
  msg.pose.translation.z = 0.732506911755
  #, -0.000808249484968, -0.00102079036455, 0.0933028354976, 
  msg.joint_position = [0.004677157402, 0.00316667556763, -0.00521075725555, 1.14958667755, 0.00421768426895, 0.0511291027069, \
                        -0.804708003998, 1.56003177166, -0.770609498024, -0.0488710962236, -0.0078706741333, -0.0163278579712, \
                        -0.775656223297, 1.54737138748, -0.783533096313, 0.0399675630033, 0.255834490061, -1.30554103851, 2.036921978, \
                        0.455057352781, 0.053745046258, -0.0125637603924, 0.30805721879, 1.37250006199, 2.04537343979, -0.403616964, 0.00682938, -1.05751430]
  return msg

def setStateAtHeight70(msg):
  msg.pose.translation.z = 0.700085043907
  #-0.00262847501533, 0.00501577123118, 0.0927118574385,
  msg.joint_position = [0.004677157402, 0.00349199771881, -0.00572419166565, 1.14974021912, 0.00549167394638, 0.0557951927185, \
                        -0.906586408615, 1.75183320045, -0.869903743267, -0.0556065551937, -0.00715255737305, -0.0153464078903, \
                        -0.873906850815, 1.73374772072, -0.883218050003, 0.0490799583495, 0.255567997694, -1.30838859081, 2.03509545326, \
                        0.451305687428, 0.050525251776, -0.0110711865127, 0.307961344, 1.37230837345, 2.04546928, -0.404479831, 0.0068293, -1.05751430988]
  return msg

def setStateAtHeight66(msg):
  msg.pose.translation.z = 0.655467739105
  # -0.00185047180292, 0.00574357871503, 0.0934642327473,
  msg.joint_position = [0.004677157402, 0.00485932826996, -0.00669956207275, 1.14996910095, 0.00459951162338, 0.0668022632599, \
                        -0.965375542641, 1.86640143394, -0.928911805153, -0.0676346570253, -0.00748014450073, -0.0108672380447, \
                        -0.932114124298, 1.85122585297, -0.945250093937, 0.0485419183969, 0.248025298119, -1.30591237545, 2.04970741272, \
                        0.446678996086, 0.0600606650114, -0.0167930833995, 0.30805721879, 1.37700617313, 2.04546928406, -0.38981112, 0.006733536, -1.032395]
  return msg

def appendJoints(msg,joint_names):
  msg.num_joints = msg.num_joints + len(joint_names)
  msg.joint_position.extend ( [0]*len(joint_names) )
  msg.joint_velocity.extend ( [0]*len(joint_names) )
  msg.joint_effort.extend ( [0]*len(joint_names) )
  msg.joint_name.extend(joint_names)
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
  global goal_pelvis_height, goal_pelvis_pitch, goal_pelvis_roll, goal_pos, goal_xy, goal_hand_config, jnames
  if (goal_hand_config[0] == -1):
    print "no hand config, not publishing ERS"
    return

  msg = getRobotStateMsg()
  quat_out = euler_to_quat(0,0, goal_yaw)
  msg.pose.rotation.w = quat_out[0]
  msg.pose.rotation.x = quat_out[1]
  msg.pose.rotation.y = quat_out[2]
  msg.pose.rotation.z = quat_out[3]
  msg.pose.translation.x = goal_xy[0]
  msg.pose.translation.y = goal_xy[1]
  print "Pelvis Height: " , goal_pelvis_height
  if (goal_pelvis_roll > 0.1):
    print "Using Leaning Pitch and Reach"
    msg = setStateAtHeight65BackAndPelvisPitchedAndArmExtended(msg)
  elif (goal_pelvis_pitch > 0.1):
    print "Using Leaning Pitch"
    if (goal_pelvis_height >  0.75 ):
      msg = setStateAtHeight85BackAndPelvisPitched(msg)
    else:
      msg = setStateAtHeight65BackAndPelvisPitched(msg)
  elif (goal_pelvis_height >  0.83 ):
    msg = setStateAtHeight86(msg)
  elif (goal_pelvis_height >  0.775 ):
    msg = setStateAtHeight80(msg)
  elif (goal_pelvis_height >  0.725 ):
    msg = setStateAtHeight75(msg)
  elif (goal_pelvis_height >  0.68 ):
    msg = setStateAtHeight70(msg)
  else:
    msg = setStateAtHeight66(msg)

  msg = appendJoints(msg, jnames.head)
  if (goal_hand_config[0] == 2):
    #print "use left sandia"
    appendJoints(msg,jnames.left_sandia)
  elif (goal_hand_config[0] == 4):
    #print "use left irobot"
    appendJoints(msg,jnames.left_irobot)

  if (goal_hand_config[1] == 3):
    #print "use right sandia"
    msg = appendJoints(msg,jnames.right_sandia)
  elif (goal_hand_config[1] == 5):
    #print "use right irobot"
    msg = appendJoints(msg,jnames.right_irobot)

  lc.publish("EST_ROBOT_STATE", msg.encode())


def on_manip_params(channel, data):
  global goal_pelvis_height,goal_pelvis_pitch,goal_pelvis_roll, goal_yaw, goal_xy
  m = atlas_behavior_manipulate_params_t.decode(data)
  goal_pelvis_height = m.desired.pelvis_height
  goal_pelvis_pitch = m.desired.pelvis_pitch
  goal_pelvis_roll = m.desired.pelvis_roll # use roll to signify arm reach
  print "Changing height and pitch"
  sendRobotStateMsg()
  
def on_walking_goal(channel, data):
  global goal_pelvis_height, goal_yaw, goal_xy
  m = walking_goal_t.decode(data)
  quat_in= [m.goal_pos.rotation.w, m.goal_pos.rotation.x, m.goal_pos.rotation.y, m.goal_pos.rotation.z]
  [roll,pitch,goal_yaw]=quat_to_euler(quat_in)
  goal_xy = [m.goal_pos.translation.x, m.goal_pos.translation.y]
  sendRobotStateMsg()

  
def on_robot_model(channel, data):
  global goal_hand_config
  m = robot_urdf_t.decode(data)
  goal_hand_config[0] =  m.left_hand
  goal_hand_config[1] =  m.right_hand
  #print "got hands",str(goal_hand_config)

#################################################################################

print 'drc-fake-robot-state'
#print 'Number of arguments:', len(sys.argv), 'arguments.'
#print 'Argument List:', str(sys.argv)


lc = lcm.LCM()
print "started"

goal_xy = [0,0]
goal_yaw = 0
goal_pelvis_height = 0
goal_pelvis_pitch = 0
goal_pelvis_roll = 0
goal_hand_config = [-1]*2
jnames = JointNames()

def lcm_thread():
  sub0 = lc.subscribe("ROBOT_MODEL", on_robot_model) 
  sub1 = lc.subscribe("ATLAS_MANIPULATE_PARAMS", on_manip_params) 
  sub2 = lc.subscribe("WALKING_GOAL", on_walking_goal) 
  while True:
    ## Handle LCM if new messages have arrived.
    lc.handle()

  lc.unsubscribe(sub0)
  lc.unsubscribe(sub1)
  lc.unsubscribe(sub2)

t2 = Thread(target=lcm_thread)
t2.start()

sleep_timing=0.5 # time between updates of the plots - in wall time
while (1==1):
  time.sleep(sleep_timing)
  sendRobotStateMsg()

