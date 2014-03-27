from drc.robot_state_t import robot_state_t
from drc.position_3d_t import position_3d_t
from drc.vector_3d_t import vector_3d_t
from drc.quaternion_t import quaternion_t
from drc.twist_t import twist_t
from drc.force_torque_t import force_torque_t
import time
import math
import numpy as np

def timestamp_now (): return int (time.time () * 1000000)

class BotTrans:
  def __init__(self):
    self.trans_vec =np.array([0,0,0])
    self.rot_quat =np.array([1,0,0,0])
  def assign(self,trans_vec, rot_quat):
    self.trans_vec = trans_vec.copy()
    self.rot_quat = rot_quat.copy()
  def print_out(self):
    print self.trans_vec
    print self.rot_quat

############################################################################
def trans_apply_trans(src1, src):
  dest = BotTrans()
  dest.trans_vec = src1.trans_vec.copy()
  dest.rot_quat = src1.rot_quat.copy()
  dest.trans_vec = quat_rotate(src.rot_quat, dest.trans_vec);
  qtmp= quat_mult(src.rot_quat, dest.rot_quat);
  dest.rot_quat = qtmp;
  dest.trans_vec = dest.trans_vec + src.trans_vec
  return dest

def trans_invert(btrans_in):
  btrans = BotTrans()
  btrans.trans_vec = btrans_in.trans_vec.copy()
  btrans.rot_quat = btrans_in.rot_quat.copy()
  btrans.trans_vec[0] = -btrans.trans_vec[0];
  btrans.trans_vec[1] = -btrans.trans_vec[1];
  btrans.trans_vec[2] = -btrans.trans_vec[2];
  btrans.trans_vec = quat_rotate_rev(btrans.rot_quat, btrans.trans_vec);
  btrans.rot_quat[1] = -btrans.rot_quat[1];
  btrans.rot_quat[2] = -btrans.rot_quat[2];
  btrans.rot_quat[3] = -btrans.rot_quat[3];
  return btrans
  
def quat_rotate (rot, v):
  ab  =  rot[0]*rot[1]; ac  =  rot[0]*rot[2]; ad  =  rot[0]*rot[3];
  nbb = -rot[1]*rot[1]; bc  =  rot[1]*rot[2]; bd  =  rot[1]*rot[3];
  ncc = -rot[2]*rot[2]; cd  =  rot[2]*rot[3]; ndd = -rot[3]*rot[3];

  res = [
    2*( (ncc + ndd)*v[0] + (bc -  ad)*v[1] + (ac + bd)*v[2] ) + v[0] , \
    2*( (ad +  bc)*v[0] + (nbb + ndd)*v[1] + (cd - ab)*v[2] ) + v[1] , \
    2*( (bd -  ac)*v[0] + (ab +  cd)*v[1] + (nbb + ncc)*v[2] ) + v[2]];
  return np.array(res)

def quat_rotate_rev (rot, v):
  b=  np.hstack(  [np.array([0]) ,v]   )
  a = quat_mult (b, rot);
  b[0] = rot[0];
  b[1] = -rot[1];
  b[2] = -rot[2];
  b[3] = -rot[3];
  c = quat_mult (b, a);
  v = [c[1], c[2], c[3]];
  return np.array(v)
  
def quat_mult (a, b):
  c = [0,0,0,0]
  c[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  c[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  c[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  c[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
  return np.array(c)


def getDrcPosition3dAsBotTrans(pose):  
  p = BotTrans()
  p.trans_vec = np.array([pose.translation.x, pose.translation.y, pose.translation.z])
  p.rot_quat = np.array([pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z])
  return p
  
def getBotCorePose3dAsBotTrans(pose):   
  p = BotTrans()
  p.trans_vec = np.array(pose.pos)
  p.rot_quat = np.array(pose.orientation)
  return p

def getBotTransAsDrcPosition3d(pose_in):
  pose = position_3d_t()
  translation = vector_3d_t()
  translation.x =pose_in.trans_vec[0]
  translation.y =pose_in.trans_vec[1]
  translation.z =pose_in.trans_vec[2]
  rotation = quaternion_t()
  rotation.w = pose_in.rot_quat[0]
  rotation.x = pose_in.rot_quat[1]
  rotation.y = pose_in.rot_quat[2]
  rotation.z = pose_in.rot_quat[3]
  pose.translation = translation
  pose.rotation = rotation
  return pose  
  
 
  
############################################################################



def drcPosition3dToXYZRPY(pose):
  xyz = [pose.translation.x, pose.translation.y, pose.translation.z]
  rpy = quat_to_euler([pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z])
  xyzrpy=xyz
  xyzrpy.extend(rpy)
  return [xyzrpy]

  
  

  
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
  return np.array([w,x,y,z])

def getPoseMsg():
  pose = position_3d_t()
  translation = vector_3d_t()
  translation.x =0
  translation.y =0
  translation.z =0
  rotation = quaternion_t()
  rotation.w = 1
  rotation.x = 0
  rotation.y = 0
  rotation.z = 0
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


################################################################################
################################################################################
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

def flipArms(name,position):
  # nb: l back rotation and r back rotation are a simple sign change
  # "back_bkx", "back_bky" shouldn't be used as non-zeros
  lname = ["l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx","back_bkz","back_bkx", "back_bky"]
  rname = ["r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx","back_bkz","back_bkx", "back_bky"]
  flip  = [1          , -1         , 1          , -1         , 1          , -1         ,-1        ,1         ,1          ]
  for i in range(len(name)):
    j = rname.index(name[i])
    name[i] = lname[j]
    position[i] = flip[j]*position[i]
  return [name,position]


def getPostureShooter():
  position=[-2.35619, 1.35674083233, 0.0874569192529, 0.270734280348, 0.785398, 0.286707222462]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getPostureShooterBack():
  position=[0,0,0,-2.35619, 1.35674083233, 0.0874569192529, 0.270734280348, 0.785398, 0.286707222462]
  name=["back_bkx", "back_bky", "back_bkz","r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getPosturePullDown():
  position=[-1.40425169468, 1.94562590122, -0.584710803126, 0.789787143469, -0.927295863628, 0.546333581209]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getPostureHandDown():
  position=[-1.01134300232, 1.96587055922, 0.0517824155395, 1.38765757637, 0.239791974425, 0.009811]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

# End of py_drc_utils.py
