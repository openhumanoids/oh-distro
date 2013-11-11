# flip x and z joints in arms
import os,sys
import lcm
import time
from lcm import LCM
import math

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")
from drc.joint_angles_t import joint_angles_t



def flipArms(name,position):
  #print name
  #print position
  lname = ["l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx"]
  rname = ["r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx"]
  flip  = [1          , -1         , 1          , -1         , 1          , -1]
  
  for i in range(len(name)):
    #print i
    j = rname.index(name[i])
    #print lname[j]
    name[i] = lname[j]
    position[i] = flip[j]*position[i]

  #print "after"
  #print name
  #print position
  return [name,position]





def timestamp_now (): return int (time.time () * 1000000)

def getCradle():
  position=[-2.1563617329, 0.100071714551, 0.0580735206604, -0.943417046462, 2.93858361244, 2.2]
  name = ["r_arm_elx", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy", "r_arm_ely"]
  return [position,name]

def getHang():
  position=[-1.03245399544, 1.79775392296, -1.17321766434, 0.101622587872, -1.228967243, 0.0]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getFarHang():
  position=[-0.281868335103, 1.79775392296, -1.17321766434, 0.351412541103, -1.0226471182, 0.0]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getShooter():
  position=[-2.35619, 1.59046280384, 0.0874569192529, 0.270734280348, 0.6641523242, 0.286707222462]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getPullDown():
  position=[-1.40425169468, 1.94562590122, -0.584710803126, 0.789787143469, -0.927295863628, 0.546333581209]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getHandDown():
  position=[-1.01134300232, 1.96587055922, 0.0517824155395, 1.38765757637, 0.239791974425, 0.009811]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]


def usage():
  print "exe-name positions [l|r]"
  print "cradle   - close to robot, hand upside down"
  print "hang     - hand just about cradle. thumb in"
  print "farhang  - above, further away"
  print "shooter  - hand up and finger away"
  print "pulldown - lower config below the current"
  print "handdown - handdown"


######################################
#print 'Number of arguments:', len(sys.argv), 'arguments.'
#print 'Argument List:', str(sys.argv)
if (len(sys.argv) <3):
  print "##### Specify a hand ########"
  usage()
  exit()

posture_side = sys.argv[2]
posture_name = sys.argv[1]
if (posture_side == "right"):
  posture_side = "r"
if (posture_side == "RIGHT"):
  posture_side = "r"
if (posture_side == "R"):
  posture_side = "r"

if (posture_side == "left"):
  posture_side = "l"
if (posture_side == "LEFT"):
  posture_side = "l"
if (posture_side == "L"):
  posture_side = "l"
usage()

lc = lcm.LCM()
msg = joint_angles_t()
msg.utime = timestamp_now ()

if (posture_name == "cradle"):
  [msg.joint_position, msg.joint_name] = getCradle()
elif (posture_name == "hang"):
  [msg.joint_position, msg.joint_name] = getHang()
elif (posture_name == "farhang"):
  [msg.joint_position, msg.joint_name] = getFarHang()
elif (posture_name == "shooter"):
  [msg.joint_position, msg.joint_name] = getShooter()
elif (posture_name == "pulldown"):
  [msg.joint_position, msg.joint_name] = getPullDown()
elif (posture_name == "handdown"):
  [msg.joint_position, msg.joint_name] = getHandDown()

if (posture_side == "l"):
  [msg.joint_name,msg.joint_position] = flipArms(msg.joint_name,msg.joint_position)

msg.num_joints = len(msg.joint_name)
lc.publish("POSTURE_GOAL", msg.encode())
print "\nPOSTURE_GOAL message sent"

