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
  # nb: l back rotation and r back rotation are a simple sign change
  # "back_bkx", "back_bky" shouldn't be used as non-zeros
  lname = ["l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx","back_bkz","back_bkx", "back_bky"]
  rname = ["r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx","back_bkz","back_bkx", "back_bky"]
  flip  = [1          , -1         , 1          , -1         , 1          , -1         ,-1        ,1         ,1          ]
  
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
  position=[-2.35619, 1.35674083233, 0.0874569192529, 0.270734280348, 0.785398, 0.286707222462]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getShooterBack():
  position=[0,0,0,-2.35619, 1.35674083233, 0.0874569192529, 0.270734280348, 0.785398, 0.286707222462]
  name=["back_bkx", "back_bky", "back_bkz","r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getPullDown():
  position=[-1.40425169468, 1.94562590122, -0.584710803126, 0.789787143469, -0.927295863628, 0.546333581209]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getHandDown():
  position=[-1.01134300232, 1.96587055922, 0.0517824155395, 1.38765757637, 0.239791974425, 0.009811]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]


def getCrane():
  position=[-1.61829870194, 2.56434223056, -1.10402787743, -0.796537280083, -0.773799479008, 0.727126836777]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getTurnBack():
  position=[-0.663225]
  name=["back_bkz"]
  return [position,name]

def getZeroBack():
  position=[0.0, 0.0, 0.0]
  name=["back_bkx", "back_bky", "back_bkz"]
  return [position,name]

def getRetract():
  position=[-0.663225, -0.0359195768833, 1.35790967941, 0.734279990196, 0.168820291758, 0.767885386944, 0.0597184896469]
  name=["back_bkz", "r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getHoseMate():
  position=[-0.899628996849, 3.14159, -0.848963201046, -0.0953866541386, -0.739547431469, 0.199334427714]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getPoolQueue():
  position=[-0.6583873,-0.493037, 3.08459508499, 1.1543736, 0.276259120, 0.785398000, 2.769477100]
  name=["back_bkz", "r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position, name]



# 0.71cm - 5foot
#def getHighValve():
#  position=[-1.36646643681, 2.5302273035, 0.003712019324, -0.867207699585, -0.961804687977, 3.14159]
#  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
#  return [position,name]

def usage():
  print "exe-name positions [l|r]"
  print "cradle      - close to robot, hand upside down"
  print "crane       - hand pointing down from in front"
  print "farhang     - above, further away"
  print "hang        - hand just about cradle. thumb in"
  print "handdown    - handdown"
  print "pulldown    - lower config below the current"
  print "shooter     - hand up and finger away"
  print "turnback    - turn the back z joint"
  print "zero        - zero the back joint"
  print "retract     - turn back and retract hand"
  print "hosemate    - config mate"
  print "shooterback - hand up and finger away, back straight"
  print "poolqueue   - draw hand back with face down"
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
elif (posture_name == "shooterback"):
  [msg.joint_position, msg.joint_name] = getShooterBack()
elif (posture_name == "pulldown"):
  [msg.joint_position, msg.joint_name] = getPullDown()
elif (posture_name == "handdown"):
  [msg.joint_position, msg.joint_name] = getHandDown()
elif (posture_name == "crane"):
  [msg.joint_position, msg.joint_name] = getCrane()
elif (posture_name == "turnback"):
  [msg.joint_position, msg.joint_name] = getTurnBack()
elif (posture_name == "zeroback"):
  [msg.joint_position, msg.joint_name] = getZeroBack()
elif (posture_name == "retract"):
  [msg.joint_position, msg.joint_name] = getRetract()
elif (posture_name == "hosemate"):
  [msg.joint_position, msg.joint_name] = getHoseMate()
elif (posture_name == "poolqueue"):
  [msg.joint_position, msg.joint_name] = getPoolQueue()
else:
  print "posture goal not found"
  sys.exit(-1)

if (posture_side == "l"):
  [msg.joint_name,msg.joint_position] = flipArms(msg.joint_name,msg.joint_position)

msg.num_joints = len(msg.joint_name)
lc.publish("POSTURE_GOAL", msg.encode())
print "\nPOSTURE_GOAL message sent"

