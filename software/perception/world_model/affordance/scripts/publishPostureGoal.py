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



def timestamp_now (): return int (time.time () * 1000000)

position= [0.438427, 1.68672585091, 1.7403498888, 0.102333297157, -0.0189416964902, -0.919958636739, 0.0, 0.00108842551708, -0.904232859612, 0.061247587204, 1.85839983821, 0.309883102747, -1.73209412982, 1.80132097006, -0.00540006160736, 0.0685999925613, -0.994326406288, 1.70440282393]
name = ["back_bky", "l_arm_elx", "l_arm_ely", "l_arm_mwx", "l_arm_shx", "l_arm_usy", "l_arm_uwy", "l_leg_akx", "l_leg_aky", "l_leg_hpx", "l_leg_kny", "neck_ay", "r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]


lc = lcm.LCM()
print "started"

msg = joint_angles_t()
msg.utime = timestamp_now ()
msg.joint_position = position
msg.joint_name = name
msg.num_joints = len(name)
lc.publish("POSTURE_GOAL", msg.encode())
