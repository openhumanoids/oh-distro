#!/usr/bin/env python
import rospy
import math
import numpy as np
from ihmc_msgs.msg import HeadOrientationPacketMessage

def euler_to_quat(rpy):
  roll =  rpy[0]
  pitch = rpy[1]
  yaw =   rpy[2]

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


def talker():
    if (1==0):
        pub = rospy.Publisher('/ihmc_ros/valkyrie/control/head_orientation', HeadOrientationPacketMessage, queue_size=10)
    else:
        pub = rospy.Publisher('/ihmc_ros/valkyrie/control/head_orientation', HeadOrientationPacketMessage, queue_size=10)

    rospy.init_node('test_script', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():

        rpy = [0, -10.0/180.0*math.pi, 55.0/180.0*math.pi]
        quat = euler_to_quat(rpy)

        msg = HeadOrientationPacketMessage()
        msg.orientation.w = quat[0]
        msg.orientation.x = quat[1]
        msg.orientation.y = quat[2]
        msg.orientation.z = quat[3]
        msg.trajectory_time = 0.1
        print "Sending neck pose", rospy.get_time()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
