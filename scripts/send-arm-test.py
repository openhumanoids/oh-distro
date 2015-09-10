#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def talker():
    if (1==0):
        pub = rospy.Publisher('/ihmc_ros/atlas/control/arm_joint_trajectory2', JointTrajectory, queue_size=10)
        jn = ["l_arm_shz","l_arm_shx","l_arm_ely","l_arm_elx","l_arm_wry","l_arm_wrx","l_arm_wry2"]
        jn_r = ["r_arm_shz","r_arm_shx","r_arm_ely","r_arm_elx","r_arm_wry","r_arm_wrx","r_arm_wry2"]
    else:
        pub = rospy.Publisher('/ihmc_ros/valkyrie/control/arm_joint_trajectory2', JointTrajectory, queue_size=10)
        jn = ["LeftShoulderPitch","LeftShoulderRoll","LeftShoulderYaw","LeftElbowPitch","LeftForearmYaw","LeftWristRoll","LeftWristPitch"]
        jn_r = ["RightShoulderPitch","RightShoulderRoll","RightShoulderYaw","RightElbowPitch","RightForearmYaw","RightWristRoll","RightWristPitch"]


    #this doesnt work:
    #jn = ["l_arm_shz","l_arm_shx","l_arm_ely","l_arm_elx","l_arm_wry","l_arm_wrx","l_arm_wry2", "r_arm_shz","l_arm_shx","r_arm_ely","r_arm_elx","r_arm_wry","r_arm_wrx","r_arm_wry2"]
    #value = 0


    rospy.init_node('send_arm_test', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():

        msg = JointTrajectory()
        value = 0.1
        value_r = 0.1

        msg.joint_names = jn 
        pt = JointTrajectoryPoint()
        pt.positions = [value]*len(jn)
        pt.velocities = [0]*len(jn)
        pt.accelerations = [0]*len(jn)
        pt.effort = [0]*len(jn)
        pt.time_from_start = rospy.Duration.from_sec(3)

        msg.points.append(pt)
        print msg.joint_names, rospy.get_time()
        pub.publish(msg)

        # TODO: add a sleep here between left and right

        msg.joint_names = jn_r
        msg.points[0].positions= [value_r]*len(jn)
        print msg.joint_names, rospy.get_time()
        pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
