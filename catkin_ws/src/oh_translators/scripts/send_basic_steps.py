#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from ihmc_msgs.msg import FootstepDataMessage
from ihmc_msgs.msg import FootstepDataListMessage

def getUnitQuaternion():
    q = Quaternion()
    q.w=1
    q.x=0
    q.y=0
    q.z=0
    return q


def insertWalkingForward(m):
    f = FootstepDataMessage()
    f.robot_side = 1
    f.location.x =0.15
    f.location.y =-0.13
    f.location.z =0.09
    f.orientation = getUnitQuaternion()
    f.trajectory_type = 1
    f.swing_height = 0.05
    m.footstep_data_list.append(f)

    f = FootstepDataMessage()
    f.robot_side = 0
    f.location.x =0.15
    f.location.y =0.13
    f.location.z =0.09
    f.orientation = getUnitQuaternion()
    f.trajectory_type = 1
    f.swing_height = 0.05
    m.footstep_data_list.append(f)

    f = FootstepDataMessage()
    f.robot_side = 1
    f.location.x =0.34
    f.location.y =-0.13
    f.location.z =0.09
    f.orientation = getUnitQuaternion()
    f.trajectory_type = 1
    f.swing_height = 0.05
    m.footstep_data_list.append(f)

    f = FootstepDataMessage()
    f.robot_side = 0
    f.location.x =0.34
    f.location.y =0.13
    f.location.z =0.09
    f.orientation = getUnitQuaternion()
    f.trajectory_type = 1
    f.swing_height = 0.05
    m.footstep_data_list.append(f)
    return m


def insertWalkingForwardAndUp(m):
    f = FootstepDataMessage()
    f.robot_side = 1
    f.location.x =0.15
    f.location.y =-0.13
    f.location.z =0.15
    f.orientation = getUnitQuaternion()
    f.trajectory_type = 1
    f.swing_height = 0.05
    m.footstep_data_list.append(f)

    f = FootstepDataMessage()
    f.robot_side = 0
    f.location.x =0.15
    f.location.y =0.13
    f.location.z =0.15
    f.orientation = getUnitQuaternion()
    f.trajectory_type = 1
    f.swing_height = 0.05
    m.footstep_data_list.append(f)

    f = FootstepDataMessage()
    f.robot_side = 1
    f.location.x =0.34
    f.location.y =-0.13
    f.location.z =0.15
    f.orientation = getUnitQuaternion()
    f.trajectory_type = 1
    f.swing_height = 0.05
    m.footstep_data_list.append(f)

    f = FootstepDataMessage()
    f.robot_side = 0
    f.location.x =0.34
    f.location.y =0.13
    f.location.z =0.15
    f.orientation = getUnitQuaternion()
    f.trajectory_type = 1
    f.swing_height = 0.05
    m.footstep_data_list.append(f)
    return m

def insertWalkingHome(m):
    f = FootstepDataMessage()
    f.robot_side = 1
    f.location.x =0.0
    f.location.y =-0.13
    f.location.z =0.09
    f.orientation = getUnitQuaternion()
    f.trajectory_type = 1
    f.swing_height = 0.05
    m.footstep_data_list.append(f)

    f = FootstepDataMessage()
    f.robot_side = 0
    f.location.x =0.0
    f.location.y =0.13
    f.location.z =0.09
    f.orientation = getUnitQuaternion()
    f.trajectory_type = 1
    f.swing_height = 0.05
    m.footstep_data_list.append(f)
    return m

def talker():
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    pub2 = rospy.Publisher('shlep', FootstepDataMessage, queue_size=10)
    pub3 = rospy.Publisher('/ihmc_ros/valkyrie/control/footstep_list', FootstepDataListMessage, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    

    hello_str = "hello world %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    #pub.publish(hello_str)
    rate.sleep()


    m = FootstepDataListMessage()
    #m = insertWalkingForward(m)
    m = insertWalkingHome(m)
    #m = insertWalkingForwardAndUp(m)

    m.swing_time = 1.5
    m.transfer_time = 2.0
    pub3.publish(m)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass