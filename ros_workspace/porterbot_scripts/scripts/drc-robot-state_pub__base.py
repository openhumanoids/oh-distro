#!/usr/bin/env python
import roslib; roslib.load_manifest('atlas_gazebo_msgs')
import rospy
import numpy
from std_msgs.msg import String
from atlas_gazebo_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

global j_p_r, j_p_l,j_p_b, j_v_r, j_v_l, j_v_b, pose, j_n, pub, openrave_header
j_p_r=(0,0,0,0,0,0,0)
j_p_l=(0,0,0,0,0,0,0)
j_v_r=(0,0,0,0,0,0,0)
j_v_l=(0,0,0,0,0,0,0)
j_p_b=(0,)
j_v_b=(0,)
pub = rospy.Publisher('true_robot_state', RobotState)
j_n='l_hokuyo_joint', 'l_j1', 'l_j2', 'l_j3', 'l_j4', 'l_j5', 'l_j6', 'l_j7', 'r_hokuyo_joint', 'r_j1', 'r_j2', 'r_j3', 'r_j4', 'r_j5', 'r_j6', 'r_j7', 'torso_joint' #hardcoded
pose = Pose()
pose.orientation.w=1

def right_callback(data):
    global j_p_r, j_p_l,j_p_b, j_v_r, j_v_l, j_v_b, pose, j_n, pub, openrave_header
    openrave_header= data.header
    j_p_r_temp=data.position
    j_p_r_list=list(j_p_r_temp)
    j_p_r_list[6]=j_p_r_list[6] -1.85 #because laser is mounted on the wrong direction. screw is in the way on the other side. Since 23.Jan.2013 the zero position has changed
    j_p_r=tuple(j_p_r_list)
    j_v_r=data.velocity
    null=(0,)
    j_p=null+j_p_l[0:7]+null+j_p_r[0:7]+j_p_b[0:1]
    j_v=null+j_v_l[0:7]+null+j_v_r[0:7]+j_v_b
    #print j_p_r, j_p 
    #rospy.init_node('robot_state_publisher')
    #r = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():
    #j_p=numpy.array([0,10,10,10,10,10,10,10,0,10,10,10,10,10,10,10,0], dtype=numpy.float64)
    #j_v=numpy.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], dtype=numpy.float64)
    pub.publish(header=openrave_header,body_pose = pose, joint_name=j_n, joint_position=j_p, joint_velocity=j_v)
    #print "loop"
    #rospy.sleep(1)


def left_callback(data):
    global j_p_r, j_p_l,j_p_b, j_v_r, j_v_l, j_v_b, pose, j_n, pub, openrave_header
    openrave_header= data.header
    j_p_l_temp=data.position
    j_p_l_list=list(j_p_l_temp)
    j_p_l_list[0]=j_p_l_list[0]*-1  #because openrave robot description is wrong
    j_p_l=tuple(j_p_l_list)
    j_v_l=data.velocity
    null=(0,)
    j_p=null+j_p_l[0:7]+null+j_p_r[0:7]+j_p_b[0:1]
    j_v=null+j_v_l[0:7]+null+j_v_r[0:7]+j_v_b
    pub.publish(header=openrave_header,body_pose = pose, joint_name=j_n, joint_position=j_p, joint_velocity=j_v)

def base_callback(data):
    print "test"
    global j_p_r, j_p_l,j_p_b, j_v_r, j_v_l, j_v_b, pose, j_n, pub, openrave_header
    openrave_header= data.header
    j_p_b_temp=data.position
    j_p_b_list=list(j_p_b_temp)
    j_p_b_list[0]=j_p_b_list[0]*-1 
    j_p_b=tuple(j_p_b_list)
    print j_p_b
    print j_v_b
    j_v_b=data.velocity
    null=(0,)
    j_p=null+j_p_l[0:7]+null+j_p_r[0:7]+j_p_b[0:1]
    j_v=null+j_v_l[0:7]+null+j_v_r[0:7]+j_v_b
    pub.publish(header=openrave_header,body_pose = pose, joint_name=j_n, joint_position=j_p, joint_velocity=j_v)
    #rospy.sleep(2)

def translator():

    #listener
    rospy.init_node('rosOpenrave2ros')
    rospy.Subscriber("/right/mechanism_state", JointState, right_callback)
    rospy.Subscriber("/left/mechanism_state", JointState, left_callback)
    rospy.Subscriber("/base/mechanism_state", JointState, base_callback)
    rospy.spin()






if __name__ == '__main__':
    try:
        translator()
    except rospy.ROSInterruptException:
        pass


#import rospy
#from tf import TransformListener

#class myNode:
#    def __init__(self, *args):
#        self.tf = TransformListener()
 #       rospy.Subscriber(... etc
#
 #   def some_method(self):
#
 #       if self.tf.frameExists("/base_link") and self.tf.frameExists("/map"):
  #          t = self.tf.getLatestCommonTime("/base_link", "/map")
   #         position, quaternion = self.tf.lookupTransform("/base_link", "/map", t)
    #        print position, quaternion
