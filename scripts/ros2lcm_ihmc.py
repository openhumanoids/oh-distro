#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ihmc_msgs.msg import ChestOrientationPacketMessage
import lcm
from bot_core.pose_t import pose_t

lc = lcm.LCM()

def callback(m):
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", m.orientation.x)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
    msg = pose_t()
    msg.utime = 0
    msg.pos = [0,0,0]
    msg.orientation = [m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z]
    lc.publish('POSE_BODY_ALT', msg.encode() )


def listener():

    

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/ihmc_ros/valkyrie/output/chest_orientation", ChestOrientationPacketMessage, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
