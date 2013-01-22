#!/usr/bin/python
import roslib; roslib.load_manifest('trajectory_control_tutorial')
import rospy
import wx
import xml.dom.minidom

from control_msgs.msg import *
from trajectory_msgs.msg import *
from std_msgs.msg import *

from math import pi
from threading import Thread

RANGE = 10000
global mode

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class JointStatePublisher():
    def __init__(self):
        description = get_param('robot_description')
        #print description
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        self.free_joints = {}
        self.joint_list = [] # for maintaining the original order of the joints
        self.dependent_joints = get_param("dependent_joints", {})
        
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                    continue
                name = child.getAttribute('name')
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))

                if name in self.dependent_joints:
                    continue
                if minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
                self.free_joints[name] = joint
                self.joint_list.append(name)

        use_gui = get_param("use_gui", True)

        if use_gui:
            app = wx.App()
            self.gui = JointStatePublisherGui("Joint State Publisher", self)
            self.gui.Show()
            Thread(target=app.MainLoop).start()
        else:
            self.gui = None

        source_list = get_param("source_list", [])
        self.sources = []
        for source in source_list:
            self.sources.append(rospy.Subscriber(source, FollowJointTrajectoryActionGoal, self.source_cb))

        self.pub = rospy.Publisher('/atlas_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal)

        self.command_pubs=[]
        for (name,joint) in self.free_joints.items():
            msg_name = name+'_position_controller/command'
            pub = rospy.Publisher(msg_name, Float64)
            self.command_pubs.append(pub)

    def source_cb(self, msg):
        for i in range(len(msg.name)):
            name = msg.name[i]
            position = msg.position[i]
            if name in self.free_joints:
                joint = self.free_joints[name]
                joint['value'] = position
        if self.gui is not None:
            self.gui.update_sliders()
        
    def publish_FollowJointTrajectoryActionGoal(self):
            msg = FollowJointTrajectoryActionGoal()
            msg.header.stamp = rospy.Time.now()
            msg.goal_id.stamp = msg.header.stamp
            msg.goal_id.id = "/asdfas-1-" + str(msg.header.stamp.secs ) + "." + str(msg.header.stamp.nsecs/ 1000000 )
            
            point = JointTrajectoryPoint()
            for (name,joint) in self.free_joints.items():
                if (name == 'head_imu_joint'):
                  continue
                if (name == 'hokuyo_joint'):
                  continue
                if (name == 'imu_joint'):
                  continue
                msg.goal.trajectory.joint_names.append(str(name))
                point.positions.append(joint['value'])
                point.velocities.append(0)
                #point.accelerations.append(0)

            point.time_from_start.secs =1
            msg.goal.trajectory.points.append(point)
            self.pub.publish(msg)

    def publish_neck(self):
            counter=0
            for (name,joint) in self.free_joints.items():
                if (name == 'head_imu_joint'):
                  continue
                if (name == 'hokuyo_joint'):
                  continue
                if (name == 'imu_joint'):
                  continue
                msg = Float64()
                msg.data = joint['value']
                self.command_pubs[counter].publish(msg)
                #self.command_pubs[0].publish(msg)
                counter=counter+1
                #msg.goal.trajectory.joint_names.append(str(name))
                #point.positions.append(joint['value'])
                #point.velocities.append(0)
                #point.accelerations.append(0)

            #point.time_from_start.secs =1
            #msg.goal.trajectory.points.append(point)
            #self.pub.publish(msg)
            #msg = Float64()
            #msg.data = -1
            #self.command_pubs[0].publish(msg)

    def loop(self):
        hz = get_param("rate", 10) # 10hz
        r = rospy.Rate(hz) 

        # Publish Joint States
        while not rospy.is_shutdown():
            if (mode=='traj'): 
              self.publish_FollowJointTrajectoryActionGoal()
            elif(mode=='joint'): 
              self.publish_neck()
            else:
              print "mode must be either traj or joint - not",mode

            # Add Free Joints
            #for (name,joint) in self.free_joints.items():
            #    msg.goal.trajectory.joint_names.append(str(name))
            #    msg.goal.trajectory.points.positions.append(joint['value'])

            # Add Dependent Joints
            #for (name,param) in self.dependent_joints.items():
            #    parent = param['parent']
            #    baseval = self.free_joints[parent]['value']
            #    value = baseval * param.get('factor', 1)

            #    msg.name.append(str(name))
            #    msg.position.append(value)


            r.sleep()

class JointStatePublisherGui(wx.Frame):
    def __init__(self, title, jsp):
        wx.Frame.__init__(self, None, -1, title, (-1, -1));
        self.jsp = jsp
        self.joint_map = {}
        panel = wx.Panel(self, wx.ID_ANY);
        box = wx.BoxSizer(wx.VERTICAL)
        font = wx.Font(9, wx.SWISS, wx.NORMAL, wx.BOLD)
        
        ### Sliders ###
        for name in self.jsp.joint_list:
            joint = self.jsp.free_joints[name]

            if joint['min'] == joint['max']:
                continue

            row = wx.GridSizer(1,2)
            label = wx.StaticText(panel, -1, name)
            label.SetFont(font)
            row.Add(label, 1, wx.ALIGN_CENTER_VERTICAL)

            display = wx.TextCtrl (panel, value=str(0), 
                        style=wx.TE_READONLY | wx.ALIGN_RIGHT)

            row.Add(display, flag= wx.ALIGN_RIGHT| wx.ALIGN_CENTER_VERTICAL)
            box.Add(row, 1, wx.EXPAND)
            slider = wx.Slider(panel, -1, RANGE/2, 0, RANGE, 
                        style= wx.SL_AUTOTICKS | wx.SL_HORIZONTAL)
            slider.SetFont(font)
            box.Add(slider, 1, wx.EXPAND)

            self.joint_map[name] = {'slidervalue':0, 'display':display, 
                                    'slider':slider, 'joint':joint}

        ### Buttons ###
        self.ctrbutton = wx.Button(panel, 1, 'Center')
        self.Bind(wx.EVT_SLIDER, self.sliderUpdate)
        
        wx.EVT_BUTTON(self, 1, self.center_event)

        box.Add(self.ctrbutton, 0, wx.EXPAND)
        
        panel.SetSizer(box)
        self.center()
        box.Fit(self)
        self.update_values()


    def update_values(self):
        for (name,joint_info) in self.joint_map.items():
            purevalue = joint_info['slidervalue']
            joint = joint_info['joint']
            value = self.sliderToValue(purevalue, joint)
            joint['value'] = value
        self.update_sliders()

    def update_sliders(self):
        for (name,joint_info) in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slidervalue'] = self.valueToSlider(joint['value'], joint)
            joint_info['slider'].SetValue(joint_info['slidervalue'])
            joint_info['display'].SetValue("%.2f"%joint['value'])

    def center_event(self, event):
        self.center()

    def center(self):
        rospy.loginfo("Centering")
        for (name,joint_info) in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slidervalue'] = self.valueToSlider(joint['zero'], joint)
        self.update_values()

    def sliderUpdate(self, event):
        for (name,joint_info) in self.joint_map.items():
            joint_info['slidervalue'] = joint_info['slider'].GetValue()
        self.update_values()

    def valueToSlider(self, value, joint):
        return (value - joint['min']) * float(RANGE) / (joint['max'] - joint['min'])
        
    def sliderToValue(self, slider, joint):
        pctvalue = slider / float(RANGE)
        return joint['min'] + (joint['max']-joint['min']) * pctvalue


print 'traj_gui.py [joint|traj]'
print 'publisher of joint angles or a single trajectory ROS msg'
print 'Argument List:', str(sys.argv)
print sys.argv[0]
if (sys.argv>1):
  mode = sys.argv[1]
else:
  mode = "joint"




if __name__ == '__main__':
    try:
        rospy.init_node('traj_yaml')
        jsp = JointStatePublisher()
        jsp.loop()
        
    except rospy.ROSInterruptException: pass

