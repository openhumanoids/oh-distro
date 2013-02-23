#!/usr/bin/python
import roslib; roslib.load_manifest('tutorial_atlas_control')
import rospy, yaml, sys
from numpy import zeros, array, linspace
import wx
import xml.dom.minidom

from osrf_msgs.msg import JointCommands
from sensor_msgs.msg import JointState

from math import pi
from threading import Thread

RANGE = 10000


atlasJointNames = [
  'atlas::back_lbz', 'atlas::back_mby', 'atlas::back_ubx', 'atlas::neck_ay',
  'atlas::l_leg_uhz', 'atlas::l_leg_mhx', 'atlas::l_leg_lhy', 'atlas::l_leg_kny', 'atlas::l_leg_uay', 'atlas::l_leg_lax',
  'atlas::r_leg_uhz', 'atlas::r_leg_mhx', 'atlas::r_leg_lhy', 'atlas::r_leg_kny', 'atlas::r_leg_uay', 'atlas::r_leg_lax',
  'atlas::l_arm_usy', 'atlas::l_arm_shx', 'atlas::l_arm_ely', 'atlas::l_arm_elx', 'atlas::l_arm_uwy', 'atlas::l_arm_mwx',
  'atlas::r_arm_usy', 'atlas::r_arm_shx', 'atlas::r_arm_ely', 'atlas::r_arm_elx', 'atlas::r_arm_uwy', 'atlas::r_arm_mwx']
currentJointState = JointState()
def jointStatesCallback(msg):
  global currentJointState
  currentJointState = msg

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

                # Only create a widget for atlas joints - ignore the finger and multisense joints
                appended_name = "atlas::" + name
                for item in atlasJointNames:
                 if item.find(appended_name) != -1:
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

        self.pub = rospy.Publisher('/atlas/joint_commands', JointCommands)
        #self.pub = rospy.Publisher('/atlas_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal)

    def source_cb(self, msg):
        for i in range(len(msg.name)):
            name = msg.name[i]
            position = msg.position[i]
            if name in self.free_joints:
                joint = self.free_joints[name]
                joint['value'] = position
        if self.gui is not None:
            self.gui.update_sliders()
        
    def publish_JointCommand(self):
        # initialize JointCommands message
        command = JointCommands()
        command.name = list(atlasJointNames)
        n = len(command.name)
        command.position     = zeros(n)
        command.velocity     = zeros(n)
        command.effort       = zeros(n)
        command.kp_position  = zeros(n)
        command.ki_position  = zeros(n)
        command.kd_position  = zeros(n)
        command.kp_velocity  = zeros(n)
        command.i_effort_min = zeros(n)
        command.i_effort_max = zeros(n)

        # now get gains from parameter server
        for i in xrange(len(command.name)):
          name = command.name[i]
          command.position[i]  = 0.0
          command.kp_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/p')
          command.ki_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i')
          command.kd_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/d')
          command.i_effort_max[i] = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i_clamp')
          command.i_effort_min[i] = -command.i_effort_max[i]


        for (name,joint) in self.free_joints.items():
          counter=0
          appended_name = "atlas::" + name
          for item in atlasJointNames:
            if item.find(appended_name) != -1:
              command.position[counter]  = joint['value']
            counter= counter+1
        self.pub.publish(command)

    def loop(self):
        hz = get_param("rate", 10) # 10hz
        r = rospy.Rate(hz) 

        # Publish Joint States
        while not rospy.is_shutdown():
          self.publish_JointCommand()
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


print 'traj_gui.py'
print 'Publisher of joint command as a single ROS msg'
print 'Argument List:', str(sys.argv)

if __name__ == '__main__':

  rospy.init_node('traj_gui')
  try:
    jsp = JointStatePublisher()
    jsp.loop()
        
  except rospy.ROSInterruptException: pass

