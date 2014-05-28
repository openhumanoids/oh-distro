#!/usr/bin/python
"""
Script to read desired position / velocity commands and write them to the actual values.
Useful for testing open-loop Inverse Kinematics controllers.
"""
import roslib
import rospy
from time import sleep
from lxml import etree

roslib.load_manifest('nasa_ros_shm')
from nasa_ros_shm import Resource

class RawSync:
    def __init__(self, node, dest, src):
        self.dest = Resource('{}/{}'.format(node, dest))
        self.src = Resource('{}/{}'.format(node, src))
        
    def update(self):
        self.dest(self.src())

class DriverSync:
    def __init__(self, driver):
        self.driver = driver
    def sync(self):
        self.driver.setPosition(self.driver.getPositionCommand())
        self.driver.setVelocity(self.driver.getVelocityCommand())

syncs = []
xml_string = rospy.get_param('/robot_description_sim')
xml = etree.fromstring(xml_string)

for elem in xml.findall('actuator'):
    (actuator_node, actuator_type) = map(elem.attrib.get, ('name', 'type'))
    (pkg_name, cls_name) = actuator_type.split('/')
    print "{}: {}".format(actuator_node, actuator_type)
    
    if False:
        # TurbodriverSimInterface.py does not exist
        roslib.load_manifest(pkg_name)
        module = __import__(cls_name)
        Driver = getattr(module, cls_name)
        driver = ActuatorDriver(actuator_node)
        syncs.append(Driver(driver))
    else:
        if 'linear' in cls_name:
            syncs.append(RawSync(actuator_node, 'Linear_Pos_m', 'Position_Des_m'))
            syncs.append(RawSync(actuator_node, 'Linear_Vel_mps', 'Velocity_Des_mps'))
        elif cls_name.startswith('Wrist'):
            # Can't sync the wrists
            pass
        else:
            syncs.append(RawSync(actuator_node, 'JointAPS_Angle_Rad', 'Position_Des_Rad'))
            syncs.append(RawSync(actuator_node, 'JointAPS_Vel_Radps', 'Velocity_Des_Radps'))

print
print "Syncing actual with desired"
while True:
    for sync in syncs:
        sync.update()
    sleep(0.005)
