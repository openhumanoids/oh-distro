#!/usr/bin/env python

'''
Created on Nov 14, 2013

@author: Twan
'''
import roslib; roslib.load_manifest('mit_helios_scripts')
from IRobotHandController import IRobotHandController
from handle_msgs.msg import HandleSensorsCalibrated
import csv
import inspect, os
import rospy

def getTimeName():
    return "time"

def getMotorHallEncoderName(i):
    return "motorHallEncoder[%s]" % i

def getProximalJointSensorName(i):
    return "proximalJointAngle[%s]" % i

def getDistalJointSensorName(i):
    return "distalJointAngle[%s]" % i

class JointAngleLogger(object):
    def __init__(self, controller):
        indices = IRobotHandController.get_close_hand_motor_indices()
        self.controller = controller
        self.fieldnames = [];
        self.fieldnames.append(getTimeName())
        for i in indices:
            self.fieldnames.append(getMotorHallEncoderName(i))
        for i in indices:
            self.fieldnames.append(getProximalJointSensorName(i))
        for i in indices:
            self.fieldnames.append(getDistalJointSensorName(i))
        
        self.data = []
        rospy.Subscriber("sensors/calibrated", HandleSensorsCalibrated, self.calibrated_sensors_callback)

    def calibrated_sensors_callback(self, data):
        data_point = {}
        data_point[getTimeName()] = rospy.get_time()
        for i in IRobotHandController.get_close_hand_motor_indices():
            data_point[getProximalJointSensorName(i)] = data.proximalJointAngle[i]
            data_point[getDistalJointSensorName(i)] = data.distalJointAngle[i].distal[0]
#             print data.distalJointAngle[i]
            data_point[getMotorHallEncoderName(i)] = controller.get_tendon_excursions_with_offset()[i]
        self.data.append(data_point)
        
    def write_csv(self):
        currentfile = inspect.getfile(inspect.currentframe())
        currentdir = os.path.dirname(os.path.abspath(currentfile))
        datadir = currentdir + '/../data/'

        with open(datadir + 'joint_angle_estimation_data.csv', 'wb') as csvfile:
            writer = csv.DictWriter(csvfile, self.fieldnames,  dialect='excel')
            writer.writeheader()
            for i in range(len(self.data)):
                writer.writerow(self.data[i])

if __name__ == '__main__':
    side = 'r'
    controller = IRobotHandController(side)
    logger = JointAngleLogger(controller)
    controller.close_hand_current_control(300)
    controller.open_hand_motor_excursion_control()
    logger.write_csv()
    