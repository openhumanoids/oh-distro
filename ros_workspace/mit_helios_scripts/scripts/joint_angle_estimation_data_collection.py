#!/usr/bin/env python

'''
Created on Nov 14, 2013

@author: Twan
'''
import roslib; roslib.load_manifest('mit_helios_scripts')
from IRobotHandController import IRobotHandController
import csv
import inspect, os

def getTimeName():
    return "time"

def getMotorHallEncoderName(i):
    return "motorHallEncoder[%s]" % i

def getProximalJointSensorName(i):
    return "proximalJointAngle[%s]" % i

class LoggingListener(object):
    def __init__(self):
        indices = IRobotHandController.get_motor_indices()
        
        self.fieldnames = [];
        self.fieldnames.append(getTimeName())
        for i in indices:
            self.fieldnames.append(getMotorHallEncoderName(i))
        for i in indices:
            self.fieldnames.append(getProximalJointSensorName(i))
        
        self.data = []

    
    def notify(self, data, time):
        
        # TODO: take offsets into account
        
        data_point = {}
        data_point[getTimeName()] = time
        for motor_index in IRobotHandController.get_motor_indices():
            data_point[getProximalJointSensorName(motor_index)] = data.proximalJointAngle[motor_index]
            data_point[getMotorHallEncoderName(motor_index)] = data.motorHallEncoder[motor_index]
        self.data.append(data_point)
        
    def write_csv(self):
        currentfile = inspect.getfile(inspect.currentframe())
        currentdir = os.path.dirname(os.path.abspath(currentfile))

        with open(currentdir + '/joint_angle_estimation_data.csv', 'wb') as csvfile:
            writer = csv.DictWriter(csvfile, self.fieldnames,  dialect='excel')
            writer.writeheader()
            for i in range(len(self.data)):
                writer.writerow(self.data[i])

if __name__ == '__main__':
    side = 'r'
    controller = IRobotHandController(side)
    logging_listener = LoggingListener()
    controller.add_sensor_data_listener(logging_listener)
    controller.close_hand_current_control(300)
    controller.remove_sensor_data_listener(logging_listener)
    controller.open_hand_motor_excursion_control()
    logging_listener.write_csv()
    