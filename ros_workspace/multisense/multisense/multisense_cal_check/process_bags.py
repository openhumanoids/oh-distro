#!/usr/bin/env python

'''
Simple Script to Processes Bag files from RawSnapshot in Release 2.0
and writes the data to file for calibration

Please direct any question to multisense@carnegierobotics.com or
    http://support.carnegierobotics.com
'''

import sys
import csv
import time
import os
import numpy as np
try:
    import rospy
    import rosbag
except:
    raise Exception("Error importing ROS. Source the ROS environment" \
                   +" in the workspace where the multisense stack is located")

class _BagProcessor():

    def __init__(self, bag_file):
        self.bag_file = bag_file

    #Wrapper method ot processes the bag file
    #Returns bag file name
    def process(self, directory='.'):
        self.process_laser(directory)
        self.process_image(directory)
        self.process_camera_yaml(directory)
        self.process_laser_yaml(directory)
        return self.rename_bag(directory)

    #Method to extract laser data from bag file and save into .csv
    def process_laser(self, directory='.'):
        bag = rosbag.Bag(self.bag_file)
        with open(directory + '/' + 'lidarData.csv', 'wb') as laser_file:
            laser_writer = csv.writer(laser_file, delimiter=',')
            for topic, msg, t in bag.read_messages(
                                 topics=['/laser/calibration/raw_lidar_data']):
                #Unbundle message
                time_start = float(msg.time_start.secs +
                                        msg.time_start.nsecs * 1e-9)
                time_end = float(msg.time_end.secs + msg.time_end.nsecs * 1e-9)
                angle_start = msg.angle_start
                angle_end = msg.angle_end
                dist = msg.distance
                reflect = msg.intensity

                #Expected Format: Unix time, time_start, time_end, angle_start,
                #angle_end, , range, , intensity, ,len(range)
                row = [time.time(), time_start, time_end,
                       angle_start, angle_end, " "] + list(dist) + [" "] \
                       + list(reflect) + [" "] + [len(list(dist))]

                laser_writer.writerow(row)

        laser_file.close()


    #Method to extract an image from a bag file
    def process_image(self, directory='.'):
        bag = rosbag.Bag(self.bag_file)
        for topic, msg, t in bag.read_messages(
                             topics=['/multisense_sl/calibration/raw_cam_data']):
            width = msg.width
            height = msg.height

            #Write to .pgm file stereo_left_0000.pgm
            if len(list(msg.gray_scale_image)) != 0:
                self.write_pgm(np.array(list(msg.gray_scale_image)),
                                directory + '/' + "stereo_left_0000.pgm",
                                width, height, 8)
            if len(list(msg.disparity_image)) != 0:
                self.write_pgm(np.array(list(msg.disparity_image),
                                dtype=np.uint16),
                                directory + '/' + "disparity_0000.pgm",
                                width, height, 16)


    #Method to write an image to a .pgm file
    def write_pgm(self, data, name, width, height, bits):
        image = open(name, 'wb')

        #Create .pgm file header
        pgm_header = 'P5' + '\n' + str(width) + ' ' \
                     + str(height) + '\n' + str(2**bits - 1) + '\n'

        #Data needs to be big endian not little endian for 16bit images
        if bits == 16:
            data = data.byteswap()

        image.write(pgm_header)
        data.tofile(image)
        image.close()

    #Extract image intrinsics from RawCamConfig.msg
    def process_camera_yaml(self, directory='.'):
        bag = rosbag.Bag(self.bag_file)
        for topic, msg, t in bag.read_messages(
                             topics=['/multisense_sl/calibration/'\
                                      +'raw_cam_config']):
            fx = msg.fx
            fy = msg.fy
            cx = msg.cx
            cy = msg.cy
            tx = msg.tx

            #Follow expected format of YAML file
            p1 =  "[ %.17e, 0., %d, 0., 0., \n" % (fx, cx) \
                 + "       %.17e, %d, 0., 0., 0., 1., 0.]" % (fy, cy) \

            p2 =  "[ %.17e, 0., %d, %.17e, 0., \n" % (fx, cx, tx*fx) \
                 + "       %.17e, %d, 0., 0., 0., 1., 0.]" % (fy, cy) \

            self.write_camera_yaml(directory + "/extrinsics_0p5mp.yml", p1, p2)


    #Writes P1 and P2 in openCV format to be used internally.
    #P1 and P2 represent the camera intrinsics
    def write_camera_yaml(self, name, p1, p2):
        yaml = open(name, 'wb')

        yaml_header = "%YAML:1.0\n"
        yaml.write(yaml_header)

        p1_header = "P1: !!opencv-matrix\n" \
                     + "   rows: 3\n" \
                     + "   cols: 4\n" \
                     + "   dt: d\n"


        p2_header = "P2: !!opencv-matrix\n" \
                     + "   rows: 3\n" \
                     + "   cols: 4\n" \
                     + "   dt: d\n"

        yaml.write(p1_header)
        yaml.write("   data: " + p1 + '\n\n')
        yaml.write(p2_header)
        yaml.write("   data: " + p2 + '\n')
        yaml.close()

    #Extract Laser To Spindle and Camera To Spindle Extrinsics from
    #RawLidarCal.msg
    def process_laser_yaml(self, directory="."):
        bag = rosbag.Bag(self.bag_file)
        i = 0
        for topic, msg, t in bag.read_messages(
                             topics=['/laser/calibration/raw_lidar_cal']):

            #Only use one message
            if i > 0:
                break
            #Laser to spindle
            lts = list(msg.laserToSpindle)
            #Camera to spindle
            cts = list(msg.cameraToSpindleFixed)

            #Mimics existing OpenCV format
            laser_t_spind = "[ %.17e, %.17e,\n" % (lts[0], lts[1]) \
                            +"      %.17e, %.17e,\n" % (lts[2], lts[3]) \
                            +"      %.17e, %.17e,\n" % (lts[4], lts[5]) \
                            +"      %.17e, %.17e,\n" % (lts[6], lts[7]) \
                            +"      %.17e, %.17e,\n" % (lts[8], lts[9]) \
                            +"      %.17e, 0., 0., 0., 0., 1. ]" % (lts[10])


            camera_t_spind = "[ %.17e, %.17e,\n" % (cts[0], cts[1]) \
                             +"      %.17e, %.17e,\n" % (cts[2], cts[3]) \
                             +"      %.17e, %.17e,\n" % (cts[4], cts[5]) \
                             +"      %.17e, %.17e,\n" % (cts[6], cts[7]) \
                             +"      %.17e, %.17e,\n" % (cts[8], cts[9]) \
                             +"      %.17e, %.17e, 0., 0., 0., 1. ]" % (cts[10], cts[11])

            self.write_laser_yaml(directory + "/laser_cal.yml",
                                  laser_t_spind, camera_t_spind)
            i = i+1

    #Write laser calibration in expected OpenCV format
    def write_laser_yaml(self, name, laser_t_spind, cam_t_spind):
        yaml = open(name, 'wb')
        yaml_header = "%YAML:1.0\n"
        yaml.write(yaml_header)

        laser_t_spind_h = "laser_T_spindle: !!opencv-matrix\n" \
                          +"   rows: 4\n" \
                          +"   cols: 4\n" \
                          +"   dt: d\n"

        cam_t_spind_h = "camera_T_spindle_fixed: !!opencv-matrix\n" \
                        +"   rows: 4\n" \
                        +"   cols: 4\n" \
                        +"   dt: d\n"

        yaml.write(laser_t_spind_h)
        yaml.write("   data: " + laser_t_spind + '\n')
        yaml.write(cam_t_spind_h)
        yaml.write("   data: " + cam_t_spind + '\n')
        yaml.close()

    #Writes out sensor information appends SN to bagfile name
    #Returns new bag file name
    def rename_bag(self, directory="."):
        bag = rosbag.Bag(self.bag_file)
        for topic, msg, t in bag.read_messages(
                             topics=['/multisense_sl/calibration/device_info']):

            sn = int(msg.serialNumber)

            info = open(directory + "/dev_info.txt", 'wb')

            info.write(str(msg))
            info.close


            bag = os.path.basename(self.bag_file)
            path = os.path.dirname(self.bag_file)

            fname = path + "/" + os.path.splitext(bag)[0]\
                               + "_SL_SN%04d_calCheck.bag" % sn


            os.rename(self.bag_file, fname)
            self.bag_file = fname
            return fname

