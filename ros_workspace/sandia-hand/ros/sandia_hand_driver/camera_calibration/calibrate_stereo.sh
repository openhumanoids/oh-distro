#!/bin/bash
rosrun camera_calibration cameracalibrator.py --size 5x4 --square 0.98 right:=/right/image_raw left:=/left/image_raw right_camera:=/right left_camera:=left

# then, use http://www.ros.org/wiki/camera_calibration_parsers to generate yaml
