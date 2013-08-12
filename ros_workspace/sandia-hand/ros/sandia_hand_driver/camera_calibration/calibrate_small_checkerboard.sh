#!/bin/bash
# sample usage: calibrate_small_checkerboard.sh image:=right/image_raw
rosrun camera_calibration cameracalibrator.py --size 5x4 --square 0.98425 $@
