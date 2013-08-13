#!/bin/bash
rosrun image_view image_view image:=right_hand/left/image_raw  _autosize:=true &
rosrun image_view image_view image:=right_hand/right/image_raw _autosize:=true &
rosrun image_view image_view image:=left_hand/left/image_raw   _autosize:=true &
rosrun image_view image_view image:=left_hand/right/image_raw  _autosize:=true &
