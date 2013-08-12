#!/bin/bash
rosrun image_view image_view image:=left/image_raw _autosize:=true  &
rosrun image_view image_view image:=right/image_raw _autosize:=true &
