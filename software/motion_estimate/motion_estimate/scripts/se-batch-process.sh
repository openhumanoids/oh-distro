#!/bin/bash
# se-batch-process.sh
# Author: Maurice Fallon
# Writes Batch Processes a set of State Estimation Logs

date_str=$(date +"%Y-%m-%d-%H-%M")
echo 'Output to directory '$date_str
#file="lcmlog-2014-01-21-16-40-robot-manip-mode"

#
path="/home/mfallon/logs/atlas/2014-04-21-vicon-walking/"
#path_out=$path"results/"$date_str
path_out='/home/mfallon/Desktop/results/'

unset files
files[0]='test' 
files=(
       'blocks1-lcmlog-2014-04-21-18-24-robot'
       'blocks2-lcmlog-2014-04-21-18-34-robot'
       'blocks3-lcmlog-2014-04-21-18-40-robot'
       'fwdbkwd-lcmlog-2014-04-21-15-29-robot'
       'longstp-lcmlog-2014-04-21-16-12-robot'
       'manip01-lcmlog-2014-04-21-15-43-robot'
       'manip02-lcmlog-2014-04-21-15-51-robot'
       'typical-lcmlog-2014-04-21-15-13-robot'
      )





process_log(){
  path=$1
  path_out=$2
  date_str=$3
  file=$4

  log_in=$path$file
  log_out=$path_out"/"$file"-result"
  log_out_legodo=$log_out"-se-leg-odometry"
  log_out_fusion=$log_out"-se-fusion"
  log_out_legodo_mat=$log_out_legodo".mat"
  log_out_fusion_mat=$log_out_fusion".mat"

  verbose=0
  if [ $verbose -eq 1 ]
    then
    echo $log_in
    echo $path_out
    echo $log_out
    echo $log_out_legodo
    echo $log_out_fusion
  fi

  #se-leg-odometry -U model_LH_RH.urdf -P drc_robot_02.cfg -L $log_in -pr 0 -r -l $log_out_legodo
  #bot-log2mat  $log_out_legodo  -c "POSE_BDI|POSE_VICON|POSE_BODY_ALT" -o $log_out_legodo_mat

  se-fusion       -U model_LH_RH.urdf -P drc_robot_02_mit.cfg -L $log_in -pr 0    -l $log_out_fusion
  bot-log2mat  $log_out_fusion  -c "POSE_BDI|POSE_VICON|POSE_BODY_ALT|POSE_BODY|POSE_MIT" -o $log_out_fusion_mat

  echo $log_out_legodo_mat
  echo $log_out_fusion_mat
}

tLen=${#files[@]}


mkdir $path_out -p

for (( i=0; i<${tLen}; i++ ));
do
  echo 'Processing: '$i' of '$tLen': '${files[i]}
  process_log $path $path_out $date_str ${files[i]}
done



#       'lcmlog-2014-01-21-16-19-robot-typical-stepping'
#       'lcmlog-2014-01-21-16-19-robot-long-stepping'
#       'lcmlog-2014-01-21-18-24-robot-wrong-normals'
#       'lcmlog-2014-01-21-16-40-robot-manip-mode'
#       'lcmlog-2014-01-21-17-48-robot-blocks-continously'
#       'lcmlog-2014-01-21-17-00-robot-dynamic1'
#       'lcmlog-2014-01-21-17-00-robot-dynamic2'
#       'lcmlog-2014-01-21-17-00-robot-dynamic3'
#       'lcmlog-2014-01-21-17-00-robot-dynamic4-turning'
#       'lcmlog-2014-01-21-17-00-robot-dynamic5-turning-in-place'
#       'lcmlog-2014-01-21-17-00-robot-dynamic6-extended-turning'
