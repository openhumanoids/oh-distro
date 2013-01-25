
ClosedGraspController is the controller that takes key presses as inputs and outputs the desired joint angles for the sandia hand
GraspCmd is the input frame
SandiaPositionRef is the output frame

End-to-end Instructions


1) Add the following line to your startup.m file:
setenv('DRC','/home/ceci91/drc/software/models/mit_gazebo_models/')

3) Add ~/drc/software/control/simple_grasp to your MATLAB path

2) cd ~/drc/software/models/mit_gazebo_models/mit_robot_hands/drake_urdfs/xacro
   rosrun xacro xacro.py sandia_hand_right_drake.urdf.xacro > ../model.sdf

3) cd ~/drc/software/models/mit_gazebo_models/mit_robot_hands/xacro
   rosrun xacro xacro.py sandia_hand_right.urdf.xacro > ../model.sdf

4) In MATLAB: runGraspCmdLCM.m

5) ~/drc/software/config
   bot-procman-sheriff -l sandia_position_control.pmd

Valid key presses (must be in the keyPresses applet for it to function):
1) Spacebar for closed grasp
2) Enter key for open hand


