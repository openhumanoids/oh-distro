#!/bin/bash
#Main actuators have: 'Park', 'Reset', 'TorqueMode', 'ImpedanceMode', 'PositionMode'
main_actuators="/trunk/laj1,/left_arm/j2,/left_arm/j3,/left_arm/j4,/left_arm/j5"
#Fingers have "Park, Servo"
finger_actuators="/left_arm/forearm_driver1/thumb_proximal,/left_arm/forearm_driver1/thumb_roll,/left_arm/forearm_driver1/wrist_left,/left_arm/forearm_driver1/wrist_right,/left_arm/forearm_driver2/index,/left_arm/forearm_driver2/middle,/left_arm/forearm_driver2/pinky,/left_arm/forearm_driver2/thumb_distal"


cmd=$1
srv_call="rosservice call /ServoController/sendCommands \"modeCommands: ["

IFS=',' read -ra actuators <<< "$main_actuators"
IFS=',' read -ra fingers <<< "$finger_actuators"

if [ "${cmd}" == "Park" ]; then
	for act in "${actuators[@]}"; do
		dict="{resource: '$act', command: '$cmd'}, "
		srv_call=$srv_call$dict
	done
	for finger in "${fingers[@]}"; do
		dict="{resource: '$finger', command: '$cmd'}, "
		srv_call=$srv_call$dict
	done
fi

if [ "${cmd}" == "Servo" ]; then
	for finger in "${fingers[@]}"; do
		dict="{resource: '$finger', command: '$cmd'}, "
		srv_call=$srv_call$dict
	done
fi

if [ "${cmd}" == "Reset" -o "${cmd}" == "ImpedanceMode"  -o "${cmd}" == "PositionMode"  -o "${cmd}" == "TorqueMode" ]; then
	for act in "${actuators[@]}"; do
		dict="{resource: '$act', command: '$cmd'}, "
		srv_call=$srv_call$dict
	done
fi


#remove extra ", "
srv_call="${srv_call%??}"
#finish command
srv_end="]\""
srv_call=$srv_call$srv_end
echo ${srv_call}

${srv_call}



