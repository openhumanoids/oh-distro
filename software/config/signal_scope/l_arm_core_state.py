execfile(os.path.join(os.path.dirname(__file__), 'rotations.py'))

# joints to plot
#joints = ['l_arm_shz','l_arm_shx','l_arm_ely','l_arm_elx','l_arm_uwy','l_arm_mwx','l_arm_lwy']
joints = ['leftShoulderPitch','leftShoulderRoll','leftShoulderYaw','leftElbowPitch','leftForearmYaw','leftWristRoll','leftWristPitch']

# string arrays for EST_ROBOT_STATE and ATLAS_COMMAND
jn = msg.joint_name
jns = msg.joint_names

# position plot
addPlot(timeWindow=15, yLimits=[-2.75, 2.75])
for joint in joints:
    addSignal('CORE_ROBOT_STATE', msg.utime, msg.joint_position[jn[joint]])

# velocity plot
addPlot(timeWindow=15, yLimits=[-2.75, 2.75])
for joint in joints:
    addSignal('CORE_ROBOT_STATE', msg.utime, msg.joint_velocity[jn[joint]])

# effort plot
addPlot(timeWindow=15, yLimits=[-2.75, 2.75])
for joint in joints:
    addSignal('CORE_ROBOT_STATE', msg.utime, msg.joint_effort[jn[joint]])
