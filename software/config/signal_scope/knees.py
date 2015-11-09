# joints to plot
joints = ['l_leg_kny','r_leg_kny']


# string arrays for EST_ROBOT_STATE and ATLAS_COMMAND
jn = msg.joint_name
jns = msg.joint_names


# position plot
addPlot(timeWindow=30, yLimits=[-2.5, 2.5])
for joint in joints:
    addSignal('EST_ROBOT_STATE', msg.utime, msg.joint_position[jn[joint]])
    addSignal('ATLAS_COMMAND', msg.utime, msg.position[jns[joint]])


# effort plot
addPlot(timeWindow=30, yLimits=[-100, 100])
for joint in joints:
    addSignal('EST_ROBOT_STATE', msg.utime, msg.joint_effort[jn[joint]])
    addSignal('ATLAS_COMMAND', msg.utime, msg.effort[jns[joint]])


# velocity plot
addPlot(timeWindow=30, yLimits=[-2, 2])
for joint in joints:
    addSignal('EST_ROBOT_STATE', msg.utime, msg.joint_velocity[jn[joint]])
    addSignal('ATLAS_COMMAND', msg.utime, msg.velocity[jns[joint]])
