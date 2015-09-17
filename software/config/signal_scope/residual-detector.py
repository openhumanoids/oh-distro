# residual state plot

joints = ['base_x', 'base_y', 'base_z']
names = msg.joint_name

addPlot(timeWindow=30, yLimits=[-2.5, 2.5])
addSignals('RESIDUAL_OBSERVER_STATE', msg.utime, msg.joint_position, joints, keyLookup=names)


# # effort plot
# addPlot(timeWindow=30, yLimits=[-100, 100])
# for joint in joints:
#     addSignal('EST_ROBOT_STATE', msg.utime, msg.joint_effort[jn[joint]])
#     addSignal('ATLAS_COMMAND', msg.utime, msg.effort[jns[joint]])


# # velocity plot
# addPlot(timeWindow=30, yLimits=[-2, 2])
# for joint in joints:
#     addSignal('EST_ROBOT_STATE', msg.utime, msg.joint_velocity[jn[joint]])
#     addSignal('ATLAS_COMMAND', msg.utime, msg.velocity[jns[joint]])
