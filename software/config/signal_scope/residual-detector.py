# residual state plot

joints = ['base_x', 'base_y', 'base_z', 'r_leg_kny', 'l_leg_kny', 'r_leg_hpy', 'l_leg_hpy', 'back_bkx', 'back_bky', 'back_bkz','l_arm_shx','l_arm_shz']
names = msg.joint_name

addPlot(timeWindow=30, yLimits=[-50, 50])
addSignals('RESIDUAL_OBSERVER_STATE', msg.utime, msg.joint_position, joints, keyLookup=names)
addSignals('RESIDUAL_OBSERVER_STATE_WITH_FORCES', msg.utime, msg.joint_position, joints, keyLookup=names)


# gravity vs. torque, should be equal, something is funky if they are not
addPlot(timeWindow=30, yLimits=[-50, 50])
addSignals('RESIDUAL_OBSERVER_STATE', msg.utime, msg.joint_velocity, joints, keyLookup=names)
addSignals('RESIDUAL_OBSERVER_STATE', msg.utime, msg.joint_effort, joints, keyLookup=names)
# addSignals('RESIDUAL_OBSERVER_STATE_WITH_FORCES', msg.utime, msg.joint_velocity, joints, keyLookup=names)
addSignals('RESIDUAL_OBSERVER_STATE_WITH_FORCES', msg.utime, msg.joint_effort, joints, keyLookup=names)

# addPlot(timeWindow=30, yLimits=[-100, 100])
# for joint in joints:
#     addSignal('EST_ROBOT_STATE', msg.utime, msg.joint_effort[jn[joint]])
#     addSignal('ATLAS_COMMAND', msg.utime, msg.effort[jns[joint]])


# # velocity plot
# addPlot(timeWindow=30, yLimits=[-2, 2])
# for joint in joints:
#     addSignal('EST_ROBOT_STATE', msg.utime, msg.joint_velocity[jn[joint]])
#     addSignal('ATLAS_COMMAND', msg.utime, msg.velocity[jns[joint]])
