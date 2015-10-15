# residual state plot

joints = ['base_x', 'base_y', 'base_z', 'r_leg_kny', 'l_leg_kny', 'l_leg_hpy', 'back_bkx', 'back_bky', 'back_bkz','l_arm_shx','l_arm_shz']

# shortened set of joint names to keep it readable for now.
joints = ['base_x','base_y','base_z','base_roll', 'base_pitch', 'base_yaw', 'r_leg_aky', 'r_leg_kny', 'r_leg_hpy', 'r_leg_hpx', 'l_leg_kny', 'l_arm_shz']

# joints = ['base_x','base_y','base_z','base_roll', 'base_pitch', 'base_yaw', 'r_leg_aky', 'r_leg_kny', 'r_leg_hpy', 'r_leg_hpx', 'l_leg_kny', 'l_arm_shz']
names = msg.joint_name
vel_names = msg.velocity_names

joints_w_dot = [];
for jointName in joints:
    joints_w_dot.append(jointName + "dot")



addPlot(timeWindow=30, yLimits=[-50, 50])
# addSignals('RESIDUAL_OBSERVER_STATE', msg.utime, msg.residual, joints, keyLookup=names)
# addSignals('RESIDUAL_OBSERVER_STATE_W_FOOT_FORCE', msg.utime, msg.residual, joints, keyLookup=names)
addSignals('RESIDUAL_OBSERVER_STATE_W_FOOT_FT', msg.utime, msg.residual, joints, keyLookup=names)
addSignals('CONTACT_FILTER_POINT_ESTIMATE', msg.utime, msg.implied_residual, joints_w_dot, keyLookup=vel_names)

# gravity vs. torque, should be equal, something is funky if they are not
# addPlot(timeWindow=30, yLimits=[-50, 50])
# # addSignals('RESIDUAL_OBSERVER_STATE', msg.utime, msg.gravity, joints, keyLookup=names)
# # addSignals('RESIDUAL_OBSERVER_STATE_W_FOOT_FORCE', msg.utime, msg.gravity, joints, keyLookup=names)
# # addSignals('RESIDUAL_OBSERVER_STATE_W_FOOT_FORCE', msg.utime, msg.internal_torque, joints, keyLookup=names)
# # addSignals('RESIDUAL_OBSERVER_STATE_W_FOOT_FORCE', msg.utime, msg.foot_contact_torque, joints, keyLookup=names)

# addSignals('RESIDUAL_OBSERVER_STATE_W_FOOT_FT', msg.utime, msg.gravity, joints, keyLookup=names)
# addSignals('RESIDUAL_OBSERVER_STATE_W_FOOT_FT', msg.utime, msg.internal_torque, joints, keyLookup=names)
# addSignals('RESIDUAL_OBSERVER_STATE_W_FOOT_FT', msg.utime, msg.foot_contact_torque, joints, keyLookup=names)

# addPlot(timeWindow=30, yLimits=[-100, 100])
# for joint in joints:
#     addSignal('EST_ROBOT_STATE', msg.utime, msg.joint_effort[jn[joint]])
#     addSignal('ATLAS_COMMAND', msg.utime, msg.effort[jns[joint]])


# # velocity plot
# addPlot(timeWindow=30, yLimits=[-2, 2])
# for joint in joints:
#     addSignal('EST_ROBOT_STATE', msg.utime, msg.joint_velocity[jn[joint]])
#     addSignal('ATLAS_COMMAND', msg.utime, msg.velocity[jns[joint]])
