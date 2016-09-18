# residual state plot

joints = ['base_x', 'base_y', 'base_z', 'r_leg_kny', 'l_leg_kny', 'l_leg_hpy', 'back_bkx', 'back_bky', 'back_bkz','l_arm_shx','l_arm_shz']

# shortened set of joint names to keep it readable for now.
base_joints = ['base_x','base_y','base_z','base_roll', 'base_pitch', 'base_yaw'];

l_leg_joints = ["leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll"]
joints = base_joints + l_leg_joints
# joints = ['base_x','base_y','base_z','base_roll', 'base_pitch', 'base_yaw']
# joints = ['base_z','base_roll', 'base_pitch', 'base_yaw']
# joints = ['base_x','base_z','base_roll', 'base_pitch', 'l_leg_kny']
# joints = ['base_z','base_roll', 'base_pitch']
# joints = ['base_z', 'base_pitch', 'l_arm_shz', 'l_arm_shx']


# joints = ['base_x','base_y','base_z','base_roll', 'base_pitch', 'base_yaw', 'back_bkz', 'back_bky', 'back_bkx','l_arm_shz', 'l_arm_shx']

# joints = ['base_x','base_y','base_z','base_roll', 'base_pitch', 'base_yaw', 'r_leg_aky', 'r_leg_kny', 'r_leg_hpy', 'r_leg_hpx', 'l_leg_kny', 'l_arm_shz']
names = msg.joint_name
vel_names = msg.velocity_names

joints_w_dot = [];
for jointName in joints:
    joints_w_dot.append(jointName + "dot")


channel_extensions = ['B_PELVIS','M_PELVIS', 'R_PELVIS', 'L_PELVIS']
# channel_extensions = ['M_PELVIS', 'R_PELVIS']


sparsePlot = False
if sparsePlot:
    channel_extensions = []



addPlot(timeWindow=10, yLimits=[-50,50])
# addSignals('RESIDUAL_OBSERVER_STATE', msg.utime, msg.residual, joints, keyLookup=names)
# addSignals('RESIDUAL_OBSERVER_STATE_W_FOOT_FORCE', msg.utime, msg.residual, joints, keyLookup=names)
addSignals('RESIDUAL_OBSERVER_STATE', msg.utime, msg.residual, joints, keyLookup=names)

# for ext in channel_extensions:
#     channel = 'CONTACT_FILTER_POINT_ESTIMATE_'+ext
#     addSignals(channel, msg.utime, msg.implied_residual, joints_w_dot, keyLookup=vel_names)


addPlot(timeWindow=10, yLimits=[-2,10])
addSignal('CONTACT_FILTER_POINT_ESTIMATE', msg.utime, msg.logLikelihood, label="squared error")





addPlot(timeWindow=10, yLimits=[0, 4])
addSignal('CONTACT_FILTER_POINT_ESTIMATE', msg.utime, msg.num_contact_points, label="num contact points")




# addPlot(timeWindow=30, yLimits=[-50,50])
# for ext in channel_extensions:
#     channel = 'CONTACT_FILTER_POINT_ESTIMATE_'+ext
#     addSignal(channel, msg.utime, msg.logLikelihood)


# addPlot(timeWindow=30, yLimits=[-10,100])
# for ext in channel_extensions:
#     channel = 'CONTACT_FILTER_POINT_ESTIMATE_'+ext
#     addSignal(channel, msg.utime, msg.contact_force_magnitude)

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
