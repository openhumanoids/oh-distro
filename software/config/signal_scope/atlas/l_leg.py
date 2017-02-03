import numpy
import colorsys

# joints to plot
joints = ["rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll"]

left_arm_joints = ['l_arm_shz','l_arm_shx','l_arm_ely','l_arm_elx','l_arm_uwy',
  'l_arm_mwx', 'l_arm_lwy']


left_leg_joints = ['l_leg_hpz','l_leg_hpx','l_leg_hpy','l_leg_kny','l_leg_aky','l_leg_akx']
left_leg_joints = ['l_leg_hpz','l_leg_kny','l_leg_aky', 'r_leg_hpz','r_leg_kny','r_leg_aky']

joints = left_leg_joints

# string arrays for EST_ROBOT_STATE and ATLAS_COMMAND
jn = msg.joint_name
jns = msg.joint_names
time_window = 5

N = len(joints)
HSV_tuples =      [(0., 1.0, 1.0),(0.15, 1.0, 1.0), (0.3, 1.0, 1.0), (0.45, 1.0, 1.0), (0.6, 1.0, 1.0), (0.75, 1.0, 1.0), (0.9, 1.0, 1.0)]
HSV_tuples_dark = [(0., 1.0, 0.5),(0.15, 1.0, 0.5), (0.3, 1.0, 0.5), (0.45, 1.0, 0.5), (0.6, 1.0, 0.5), (0.75, 1.0, 0.5), (0.9, 1.0, 0.5)]
HSV_tuples_v3 = [(0.,0.75, 0.5),(0.15,0.75, 0.5), (0.3,0.75, 0.5), (0.45,0.75, 0.5), (0.6,0.75, 0.5), (0.75,0.75, 0.5), (0.9,0.75, 0.5)]

HSV_tuples =      [(0.15, 1.0, 1.0), (0.3, 1.0, 1.0), (0.45, 1.0, 1.0), (0.6, 1.0, 1.0), (0.75, 1.0, 1.0), (0.9, 1.0, 1.0)]
HSV_tuples_dark = [(0., 1.0, 0.5),(0.15, 1.0, 0.5), (0.3, 1.0, 0.5), (0.45, 1.0, 0.5), (0.6, 1.0, 0.5), (0.75, 1.0, 0.5), (0.9, 1.0, 0.5)]
HSV_tuples_v3 = [(0.3,0.75, 0.5), (0.45,0.75, 0.5), (0.6,0.75, 0.5), (0.75,0.75, 0.5), (0.9,0.75, 0.5)]

RGB_tuples = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples)
RGB_tuples_dark = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples_dark)
RGB_tuples_v3 = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples_v3)

# velocity plot
addPlot(timeWindow=time_window, yLimits=[-2, 2])

addSignals('EST_ROBOT_STATE', msg.utime, msg.joint_velocity, joints, keyLookup=jn, colors=RGB_tuples_v3)
addSignals('CONTROLLER_STATE', msg.timestamp, msg.vref_integrator_state, joints, keyLookup=jn, colors=RGB_tuples)

# addSignals('CORE_ROBOT_STATE', msg.utime, msg.joint_velocity, joints, keyLookup=jn, colors=RGB_tuples_dark)

# effort plot
addPlot(timeWindow=time_window, yLimits=[-200,200])

# addSignals('CONTROLLER_STATE', msg.timestamp, msg.u, joints, keyLookup=jn,colors=RGB_tuples_dark)

addSignals('EST_ROBOT_STATE', msg.utime, msg.joint_effort, joints, keyLookup=jn, colors=RGB_tuples_v3)


# addSignals('CORE_ROBOT_STATE', msg.utime, msg.joint_effort, joints, keyLookup=jn, colors=RGB_tuples_dark)

# addSignals('ATLAS_COMMAND', msg.utime, msg.effort, joints, keyLookup=jns, colors=RGB_tuples)




