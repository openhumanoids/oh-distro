import numpy
import colorsys
# execfile(os.path.join(os.path.dirname(__file__), 'rotations.py'))

# joints to plot
joints = ["rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll"]
joints = ['rightShoulderPitch']

# string arrays for EST_ROBOT_STATE and ATLAS_COMMAND
jn = msg.joint_name
jns = msg.joint_names

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

timeWindow = 5

# position plot
addPlot(timeWindow=timeWindow, yLimits=[-1.5, 1.5])
addSignals('CORE_ROBOT_STATE', msg.utime, msg.joint_position, joints, keyLookup=jn, colors=RGB_tuples)
# addSignals('VAL_CORE_ROBOT_STATE', msg.utime, msg.joint_position, joints, keyLookup=jn, colors=RGB_tuples_dark)
addSignals('ROBOT_COMMAND', msg.utime, msg.position, joints, keyLookup=jns)
addSignals('EST_ROBOT_STATE', msg.utime, msg.joint_position, joints, keyLookup=jn, colors=RGB_tuples_v3)
addSignals('CONTROLLER_Q_DES', msg.utime, msg.joint_position, joints, keyLookup=jn)
# addSignal('QP_CONTROLLER_INPUT', msg.timestamp, msg.whole_body_data.q_des[7])

# velocity plot
addPlot(timeWindow=timeWindow, yLimits=[-1.5, 1.5])
addSignals('CORE_ROBOT_STATE', msg.utime, msg.joint_velocity, joints, keyLookup=jn, colors=RGB_tuples)
# addSignals('VAL_CORE_ROBOT_STATE', msg.utime, msg.joint_velocity, joints, keyLookup=jn, colors=RGB_tuples_dark)
addSignals('EST_ROBOT_STATE', msg.utime, msg.joint_velocity, joints, keyLookup=jn, colors=RGB_tuples_v3)
addSignals('CONTROLLER_Q_DES', msg.utime, msg.joint_velocity, joints, keyLookup=jn)

# effort plot
addPlot(timeWindow=timeWindow, yLimits=[-40, 40])
# addSignals('CORE_ROBOT_STATE', msg.utime, msg.joint_effort, joints, keyLookup=jn, colors=RGB_tuples)
# addSignals('LCM2ROSCONTROL_lcm_torque', msg.utime, msg.joint_position, joints, keyLookup=jn, colors=RGB_tuples_dark)
# addSignals('VAL_CORE_ROBOT_STATE', msg.utime, msg.joint_effort, joints, keyLookup=jn, colors=RGB_tuples_dark)
addSignals('EST_ROBOT_STATE', msg.utime, msg.joint_effort, joints, keyLookup=jn, colors=RGB_tuples_v3)
addSignals('ROBOT_COMMAND', msg.utime, msg.effort, joints, keyLookup=jns)
addSignals('ROBOT_COMMAND_LOG', msg.utime, msg.effort, joints, keyLookup=jns)
