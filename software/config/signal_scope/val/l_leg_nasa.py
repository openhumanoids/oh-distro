import numpy
import colorsys

# joints to plot
joints = ["leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll"]
# string arrays for EST_ROBOT_STATE and ATLAS_COMMAND
jn = msg.joint_name

N = len(joints)
#HSV_tuples = [(x*1.0/N, 1.0, 1.0) for x in range(N)]
HSV_tuples =      [(0., 1.0, 1.0),(0.15, 1.0, 1.0), (0.3, 1.0, 1.0), (0.45, 1.0, 1.0), (0.6, 1.0, 1.0), (0.75, 1.0, 1.0), (0.9, 1.0, 1.0)]
HSV_tuples_dark = [(0., 1.0, 0.5),(0.15, 1.0, 0.5), (0.3, 1.0, 0.5), (0.45, 1.0, 0.5), (0.6, 1.0, 0.5), (0.75, 1.0, 0.5), (0.9, 1.0, 0.5)]

RGB_tuples = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples)
RGB_tuples_dark = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples_dark)


# position plot
addPlot(timeWindow=15, yLimits=[-2.75, 2.75])
addSignals('VAL_COMMAND_FEEDBACK', msg.utime, msg.joint_position, joints, keyLookup=jn, colors=RGB_tuples)
addSignals('VAL_CORE_ROBOT_STATE', msg.utime, msg.joint_position, joints, keyLookup=jn, colors=RGB_tuples_dark)

# velocity plot
addPlot(timeWindow=15, yLimits=[-2.75, 2.75])
addSignals('VAL_COMMAND_FEEDBACK', msg.utime, msg.joint_velocity, joints, keyLookup=jn, colors=RGB_tuples)
addSignals('VAL_CORE_ROBOT_STATE', msg.utime, msg.joint_velocity, joints, keyLookup=jn, colors=RGB_tuples_dark)

# effort plot
addPlot(timeWindow=15, yLimits=[-2.75, 2.75])
addSignals('VAL_COMMAND_FEEDBACK', msg.utime, msg.joint_effort, joints, keyLookup=jn, colors=RGB_tuples)
addSignals('VAL_CORE_ROBOT_STATE', msg.utime, msg.joint_effort, joints, keyLookup=jn, colors=RGB_tuples_dark)
