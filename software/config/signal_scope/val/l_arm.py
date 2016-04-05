import numpy
import colorsys

execfile(os.path.join(os.path.dirname(__file__), 'rotations.py'))

# joints to plot
#joints = ['l_arm_shz','l_arm_shx','l_arm_ely','l_arm_elx','l_arm_uwy','l_arm_mwx','l_arm_lwy']
#joints = ['leftShoulderPitch','leftShoulderRoll','leftShoulderYaw','leftElbowPitch','leftForearmYaw','leftWristRoll','leftWristPitch']
joints = ['leftShoulderPitch','leftShoulderRoll','leftShoulderYaw','leftElbowPitch']#,'leftForearmYaw']#,'leftWristRoll','leftWristPitch']

# string arrays for EST_ROBOT_STATE and ATLAS_COMMAND
jn = msg.joint_name
jns = msg.joint_names


N = len(joints)
#HSV_tuples = [(x*1.0/N, 1.0, 1.0) for x in range(N)]
HSV_tuples =      [(0., 1.0, 1.0),(0.15, 1.0, 1.0), (0.3, 1.0, 1.0), (0.45, 1.0, 1.0), (0.6, 1.0, 1.0), (0.75, 1.0, 1.0), (0.9, 1.0, 1.0)]
HSV_tuples_dark = [(0., 1.0, 0.5),(0.15, 1.0, 0.5), (0.3, 1.0, 0.5), (0.45, 1.0, 0.5), (0.6, 1.0, 0.5), (0.75, 1.0, 0.5), (0.9, 1.0, 0.5)]

RGB_tuples = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples)
RGB_tuples_dark = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples_dark)

# effort plot
addPlot(timeWindow=15, yLimits=[-2.75, 2.75])
addSignals('VAL_COMMAND_FEEDBACK', msg.utime, msg.joint_position, joints, keyLookup=jn, colors=RGB_tuples)
addSignals('VAL_CORE_ROBOT_STATE', msg.utime, msg.joint_position, joints, keyLookup=jn, colors=RGB_tuples_dark)

# position plot
#addPlot(timeWindow=15, yLimits=[-2.75, 2.75])
#for joint in joints:
#    addSignal('EST_ROBOT_STATE', msg.utime, msg.joint_position[jn[joint]])
#    addSignal('COMMITTED_ROBOT_PLAN_STATES', msg.utime, msg.joint_position[jn[joint]])
