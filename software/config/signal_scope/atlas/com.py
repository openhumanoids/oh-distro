import numpy
import colorsys

# joints to plot
joints = ["rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll"]

left_arm_joints = ['l_arm_shz','l_arm_shx','l_arm_ely','l_arm_elx','l_arm_uwy',
  'l_arm_mwx', 'l_arm_lwy']


joints = left_arm_joints

joints = ['l_arm_shx']
# string arrays for EST_ROBOT_STATE and ATLAS_COMMAND
jn = msg.joint_name
jns = msg.joint_names

time_window = 2

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

keys = [0,1]

# position plot
addPlot(timeWindow=time_window, yLimits=[-2, 2])

addSignals('PLAN_EVAL_DEBUG', msg.timestamp, msg.com_des, keys)
addSignals('PLAN_EVAL_DEBUG', msg.timestamp, msg.comdot_des, keys)


addPlot(timeWindow = time_window)
addSignals('PLAN_EVAL_DEBUG', msg.timestamp, msg.comddot_des, keys)

# this is after applying the alpha filter to desired com acceleration
addSignals('CONTROLLER_STATE', msg.timestamp, msg.comdd_des, keys)
addSignals('CONTROLLER_STATE', msg.timestamp, msg.comdd_des_unfiltered, keys)

