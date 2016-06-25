import numpy
import colorsys

execfile(os.path.join(os.path.dirname(__file__), 'rotations.py'))

# joints to plot
#joints = ['l_arm_shz','l_arm_shx','l_arm_ely','l_arm_elx','l_arm_uwy','l_arm_mwx','l_arm_lwy']
#joints = ['rightShoulderPitch','rightShoulderRoll','rightShoulderYaw','rightElbowPitch','rightForearmYaw','rightWristRoll','rightWristPitch']
joints = ['rightShoulderPitch','rightShoulderRoll','rightShoulderYaw','rightElbowPitch','rightForearmYaw','rightWristRoll','rightWristPitch']

# string arrays for EST_ROBOT_STATE and ATLAS_COMMAND
jn = msg.joint_name
jns = msg.joint_name


N = len(joints)
#HSV_tuples = [(x*1.0/N, 1.0, 1.0) for x in range(N)]
HSV_tuples =      [(0., 1.0, 1.0),(0.15, 1.0, 1.0), (0.3, 1.0, 1.0), (0.45, 1.0, 1.0), (0.6, 1.0, 1.0), (0.75, 1.0, 1.0), (0.9, 1.0, 1.0)]
HSV_tuples_dark = [(0., 1.0, 0.5),(0.15, 1.0, 0.5), (0.3, 1.0, 0.5), (0.45, 1.0, 0.5), (0.6, 1.0, 0.5), (0.75, 1.0, 0.5), (0.9, 1.0, 0.5)]

RGB_tuples = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples)
RGB_tuples_dark = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples_dark)


import numpy

def rpyFunction(msg):
    return quat_to_euler([msg.pose.rotation.w, msg.pose.rotation.x, msg.pose.rotation.y, msg.pose.rotation.z])
def rollFunction(msg):
    '''roll'''
    return msg.utime, rpyFunction(msg)[0]* 180.0/math.pi
def pitchFunction(msg):
    '''pitch'''
    return msg.utime, rpyFunction(msg)[1]* 180.0/math.pi
def yawFunction(msg):
    '''yaw'''
    return msg.utime, rpyFunction(msg)[2]* 180.0/math.pi

addPlot(timeWindow=15, yLimits=[-180, 180])
addSignalFunction('EST_ROBOT_STATE', rollFunction)
addSignalFunction('EST_ROBOT_STATE', pitchFunction)
addSignalFunction('EST_ROBOT_STATE', yawFunction)
addSignalFunction('COMMITTED_ROBOT_PLAN_STATES', rollFunction)
addSignalFunction('COMMITTED_ROBOT_PLAN_STATES', pitchFunction)
addSignalFunction('COMMITTED_ROBOT_PLAN_STATES', yawFunction)

addPlot(timeWindow=15, yLimits=[-1, 1])
addSignal('EST_ROBOT_STATE', msg.utime, msg.pose.translation.x)
addSignal('EST_ROBOT_STATE', msg.utime, msg.pose.translation.y)
addSignal('EST_ROBOT_STATE', msg.utime, msg.pose.translation.z)
addSignal('COMMITTED_ROBOT_PLAN_STATES', msg.utime, msg.pose.translation.x)
addSignal('COMMITTED_ROBOT_PLAN_STATES', msg.utime, msg.pose.translation.y)
addSignal('COMMITTED_ROBOT_PLAN_STATES', msg.utime, msg.pose.translation.z)



# effort plot
#addPlot(timeWindow=15, yLimits=[-2.75, 2.75])
#addSignals('VAL_COMMAND_FEEDBACK', msg.utime, msg.joint_position, joints, keyLookup=jn, colors=RGB_tuples)
#addSignals('VAL_CORE_ROBOT_STATE', msg.utime, msg.joint_position, joints, keyLookup=jn, colors=RGB_tuples_dark)

# position plot
addPlot(timeWindow=15, yLimits=[-2.75, 2.75])

addSignals('EST_ROBOT_STATE', msg.utime, msg.joint_position, joints, keyLookup=jns, colors=RGB_tuples)
addSignals('COMMITTED_ROBOT_PLAN_STATES', msg.utime, msg.joint_position, joints, keyLookup=jns, colors=RGB_tuples_dark)

#for joint in joints:
#    addSignal('EST_ROBOT_STATE', msg.utime, msg.joint_position[jn[joint]])
#    addSignal('COMMITTED_ROBOT_PLAN_STATES', msg.utime, msg.joint_position[jn[joint]])
