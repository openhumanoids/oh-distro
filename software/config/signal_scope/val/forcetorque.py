import numpy
import math
execfile(os.path.join(os.path.dirname(__file__), 'rotations.py'))


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



addPlot(timeWindow=15, yLimits=[-1, 1])
addSignal('EST_ROBOT_STATE', msg.utime, msg.force_torque.l_foot_force_z)
addSignal('EST_ROBOT_STATE', msg.utime, msg.force_torque.r_foot_force_z)

addSignal('FORCE_TORQUE', msg.utime, msg.sensors[0].force[2],label="left")
addSignal('FORCE_TORQUE', msg.utime, msg.sensors[1].force[2],label="right")

#addPlot(timeWindow=15, yLimits=[-1, 1])
#addSignal('EST_ROBOT_STATE', msg.utime, msg.force_torque.l_foot_torque_x)
#addSignal('EST_ROBOT_STATE', msg.utime, msg.force_torque.l_foot_torque_y)
#addSignal('EST_ROBOT_STATE', msg.utime, msg.force_torque.r_foot_torque_x)
#addSignal('EST_ROBOT_STATE', msg.utime, msg.force_torque.r_foot_torque_y)



