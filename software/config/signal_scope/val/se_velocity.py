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

addPlot(timeWindow=5, yLimits=[-1, 1])
addSignal('POSE_BDI', msg.utime, msg.vel[0])
addSignal('POSE_BODY', msg.utime, msg.vel[0])
addSignal('POSE_BODY_LEGODO_VELOCITY', msg.utime, msg.vel[0])
addSignal('POSE_BODY_LEGODO_VELOCITY_FAIL', msg.utime, msg.vel[0])

addPlot(timeWindow=5, yLimits=[-1, 1])
addSignal('POSE_BDI', msg.utime, msg.vel[1])
addSignal('POSE_BODY', msg.utime, msg.vel[1])
addSignal('POSE_BODY_LEGODO_VELOCITY', msg.utime, msg.vel[1])
addSignal('POSE_BODY_LEGODO_VELOCITY_FAIL', msg.utime, msg.vel[1])

addPlot(timeWindow=5, yLimits=[-1, 1])
addSignal('POSE_BDI', msg.utime, msg.vel[2])
addSignal('POSE_BODY', msg.utime, msg.vel[2])
addSignal('POSE_BODY_LEGODO_VELOCITY', msg.utime, msg.vel[2])
addSignal('POSE_BODY_LEGODO_VELOCITY_FAIL', msg.utime, msg.vel[2])

#addPlot(timeWindow=5, yLimits=[-1500, 0])
#addSignal('FORCE_TORQUE', msg.utime, msg.sensors[0].force[2])
#addSignal('FORCE_TORQUE', msg.utime, msg.sensors[1].force[2])

addPlot(timeWindow=5, yLimits=[-5, 5])
addSignal('LEG_ODOMETRY_DELTA', msg.utime, msg.translation[0])
addSignal('LEG_ODOMETRY_DELTA', msg.utime, msg.translation[1])
addSignal('LEG_ODOMETRY_DELTA', msg.utime, msg.translation[2])


