import numpy
import math
execfile(os.path.join(os.path.dirname(__file__), 'rotations.py'))


import numpy

def rpyFunctionPose(msg):
    return quat_to_euler(msg.orientation)
def rollFunctionPose(msg):
    '''roll'''
    return msg.utime, rpyFunctionPose(msg)[0]* 180.0/math.pi
def pitchFunctionPose(msg):
    '''pitch'''
    return msg.utime, rpyFunctionPose(msg)[1]* 180.0/math.pi
def yawFunctionPose(msg):
    '''yaw'''
    return msg.utime, rpyFunctionPose(msg)[2]* 180.0/math.pi

addPlot(timeWindow=15, yLimits=[-180, 180])
addSignalFunction('POSE_BODY', rollFunctionPose)
addSignalFunction('POSE_BODY_ALT', rollFunctionPose)
addSignalFunction('POSE_VICON', rollFunctionPose)

addPlot(timeWindow=15, yLimits=[-180, 180])
addSignalFunction('POSE_BODY', pitchFunctionPose)
addSignalFunction('POSE_BODY_ALT', pitchFunctionPose)
addSignalFunction('POSE_VICON', pitchFunctionPose)

addPlot(timeWindow=15, yLimits=[-180, 180])
addSignalFunction('POSE_BODY', yawFunctionPose)
addSignalFunction('POSE_BODY_ALT', yawFunctionPose)
addSignalFunction('POSE_VICON', yawFunctionPose)

addPlot(timeWindow=15, yLimits=[-180, 180])
addSignal('POSE_BODY', msg.utime, msg.pos[0])
addSignal('POSE_BODY', msg.utime, msg.pos[1])
addSignal('POSE_BODY', msg.utime, msg.pos[2])


