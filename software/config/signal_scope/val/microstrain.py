import numpy
import colorsys

execfile(os.path.join(os.path.dirname(__file__), 'rotations.py'))

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


def rollFunctionSimple(msg):
    '''rollins'''
    return msg.utime, quat_to_euler(msg.quat)[0]* 180.0/math.pi
def pitchFunctionSimple(msg):
    '''pitchins'''
    return msg.utime, quat_to_euler(msg.quat)[1]* 180.0/math.pi
def yawFunctionSimple(msg):
    '''yawins'''
    return msg.utime, quat_to_euler(msg.quat)[2]* 180.0/math.pi

addPlot(timeWindow=15, yLimits=[-180, 180])
#addSignalFunction('EST_ROBOT_STATE', rollFunction)
#addSignalFunction('EST_ROBOT_STATE', pitchFunction)
#addSignalFunction('EST_ROBOT_STATE', yawFunction)

addSignalFunction('IMU_pelvisRearImu', rollFunctionSimple)
addSignalFunction('IMU_pelvisRearImu', pitchFunctionSimple)
addSignalFunction('IMU_pelvisRearImu', yawFunctionSimple)

addPlot(timeWindow=15, yLimits=[-1, 1])
addSignal('IMU_pelvisRearImu', msg.utime, msg.gyro[0])
addSignal('IMU_pelvisRearImu', msg.utime, msg.gyro[1])
addSignal('IMU_pelvisRearImu', msg.utime, msg.gyro[2])

addPlot(timeWindow=15, yLimits=[-1, 1])
addSignal('IMU_pelvisRearImu', msg.utime, msg.accel[0])
addSignal('IMU_pelvisRearImu', msg.utime, msg.accel[1])
addSignal('IMU_pelvisRearImu', msg.utime, msg.accel[2])



#addPlot(timeWindow=15, yLimits=[-1, 1])
#addSignal('EST_ROBOT_STATE', msg.utime, msg.pose.translation.x)
#addSignal('EST_ROBOT_STATE', msg.utime, msg.pose.translation.y)
#addSignal('EST_ROBOT_STATE', msg.utime, msg.pose.translation.z)

