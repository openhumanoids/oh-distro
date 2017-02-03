import numpy
import numpy as np
import colorsys
from director import transformUtils

# joints to plot
joints = ["rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll"]

left_arm_joints = ['l_arm_shz','l_arm_shx','l_arm_ely','l_arm_elx','l_arm_uwy',
  'l_arm_mwx', 'l_arm_lwy']


joints = left_arm_joints

joints = ['l_arm_shx']
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

def myFunction(msg):
	rotation = msg.pose.rotation
	quat = numpy.array([rotation.w, rotation.x, rotation.y, rotation.z])
	rpy = transformUtils.quaternionToRollPitchYaw(quat)
	return msg.utime, rpy[2]

def myFunctionBDI(msg):
	quat = numpy.array([msg.orientation[0], msg.orientation[1], msg.orientation[2], msg.orientation[3]])
	rpy = transformUtils.quaternionToRollPitchYaw(quat)
	return msg.utime, rpy[2]



def poseY(msg):
	rotation = msg.pose.rotation
	quat = numpy.array([rotation.w, rotation.x, rotation.y, rotation.z])
	rpy = transformUtils.quaternionToRollPitchYaw(quat)
	return msg.utime, rpy[1]

def poseYBDI(msg):
	quat = numpy.array([msg.orientation[0], msg.orientation[1], msg.orientation[2], msg.orientation[3]])
	rpy = transformUtils.quaternionToRollPitchYaw(quat)
	return msg.utime, rpy[1]

# msg is POSE_BODY_ALT coming from BDI
def velInBodyFrame(msg):
	quat = numpy.array([msg.orientation[0], msg.orientation[1], msg.orientation[2], msg.orientation[3]])

	vel_in_world = np.array([msg.vel[0],msg.vel[1],msg.vel[2]])

	pose_hack = np.array([0,0,0])

	T_body_to_world = transformUtils.transformFromPose(pose_hack, quat)
	T_world_to_body = T_body_to_world.GetLinearInverse()

	vel_in_body = T_world_to_body.TransformVector(vel_in_world)

	return vel_in_body


def x_velInBodyFrame(msg):
	vel_in_body = velInBodyFrame(msg)
	return msg.utime, vel_in_body[0]





bdi_pose_channel = "POSE_BODY_ALT"

# position plot
addPlot(timeWindow=5, yLimits=[-1.5, 1.5])

addSignalFunction('EST_ROBOT_STATE', myFunction)
addSignal('EST_ROBOT_STATE', msg.utime, msg.pose.translation.x);

# addSignalFunction('EST_ROBOT_STATE_1', myFunction)
# addSignalFunction('EST_ROBOT_STATE_ORIGINAL', myFunction)
addSignalFunction('POSE_BODY', myFunctionBDI)
addSignalFunction(bdi_pose_channel, myFunctionBDI)

# addSignalFunction('EST_ROBOT_STATE', poseY)
# addSignalFunction(pose_body_alt, poseYBDI)


# velocity plot
addPlot(timeWindow=5, yLimits=[-1.5, 1.5])
#addSignal('POSE_BODY', msg.utime, msg.rotation_rate[2])
#addSignal('POSE_BODY_ORIGINAL', msg.utime, msg.rotation_rate[2])

addSignal('EST_ROBOT_STATE', msg.utime, msg.twist.angular_velocity.z)
addSignal('POSE_BODY', msg.utime, msg.rotation_rate[2])


# addSignal('EST_ROBOT_STATE', msg.utime, msg.twist.angular_velocity.y)
# addSignal('POSE_BDI', msg.utime, msg.rotation_rate[1])

addPlot(timeWindow=5, yLimits=[-1.5, 1.5])
# addSignal('EST_ROBOT_STATE', msg.utime, msg.twist.linear_velocity.x)
addSignal('POSE_BODY', msg.utime, msg.vel[0])
addSignal(bdi_pose_channel, msg.utime, msg.vel[0])
#addSignalFunction(bdi_pose_channel, x_velInBodyFrame)











