import numpy
import colorsys
import functools

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


# msg should be the CONTROLLER_STATE message
def getFootForce(msg, side="l_foot"):
	normalForce = 0 # this is the default

	contact_output_vec = msg.contact_output

	for contactData in contact_output_vec:
		bodyName = str(contactData.body_name)

		if (bodyName == side):
			normalForce = contactData.wrench[5]

	return msg.timestamp, normalForce

# this should be QP_CONTROLLER_INPUT msg
def getFootZForceBound(msg, side="l_foot"):
	zForceBound = 0

	for support_data in msg.support_data:
		bodyName = str(support_data.body_name)

		if (bodyName == side):
			zForceBound = support_data.total_normal_force_upper_bound


	return msg.timestamp, zForceBound


# foot forces
addPlot(timeWindow=time_window, yLimits=[0, 1000])

addSignal('FORCE_TORQUE', msg.utime, msg.sensors[0].force[2]) # left foot
addSignal('FORCE_TORQUE', msg.utime, msg.sensors[1].force[2]) # right foot

addSignalFunction("CONTROLLER_STATE", functools.partial(getFootForce, side='l_foot'))
addSignalFunction("CONTROLLER_STATE", functools.partial(getFootForce, side='r_foot'))


# plot the force bounds
addSignalFunction("QP_CONTROLLER_INPUT", functools.partial(getFootZForceBound, side='l_foot'))

addSignalFunction("QP_CONTROLLER_INPUT", functools.partial(getFootZForceBound, side='r_foot'))


# foot contact estimate
addPlot(timeWindow=time_window, yLimits=[0,1.5])

addSignal("FOOT_CONTACT_ESTIMATE", msg.utime, msg.left_contact)
addSignal("FOOT_CONTACT_ESTIMATE", msg.utime, msg.right_contact)