import numpy
import numpy as np
import colorsys
from director import transformUtils

# global atlas_command_last_utime, core_robot_state_last_utime, est_robot_state_last_utime
core_robot_state_last_utime = 0
atlas_command_last_utime = 0
est_robot_state_last_utime = 0

def dtAtlasCommand(msg, storage=[0]):
	dt_in_ms = 1.0/1e3 * (msg.utime - storage[0])
	storage[0] = msg.utime
	return msg.utime, dt_in_ms

def dtCoreRobotState(msg, storage=[0]):
	dt_in_ms = 1.0/1e3 * (msg.utime - storage[0])
	storage[0] = msg.utime
	return msg.utime, dt_in_ms

def dtEstRobotState(msg, storage=[0]):
	dt_in_ms = 1.0/1e3 * (msg.utime - storage[0])
	storage[0] = msg.utime
	return msg.utime, dt_in_ms

def dtPoseBody(msg, storage=[0]):
	dt_in_ms = 1.0/1e3 * (msg.utime - storage[0])
	storage[0] = msg.utime
	return msg.utime, dt_in_ms

def dtQPControllerInput(msg, storage=[0]):
	dt_in_ms = 1.0/1e3 * (msg.timestamp - storage[0])
	storage[0] = msg.timestamp
	return msg.timestamp, dt_in_ms


# position plot
addPlot(timeWindow=5, yLimits=[-2, 10])
addSignalFunction('CORE_ROBOT_STATE', dtCoreRobotState)
addSignalFunction('EST_ROBOT_STATE', dtEstRobotState)
addSignalFunction('ATLAS_COMMAND', dtAtlasCommand)
addSignalFunction('QP_CONTROLLER_INPUT', dtQPControllerInput)
addSignalFunction('POSE_BODY', dtPoseBody)













