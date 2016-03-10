import numpy
import math
execfile(os.path.join(os.path.dirname(__file__), 'rotations.py'))

addPlot(timeWindow=15, yLimits=[-1, 1])
addSignal('EST_ROBOT_STATE', msg.utime, msg.force_torque.l_foot_force_z)
addSignal('EST_ROBOT_STATE', msg.utime, msg.force_torque.r_foot_force_z)


addPlot(timeWindow=15, yLimits=[-1, 1])
addSignal('EST_ROBOT_STATE', msg.utime, msg.force_torque.l_foot_torque_x)
addSignal('EST_ROBOT_STATE', msg.utime, msg.force_torque.l_foot_torque_y)
addSignal('EST_ROBOT_STATE', msg.utime, msg.force_torque.r_foot_torque_x)
addSignal('EST_ROBOT_STATE', msg.utime, msg.force_torque.r_foot_torque_y)



