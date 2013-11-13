#!/usr/bin/env python

from mit_irobot_hand_control import IRobotHandController
import numpy

def get_motor_encoder_offsets(controller):
    return dict((i, controller.config_parser.get_motor_encoder_offset(i)) for i in controller.get_motor_indices())

if __name__ == '__main__':
    controller = IRobotHandController('r')
    motor_indices = controller.get_motor_indices()
    zero_pose = dict((i, 0) for i in motor_indices)
    
    repetitions = 1
    open_hand_time = 5 # to settle down and also to let the motors cool down
    open_hand_amount = 4000
    open_hand_desireds = dict((i, -open_hand_amount) for i in motor_indices)
    
    calibration_poses = dict((i, []) for i in motor_indices)
    for i in range(repetitions):
        # calibrate with zero offset
        controller.calibrate_motor_encoder_offsets(zero_pose)
        offsets = get_motor_encoder_offsets(controller)
        
        # save calibration pose
        [calibration_poses[motor_index].append(offsets[motor_index]) for motor_index in motor_indices]
        
        # move away from calibration pose
        controller.motor_excursion_control_loop(open_hand_desireds, open_hand_time)

    means = dict((i, numpy.mean(calibration_poses[i])) for i in motor_indices)
    standard_deviations = dict((i, numpy.std(calibration_poses[i])) for i in motor_indices)

    print calibration_poses
    print "means: %s" % means
    print "standard deviations: %s" % standard_deviations
