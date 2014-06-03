#!/bin/bash
rosservice call /ServoController/sendCommands "modeCommands: [{resource: '/pelvis/lj1', command: 'Park'}, {resource: '/left_leg/j2', command: 'Park'}, {resource: '/left_leg/j3', command: 'Park'}, {resource: '/left_leg/j4', command: 'Park'}, {resource: '/left_leg/j5', command: 'Park'}, {resource: '/left_leg/j6', command: 'Park'}]"
