#!/bin/bash
rosservice call /ServoController/sendCommands "modeCommands: [{resource: '/pelvis/lj1', command: 'PositionMode'}, {resource: '/left_leg/j2', command: 'PositionMode'}, {resource: '/left_leg/j3', command: 'PositionMode'}, {resource: '/left_leg/j4', command: 'PositionMode'}, {resource: '/left_leg/j5', command: 'PositionMode'}, {resource: '/left_leg/j6', command: 'PositionMode'}]"
