#!/bin/bash
rosservice call /ServoController/sendCommands "modeCommands: [{resource: '/pelvis/rj1', command: 'PositionMode'}, {resource: '/right_leg/j2', command: 'PositionMode'}, {resource: '/right_leg/j3', command: 'PositionMode'}, {resource: '/right_leg/j4', command: 'PositionMode'}, {resource: '/right_leg/j5', command: 'PositionMode'}, {resource: '/right_leg/j6', command: 'PositionMode'}]"
