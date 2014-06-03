#!/bin/bash
rosservice call /ServoController/sendCommands "modeCommands: [{resource: '/pelvis/rj1', command: 'Park'}, {resource: '/right_leg/j2', command: 'Park'}, {resource: '/right_leg/j3', command: 'Park'}, {resource: '/right_leg/j4', command: 'Park'}, {resource: '/right_leg/j5', command: 'Park'}, {resource: '/right_leg/j6', command: 'Park'}]"
