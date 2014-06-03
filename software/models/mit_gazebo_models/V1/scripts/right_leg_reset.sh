#!/bin/bash
rosservice call /ServoController/sendCommands "modeCommands: [{resource: '/pelvis/rj1', command: 'Reset'}, {resource: '/right_leg/j2', command: 'Reset'}, {resource: '/right_leg/j3', command: 'Reset'}, {resource: '/right_leg/j4', command: 'Reset'}, {resource: '/right_leg/j5', command: 'Reset'}, {resource: '/right_leg/j6', command: 'Reset'}]"
