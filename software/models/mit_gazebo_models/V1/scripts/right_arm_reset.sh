#!/bin/bash
rosservice call /ServoController/sendCommands "modeCommands: [{resource: '/trunk/raj1', command: 'Reset'}, {resource: '/right_arm/j2', command: 'Reset'}, {resource: '/right_arm/j3', command: 'Reset'}, {resource: '/right_arm/j4', command: 'Reset'}, {resource: '/right_arm/j5', command: 'Reset'}]"
