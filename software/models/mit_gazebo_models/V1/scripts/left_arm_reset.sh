#!/bin/bash
rosservice call /ServoController/sendCommands "modeCommands: [{resource: '/trunk/laj1', command: 'Reset'}, {resource: '/left_arm/j2', command: 'Reset'}, {resource: '/left_arm/j3', command: 'Reset'}, {resource: '/left_arm/j4', command: 'Reset'}, {resource: '/left_arm/j5', command: 'Reset'}]"
