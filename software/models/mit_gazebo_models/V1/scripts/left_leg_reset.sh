#!/bin/bash
rosservice call /ServoController/sendCommands "modeCommands: [{resource: '/pelvis/lj1', command: 'Reset'}, {resource: '/left_leg/j2', command: 'Reset'}, {resource: '/left_leg/j3', command: 'Reset'}, {resource: '/left_leg/j4', command: 'Reset'}, {resource: '/left_leg/j5', command: 'Reset'}, {resource: '/left_leg/j6', command: 'Reset'}]"
