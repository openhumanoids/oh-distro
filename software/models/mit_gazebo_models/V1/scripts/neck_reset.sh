#!/bin/bash
rosservice call /ServoController/sendCommands "modeCommands: [{resource: '/neck/j1', command: 'Reset'}, {resource: '/neck/j2', command: 'Reset'}, {resource: '/neck/j3', command: 'Reset'}]"
