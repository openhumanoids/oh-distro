#!/bin/bash
rosservice call /ServoController/sendCommands "modeCommands: [{resource: '/neck/j1', command: 'Park'}, {resource: '/neck/j2', command: 'Park'}, {resource: '/neck/j3', command: 'Park'}]"
