#!/bin/bash
rosservice call /ServoController/sendCommands "modeCommands: [{resource: '/neck/j1', command: 'PositionMode'}, {resource: '/neck/j2', command: 'PositionMode'}, {resource: '/neck/j3', command: 'PositionMode'}]"
