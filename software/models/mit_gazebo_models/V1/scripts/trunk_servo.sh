#!/bin/bash
rosservice call /ServoController/sendCommands "modeCommands: [{resource: '/pelvis/wj1', command: 'PositionMode'}, {resource: '/trunk/wj2', command: 'PositionMode'}, {resource: '/trunk/wj3', command: 'PositionMode'}]"
