#!/bin/bash
rosservice call /ServoController/sendCommands "modeCommands: [{resource: '/pelvis/wj1', command: 'Park'}, {resource: '/trunk/wj2', command: 'Park'}, {resource: '/trunk/wj3', command: 'Park'}]"
