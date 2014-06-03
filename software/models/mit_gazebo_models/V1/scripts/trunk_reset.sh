#!/bin/bash
rosservice call /ServoController/sendCommands "modeCommands: [{resource: '/pelvis/wj1', command: 'Reset'}, {resource: '/trunk/wj2', command: 'Reset'}, {resource: '/trunk/wj3', command: 'Reset'}]"
