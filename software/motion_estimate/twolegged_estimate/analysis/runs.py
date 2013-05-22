import numpy as np
import subprocess
import os
import sys
import shutil
import time
import shlex
import errno

def writeWeights(maxDuration, numSamples, profileType, fileName):
    step = maxDuration/numSamples
    times = (np.arange(numSamples)+1)*step
    weights = np.empty(np.shape(times))
    n = len(times)
    for i in range(n):
        if (profileType == 'uniform'):
            weights[i] = 1.0
        elif (profileType == 'tri'):
            if (i < n/2):
                weights[i] = i+1
            else:
                weights[i] = n-i
        elif (profileType == 'ramp'):
            weights[i] = i+1
        elif (profileType == 'revramp'):
            weights[i] = n-i
        elif (profileType == 'revtri'):
            if (i < n/2):
                weights[i] = n-i
            else:
                weights[i] = i+1
    weights /= weights.sum()
    
    f = open(fileName,'w')
    f.write(str(1000) + '\n')
    for i in range(n):
        f.write(str(times[i]*1000) + ', ' + str(weights[i]) + '\n')


def doRun(logFile, maxDuration, profileType, prefix):
    #prefixName = os.path.split(prefix)[1]
    logName = os.path.split(os.path.split(logFile)[0])[1];
    runName = logName + '_' + str(maxDuration) + '_' + profileType
    sys.stdout.write('starting run %s ...\n' % (runName))
    
    # write weights file
    weightFile = '/home/antone/TODO_FILL_ME_IN.txt';
    writeWeights(maxDuration, 30, profileType, weightFile)
    sys.stdout.write('  wrote weights file %s\n' % weightFile)
    
    # start up param server
    #paramServerExe = 'bot-param-server'
    #paramServerArgs = os.path.expandvars('${HOME}/drc/software/config/drc_robot.cfg')
    #paramServerCmd = shlex.split(paramServerExe + ' ' + paramServerArgs)
    #paramServerProc = subprocess.Popen(paramServerCmd)
    #sys.stdout.write('  spawned param server\n')

    ## start up model publisher
    #modelPublisherExe = 'robot_model_publisher'
    #modelPublisherArgs = os.path.expandvars('-u ${HOME}/drc/software/models/mit_gazebo_models/mit_robot_nohands/model.urdf')
    #modelPublisherCmd = shlex.split(modelPublisherExe + ' ' + modelPublisherArgs)
    #modelPublisherProc = subprocess.Popen(modelPublisherCmd)
    #sys.stdout.write('  spawned model publisher\n')

    # start up motion estimator
    motionExe = 'drc-legged-odometry'
    motionArgs = '-e -l'
    motionCmd = shlex.split(motionExe + ' ' + motionArgs)
    motionProc = subprocess.Popen(motionCmd, cwd='/tmp')
    sys.stdout.write('  spawned motion estimator\n')
    
    # start up logplayer
    playerExe = 'lcm-logplayer'
    playerArgs = '-s 8 %s' % logFile
    playerCmd = shlex.split(playerExe + ' ' + playerArgs)
    playerProc = subprocess.Popen(playerCmd)
    sys.stdout.write('  playing log %s...' % logFile)
    playerProc.wait()
    sys.stdout.write('done\n')

    # kill everything when logplayer is done
    time.sleep(3)
    sys.stdout.write('  terminating processes...')
    motionProc.terminate()
    #modelPublisherProc.terminate()
    #paramServerProc.terminate()
    sys.stdout.write('done\n')
    
    # copy files
    shutil.move('/tmp/true_estimated_states.csv','%s/%s.csv' % (prefix,runName))
    sys.stdout.write('********** ALL DONE ***********\n')
    

def doAllRuns(logListFile,prefix):
    prefix = os.path.expandvars(prefix)
    try:
        os.makedirs(prefix)
    except OSError as exc:
        if exc.errno == errno.EEXIST and os.path.isdir(prefix): pass
        else: raise
    files = open(os.path.expandvars(logListFile),'r').readlines()
    windowDurations = [30, 60, 90]
    weightProfiles = ['uniform','tri','ramp','revramp','revtri']
    for filename in files:
        for duration in windowDurations:
            for profile in weightProfiles:
                doRun(filename, duration, profile, prefix)
