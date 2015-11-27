#!/usr/bin/python
import os,sys
import time
import datetime
import subprocess

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

import lcm
import multisense as lcmmultisense

print "drc-send-rotate-and-record: "

class LCMLoggerManager(object):
    '''
    This class provides some convenient methods for managing instances of
    the lcm-logger process.  You can start/stop instances of the lcm-logger
    process, and search for existing instances of the process.  It also has
    support for parsing command lines to attempt to extract log file names
    from running instances of lcm-logger.
    '''

    def __init__(self):
        self.filePatternPrefix = 'multisenselog'
        self.baseDir = os.path.expanduser('~/logs/raw')
        self.existingLoggerProcesses = {}

    @staticmethod
    def getTimeTag():
        return datetime.datetime.now().strftime('%Y-%m-%d__%H-%M')

    def timestamp_now(): 
        return int (time.time () * 1000000)

    def startNewLogger(self, tag='', baseDir=None):
        filePattern = [self.filePatternPrefix, self.getTimeTag()]
        if tag:
            filePattern.append(tag)
        filePattern = '__'.join(filePattern)

        if baseDir is None:
            baseDir = self.baseDir

        fileArg = os.path.join(baseDir, filePattern)
        command = ['lcm-logger', fileArg]

        devnull = open(os.devnull, 'w')
        p = subprocess.Popen(command, stdout=devnull, stderr=devnull)
        return p.pid

    def killAllLoggingProcesses(self):
        for pid, command in self.existingLoggerProcesses.iteritems():
            processName, args = command
            os.system('kill %d' % pid)

logger = LCMLoggerManager()
print "Start recording..."
#logger.startNewLogger('', home_dir + "/logs/raw")
logger.startNewLogger()

print "Waiting 5 seconds..."
lc = lcm.LCM()
time.sleep(5)
msg = lcmmultisense.command_t()
msg.utime = timestamp_now()
msg.fps = 15
msg.gain = -1
msg.exposure_us = 10000
msg.agc = -1
msg.rpm = 5
msg.leds_flash = False
msg.leds_duty_cycle = 0
lc.publish("MULTISENSE_COMMAND", msg.encode())
print "Publishing Multisense command to spin at 5rpm!"

print "Recording for 60 seconds more..."
time.sleep(60)

logger.killAllLoggingProcesses()
print "Stop recording!"