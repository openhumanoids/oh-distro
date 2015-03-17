#!/bin/env python

#**********************************************************************
# $Id: track.py 71 2015-03-10 16:35:27Z karl $
#**********************************************************************

# http://stackoverflow.com/questions/2398661/schedule-a-repeating-event-in-python-3

import argparse
import datetime
import json
import os
import signal
import string
import sys
import syslog
import threading
import time
import traceback

import dce_ctl

#######################################################################
# Begin repeater
#######################################################################
#**********************************************************************
# RepeatedTimer
#
# Features:
#  - Standard library only, no external dependencies.
#  - Start() and stop() are safe to call multiple times even if the
#    timer has already started/stopped.
#  - Function to be called can have positional and named arguments.
#  - You can change interval anytime, it will be effective after
#    next run. Same for args, kwargs and even the function.
#**********************************************************************
class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.function   = function
        self.interval   = interval
        self.halfinterval   = interval/2.0  # Tiny optimization to take this
                                            # out of the RunNextRequest loop
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(self, *self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = threading.Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False
        

#**********************************************************************
# DoRequestBase
#**********************************************************************
class DoRequestBase(object):
    def __init__(self, reqname, dodelta):
        """
        @param reqname: The name of the request, a single line of text.
        @type reqname: String
        @param dodelta: A datetime.timedelta object indicating when this
                        request should be run.  It is expressed as seconds
                        after the run starts.
        @type dodelta: datetime.timedelta
        """
        self.__Name = reqname
        self.__Done = False
        # The following two variables specify when to run this request.
        # The DoAtDelta is a datetime.timedelta indicating how long after
        # the run is started that the job should be run.
        # The DoAtTime is the actual datetime.time when the job should be run.
        # We build the job list using the DoAtDelta fields.  Then when it
        # is time to start the run we will generate the DoAtTime fields
        # from the start time.  This allows us to do all the heavy lifting
        # of building the job list and sorting it before we know the actual
        # start time.
        self.__DoAtDelta = dodelta  # datetime.timedelta after triger time
        self.__DoAtTime = None   # datetime.time 

    def SetStartBaseline(self, baseline):
        self.__DoAtTime = baseline + self.__DoAtDelta

    def __lt__(self, other):
        return self.__DoAtDelta < other.__DoAtDelta
    
    def __le__(self, other):
        return self.__DoAtDelta <= other.__DoAtDelta

    def __eq__(self, other):
        return self.__DoAtDelta == other.__DoAtDelta

    def __ne__(self, other):
        return self.__DoAtDelta != other.__DoAtDelta

    def __gt__(self, other):
        return self.__DoAtDelta > other.__DoAtDelta

    def __ge__(self, other):
        return self.__DoAtDelta <= other.__DoAtDelta

    def __repr__(self):
        return repr({
                "Name": self.__Name,
                "DoDelta": self.__DoAtDelta,
                "DoAt": self.__DoAtTime
                })

    def __str__(self):
        return self.__repr__()

    @property
    def Name(self):
        return self.__Name

    @property
    def DoDelta(self):
        return self.__DoAtDelta

    @property
    def DoAt(self):
        return self.__DoAtTime

    @property
    def IsDone(self):
        return self.__Done

    def SetDone(self):
        self.__Done = True

    def IsReadyToGo(self, now):
        return (not self.__Done) and (self.__DoAtTime <= now)

    def Run(self, now):
        raise NotImplementedError

#**********************************************************************
# RunListBase
#**********************************************************************
class RunListBase(object):
    def __init__(self):
        self.__WorkList = []
        self.__StartTime = None
        self.__FinalizedSeq = False
        self.__FinalizedTime = False
        self.__AllDone = False
        self.__AllDoneEdge = False
        self.__HadError = False

    # ShowMsg can be elaborated in a derived class
    def ShowMsg(self, *args):
        pass

    @property
    def IsAllDone(self):
        return self.__AllDone

    @property
    # Return a done indication only once...
    def IsAllDoneEdge(self):
        if self.__AllDone and (not self.__AllDoneEdge):
            self.__AllDoneEdge = True
            return True
        return False

    @property
    def HadError(self):
        return self.__HadError

    def AddRequest(self, req):
        self.__WorkList.append(req)

    def FinalizeSequence(self):
        self.__WorkList.sort()
        self.__FinalizedSeq = True

    def FinalizeStartTime(self, starttime):
        assert(self.__FinalizedSeq)
        self.__StartTime = starttime
        for req in self.__WorkList:
            req.SetStartBaseline(starttime)
        self.__FinalizedTime = True

    def RunNextRequest(self, reptr_obj, nowtime):
        assert(self.__FinalizedSeq)
        assert(self.__FinalizedTime)

        # Do not proceed if we had an error
        if self.__HadError:
            return False

        # We are going to do a sequential scan of the work list to
        # find the first one that has not already been done and
        # that has reached its start time.
        # The list is ordered by increasing time, so we can
        # terminate the scan when we reach an item with a start
        # time in the future.
        # In order to accomodate the tick interval of our basic timer
        # we will run jobs as much as half a tick early.
        ttime = nowtime + datetime.timedelta(0, reptr_obj.halfinterval)
        for ndx in range(len(self.__WorkList)):
            req = self.__WorkList[ndx]
            if req.IsReadyToGo(ttime):
                del self.__WorkList[ndx]
                self.ShowMsg("Starting task \"%s\"" % req.Name)
                try:
                    req.Run(ttime)
                except Exception, err:
                    self.__HadError = True
                    self.ShowMsg("Exception in Task \"%s\" - %s" % (req.Name, str(err)))
                    #traceback.print_exc()
                    raise
                return True
            if req.DoAt > ttime:
                return False
        self.__AllDone = True
        return False

    @property
    def WorkList(self):
        return self.__WorkList

    @property
    def StartTime(self):
        return self.__StartTime

    @property
    def LastRequestStartTime(self):
        assert(self.__FinalizedSeq)
        assert(self.__FinalizedTime)
        if len(self.__WorkList) == 0:
               return datetime.datetime.now()
        return self.__WorkList[-1].DoAt

    def PrintMe(self):
        for d in self.__WorkList:
            self.ShowMsg("RunRequest", str(d))

#######################################################################
# End repeater
#######################################################################

#**********************************************************************
# OneTrackError
#**********************************************************************
class OneTrackError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class OneTrackStateEventError(OneTrackError):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

#**********************************************************************
# ShowMessage()
#**********************************************************************
UseSyslog = False # Set to True to send things to the syslog
TrackName = "TRACK UNKNOWN" # Should be set to the actual track name

def ShowMessage(*args):
    global TrackName
    global UseSyslog
    msg = datetime.datetime.now().isoformat() \
          + ' "' + TrackName + '" ' \
          + " ".join(map(str, args))
    print msg
    if UseSyslog:
        syslog.syslog(syslog.LOG_INFO | syslog.LOG_USER, msg)
    sys.stdout.flush()

#**********************************************************************
# TrackReqBase
#**********************************************************************
class TrackReqBase(DoRequestBase):
    def __init__(self, reqname, doafter, trackname, maxprohost):
        """
        @param reqname: The name of the request, a single line of text.
        @type reqname: String
        @param doafter: A positive integer indicating how many seconds
                         should pass before launching this job request.
        @type doafter: integer
        @param trackname: The name of the track.
        @type trackname: String
        @param maxprohost: The name or IP address of the target host.
        @type maxprohost: String
        """

        super(TrackReqBase, self).__init__(reqname, datetime.timedelta(0, doafter))
        self.__TrackName = trackname
        self.__MaxprohostName = maxprohost

    @property
    def TrackName(self):
        return self.__TrackName

    @property
    def HostName(self):
        return self.__MaxprohostName

    def __repr__(self):
        sd = self.__class__.__name__ + ": " + \
             super(TrackReqBase, self).__repr__()
        return sd + '\n' + repr({
                "Track": self.TrackName,
                "HostName": self.HostName
                })

    def __str__(self):
        return self.__repr__()

    def Run(self, now):
        raise NotImplementedError

#**********************************************************************
# TrackReqGoodPeriod
#**********************************************************************
class TrackReqGoodPeriod(TrackReqBase):
    def __init__(self, reqname, doafter, trackname, maxprohost):
        super(TrackReqGoodPeriod, self).__init__(reqname, doafter,
                                                 trackname, maxprohost)

    def Run(self, now):
        assert self.IsReadyToGo(now)
        StartNormalEvent()
        self.__Done = True

#**********************************************************************
# TrackReqInterruptedPeriod
#**********************************************************************
class TrackReqInterruptedPeriod(TrackReqBase):
    def __init__(self, reqname, doafter, trackname, maxprohost):
        super(TrackReqInterruptedPeriod, self).__init__(reqname, doafter,
                                                        trackname, maxprohost)
    def Run(self, now):
        assert self.IsReadyToGo(now)
        StartBlackoutEvent()
        self.__Done = True

#**********************************************************************
# RunList
#**********************************************************************
class RunList(RunListBase):
    def __init__(self):
        super(RunList, self).__init__()

    def ShowMsg(self, *args):
        ShowMessage(*args)

    def PrintMeF(self, pfunc):
        for d in self.WorkList:
            ShowMessage(pfunc(d))

#**********************************************************************
# SetupJobsFromCSV()
#**********************************************************************
def SetupJobsFromCSV(csvfile, jobs, trackname, mm_hostname):

    def CSVTimeToTimeDelta(csvtime):
        try:
            # Example: 0:45:15
            hourstr,minstr, secstr = csvtime.split(':')
            return datetime.timedelta(hours=int(hourstr),
                                      minutes=int(minstr),
                                      seconds=int(secstr))
        except:
            return datetime.timedelta(0)

    csvfd = open(csvfile, 'r')

    linenum = 1
    # Throw away the first line
    csvfd.readline()

    prevstart = datetime.timedelta.min
    expectstart = datetime.timedelta(0)

    try:
        while True:
            linenum += 1

            ln = csvfd.readline().strip()
            if not ln:
                break

            good_start_time_str, blackout_seconds_str, \
                blackout_start_time_str = ln.split(',')
            good_start_time = CSVTimeToTimeDelta(good_start_time_str)
            try:
                blackout_seconds = int(blackout_seconds_str)
            except:
                blackout_seconds = 0
            blackout_start_time = CSVTimeToTimeDelta(blackout_start_time_str)

            assert(good_start_time > prevstart)
            assert(good_start_time == expectstart)

            if (not blackout_seconds_str) or (not blackout_start_time_str):
                # Start a final good period
                totsecs = good_start_time.total_seconds()
                mins = totsecs/60
                secs = totsecs%60
                jobs.AddRequest(TrackReqGoodPeriod("Req good (final) - %u (%2.2u:%2.2u)" % \
                                                   (totsecs, mins, secs),
                                                    totsecs, trackname, mm_hostname))
                break

            assert(blackout_seconds > 0)
            assert(blackout_start_time > good_start_time)
            prevstart = blackout_start_time
            expectstart = blackout_start_time + \
                          datetime.timedelta(seconds=blackout_seconds)

            # Start a good period
            totsecs = good_start_time.total_seconds()
            mins = totsecs/60
            secs = totsecs%60
            jobs.AddRequest(TrackReqGoodPeriod("Req good - %u (%2.2u:%2.2u)" % \
                                               (totsecs, mins, secs),
                                               totsecs, trackname, mm_hostname))

            # Start an interruption
            totsecs = blackout_start_time.total_seconds()
            mins = totsecs/60
            secs = totsecs%60
            jobs.AddRequest(TrackReqInterruptedPeriod("Req interruption - %u (%2.2u:%2.2u)" % \
                                                      (totsecs, mins, secs),
                                                      totsecs, trackname, mm_hostname))
        return True

    except AssertionError, err:
        ShowMessage("Data consistency error occurred at or just prior to line %u in %s." % \
                    (linenum, csvfile))
        return False

    except Exception, err:
        ShowMessage("Error occurred at or just prior to line %u in %s." % \
                    (linenum, csvfile))
        return False

#**********************************************************************
# main()
#
# States:
#   - INITIAL_STATE
#   - OUTSIDE_NORMAL_STATE
#   - OUTSIDE_BLACKOUT_STATE
#   - INSIDE_NORMAL_STATE
#   - INSIDE_BLACKOUT_STATE
#
# Events:
#
#   - OUTSIDE_TO_INSIDE_EVENT
#   - INSIDE_TO_OUTSIDE_EVENT (should never occur)
#   - START_NORMAL_EVENT
#   - START_BLACKOUT_EVENT
#
# Initial state: OUTSIDE_NORMAL_STATE with normal conditions
#   Action to establish initial state: EstablishInitialConditions()
#
#   STATE                    EVENT                     NEXT_STATE               ACTION
# +========================+=========================+========================+=================================+
# | INITIAL_STATE          | START_NORMAL_EVENT      | OUTSIDE_NORMAL_STATE   | EstablishLink2OutsideOpen()     |
# +------------------------+-------------------------+------------------------+---------------------------------+
# | INITIAL_STATE          | START_BLACKOUT_EVENT    | N/A                    | *ERROR*                         |
# +------------------------+-------------------------+------------------------+---------------------------------+
# | INITIAL_STATE          | OUTSIDE_TO_INSIDE_EVENT | N/A                    | *ERROR*                         |
# +------------------------+-------------------------+------------------------+---------------------------------+
# | INITIAL_STATE          | INSIDE_TO_OUTSIDE_EVENT | N/A                    | *ERROR*                         |
# +========================+=========================+========================+=================================+
# | OUTSIDE_NORMAL_STATE   | START_NORMAL_EVENT      | N/A                    | *ERROR*                         |
# +------------------------+-------------------------+------------------------+---------------------------------+
# | OUTSIDE_NORMAL_STATE   | START_BLACKOUT_EVENT    | OUTSIDE_BLACKOUT_STATE | EstablishLink2OutsideBlackout() |
# +------------------------+-------------------------+------------------------+---------------------------------+
# | OUTSIDE_NORMAL_STATE   | OUTSIDE_TO_INSIDE_EVENT | INSIDE_NORMAL_STATE    | EstablishLink2OInsideOpen()     |
# +------------------------+-------------------------+------------------------+---------------------------------+
# | OUTSIDE_NORMAL_STATE   | INSIDE_TO_OUTSIDE_EVENT | N/A                    | *ERROR*                         |
# +========================+=========================+========================+=================================+
# | OUTSIDE_BLACKOUT_STATE | START_NORMAL_EVENT      | OUTSIDE_NORMAL_STATE   | EstablishLink2OutsideOpen()     |
# +------------------------+-------------------------+------------------------+---------------------------------+
# | OUTSIDE_BLACKOUT_STATE | START_BLACKOUT_EVENT    | N/A                    | *ERROR*                         |
# +------------------------+-------------------------+------------------------+---------------------------------+
# | OUTSIDE_BLACKOUT_STATE | OUTSIDE_TO_INSIDE_EVENT | INSIDE_BLACKOUT_STATE  | EstablishLink2OInsideBlackout() |
# +------------------------+-------------------------+------------------------+---------------------------------+
# | OUTSIDE_BLACKOUT_STATE | INSIDE_TO_OUTSIDE_EVENT | N/A                    | *ERROR*                         |
# +========================+=========================+========================+=================================+
# | INSIDE_NORMAL_STATE    | START_NORMAL_EVENT      | N/A                    | *ERROR*                         |
# +------------------------+-------------------------+------------------------+---------------------------------+
# | INSIDE_NORMAL_STATE    | START_BLACKOUT_EVENT    | INSIDE_BLACKOUT_STATE  | EstablishLink2OInsideBlackout() |
# +------------------------+-------------------------+------------------------+---------------------------------+
# | INSIDE_NORMAL_STATE    | OUTSIDE_TO_INSIDE_EVENT | N/A                    | *ERROR*                         |
# +------------------------+-------------------------+------------------------+---------------------------------+
# | INSIDE_NORMAL_STATE    | INSIDE_TO_OUTSIDE_EVENT | OUTSIDE_NORMAL_STATE   | EstablishLink2OutsideOpen()     |
# +========================+=========================+========================+=================================+
# | INSIDE_BLACKOUT_STATE  | START_NORMAL_EVENT      | INSIDE_NORMAL_STATE    | EstablishLink2OInsideOpen()     |
# +------------------------+-------------------------+------------------------+---------------------------------+
# | INSIDE_BLACKOUT_STATE  | START_BLACKOUT_EVENT    | N/A                    | *ERROR*                         |
# +------------------------+-------------------------+------------------------+---------------------------------+
# | INSIDE_BLACKOUT_STATE  | OUTSIDE_TO_INSIDE_EVENT | N/A                    | *ERROR*                         |
# +------------------------+-------------------------+------------------------+---------------------------------+
# | INSIDE_BLACKOUT_STATE  | INSIDE_TO_OUTSIDE_EVENT | OUTSIDE_BLACKOUT_STATE | EstablishLink2OutsideBlackout() |
# +========================+=========================+========================+=================================+
#
#**********************************************************************

INITIAL_STATE = 0
OUTSIDE_NORMAL_STATE = 1
OUTSIDE_BLACKOUT_STATE = 2
INSIDE_NORMAL_STATE = 3
INSIDE_BLACKOUT_STATE = 4

ValidStates = (INITIAL_STATE, OUTSIDE_NORMAL_STATE, OUTSIDE_BLACKOUT_STATE,
               INSIDE_NORMAL_STATE, INSIDE_BLACKOUT_STATE)
    
StateNames = {INITIAL_STATE: "INITIAL_STATE",
              OUTSIDE_NORMAL_STATE: "OUTSIDE_NORMAL_STATE",
              OUTSIDE_BLACKOUT_STATE: "OUTSIDE_BLACKOUT_STATE",
              INSIDE_NORMAL_STATE: "INSIDE_NORMAL_STATE",
              INSIDE_BLACKOUT_STATE: "INSIDE_BLACKOUT_STATE"}

CurrentState = None
CurrentStateStartTime = None

# Indicate how many cycles of jobtick() between checks of
# the inside/outside switch
JtickPeriod = 1
JtickCounter = 0

InsideFlag = False
InsideMarkerFile = None

DCECtrl = None

#############################################################
# Begin routines to actually set up the impairment regime
#############################################################
#******************************
# EstablishInitialConditions
#******************************
def EstablishInitialConditions():
    global DCECtrl

    assert DCECtrl is not None

    ShowMessage("Establishing initial conditions")
    DCECtrl.ClearAllFlows()
    DCECtrl.InitFastPath()	# Flow 0
    DCECtrl.InitSlowPath()	# Flow 1
    DCECtrl.InitIcmpBypass()	# Flow 2
    DCECtrl.InitArpBypass()	# Flow 3
    DCECtrl.InitBitBucket()	# Flow 4
    ShowMessage("Initial conditions established")

#******************************
# EstablishLink2OutsideOpen
#******************************
def EstablishLink2OutsideOpen():
    global DCECtrl

    assert DCECtrl is not None

    ShowMessage("Imposing OUTSIDE OPEN conditions on link 2")
    DCECtrl.EstablishFastPathOutsideOpen()

#******************************
# EstablishLink2OutsideBlackout
#******************************
def EstablishLink2OutsideBlackout():
    global DCECtrl

    assert DCECtrl is not None

    ShowMessage("Imposing OUTSIDE BLACKOUT conditions on link 2")
    DCECtrl.EstablishFastPathOutsideBlackout()

#******************************
# EstablishLink2InsideOpen
#******************************
def EstablishLink2OInsideOpen():
    global DCECtrl

    assert DCECtrl is not None

    ShowMessage("Imposing INSIDE OPEN conditions on link 2")
    DCECtrl.EstablishFastPathInsideOpen()

#******************************
# EstablishLink2InsideBlackout
#******************************
def EstablishLink2OInsideBlackout():
    global DCECtrl

    assert DCECtrl is not None

    ShowMessage("Imposing INSIDE BLACKOUT conditions on link 2")
    DCECtrl.EstablishFastPathInsideBlackout()

#############################################################
# End routines to actually set up the impairment regime
#############################################################

#############################################################
# Begin the state machine event handlers
#############################################################
def ChangeStateHelper(newstate):
    global CurrentState
    global CurrentStateStartTime

    CurrentState = newstate
    ShowMessage("Changing state to %s" % StateNames[newstate])

    now = datetime.datetime.now()
    if CurrentStateStartTime:
        tdelta = now - CurrentStateStartTime
        # Round the time to the nearest full second
        ranfor = ((tdelta.seconds*1000000) + tdelta.microseconds + 500000)/1000000
        ShowMessage("Prior state ran for %2.2u:%2.2u" % (ranfor/60, ranfor%60))
    CurrentStateStartTime = now
    StatusBeacon.Emit({})

#******************************
# StartNormalEvent()
#******************************
def StartNormalEvent():
    global CurrentState
    assert CurrentState in ValidStates

    ShowMessage("Received StartNormalEvent")

    if CurrentState == INITIAL_STATE:
        ChangeStateHelper(OUTSIDE_NORMAL_STATE)
        EstablishLink2OutsideOpen()

    elif CurrentState == OUTSIDE_BLACKOUT_STATE:
        ChangeStateHelper(OUTSIDE_NORMAL_STATE)
        EstablishLink2OutsideOpen()

    elif CurrentState == INSIDE_BLACKOUT_STATE:
        ChangeStateHelper(INSIDE_NORMAL_STATE)
        EstablishLink2OInsideOpen()

    else: # Anything not above
        raise OneTrackStateEventError, \
            "Unexpected Start Normal event in state %u" % CurrentState

#******************************
# StartBlackoutEvent()
#******************************
def StartBlackoutEvent():
    global CurrentState
    assert CurrentState in ValidStates

    ShowMessage("Received StartBlackoutlEvent")

    if CurrentState == OUTSIDE_NORMAL_STATE:
        ChangeStateHelper(OUTSIDE_BLACKOUT_STATE)
        EstablishLink2OutsideBlackout()

    elif CurrentState == INSIDE_NORMAL_STATE:
        ChangeStateHelper(INSIDE_BLACKOUT_STATE)
        EstablishLink2OInsideBlackout()

    else: # Anything not above
        raise OneTrackStateEventError, \
            "Unexpected Start Blackout event in state %u" % CurrentState

#******************************
# OutsideToInsideEvent()
#******************************
def OutsideToInsideEvent():
    global CurrentState
    assert CurrentState in ValidStates

    ShowMessage("Received OutsideToInsideEvent")

    if CurrentState == OUTSIDE_NORMAL_STATE:
        ChangeStateHelper(INSIDE_NORMAL_STATE)
        EstablishLink2OInsideOpen()

    elif CurrentState == OUTSIDE_BLACKOUT_STATE:
        ChangeStateHelper(INSIDE_BLACKOUT_STATE)
        EstablishLink2OInsideBlackout()

    else: # Anything not above
        raise OneTrackStateEventError, \
            "Unexpected Outside-to-inside event in state %u" % CurrentState

#******************************
# InsideToOutsideEvent()
#******************************
def InsideToOutsideEvent():
    global CurrentState
    assert CurrentState in ValidStates

    ShowMessage("Received InsideToOutside Event")

    if CurrentState == INSIDE_NORMAL_STATE:
        ChangeStateHelper(OUTSIDE_NORMAL_STATE)
        EstablishLink2OutsideOpen()

    elif CurrentState == INSIDE_BLACKOUT_STATE:
        ChangeStateHelper(OUTSIDE_BLACKOUT_STATE)
        EstablishLink2OutsideBlackout()

    else: # Anything not above
        raise OneTrackStateEventError, \
            "Unexpected Inside-to-outside event in state %u" % CurrentState

#############################################################
# End the state machine event handlers
#############################################################

#**********************************************************************
# AddCRLF()
# Add a CRLF (\r\n) pair to a string if it doesn't already end that way.
#**********************************************************************
def AddCRLF(jsnstr):
    if (len(jsnstr) > 1) and (jsnstr[-2:] == '\r\n'):
        return jsnstr
    else:
        return jsnstr + '\r\n'

#**********************************************************************
# StatusBeacon - Send JSON encoded status information into a named pipe
# Every JSON message will end with a CRLF
#**********************************************************************
StatusBeacon = None

PGM_INITIALIZING = 0
PGM_WAITING_TO_START = 1
PGM_RUNNING = 2
PGM_DONE = 3

ValidPgmStatus = (PGM_INITIALIZING, PGM_WAITING_TO_START, PGM_RUNNING, PGM_DONE)

PgmStatusName = {PGM_INITIALIZING: "Initializing",
                 PGM_WAITING_TO_START: "Waiting to start",
                 PGM_RUNNING: "Running",
                 PGM_DONE: "Finished"}

PGMStatus = PGM_INITIALIZING

PGMMsg = ""

class StatusBeaconObj(object):
    def __init__(self, pipefilename):
        self.__PipeFileName = pipefilename
        self.__PipeFD = None
        self.OpenPipe()
   
    def OpenPipe(self):
        if self.__PipeFD is None:
            if self.__PipeFileName:
                try:
                    self.__PipeFD = os.open(self.__PipeFileName, os.O_WRONLY | os.O_NONBLOCK)
                except Exception, err:
                    self.__PipeFD = None

    def ClosePipe(self):
        if self.__PipeFD is not None:
            try:
                os.close(self.__PipeFD)
            except:
                pass
            self.__PipeFD = None

    def Emit(self, jdict):
        assert PGMStatus in ValidPgmStatus

        try:
            estatus = CurrentState
            estatus_txt = StateNames[CurrentState]
        except:
            estatus = -1
            estatus_txt = "Unknown"

        self.OpenPipe()
        if self.__PipeFD is not None:
            try:
                outdict = {"TIME": datetime.datetime.now().isoformat(),
                           "PROGRAM_STATUS": PGMStatus,
                           "PROGRAM_STATUS_TEXT": PgmStatusName[PGMStatus],
                           "EMULATION_STATUS": estatus,
                           "EMULATION_STATUS_TEXT": estatus_txt,
                           "INSIDE_FLAG": InsideFlag,
                           "PGM_MSG": PGMMsg,
                           "PARAMS": jdict}
                os.write(self.__PipeFD,
                         AddCRLF(json.dumps(outdict, ensure_ascii=True)))
#--#                         AddCRLF(yaml.dump(outdict, default_flow_style=True, default_style='"')))
            except Exception, err:
                self.ClosePipe()

#**********************************************************************
# main()
#**********************************************************************
def main():

    global TrackName
    global UseSyslog
    global JtickPeriod
    global JtickCounter
    global InsideMarkerFile
    global StatusBeacon
    global PGMStatus
    global DCECtrl
    global PGMMsg

    #******************************
    # jobtick() - Where we run the job
    #******************************
    def jobtick(reptr_obj):
        global JtickPeriod
        global JtickCounter
        global PGMMsg
        global StatusBeacon
        
        # Lets run the job, if any, that is due at this time
        # We will do the inside/outside test afterwords because it
        # has less rigorous time requirements.
        try:
            Jobs.RunNextRequest(reptr_obj, datetime.datetime.now())
        except Exception, err:
            PGMMsg = str(err)
            StatusBeacon.Emit({})
            ShowMessage(str(err))

        if Jobs.IsAllDoneEdge:
            PGMMsg = "Scheduler complete"
            StatusBeacon.Emit({})

        JtickCounter += 1
        if JtickCounter >= JtickPeriod:
            JtickCounter = 0
            CheckInsideStatus()

    #******************************
    # CheckInsideStatus
    #******************************
    def CheckInsideStatus():
        global InsideFlag
        global InsideMarkerFile
        newstatus = os.access(InsideMarkerFile, os.F_OK)
        if newstatus != InsideFlag:
            if newstatus:
                InsideFlag = True
                OutsideToInsideEvent()
            else:
                InsideFlag = False
                InsideToOutsideEvent()

    #******************************
    # Deal with the arguments
    #******************************
    #**********************************************************************
    # From http://stackoverflow.com/questions/3853722/python-argparse-how-to-insert-newline-the-help-text
    #**********************************************************************
    class SmartFormatter(argparse.HelpFormatter):

        def _split_lines(self, text, width):
            # this is the RawTextHelpFormatter._split_lines
            if text.startswith('R|'):
                return text[2:].splitlines()  
            return argparse.HelpFormatter._split_lines(self, text, width)

    parser = argparse.ArgumentParser(description="Start track impairment schedule.",
                                   usage="%(prog)s [options]  Use -h for help.",
                                   formatter_class=SmartFormatter)

    parser.add_argument("-I", "--interval", required=False, metavar="<timer interval>",
                        type=float, default=0.01,
                        help="R|Timer interval [default: %(default)s]")

    parser.add_argument('-f', '--inside_filename',
                        dest='insidefile',
                        required=True,
                        help="R|Inside/Outside marker file name")

    parser.add_argument('-c', '--csvfile',
                        dest='csvfile',
                        required=False,
                        default="schedule.csv",
                        help="R|CSV schedule file file name [default: %(default)s]")

    parser.add_argument("-l", "--linger", required=False, metavar="<timer interval>",
                        type=int, default=3600,
                        help="R|Seconcs to linger after complete [default: %(default)s]")

    parser.add_argument("--syslog", action="store_true",
                        help="R|Send reports to the system defined syslog server.\n"
                             "Level LOG_INFO, Facility LOG_USER).")

    parser.add_argument("-B", "--beaconpipe", metavar="<Status beacon named pipe name>",
                        help="R|Name of a named pipe into which to emit status reports.")

    parser.add_argument("-r", "--slowrate", required=False, metavar="<slow link bit rate>",
                        type=int, default=9600,
                        help="R|Slow link bit rate [default: %(default)s]")

    parser.add_argument("-i", "--icmprate", required=False, metavar="<icmp link bit rate>",
                        type=int, default=4800,
                        help="R|ICMP link bit rate [default: %(default)s]")

    parser.add_argument("-t", "--track", required=True, metavar="<track name>",
                        help="R|Track name.")

    parser.add_argument("maxpro_hostname",  metavar="<Maxwell Pro host name or IP address>",
			help='R|The host name or IP address of the Maxwell Pro.')

    parser.add_argument("--start_time", metavar="<start time>",
                        help="R|Start job sequence at given time.\n"
                             "(format: hh:mm:ss GMT)")

    parser.add_argument("--start_file_appears", metavar="<start file>",
                        help="R|Start job sequence when given file appears.\n"
                             "(Pre-existing file will be removed.)")

    parser.add_argument("--start_file_exists", metavar="<start file>",
                        help="R|Start job sequence when given file exists.\n"
                             "(Pre-existing file will be used.)")

    parser.add_argument("--start_on_signal", action="store_true",
                        help="R|Start job sequence on SIGUSR1.")

    pargs = parser.parse_args()

    UseSyslog = pargs.syslog

    # It seems that parser adds an item in the pargs dictionary for every parameter
    # whether it is present or not.

    start_on_signal = pargs.start_on_signal
    beaconpipename = os.path.abspath(pargs.beaconpipe) \
                     if pargs.beaconpipe else None
    start_file_appears = os.path.abspath(pargs.start_file_appears) \
                         if pargs.start_file_appears else None
    start_file_exists = os.path.abspath(pargs.start_file_exists) \
                        if pargs.start_file_exists else None
    start_time = datetime.datetime.strptime(pargs.start_time, "%H:%M:%S").time() \
                 if pargs.start_time else None
    InsideMarkerFile = os.path.abspath(pargs.insidefile)

    if reduce(lambda x,y: x+(1 if y else 0), \
                         (start_on_signal,
                          start_file_exists, start_file_appears,
                          start_time), 0) > 1:
        ShowMessage("Error: Only one of start_* option may be specified. - Aborting")
        sys.exit(1)

    TrackName = pargs.track

    StatusBeacon = StatusBeaconObj(beaconpipename)

    # Check the inside/outside switch at least once every 400milliseconds...
    repeater_interval = pargs.interval
    if repeater_interval >= 0.4:
        JtickPeriod = 1
    else:
        JtickPeriod = int(0.4/repeater_interval)

    maxpro_hostname = pargs.maxpro_hostname
    csvfile = os.path.abspath(pargs.csvfile)
    linger = pargs.linger

    ShowMessage("Repeater Interval:", repeater_interval)
    ShowMessage("Seconds to linger when finished:", linger)
    ShowMessage("Track:", TrackName)
    ShowMessage("Slow link bit rate:", pargs.slowrate)
    ShowMessage("ICMP link bit rate:", pargs.icmprate)
    ShowMessage("Status pipe name:", beaconpipename)
    ShowMessage("Use Syslog:", UseSyslog)
    ShowMessage("MPHost:", maxpro_hostname)
    ShowMessage("CSV File:", csvfile)
    ShowMessage("Inside Marker File:", InsideMarkerFile)
    if start_on_signal:
        ShowMessage("Start: SIGUSR1")
    if start_file_appears:
        ShowMessage("Start: When file \"%s\" appears" % start_file_appears)
    elif start_file_exists:
        ShowMessage("Start: When file \"%s\" exists" % start_file_exists)
    elif start_time:
        ShowMessage("Start: At time %s" % start_time.isoformat())
    else:
        ShowMessage("Start: immediately")

    ShowMessage("Scheduler starting. CSV file:", csvfile,
                "DCE:", maxpro_hostname)

    if not os.access(csvfile, os.R_OK):
        ShowMessage("Error: Can not read CSV scheduling file: \"%s\" - Aborting" % csvfile)
        sys.exit(1)

    Jobs = RunList()

    if not SetupJobsFromCSV(csvfile, Jobs, TrackName, maxpro_hostname):
        ShowMessage("Error when reading CSV scheduling file: \"%s\" - Aborting" % csvfile)
        sys.exit(1)

    try:
        ShowMessage("OPENING DCE")
        DCECtrl = dce_ctl.DCEControl(maxpro_hostname,
                                     slow_rate=pargs.slowrate,
                                     icmp_rate=pargs.icmprate)
        ShowMessage("CONNECTING DCE")
        DCECtrl.Connect()
        ShowMessage("CONNECTED")
        DCECtrl.ClearAllFlows()
        ShowMessage("CLEARED")
    except Exception, err:
        ShowMessage("Can not connect to DCE:", str(err))
        DCECtrl = None
        PGMStatus = PGM_DONE
        PGMMsg = "DCE error: "+ str(err)
        StatusBeacon.Emit({})
        StatusBeacon.ClosePipe()
        ShowMessage("Stopping - Could not connect to DCE - Finished")
        sys.exit(1)

    EstablishInitialConditions()
    ChangeStateHelper(INITIAL_STATE)

    Jobs.FinalizeSequence()

    PGMStatus = PGM_WAITING_TO_START
    PGMMsg = "Waiting to Start"
    StatusBeacon.Emit({})
    # Wait for the start condition, if any...
    if start_on_signal:
        startevent = threading.Event()

        def usr1handler(signum, frame):
            if signum == signal.SIGUSR1:
                startevent.set()

        oldhandler = signal.signal(signal.SIGUSR1, usr1handler)
        ShowMessage("Waiting for signal SIGUSR1 to this process (%u)" % os.getpid())
        while not startevent.wait(repeater_interval):
            pass
        # We got the signal
        signal.signal(signal.SIGUSR1, oldhandler)

    elif start_file_appears:
        try:
            os.remove(start_file_appears)
        except:
            pass
        ShowMessage("Waiting for file \"%s\" to appear." % start_file_appears)
        while True:
            if os.access(start_file_appears, os.F_OK):
                break
            time.sleep(repeater_interval)

    elif start_file_exists:
        ShowMessage("Waiting for file \"%s\" to exit." % start_file_appears)
        while True:
            if os.access(start_file_exists, os.F_OK):
                break
            time.sleep(repeater_interval)
    elif start_time:
        today = datetime.datetime.utcnow()
        start_datetime = datetime.datetime.combine(today,
                                                   start_time)
        waitdelta = start_datetime - today
        sleepsecs = waitdelta.total_seconds()
        if (sleepsecs < 0):
            ShowMessage("Error: Target start time is in the past")
            sys.exit(1)
        ShowMessage("Sleeping for %f seconds" % sleepsecs)
        time.sleep(sleepsecs)
        
    PGMStatus = PGM_RUNNING
    ShowMessage("Beginning scheduled events")
    PGMMsg = "Scheduler started"
    StatusBeacon.Emit({})

    TimeBase = datetime.datetime.now()
    ShowMessage("TBase:", TimeBase)

    Jobs.FinalizeStartTime(TimeBase)

    if not True:
        def pfunc(request):
            dw = request.DoAt - TimeBase
            minutes = dw.seconds/60
            seconds = dw.seconds%60
            return "%s: %2.2u:%2.2u" % \
                (request.__class__.__name__, minutes, seconds)

        Jobs.PrintMe()
        Jobs.PrintMeF(pfunc)
        try:
            DCECtrl.Disconnect()
        except:
            pass
        DCECtrl = None
        PGMStatus = PGM_DONE
        StatusBeacon.Emit({})
        StatusBeacon.ClosePipe()
        ShowMessage("Stopping - Finished")
        sys.exit(0)

    # Give ourselves a lifetime.
    # Add a few seconds to let the last job complete.
    sleepsecs = (Jobs.LastRequestStartTime - TimeBase).total_seconds() + linger

#    sys.exit(0) # **TEMPORARY**

    rt = None

    def termhandler(signum, frame):
        if signum == signal.SIGTERM:
            if rt is not None:
                rt.stop()

    oldhandler = signal.signal(signal.SIGTERM, termhandler)

    # Begin the timer system with a one second interval
    rt = RepeatedTimer(repeater_interval, jobtick)

    try:
        ShowMessage("Start running for %u seconds." % sleepsecs)
        time.sleep(sleepsecs)
    finally:
        rt.stop() # better in a try/finally block to make sure the program ends!

    signal.signal(signal.SIGTERM, oldhandler)
    try:
        DCECtrl.Disconnect()
    except:
        pass
    DCECtrl = None
    PGMStatus = PGM_DONE
    StatusBeacon.Emit({})
    StatusBeacon.ClosePipe()
    ShowMessage("Stopping - Finished")

#**********************************************************************
# Entry
#**********************************************************************
if __name__ == "__main__":

    try:
        main()
    except Exception, err:
        ShowMessage("Unhandled exception:", str(err))
        traceback.print_exc()

    except KeyboardInterrupt:
        ShowMessage("Keyboard Interrupt")
