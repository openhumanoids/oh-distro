#!/usr/bin/env python

#**********************************************************************
# $Id: drcctrl.py 70 2015-03-10 16:24:12Z karl $
#**********************************************************************

# http://wxpython.org/Phoenix/docs/html/main.html

import argparse
import datetime
import errno
import json
import os
import select
import signal
import string
import subprocess
import sys
import syslog
import tempfile
import threading
import time
import types
import urllib2
import traceback
import yaml

import wx
#import wx.lib.dialogs
#import wx.lib.pubsub

#import buttons

#**********************************************************************
# Constants -- *MUST* match ose in onetrack.py
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

PGM_INITIALIZING = 0
PGM_WAITING_TO_START = 1
PGM_RUNNING = 2
PGM_DONE = 3

ValidPgmStatus = (PGM_INITIALIZING, PGM_WAITING_TO_START, PGM_RUNNING, PGM_DONE)

PgmStatusName = {PGM_INITIALIZING: "Initializing",
                 PGM_WAITING_TO_START: "Waiting to start",
                 PGM_RUNNING: "Running",
                 PGM_DONE: "Finished"}

#**********************************************************************
# DRCCtrlError
#**********************************************************************
class DRCCtrlError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

#**********************************************************************
# GetSched()
#**********************************************************************
def GetSched(yamlfileURI):
    slots = []

    ufd = urllib2.urlopen(yamlfileURI, timeout=2)
    ydoc = yaml.safe_load(ufd.read())

    # Make sure we got what we want...
    try:
        sd = ydoc["schedule_date"]
        assert isinstance(sd, datetime.datetime)
        sch_slots = ydoc["slots"]
        assert isinstance(sch_slots, (types.ListType, types.TupleType))
    except:
        raise DRCCtrlError, "Badly formatted or missing schedule file"

    for slt in sch_slots:
        if isinstance(slt, datetime.datetime):
            slots.append(slt)
        elif isinstance(slt, types.StringType):
            st = datetime.datetime.strptime(slt, "%H:%M:%S")
            slots.append(datetime.datetime.combine(datetime.date.today(),
                                                   st.time()))
        elif isinstance(slt, types.IntType):
            hr = slt/3600
            rem = slt - (hr * 3600)
            mn = rem/60
            sec = rem % 60
            st = datetime.time(hr, mn, sec)
            slots.append(datetime.datetime.combine(datetime.date.today(),
                                                   st))
        else:
            raise DRCCtrlError, "Can not decipher date/time"

    slots.sort()
    return slots

#**********************************************************************
# GetNextSlot()
#**********************************************************************
def GetNextSlot(nowdatetime, yamlfileURI):
    assert isinstance(nowdatetime, datetime.datetime)
    slots = GetSched(yamlfileURI)
    rslt = None
    for slt in slots:
        assert isinstance(slt, datetime.datetime)
        if slt >= nowdatetime:
            return slt
    return None

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
# AddCRLF()
# Add a CRLF (\r\n) pair to a string if it doesn't already end that way.
#**********************************************************************
def AddCRLF(jsnstr):
    if (len(jsnstr) > 1) and (jsnstr[-2:] == '\r\n'):
        return jsnstr
    else:
        return jsnstr + '\r\n'

#**********************************************************************
# StatusBar
#**********************************************************************
class StatusBar(wx.StatusBar):
    def __init__(self, parent):
        wx.StatusBar.__init__(self, parent, -1)

        self.SetFieldsCount(3)
        # Sets the three fields to be relative widths to each other.
        self.SetStatusWidths([-3, -1, -2])
        self.sizeChanged = False
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_IDLE, self.OnIdle)

        self.ColorA = wx.ColourDatabase.Find(wx.ColourDatabase(), "RED")
        self.ColorB = wx.ColourDatabase.Find(wx.ColourDatabase(), "GREEN")
        self.ColorC = wx.ColourDatabase.Find(wx.ColourDatabase(), "BLACK")
        self.ColorD = wx.ColourDatabase.Find(wx.ColourDatabase(), "WHITE")
        self.ColorE = wx.ColourDatabase.Find(wx.ColourDatabase(), "YELLOW")

        # Field 0 ... just text
        self.SetStatusText("", 0)

        # Field 1 (the second field) - not used

        # Field 2 (the third field) - a clock
        #self.ClockTextCtrl = wx.TextCtrl(self,-1, "Disconnected", style = wx.TE_READONLY)
        self.ClockTextCtrl = wx.TextCtrl(self,-1, "", style = wx.TE_READONLY)
        self.SetClockTextCtrl("000:00", self.ColorC, self.ColorE)
        self.Reposition()

        # We're going to use a timer to drive a 'clock' in the last
        # field.
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnTimeTick, self.timer)

        self.StartTime = None

        self.timer.Start(1000)
        self.OnTimeTick(None)

    def SnapStartTime(self):
        self.StartTime = datetime.datetime.now()

    #**************************************************
    # Handles events from the timer we started in __init__().
    # We're using it to drive a 'clock' in field 2 (the third field).
    def OnTimeTick(self, event):
        if self.StartTime is None:
            return
        tdelta = datetime.datetime.now() - self.StartTime
        ts = tdelta.total_seconds()
        st = "%3.3u:%2.2u" % (ts/60, ts%60)
        #self.SetStatusText(st, 1)
        self.SetClockTextCtrl(st, self.ColorC, self.ColorB)
        if event:
            event.Skip()

    #**************************************************
    def OnSize(self, event):
        self.Reposition()  # for normal size events
        # Set a flag so the idle time handler will also do the repositioning.
        # It is done this way to get around a buglet where GetFieldRect is not
        # accurate during the EVT_SIZE resulting from a frame maximize.
        self.sizeChanged = True

    #**************************************************
    def OnIdle(self, event):
        if self.sizeChanged:
            self.Reposition()
        event.Skip()

    #**************************************************
    def Reposition(self):
        rect = self.GetFieldRect(2)
        self.ClockTextCtrl.SetPosition((rect.x + 1, rect.y + 1))
        self.ClockTextCtrl.SetSize((rect.width - 2, rect.height - 2))
        self.sizeChanged = False

    #**************************************************
    def SetClockTextCtrl(self, val, forecolor, backcolor):
        self.ClockTextCtrl.SetValue(val)
        self.ClockTextCtrl.SetForegroundColour(forecolor)
        self.ClockTextCtrl.SetBackgroundColour(backcolor)

    #**************************************************
    def SetStatusMsg(self, msg):
        self.SetStatusText(msg, 0)

    #**************************************************
    def ClearStatusMsg(self):
        self.SetStatusMsg("")

#==============================================================================
class DRCCFrame(wx.Frame):
    def __init__(self, app, trackname, position, trackcolor, autostart_uri,
                 schedpgm, fifofilename, insidefilename,
                 csvfile, maxpro_hostname, slowrate, icmprate):
        self.App = app
        self.TrackName = trackname
        self.OurPosition = position
        self.TrackColor = trackcolor
        self.AutostartURI = autostart_uri
        self.AutostartTime = None
        self.SchedPgm = schedpgm
        self.FifoFileName = fifofilename
        self.InsideIndicatorFileName = insidefilename
        self.CsvFile = csvfile
        self.MaxProHostname = maxpro_hostname
        self.SlowRate = slowrate
        self.ICMPRate = icmprate
#--#        self.errMsgInfo = None
        self.SchedProcess = None
        wx.Frame.__init__(self, None, -1,
                          "DRC Control - Track %s" % self.TrackName,
                          style = wx.NO_FULL_REPAINT_ON_RESIZE |
                                  wx.DEFAULT_FRAME_STYLE)

        self.SetBackgroundColour(self.TrackColor)

        # The 4D systems 4.3 cape is 480x272, but we have to leave room for
        # the various window manager and window decorators.
        self.DimensionX = 474
        self.DimensionY = 220

        self.SetMinSize((self.DimensionX, self.DimensionY))
        self.SetMaxSize((self.DimensionX, self.DimensionY))
        self.SetSize((self.DimensionX, self.DimensionY))

        if self.OurPosition is not None:
            if self.OurPosition.lower() == "center":
                self.CenterOnScreen(wx.BOTH)
            else:
                self.SetPosition(map(int, self.OurPosition.split('x')))

        self.statusBar = StatusBar(self)
        self.SetStatusBar(self.statusBar)
        self.Prepared = False
        self.Running = False
        self.Inside = False
        self.Stopped = False
        self.SchedPgmStatus = 0
        self.SchedPgmStatusTxt = ""
        self.SchedEmlStatus = 0
        self.SchedEmlStatusText = ""
        self.SchedInsideStatus = False
        self.SchedPgmMsg = ""

        #------------------------------------------------------------
        # Build up the menus
        #------------------------------------------------------------
        self.menuData = \
            [("&File",
              [
                ["E&xit", "Terminate the GUI application.", self.OnMenuExit]
              ]
             )
            ]

        helpMenu = \
             ("&Help", [])

        helpMenu[1].append(
            ["&About", "General information about this application.",
             self.OnMenuAbout])

        self.menuData.append(helpMenu)
#        self.CreateMenuBar(self.menuData)

        #------------------------------------------------------------
        # Set up a close handler
        #------------------------------------------------------------
        wx.EVT_CLOSE(self, self.OnCloseFrame)

#--#        self.statusBar.SetStatusText("", 0)
        self.statusBar.SetStatusMsg("")

        self.BlueBall = wx.Bitmap("blueball-16.png", wx.BITMAP_TYPE_ANY)
        self.RedBall = wx.Bitmap("redball-16.png", wx.BITMAP_TYPE_ANY)
        self.GreenBall = wx.Bitmap("greenball-16.png", wx.BITMAP_TYPE_ANY)
        self.YellowBall = wx.Bitmap("yellowball-16.png", wx.BITMAP_TYPE_ANY)
        self.BlackBall = wx.Bitmap("blackball-16.png", wx.BITMAP_TYPE_ANY)

        self.BtnDisableFgColor = wx.ColourDatabase.Find(wx.ColourDatabase(), "WHITE")
        self.BtnDisableBgColor = wx.ColourDatabase.Find(wx.ColourDatabase(), "GREY")

        self.BtnEnableFgColor = wx.ColourDatabase.Find(wx.ColourDatabase(), "BLACK")
        self.BtnEnableBgColor = wx.ColourDatabase.Find(wx.ColourDatabase(), "GREEN")
        self.BtnAutoBgColor = wx.ColourDatabase.Find(wx.ColourDatabase(), "YELLOW")

        prep_x = 10
        start_x = prep_x + 108
        inside_x = start_x + 108
        stop_x = inside_x + 144

        btn_y=14
        btn_sz_x=98
        btn_sz_y=54

        self.PrepBtn = wx.ToggleButton(self, label='PREPARE', pos=(prep_x, btn_y),
                                       size=(btn_sz_x, btn_sz_y))
        self.EnableButton(self.PrepBtn)
        self.PrepBtn.SetToolTip(wx.ToolTip("Prepare system"))
        #self.PrepBtn.SetBitmap(None)

        self.StartBtn = wx.ToggleButton(self, label='START', pos=(start_x, btn_y),
                                        size=(btn_sz_x, btn_sz_y))
        self.StartBtn.SetToolTip(wx.ToolTip("Start robot run"))
        #self.StartBtn.SetBitmap(None)
        self.DisableButton(self.StartBtn)

        self.InsideBtn = wx.ToggleButton(self, label='IN/OUT', pos=(inside_x, btn_y),
                                         size=(btn_sz_x, btn_sz_y))
        self.InsideBtn.SetToolTip(wx.ToolTip("Robot is OUTSIDE"))
        #self.InsideBtn.SetBitmap(None)
        self.DisableButton(self.InsideBtn)

        self.StopBtn = wx.ToggleButton(self, label='STOP', pos=(stop_x, btn_y),
                                       size=(btn_sz_x, btn_sz_y))
        self.StopBtn.SetToolTip(wx.ToolTip("Stop system"))
        #self.StopBtn.SetBitmap(None)
        self.DisableButton(self.StopBtn)

        self.RestartBtn = wx.ToggleButton(self, label='RESTART', pos=(stop_x, btn_y+110),
                                          size=(btn_sz_x, btn_sz_y))
        self.RestartBtn.SetToolTip(wx.ToolTip("Restart system"))
        #self.RestartBtn.SetBitmap(None)
        self.DisableButton(self.RestartBtn)

        bias = 40
        ball = self.BlackBall
        ball_y = 74
        self.PrepLed = wx.StaticBitmap(self,
                                       pos=(prep_x+bias, ball_y),
                                       bitmap=ball,
                                       size=(ball.GetWidth()+10,
                                             ball.GetHeight()+10))

        self.StartLed = wx.StaticBitmap(self,
                                        pos=(start_x+bias, ball_y),
                                        bitmap=ball,
                                        size=(ball.GetWidth()+10, ball.GetHeight()+10))

        self.InsideLed = wx.StaticBitmap(self,
                                         pos=(inside_x+bias, ball_y),
                                         bitmap=ball,
                                         size=(ball.GetWidth()+10, ball.GetHeight()+10))

        self.StopLed = wx.StaticBitmap(self,
                                       pos=(stop_x+bias, ball_y),
                                       bitmap=ball,
                                       size=(ball.GetWidth()+10, ball.GetHeight()+10))


        panel = wx.Panel(self, pos=(10,130), size=(self.DimensionX-200, 54), style=wx.BORDER_SIMPLE)

        box = wx.StaticBox(panel, wx.ID_ANY)
        wx.StaticText(panel, wx.ID_ANY, "Scheduler",pos=(100,0))
        ball_y = 19
        labely = 37

        self.SchedPrepLed = wx.StaticBitmap(panel,
                                            pos=(18,ball_y),
                                            bitmap=ball,
                                            size=(ball.GetWidth()+10,
                                                  ball.GetHeight()+10))
        wx.StaticText(panel, wx.ID_ANY, "Prep", pos=(12,labely))

        self.SchedRunLed = wx.StaticBitmap(panel,
                                           pos=(88,ball_y),
                                           bitmap=ball,
                                           size=(ball.GetWidth()+10,
                                                 ball.GetHeight()+10))
        wx.StaticText(panel, wx.ID_ANY, "Run", pos=(84,labely))

        self.SchedBlackoutLed = wx.StaticBitmap(panel,
                                                pos=(158,ball_y),
                                                bitmap=ball,
                                                size=(ball.GetWidth()+10,
                                                      ball.GetHeight()+10))
        wx.StaticText(panel, wx.ID_ANY, "BlkOut", pos=(144,labely))

        self.SchedInsideLed = wx.StaticBitmap(panel,
                                              pos=(228,ball_y),
                                              bitmap=ball,
                                              size=(ball.GetWidth()+10,
                                                    ball.GetHeight()+10))
        wx.StaticText(panel, wx.ID_ANY, "Inside", pos=(218,labely))

#--#        self.SchedDoneLed = wx.StaticBitmap(panel,
#--#                                            pos=(290,ball_y),
#--#                                            bitmap=ball,
#--#                                            size=(ball.GetWidth()+10,
#--#                                                  ball.GetHeight()+10))
#--#        wx.StaticText(panel, wx.ID_ANY, "Done", pos=(286,labely))

        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnPrepare, self.PrepBtn)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnStart, self.StartBtn)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnInside, self.InsideBtn)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnStop, self.StopBtn)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnRestart, self.RestartBtn)

        self.SchedulerPostsThread = UtilityThread(self.FifoFileName, self.GotSchedMsg)
#--#        time.sleep(2)	# Give the new thread some time to get started.

    #**************************************************
    # GotSchedMsg
    #**************************************************
    def GotSchedMsg(self, ydata):
        # Here we want to keep track of what the scheduler
        # has most recently said about its state:
        #  "TIME": datetime.datetime.now().isoformat(),
        #   "PROGRAM_STATUS": PGMStatus,
        #   "PROGRAM_STATUS_TEXT": PgmStatusName[PGMStatus],
        #   "EMULATION_STATUS": estatus,
        #   "EMULATION_STATUS_TEXT": estatus_txt,
        #   "INSIDE_FLAG": InsideFlag,
        #   "PGM_MSG": Messages from the scheduler
        #   "PARAMS": jdict}

        ShowMessage("Msg rcvd from scheduler:", repr(ydata)) 

        # Was there a serious error?
        if ydata is None:
            self.SchedPrepLed.SetBitmap(self.RedBall)  
            self.SchedRunLed.SetBitmap(self.RedBall)  
            self.SchedInsideLed.SetBitmap(self.BlackBall)  
            self.SchedBlackoutLed.SetBitmap(self.BlackBall)  
            return

        # Do not listen to ghosts of things that are gone
        if self.SchedProcess is None:
            return

        self.SchedPgmStatus = ydata.get("PROGRAM_STATUS", None)
        self.SchedPgmStatusTxt = ydata.get("PROGRAM_STATUS_TEXT", "")
        self.SchedEmlStatus = ydata.get("EMULATION_STATUS", None)
        self.SchedEmlStatusText = ydata.get("EMULATION_STATUS_TEXT", "")
        self.SchedInsideStatus = ydata.get("INSIDE_FLAG", None)
        self.SchedPgmMsg = ydata.get("PGM_MSG", "")

        self.statusBar.SetStatusMsg(self.SchedPgmMsg)

        if self.SchedPgmStatus is not None:
            if self.SchedPgmStatus == 0: # Initializing
                self.SchedPrepLed.SetBitmap(self.GreenBall)  
                self.SetPrepared()
            elif self.SchedPgmStatus == 1: # Waiting to start
                self.SchedRunLed.SetBitmap(self.YellowBall)  
                self.SetWaitingToStart()
            elif self.SchedPgmStatus == 2: # Running
                self.SchedRunLed.SetBitmap(self.GreenBall)  
            elif self.SchedPgmStatus == 3: # Done
                self.SchedRunLed.SetBitmap(self.BlueBall)  

        if self.SchedEmlStatus is not None:
            if self.SchedEmlStatus == 0: # Initial
                self.SchedBlackoutLed.SetBitmap(self.BlackBall)  
                self.SchedInsideLed.SetBitmap(self.BlackBall)  
            elif self.SchedEmlStatus == 1: # Outside Normal
                self.SchedBlackoutLed.SetBitmap(self.BlackBall)  
                self.SchedInsideLed.SetBitmap(self.BlackBall)  
            elif self.SchedEmlStatus == 2: # Outside Blackout
                self.SchedBlackoutLed.SetBitmap(self.YellowBall)  
                self.SchedInsideLed.SetBitmap(self.BlackBall)  
            elif self.SchedEmlStatus == 3: # Inside Normal
                self.SchedBlackoutLed.SetBitmap(self.GreenBall)  
                self.SchedInsideLed.SetBitmap(self.GreenBall)  
            elif self.SchedEmlStatus == 4: # Inside Blackout
                self.SchedBlackoutLed.SetBitmap(self.RedBall)  
                self.SchedInsideLed.SetBitmap(self.GreenBall)  

    #**************************************************
    # OnPrepare
    #**************************************************
    def OnPrepare(self, event):
        global UseSyslog
        btn = event.GetEventObject()
        self.DisableButton(btn)

        ShowMessage("Prepare button clicked")

        self.StartBtn.SetValue(False)
        self.StopBtn.SetValue(False)
        self.InsideBtn.SetValue(False)

        # We can not go to green until we get a confirmation
        # from the scheduling program
        self.PrepLed.SetBitmap(self.YellowBall)

        if self.AutostartURI is not None:
            self.statusBar.SetStatusMsg("Fetching autostart data")
            nowdt = datetime.datetime.now()
            self.AutostartTime = GetNextSlot(nowdt, self.AutostartURI)
            if self.AutostartTime is None:
                ShowMessage("No Autostart time found - Aborting")
                self.ErrWin("No Autostart time found - Aborting")
                self.OnMenuExit(None)
                sys.exit(0)
            else:
                self.statusBar.SetStatusMsg("Confirming autostart time")
                rc = self.MsgWin("Autostart at %s?" % self.AutostartTime)
                if not rc:
                    ShowMessage("Autostart not confirmed - Aborting")
                    self.OnMenuExit(None)
                    sys.exit(0)
                ShowMessage("Autostart at %s confirmed" % self.AutostartTime)

        ShowMessage("Connecting to DCE")
        self.statusBar.SetStatusMsg("Connecting to DCE")

        fifofd = os.open(self.FifoFileName, os.O_RDONLY | os.O_NONBLOCK)
 
        fdnull = open("/dev/null", "w")
        try:
            subargs = ["/usr/bin/python", self.SchedPgm,
                       "--start_on_signal",
                       "--track", self.TrackName,
                       "--beaconpipe", self.FifoFileName,
                       "--csvfile", self.CsvFile,
                       "--inside_filename", self.InsideIndicatorFileName,
                       "--slowrate", str(self.SlowRate),
                       "--icmprate", str(self.ICMPRate)]
            if UseSyslog:
                subargs.append("--syslog")
            subargs.append(self.MaxProHostname)
            self.SchedProcess = subprocess.Popen(subargs,
                                                 stdout=sys.stdout, stderr=subprocess.STDOUT,
                                                 close_fds=True, shell=False)
#                        stdout=fdnull, stderr=fdnull,
        except Exception, err:
            self.statusBar.SetStatusMsg("Failed to launch scheduler")
            ShowMessage("Failed to launch scheduler", str(err))

        ShowMessage("Scheduler process started")
        fdnull.close()
        del fdnull

    def SetPrepared(self):
        if not self.Prepared:
            self.Prepared = True
            self.PrepLed.SetBitmap(self.GreenBall)
            self.StartLed.SetBitmap(self.YellowBall)
            if self.AutostartTime is None:
                self.EnableButton(self.StartBtn)

    def SetWaitingToStart(self):
        if self.AutostartTime is not None:
            self.DisableButton(self.StartBtn)
            self.StartBtn.SetBackgroundColour(self.BtnAutoBgColor)
            #self.EnableButton(self.RestartBtn)
            nowdt = datetime.datetime.now()
            if nowdt > self.AutostartTime:
                ShowMessage("Too late to autostart - Aborting")
                self.ErrWin("Too late to autostart - Aborting")
                self.OnMenuExit(None)
                sys.exit(0)
            msg = "Autostart %s" % self.AutostartTime
            self.statusBar.SetStatusMsg(msg)
            ShowMessage(msg)
 
            sleep4 = (self.AutostartTime - \
                     datetime.datetime.now()).total_seconds() - \
                     0.002  # Empirical fudge factor of 2ms.

            wx.CallLater(int(max(sleep4, 0.0) * 1000), self.WaitedToStart)

    def WaitedToStart(self):
            ShowMessage("Autostarting")
            self.StartHelper()

    def DisableButton(self, btn):
        #tn.SetForegroundColour(self.BtnDisableFgColor)
        #btn.SetBackgroundColour(self.BtnDisableBgColor)
        btn.SetBackgroundColour(None)
        btn.Disable()

    def EnableButton(self, btn):
        #btn.SetForegroundColour(self.BtnEnableFgColor)
        btn.SetBackgroundColour(self.BtnEnableBgColor)
        btn.Enable()

    #**************************************************
    # OnStart
    #**************************************************
    def OnStart(self, event):

        if not self.Prepared:
            self.StartBtn.SetValue(False)
            return

        ShowMessage("Start button clicked")

        self.StartHelper()

    #**************************************************
    # StartHelper
    #**************************************************
    def StartHelper(self):
        self.DisableButton(self.StartBtn)
        self.StopBtn.SetValue(False)
        self.InsideBtn.SetValue(False)

        self.StartLed.SetBitmap(self.YellowBall)
        self.StopLed.SetBitmap(self.YellowBall)
        self.InsideLed.SetBitmap(self.YellowBall)

        self.statusBar.SetClockTextCtrl("000:00",
                                        self.statusBar.ColorC,
                                        self.statusBar.ColorB)
        self.statusBar.SnapStartTime()
        self.StartLed.SetBitmap(self.GreenBall)

        self.EnableButton(self.InsideBtn)
        self.EnableButton(self.StopBtn)

        self.Running = True # **TEMPORARY**

        # Send the start signal to the sched process
        assert self.SchedProcess is not None
        self.SchedProcess.send_signal(signal.SIGUSR1)

    #**************************************************
    # StopSchedProcess
    #**************************************************
    def StopSchedProcess(self):
        if self.SchedProcess is not None:
            ShowMessage("Stopping scheduler process")
            try:
                self.SchedProcess.terminate()
                time.sleep(1)
                self.SchedProcess.kill()
            except:
                pass
            self.SchedProcess = None
            self.SchedPrepLed.SetBitmap(self.BlackBall)  
            self.SchedRunLed.SetBitmap(self.BlackBall)  
            self.SchedInsideLed.SetBitmap(self.BlackBall)  
            self.SchedBlackoutLed.SetBitmap(self.BlackBall)  

    #**************************************************
    # OnStop
    #**************************************************
    def OnStop(self, event):
        ShowMessage("Stop button clicked")
        btn = event.GetEventObject()
        self.DisableButton(btn)

        self.DisableButton(self.PrepBtn)
        self.DisableButton(self.StartBtn)
        self.DisableButton(self.InsideBtn)
        self.DisableButton(self.StopBtn)

        self.Stopped = True
        self.Running = False
        self.Prepared = False

        self.StopSchedProcess()
        self.StopSchedulerThread()
        self.PrepLed.SetBitmap(self.RedBall)
        self.StopLed.SetBitmap(self.RedBall)
        self.StartLed.SetBitmap(self.RedBall)

        self.EnableButton(self.RestartBtn)

    #**************************************************
    # OnRestart
    #**************************************************
    def OnRestart(self, event):
        ShowMessage("Restart button clicked")
        self.StopSchedProcess()
        self.StopSchedulerThread()
        self.App.Restart = True
        self.Close()

    #**************************************************
    # OnInside
    #**************************************************
    def OnInside(self, event):

        if not self.Running:
            self.InsideBtn.SetValue(False)
            return

        btn = event.GetEventObject()
        btnval = btn.GetValue()  # True or False
        #self.DisableButton(btn)

        if btnval:
            # Robot has moved *to* INSIDE
            ShowMessage("Inside/outside button clicked - now INSIDE")
            #clr = wx.ColourDatabase.Find(wx.ColourDatabase(), "AQUAMARINE")
            #clr = wx.ColourDatabase.Find(wx.ColourDatabase(), "YELLOW GREEN")
            #clr = wx.ColourDatabase.Find(wx.ColourDatabase(), "KHAKI")
            #clr = wx.ColourDatabase.Find(wx.ColourDatabase(), "WHEAT")
            clr = wx.ColourDatabase.Find(wx.ColourDatabase(), "SKY BLUE")
            self.InsideBtn.SetBackgroundColour(clr)
            self.InsideBtn.SetToolTip(wx.ToolTip("Robot is INSIDE"))

            #self.InsideBtn.SetLabel("OUTSIDE")
            self.InsideLed.SetBitmap(self.GreenBall)
            # Indicate inside status to the sched process
            subprocess.call(["touch", self.InsideIndicatorFileName], shell=False)
        else:
            # Robot has moved *to* OUTSIDE
            ShowMessage("Inside/outside button clicked - now OUTSIDE")
            clr = wx.ColourDatabase.Find(wx.ColourDatabase(), "GREEN")
            self.InsideBtn.SetBackgroundColour(clr)
            self.InsideBtn.SetToolTip(wx.ToolTip("Robot is OUTSIDE"))
            self.InsideLed.SetBitmap(self.YellowBall)
            # Indicate outside status to the sched process
            subprocess.call(["rm", "-f", self.InsideIndicatorFileName], shell=False)

    def StopSchedulerThread(self):
        if self.SchedulerPostsThread is not None:
            self.SchedulerPostsThread.PleaseStop()
            self.SchedulerPostsThread.join(10)
            self.SchedulerPostsThread = None

    #**************************************************
    #**************************************************
    def OnCloseFrame(self, event):
####        wx.lib.pubsub.Publisher.unsubscribe(self.GotPubMsg, topics=None)
        self.StopSchedProcess()
        self.StopSchedulerThread()
        self.Destroy()

    #**************************************************
    #**************************************************
    def ErrWin(self, msg, hdr = "Error",
                    flags = wx.OK | wx.ICON_EXCLAMATION):
        dlg = wx.MessageDialog(self, msg, hdr, flags)
        dlg.ShowModal()
        dlg.Destroy()

    #**************************************************
    #**************************************************
    def MsgWin(self, msg, hdr = "Question",
                    flags = wx.YES_NO | wx.YES_DEFAULT | wx.ICON_EXCLAMATION):
        dlg = wx.MessageDialog(self, msg, hdr, flags)
        rc = dlg.ShowModal()
        dlg.Destroy()
        return rc == wx.ID_YES

    #**************************************************
    #**************************************************
    def OnMenuExit(self, event):
        self.StopSchedProcess()
        self.StopSchedulerThread()
        self.Close(True)

    #**************************************************
    #**************************************************
    def OnMenuAbout(self, event):
        dlg = wx.lib.dialogs.ScrolledMessageDialog(self,
                                                   "Some about text **TODO**",
                                                   "About DRCC",
                                                   size = (540, 500))
        try:
            dlg.ShowModal()
        finally:
            dlg.Destroy()

    #**************************************************
    #**************************************************
    def CreateMenuBar(self, menuData):
        menuBar = wx.MenuBar()
        for menuLabel, menuNodes in menuData:
            menuBar.Append(self.CreateMenu(menuNodes), menuLabel)
        self.SetMenuBar(menuBar)

    #**************************************************
    #**************************************************
    def CreateMenu(self, menuNodes):
        menu = wx.Menu()
        for node in menuNodes:
            if len(node) == 2:
                menu.AppendMenu(wx.NewId(), node[0], self.CreateMenu(node[1]))
            else:
                if len(node) == 3:
                    node.append(wx.ITEM_NORMAL)
                if node[0] == "":
                    entry = menu.AppendSeparator()
                else:
                    label, status, handler, kind = node
                    entry = menu.Append(-1, label, status, kind)
                    self.Bind(wx.EVT_MENU, handler, entry)
                node.append(entry)
        return menu

#**********************************************************************
# DRCCApp
#**********************************************************************
class DRCCApp(wx.App):
    def __init__(self, trackname, position, trackcolorname, autostart_uri,
                 schedpgm, fifofilename, insidefilename,
                 csvfile, maxpro_hostname, slowrate, icmprate):
        self.TrackName = trackname
        self.OurPosition = position
        self.TrackColorName = trackcolorname
        self.AutostartURI = autostart_uri
        self.SchedPgm = schedpgm
        self.FifoFileName = fifofilename
        self.InsideIndicatorFileName = insidefilename
        self.CsvFile = csvfile
        self.MaxProHostname = maxpro_hostname
        self.SlowRate = slowrate
        self.ICMPRate = icmprate
        self.Restart = False
        wx.App.__init__(self, 0)

    def OnInit(self):
        wx.InitAllImageHandlers()
        self.TrackColor = wx.ColourDatabase.Find(wx.ColourDatabase(),
                                                 self.TrackColorName)
        self.DrccFrame = DRCCFrame(self,
                                   self.TrackName,
                                   self.OurPosition,
                                   self.TrackColor,
                                   self.AutostartURI,
                                   self.SchedPgm,
                                   self.FifoFileName,
                                   self.InsideIndicatorFileName,
                                   self.CsvFile, 
                                   self.MaxProHostname,
                                   self.SlowRate,
                                   self.ICMPRate)
        self.DrccFrame.Show(True)
        self.SetTopWindow(self.DrccFrame)
#--#        if self.DrccFrame.errMsgInfo:
#--#            self.DrccFrame.ErrWin(self.DrccFrame.errMsgInfo[0],
#--#                                  self.DrccFrame.errMsgInfo[1])
#--#        self.DrccFrame.errMsgInfo = ()
        return True

#--#    def OnExit(self):
#--#        pass

########################################################################
#http://www.blog.pythonlibrary.org/2010/05/22/wxpython-and-threads/

class UtilityThread(threading.Thread):
    def __init__(self, fifofilename, schedmsgfunc):
        self.FifoFileName = fifofilename
        self.SchedMsgFunc = schedmsgfunc
        self.__PleaseStop = False
        # The utility thread needs some time to initialize.
        # So we create a semaphore so that it can indicate when the
        # initialization is complete.
        self.StartSema = threading.Semaphore(0)
        threading.Thread.__init__(self)
        self.start()    # start the thread
        self.StartSema.acquire() # Wait for the thread to initialize
 
    def PleaseStop(self):
        self.__PleaseStop = True

#--#    def SafeRead(fifofd, size=1024):
#--#        ''' reads data from a pipe and returns `None` on EAGAIN '''
#--#        try:
#--#            return os.read(fifofd, size)
#--#        except OSError, exc:
#--#            if exc.errno == errno.EAGAIN:
#--#                return None
#--#            raise

    #----------------------------------------------------------------------
    def run(self):
        """Run Worker Thread."""
        # This is the code executing in the new thread.

        fifofd = None
        while True:
            if self.__PleaseStop:
                self.StartSema.release() # Let the main thread continue
                return
            try:
                fifofd = os.open(self.FifoFileName, os.O_RDONLY | os.O_NONBLOCK)
            except Exception, err:
                self.StartSema.release() # Let the main thread continue
                wx.CallAfter(self.SchedMsgFunc, str(err))
                return
            break

        fifoobj = os.fdopen(fifofd, "r")
        self.StartSema.release() # Let the main thread continue
        while True:
            if self.__PleaseStop:
                break

            rlist, wlist, xlist = select.select((fifofd,), (), (), 1)

            if self.__PleaseStop:
                break

            sdata = None
            if fifofd in rlist:
                try:
                    sdata = fifoobj.readline(1024)
                    if len(sdata) == 0:
                        break
                    ydata = yaml.safe_load(sdata)
                    wx.CallAfter(self.SchedMsgFunc, ydata)
                except OSError, err:
                    if err.errno == errno.EAGAIN:
                        continue
                    break
                except Exception, err:
                    print "KABOOM", str(err)
                    print repr(sdata)
                    if sdata is not None:
                        fd = open("/tmp/sdata.log", "w") # truncates existing file, if any
                        fd.write(sdata)
                        fd.close()
                    wx.CallAfter(self.SchedMsgFunc, None)
                    break

        try:
            fifoobj.close()
        except:
            pass
 
#**********************************************************************
# main()
#**********************************************************************
def main():
    global TrackName
    global UseSyslog

    try:
        x = yaml.safe_load('{"hello": 0}\r\n')
    except Exception, err:
        print "YTEST FAIL", str(err)
        sys.exit(0)

    #******************************
    # Deal with the arguments
    #******************************
    #**********************************************************************
    # From http://stackoverflow.com/questions/3853722/
    #             python-argparse-how-to-insert-newline-the-help-text
    #**********************************************************************
    class SmartFormatter(argparse.HelpFormatter):

        def _split_lines(self, text, width):
            # this is the RawTextHelpFormatter._split_lines
            if text.startswith('R|'):
                return text[2:].splitlines()  
            return argparse.HelpFormatter._split_lines(self, text, width)


    parser = argparse.ArgumentParser(description="DRC Finals Control Panel.",
                                   usage="%(prog)s [options]  Use -h for help.",
                                   formatter_class=SmartFormatter)

    parser.add_argument('-a', '--autostart_uri',
                        dest='autostart_uri',
                        required=False,
                        help="R|Track schedule file file URI.\n"
                             "(The presence of this parameter triggers autostart mode.)")

    parser.add_argument('-b', '--blackout_file',
                        dest='blackout_file',
                        required=False,
                        default="Blackouts.csv",
                        help="R|CSV schedule file file name [default: %(default)s]\n"
                             "(This file name is relative to the --trackdir directory.)")

    parser.add_argument("-P", "--position",
                        type=str, dest="position", default=None,
                        help="R|Indicate upper left corner position of application window\n"
                             "Use 'center' to center the window.\n"
                             "Format: XxY | center")

    parser.add_argument("-r", "--slowrate", required=False, metavar="<slow link bit rate>",
                        type=int, default=9600,
                        help="R|Slow link bit rate [default: %(default)s]")

    parser.add_argument("-i", "--icmprate", required=False, metavar="<icmp link bit rate>",
                        type=int, default=4800,
                        help="R|ICMP link bit rate [default: %(default)s]")

    parser.add_argument("-t", "--trackname", required=True, metavar="<track name>",
                        dest='trackname',
                        help="R|Track name.")

    parser.add_argument("-C", "--trackcolor", metavar="<track color>",
                        dest='trackcolor',
                        default="PURPLE",
                        help="R|Track color [default: %(default)s]\n"
                             "Red Track:    'CORAL'\n"
                             "Green Track:  'PALE GREEN'\n"
                             "Blue Track:   'LIGHT BLUE'\n"
                             "Yellow Track: 'GOLDENROD'")

    parser.add_argument('--scheduler',
                        dest='tracksched',
                        required=False,
                        default="track.py",
                        help="R|Name of the track scheduler program [default: %(default)s]\n"
                              "(This file name is relative to the --trackdir directory.)")

    parser.add_argument("--syslog", action="store_true",
                        help="R|Send reports to the system defined syslog server\n"
                             "Level LOG_INFO, Facility LOG_USER).")

    parser.add_argument('-D', '--trackdir',
                        dest='trackdir',
                        required=False,
                        default=".",
                        help="R|Directory that contains the track scheduler program\n"
                             "and the CSV file.  [default: %(default)s]\n")

    parser.add_argument("maxpro_hostname",  metavar="<Maxwell Pro host name or IP address>",
			help='R|The host name or IP address of the Maxwell Pro.')

    pargs = parser.parse_args()

    trackdir = os.path.abspath(pargs.trackdir)
    blackout_file = os.path.abspath(os.path.join(trackdir, pargs.blackout_file))
    schedpgm = os.path.abspath(os.path.join(trackdir, pargs.tracksched))

    # Create a temporary directory for our named pipe from the scheduling program
    # and for the inside/outside indicator file
    tempdir = os.path.abspath(tempfile.mkdtemp())

    UseSyslog = pargs.syslog
    TrackName = pargs.trackname

    cycle = 0
    while True:
        cycle += 1
        fifofilename = os.path.abspath(os.path.join(tempdir, "fromsched_%u.pipe" % cycle))
        insidefilename = os.path.abspath(os.path.join(tempdir, "inside_indicator_%u" % cycle))

        ShowMessage("AUTOSTART", pargs.autostart_uri)
        ShowMessage("POSITION", pargs.position)
        ShowMessage("CYCLE", cycle)
        ShowMessage("TRACK", TrackName)
        ShowMessage("TRACK COLOR", pargs.trackcolor)
        ShowMessage("TDIR", tempdir)
        ShowMessage("FIFO", fifofilename)
        ShowMessage("INSIDE INDICATOR", insidefilename)
        ShowMessage("TRACKDIR", trackdir)
        ShowMessage("BLACKOUT_FILE", blackout_file)
        ShowMessage("SCHEDPGM", schedpgm)
        ShowMessage("MAXPRO", pargs.maxpro_hostname)
        ShowMessage("SLOW BITRATE", pargs.slowrate)
        ShowMessage("ICMP BITRATE", pargs.icmprate)

        #**************************************************
        # Do some early validation
        #**************************************************
        if not os.access(blackout_file, os.F_OK):
            raise DRCCtrlError, "Can not locate blackout CSV file" 

        if not os.access(schedpgm, os.F_OK):
            raise DRCCtrlError, "Can not locate scheduler program file" 

        try:
            os.remove(fifofilename)
        except:
            pass
        os.mkfifo(fifofilename)

        try:
            os.remove(insidefilename)
        except:
            pass

        app = DRCCApp(TrackName, pargs.position, pargs.trackcolor,
                      pargs.autostart_uri,
                      schedpgm, fifofilename, insidefilename,
                      blackout_file, pargs.maxpro_hostname,
                      pargs.slowrate, pargs.icmprate)
        app.MainLoop()

        restart = app.Restart

        try:
            os.remove(fifofilename)
        except:
            pass

        try:
            os.remove(insidefilename)
        except:
            pass

        del app

        if not restart:
            break

    try:
        os.rmdir(tempdir)
    except:
        pass

#**********************************************************************
# Entry point
#**********************************************************************
if __name__ == '__main__':
    try:
        main()
    except Exception, err:
        print "Exception occurrred:", str(err)
        traceback.print_exc()        
