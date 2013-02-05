# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# Copyright (c) 2012, hiDOF, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('multisense_dashboard')

import wx
import wx.aui
import wx.py.shell
import rxtools
import rxtools.cppwidgets as rxtools

from status_control import StatusControl
import robot_monitor
from robot_monitor.robot_monitor_panel import RobotMonitorPanel
import diagnostic_msgs.msg
import std_msgs.msg
import std_srvs.srv

import rospy
from rosgraph import rosenv

from os import path
import threading

from diagnostics_frame import DiagnosticsFrame
from rosout_frame import RosoutFrame

class MultisenseFrame(wx.Frame):
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"

    def __init__(self, parent, id=wx.ID_ANY, title='Multisense Dashboard', pos=wx.DefaultPosition, size=(400, 50), style=wx.CAPTION|wx.CLOSE_BOX|wx.STAY_ON_TOP):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)

        wx.InitAllImageHandlers()

        rospy.init_node('multisense_dashboard', anonymous=True)
        try:
            getattr(rxtools, "initRoscpp")
            rxtools.initRoscpp("multisense_dashboard_cpp", anonymous=True)
        except AttributeError:
            pass

        self.SetTitle('Multisense Dashboard (%s)'%rosenv.get_master_uri())

        icons_path = path.join(roslib.packages.get_pkg_dir('multisense_dashboard'), "icons/")

        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.SetSizer(sizer)

        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Diagnostic"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)

        # Diagnostics
        self._diagnostics_button = StatusControl(self, wx.ID_ANY, icons_path, "diag", True)
        self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics"))
        static_sizer.Add(self._diagnostics_button, 0)

        # Rosout
        self._rosout_button = StatusControl(self, wx.ID_ANY, icons_path, "rosout", True)
        self._rosout_button.SetToolTip(wx.ToolTip("Rosout"))
        static_sizer.Add(self._rosout_button, 0)

        # Text
        static_sizer.AddSpacer(12)
        vert_sizer = wx.BoxSizer(wx.VERTICAL)
        vert_sizer.AddSpacer(7)
        vert_sizer.Add(wx.StaticText(self, wx.ID_ANY, "MultiSense", style=wx.ALIGN_CENTER))
        static_sizer.Add(vert_sizer)
        static_sizer.AddSpacer(8)

        self._config = wx.Config("multisense_dashboard")

        self.Bind(wx.EVT_CLOSE, self.on_close)

        self.Layout()
        self.Fit()

        self._diagnostics_frame = DiagnosticsFrame(self, wx.ID_ANY, "Diagnostics")
        self._diagnostics_frame.Hide()
        self._diagnostics_frame.Center()
        self._diagnostics_button.Bind(wx.EVT_BUTTON, self.on_diagnostics_clicked)

        self._rosout_frame = RosoutFrame(self, wx.ID_ANY, "Rosout")
        self._rosout_frame.Hide()
        self._rosout_frame.Center()
        self._rosout_button.Bind(wx.EVT_BUTTON, self.on_rosout_clicked)

        self.load_config()

        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self._timer.Start(500)

        self._dashboard_agg_sub = rospy.Subscriber('diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray, self.dashboard_callback)

        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0

    def __del__(self):
        self._dashboard_agg_sub.unregister()

    def on_timer(self, evt):
      level = self._diagnostics_frame.get_top_level_state()
      if (level == -1 or level == 3):
        if (self._diagnostics_button.set_stale()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Stale"))
      elif (level >= 2):
        if (self._diagnostics_button.set_error()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Error"))
      elif (level == 1):
        if (self._diagnostics_button.set_warn()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Warning"))
      else:
        if (self._diagnostics_button.set_ok()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: OK"))

      self.update_rosout()

      if (rospy.is_shutdown()):
        self.Close()

    def on_diagnostics_clicked(self, evt):
      self._diagnostics_frame.Show()
      self._diagnostics_frame.Raise()

    def on_rosout_clicked(self, evt):
      self._rosout_frame.Show()
      self._rosout_frame.Raise()


    def dashboard_callback(self, msg):
      wx.CallAfter(self.new_dashboard_message, msg)

    def new_dashboard_message(self, msg):
      self._dashboard_message = msg
      self._last_dashboard_message_time = rospy.get_time()


    def update_rosout(self):
      summary_dur = 30.0
      if (rospy.get_time() < 30.0):
          summary_dur = rospy.get_time() - 1.0

      if (summary_dur < 0):
          summary_dur = 0.0

      summary = self._rosout_frame.get_panel().getMessageSummary(summary_dur)

      if (summary.fatal or summary.error):
        self._rosout_button.set_error()
      elif (summary.warn):
        self._rosout_button.set_warn()
      else:
        self._rosout_button.set_ok()


      tooltip = ""
      if (summary.fatal):
        tooltip += "\nFatal: %s"%(summary.fatal)
      if (summary.error):
        tooltip += "\nError: %s"%(summary.error)
      if (summary.warn):
        tooltip += "\nWarn: %s"%(summary.warn)
      if (summary.info):
        tooltip += "\nInfo: %s"%(summary.info)
      if (summary.debug):
        tooltip += "\nDebug: %s"%(summary.debug)

      if (len(tooltip) == 0):
        tooltip = "Rosout: no recent activity"
      else:
        tooltip = "Rosout: recent activity:" + tooltip

      if (tooltip != self._rosout_button.GetToolTip().GetTip()):
          self._rosout_button.SetToolTip(wx.ToolTip(tooltip))

    def load_config(self):
      # Load our window options
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      if (self._config.HasEntry(self._CONFIG_WINDOW_X)):
          x = self._config.ReadInt(self._CONFIG_WINDOW_X)
      if (self._config.HasEntry(self._CONFIG_WINDOW_Y)):
          y = self._config.ReadInt(self._CONFIG_WINDOW_Y)

      self.SetPosition((x, y))
      self.SetSize((width, height))

    def save_config(self):
      config = self._config

      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      config.WriteInt(self._CONFIG_WINDOW_X, x)
      config.WriteInt(self._CONFIG_WINDOW_Y, y)

      config.Flush()

    def on_close(self, event):
      self.save_config()

      self.Destroy()

