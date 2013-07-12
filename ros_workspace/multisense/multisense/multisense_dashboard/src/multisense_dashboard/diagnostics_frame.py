# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

import rospy
from diagnostic_msgs.msg import DiagnosticStatus

import threading
import wx
import robot_monitor
from robot_monitor.robot_monitor_panel import RobotMonitorPanel

class DiagnosticsFrame(wx.Frame):
  def __init__(self, parent, id, title):
    wx.Frame.__init__(self, parent, id, title, wx.DefaultPosition, wx.Size(400, 600))
    wx.Frame.Hide(self)

    self.Bind(wx.EVT_CLOSE, self.on_close)

    self._diagnostics_panel = None
    self._is_stale = True
    self._top_level_state = -1

    self._diagnostics_toplevel_state_sub = rospy.Subscriber('diagnostics_toplevel_state', DiagnosticStatus, self.dashboard_callback)
    self._stall_timer = None

  def on_close(self, evt):
    self.Hide()

  def Show(self):
    self._diagnostics_panel = RobotMonitorPanel(self)
    self._diagnostics_panel.SetSize(self.GetClientSize())
    wx.Frame.Show(self)

  def Hide(self):
    wx.Frame.Hide(self)
    if self._diagnostics_panel is not None:
      self._diagnostics_panel.Close()
      self._diagnostics_panel.Destroy()
      self._diagnostics_panel = None

  def dashboard_callback(self, msg):
    self._is_stale = False
    if self._stall_timer is not None:
      self._stall_timer.cancel()
    # timer to mark stalled when not receiving messages for some time
    self._stall_timer = threading.Timer(10, self._stalled)

    self._top_level_state = msg.level

  def _stalled(self, event):
   self._stall_timer = None
   self._is_stale = True

  def get_top_level_state(self):
    if (self._is_stale):
      return 3

    return self._top_level_state

