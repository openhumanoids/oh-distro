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

import wx

from os import path

class StatusControl(wx.Window):
  def __init__(self, parent, id, icons_path, base_name, toggleable):
    wx.Window.__init__(self, parent, id)
    self.SetSize(wx.Size(32, 32))

    if (toggleable):
      self._ok = (wx.Bitmap(path.join(icons_path, "%s-green-untoggled.png"%(base_name)), wx.BITMAP_TYPE_PNG),
                  wx.Bitmap(path.join(icons_path, "%s-green-toggled.png"%(base_name)), wx.BITMAP_TYPE_PNG))
      self._warn = (wx.Bitmap(path.join(icons_path, "%s-yellow-untoggled.png"%(base_name)), wx.BITMAP_TYPE_PNG),
                    wx.Bitmap(path.join(icons_path, "%s-yellow-toggled.png"%(base_name)), wx.BITMAP_TYPE_PNG))
      self._error = (wx.Bitmap(path.join(icons_path, "%s-red-untoggled.png"%(base_name)), wx.BITMAP_TYPE_PNG),
                   wx.Bitmap(path.join(icons_path, "%s-red-toggled.png"%(base_name)), wx.BITMAP_TYPE_PNG))
      self._stale = (wx.Bitmap(path.join(icons_path, "%s-grey-untoggled.png"%(base_name)), wx.BITMAP_TYPE_PNG),
                     wx.Bitmap(path.join(icons_path, "%s-grey-toggled.png"%(base_name)), wx.BITMAP_TYPE_PNG))
    else:
      ok = wx.Bitmap(path.join(icons_path, "%s-green.png"%(base_name)), wx.BITMAP_TYPE_PNG)
      warn = wx.Bitmap(path.join(icons_path, "%s-yellow.png"%(base_name)), wx.BITMAP_TYPE_PNG)
      error = wx.Bitmap(path.join(icons_path, "%s-red.png"%(base_name)), wx.BITMAP_TYPE_PNG)
      stale = wx.Bitmap(path.join(icons_path, "%s-grey.png"%(base_name)), wx.BITMAP_TYPE_PNG)
      self._ok = (ok, ok)
      self._warn = (warn, warn)
      self._error = (error, error)
      self._stale = (stale, stale)

    self._color = None
    self.set_stale()

    self.Bind(wx.EVT_PAINT, self.on_paint)
    self.Bind(wx.EVT_LEFT_UP, self.on_left_up)
    self.Bind(wx.EVT_LEFT_DOWN, self.on_left_down)
    self.Bind(wx.EVT_LEAVE_WINDOW, self.on_leave_window)
    self.Bind(wx.EVT_ENTER_WINDOW, self.on_enter_window)

    self._toggled = False
    self._left_down = False

  def toggle(self, tog):
    if (self._toggled == tog):
        return False

    self._toggled = tog
    self.Refresh()

    return True

  def on_left_down(self, evt):
    self.toggle(True)
    self._left_down = True
    self.Refresh()

  def on_left_up(self, evt):
    self.toggle(False)
    self._left_down = False
    x = evt.GetX()
    y = evt.GetY()
    if (x >= 0 and y >= 0 and x < self.GetSize().GetWidth() and y < self.GetSize().GetHeight()):
      event = wx.CommandEvent(wx.EVT_BUTTON._getEvtType(), self.GetId())
      wx.PostEvent(self, event)

    self.Refresh()

  def on_leave_window(self, evt):
    self.toggle(False)
    self.Refresh()

  def on_enter_window(self, evt):
    if (self._left_down):
      self.toggle(True)

    self.Refresh()

  def on_paint(self, evt):
    dc = wx.BufferedPaintDC(self)
    dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
    dc.Clear()

    size = self.GetSize();

    bitmap = None
    if (self._toggled):
      bitmap = self._color[1]
    else:
      bitmap = self._color[0]

    dc.DrawBitmap(bitmap, (size.GetWidth() - bitmap.GetWidth()) / 2.0, (size.GetHeight() - bitmap.GetHeight()) / 2.0, True)

  def set_ok(self):
    if (self._color == self._ok):
        return False

    self._color = self._ok
    self.update()

    return True

  def set_warn(self):
    if (self._color == self._warn):
        return False

    self._color = self._warn
    self.update()

    return True

  def set_error(self):
    if (self._color == self._error):
        return False

    self._color = self._error
    self.update()

    return True

  def set_stale(self):
    if (self._color == self._stale):
        return False

    self._color = self._stale
    self.update()

    return True

  def update(self):
    self.Refresh()
