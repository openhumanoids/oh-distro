#!/usr/bin/python

#**********************************************************************
# $Id: servercomm.py 39 2015-01-23 01:10:14Z jim $
#**********************************************************************

import socket
import time
import signal
import sys
import os

# When the debugFlags global variable is set to non-zero values then
# debugging info is printed to stdout. The currently valid bit flags are:
# 1: Print the command actually sent to the stdiserver process just before
#    it is sent.
# 2: Print the response actually received from the stdiserver process just
#    before it is processed.
debugFlags = 0

#==============================================================================
def GetEthDevices():
    ethDevices = os.listdir("/proc/sys/net/ipv4/conf/")
    ethDevices.remove("lo")
    ethDevices.remove("all")
    ethDevices.remove("default")
    fp = open("/proc/net/route")
    controlPortFnd = False
    line = fp.readline()
    while line:
        toks = line.split("\t")
        if toks[1] == "00000000":
            # Found default route device - disallow it from device list
            # since it is probably the control port device.
            ethDevices.remove(toks[0])
            controlPortFnd = True
            break
        line = fp.readline()
    fp.close()
    if not controlPortFnd:
        if "eth0" in ethDevices:
            ethDevices.remove("eth0")
        elif "em1" in ethDevices:
            ethDevices.remove("em1")
    # The maxtap entry must always be at the end of the list because
    # the restart logic assumes the first two devices are the standard
    # devices. Otherwise we present the devices in alphabetical order.
    if "maxtap" in ethDevices:
        ethDevices.remove("maxtap")
    ethDevices.sort()
    ethDevices.append("maxtap")
    return ethDevices

#==============================================================================
class ServerError(Exception):
    """Server error responses and socket errors are transformed to instances of
    this class. The string representation of the object contains the human
    readable error reason.
    """
    def __init__(self, msg):
        "Constructor takes a single string description of the error."
        self.msg = msg

    def __str__(self):
        return self.msg

#==============================================================================
class TokenList:
    """An internal class used to parse a response line into tokens that are
    then converted to appropriate values.
    """
    def __init__(self):
        self.index = 0
        self.inQuote = False
        self.inBackSlash = False
        self.inToken = False
        self.tokens = []
        self.token = ""

    def AddText(self, src):
        for ch in src:
            if self.inBackSlash:
                if self.inToken:
                    self.token += ch
                elif ch <> '\n':
                    self.inToken = True
                    self.token = ch
                self.inBackSlash = False
            elif ch == '\\':
                self.inBackSlash = True
            elif self.inQuote:
                if ch == '"':
                    self.tokens.append(self.token)
                    self.inToken = self.inQuote = False
                    self.token = ""
                else:
                    self.token += ch
            else:
                if ch <= ' ':
                    if self.inToken:
                        self.tokens.append(self.token)
                        self.inToken = False
                        self.token = ""
                    if ch == '\n':
                        return False
                elif ch == '"':
                    self.inToken = self.inQuote = True
                    self.token = ""
                else:
                    self.inToken = True
                    self.token += ch
        return True

    def Pop(self, toLower = True):
        "Return and consume the next token string."
        token = self.tokens[self.index]
        if toLower:
            token = token.lower()
        self.index = self.index + 1
        return token

    def PopInt(self):
        "Return and consume the next token and convert it to an int."
        return int(self.Pop())

    def PopFloat(self):
        "Return and consume the next token and convert it to a float."
        return float(self.Pop())

    def Next(self, toLower = True):
        """Return the next token without consuming it. Returns None if no
        next token.
        """
        if self.Remain():
            token = self.tokens[self.index]
            if toLower:
                token = token.lower()
            return token
        else:
            return None

    def SkipOne(self):
        "Consumes the next token and returns None."
        self.index = self.index + 1

    def Remain(self):
        "Returns True if more unconsumed tokens remain."
        return self.index < len(self.tokens)

#==============================================================================
class PluginMetaInfo:
    """An instance of this class is returned by the GetPluginMeta method of the
    ServerControl class. It contains the information that describes what
    control fields are relevant for the currently installed plugin, if any.
    The return values of the getpluginmeta command are placed in the following

    Public Attributes:

    path: string
    opts: string
    ver: string
    id: string
    desc: string
    checkBoxes: list of CheckBoxMeta instances
    radioButtons: list of RadioButtonMeta instances
    numberFields: list of NumberFieldMeta instances
    """
    def __init__(self, tokens = None):
        self.Reset()
        if tokens:
            tokens.Pop()
            self.path = tokens.Pop(False)
            tokens.Pop()
            self.opts = tokens.Pop(False)
            tokens.Pop()
            self.ver = tokens.Pop(False)
            tokens.Pop()
            self.id = tokens.Pop(False)
            tokens.Pop()
            self.desc = tokens.Pop(False)
            while tokens.Next() == "checkbox":
                tokens.SkipOne()
                self.checkBoxes.append(CheckBoxMeta(tokens))
            while tokens.Next() == "radiobuttons":
                tokens.SkipOne()
                self.radioButtons = RadioButtonMeta(tokens)
            while tokens.Next() == "number":
                tokens.SkipOne()
                self.numberFields.append(NumberFieldMeta(tokens))

    def Reset(self):
        "Reset the public attributes to their default values."
        self.path = ""
        self.opts = ""
        self.ver = ""
        self.id = ""
        self.desc = ""
        self.checkBoxes = []
        self.radioButtons = None
        self.numberFields = []

    def __str__(self):
        desc = "Path: '%s'\nOpts: '%s'\nVer: '%s'\nID: '%s'\nDesc: '%s'\n" % \
                (self.path, self.opts, self.ver, self.id, self.desc)
        for checkBox in self.checkBoxes:
            desc = "%sCheckbox %s\n" % (desc, str(checkBox))
        if self.radioButtons:
            desc = "%sRadiobuttons %s\n" % (desc, str(self.radioButtons))
        for numberField in self.numberFields:
            desc = "%sNum Field %s\n" % (desc, str(numberField))
        return desc

#==============================================================================
class CheckBoxMeta:
    """Instances of this class are placed in the checkBoxes list attribute of
    the PluginMetaInfo instance.

    Public Attributes:

    index: int
    label: string
    tooltip: string
    enableExp: string
    initValue: int
    """
    def __init__(self, tokens):
        self.index = tokens.PopInt()
        self.label = tokens.Pop(False)
        self.tooltip = tokens.Pop(False)
        self.enableExp = tokens.Pop(False)
        self.initValue = tokens.PopInt()

    def __str__(self):
        return "%d Label: '%s' Tip: '%s' Exp: '%s' Init: %d" % \
                (self.index, self.label, self.tooltip, self.enableExp,
                 self.initValue)

#==============================================================================
class RadioButtonMeta:
    """Instances of this class are placed in the radioButtons list attribute of
    the PluginMetaInfo instance.

    Public Attributes:

    label: string
    tooltip: string
    enableExp: string
    initValue: int
    buttons: array of strings
    """
    def __init__(self, tokens):
        self.label = tokens.Pop(False)
        self.tooltip = tokens.Pop(False)
        self.enableExp = tokens.Pop(False)
        self.initValue = tokens.PopInt()
        self.buttons = []
        while tokens.Next() <> "end":
            index = tokens.PopInt()
            label = tokens.Pop(False)
            while len(self.buttons) <= index:
                self.buttons.append("")
            self.buttons[index] = label
        if tokens.Next() == "end":
            tokens.SkipOne()

    def __str__(self):
        desc = "Label: '%s' Tip: '%s' Exp: '%s' Init: %d\n" % \
                (self.label, self.tooltip, self.enableExp, self.initValue)
        for idx in range(len(self.buttons)):
            desc = "%s%5d: '%s'\n" % (desc, idx, self.buttons[idx])
        return desc

#==============================================================================
class NumberFieldMeta:
    """Instances of this class are placed in the numberFields list attribute of
    the PluginMetaInfo instance.

    Public Attributes: 

    index: int
    label: string
    tooltip: string
    enableExp: string
    type: string
    hardMin: int
    hardMax: int
    initMin: int
    initMax: int
    initValue: int
    """
    def __init__(self, tokens):
        self.index = tokens.PopInt()
        self.label = tokens.Pop(False)
        self.tooltip = tokens.Pop(False)
        self.enableExp = tokens.Pop(False)
        self.type = ["slider", "spinner", "simple"][tokens.PopInt()]
        self.hardMin = tokens.PopInt()
        self.hardMax = tokens.PopInt()
        self.initMin = tokens.PopInt()
        self.initMax = tokens.PopInt()
        self.initValue = tokens.PopInt()

    def __str__(self):
        return "%d Label: '%s' Tip: '%s' Exp: '%s' Init: %d\n" \
               "Type: %s Hmin: %d Hmax: %d Imin: %d Imax: %d\n" % \
                (self.index, self.label, self.tooltip, self.enableExp,
                 self.initValue, self.type, self.hardMin, self.hardMax,
                 self.initMin, self.initMax)

#==============================================================================
class AdHocRec:
    """This class acts as a generic structure that "self-builds" public
    attributes based on the name/value pairs that are supplied to the
    constructor. Saves having to define many small structure-only classes.
    """
    def __init__(self, *va, **kw):
        self.initVals = { }
        for k, v in kw.items():
            self.__dict__[k] = v
            self.initVals[k] = v

    def Reset(self):
        "Reset the attributes to the values set when the instance was created."
        self.__dict__.update(self.initVals)

    def Swap(self, other):
        """Swap the values from AdHocRec other instance with the ones in this
        instance. The two objects must have the same names defined in their
        namespace for this to succeed.
        """
        for k in self.__dict__.keys():
            self.__dict__[k], other.__dict__[k] = \
                    other.__dict__[k], self.__dict__[k]

#==============================================================================
class AddressMatch:
    """Objects of this class contain the direction, match, and mask info needed
    for matching MAC addresses, IPv4 addresses, IPv6 addresses, or port numbers.

    Public Attributes: 

    direc: string ("src", "dst", "both", or "notused")
    match: string or int
    mask: string or int
    """
    def __init__(self, direc, match, mask):
        self.direc = self.initDirec = direc
        self.match = self.initMatch = match
        self.mask = self.initMask = mask

    def Reset(self):
        "Reset the public attributes to their default values."
        self.direc = self.initDirec
        self.match = self.initMatch
        self.mask = self.initMask

    def Swap(self, other):
        """Swap the public attributes of the AddressMatch object 'other' with
        the public attributes of this instance.
        """
        self.direc, other.direc = other.direc, self.direc
        self.match, other.match = other.match, self.match
        self.mask, other.mask = other.mask, self.mask

#==============================================================================
class MatchLan:
    """Objects of this class contain Lan header match information.

    Public Attributes: 

    enabled: boolean
    usesVlan: boolean
    vlanIdMatch: int
    vlanIdMask: int
    priorityMatch: int
    priorityMask: int
    typeMatch: int
    typeMask: int
    addr: array of AddressMatch
    """
    def __init__(self, tokens = None):
        self.addr = [ ]
        for idx in range(2):
            self.addr.append(AddressMatch("notused", "000000000000",
                                          "000000000000"))
        self.Reset()
        if tokens:
            self.enabled = True
            if tokens.Next() == "usesvlan":
                tokens.SkipOne()
                self.usesVlan = True
            if tokens.Next() == "vlanid":
                tokens.SkipOne()
                self.vlanIdMatch = tokens.PopInt()
                self.vlanIdMask = tokens.PopInt() 
            if tokens.Next() == "priority":
                tokens.SkipOne()
                self.priorityMatch = tokens.PopInt() 
                self.priorityMask = tokens.PopInt() 
            if tokens.Next() == "type":
                tokens.SkipOne()
                self.typeMatch = tokens.PopInt() 
                self.typeMask = tokens.PopInt() 
            idx = 0
            while tokens.Next() == "addr":
                tokens.SkipOne()
                self.addr[idx] = AddressMatch(tokens.Pop(), tokens.Pop(False),
                                              tokens.Pop(False))
                idx = idx + 1

    def Reset(self):
        "Reset the public attributes to their default values."
        self.enabled = False
        self.usesVlan = False
        self.vlanIdMatch = 0
        self.vlanIdMask = 0
        self.priorityMatch = 0
        self.priorityMask = 0
        self.typeMatch = 0
        self.typeMask = 0
        for addr in self.addr:
            addr.Reset()

    def Swap(self, other):
        """Swap the public attributes of the MatchLan object 'other' with the
        public attributes of this instance.
        """
        for k in self.__dict__.keys():
            if k == "addr":
                for idx in [0, 1]:
                    self.__dict__[k][idx].Swap(other.__dict__[k][idx])
            else:
                self.__dict__[k], other.__dict__[k] = \
                        other.__dict__[k], self.__dict__[k]

    def __str__(self):
        cmd = "lan"
        if self.usesVlan:
            cmd = "%s usesvlan" % (cmd, )
        cmd = "%s vlanid %d %d priority %d %d type %d %d" % (cmd,
                    self.vlanIdMatch, self.vlanIdMask,
                    self.priorityMatch, self.priorityMask,
                    self.typeMatch, self.typeMask)
        for addr in self.addr:
            cmd = "%s addr %s %s %s" % (cmd, addr.direc, addr.match, addr.mask)
        return cmd

#==============================================================================
class MatchIpV4:
    """Objects of this class contain IPv4 header match information.

    Public Attributes: 

    enabled: boolean
    port: array of AddressMatch
    addr: array of AddressMatch
    qosMatch: int
    qosMask: int
    flagsMatch: int
    flagsMask: int
    protoMatch: int
    protoMask: int
    """
    def __init__(self, tokens = None):
        self.port = [ ]
        self.addr = [ ]
        for idx in range(2):
            self.port.append(AddressMatch("notused", 0, 0))
            self.addr.append(AddressMatch("notused", "0.0.0.0","0.0.0.0"))
        self.Reset()
        if tokens:
            self.enabled = True
            idx = 0
            while tokens.Next() == "port":
                tokens.SkipOne()
                self.port[idx] = AddressMatch(tokens.Pop(),
                                              tokens.PopInt(),
                                              tokens.PopInt())
                idx = idx + 1
            idx = 0
            while tokens.Next() == "addr":
                tokens.SkipOne()
                self.addr[idx] = AddressMatch(tokens.Pop(),
                                              tokens.Pop(False),
                                              tokens.Pop(False))
                idx = idx + 1
            if tokens.Next() == "qos":
                tokens.SkipOne()
                self.qosMatch = tokens.PopInt()
                self.qosMask = tokens.PopInt()
            if tokens.Next() == "flags":
                tokens.SkipOne()
                self.flagsMatch = tokens.PopInt()
                self.flagsMask = tokens.PopInt()
            if tokens.Next() == "proto":
                tokens.SkipOne()
                self.protoMatch = tokens.PopInt()
                self.protoMask = tokens.PopInt()

    def Reset(self):
        "Reset the public attributes to their default values."
        self.enabled = False
        for port in self.port:
            port.Reset()
        for addr in self.addr:
            addr.Reset()
        self.qosMatch = 0
        self.qosMask = 0
        self.flagsMatch = 0
        self.flagsMask = 0
        self.protoMatch = 0
        self.protoMask = 0

    def Swap(self, other):
        """Swap the public attributes of the MatchIpV4 object 'other' with the
        public attributes of this instance.
        """
        for k in self.__dict__.keys():
            if k == "addr" or k == "port":
                for idx in [0, 1]:
                    self.__dict__[k][idx].Swap(other.__dict__[k][idx])
            else:
                self.__dict__[k], other.__dict__[k] = \
                        other.__dict__[k], self.__dict__[k]

    def __str__(self):
        cmd = "ipv4"
        for port in self.port:
            cmd = "%s port %s %d %d" % (cmd, port.direc, port.match, port.mask)
        for addr in self.addr:
            cmd = "%s addr %s %s %s" % (cmd, addr.direc, addr.match, addr.mask)
        if self.qosMask:
            cmd = "%s qos %d %d" % (cmd, self.qosMatch, self.qosMask)
        if self.flagsMask:
            cmd = "%s flags %d %d" % (cmd, self.flagsMatch, self.flagsMask)
        if self.protoMask:
            cmd = "%s proto %d %d" % (cmd, self.protoMatch, self.protoMask)
        return cmd

#==============================================================================
class MatchIpV6:
    """Objects of this class contain IPv6 header match information.

    Public Attributes: 

    enabled: boolean
    port: array of AddressMatch
    addr: array of AddressMatch
    classMatch: int
    classMask: int
    flowMatch: int
    flowMask: int
    protoMatch: int
    protoMask: int
    """
    def __init__(self, tokens = None):
        self.port = [ ]
        self.addr = [ ]
        for idx in range(2):
            self.port.append(AddressMatch("notused", 0, 0))
            self.addr.append(AddressMatch("notused","::0", "::0"))
        self.Reset()
        if tokens:
            self.enabled = True
            idx = 0
            while tokens.Next() == "port":
                tokens.SkipOne()
                self.port[idx] = AddressMatch(tokens.Pop(), tokens.PopInt(),
                                              tokens.PopInt())
                idx = idx + 1
            idx = 0
            while tokens.Next() == "addr":
                tokens.SkipOne()
                self.addr[idx] = AddressMatch(tokens.Pop(), tokens.Pop(False),
                                              tokens.Pop(False))
                idx = idx + 1
            if tokens.Next() == "class":
                tokens.SkipOne()
                self.classMatch = tokens.PopInt()
                self.classMask = tokens.PopInt()
            if tokens.Next() == "flow":
                tokens.SkipOne()
                self.flowMatch = tokens.PopInt()
                self.flowMask = tokens.PopInt()
            if tokens.Next() == "proto":
                tokens.SkipOne()
                self.protoMatch = tokens.PopInt()
                self.protoMask = tokens.PopInt()

    def Reset(self):
        "Reset the public attributes to their default values."
        self.enabled = False
        for port in self.port:
            port.Reset()
        for addr in self.addr:
            addr.Reset()
        self.classMatch = 0
        self.classMask = 0
        self.flowMatch = 0
        self.flowMask = 0
        self.protoMatch = 0
        self.protoMask = 0

    def Swap(self, other):
        """Swap the public attributes of the MatchIpV6 object 'other' with the
        public attributes of this instance.
        """
        for k in self.__dict__.keys():
            if k == "addr" or k == "port":
                for idx in [0, 1]:
                    self.__dict__[k][idx].Swap(other.__dict__[k][idx])
            else:
                self.__dict__[k], other.__dict__[k] = \
                        other.__dict__[k], self.__dict__[k]

    def __str__(self):
        cmd = "ipv6"
        for port in self.port:
            cmd = "%s port %s %d %d" % (cmd, port.direc, port.match, port.mask)
        for addr in self.addr:
            cmd = "%s addr %s %s %s" % (cmd, addr.direc, addr.match, addr.mask)
        if self.classMask:
            cmd = "%s class %d %d" % (cmd, self.classMatch, self.classMask)
        if self.flowMask:
            cmd = "%s flow %d %d" % (cmd, self.flowMatch, self.flowMask)
        if self.protoMask:
            cmd = "%s proto %d %d" % (cmd, self.protoMatch, self.protoMask)
        return cmd

#==============================================================================
class Trigger:
    """Objects of this class contain flow impairment triggering information.

    Public Attributes: 

    enabled: boolean
    flowTriggerType: string ("basic" or "tcpudp")
    impairAtCnt: int
    impairLen: int
    repeatCnt: int
    """
    def __init__(self, tokens = None):
        self.Reset()
        if tokens:
            self.enabled = True
            self.flowTriggerType = tokens.Pop()
            self.impairAtCnt = tokens.PopInt()
            self.impairLen = tokens.PopInt()
            self.repeatCnt = tokens.PopInt()

    def Reset(self):
        "Reset the public attributes to their default values."
        self.enabled = False
        self.flowTriggerType = "basic"
        self.impairAtCnt = 0
        self.impairLen = 0
        self.repeatCnt = 0

    def Swap(self, other):
        """Swap the public attributes of the Trigger object 'other' with the
        public attributes of this instance.
        """
        for k in self.__dict__.keys():
            self.__dict__[k], other.__dict__[k] = \
                    other.__dict__[k], self.__dict__[k]

    def __str__(self):
        return "trigger %s %d %d %d" % (self.flowTriggerType, self.impairAtCnt,
                                        self.impairLen, self.repeatCnt)

#==============================================================================
class MatchSet:
    """Objects of this class contain all the filter criteria you may set for an
    interface flow. An instance of this is returned by the GetMatch method and
    supplied to the SetMatch method of ServerControl.

    Public Attributes: 

    criteria: string ("vars", "none" or "all")
    allbut: boolean
    bytesEnabled: boolean
    bytes: array of AdHocRec, which have these public attributes:
        offsetType: string ("notinuse", "frame", "framedata", "ipdata", or
                            "tcpudpdata")
        offset: int
        match: int
        mask: int
    lan: MatchLan
    ipV4: MatchIpV4
    ipV6: MatchIpV6
    trigger: Trigger
    """
    def __init__(self, tokens = None):
        self.bytes = [ ]
        for idx in range(4):
            self.bytes.append(AdHocRec(offsetType = "notinuse",
                                       offset = 0, match = 0, mask = 0))
        self.lan = MatchLan()
        self.ipV4 = MatchIpV4()
        self.ipV6 = MatchIpV6()
        self.trigger = Trigger()
        self.Reset()
        if tokens:
            if tokens.Next() in ["all", "none"]:
                self.criteria = tokens.Pop()
            else:
                self.criteria = "vars"
                if tokens.Next() == "allbut":
                    tokens.SkipOne()
                    self.allbut = True
                if tokens.Next() == "bytes":
                    idx = 0
                    self.bytesEnabled = True
                    while tokens.Next() in ["notinuse", "frame", "framedata",
                                            "ipdata", "tcpudpdata"]:
                        self.bytes[idx] = AdHocRec(offsetType = tokens.Pop(),
                                                   offset = tokens.PopInt(),
                                                   match = tokens.PopInt(),
                                                   mask = tokens.PopInt())
                if tokens.Next() == "lan":
                    tokens.SkipOne()
                    self.lan = MatchLan(tokens)
                if tokens.Next() == "ipv4":
                    tokens.SkipOne()
                    self.ipV4 = MatchIpV4(tokens)
                elif tokens.Next() == "ipv6":
                    tokens.SkipOne()
                    self.ipV6 = MatchIpV6(tokens)
                if tokens.Next() == "trigger":
                    tokens.SkipOne()
                    self.trigger = Trigger(tokens)

    def Reset(self):
        "Reset the public attributes to their default values."
        self.criteria = "all"
        self.allbut = False
        self.bytesEnabled = False
        for bytes in self.bytes:
            bytes.Reset()
        self.lan.Reset()
        self.ipV4.Reset()
        self.ipV6.Reset()
        self.trigger.Reset()

    def Swap(self, other):
        """Swap the public attributes of the MatchSet object 'other' with the
        public attributes of this instance.
        """
        self.criteria, other.criteria = other.criteria, self.criteria
        self.allbut, other.allbut = other.allbut, self.allbut
        self.bytesEnabled, other.bytesEnabled = \
                other.bytesEnabled, self.bytesEnabled
        for idx in range(len(self.bytes)):
            self.bytes[idx].Swap(other.bytes[idx])
        self.lan.Swap(other.lan)
        self.ipV4.Swap(other.ipV4)
        self.ipV6.Swap(other.ipV6)
        self.trigger.Swap(other.trigger)

    def DoVersionUpdate(self):
        for attr, val in MatchSet().__dict__.items():
            if self.__dict__.has_key(attr):
                # Check for any updates in subclasses.
                var = self.__dict__[attr]
                if var.__class__.__dict__.has_key("DoVersionUpdate"):
                    var.DoVersionUpdate()
            else:
                # Handle attribute additions that don't exist in older versions.
                self.__dict__[attr] = val

    def __str__(self):
        if self.criteria == "vars":
            if self.allbut:
                cmd = "allbut"
            else:
                cmd = ""
            if self.bytesEnabled:
                cmd = "%s bytes" % (cmd, )
                for byte in self.bytes:
                    cmd = "%s %s %d %d %d" % (cmd, byte.offsetType, byte.offset,
                                              byte.match, byte.mask)
            if self.lan.enabled:
                cmd = "%s %s" % (cmd, self.lan)
            if self.ipV4.enabled:
                cmd = "%s %s" % (cmd, self.ipV4)
            if self.ipV6.enabled:
                cmd = "%s %s" % (cmd, self.ipV6)
            if self.trigger.enabled:
                cmd = "%s %s" % (cmd, self.trigger)
        else:
            # The value in self.criteria at this point must be "none" or "all".
            cmd = self.criteria
        return cmd

#==============================================================================
class PluginSet:
    """Objects of this class contain the information needed to configure
    a plugin. An instance of this class is returned by the GetPlugin method
    and supplied to the SetPlugin method of the ServerControl object.

    Public Attributes: 

    enabled: boolean
    init: boolean
    button: int
    checked: array of boolean
    num: array of int
    argv: array of string
    rspArgs: array of string
    """
    def __init__(self, tokens = None):
        self.Reset()
        if tokens:
            while tokens.Remain():
                token = tokens.Pop()
                if token == "off":
                    self.enabled = False
                elif token == "on":
                    self.enabled = True
                elif token == "init":
                    self.init = True
                elif token == "button":
                    self.button = tokens.PopInt()
                elif token == "check":
                    self.AddCheck(tokens.PopInt(), 1)
                elif token == "uncheck":
                    self.AddCheck(tokens.PopInt(), 0)
                elif token == "num":
                    tokens.SkipOne()
                    self.num.append(tokens.PopInt())
                elif token == "argv":
                    while tokens.Remain():
                        self.argv.append(tokens.Pop(False))

    def Reset(self):
        "Reset the public attributes to their default values."
        self.enabled = None
        self.init = False
        self.button = -1
        self.checked = []
        self.num = []
        self.argv = []
        self.rspArgs = []

    def AddCheck(self, index, isChecked):
        while len(self.checked) <= index:
            self.checked.append(False)
        self.checked[index] = isChecked

    def __str__(self):
        if self.enabled:
            cmd = "on "
        elif self.enabled == False:
            cmd = "off "
        else:
            cmd = ""
        if self.init:
            cmd += "init "
        for idx in range(len(self.checked)):
            if self.checked[idx]:
                cmd = "%s check %d" % (cmd, idx)
            else:
                cmd = "%s uncheck %d" % (cmd, idx)
        if self.button >= 0:
            cmd = "%s button %d" % (cmd, self.button)
        for idx in range(len(self.num)):
            cmd = "%s num %d %d" % (cmd, idx, self.num[idx])
        if self.argv:
            cmd = "%s argv" % (cmd, )
            for arg in self.argv:
                cmd = ' %s "%s"' % (cmd, arg)
        return cmd

#==============================================================================
class DropSet:
    """Objects of this class contain the information needed to configure
    drops. An instance of this class is in the ImpairSet object.

    Public Attributes: 

    enabled: boolean
    pct: float
    couplingEnabled: boolean
    couplingPct: float 
    couplingInt: int (microseconds)
    slew: boolean
    """
    def __init__(self, tokens = None):
        self.Reset()
        if tokens:
            self.enabled = True
            self.pct = tokens.PopFloat()
            if tokens.Next() == "couple":
                tokens.SkipOne()
                self.couplingEnabled = True
                self.couplingPct = tokens.PopFloat()
                self.couplingInt = tokens.PopInt()
                if tokens.Next() == "slew":
                    tokens.SkipOne()
                    self.slew = True

    def Reset(self):
        "Reset the public attributes to their default values."
        self.enabled = False
        self.pct = 0.0
        self.couplingEnabled = False
        self.couplingPct = 0.0
        self.couplingInt = 0
        self.slew = False

    def __str__(self):
        cmd = "drop %6.2f" % self.pct
        if self.couplingEnabled:
            cmd = "%s couple %6.2f %d" % (cmd, self.couplingPct,
                                          self.couplingInt)
            if self.slew:
                cmd += " slew"
        return cmd

#==============================================================================
class DupSet:
    """Objects of this class contain the information needed to configure
    dups. An instance of this class is in the ImpairSet object.

    Public Attributes: 

    enabled: boolean
    pct: float
    couplingEnabled: boolean
    couplingPct: float 
    """
    def __init__(self, tokens = None):
        self.Reset()
        if tokens:
            self.enabled = True
            self.pct = tokens.PopFloat()
            if tokens.Next() == "couple":
                tokens.SkipOne()
                self.couplingEnabled = True
                self.couplingPct = tokens.PopFloat()

    def Reset(self):
        "Reset the public attributes to their default values."
        self.enabled = False
        self.pct = 0.0
        self.couplingEnabled = False
        self.couplingPct = 0.0

    def __str__(self):
        cmd = "dup %6.2f" % self.pct
        if self.couplingEnabled:
            cmd = "%s couple %6.2f" % (cmd, self.couplingPct)
        return cmd

#==============================================================================
class DelaySet:
    """Objects of this class contain the information needed to configure
    delay. An instance of this class is in the ImpairSet object.

    Public Attributes: 

    enabled: boolean
    constDelay: int (microseconds)
    """
    def __init__(self, tokens = None):
        self.Reset()
        if tokens:
            self.enabled = True
            self.constDelay = tokens.PopInt()

    def Reset(self):
        "Reset the public attributes to their default values."
        self.enabled = False
        self.constDelay = 0

    def __str__(self):
        cmd = "delay %d" % (self.constDelay, )
        return cmd

#==============================================================================
class RateSet:
    """Objects of this class contain the information needed to configure
    rate limiting. An instance of this class is in the ImpairSet object.

    Public Attributes: 

    enabled: boolean
    rate: int
    minSize: int
    maxSize: int
    overhead: int
    qlen: int
    actualRate: int
    actualDelay: int (microseconds)
    """
    def __init__(self, tokens = None):
        self.Reset()
        if tokens:
            self.enabled = True
            self.rate = tokens.PopInt()
            self.minSize = tokens.PopInt()
            self.maxSize = tokens.PopInt()
            self.overhead = tokens.PopInt()
            self.qlen = tokens.PopInt()
            self.actualRate = tokens.PopInt()
            self.actualDelay = tokens.PopInt()

    def Reset(self):
        "Reset the public attributes to their default values."
        self.enabled = False
        self.rate = 0
        self.minSize = 8
        self.maxSize = 12000
        self.overhead = 160
        self.qlen = 36000
        self.actualRate = 0
        self.actualDelay = 0

    def __str__(self):
        return "rate %d %d %d %d %d %d %d" % (self.rate, self.minSize,
                                              self.maxSize, self.overhead,
                                              self.qlen, self.actualRate,
                                              self.actualDelay)

#==============================================================================
class Piece:
    """Objects of this class contain the information needed to configure one
    element of the pieces attribute in a JitterSet object.

    Public Attributes: 

    width: int
    pieceType: string ("fixed", "linear", "gaussian", or "none")
    fixed: int (microseconds)
    low: int (microseconds)
    high: int (microseconds)
    mean: int (microseconds)
    stdDev: int (microseconds)
    """
    def __init__(self, width = 0, pieceType = "none", fixed = 0, low = 0,
                 high = 0, mean = 0, stdDev = 0):
        self.width = width
        self.pieceType = pieceType
        self.fixed = fixed
        self.low = low
        self.high = high
        self.mean = mean
        self.stdDev = stdDev

#==============================================================================
class JitterSet:
    """Objects of this class contain the information needed to configure
    jitter. An instance of this class is in the ImpairSet object.

    Public Attributes: 

    enabled: boolean
    pct: float
    jitterType: string ("fixed", "linear", "gaussian", "piecewise", or "none")
    fixed: int (microseconds)
    pieces: array of Piece objects
    low: int (microseconds)
    high: int (microseconds)
    mean: int (microseconds)
    stdDev: int (microseconds)
    reorderOk: boolean
    couplingEnabled: boolean
    couplingPct: float
    couplingInt: int (microseconds)
    slew: boolean
    """
    def __init__(self, tokens = None):
        self.Reset()
        if tokens:
            self.enabled = True
            self.pct = tokens.PopFloat()
            self.jitterType = tokens.Pop()
            if self.jitterType == "fixed":
                self.fixed = tokens.PopInt()
            elif self.jitterType == "piecewise":
                idx = 0
                while tokens.Next() and tokens.Next() != "end":
                    width = tokens.PopInt()
                    pieceType = tokens.Pop()
                    if pieceType == "fixed":
                        self.pieces[idx] = Piece(width, pieceType,
                                                 fixed = tokens.PopInt())
                    elif pieceType == "linear":
                        self.pieces[idx] = Piece(width, pieceType,
                                                 low = tokens.PopInt(),
                                                 high = tokens.PopInt())
                    elif pieceType == "gaussian":
                        self.pieces[idx] = Piece(width, pieceType,
                                                 mean = tokens.PopInt(),
                                                 stdDev = tokens.PopInt())
                    idx = idx + 1
                if tokens.Next() == "end":
                    tokens.SkipOne()
            elif self.jitterType == "linear":
                self.low = tokens.PopInt()
                self.high = tokens.PopInt() 
            elif self.jitterType == "gaussian":
                self.mean = tokens.PopInt()
                self.stdDev = tokens.PopInt() 
            if tokens.Next() == "reorderok":
                tokens.SkipOne()
                self.reorderOk = True
            if tokens.Next() == "couple":
                tokens.SkipOne()
                self.couplingEnabled = True
                self.couplingPct = tokens.PopFloat()
                self.couplingInt = tokens.PopInt()
                if tokens.Next() == "slew":
                    tokens.SkipOne()
                    self.slew = True

    def Reset(self):
        "Reset the public attributes to their default values."
        self.enabled = False
        self.pct = 0.0
        self.jitterType = "fixed"
        self.fixed = 0
        self.pieces = [ ]
        for idx in range(8):
            self.pieces.append(Piece())
        self.low = 0
        self.high = 0
        self.mean = 0
        self.stdDev = 0
        self.reorderOk = False
        self.couplingEnabled = False
        self.couplingPct = 0.0
        self.couplingInt = 0
        self.slew = False

    def __str__(self):
        self.jitterType = self.jitterType.lower()
        cmd = "jitter %6.2f" % self.pct
        if self.jitterType == "fixed":
            cmd = "%s fixed %d" % (cmd, self.fixed)
        elif self.jitterType == "piecewise":
            cmd += " piecewise"
            for piece in self.pieces:
                if piece.pieceType == "fixed":
                    cmd = "%s %d fixed %d" % (cmd, piece.width, piece.fixed)
                elif piece.pieceType == "linear":
                    cmd = "%s %d linear %d %d" % (cmd, piece.width,
                                                  piece.low, piece.high)
                elif piece.pieceType == "gaussian":
                    cmd = "%s %d gaussian %d %d" % (cmd, piece.width,
                                                    piece.mean, piece.stdDev)
            cmd += " end"
        elif self.jitterType == "linear":
            cmd = "%s linear %d %d" % (cmd, self.low, self.high)
        elif self.jitterType == "gaussian":
            cmd = "%s gaussian %d %d" % (cmd, self.mean, self.stdDev)

        if self.reorderOk:
            cmd += " reorderok"
        if self.couplingEnabled:
            cmd = "%s couple %6.2f %d" % (cmd, self.couplingPct,
                                          self.couplingInt)
            if self.slew:
                cmd += " slew"
        return cmd

#==============================================================================
class CorruptSet:
    """Objects of this class contain the information needed to configure
    bit corruption. An instance of this class is in the ImpairSet object.

    Public Attributes: 

    enabled: boolean
    payload: boolean
    bitErrorProbability: float
    """
    def __init__(self, tokens = None):
        self.Reset()
        if tokens:
            self.enabled = True
            if tokens.Next() == "payload":
                tokens.SkipOne()
                self.payload = True
            self.bitErrorProbability = tokens.PopFloat()

    def Reset(self):
        "Reset the public attributes to their default values."
        self.enabled = False
        self.payload = False
        self.bitErrorProbability = 0.0

    def __str__(self):
        return "corrupt%s %g" % (["", " payload"][self.payload],
                                 self.bitErrorProbability)

#==============================================================================
class AlterSet:
    """Objects of this class contain the information needed to configure
    packet alterations. An instance of this class is in the ImpairSet object.

    Public Attributes: 

    enabled: boolean
    which: string ("before", "after", or "")
    args: [string, ...]
    """
    def __init__(self, tokens = None):
        self.Reset()
        if tokens:
            self.enabled = True
            if tokens.Next() in ["before", "after"]:
                self.which = tokens.Pop()
            while tokens.Remain():
                self.args.append(tokens.Pop(False))

    def Reset(self):
        "Reset the public attributes to their default values."
        self.enabled = False
        self.which = "" # "before", "after", or ""
        self.origTxt = "" # Original text - may include newlines.
        # List of commands and arguments: [string, ...]
        self.args = [ ]

    def __str__(self):
        return "alter %s %s" % (self.which, " ".join(self.args))

#==============================================================================
class ImpairSet:
    """Objects of this class contain all the impairment criteria you may set
    for an interface flow. An instance of this is returned by the GetImpair
    method and supplied to the SetImpair method of ServerControl.

    Public Attributes: 

    drop: DropSet
    dup: DupSet
    delay: DelaySet
    rate: RateSet
    jitter: JitterSet
    corrupt: CorruptSet
    """
    def __init__(self, tokens = None):
        self.Reset()
        if tokens:
            if tokens.Next() == "dropall":
                tokens.Pop()
                self.drop.enabled = True
                self.drop.pct = 100.0
            elif tokens.Next()  == "forwardall":
                tokens.Pop()
            else:
                if tokens.Next() == "drop":
                    tokens.SkipOne()
                    self.drop = DropSet(tokens)
                if tokens.Next() == "dup":
                    tokens.SkipOne()
                    self.dup = DupSet(tokens)
                if tokens.Next() == "delay":
                    tokens.SkipOne()
                    self.delay = DelaySet(tokens)
                if tokens.Next() == "rate":
                    tokens.SkipOne()
                    self.rate = RateSet(tokens)
                if tokens.Next() == "jitter":
                    tokens.SkipOne()
                    self.jitter = JitterSet(tokens)
                if tokens.Next() == "corrupt":
                    tokens.SkipOne()
                    self.corrupt = CorruptSet(tokens)
                if tokens.Next() == "alter":
                    tokens.SkipOne()
                    self.alter = AlterSet(tokens)

    def Reset(self):
        "Reset the public attributes to their default values."
        self.drop = DropSet()
        self.dup = DupSet()
        self.delay = DelaySet()
        self.rate = RateSet()
        self.jitter = JitterSet()
        self.corrupt = CorruptSet()
        self.alter = AlterSet()

    def __str__(self):
        cmd = ""
        if self.drop.enabled:
            cmd = "%s %s" % (cmd, self.drop)
        if self.dup.enabled:
            cmd = "%s %s" % (cmd, self.dup)
        if self.delay.enabled:
            cmd = "%s %s" % (cmd, self.delay)
        if self.rate.enabled:
            cmd = "%s %s" % (cmd, self.rate)
        if self.jitter.enabled:
            cmd = "%s %s" % (cmd, self.jitter)
        if self.corrupt.enabled:
            cmd = "%s %s" % (cmd, self.corrupt)
        if self.alter.enabled:
            cmd = "%s %s" % (cmd, self.alter)
        return cmd

#==============================================================================
class LogFlags:
    """Objects of this class contain all the logging criteria you may set
    for an interface flow. An instance of this is returned by the GetLogging
    method and supplied to the SetLogging method of ServerControl.

    Public Attributes: 

    inp: boolean
    out: boolean
    details_in: boolean
    details_out: boolean
    settings: boolean
    fate: boolean
    stats: boolean
    """
    def __init__(self, tokens = None):
        self.Reset()
        if tokens:
            while tokens.Remain():
                flagName = tokens.Pop()
                if flagName == "clear":
                    self.Reset()
                else:
                    if flagName == "in":
                        flagName = "inp"
                    self.__dict__[flagName] = True

    def Reset(self):
        "Reset the public attributes to their default values."
        self.inp = False
        self.out = False
        self.details_in = False
        self.details_out = False
        self.settings = False
        self.fate = False
        self.stats = False

    def Swap(self, other):
        """Swap the public attributes of the LogFlags object 'other' with the
        public attributes of this instance.
        """
        self.inp, other.inp = other.inp, self.inp
        self.out, other.out = other.out, self.out
        self.details_in, other.details_in = other.details_in, self.details_in
        self.details_out, other.details_out = \
                                            other.details_out, self.details_out
        self.settings, other.settings = other.settings, self.settings
        self.fate, other.fate = other.fate, self.fate
        self.stats, other.stats = other.stats, self.stats

    def __str__(self):
        cmd = "clear"
        for flagName, enabled in self.__dict__.items():
            if enabled:
                if flagName.lower() == "inp":
                    flagName = "in"
                cmd = "%s %s" % (cmd, flagName)
        return cmd

#==============================================================================
class StartArgs:
    """Objects of this class contain as attributes the command line argument
    values that control the initialization of the stdiserver process. An
    instance may be passed to the RestartServer method of ServerControl.
    Public attributes are defined by the argDesc class variable.
    """
    argDesc = \
        {
            "-b": (int,  "bufferCount",      524288, 8),
            "-B": (int,  "bufferSize",       1536, 80),
            "-m": (int,  "mtu",              1500, 1400),
            "-L": (str,  "lusPath",          "", ""),
            "-O": (str,  "lusOptions",       "", ""),
            "-l": (str,  "port0Name",        "", ""),
            "-h": (str,  "port1Name",        "", ""),
            "-i": (int,  "itemCount",        8, 1),
            "-P": (int,  "remoteApiPortNum", 7021, 7021),
            "-g": (str,  "logfileName",      "", ""),
            "-T": (str,  "tapNum",           "", ""),
            "-t": (int,  "traceMask",        0, 0),
            "--bpf0": (str, "bpf0",         "", ""),
            "--bpf1": (str, "bpf1",         "", ""),
            "-u": (None, "noRealtimeFlag",   False, False),
            "-M": (None, "skipLockCheck",    False, False),
            "-p": (None, "pageableFlag",     False, False)
            #, # The following are commented out as a temporary
            #  # measure to avoid sending bad affinity settings
            #  # during a restart.  I'm leaving the code here
            #  # to make it easier to resurrect should we want to
            #  # do so
            #"--rx_affinity0": (int, "rx_affinity0", 1, 0),
            #"--rx_affinity1": (int, "rx_affinity1", 1, 0),
            #"--tx_affinity0": (int, "tx_affinity0", 2, 0),
            #"--tx_affinity1": (int, "tx_affinity1", 2, 0),
            #"--logger_affinity":(int, "logger_affinity", 0, 0),
            #"--port_affinity0":(int, "port_affinity0", 0, 0),
            #"--port_affinity1":(int, "port_affinity1", 0, 0)
        }

    def __init__(self, tokens = None):
        self.Reset()
        if tokens:
            while tokens.Remain():
                argDesc = self.argDesc.get(tokens.Pop(False), None)
                if argDesc:
                    cast, vname, default, minVal = argDesc
                    if cast:
                        if cast == int:
                            self.__dict__[vname] = int(tokens.Pop(), 0)
                        else:
                            self.__dict__[vname] = cast(tokens.Pop())
                    else:
                        self.__dict__[vname] = not default

    def Reset(self):
        "Reset the public attributes to their default values."
        for cast, vname, default, minVal in self.argDesc.values():
            self.__dict__[vname] = default
            self.__dict__[vname + "_min"] = minVal
        # These attributes aren't delivered as command line arguments, so
        # have to be treated differently.
        self.tapV4Addr = ""
        self.tapV6Addr = ""

    def SetToMins(self):
        for cast, vname, default, minVal in self.argDesc.values():
            self.__dict__[vname] = minVal
        self.tapV4Addr = ""
        self.tapV6Addr = ""

    def __str__(self):
        cmd = ""
        for opt, (cast, vname, default, minVal) in self.argDesc.items():
            val = self.__dict__[vname]
            if cast == str:
                if val != "":
                    cmd = '%s "%s" "%s"' % (cmd, opt, val)
            elif cast == int:
                val = int(val)
                if (val >= 0):
                    cmd = '%s "%s" %d' % (cmd, opt, val)
            elif val:
                cmd = '%s "%s"' % (cmd, opt)
        return cmd

#==============================================================================
class AllStats:
    """An AllStats object is returned by the GetAllStats method of the
    ServerControl class.

    Public Attributes:

    timestamp: integer. The time the stats were grabbed, in microseconds since
                        the beginning of the Unix epoch. Probably accurate to
                        no more than a few hundred microseconds.
    stats: A dictionary where each key names a FLOW statistic and each value
           is a list (ordered by flow number) of tuples, where each tuple
           contains the statistic's value for interface port 0 and interface
           port 1.  Schematically the dictionary takes on this form:
           {statisticID:
                [(flow0_if0stat, flow0_if1stat), (flow1_if0stat, flow1_if1stat),
                 ... ], ...}
    itfStats: A dictionary where each key names an INTERFACE statistic and the
              value is a tuple of two numbers, the first being for interface
              port 0 and the second for interface port 1. Schematically the
              dictionary takes on this form:
              {statisticID: (if0stat, if1stat), ...}
    """
    def __init__(self, tokens = None):
        if tokens:
            self.timestamp = tokens.PopInt()
            self.stats = { }
            self.itfStats = { }
            while tokens.Remain() and tokens.Next(False) != "End":
                statName = tokens.Pop(False)
                self.stats[statName] = [ ]
                while tokens.Remain() and tokens.Next().isdigit():
                    if0stat = tokens.PopInt()
                    if1stat = tokens.PopInt()
                    self.stats[statName].append((if0stat, if1stat))
            if tokens.Next(False) == "End":
                tokens.Pop() # Consume End token.
                while tokens.Remain():
                    statName = tokens.Pop(False)
                    if0stat = tokens.PopInt()
                    if1stat = tokens.PopInt()
                    self.itfStats[statName] = (if0stat, if1stat)
        else:
            self.Reset()

    def Reset(self):
        "Reset the public attributes to their default values."
        self.timestamp = 0
        self.stats = { }
        self.itfStats = { }

#==============================================================================
class Stats:
    """A Stats object is returned by the GetStats method of the ServerControl
    class.

    Public Attributes:

    These are derived from the returned IDs.
    """
    def __init__(self, tokens = None):
        if tokens:
            while tokens.Remain():
                statName = tokens.Pop()
                statVal = tokens.PopInt()
                self.__dict__[statName] = statVal
        else:
            self.Reset()

    def Reset(self):
        "Reset the public attributes to their default values."
        for statName in self.__dict__.keys():
            self.__dict__[statName] = 0

#==============================================================================
class TriggerStats:
    """A TriggerStats object is returned by the GetTriggerStats method of the
    ServerControl class.

    Public Attributes:

    protoCnts: [(protoIDstring, flowsTriggered, flowsStarted, flowsCompleted) ]
    """
    protos = ["Basic", "TCPv4", "UDPv4", "TCPv6", "UDPv6"]
    def __init__(self, tokens = None):
        self.Reset()
        if tokens:
            for idx in range(len(self.protos)):
                tokens.Pop()
                self.protoCnts[idx] = ((self.protos[idx], tokens.PopInt(),
                                        tokens.PopInt(), tokens.PopInt()))

    def Reset(self):
        "Reset the public attributes to their default values."
        self.protoCnts = []
        for proto in self.protos:
            self.protoCnts.append((proto, 0, 0, 0))

#==============================================================================
class IfInfo:
    """Objects of this class contain the cable connection state, negotiated
    speed, and duplex of the named interface.

    Public Attributes:

    name: string
    state: "Connected" or "Unconnected"
    speedStr: string ending in "Mbps"
    speed: int
    duplex: "Full", "Half", or "None"
    """
    def __init__(self, tokens = None):
        if tokens:
            self.name = tokens.Pop()
            self.state = tokens.Pop().capitalize()
            tokens.Pop()
            self.speedStr = tokens.Pop(False)
            self.speed = int(self.speedStr[:-4])*1000000
            tokens.Pop()
            self.duplex = tokens.Pop().capitalize()
        else:
            self.Reset()

    def Reset(self):
        "Reset the public attributes to their default values."
        self.name = "Unknown"
        self.state = "Unconnected"
        self.speedStr = "0Mbps"
        self.speed = 0
        self.duplex = "Half"

#==============================================================================
class ServerControl:
    """A ServerControl object is used to connect to a local or remote stdiserver
    process and then send and receive messages with it. After or during
    creation the host and port attributes are set to the desired target.
    The ServerConnect method is then used to connect to the target server.
    If any communications problems arise or an error response returned by
    the server, then a ServerError exception object is raised.

    Public Attributes:

    host: string
    port: int
    """
    def __init__(self, host = "127.0.0.1", port = 7021):
        self.host = host
        self.port = port
        self.sock = None

    def ServerConnect(self):
        """Attempt to connect to stdiserver at the previously set host and port.
        Raises ServerError exception if unable to connect.
        Returns nothing if successful.
        """
        if self.IsNotConnected():
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                # Notes:
                # If attempting to connect to a dead IP address the
                # following will cause the connect to raise a timeout
                # exception after 3 seconds.
                # If the IP address is alive then the connect will immediately
                # return on success or immediately throw a connection refused
                # exception.
                self.sock.settimeout(3.0)
                self.sock.connect((self.host, self.port))
            except Exception, err:
                self.sock = None
                raise ServerError(str(err))
            self.GetCmdRsp("useversion 2")

    def ServerDisconnect(self):
        """Disconnect from the server - does nothing if not already connected.
        Returns nothing.
        """
        if self.IsConnected():
            self.sock.close()
            self.sock = None

    def Reconnect(self):
        """Disconnects from server and attempts to reconnect, trying 5 times
        every second until either it is succesful or after 3 seconds it
        raises a ServerError exception.
        Returns nothing.
        """
        self.ServerDisconnect()
        # Notes:
        # If attempting to connect to a dead IP address the following will
        # cause the connect to raise a timeout exception after 3 seconds.
        # If the IP address is alive then the connect will immediately return
        # on success or immediate connection refused exceptions will be caught
        # and the connection will be attempted 5 times a second for 3 seconds.
        # After 3 seconds of failed attempts a connection refused exception
        # will be raised to the caller.
        startTime = time.time()
        while True:
            try:
                ServerControl.ServerConnect(self)
                return
            except Exception, err:
                if time.time() - startTime >= 3.0:
                    raise ServerError(str(err))
                time.sleep(0.2)

    def IsConnected(self):
        "Returns True only if connected to server."
        return self.sock != None

    def IsNotConnected(self):
        "Returns True only if not connected to server."
        return self.sock == None

    def GetPluginMeta(self):
        """Returns a PluginMetaInfo object containing plugin meta information.
        Raises ServerError exception if no plugin or no connection to server.
        """
        return PluginMetaInfo(self.GetCmdRsp("getpluginmeta"))

    def GetPlugin(self, flowNum, interface):
        """Returns a PluginSet object for the given flowNum and interface, which
        contains the current values of the plugin's configuration attributes.
        Raises ServerError exception if no plugin or no connection to server.
        """
        return PluginSet(self.GetCmdRsp("getplugin %d %d" %
                                        (flowNum, interface)))

    def SetPlugin(self, flowNum, interface, pluginSet):
        """Sets the plugin's configuration attributes for the given flowNum and
        interface using the contents of the pluginSet object.
        Returns optional information strings from plugin on success.
        Raises ServerError exception if bad values or no connection to server.
        """
        rspTokens = self.GetCmdRsp("setplugin %d %d %s" %
                              (flowNum, interface, pluginSet))
        pluginSet.rspArgs = []
        while rspTokens.Remain():
            pluginSet.rspArgs.append(rspTokens.Pop(False))
        return pluginSet.rspArgs

    def SendOnlyPluginArgv(self, argv, flowNum = 0, interface = 0):
        """Performs a SetPlugin using only the arguments specified by the
        argv list. This is primarily used to perform operations on plugins
        that are independent of the flow or interface. Such as getting or
        setting plugin meta information. It returns a PluginSet object
        containing any response information or None if no connection was
        established. Raises ServerError exception if bad values or connection
        to server is lost during the transaction.
        """
        pluginSet = PluginSet()
        pluginSet.argv = argv
        self.SetPlugin(flowNum, interface, pluginSet)
        return pluginSet

    def ResetPlugin(self, flowNum, interface):
        """Resets the plugin's configuration attributes to their default values
        for the given flowNum and interface.
        Returns nothing on success.
        Raises ServerError exception if bad values or no connection to server.
        """
        self.GetCmdRsp("resetplugin %d %d" % (flowNum, interface))

    def GetNumFlows(self):
        """Returns the number of flows currently available. This is equal to the
        value of itemCount/2, where itemCount is the attribute from the
        StartArgs class.
        Raises ServerError exception if no connection to server.
        """
        return self.GetCmdRsp("getnumflows").PopInt()

    def ClearAllFlows(self):
        """Resets statistic counts to zero and match criteria, impairments, and
        plugin field values (if any) to their initial values for all flows on
        both interfaces.
        Returns nothing on success.
        Raises ServerError exception if bad values or no connection to server.
        """
        self.GetCmdRsp("clearallflows")

    def SetMatch(self, flowNum, interface, matchSet, zeroStats):
        """Set the match criteria for the given flowNum and interface to the
        state specified in the matchSet object. If boolean zeroStats argument is
        True, then the packet statistics counts for that flowNum and interface
        are also zeroed.
        Returns nothing on success.
        Raises ServerError exception if bad values or no connection to server.
        """
        self.GetCmdRsp("setmatch %d %d %s %s" %
                       (flowNum, interface, ["", "zerostats"][zeroStats],
                        matchSet))

    def SetImpair(self, flowNum, interface, impairSet, zeroStats):
        """Set the impairment criteria for the given flowNum and interface to
        the state specified in the impairSet object. If boolean zeroStats
        argument is True, then the packet statistics counts for that flowNum
        and interface are also zeroed.
        Returns nothing on success.
        Raises ServerError exception if bad values or no connection to server.
        """
        self.GetCmdRsp("setimpair %d %d %s %s" %
                       (flowNum, interface, ["", "zerostats"][zeroStats],
                        impairSet))

    def ClearQueue(self, flowNum, interface):
        """Removes all packets on the delay and rate limiter queue for the
        given flowNum and interface.
        Returns nothing on success.
        Raises ServerError exception if bad values or no connection to server.
        """
        self.GetCmdRsp("clearqueue %d %d" % (flowNum, interface))

    def ZeroStats(self, flowNum, interface):
        """Zeroes the packet statistics counts for the given flowNum and
        interface.
        Returns nothing on success.
        Raises ServerError exception if bad values or no connection to server.
        """
        self.GetCmdRsp("zerostats %d %d" % (flowNum, interface))

    def GetMatch(self, flowNum, interface):
        """Returns a MatchSet object that contains the match criteria state of
        the given flowNum and interface.
        Raises ServerError exception if bad values or no connection to server.
        """
        return MatchSet(self.GetCmdRsp("getmatch %d %d" % (flowNum, interface)))

    def GetImpair(self, flowNum, interface):
        """Returns an ImpairSet object that contains the impairment state of the
        given flowNum and interface.
        Raises ServerError exception if bad values or no connection to server.
        """
        return ImpairSet(self.GetCmdRsp("getimpair %d %d" %
                                        (flowNum, interface)))

    def SetLogging(self, flowNum, interface, logFlags):
        """Sets the information to be logged according to the logFlags argument
        for the given flowNum and interface.
        Returns nothing on success.
        Raises ServerError exception if bad values or no connection to server.
        """
        self.GetCmdRsp("setlogging %d %d %s" % (flowNum, interface, logFlags))

    def GetLogging(self, flowNum, interface):
        """Returns a LogFlags object that contains the logging criteria for the
        given flowNum and interface.
        Raises ServerError exception if bad values or no connection to server.
        """
        return LogFlags(self.GetCmdRsp("getlogging %d %d" %
                                       (flowNum, interface)))

    def GetStatInfo(self):
        """Returns an array of tuples, where each tuple contains three
        string elements: ID, Label, and Info.
        Raises ServerError exception if no connection to server.
        """
        toks = self.GetCmdRsp("getstats info")
        statInfo = [ ]
        while toks.Remain():
            entry = (toks.Pop(False), toks.Pop(False), toks.Pop(False))
            statInfo.append(entry)
        return statInfo

    def GetAllStats(self):
        """Returns an AllStats object that contains the statistical packet
        counts for all the flow interfaces.
        Raises ServerError exception if no connection to server.
        """
        return AllStats(self.GetCmdRsp("getstats all"))

    def GetStats(self, flowNum, interface):
        """Returns a Stats object that contains the statistical packet counts
        for the given flowNum and interface.
        Raises ServerError exception if bad values or no connection to server.
        """
        return Stats(self.GetCmdRsp("getstats %d %d all" %
                                    (flowNum, interface)))

    def GetTriggerStats(self, flowNum, interface):
        """Returns a TriggerStats object that contains counts for the number
        of triggered flows and the number in each trigger state.
        Raises ServerError exception if bad values or no connection to server.
        """
        return TriggerStats(self.GetCmdRsp("getstats %d %d trigger" %
                                           (flowNum, interface)))

    def SetLogFile(self, logfile):
        """Sets the file that packet fate logging should be sent to. The
        logfile argument is a string value that must contain the full path
        of the destination file. If the string is zero length then that
        effectively turns off fate logging.
        Raises ServerError exception if there are any problems creating or
        opening an existing file or there is no connection to a server.
        """
        self.GetCmdRsp("setlogfile %s" % (logfile,))

    def GetLogFile(self):
        """Returns a string that contains the path name of the file receiving
        the log information specified by the LogFlags objects set on each
        interface flow.
        Raises ServerError exception if no connection to server.
        """
        return self.GetCmdRsp("getlogfile").Pop(False)

    def GetIfInfo(self, interface):
        """Returns an IfInfo object containing information on the interface.
        Raises ServerError exception if no connection to server.
        """
        return IfInfo(self.GetCmdRsp("getifinfo %d" % interface))

    def GetStartArgs(self):
        """Returns a StartArgs object that contains the command line parameter
        values being used by the stdiserver process.
        Raises ServerError exception if no connection to server.
        """
        return StartArgs(self.GetCmdRsp("getstartargs"))

    def SetMaxtap(self, req, param, oldVal = ""):
        """Allows setting of the maxtap address and netmask for IPv4 and IPv6
        as well as adding and replacement of static routes. Valid parameters
        for req, param, and oldVal respectively are:
        addrnet4 <IPv4 address/netmask>
        addrnet6 <IPv6 address/netmask>
        route4 <IPv4 address to add> [<IPv4 address to delete>]
        route6 <IPv6 address to add> [<IPv6 address to delete>]
        """
        self.GetCmdRsp('setmaxtap %s "%s" "%s"' % (req, param, oldVal))

    def SetMaxtapRoutes(self, startArgs, v4Addr, v6Addr,
                        tapV4Addr, tapV6Addr, pluginRecord):
        if "maxtap" in [startArgs.port0Name, startArgs.port1Name]:
            try:
                if tapV4Addr or startArgs.tapV4Addr:
                    self.SetMaxtap("addrnet4", tapV4Addr, startArgs.tapV4Addr)
                startArgs.tapV4Addr = tapV4Addr
                if tapV6Addr or startArgs.tapV6Addr:
                    self.SetMaxtap("addrnet6", tapV6Addr, startArgs.tapV6Addr)
                startArgs.tapV6Addr = tapV6Addr
                if v4Addr or pluginRecord.v4Addr:
                    self.SetMaxtap("route4", v4Addr, pluginRecord.v4Addr)
                pluginRecord.v4Addr = v4Addr
                if v6Addr or pluginRecord.v6Addr:
                    self.SetMaxtap("route6", v6Addr, pluginRecord.v6Addr)
                pluginRecord.v6Addr = v6Addr
            except (ServerError, ), err:
                errMsg = str(err)
                if "File exists" not in errMsg:
                    raise

    def RestartServer(self, startArgs):
        """Attempts to restart the stdiserver using the command line parameter
        values in the startArgs object.
        Always returns nothing, whether successful or not and whether connected
        or not.
        May raise exception if unable to reconnect or problems with maxtap
        address/netmask values.
        """
        try:
            self.GetCmdRsp("restartserver %s" % startArgs)
        except:
            pass
        time.sleep(2)	# Give some time for the old process/threads to clean up
        		# and die and for the new process to start.
        self.Reconnect()
        if startArgs.tapNum:
            if startArgs.tapV4Addr:
                self.SetMaxtap("addrnet4", startArgs.tapV4Addr)
            if startArgs.tapV6Addr:
                self.SetMaxtap("addrnet6", startArgs.tapV6Addr)

    def GetCmdRsp(self, cmd):
        """Internal use method that issues the string in cmd to the server and
        returns a TokenList object that contains any response.
        Raises ServerError exception if any errors found in the cmd string or
        there is no connection to server.
        """
        # First send command.
        if debugFlags & 1:
            print cmd
        if self.IsNotConnected():
            raise ServerError("Fail: Not connected to server.")
        try:
            self.sock.send(cmd + "\n")
        except:
            self.ServerDisconnect()
            raise ServerError("Fail: Send %s %s" %
                              (cmd, str(sys.exc_info()[1])))

        # Prep for response lines.
        tokens = TokenList()
        data = ""
        notEol = True
        while notEol:
            try:
                data = self.sock.recv(8192)
            except:
                if "Interrupted system call" in str(sys.exc_info()[1]):
                    continue
                else:
                    self.ServerDisconnect()
                    raise ServerError("Fail: Recv %s %s" %
                                      (cmd, str(sys.exc_info()[1])))
            if len(data) == 0:
                self.ServerDisconnect()
                raise ServerError("Fail: Server closed connection.")
            if debugFlags & 2:
                print data,
            notEol = tokens.AddText(data)
        result = tokens.Pop(False)
        if result == "Fail":
            reason = tokens.Pop(False)
            raise ServerError(result + ": " + reason)
        return tokens

