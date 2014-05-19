#! /usr/bin/python
# (C) 2012 Biorobotics Lab and Nonolith Labs
# Written by Ian Daniher
# Licensed under the terms of the GNU GPLv3+

import usb
import re
import itertools
import atexit

# This driver was originally written by takktile inc, see note above
# This version has had LCM publishing added for drc use
import lcm
import takktile

_unTwos = lambda x, bitlen: x-(1<<bitlen) if (x&(1<<(bitlen-1))) else x
_chunk = lambda l, x: [l[i:i+x] for i in xrange(0, len(l), x)]
_flatten = lambda l: list(itertools.chain(*[[x] if type(x) not in [list] else x for x in l]))

class TakkTile:

    # get I2C address for a given cell from varying other references
    _getTinyAddressFromRowColumn = lambda self, row, column: (((row)&0x0F) << 4 | (column&0x07) << 1)
    _getTinyAddressFromIndex = lambda self, index: (((index/5)&0x0F) << 4 | ((index%5)&0x07) << 1)

    def __init__(self, arrayID = 0):
        self.sampling = False
        # search for a USB device with the proper VID/PID combo
        self.dev = usb.core.find(idVendor=0x59e3, idProduct=0x74C7)
        if self.dev == None:
            print("Can't find TakkTile USB interface!")
            quit()
        self.arrayID = arrayID
        # populates bitmap of live sensors
        self.alive = self.getAlive()
        # calibrationCoefficients is a dictionary mapping cell index to a dictionary of calibration variables
        self.calibrationCoefficients = dict(map(self.getCalibrationCoefficients, self.alive))
        # populate self.UID with vendor request to get the xmega's serialNumber
        self.UID = self.dev.ctrl_transfer(0x80, usb.REQ_GET_DESCRIPTOR,
            (usb.util.DESC_TYPE_STRING << 8) | self.dev.iSerialNumber, 0, 255)[2::].tostring().decode('utf-16')
        atexit.register(self.stopSampling)
        # save data history for wrapping issue
        self.pressureHistory = []

    def getAlive(self):
        """ Return an array containing the cell number of all alive cells. """
        data = self.dev.ctrl_transfer(0x40|0x80, 0x5C, 0, 0, 40)
        return [i[0] for i in enumerate(data) if i[1] == 255]

    def getCalibrationCoefficients(self, index):
        """ This function implements the compensation & calibration coefficient calculations from page 15 of AN3785. """
        # get raw calibration data from a specified location
        cd = self.getCalibrationData(index) + [0, 0, 0, 0]
        cc = {"a0":0, "b1":0, "b2":0, "c12":0, "c11":0, "c22":0}
        # cell not alive
        if max(cd) != 0:
            # undo Two's complement if applicable, pack into proper bit width
            cc["a0"] = _unTwos(((cd[0] << 8) | cd[1]), 16)
            cc["b1"] = _unTwos(((cd[2] << 8) | cd[3]), 16)
            cc["b2"] = _unTwos(((cd[4] << 8) | cd[5]), 16)
            cc["c12"] = _unTwos(((cd[6] << 6) | (cd[7] >> 2)), 14)
            cc["c11"] = _unTwos(((cd[8] << 3) | (cd[9] >> 5)), 11)
            cc["c22"] = _unTwos(((cd[10] << 3) | (cd[11] >> 5)), 11)
            # divide by float(1 << (fractionalBits + zeroPad)) to handle weirdness
            cc["a0"] /= float(1 << 3)
            cc["b1"] /= float(1 << 13)
            cc["b2"] /= float(1 << 14)
            cc["c12"] /= float(1 << 22)
            cc["c11"] /= float(1 << 21)
            cc["c22"] /= float(1 << 25)
        return (index, cc)

    def getDataRaw(self):
        """Query the TakkTile USB interface for the pressure and temperature samples from a specified row of sensors.."""
        if self.sampling:
            data = self.dev.read(0x81, 720, 0, 100)
            try:
                assert len(data) % 4 == 0
            except:
                print "got: ", len(data)
                print "expected:", len(self.alive)*4
                print data
                raise Exception("data read from USB endpoint is not correct length")
        else:
            # not sampling, so trigger a single shot conversion
            data = self.dev.ctrl_transfer(0x40|0x80, 0x6D, 0xFF, 0xFF, 0)
            data = self.dev.ctrl_transfer(0x40|0x80, 0x6D, 0x0F, 0x0F, 0)
            for i in range(8):
                data += tact.dev.ctrl_transfer(0x40|0x80, 0x6D, 0, 0, 64)
        data = _chunk(data, 4)
        # temperature is contained in the last two bytes of each four byte chunk, pressure in the first two
        # each ten bit number is encoded in two bytes, MSB first, zero padded / left alligned
        temperature = [datum[3] >> 6| datum[2] << 2 for datum in data] #if datum.count(0) != 4]
        pressure = [datum[1] >> 6| datum[0] << 2 for datum in data] #if datum.count(0) != 4]
        pressure = map(abs, pressure)

        # dealing with wrapping aspect of the most significant bit
        # 1) calculating delta (value_current-value_hostory);
        #     where delta_up is if the value_current is positive
        #     and delta_down is if the value_current is negative (i.e. significant bit indicates negative values)
        # 2) it is assumed that the delta between subsequental readings will be smaller rather than bigger
        #     therefore the "smaller" delta between delta_up and delta_down is selected
        #       and then it is added / substracted to the value_history

        if (len(self.pressureHistory)>0):
            i=0
            for p_current in pressure:
                p_history=self.pressureHistory[i]
                delta_up=p_current-p_history
                delta_down=p_history-(p_current-1024)
                if (delta_up<delta_down):
                    pressure[i]=p_history+delta_up
                else:
                    pressure[i]=p_history-delta_down
                i+=1

        self.pressureHistory=pressure
        temperature = map(abs, temperature)
        # return a dictionary mapping sensor indexes to a tuple containing (pressure, temperature)
        new = dict([(i, (p, t)) for (i, (p, t)) in enumerate(zip(pressure, temperature)) if i in self.alive])
        old = dict(zip(self.alive, zip(pressure, temperature)))
        return new

    def getData(self):
        """Return measured pressure in kPa, temperature compensated and factory calibrated."""
        # get raw 10b data
        data = self.getDataRaw()
        # helper functions to increase readability
        Padc = lambda cell: data[cell][0]
        Tadc = lambda cell: data[cell][1]
        # initialize array for compensated pressure readings
        Pcomp = {}
        # for element in the returned pressure data...
        for cell in self.alive:
            # load the calibration coefficients calculated when the TakkTile class is initialized
            cc = self.calibrationCoefficients[cell]
            # apply the formula contained on page 13 of Freescale's AN3785
            # "The 10-bit compensated pressure output for MPL115A, Pcomp, is calculated as follows:
            #  Pcomp = a0 + (b1 + c11*Padc + c12*Tadc) * Padc + (b2 + c22*Tadc) * Tadc"
            Pcomp[cell] = cc["a0"] + (cc["b1"] + cc["c11"]*Padc(cell) + cc["c12"]*Tadc(cell))*Padc(cell) + (cc["b2"] + cc["c22"]*Tadc(cell))*Tadc(cell)
            # convert from 10b number to kPa
            Pcomp[cell] = 65.0/1023.0*Pcomp[cell]+50
            # round to keep sane sigfig count
            Pcomp[cell] = round(Pcomp[cell], 4)
        return Pcomp

    def getCalibrationData(self, index):
        """Request the 12 calibration bytes from a sensor at a specified index."""
        # get the attiny's virtual address for the specified index
        # read the calibration data via vendor request and return it
        return list(self.dev.ctrl_transfer(0x40|0x80, 0x6C, 0, index, 8))

    def startSampling(self, dt = 120):
        self.sampling = True
        return self.dev.ctrl_transfer(0x40|0x80, 0xC7, dt, 0xFF, 1)[0]

    def stopSampling(self):
        self.sampling = False
        return self.dev.ctrl_transfer(0x40|0x80, 0xC7, 0, 0, 1)[0]

if __name__ == "__main__":
    import sys, pprint
    tact = TakkTile()
    print tact.UID
    print tact.getAlive()
    try:
        channel = str(args.side.upper())
        count = int(sys.argv[1])
    except:
        print "USAGE: takktile_usb <left/right>"

    #Start LCM
    lc = lcm.LCM()
    dataChannel = 'TAKKTILE_RAW_' + channel

    import time
    tact.startSampling(200)
    start = time.time()

    while True:
        data = tact.getData()
        stateMsg = takktile.state_t()
        stateMsg.data_length = len(data.keys())
	for key in data.keys():
            stateMsg.id.append(key)
            stateMsg.force.append(data[key])
            stateMsg.temp.append(500)
        lc.publish(dataChannel, stateMsg.encode())
	time.sleep(0.015)  # just to slow things down
                           # raw data alone seems to be about 150Hz+
	                   # this should slow it down to about 60Hz

        #print(data)

    end = time.time()
    tact.stopSampling()
    print (end-start)/int(count)
