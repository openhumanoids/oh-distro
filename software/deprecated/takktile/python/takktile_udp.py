#!/usr/bin/env python
# -*- coding: utf-8 ; mode: python -*-
#
# takktile_udp.py
#
# Copyright (C) 2014 Nuno Sucena Almeida
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import logging
import socket
import asyncore
import threading
import time
import Queue
import struct
import io
import argparse

logger = logging.getLogger(__name__)

class TakktileUDPDispatcher(asyncore.dispatcher):
    """
    """
    def __init__(self, host, port):
        asyncore.dispatcher.__init__(self)

        self.create_socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.set_reuse_addr()
        self.bind((host, port))

        self._queue = Queue.Queue()

    def handle_read(self):
        message, address = self.recvfrom(8192)
        if message is not None:
            self._queue.put(message)

class TakktileUDPDispatcherThread(threading.Thread):
    """
    """
    def __init__(self, host, port):
        threading.Thread.__init__(self)
        self._dispatcher = TakktileUDPDispatcher(host,port)

    def run(self, ):
        asyncore.loop(timeout=5)

    def close(self, ):
        self._dispatcher.close()

    @property
    def queue(self,):
        return self._dispatcher._queue

class TakktileUDP(object):
    """
    """

    DEFAULT_REMOTE_HOSTNAME = "takktile-01"
    DEFAULT_REMOTE_LISTEN_PORT = 31337
    DEFAULT_LOCAL_HOSTNAME = "0.0.0.0"
    DEFAULT_LOCAL_LISTEN_PORT = DEFAULT_REMOTE_LISTEN_PORT
    DEFAULT_TIMEOUT = 0.1

    def __init__(self, remoteHostname=None, remotePort=None, localHostname=None, localPort=None, timeout=None):
        """

        Arguments:
        - `remoteHostname`:
        - `remotePort`:
        - `localHostname`:
        - `localPort`:
        - `timeout`:
        """

        self._remoteHostname = remoteHostname if remoteHostname is not None else self.DEFAULT_REMOTE_HOSTNAME
        self._remotePort = remotePort if remotePort is not None else self.DEFAULT_REMOTE_LISTEN_PORT
        self._localHostname = localHostname if localHostname is not None else self.DEFAULT_LOCAL_HOSTNAME
        self._localPort = localPort if localPort is not None else self.DEFAULT_LOCAL_LISTEN_PORT
        self._timeout = timeout if timeout is not None else self.DEFAULT_TIMEOUT
        self._sampling = False
        self._alive = None
        self._dataRaw = None
        self._data = None

        logger.info("Listening on {}:{}".format(self._localHostname,self._localPort))
        logger.info("Requesting from {}:{}".format(self._remoteHostname,self._remotePort))

        self._sendSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self._receiver = TakktileUDPDispatcherThread(self._localHostname,self._localPort)
        self._receiver.start()

        self._rbuf = io.BytesIO()

    def __enter__(self):
        self.startSampling()
        return self

    def __exit__(self, type, value, traceback):
        self.stopSampling()

    def close(self, ):
        self._receiver.close()
        self._receiver.join()

    def queryData(self,):
        numberSensors=None
        try:
            self._sendSocket.sendto("t",(self._remoteHostname,self._remotePort))
            data = self._receiver.queue.get(True,timeout=self._timeout)

            self._rbuf.truncate()
            self._rbuf.seek(0,io.SEEK_SET)
            self._rbuf.write(data)
            self._rbuf.seek(0,io.SEEK_SET)
            header = self._rbuf.read(1)
            numberSensors = struct.unpack("!B",header)[0]

            logger.debug("numberSensors={}, data={}".format(numberSensors,len(data)))

            expected = 1+numberSensors*(4*2+1)
            assert expected == len(data)

        except AssertionError as e:
            logger.error("data length received={}, expected={}".format(len(data),(expected)))
            raise

        except Queue.Empty, q:
            logger.error("Queue is empty {}".format(q))

        except socket.error, e:
            logger.error("Failed to open {} : I/O error({})".format((self._remoteHostname,self._remotePort),e))
            raise

        return numberSensors

    @property
    def alive(self, ):
        """ set containing the cell number of all alive cells.
        """
        numberSensors = self.queryData()
        if (numberSensors is not None):
            fmtAddresses = "<%dB"%(numberSensors)
            self._alive = set(struct.unpack(fmtAddresses,self._rbuf.read(numberSensors)))
        return self._alive

    @property
    def data(self, ):
        """ Return measured pressure in kPa, temperature compensated and factory calibrated
        """
        # TODO: data
        return self._data

    @property
    def dataRaw(self, ):
        """ Return raw data as a dictionary mapping sensor indexes to a tuple containing (pressure,temperature).
        """
        self._dataRaw=None
        numberSensors = self.queryData()
        if (numberSensors is not None):
            fmtAddresses = "<%dB"%(numberSensors)
            addressValues = list(struct.unpack(fmtAddresses,self._rbuf.read(numberSensors)))
            fmtValues = "<%df" % (numberSensors)
            pressureValues = list(struct.unpack(fmtValues,self._rbuf.read(numberSensors*4)))
            temperatureValues = list(struct.unpack(fmtValues,self._rbuf.read(numberSensors*4)))

            self._dataRaw = dict([(i,(int(p),int(t))) for i,p,t in zip(addressValues,pressureValues,temperatureValues)])

        return self._dataRaw

    @property
    def calibrationCoefficients(self, index ):
        """
        """
        # TODO: calibrationCoefficients
        pass

    @property
    def calibrationData(self, index):
        """ return the 12 calibration bytes from a sensor at a specified index.
        """
        # TODO: calibrationData
        pass

    # for compatibility reasons

    def startSampling(self, ):
        """
        """
        logger.debug("Starting sampling")
        self._sampling = True

    def stopSampling(self, ):
        """
        """
        self.close()
        logger.debug("Stopping sampling")
        self._sampling = False

    def getDataRaw(self, ):
        """
        """
        return self.dataRaw

if __name__ == '__main__':
    LEVELS = {"debug": logging.DEBUG, "info": logging.INFO, "warning": logging.WARNING,
              "error": logging.ERROR, "critical": logging.CRITICAL}

    parser = argparse.ArgumentParser(description="Query tool for Takktile UDP based sensor")
    parser.add_argument("--version", action="version", version="%(prog)s 1.0")
    parser.add_argument("--log-level", "-l", type=str,dest="loglevel",choices=LEVELS.keys(),
                        default="error")
    parser.add_argument("--remote-host","-H",type=str,help="remote host",dest="remoteHost",
                        default=TakktileUDP.DEFAULT_REMOTE_HOSTNAME)
    parser.add_argument("--remote-port","-P",type=int,help="remote port",dest="remotePort",
                        default=TakktileUDP.DEFAULT_REMOTE_LISTEN_PORT)
    parser.add_argument("--local-host","-M",type=str,help="local host",dest="localHost",
                        default=TakktileUDP.DEFAULT_LOCAL_HOSTNAME)
    parser.add_argument("--local-port","-L",type=int,help="local port",dest="localPort",
                        default=TakktileUDP.DEFAULT_LOCAL_LISTEN_PORT)
    parser.add_argument("--timeout","-t",type=float,help="how long to wait for a response [seconds]",dest="timeout",
                        default=TakktileUDP.DEFAULT_TIMEOUT)
    parser.add_argument("--samples","-s",type=int,help="how many sample queries",dest="samples",
                        default=300)

    args = parser.parse_args()

    level = LEVELS.get(args.loglevel,logging.INFO)
    logging.basicConfig(format='%(asctime)s - %(filename)s - %(levelname)s - %(message)s',
                        level=level)

    with TakktileUDP(args.remoteHost,args.remotePort,args.localHost,args.localPort,args.timeout) as tk:
        while tk.alive is None:
            print("Waiting for {}".format(args.remoteHost))
        alive = tk.alive
        print ("Alive: {} sensors: {}".format(len(alive),alive))

        samples = args.samples
        start = time.time()
        for s in range(samples):
            print ("{:>4} Data: {}".format(s,tk.dataRaw))
        stop = time.time()
        took = stop-start
        print("Took {:.5} seconds to finish. {:.5} Hz".format(took,samples/took))
