#!/usr/bin/env python
import socket
import time
UDP_IP="127.0.0.1"
UDP_PORT=41234
import math

 
sock = socket.socket( socket.AF_INET, # Internet
                       socket.SOCK_DGRAM ) # UDP
i = 0
while True:
    msg='{"S":%d, "V":%d, "C":%d}' % (((math.sin(i) + 1) *50), ((math.cos(i) + 1) * 50), (math.cos(i)) )
    sock.sendto( msg, (UDP_IP, UDP_PORT) )
    print msg
    time.sleep(.1)
    i = i + .1
