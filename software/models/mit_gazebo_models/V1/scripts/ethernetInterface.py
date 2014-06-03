#!/usr/bin/python

import netifaces
import subprocess

ethernetInterface = None
for intf in netifaces.interfaces():
    # print "intf: ", intf
    try:
        eInt = netifaces.ifaddresses(intf)[2][0]['addr']
        # print "eInt: ", eInt
        if "139.169.44" in eInt:
            ethernetInterface = intf
            break
    except:
        pass

# print "ethernetInterface: ", ethernetInterface

if ethernetInterface is not None:
    # cmd = "sudo ptpd -b %s -g -f /var/log/ptpd.log" % ethernetInterface
    cmd = "sudo ptpd -b %s -g -D -c" % ethernetInterface
    print cmd
    subprocess.call(cmd, shell=True)