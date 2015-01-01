#!/usr/bin/python

import os
from subprocess import call

os.chdir(os.path.dirname(os.path.realpath(__file__)))

scripts = ["atlas_skeleton_v4.py",
           "atlas_transmission_v4.py"]

for script in scripts:
    print "Running '%s'" % script
    call(["python", script])
