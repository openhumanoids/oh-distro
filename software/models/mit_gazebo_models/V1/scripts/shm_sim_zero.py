#!/usr/bin/python

import os
import pwd
import mmap
fname = '/dev/shm/robonet_{}'.format(pwd.getpwuid(os.getuid())[0])
size = os.path.getsize(fname)
f = open(fname, 'r+w')
robonet = mmap.mmap(f.fileno(), size)
for i in range(size):
   robonet[i] = '\0'
