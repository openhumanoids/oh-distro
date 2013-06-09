#!/usr/bin/python
# Filename: csv_parse.py
import csv


class Joint(object):
    def __init__(self):
        self.name=''
        self.kp=0
        self.kd=0
        self.points=[]

class Data(object):
    def __init__(self):
        self.joints=[] # joint names
        self.traj_len = 0
        self.dt =[]

def csv_parse(fname):
  d = Data()

  with open(fname, 'rb') as csvfile:
    greader = csv.reader(csvfile, delimiter=',', quotechar='|')
    init=False
    for row in greader:
      if(init==False):
        init=True
        for k in range(3,len(row) ):
          d.dt.append( float(row[k]) )
      else:
        j=Joint()
        j.name = row[0]
        j.kp = float(row[1])
        j.kd = float(row[2])
        points=[]
        for k in range(3,len(row) ):
          points.append( float(row[k]) )
        j.points=points
        d.joints.append(j)
        d.traj_len = len(points)

  #for j in d.joints:
  #    print "%s - %f %f %d points" %(j.name, j.kp, j.kd, len(j.points))

  return d
