import numpy as np
from collections import namedtuple
import drc
import time
import lcm

Pos = namedtuple('Pos', 'x y z roll pitch yaw')

def rpy2rotmat(rpy):
    rotMat = np.matrix([[np.cos(rpy[2])*np.cos(rpy[1]),
                         np.cos(rpy[2])*np.sin(rpy[1])*np.sin(rpy[0])-np.sin(rpy[2])*np.cos(rpy[0]),
                         np.cos(rpy[2])*np.sin(rpy[1])*np.cos(rpy[0])+np.sin(rpy[2])*np.sin(rpy[0])],
                        [np.sin(rpy[2])*np.cos(rpy[1]),
                         np.sin(rpy[2])*np.sin(rpy[1])*np.sin(rpy[0])+np.cos(rpy[2])*np.cos(rpy[0]),
                         np.sin(rpy[2])*np.sin(rpy[1])*np.cos(rpy[0])-np.cos(rpy[2])*np.sin(rpy[0])],
                        [-np.sin(rpy[1]),
                         np.cos(rpy[1])*np.sin(rpy[0]),
                         np.cos(rpy[1])*np.cos(rpy[0])]])
    return rotMat

def quat2rpy(q):
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    rpy = np.array([[np.arctan2(2*(w*x + y*z), w*w + z*z -(x*x +y*y))],
                    [np.arcsin(2*(w*y - z*x))],
                    [np.arctan2(2*(w*z + x*y), w*w + x*x-(y*y+z*z))]])
    return rpy

def angleDiff(phi1, phi2):
    return np.mod(phi2 - phi1 + np.pi, 2 * np.pi) - np.pi

def send_status(system, importance, frequency, value):
    status = drc.system_status_t()
    status.utime = time.time() * 1e6
    status.system = system
    status.importance = importance
    status.frequency = 0
    status.value = value
    lcm.LCM().publish('SYSTEM_STATUS', status.encode())
