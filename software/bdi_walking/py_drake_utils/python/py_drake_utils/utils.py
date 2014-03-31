import numpy as np
from collections import namedtuple
import drc
import time
import lcm

Pos = namedtuple('Pos', 'x y z roll pitch yaw')

def rpy2rotmat(rpy):
    rotMat = np.array([[np.cos(rpy[2])*np.cos(rpy[1]),
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
    rpy = np.array([np.arctan2(2*(w*x + y*z), w*w + z*z -(x*x +y*y)),
                    np.arcsin(2*(w*y - z*x)),
                    np.arctan2(2*(w*z + x*y), w*w + x*x-(y*y+z*z))])
    return rpy

def rotmat2rpy(R):
    if R.shape == (2,2):
        return np.arctan2(R[1,0], R[0,0])
    else:
        # NOTE: assumes we're using an X-Y-Z convention to construct R
        return np.array([np.arctan2(R[2,1],R[2,2]),
                         np.arctan2(-R[2,0],np.sqrt(R[2,1]**2 + R[2,2]**2)),
                         np.arctan2(R[1,0],R[0,0])])

def rpy2quat(rpy):
    # converts ROS's rpy representation of angles to unit quaternions
    # from https://code.ros.org/trac/ros-pkg/ticket/4247, but with
    #  q = [w;x;y;z]
    rpy = np.array(rpy)
    s=np.sin(rpy/2)
    c=np.cos(rpy/2);

    q = np.array([c[0]*c[1]*c[2] + s[0]*s[1]*s[2],
                  s[0]*c[1]*c[2] - c[0]*s[1]*s[2],
                  c[0]*s[1]*c[2] + s[0]*c[1]*s[2],
                  c[0]*c[1]*s[2] - s[0]*s[1]*c[2]])

    return q/(np.linalg.norm(q)+np.spacing(1));

def mk_transform(xyz, rpy):
    return np.vstack((np.hstack((rpy2rotmat(rpy), np.array(xyz).reshape((3,1)))),
                      np.array([0,0,0,1])))

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
