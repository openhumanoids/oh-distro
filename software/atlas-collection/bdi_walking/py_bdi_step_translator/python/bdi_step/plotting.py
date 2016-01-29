import bot_lcmgl as lcmgl
import numpy as np

def draw_swing(gl, pos0, pos1, swing_height, is_stepping=False, lift_height=None):
    if is_stepping:
        assert lift_height is not None, "Must provide lift_height for step behavior"
        x,y,z = step_swing_pts(pos0, pos1, swing_height, lift_height)
    else:
        x,y,z = walk_swing_pts(pos0, pos1, swing_height)
    gl.glColor3f(0, .5, .9)
    gl.glLineWidth(4)
    gl.glBegin(lcmgl.GL_LINES)
    for j in range(len(x)-1):
        gl.glVertex3f(x[j], y[j], z[j])
        gl.glVertex3f(x[j+1], y[j+1], z[j+1])
    gl.glEnd()

def step_swing_pts(pos0, pos1, swing_height, lift_height):
    midpoint = np.mean([pos0, pos1], axis=0)
    x = np.array([pos0[0], pos0[0], midpoint[0], pos1[0], pos1[0]])
    y = np.array([pos0[1], pos0[1], midpoint[1], pos1[1], pos1[1]])
    lift_start = max(pos0[2], pos1[2])
    z = np.array([pos0[2], lift_start + lift_height, lift_start + lift_height + swing_height, lift_start + lift_height, pos1[2]])
    return x,y,z

def walk_swing_pts(pos0, pos1, swing_height):
    midpoint = np.mean([pos0, pos1], axis=0)
    x = np.array([pos0[0], midpoint[0], pos1[0]])
    y = np.array([pos0[1], midpoint[1], pos1[1]])
    z = np.array([pos0[2], midpoint[2] + swing_height, pos1[2]])
    return x,y,z


