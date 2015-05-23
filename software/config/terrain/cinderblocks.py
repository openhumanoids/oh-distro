import numpy as np

blockName = 'cinderblock'
blockSize = np.array([15 + 5/8.0, 15 + 3/8.0, 5 + 5/8.0]) * 0.0254 # meters
blockTiltAngle = 15 # degrees


# F=sloping up forward (+x), B=sloping up backward (-x),
# R=sloping up rightward (-y), L=sloping up leftward (+y)
# last row is closest to robot (robot is on bottom looking up)
# column order is left-to-right on robot (+y to -y)
blockTypes = [
    [ 'B', 'L', 'F', 'R', 'B', 'L' ],
    [ 'L', 'F', 'R', 'B', 'L', 'F' ],
    [ 'F', 'R', 'B', 'L', 'F', 'R' ],
    [ 'R', 'B', 'L', 'F', 'R', 'B' ],
    [ 'B', 'L', 'F', 'R', 'B', 'L' ],
    [ 'L', 'F', 'R', 'B', 'L', 'F' ],
    [ 'F', 'R', 'B', 'L', 'F', 'R' ]
]
blockTypes.reverse()

# 0=ground level, 1=one cinderblock offset, etc
blockLevels = [
    [ 0, 0, 0, 0, 0, 0 ],
    [ 1, 1, 1, 1, 1, 1 ],
    [ 1, 2, 1, 1, 2, 1 ],
    [ 0, 1, 1, 1, 1, 0 ],
    [ 0, 0, 1, 1, 0, 0 ],
    [ 0, 0, 1, 1, 0, 0 ],
    [ 0, 0, 0, 0, 0, 0 ]
]
blockLevels.reverse()

# map between block types and yaw angles (degrees)
blockAngleMap = { 'F': 180, 'B': 0, 'R': 90, 'L': 270 }

# TODO: this is just an example
# which foot, block (row,col), offset (x,y), support
# (row,col) refer to which block
# (x,y) are offsets wrt the block center, in meters
# support is an enum indicating foot support type
#   0=heel-toe, 1=midfoot-toe, 2=heel-midfoot
footstepData = [
    [ 'right', (0,1), (-0.05, 0.05),  0 ],
    [ 'left',  (0,0), (0.06, -0.12),  0 ],
    [ 'right', (1,1), (-0.01, 0.11),  0 ],
    [ 'left',  (1,0), (0.03, -0.06),  0 ],
    [ 'right', (2,1), (-0.04, 0.11),  0 ],
    [ 'left',  (2,0), (0.03, -0.08),  0 ],
    [ 'right', (3,1), (-0.06, 0.12),  0 ],
    [ 'left',  (4,0), (-0.05, -0.07),  0 ],
    [ 'right', (4,1), (0.00, 0.08),  0 ],
    [ 'left',  (4,0), (0.06, -0.07),  0 ],
    [ 'right', (5,1), (-0.01, 0.12),  0 ],
    [ 'left',  (5,0), (0.06, -0.07),  0 ],
    [ 'right', (6,1), (-0.02, 0.12),  0 ],
    [ 'left',  (6,0), (0.06, -0.07),  0 ],
]

# where to stand, relative to front of first block
startingPosition = np.array([-0.39, 0.8, 0])
startingYaw = 0   # degrees

blockColor = [0.4, 0.6, 0.4]
blockColorMatched = [0.5, 0.8, 0.5]
