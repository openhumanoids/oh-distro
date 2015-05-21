import numpy as np

blockName = 'stair'
blockSize = np.array([1.0, 0.28, 0.22]) # meters
blockTiltAngle = 0 # degrees


# F=sloping up forward (+x), B=sloping up backward (-x),
# R=sloping up rightward (-y), L=sloping up leftward (+y)
# last row is closest to robot (robot is on bottom looking up)
# column order is left-to-right on robot (+y to -y)
blockTypes = [
    [ 'B' ],
    [ 'A' ],
    [ 'A' ],
    [ 'A' ]
]
blockTypes.reverse()

# 0=ground level, 1=one cinderblock offset, etc
blockLevels = [
    [ 3 ],
    [ 2 ],
    [ 1 ],
    [ 0 ]
]
blockLevels.reverse()

# map between block types and yaw angles (degrees)
blockAngleMap = { 'A': 90, 'B': 90 }

# TODO: this is just an example
# which foot, block (row,col), offset (x,y), support
# (row,col) refer to which block
# (x,y) are offsets wrt the block center, in meters
# support is an enum indicating foot support type
#   0=heel-toe, 1=midfoot-toe, 2=heel-midfoot
footstepData = [
    [ 'left',  (0,0), (0.00, 0.00),  0 ],
    [ 'right', (0,0), (0.00, 0.00),  0 ],
    [ 'left',  (1,0), (-0.10, 0.00), 0 ],
    [ 'right', (1,0), (0.00, 0.05),  0 ],
    [ 'left',  (2,0), (0.10, -0.05), 0 ],
    [ 'right', (2,0), (0.10, 0.10),  0 ]
]

blockColor = [0.4, 0.6, 0.4]
blockColorMatched = [0.5, 0.8, 0.5]
