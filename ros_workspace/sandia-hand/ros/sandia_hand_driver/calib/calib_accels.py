#!/usr/bin/env python
import yaml, sys

def calib_from_file(filename):
  y = yaml.load(file(filename, 'r'))
  lower = [[1e6]*3, [1e6]*3, [1e6]*3]
  upper = [[-1e6]*3, [-1e6]*3, [-1e6]*3]
  boards = { 'mm': 0, 'pp': 1, 'dp': 2 }
  for sample in y:
    for board_name, board_idx in boards.iteritems():
      for axis in xrange(0,3):
        if sample[board_name][axis] > upper[board_idx][axis]:
          upper[board_idx][axis] = sample[board_name][axis]
        if sample[board_name][axis] < lower[board_idx][axis]:
          lower[board_idx][axis] = sample[board_name][axis]
  bias = [[0]*3, [0]*3, [0]*3]
  scale = [[0]*3, [0]*3, [0]*3]
  for board_name, board_idx in boards.iteritems():
    for axis in xrange(0, 3):
      bias[board_idx][axis] =  (upper[board_idx][axis] + 
                                lower[board_idx][axis]) / 2
      scale[board_idx][axis] = (upper[board_idx][axis] -
                                lower[board_idx][axis]) / 2

  print "min:   " + str(lower)
  print "max:   " + str(upper)
  print "bias:  " + str(bias)
  print "scale: " + str(scale)

if __name__ == '__main__':
  if len(sys.argv) != 2:
    print "usage: calib_accels.py YAML_FILENAME"
    sys.exit(1)
  calib_from_file(sys.argv[1])

