from __future__ import division

import lcm
import drc
import argparse

def main():
    parser = argparse.ArgumentParser(description="Set Atlas velocity for a specified duration")
    parser.add_argument('-x', metavar='v_x', type=float, default=0)
    parser.add_argument('-y', metavar='v_y', type=float, default=0)
    parser.add_argument('-z', metavar='v_z', type=float, default=0)
    parser.add_argument('-d', '--duration', metavar='T', type=float, default=0.1, help="Duration of the velocity kick")
    args = parser.parse_args()

    msg = drc.atlas_push_t()
    msg.velocity = drc.vector_3d_t()
    msg.velocity.x = args.x
    msg.velocity.y = args.y
    msg.velocity.z = args.z
    msg.duration = args.duration
    print "Kicking Atlas up to velocity: ({vel.x}, {vel.y}, {vel.z}) for {d} seconds".format(vel=msg.velocity, d = msg.duration)
    lcm.LCM().publish('ATLAS_PUSH', msg.encode())


if __name__ == "__main__":
    main()
