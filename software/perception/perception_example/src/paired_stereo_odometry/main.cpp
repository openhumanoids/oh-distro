// This program is essentially a duplicate of the stereo odometry
// algorithm in fovis - except it uses a different message type,
// Instead of a single image containing left and right stacked 
// this program assumes them to be each individual bot_core_images
// inside a light wrapper.
// mfallon 2012 aug

#include "paired-stereo-odometry.hpp"

int main(int argc, char** argv)
{
  fovis::StereoOdometry stereo_odom;
  int status = stereo_odom.initialize(argc, argv);
  if (status) { return status; }
  stereo_odom.go();
  return 0;
}
