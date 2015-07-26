#ifndef LIDAR_ODOM_HPP_
#define LIDAR_ODOM_HPP_

#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/scanmatch.hpp>
#include <scanmatch/ScanMatcher.hpp>

using namespace std;
using namespace scanmatch;

class LidarOdom
{
public:
    LidarOdom(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~LidarOdom();

    void doOdometry(const float* ranges, int nranges, float rad0, float radstep, int64_t utime);

private:
    boost::shared_ptr<lcm::LCM> lcm_;

    int64_t utime_cur_, utime_prev_;


    BotParam* botparam_;
    BotFrames* botframes_;


    ScanMatcher* sm_;
    sm_laser_type_t laserType_;
    int beamSkip_; //downsample ranges by only taking 1 out of every beam_skip points
    double spatialDecimationThresh_; //don't discard a point if its range is more than this many std devs from the mean range (end of hallway)
    double maxRange_; //discard beams with reading further than this value
    float validBeamAngles_[2]; //valid part of the field of view of the laser in radians, 0 is the center beam
    sm::rigid_transform_2d_t prevOdom_;

    bool publishRelative_;
    bool publishPose_;
    bool doDrawing_;
};

#endif
