#ifndef LIDAR_ODOM_HPP_
#define LIDAR_ODOM_HPP_

#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
// #include <lcmtypes/scanmatch.hpp>
// #include <scanmatch/ScanMatcher.hpp>
#include <frsm/frsm.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace frsm;


class LidarOdom
{
public:
    LidarOdom(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~LidarOdom();

    void doOdometry(float* ranges, int nranges, float rad0, float radstep, int64_t utime);

    Eigen::Isometry3d getCurrentPose(){  return currOdom_;  }

    Eigen::Isometry3d getMotion(){ return  (currOdom_ *  prevOdom_.inverse()); }


private:
    boost::shared_ptr<lcm::LCM> lcm_;

    int64_t utime_cur_, utime_prev_;

    ScanMatcher* sm_;
    frsm_laser_type_t laserType_;
    int beamSkip_; //downsample ranges by only taking 1 out of every beam_skip points
    double spatialDecimationThresh_; //don't discard a point if its range is more than this many std devs from the mean range (end of hallway)
    double maxRange_; //discard beams with reading further than this value
    float validBeamAngles_[2]; //valid part of the field of view of the laser in radians, 0 is the center beam

    bool publishRelative_;
    bool publishPose_;
    bool doDrawing_;

    Eigen::Isometry3d prevOdom_;
    Eigen::Isometry3d currOdom_;
    int64_t prevUtime_;
    int64_t currUtime_;
};

#endif
