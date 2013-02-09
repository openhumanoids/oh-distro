#ifndef VOESTIMATOR_HPP_
#define VOESTIMATOR_HPP_

#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <lcm/lcm-cpp.hpp>
#include <bot_frames/bot_frames.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <lcmtypes/bot_core.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds


class VoEstimator
{
public:
  VoEstimator(boost::shared_ptr<lcm::LCM> &lcm_, BotFrames* botframes_);
  ~VoEstimator();

  void voUpdate(int64_t utime, Eigen::Isometry3d delta_camera);

  Eigen::Isometry3d getCameraPose(){ return local_to_head_*head_to_camera_; }
private:
  boost::shared_ptr<lcm::LCM> lcm_;
  pointcloud_vis* pc_vis_;
  int64_t utime_;

  BotFrames* botframes_;
  bot::frames* botframes_cpp_;

  Eigen::Isometry3d camera_to_head_, head_to_camera_, local_to_head_;
};

#endif
