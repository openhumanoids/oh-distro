/*
 * pfgrasp.hpp
 *
 *  Created on: Apr 22, 2014
 *      Author: drc
 */



#ifndef PFGRASP_HPP_
#define PFGRASP_HPP_

#include <drc_utils/BotWrapper.hpp>
#include <drc_utils/LcmWrapper.hpp>
#include <bot_core/camtrans.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/drc/pfgrasp_command_t.hpp>
#include <lcmtypes/perception/image_roi_t.hpp>
#include <lcmtypes/bot_frames/update_t.hpp>

#include "ImageWarper.hpp"
#include "libparticle/particle_filter.hpp"
#include <tld-tracker/tld-tracker.hpp>


struct PFGraspOptions
{
  bool debug;
  float scale;
  std::string cameraChannelName;
  std::string segmenterChannelName;
  std::string commandChannelName;
  std::string reachGoalFrameName;
  std::string reachGoalChannelName;

  PFGraspOptions() :
      cameraChannelName("CAMERALHAND"), scale(1.f), debug(false), segmenterChannelName(
          "TLD_OBJECT_ROI"), commandChannelName("PFGRASP_CMD"), reachGoalFrameName("LHAND_FACE"),
          reachGoalChannelName("REACH_TARGET_POSE")
  {
  }
};

class PFGrasp
{
public:
  std::shared_ptr<drc::BotWrapper> botWrapper_;
  PFGraspOptions options_;
  BotCamTrans* warpedCamTrans;

  // needed for particle filter measurement update
  BotTrans localToCam_;
  float bearing_a_,bearing_b_;
  int64_t img_utime_;
  double bound;

  PFGrasp(PFGraspOptions options);
  ~PFGrasp()
  {
  }

  void
  start()
  {
    lcmWrapper_->startHandleThread(true);
  }
private:
  std::shared_ptr<drc::LcmWrapper> lcmWrapper_;
  std::shared_ptr<lcm::LCM> lcm_;
  BotFrames* botFrames_;
  bot_lcmgl_t* lcmgl_;

  // Parameters for the camera
  CameraParams cameraParams_;

  // Img, and warped image
  cv::Mat img_, wimg_;

  // TLD Tracker
  TLDTracker* tracker_;

  // Image warper
  ImageWarper* warper_;

  int counter_;


  ParticleFilter* pf;
  int rng_seed;
  double resample_threshold;
  int N_p;

  int64_t cmd_utime_;
  // handle reset, run-one-iteration
  void
  commandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
      const drc::pfgrasp_command_t* msg);

  // get image from hand camera
  void
  imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
      const bot_core::image_t* msg);

  // get segment from track segmenter
  void
  segmentHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
      const perception::image_roi_t* msg);

  void
  initParticleFilter();

  void
  runOneIter();

  void
  releaseParticleFilter();

  void
  publishHandReachGoal(const BotTrans& bt);
};


#endif /* PFGRASP_HPP_ */
