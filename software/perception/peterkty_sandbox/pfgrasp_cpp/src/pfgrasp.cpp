#include <memory>
#include <string>

#include <drc_utils/BotWrapper.hpp>
#include <drc_utils/LcmWrapper.hpp>
#include <bot_core/camtrans.h>
#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/perception/pfgrasp_command_t.hpp>
#include <lcmtypes/perception/image_roi_t.hpp>

#include <opencv2/opencv.hpp>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <ConciseArgs>

#include <tld-tracker/tld-tracker.hpp>
#include "ImageWarper.hpp"
#include "libparticle/particle_filter.hpp"

struct PFGraspOptions
{
  bool debug;
  float scale;
  std::string cameraChannelName;
  std::string segmenterChannelName;
  std::string commandChannelName;

  PFGraspOptions() :
      cameraChannelName("CAMERALHAND"), scale(1.f), debug(false), segmenterChannelName(
          "TLD_OBJECT_ROI"), commandChannelName("TLD_CMD")
  {
  }
};

class PFGrasp
{
public:
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
  std::shared_ptr<drc::BotWrapper> botWrapper_;
  std::shared_ptr<drc::LcmWrapper> lcmWrapper_;
  std::shared_ptr<lcm::LCM> lcm_;
  PFGraspOptions options_;

  // Parameters for the camera
  CameraParams cameraParams_;

  // Img, and warped image
  cv::Mat img_, wimg_;
  int64_t img_utime_;

  // TLD Tracker
  TLDTracker* tracker_;

  // Image warper
  ImageWarper* warper_;

  int counter_;

  float bearing_a_,bearing_b_;

  ParticleFilter pf;
  // handle reset, run-one-iteration
  void
  commandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
      const perception::pfgrasp_command_t* msg);

  // get image from hand camera
  void
  imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
      const bot_core::image_t* msg);

  // get segment from track segmenter
  void
  segmentHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
      const perception::image_roi_t* msg);

  void
  runOneIter();

  void
  initParticleFilter();
};

void
decode_image(const bot_core::image_t* msg, cv::Mat& img)
{
  if (img.empty() || img.rows != msg->height
      || img.cols != msg->width)
    img.create(msg->height, msg->width, CV_8UC3);

  switch (msg->pixelformat) {
    case bot_core::image_t::PIXEL_FORMAT_RGB:
        memcpy(img.data, msg->data.data(), sizeof(uint8_t) * msg->width * msg->height * 3);
        cv::cvtColor(img, img, CV_RGB2BGR);
      break;
    case bot_core::image_t::PIXEL_FORMAT_MJPEG:
      // for some reason msg->row_stride is 0, so we use msg->width instead.
      jpeg_decompress_8u_rgb(msg->data.data(),
                              msg->size,
                              img.data,
                              msg->width,
                              msg->height,
                              msg->row_stride);
      cv::cvtColor(img, img, CV_RGB2BGR);
      break;
    case bot_core::image_t::PIXEL_FORMAT_GRAY:
      //memcpy(img.data, msg->data.data(), sizeof(uint8_t) * msg->width * msg->height);
      fprintf(stderr, "Gray image not supported\n");
      break;
    default:
      fprintf(stderr, "Unrecognized image format\n");
      break;
  }

  return;
}

void
PFGrasp::imageHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string &channel, const bot_core::image_t* msg)
{
  if (!msg->width || !msg->height) return;

  double tic = bot_timestamp_now();
  decode_image(msg, img_);

  if(wimg_.empty() || wimg_.cols != img_.cols || wimg_.rows != img_.rows || wimg_.channels() != img_.channels() || wimg_.type() != img_.type()){
    wimg_.create(img_.rows, img_.cols, CV_8UC3);
  }

  warper_->unWarp(img_, wimg_);
  // Update step
  std::vector< Eigen::Vector3d > pts;
  Eigen::Isometry3d local_to_camera;

  if (tracker_->initialized)
      tracker_->update(wimg_, pts, local_to_camera);    // ?????? can add real local to camera value here

  if (++counter_ == 10) {
      printf("===> TLD TRACKER: %4.2f ms\n", (bot_timestamp_now() - tic) * 1.f * 1e-3);
      counter_ = 0;
  }

  // Update bearing vector, save the result for particle filter
  img_utime_ = msg->utime;
  if (tracker_->detection_valid) {
    const cv::Rect& currBB = tracker_->currBB;
    float roix = currBB.x + currBB.width/2, roiy = currBB.y + currBB.height/2;
    float u = -(cameraParams_.cx - roix);
    float v = -(cameraParams_.cy - roiy);
    if (!cameraParams_.fx && !cameraParams_.fy)
      bearing_a_ = bearing_b_ = 0;
    else
      bearing_a_ = u*1.f/cameraParams_.fx, bearing_b_ = v*1.f/cameraParams_.fy;
  }

  // Viz
  cv::Mat dispimg = wimg_.clone();

  if (tracker_->detection_valid) {
      const cv::Rect& currBB = tracker_->currBB;
      std::cerr << "TLD currBB: " << currBB.tl() << " " << currBB.br() << std::endl;

      rectangle(dispimg, currBB, Scalar(0,0,255), 3, 8, 0);
  }
  Mat scaleddispimg;
  cv::resize(dispimg, scaleddispimg, Size(dispimg.cols/2,dispimg.rows/2), 0,0,INTER_LINEAR);
  cv::imshow( "Display window", scaleddispimg );
  cv::waitKey(1);

  return;
}

void
PFGrasp::segmentHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string &channel, const perception::image_roi_t* msg)
{
  std::cerr << "SEGMENTATION msg: " << msg->utime << " " << msg->roi.x << " "
      << msg->roi.y << " " << msg->roi.width << " " << msg->roi.height
      << std::endl;

  float sw = cameraParams_.width, sh = cameraParams_.height;
  Rect selection(msg->roi.x * sw, msg->roi.y * sh, msg->roi.width * sw,
      msg->roi.height * sh);

  if (msg->feature_id < 0)
    {
      std::cerr << "INVALID feature_id" << std::endl;
      return;
    }

  // Ensure tracker is initialized
  if (!tracker_->initialized)
    {
      std::cerr << "Tracker Not Initialized!" << std::endl;
      assert(0);
    }

  // Initialize with image and mask
  tracker_->initialize(img_, selection);

  if (options_.debug)
    {
      cv::Mat imgd = img_.clone();
      rectangle(imgd, selection.tl(), selection.br(), CV_RGB(255, 0, 0), 2);
      cv::imshow("Captured Image ROI", imgd);
    }
}


void
PFGrasp::commandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
    const perception::pfgrasp_command_t* msg){
  switch (msg->command) {
  case perception::pfgrasp_command_t::STOP:   // stop tracking, unsubscribe image, release particles
  break;
  case perception::pfgrasp_command_t::START:  // start tracking, subscribe image, initialize particles
    initParticleFilter();
  break;
  case perception::pfgrasp_command_t::RUN_ONE_ITER:
    runOneIter();
  break;
  }
}


void
PFGrasp::initParticleFilter(){
  int rng_seed = 0;
  double resample_threshold =0.5;
  pf = new ParticleFilter(N_p, gen_x0, p_yk_given_xk, gen_sys_noise, rng_seed, resample_threshold);

}

PFGrasp::PFGrasp(PFGraspOptions options) :
    options_(options)
{
  lcmWrapper_.reset(new drc::LcmWrapper());
  lcm_ = lcmWrapper_->get();
  botWrapper_.reset(new drc::BotWrapper(lcm_));

  // Camera Params
  cameraParams_ = CameraParams(botWrapper_->getBotParam(),
      "cameras." + options_.cameraChannelName + ".intrinsic_cal");

  // subscribing to image, segmenter, commands
  lcm::Subscription* sub1 = lcm_->subscribe(options_.cameraChannelName.c_str(), &PFGrasp::imageHandler,
      this);
  lcm::Subscription* sub2 = lcm_->subscribe(options_.segmenterChannelName.c_str(), &PFGrasp::segmentHandler,
      this);
  lcm::Subscription* sub3 = lcm_->subscribe(options_.commandChannelName.c_str(), &PFGrasp::commandHandler,
      this);
  sub1->setQueueCapacity(1);
  sub2->setQueueCapacity(1);
  sub3->setQueueCapacity(1);
  // Initialize TLD tracker
  tracker_ = new TLDTracker(cameraParams_.width, cameraParams_.height,
      options_.scale);

  // Initialize Image warper
  warper_ = new ImageWarper(options_.cameraChannelName);

  counter_ = 0;
}

int
main(int argc, char** argv)
{
  PFGraspOptions options;

  ConciseArgs opt(argc, (char**) argv);
  opt.add(options.cameraChannelName, "c", "camera-channel",
      "Camera Channel [CAMERALHAND]");
  opt.add(options.scale, "s", "scale", "Scale Factor");
  opt.add(options.debug, "d", "debug", "Debug");
  opt.parse();

  PFGrasp pfgrasp(options);

  std::cout << "\npfgrasp ready\n";
  pfgrasp.start();
  return 0;
}
