#include <memory>
#include <string>

#include "pfgrasp.hpp"
#include <opencv2/opencv.hpp>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <ConciseArgs>
#include <bot_lcmgl_client/lcmgl.h>


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

  BotTrans tmpBT;
  bot_frames_get_trans_with_utime(botFrames_, "local", options_.cameraChannelName.c_str(),  msg->utime, &tmpBT);

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
  // May need lock

  if (tracker_->detection_valid) {
    img_utime_ = msg->utime;
    localToCam_ = tmpBT;

    const cv::Rect& currBB = tracker_->currBB;
    float roix = currBB.x + currBB.width/2, roiy = currBB.y + currBB.height/2;
    float u = -(cameraParams_.cx - roix);
    float v = -(cameraParams_.cy - roiy);
    if (!cameraParams_.fx && !cameraParams_.fy)
      bearing_a_ = bearing_b_ = 0;
    else
      bearing_a_ = u*1.f/cameraParams_.fx, bearing_b_ = v*1.f/cameraParams_.fy;

    perception::image_roi_t trackmsg;

    float sw = cameraParams_.width, sh = cameraParams_.height;
    trackmsg.roi.x = currBB.x / sw;
    trackmsg.roi.y = currBB.y / sh;
    trackmsg.roi.width = currBB.width / sw;
    trackmsg.roi.height = currBB.height / sh;
    lcm_->publish("TLD_OBJECT_ROI_RESULT", &trackmsg);
  }
  else {
    perception::image_roi_t trackmsg;
    trackmsg.roi.x = 0;
    trackmsg.roi.y = 0;
    trackmsg.roi.width = 0;
    trackmsg.roi.height = 0;
    lcm_->publish("TLD_OBJECT_ROI_RESULT", &trackmsg);
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
  tracker_->initialize(wimg_, selection);

  if (options_.debug)
    {
      cv::Mat imgd = wimg_.clone();
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
  case perception::pfgrasp_command_t::RESTART:   // stop tracking, unsubscribe image, release particles
    initParticleFilter();
  break;
  case perception::pfgrasp_command_t::START:  // start tracking, subscribe image, initialize particles
    initParticleFilter();
  break;  // also run one iteration
  case perception::pfgrasp_command_t::RUN_ONE_ITER:
    runOneIter();
  break;
  }
}

void
PFGrasp::releaseParticleFilter(){
  pf = new ParticleFilter(N_p, rng_seed, resample_threshold, (void*)this);
  delete pf;

  bot_lcmgl_switch_buffer(lcmgl_);
}


void
PFGrasp::initParticleFilter(){
  std::cout << "dbg-initParticleFilter1" << std::endl;
  pf = new ParticleFilter(N_p, rng_seed, resample_threshold, (void*)this);
  std::cout << "dbg-initParticleFilter2" << std::endl;


  bot_lcmgl_color3f(lcmgl_, 1,0,1);
  for (int i=0; i<N_p; i+=3 ){
    Eigen::Vector3d xs = pf->GetParticleState(i).position;
    double xss[3] = {xs[0], xs[1], xs[2]};

    bot_lcmgl_sphere(lcmgl_, xss, 0.01, 100, 100);
  }
  bot_lcmgl_switch_buffer(lcmgl_);
}

void
PFGrasp::runOneIter(){
  if(this->bearing_a_ == 0 || this->bearing_b_ == 0)
    return;

  std::cout << "dbg-runOneIter1" << std::endl;
  pf->MoveParticles();
  std::cout << "dbg-runOneIter2" << std::endl;
  pf->UpdateWithLogLikelihoodParticles();
  std::cout << "dbg-runOneIter3" << std::endl;
  double ESS = pf->ConsiderResample();
  std::cout << "dbg-runOneIter3-1" << std::endl;
  // use lcmgl to draw particles, and mean estimation

  bot_lcmgl_color3f(lcmgl_, 1,0,1);
  for (int i=0; i<N_p; i+=3 ){
    Eigen::Vector3d xs = pf->GetParticleState(i).position;
    double xss[3] = {xs[0], xs[1], xs[2]};

    bot_lcmgl_sphere(lcmgl_, xss, 0.01, 100, 100);
  }


  Eigen::Vector3d xh = pf->Integrate().position;
  std::cout << "dbg-runOneIter3-2 xh" << xh[0] << " " << xh[1] << " " << xh[2] << std::endl;
  double xhh[3] = {xh[0], xh[1], xh[2]};
  bot_lcmgl_color3f(lcmgl_, 1,0,0);
  bot_lcmgl_sphere(lcmgl_, xhh, 0.05, 100, 100);

  std::cout << "dbg-runOneIter4" << std::endl;

  // We've got xh
  // get handface pose
  double hpose_[16];
  Eigen::Matrix4d hpose;
  //Eigen::Matrix<double, 4, 4, RowMajor> hpose;
  bot_frames_get_trans_mat_4x4(this->botFrames_, options_.reachGoalFrameName.c_str(), "local", hpose.data());
  hpose.transposeInPlace();  // eigen store matrix in column major by default
  BotTrans bt;
  bot_frames_get_trans(this->botFrames_, options_.reachGoalFrameName.c_str(), "local", &bt);
  cout << "dbg-mat:" << hpose << endl;

  typedef Eigen::Vector3d V;
  typedef Eigen::Matrix3d M;
  V hpos = hpose.block(0,3,3,1);  // location of hand
  V delta = xh - hpos;
  V ux = hpose.block(0,0,3,1);
  V uy = hpose.block(0,1,3,1);
  V uz = hpose.block(0,2,3,1);
  V dz = uz.dot(delta) * uz;   // from the hand, z is horizontal, y points forward, x points downward
  V dx = ux.dot(delta) * ux;   // so we want z, and x adjustment
  V newhpos = hpos + dx + dz;

  cout << "dbg-newhpos:" << newhpos << endl;
  bot_lcmgl_color3f(lcmgl_, 1,1,0);
  bot_lcmgl_sphere(lcmgl_, newhpos.data(), 0.03, 100, 100);

  bt.trans_vec[0] = newhpos[0];
  bt.trans_vec[1] = newhpos[1];
  bt.trans_vec[2] = newhpos[2];

  bot_lcmgl_switch_buffer(lcmgl_);
  publishHandReachGoal(bt);
/*
  delta = (xk - campose(1:3,k));
           camUnitVec = inv(quat2dcm(campose(4:7,k)'));
           dx = camUnitVec(:,1)' * delta * camUnitVec(:,1);  % dx on camera frame wrt world
           dy = camUnitVec(:,2)' * delta * camUnitVec(:,2);  % dy on camera frame wrt world if we want to use
           new_campose = [campose(1:3,k) + dx ; campose(4:7,k)];  % camera orient stay the same

           sendNewCampose(new_campose, LR);
           fprintf('old campose: %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n', campose(1:7,k));
           fprintf('new campose: %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n', new_campose(1:7));

           lcmgl.glColor3f(1,1,0);
           lcmgl.sphere(new_campose,0.03,100,100);
           */

}


void
PFGrasp::publishHandReachGoal(const BotTrans& bt){

  bot_frames::update_t msg;
  msg.utime = bot_timestamp_now();
  msg.frame = options_.reachGoalFrameName;
  msg.relative_to = "local";
  std::copy_n(bt.trans_vec, 3, msg.trans);
  std::copy_n(bt.rot_quat, 4, msg.quat);

  this->lcm_->publish(options_.reachGoalChannelName, &msg);
}

PFGrasp::PFGrasp(PFGraspOptions options) :
    options_(options), pf(NULL), bearing_a_(0), bearing_b_(0)
{
  // should move into options
  bound = 0.5;
  rng_seed = 1;
  resample_threshold = 0.5;
  N_p = 1000;

  lcmWrapper_.reset(new drc::LcmWrapper());
  lcm_ = lcmWrapper_->get();
  botWrapper_.reset(new drc::BotWrapper(lcm_));
  botFrames_ = botWrapper_->getBotFrames();
  lcmgl_ = bot_lcmgl_init(lcm_->getUnderlyingLCM(), "pfgrasp");

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
  // Initialize TLD tracker
  tracker_ = new TLDTracker(cameraParams_.width, cameraParams_.height,
      options_.scale);

  // Initialize Image warper
  warper_ = new ImageWarper(options_.cameraChannelName, &warpedCamTrans);

  counter_ = 0;
}

int
main(int argc, char** argv)
{
  PFGraspOptions options;

  ConciseArgs opt(argc, (char**) argv);
  opt.add(options.cameraChannelName, "c", "camera-channel",  "Camera Channel [CAMERALHAND]");
  opt.add(options.scale, "s", "scale", "TLD tracker scale Factor");
  opt.add(options.reachGoalFrameName, "g", "plan-frame", "Planning on what frame [LHAND_FACE]");
  opt.add(options.commandChannelName, "m", "command", "Listen command from which channel [PFGRASP_CMD]");

  opt.add(options.debug, "d", "debug", "Debug");
  opt.parse();

  PFGrasp pfgrasp(options);

  std::cout << "\npfgrasp ready\n";
  pfgrasp.start();
  return 0;
}
