// simple test program:
// se-simple-vo -L ~/drc-testing-data/state_est/run1_field_camera_snippet.lcmlog -P drc_robot.cfg -p

// Just do VO, not do any imu integration, dont correct for any urdf/cfg offsets
// Output motion in x forward, z up frame

#include <zlib.h>
#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/multisense.hpp>
#include <lcmtypes/microstrain_comm.hpp>

#include "drcvision/voconfig.hpp"
#include "drcvision/vofeatures.hpp"
#include "drcvision/voestimator.hpp"
#include "fovision.hpp"

#include <pronto_utils/pronto_vis.hpp> // visualize pt clds
#include <ConciseArgs>

/// For Forward Kinematics from body to head:
#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <model-client/model-client.hpp>
#include <path_util/path_util.h>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

#include <opencv/cv.h> // for disparity 

using namespace std;
using namespace cv; // for disparity ops

struct CommandLineConfig
{
  std::string camera_config; // which block from the cfg to read
  int fusion_mode;
  bool feature_analysis;
  std::string output_extension;
  bool output_signal;
  bool vicon_init; // initializae off of vicon
  std::string input_channel;
  bool verbose;
  std::string in_log_fname;
  std::string param_file;
};

class StereoOdom{
  public:
    StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_, const CommandLineConfig& cl_cfg_);
    
    ~StereoOdom(){
      free (left_buf_);
      free(right_buf_);
    }

  private:
    const CommandLineConfig cl_cfg_;    
    
    int image_size_; // just the resolution of the image
    uint8_t* left_buf_;
    uint8_t* right_buf_;
    mutable std::vector<float> disparity_buf_; // Is mutable necessary?
    uint8_t* rgb_buf_ ;
    uint8_t* decompress_disparity_buf_;    
    image_io_utils*  imgutils_;    
    
    int64_t utime_cur_;

    boost::shared_ptr<lcm::LCM> lcm_recv_;
    boost::shared_ptr<lcm::LCM> lcm_pub_;
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;
    voconfig::KmclConfiguration* config;

    //
    FoVision* vo_;

    //
    VoFeatures* features_;
    int64_t utime_prev_;
    uint8_t* left_buf_ref_; // copies of the reference images - probably can be extracted from fovis directly
    int64_t ref_utime_;
    Eigen::Isometry3d ref_camera_pose_; // [pose of the camera when the reference frames changed
    bool changed_ref_frames_;

    VoEstimator* estimator_;

    void featureAnalysis();
    void updateMotion(int64_t utime);

    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg);
    void multisenseLDHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg);
    void multisenseLRHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg);
    void viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg);
    
    bool pose_initialized_;
    
    Eigen::Isometry3d world_to_camera_;
    Eigen::Isometry3d world_to_body_;    

};    

StereoOdom::StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_, const CommandLineConfig& cl_cfg_) : 
       lcm_recv_(lcm_recv_), lcm_pub_(lcm_pub_), cl_cfg_(cl_cfg_), utime_cur_(0), utime_prev_(0), 
       ref_utime_(0), changed_ref_frames_(false)
{
  if (cl_cfg_.param_file.empty()) {
    std::cout << "Get param from LCM\n";
    botparam_ = bot_param_get_global(lcm_recv_->getUnderlyingLCM(), 0);
  } else {
    std::cout << "Get param from file\n";
    botparam_ = bot_param_new_from_file(cl_cfg_.param_file.c_str());
  }
  if (botparam_ == NULL) {
    exit(1);
  }
  botframes_ = bot_frames_get_global(lcm_recv_->getUnderlyingLCM(), botparam_);
  botframes_cpp_ = new bot::frames(botframes_);


  
  // Read config from file:
  config = new voconfig::KmclConfiguration(botparam_, cl_cfg_.camera_config);

  boost::shared_ptr<fovis::StereoCalibration> stereo_calibration_;
  stereo_calibration_ = boost::shared_ptr<fovis::StereoCalibration>(config->load_stereo_calibration());
  image_size_ = stereo_calibration_->getWidth() * stereo_calibration_->getHeight();
  left_buf_ = (uint8_t*) malloc(3*image_size_);
  right_buf_ = (uint8_t*) malloc(3*image_size_);
  left_buf_ref_ = (uint8_t*) malloc(3*image_size_); // used of feature output 
  rgb_buf_ = (uint8_t*) malloc(10*image_size_ * sizeof(uint8_t)); 
  decompress_disparity_buf_ = (uint8_t*) malloc( 4*image_size_*sizeof(uint8_t));  // arbitary size chosen..
  

  vo_ = new FoVision(lcm_pub_ , stereo_calibration_);
  features_ = new VoFeatures(lcm_pub_, stereo_calibration_->getWidth(), stereo_calibration_->getHeight() );
  estimator_ = new VoEstimator(lcm_pub_ , botframes_, cl_cfg_.output_extension );
  lcm_recv_->subscribe( cl_cfg_.input_channel,&StereoOdom::multisenseHandler,this);

 
  pose_initialized_ = false;
  if (!cl_cfg_.vicon_init){
    std::cout << "Init internal est using default\n";
    // Initialise the nominal camera frame with the head pointing horizontally
    Eigen::Matrix3d M;
    M <<  0,  0, 1,
        -1,  0, 0,
          0, -1, 0;

    world_to_camera_ = M * Eigen::Isometry3d::Identity();
    world_to_camera_.translation().x() = 0;
    world_to_camera_.translation().y() = 0;
    world_to_camera_.translation().z() = 1.65; // nominal head height
    pose_initialized_ = true;
  }else{
    lcm_recv_->subscribe("VICON_BODY|VICON_FRONTPLATE",&StereoOdom::viconHandler,this);
  }
  
  
  imgutils_ = new image_io_utils( lcm_pub_->getUnderlyingLCM(), stereo_calibration_->getWidth(), 2*stereo_calibration_->getHeight()); // extra space for stereo tasks
  cout <<"StereoOdom Constructed\n";
}


int counter =0;
void StereoOdom::featureAnalysis(){

  /// Incremental Feature Output:
  if (counter%5 == 0 ){
    features_->setFeatures(vo_->getMatches(), vo_->getNumMatches() , utime_cur_);
    features_->setCurrentImage(left_buf_);
    //features_->setCurrentImages(left_buf_, right_buf_);
    //features_->setCurrentCameraPose( estimator_->getCameraPose() );
    features_->setCurrentCameraPose( world_to_camera_ );
    features_->doFeatureProcessing(1); // 1 = send the FEATURES_CUR
  }
  
  /// Reference Feature Output: ///////////////////////////////////////////////
  // Check we changed reference frame last iteration, if so output the set of matching inliers:
  if (changed_ref_frames_) {
    if (ref_utime_ > 0){ // skip the first null image
      if(vo_->getNumMatches() > 200){ // if less than 50 features - dont bother writing
      // was:      if(featuresA.size() > 50){ // if less than 50 features - dont bother writing
        cout << "ref frame from " << utime_prev_ << " at " << utime_cur_ <<  " with " <<vo_->getNumMatches()<<" matches\n";
        features_->setFeatures(vo_->getMatches(), vo_->getNumMatches() , ref_utime_);
        features_->setReferenceImage(left_buf_ref_);
        features_->setReferenceCameraPose( ref_camera_pose_ );
        features_->doFeatureProcessing(0); // 0 = send the FEATURES_REF
      }
    }
    changed_ref_frames_=false;
  }

  
  if (vo_->getChangeReferenceFrames()){ // If we change reference frame, note the change for the next iteration.
    std::cout << "ref frame from " << ref_utime_ << " to " << utime_cur_  << " " << (utime_cur_-ref_utime_)*1E-6 << "sec\n";
    ref_utime_ = utime_cur_;
    // ref_camera_pose_ = estimator_->getCameraPose(); // publish this pose when the 
    ref_camera_pose_ = world_to_camera_; // publish this pose when the 
    // TODO: only copy gray data if its grey
    std::copy( left_buf_ , left_buf_ + 3*image_size_  , left_buf_ref_); // Keep the image buffer to write with the features:
    changed_ref_frames_=true;
  }
  
  counter++;
}



void StereoOdom::updateMotion(int64_t utime){
  
  // Update the camera position in world frame
  Eigen::Isometry3d delta_camera;
  Eigen::MatrixXd delta_camera_cov;
  fovis::MotionEstimateStatusCode delta_status;
  vo_->getMotion(delta_camera, delta_camera_cov, delta_status );
  vo_->fovis_stats();
  world_to_camera_  = world_to_camera_ * delta_camera;
  
  // Determine the body position in world frame:
  Eigen::Isometry3d camera_to_body;
  int status = botframes_cpp_->get_trans_with_utime( botframes_ ,  "body", "CAMERA_LEFT"  , utime, camera_to_body);
  Eigen::Isometry3d new_world_to_body = world_to_camera_ * camera_to_body;  
  
  // Find the resultant delta by comparing the body position estimate with its previous
  Eigen::Isometry3d delta_body =  new_world_to_body * ( world_to_body_.inverse() );

  if (cl_cfg_.output_signal ){
    estimator_->publishPose(utime, "POSE_CAMERA_LEFT_ALT", world_to_camera_, Eigen::Vector3d::Identity(), Eigen::Vector3d::Identity());
    estimator_->publishPose(utime, "POSE_BODY", new_world_to_body, Eigen::Vector3d::Identity(), Eigen::Vector3d::Identity());
  }
  
  // THIS IS NOT THE CORRECT COVARIANCE - ITS THE COVARIANCE IN THE CAMERA FRAME!!!!
  vo_->send_delta_translation_msg(delta_body,
          delta_camera_cov, "VO_DELTA_BODY" );  
  
  world_to_body_ = new_world_to_body;
  
  /*
  stringstream ss;
  ss << "Number of Visual Odometry inliers: " << vo_->getNumInliers();
  drc::system_status_t status_msg;
  status_msg.utime =  utime_cur_;
  status_msg.system = 1;// use enums!!
  status_msg.importance = 0;// use enums!!
  status_msg.frequency = 1;// use enums!!
  status_msg.value = ss.str();
  lcm_pub_->publish("SYSTEM_STATUS", &status_msg);
  */
}

/// Added for RGB-to-Gray:
int pixel_convert_8u_rgb_to_8u_gray (uint8_t *dest, int dstride, int width,
        int height, const uint8_t *src, int sstride)
{
  int i, j;
  for (i=0; i<height; i++) {
    uint8_t *drow = dest + i * dstride;
    const uint8_t *srow = src + i * sstride;
    for (j=0; j<width; j++) {
      drow[j] = 0.2125 * srow[j*3+0] +
        0.7154 * srow[j*3+1] +
        0.0721 * srow[j*3+2];
    }
  }
  return 0;
}

void StereoOdom::multisenseHandler(const lcm::ReceiveBuffer* rbuf,
     const std::string& channel, const  bot_core::images_t* msg){

  if (!pose_initialized_){
    return;
  }

  utime_prev_ = utime_cur_;
  utime_cur_ = msg->utime;

  // Detect the image stream and process accordingly
  if ( (msg->image_types[0] ==  bot_core::images_t::LEFT) &&
       (msg->image_types[1] ==  bot_core::images_t::RIGHT) ) {

    multisenseLRHandler(rbuf, channel, msg);
    vo_->doOdometry(left_buf_,right_buf_, msg->utime);
  }else if( (msg->image_types[0] ==  bot_core::images_t::LEFT) &&
       (msg->image_types[1] ==  bot_core::images_t::DISPARITY_ZIPPED) ) {

    multisenseLDHandler(rbuf, channel, msg);
    vo_->doOdometry(left_buf_,disparity_buf_.data(), msg->utime );
  }else{
    std::cout << "StereoOdom::multisenseHandler | image pairings not understood\n";
    return;
  }
  updateMotion(msg->utime);

  if(cl_cfg_.feature_analysis)
    featureAnalysis();

}


void StereoOdom::multisenseLDHandler(const lcm::ReceiveBuffer* rbuf,
     const std::string& channel, const  bot_core::images_t* msg){

  int w = msg->images[0].width;
  int h = msg->images[0].height;

  if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB ){
    rgb_buf_ = (uint8_t*) msg->images[0].data.data();
  }else if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY ){
    rgb_buf_ = (uint8_t*) msg->images[0].data.data();
  }else if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG ){
    jpeg_decompress_8u_rgb ( msg->images[0].data.data(), msg->images[0].size, rgb_buf_, w, h, w* 3);
    pixel_convert_8u_rgb_to_8u_gray(  left_buf_, w, w, h, rgb_buf_,  w*3);
  }else{
    std::cout << "StereoOdom image type not understood\n";
    exit(-1);
  }

  // TODO: support other modes (as in the renderer)
  if (msg->image_types[1] == bot_core::images_t::DISPARITY_ZIPPED) {
    unsigned long dlen = w*h*2;
    uncompress(decompress_disparity_buf_ , &dlen, msg->images[1].data.data(), msg->images[1].size);
  } else{
    std::cout << "StereoOdom depth type not understood\n";
    exit(-1);
  }
  
  // Convert Carnegie disparity format into floating point disparity. Store in local buffer
  Mat disparity_orig_temp = Mat::zeros(h,w,CV_16UC1); // h,w
  disparity_orig_temp.data = (uchar*) decompress_disparity_buf_;   // ... is a simple assignment possible?  
  cv::Mat_<float> disparity_orig(h, w);
  disparity_orig = disparity_orig_temp;
  disparity_buf_.resize(h * w);
  cv::Mat_<float> disparity(h, w, &(disparity_buf_[0]));
  disparity = disparity_orig / 16.0;
  
  return;
}


void StereoOdom::multisenseLRHandler(const lcm::ReceiveBuffer* rbuf,
     const std::string& channel, const  bot_core::images_t* msg){

  int w = msg->images[0].width;
  int h = msg->images[0].height;

  if (msg->images[0].pixelformat != msg->images[1].pixelformat){
    std::cout << "Pixel formats not identical, not supported\n";
    exit(-1);
  }

  switch (msg->images[0].pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY:
      memcpy(left_buf_,  msg->images[0].data.data() , msg->images[0].size);
      memcpy(right_buf_,  msg->images[1].data.data() , msg->images[1].size);
      break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
      // image came in as raw RGB buffer.  convert to grayscale:
      pixel_convert_8u_rgb_to_8u_gray(  left_buf_ , w, w, h, msg->images[0].data.data(),  w*3);
      pixel_convert_8u_rgb_to_8u_gray(  right_buf_, w, w, h, msg->images[1].data.data(),  w*3);
      break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
      // not sure why but the same setting seem to work for both jpeg compressed color (left) and grey (right):
      jpeg_decompress_8u_gray(msg->images[0].data.data(), msg->images[0].size,
                              left_buf_, w, h, w);
      jpeg_decompress_8u_gray(msg->images[1].data.data(), msg->images[1].size,
                              right_buf_, w, h, w);
      break;
    default:
      std::cout << "Unrecognized image format\n";
      exit(-1);
      break;
  }

  return;
}



static inline bot_core::pose_t getPoseAsBotPose(Eigen::Isometry3d pose, int64_t utime){
  bot_core::pose_t pose_msg;
  pose_msg.utime =   utime;
  pose_msg.pos[0] = pose.translation().x();
  pose_msg.pos[1] = pose.translation().y();
  pose_msg.pos[2] = pose.translation().z();  
  Eigen::Quaterniond r_x(pose.rotation());
  pose_msg.orientation[0] =  r_x.w();  
  pose_msg.orientation[1] =  r_x.x();  
  pose_msg.orientation[2] =  r_x.y();  
  pose_msg.orientation[3] =  r_x.z();  
  return pose_msg;
}


void StereoOdom::viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg){
  
  if ( cl_cfg_.vicon_init && !pose_initialized_ ){
    std::cout << "Init internal est using Vicon\n";
  
    Eigen::Isometry3d worldvicon_to_frontplate_vicon;
    worldvicon_to_frontplate_vicon.setIdentity();
    worldvicon_to_frontplate_vicon.translation()  << msg->trans[0], msg->trans[1] , msg->trans[2];
    Eigen::Quaterniond quat = Eigen::Quaterniond(msg->quat[0], msg->quat[1], 
                                                msg->quat[2], msg->quat[3]);
    worldvicon_to_frontplate_vicon.rotate(quat); 
    
    // Apply the body to frontplate transform
    Eigen::Isometry3d frontplate_vicon_to_body_vicon = botframes_cpp_->get_trans_with_utime(botframes_, "body_vicon" , "frontplate_vicon", msg->utime);
    Eigen::Isometry3d body_to_camera = botframes_cpp_->get_trans_with_utime(botframes_, "CAMERA_LEFT" , "body", msg->utime);

    Eigen::Isometry3d worldvicon_to_camera = worldvicon_to_frontplate_vicon* frontplate_vicon_to_body_vicon * body_to_camera;
    
    bot_core::pose_t pose_msg = getPoseAsBotPose(worldvicon_to_camera, msg->utime);
    lcm_pub_->publish("POSE_BODY_ALT", &pose_msg );
    
    world_to_camera_ =  worldvicon_to_camera;
    // prev_vicon_utime_ = msg->utime;
    
    pose_initialized_ = TRUE;
  }

}





int main(int argc, char **argv){
  CommandLineConfig cl_cfg;
  cl_cfg.camera_config = "CAMERA";
  cl_cfg.input_channel = "CAMERA";
  cl_cfg.output_signal = FALSE;
  cl_cfg.feature_analysis = FALSE; 
  cl_cfg.vicon_init = FALSE;
  cl_cfg.fusion_mode = 0;
  cl_cfg.output_extension = "";
  cl_cfg.in_log_fname = "";
  std::string param_file = ""; // actual file
  cl_cfg.param_file = ""; // full path to file

  ConciseArgs parser(argc, argv, "fovision-odometry");
  parser.add(cl_cfg.camera_config, "c", "camera_config", "Camera Config block to use: CAMERA, stereo, stereo_with_letterbox");
  parser.add(cl_cfg.output_signal, "p", "output_signal", "Output POSE_BODY and POSE_BODY_ALT signals");
  parser.add(cl_cfg.feature_analysis, "f", "feature_analysis", "Publish Feature Analysis Data");
  parser.add(cl_cfg.vicon_init, "g", "vicon_init", "Bootstrap internal estimate using VICON_FRONTPLATE");
  parser.add(cl_cfg.fusion_mode, "m", "fusion_mode", "0 none, 1 at init, 2 every second, 3 init from gt, then every second");
  parser.add(cl_cfg.input_channel, "i", "input_channel", "input_channel - CAMERA or CAMERA_BLACKENED");
  parser.add(cl_cfg.output_extension, "o", "output_extension", "Extension to pose channels (e.g. '_VO' ");
  parser.add(cl_cfg.in_log_fname, "L", "in_log_fname", "Process this log file");
  parser.add(param_file, "P", "param_file", "Pull params from this file instead of LCM");
  parser.parse();
  cout << cl_cfg.fusion_mode << " is fusion_mode\n";
  cout << cl_cfg.camera_config << " is camera_config\n";
  
  cl_cfg.param_file = std::string(getConfigPath()) +'/' + std::string(param_file);
  if (param_file.empty()) { // get param from lcm
    cl_cfg.param_file = "";
  }

  //
  bool running_from_log = !cl_cfg.in_log_fname.empty();
  boost::shared_ptr<lcm::LCM> lcm_recv;
  boost::shared_ptr<lcm::LCM> lcm_pub;
  if (running_from_log) {
    printf("running from log file: %s\n", cl_cfg.in_log_fname.c_str());
    //std::string lcmurl = "file://" + in_log_fname + "?speed=0";
    std::stringstream lcmurl;
    //lcmurl << "file://" << in_log_fname << "?speed=" << processing_rate << "&start_timestamp=" << begin_timestamp;
    lcmurl << "file://" << cl_cfg.in_log_fname ;
    lcm_recv = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcmurl.str()));
    if (!lcm_recv->good()) {
      fprintf(stderr, "Error couldn't load log file %s\n", lcmurl.str().c_str());
      exit(1);
    }
  }
  else {
    lcm_recv = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
  }
  lcm_pub = boost::shared_ptr<lcm::LCM>(new lcm::LCM);

  StereoOdom fo= StereoOdom(lcm_recv, lcm_pub, cl_cfg);
  while(0 == lcm_recv->handle());
}
