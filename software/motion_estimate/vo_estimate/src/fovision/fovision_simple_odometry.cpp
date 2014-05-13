// Main VO module



#include <zlib.h>
#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/multisense.hpp>
#include <lcmtypes/microstrain_comm.hpp>
#include "lcmtypes/drc/robot_state_t.hpp"
#include "lcmtypes/drc/imu_t.hpp"

#include "drcvision/voconfig.hpp"
#include "drcvision/vofeatures.hpp"
#include "fovision.hpp"

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <ConciseArgs>

/// For Forward Kinematics from body to head:
#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <model-client/model-client.hpp>

#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

#include <opencv/cv.h> // for disparity 
using namespace std;
using namespace cv; // for disparity ops

class StereoOdom{
  public:
    StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_, string camera_config_,
      bool output_signal_);
    
    ~StereoOdom(){
      free (left_buf_);
      free(right_buf_);
    }

  private:
    int image_size_; // just the resolution of the image
    uint8_t* left_buf_;
    uint8_t* right_buf_;

    // Is mutable necessary?
    mutable std::vector<float> disparity_buf_;    
    
    int64_t utime_cur_;

    boost::shared_ptr<lcm::LCM> lcm_;
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

    bool output_signal_;

    void featureAnalysis();
    void updateMotion(int64_t utime);
    
    string camera_config_; // which block from the cfg to read
    
    void unpack_multisense(const multisense::images_t *msg);
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    void multisenseLRHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg);   
    void multisenseLDHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg);   
    void publishPose(Eigen::Isometry3d pose, int64_t utime, std::string channel);
    
    boost::shared_ptr<ModelClient> model_;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
    
    bool pose_initialized_;
    int imu_counter_;

    drc::robot_state_t last_robot_state_msg_;
    
  
    image_io_utils*  imgutils_;    
    
    uint8_t* rgb_buf_ ;
    uint8_t* decompress_disparity_buf_;    
    
    Eigen::Isometry3d world_to_camera_;
    Eigen::Isometry3d world_to_body_;
    
};    

StereoOdom::StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_, string camera_config_,
       bool output_signal_) : 
       lcm_(lcm_), utime_cur_(0), utime_prev_(0), 
       ref_utime_(0), changed_ref_frames_(false), 
       camera_config_(camera_config_),
       output_signal_(output_signal_)
{
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  KDL::Tree tree;
  if (!kdl_parser::treeFromString( model_->getURDFString() ,tree)){
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    exit(-1);
  }
  fksolver_ = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
  
  // Read config from file:
  config = new voconfig::KmclConfiguration(botparam_, camera_config_);
  std::cout << "tip\n";

  boost::shared_ptr<fovis::StereoCalibration> stereo_calibration_;
  stereo_calibration_ = boost::shared_ptr<fovis::StereoCalibration>(config->load_stereo_calibration());
  image_size_ = stereo_calibration_->getWidth() * stereo_calibration_->getHeight();
  left_buf_ = (uint8_t*) malloc(3*image_size_);
  right_buf_ = (uint8_t*) malloc(3*image_size_);

  left_buf_ref_ = (uint8_t*) malloc(3*image_size_); // used of feature output 
  
  vo_ = new FoVision(lcm_ , stereo_calibration_);
  features_ = new VoFeatures(lcm_, stereo_calibration_->getWidth(), stereo_calibration_->getHeight() );
  
  // Assumes CAMERA is image_t type:
  //lcm_->subscribe("CAMERA",&StereoOdom::imageHandler,this);
  //lcm_->subscribe("MULTISENSE_LR",&StereoOdom::multisenseLRHandler,this);
  lcm_->subscribe("CAMERA",&StereoOdom::multisenseLDHandler,this);
  
  cout <<"StereoOdom Constructed\n";
  
  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), stereo_calibration_->getWidth(), 2*stereo_calibration_->getHeight()); // extra space for stereo tasks
 
  
  rgb_buf_ = (uint8_t*) malloc(10*1024 * 1024 * sizeof(uint8_t)); 
  decompress_disparity_buf_ = (uint8_t*) malloc( 4*1024*1204*sizeof(uint8_t));  // arbitary size chosen..
  
  
  
  // Initialise the nominal camera frame with the head pointing horizontally
  Eigen::Matrix3d M;
  M <<  0,  0, 1,
       -1,  0, 0,
        0, -1, 0;

  world_to_camera_ = M * Eigen::Isometry3d::Identity();
  world_to_camera_.translation().x() = 0;
  world_to_camera_.translation().y() = 0;
  world_to_camera_.translation().z() = 1.65; // nominal head height
  
  /*
  Eigen::Vector3d translation(cam_to_local.translation());
  Eigen::Quaterniond rotation;
  rotation  << 0,0,0,0;
  rotation = rotation * M.transpose();  
  */
  
}



void StereoOdom::publishPose(Eigen::Isometry3d pose, int64_t utime, std::string channel){
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
  lcm_->publish( channel, &pose_msg);
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

  if (output_signal_){
    publishPose(world_to_camera_, utime, "POSE_BODY_ALT"); // same as CAMERA_LEFT frame solved by state-sync
    publishPose(new_world_to_body, utime, "POSE_BODY");
  }
  
  // THIS IS NOT THE CORRECT COVARIANCE - ITS THE COVARIANCE IN THE CAMERA FRAME!!!!
  vo_->send_delta_translation_msg(delta_body,
          delta_camera_cov, "VO_DELTA_BODY" );  
  
  world_to_body_ = new_world_to_body;
  
  return; 
  
  
  /*
  stringstream ss;
  ss << "Number of Visual Odometry inliers: " << vo_->getNumInliers();
  drc::system_status_t status_msg;
  status_msg.utime =  utime_cur_;
  status_msg.system = 1;// use enums!!
  status_msg.importance = 0;// use enums!!
  status_msg.frequency = 1;// use enums!!
  status_msg.value = ss.str();
  lcm_->publish("SYSTEM_STATUS", &status_msg);
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

////// Camera handlers:
int counter_cam=0;
// stacked LR image
void StereoOdom::imageHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  bot_core::image_t* msg){
  utime_prev_ = utime_cur_;
  utime_cur_ = msg->utime;
  
  if (msg->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG){
    if (counter_cam % 10 ==0){
      std::cout << channel << " "<< msg->utime << " frame:"<< counter_cam << " MJPEG received\n"; 
    }
    counter_cam++;
    imgutils_->decodeStereoImageToGray(msg, left_buf_, right_buf_);
   
  }else{
    std::cout << "camera not MJPEG - not supported/tested\n"; 
    exit(-1);
    //memcpy(left_buf_,  msg->data.data() , msg->size/2);
    //memcpy(right_buf_,  msg->data.data() + msg->size/2 , msg->size/2);
  }
  
  
  vo_->doOdometry(left_buf_,right_buf_, msg->utime);
  updateMotion(msg->utime);
  
  return; // full estimator VO disabled for VRC competition, 8 june 2013
}

void StereoOdom::multisenseLRHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  multisense::images_t* msg){
  utime_prev_ = utime_cur_;
  utime_cur_ = msg->utime;
  
  
  int w = msg->images[0].width;
  int h = msg->images[0].height;
  
  // extract image data
  switch (msg->images[0].pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY:
      memcpy(left_buf_,  msg->images[0].data.data() , msg->images[0].size);
      memcpy(right_buf_,  msg->images[1].data.data() , msg->images[1].size);
      break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
      // image came in as raw RGB buffer.  convert to grayscale:
      pixel_convert_8u_rgb_to_8u_gray(  left_buf_, w,          w, h, msg->images[0].data.data(),  w*3);
      pixel_convert_8u_rgb_to_8u_gray(  right_buf_, w,          w, h, msg->images[1].data.data(),  w*3);
      break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
      fprintf(stderr, "MPEG not supported yet\n");
      /*// for some reason msg->row_stride is 0, so we use msg->width instead.
      jpeg_decompress_8u_gray(msg->data,
                              msg->size,
                              _images_buf,
                              msg->width,
                              msg->height,
                              msg->width);
      std::copy(_images_buf           , _images_buf+_buf_size   , _image_left_buf);
      std::copy(_images_buf+_buf_size , _images_buf+2*_buf_size , _image_right_buf);
      */
      break;
    default:
      fprintf(stderr, "Unrecognized image format\n");
      break;
  }  
  
  vo_->doOdometry(left_buf_,right_buf_, msg->utime);
  updateMotion(msg->utime);
  
  return; // full estimator VO disabled for VRC competition, 8 june 2013
}





void StereoOdom::unpack_multisense(const multisense::images_t *msg){
  bool is_rgb=true;
  if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB ){
    rgb_buf_ = (uint8_t*) msg->images[0].data.data();
    is_rgb = true;
  }else if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY ){
    rgb_buf_ = (uint8_t*) msg->images[0].data.data();
    is_rgb = false;
  }else if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG ){
    jpeg_decompress_8u_rgb ( msg->images[0].data.data(), msg->images[0].size,
        rgb_buf_, msg->images[0].width, msg->images[0].height, msg->images[0].width* 3);
    
    pixel_convert_8u_rgb_to_8u_gray(  left_buf_, msg->images[0].width, msg->images[0].width, msg->images[0].height, 
                                      rgb_buf_,  msg->images[0].width*3);
    
    //jpegijg_decompress_8u_rgb(msg->image.image_data, msg->image.image_data_nbytes,
    //        rgb_data, msg->image.width, msg->image.height, msg->image.width* 3);
    is_rgb = true;
  }else{
    std::cout << "pointcloud_lcm::unpack_multisense | image type not understood\n";
    exit(-1);
  }
  
  // TODO: support other modes (as in the renderer)
  if (msg->image_types[1] == multisense::images_t::DISPARITY_ZIPPED) {
    unsigned long dlen = msg->images[0].width*msg->images[0].height*2 ;//msg->depth.uncompressed_size;
    uncompress(decompress_disparity_buf_ , &dlen, msg->images[1].data.data(), msg->images[1].size);
  } else{
    std::cout << "pointcloud_lcm::unpack_multisense | depth type not understood\n";
    exit(-1);
  }
 
}


void StereoOdom::multisenseLDHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  multisense::images_t* msg){
  int w = msg->images[0].width;
  int h = msg->images[0].height;

  utime_prev_ = utime_cur_;
  utime_cur_ = msg->utime;
  
  unpack_multisense(msg);
  
//  memcpy(left_buf_,  msg->images[0].data.data() , msg->images[0].size);
  
  // Convert Carnegie disparity format into floating point disparity. Store in local buffer
  Mat disparity_orig_temp = Mat::zeros(h,w,CV_16UC1); // h,w
//  const uint8_t* raw_data = msg->images[1].data.data();
  disparity_orig_temp.data = (uchar*) decompress_disparity_buf_;   // ... is a simple assignment possible?  
  cv::Mat_<float> disparity_orig(h, w);
  disparity_orig = disparity_orig_temp;
  disparity_buf_.resize(h * w);
  cv::Mat_<float> disparity(h, w, &(disparity_buf_[0]));
  disparity = disparity_orig / 16.0;
  
  vo_->doOdometry(left_buf_,disparity_buf_.data(), msg->utime );
  updateMotion(msg->utime);

  return; // full estimator VO disabled for VRC competition, 8 june 2013
}



int main(int argc, char **argv){
  ConciseArgs parser(argc, argv, "fovision-odometry");
  string camera_config = "CAMERA";
  bool output_signal = FALSE;
  parser.add(camera_config, "c", "camera_config", "Camera Config block to use: CAMERA, stereo, stereo_with_letterbox");
  parser.add(output_signal, "o", "output_signal", "Output POSE_BODY and POSE_BODY_ALT signals");
  parser.parse();
  cout << camera_config << " is camera_config\n"; 
  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  StereoOdom fo= StereoOdom(lcm,camera_config, output_signal);    
  while(0 == lcm->handle());
}
