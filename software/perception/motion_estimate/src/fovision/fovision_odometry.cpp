// Demonstrations application for fovis wrapper library - fovision
#include <zlib.h>
#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/multisense.hpp>
#include <lcmtypes/microstrain_comm.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include "drcvision/voconfig.hpp"
#include "drcvision/vofeatures.hpp"
#include "drcvision/voestimator.hpp"
#include "fovision.hpp"

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <ConciseArgs>

#include <opencv/cv.h> // for disparity 

using namespace std;

using namespace cv; // for disparity ops

class StereoOdom{
  public:
    StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_, int imu_fusion_mode_, string camera_config_);//lcm_t* publish_lcm);
    
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
    void updateMotion();
    
    string camera_config_; // which block from the cfg to read
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    void multisenseLRHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg);   
    void multisenseLDHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg);   
    
    
    int imu_fusion_mode_;
    bool imu_initialized_;
    int imu_counter_;
    void microstrainHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  microstrain::ins_t* msg);
    void gazeboHeadIMUHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::imu_t* msg);
    void fuseInterial(Eigen::Quaterniond imu_robotorientation,int correction_frequency);
};    

StereoOdom::StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_, int imu_fusion_mode_, string camera_config_) : 
       lcm_(lcm_), utime_cur_(0), utime_prev_(0), 
       ref_utime_(0), changed_ref_frames_(false), 
       imu_fusion_mode_( imu_fusion_mode_ ), camera_config_(camera_config_)
{
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  // Read config from file:
  config = new voconfig::KmclConfiguration(botparam_, camera_config_);

  boost::shared_ptr<fovis::StereoCalibration> stereo_calibration_;
  stereo_calibration_ = boost::shared_ptr<fovis::StereoCalibration>(config->load_stereo_calibration());
  image_size_ = stereo_calibration_->getWidth() * stereo_calibration_->getHeight();
  left_buf_ = (uint8_t*) malloc(3*image_size_);
  right_buf_ = (uint8_t*) malloc(3*image_size_);

  left_buf_ref_ = (uint8_t*) malloc(3*image_size_); // used of feature output 
  
  vo_ = new FoVision(lcm_ , stereo_calibration_);

  features_ = new VoFeatures(lcm_, stereo_calibration_->getWidth(), stereo_calibration_->getHeight() );
  estimator_ = new VoEstimator(lcm_ , botframes_);

  // DEPRECATED - assumes CAMERA is image_t type:
  lcm_->subscribe("CAMERA",&StereoOdom::imageHandler,this);
  //lcm_->subscribe("CAMERA",&StereoOdom::multisenseLRHandler,this);
  lcm_->subscribe("MULTISENSE_LR",&StereoOdom::multisenseLRHandler,this);
  lcm_->subscribe("MULTISENSE_LD",&StereoOdom::multisenseLDHandler,this);
  
  // IMU:
  imu_initialized_=false;
  imu_counter_=0;
  lcm_->subscribe("MICROSTRAIN_INS",&StereoOdom::microstrainHandler,this);
  lcm_->subscribe("HEAD_IMU",&StereoOdom::gazeboHeadIMUHandler,this);
  
  cout <<"StereoOdom Constructed\n";
}


int counter =0;
void StereoOdom::featureAnalysis(){

  /// Incremental Feature Output:
  if (counter%5 == 0 ){
    features_->setFeatures(vo_->getMatches(), vo_->getNumMatches() , utime_cur_);
    features_->setCurrentImage(left_buf_);
    //features_->setCurrentImages(left_buf_, right_buf_);
    features_->setCurrentCameraPose( estimator_->getCameraPose() );
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
    ref_utime_ = utime_cur_;
    ref_camera_pose_ = estimator_->getCameraPose(); // publish this pose when the 
    // TODO: only copy gray data if its grey
    std::copy( left_buf_ , left_buf_ + 3*image_size_  , left_buf_ref_); // Keep the image buffer to write with the features:
    changed_ref_frames_=true;
  }
  counter++;
}

void StereoOdom::updateMotion(){
  Eigen::Isometry3d delta_camera;
  Eigen::MatrixXd delta_cov;
  fovis::MotionEstimateStatusCode delta_status;
  vo_->getMotion(delta_camera, delta_cov, delta_status );
  vo_->fovis_stats();
  estimator_->voUpdate(utime_cur_, delta_camera);
  
  stringstream ss;
  ss << "Number of Visual Odometry inliers: " << vo_->getNumInliers();
  drc::system_status_t status_msg;
  status_msg.utime =  utime_cur_;
  status_msg.system = 1;// use enums!!
  status_msg.importance = 0;// use enums!!
  status_msg.frequency = 1;// use enums!!
  status_msg.value = ss.str();
  lcm_->publish("SYSTEM_STATUS", &status_msg);
  
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
void StereoOdom::imageHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  bot_core::image_t* msg){
  utime_prev_ = utime_cur_;
  utime_cur_ = msg->utime;
  memcpy(left_buf_,  msg->data.data() , msg->size/2);
  memcpy(right_buf_,  msg->data.data() + msg->size/2 , msg->size/2);
  vo_->doOdometry(left_buf_,right_buf_);
  updateMotion();
  featureAnalysis();
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
  
  vo_->doOdometry(left_buf_,right_buf_);
  updateMotion();
  featureAnalysis();
}

void StereoOdom::multisenseLDHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  multisense::images_t* msg){
  int w = msg->images[0].width;
  int h = msg->images[0].height;

  cout <<"disparity mode distabled for now\n";
  exit(-1);
//disabledfornow
/*  utime_prev_ = utime_cur_;
  utime_cur_ = msg->utime;
  memcpy(left_buf_,  msg->images[0].data.data() , msg->images[0].size);
  
  // Convert Carnegie disparity format into floating point disparity. Store in local buffer
  Mat disparity_orig_temp = Mat::zeros(h,w,CV_16UC1); // h,w
  const uint8_t* raw_data = msg->images[1].data.data();
  disparity_orig_temp.data = (uchar*) raw_data;   // ... is a simple assignment possible?  
  cv::Mat_<float> disparity_orig(h, w);
  disparity_orig = disparity_orig_temp;
  disparity_buf_.resize(h * w);
  cv::Mat_<float> disparity(h, w, &(disparity_buf_[0]));
  disparity = disparity_orig / 16.0;
  
  vo_->doOdometry(left_buf_,disparity_buf_.data() );
  updateMotion();
  featureAnalysis();
*/
}




// Transform the Gazebo head IMU orientation into the head frame:
// TODO: add an imu frame into the config
Eigen::Quaterniond gazeboHeadIMUToRobotOrientation(const drc::imu_t *msg){
  Eigen::Quaterniond m(msg->orientation[0],msg->orientation[1],msg->orientation[2],msg->orientation[3]);
  return m;
}

// Transform the Microstrain IMU orientation into the head frame:
// TODO: add an imu frame into the config
Eigen::Quaterniond microstrainIMUToRobotOrientation(const microstrain::ins_t *msg){
  Eigen::Quaterniond m(msg->quat[0],msg->quat[1],msg->quat[2],msg->quat[3]);
  Eigen::Isometry3d motion_estimate;
  motion_estimate.setIdentity();
  motion_estimate.translation() << 0,0,0;
  motion_estimate.rotate(m);

  // rotate coordinate frame so that look vector is +X, and up is +Z
  Eigen::Matrix3d M;

  //convert imu on bocelli cane from:
  //x+ right, z+ down, y+ back
  //to x+ forward, y+ left, z+ up (robotics)
  //M <<  0,  -1, 0,
  //    -1, 0, 0,
  //    0, 0, -1;

  //convert imu on drc rig from:
  //x+ back, z+ down, y+ left
  //to x+ forward, y+ left, z+ up (robotics)
  M <<  -1,  0, 0,
      0, 1, 0,
      0, 0, -1;

  motion_estimate= M * motion_estimate;
  Eigen::Vector3d translation(motion_estimate.translation());
  Eigen::Quaterniond rotation(motion_estimate.rotation());
  rotation = rotation * M.transpose();

  Eigen::Isometry3d motion_estimate_out;
  motion_estimate_out.setIdentity();
  motion_estimate_out.translation() << translation[0],translation[1],translation[2];
  motion_estimate_out.rotate(rotation);
  Eigen::Quaterniond r_x(motion_estimate_out.rotation());

  return r_x;
}

void StereoOdom::fuseInterial(Eigen::Quaterniond imu_robotorientation,int correction_frequency){
  if (imu_fusion_mode_ >0 ){
    if (!imu_initialized_){
      Eigen::Isometry3d init_pose;
      init_pose.setIdentity();
      init_pose.translation() << 0,0,0;
      init_pose.rotate( imu_robotorientation );
      estimator_->setHeadPose(init_pose);
      imu_initialized_ = true;
      cout << "got first IMU measurement\n";
    }else if(imu_fusion_mode_ ==2){
      if (imu_counter_== correction_frequency){
        // Every X frames: replace the pitch and roll with that from the IMU
        // convert the camera pose to body frame
        // extract xyz and yaw from body frame
        // extract pitch and roll from imu (in body frame)
        // combine, convert to camera frame... set as pose
        bool verbose = false;
        
        Eigen::Isometry3d local_to_head = estimator_->getHeadPose();// _local_to_camera *cam2head;
        std::stringstream ss2;
        print_Isometry3d(local_to_head, ss2);
        double ypr[3];
        quat_to_euler(  Eigen::Quaterniond(local_to_head.rotation()) , ypr[0], ypr[1], ypr[2]);
        
        if (verbose){
          std::cout << "_local_to_head: " << ss2.str() << " | "<< 
            ypr[0]*180/M_PI << " " << ypr[1]*180/M_PI << " " << ypr[2]*180/M_PI << "\n";        

          double xyz[3];
          xyz[0] = local_to_head.translation().x();
          xyz[1] = local_to_head.translation().y();
          xyz[2] = local_to_head.translation().z();
          std::cout << xyz[0] << " " << xyz[1] << " " << xyz[2] << " pose xyz\n";
        }
          
        double ypr_imu[3];
        quat_to_euler( imu_robotorientation , 
                        ypr_imu[0], ypr_imu[1], ypr_imu[2]);
        if (verbose){
          std::cout <<  ypr_imu[0]*180/M_PI << " " << ypr_imu[1]*180/M_PI << " " << ypr_imu[2]*180/M_PI << " imuypr\n";        
        }
        cout << "IMU correction | pitch roll | was: "
             << ypr[1]*180/M_PI << " " << ypr[2]*180/M_PI << " | now: "
             << ypr_imu[1]*180/M_PI << " " << ypr_imu[2]*180/M_PI << "\n";
        
        Eigen::Quaterniond revised_local_to_head_quat = euler_to_quat( ypr[0], ypr_imu[1], ypr_imu[2]);             
        Eigen::Isometry3d revised_local_to_head;
        revised_local_to_head.setIdentity();
        revised_local_to_head.translation() = local_to_head.translation();
        revised_local_to_head.rotate(revised_local_to_head_quat);
        
        if (verbose){
          std::stringstream ss4;
          print_Isometry3d(revised_local_to_head, ss4);
          quat_to_euler(  Eigen::Quaterniond(revised_local_to_head.rotation()) , ypr[0], ypr[1], ypr[2]);
          std::cout << "_local_revhead: " << ss4.str() << " | "<< 
            ypr[0]*180/M_PI << " " << ypr[1]*180/M_PI << " " << ypr[2]*180/M_PI << "\n";        
        }
        estimator_->setHeadPose(revised_local_to_head);
      }
      if (imu_counter_ > correction_frequency) { imu_counter_ =0; }
      imu_counter_++;
    }
  }else{
//    cout << "got IMU measurement - not incorporating them\n";
  }  
}

void StereoOdom::microstrainHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  microstrain::ins_t* msg){
  Eigen::Quaterniond imu_robotorientation;
  imu_robotorientation =microstrainIMUToRobotOrientation(msg);
  int correction_frequency=100;
  fuseInterial(imu_robotorientation, correction_frequency);
}

void StereoOdom::gazeboHeadIMUHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  drc::imu_t* msg){
  Eigen::Quaterniond imu_robotorientation;
  imu_robotorientation =gazeboHeadIMUToRobotOrientation(msg);
  int correction_frequency=1000;
  fuseInterial(imu_robotorientation, correction_frequency);  
}


int main(int argc, char **argv){
  ConciseArgs parser(argc, argv, "fovision-odometry");
  int imu_fusion_mode=0;
  string camera_config = "CAMERA";
  parser.add(imu_fusion_mode, "i", "imu_fusion_mode", "0 none, 1 at init, 2 every second");
  parser.add(camera_config, "c", "camera_config", "Camera Config block to use: CAMERA, stereo, stereo_with_letterbox");
  parser.parse();
  cout << imu_fusion_mode << " is imu_fusion_mode\n"; 
  cout << camera_config << " is camera_config\n"; 
  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  StereoOdom fo= StereoOdom(lcm, imu_fusion_mode,camera_config);    
  while(0 == lcm->handle());
}
