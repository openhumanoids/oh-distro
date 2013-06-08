// Main VO module

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

/// For Forward Kinematics from body to head:
#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <model-client/model-client.hpp>

#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
///

#include <opencv/cv.h> // for disparity 

using namespace std;

using namespace cv; // for disparity ops

// to be removed:
bool zheight_clamp = false;

class StereoOdom{
  public:
    StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_, int fusion_mode_, string camera_config_,
      string output_extension_);
    
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
    string output_extension_;

    void featureAnalysis();
    void updateMotion();
    
    string camera_config_; // which block from the cfg to read
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    void multisenseLRHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg);   
    void multisenseLDHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg);   
    
    void trueRobotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);
    boost::shared_ptr<ModelClient> model_;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
    bool getWorldToHead(const drc::robot_state_t* msg, Eigen::Isometry3d &local_to_head);

    
    int fusion_mode_;
    bool pose_initialized_;
    int imu_counter_;
    void microstrainHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  microstrain::ins_t* msg);
    void gazeboHeadIMUHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::imu_t* msg);
    void fuseInterial(Eigen::Quaterniond imu_robotorientation,int correction_frequency, int64_t utime);

    void gazeboBodyIMUHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::imu_t* msg);  
    drc::robot_state_t last_robot_state_msg_;
    
  
    image_io_utils*  imgutils_;    
    
};    

StereoOdom::StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_, int fusion_mode_, string camera_config_,
       string output_extension_) : 
       lcm_(lcm_), utime_cur_(0), utime_prev_(0), 
       ref_utime_(0), changed_ref_frames_(false), 
       fusion_mode_( fusion_mode_ ), camera_config_(camera_config_),
       output_extension_(output_extension_)
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

  boost::shared_ptr<fovis::StereoCalibration> stereo_calibration_;
  stereo_calibration_ = boost::shared_ptr<fovis::StereoCalibration>(config->load_stereo_calibration());
  image_size_ = stereo_calibration_->getWidth() * stereo_calibration_->getHeight();
  left_buf_ = (uint8_t*) malloc(3*image_size_);
  right_buf_ = (uint8_t*) malloc(3*image_size_);

  left_buf_ref_ = (uint8_t*) malloc(3*image_size_); // used of feature output 
  
  vo_ = new FoVision(lcm_ , stereo_calibration_);

  features_ = new VoFeatures(lcm_, stereo_calibration_->getWidth(), stereo_calibration_->getHeight() );
  estimator_ = new VoEstimator(lcm_ , botframes_, output_extension_ );

  // Assumes CAMERA is image_t type:
  lcm_->subscribe("CAMERA",&StereoOdom::imageHandler,this);
  lcm_->subscribe("MULTISENSE_LR",&StereoOdom::multisenseLRHandler,this);
  lcm_->subscribe("MULTISENSE_LD",&StereoOdom::multisenseLDHandler,this);

  // For dev of integration:
  lcm_->subscribe("TRUE_ROBOT_STATE",&StereoOdom::trueRobotStateHandler,this);
  
  // IMU:
  pose_initialized_=false;
  imu_counter_=0;
  lcm_->subscribe("MICROSTRAIN_INS",&StereoOdom::microstrainHandler,this);
  lcm_->subscribe("HEAD_IMU",&StereoOdom::gazeboHeadIMUHandler,this);
  lcm_->subscribe("TORSO_IMU",&StereoOdom::gazeboBodyIMUHandler,this);

  
  cout <<"StereoOdom Constructed\n";
  if (!zheight_clamp){
    estimator_->setHeadPoseZInitialized();  
    cout << "will not set the z-height from ground truth\n";
  }
  
  
  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), stereo_calibration_->getWidth(), 2*stereo_calibration_->getHeight()); // extra space for stereo tasks
  
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
  
  
  return; // full estimator VO disabled for VRC competition, 8 june 2013
  
  estimator_->voUpdate(utime_cur_, delta_camera);
  
  
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
void StereoOdom::imageHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  bot_core::image_t* msg){
  utime_prev_ = utime_cur_;
  utime_cur_ = msg->utime;
  
  if (msg->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG){
    std::cout << "mjepg rxd\n"; 
    imgutils_->decodeStereoImage(msg, left_buf_, right_buf_);
   
  }else{
    std::cout << "camera not MJPEG - not supported/tested\n"; 
    exit(-1);
    //memcpy(left_buf_,  msg->data.data() , msg->size/2);
    //memcpy(right_buf_,  msg->data.data() + msg->size/2 , msg->size/2);
  }
  
  
  vo_->doOdometry(left_buf_,right_buf_, msg->utime);
  updateMotion();
  
  return; // full estimator VO disabled for VRC competition, 8 june 2013
  
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
  
  vo_->doOdometry(left_buf_,right_buf_, msg->utime);
  updateMotion();
  featureAnalysis();
}

void StereoOdom::multisenseLDHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  multisense::images_t* msg){
  int w = msg->images[0].width;
  int h = msg->images[0].height;

  utime_prev_ = utime_cur_;
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
  
  vo_->doOdometry(left_buf_,disparity_buf_.data(), msg->utime );
  updateMotion();
  featureAnalysis();
}


// Given the incoming message, determing the world-to-head frame
bool StereoOdom::getWorldToHead(const drc::robot_state_t* msg, Eigen::Isometry3d &local_to_head){
  bool head_found =false;

  // 1. Take the simulator-created robot root position in world frame:
  Eigen::Isometry3d local_to_body;
  local_to_body.setIdentity();
  local_to_body.translation() << msg->origin_position.translation.x, msg->origin_position.translation.y,
                                 msg->origin_position.translation.z;
  local_to_body.rotate(  Eigen::Quaterniond(msg->origin_position.rotation.w, msg->origin_position.rotation.x, 
                                            msg->origin_position.rotation.y, msg->origin_position.rotation.z) );
  
  map<string, double> jointpos_in;
  map<string, drc::transform_t > cartpos_out;
  for (uint i=0; i< (uint) msg->num_joints; i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(msg->joint_name[i], msg->joint_position[i]));

  // Calculate forward position kinematics
  bool kinematics_status;
  bool flatten_tree=true; // determines absolute transforms to robot origin, otherwise relative transforms between joints.
  kinematics_status = fksolver_->JntToCart(jointpos_in,cartpos_out,flatten_tree);
  if(kinematics_status>=0){
    // cout << "Success!" <<endl;
  }else{
    cerr << "Error: could not calculate forward kinematics!" <<endl;
    return false;
  }
  
  // 2. Determine the head position:
  for( map<string, drc::transform_t>::iterator ii=cartpos_out.begin(); ii!=cartpos_out.end(); ++ii){
    std::string joint = (*ii).first;
    if (   (*ii).first.compare( "head" ) == 0 ){
      Eigen::Isometry3d body_to_head;
      body_to_head.setIdentity();
      body_to_head.translation()  << (*ii).second.translation.x, (*ii).second.translation.y, (*ii).second.translation.z;
      Eigen::Quaterniond quat = Eigen::Quaterniond((*ii).second.rotation.w, (*ii).second.rotation.x, (*ii).second.rotation.y, (*ii).second.rotation.z);
      body_to_head.rotate(quat);    
      head_found =true;
      // Determine the head position in world frame:
      local_to_head = local_to_body * body_to_head; 
      break;
    }
  }     
  return head_found ;
}


void StereoOdom::trueRobotStateHandler(const lcm::ReceiveBuffer* rbuf,
                                       const std::string& channel, const  drc::robot_state_t* msg){
  if (fusion_mode_==3){
    // If successful, set the robot's head position and start state estimation:
    Eigen::Isometry3d local_to_head_true;
    if ( getWorldToHead(msg, local_to_head_true)   ){
      if (!pose_initialized_){
        std::stringstream ss2;
        print_Isometry3d(local_to_head_true, ss2);
        double ypr[3];
        quat_to_euler(  Eigen::Quaterniond(local_to_head_true.rotation()) , ypr[0], ypr[1], ypr[2]);
        
        if (1==1){//verbose
          std::cout << "gt local_to_head_true: " << ss2.str() << " | "<< 
            ypr[0]*180/M_PI << " " << ypr[1]*180/M_PI << " " << ypr[2]*180/M_PI << "\n";         
        }
      
        estimator_->setHeadPose(local_to_head_true);
        pose_initialized_ = true;
        cout << "got first GT Pose Head measurement from true\n";      
      }
      
      // clamp the estimated height !!!! HACK CHEAT HACK CHEAT !!!!
      if (zheight_clamp){
        estimator_->setHeadPoseZ( local_to_head_true.translation().z() );
      }
    }
  }
  
  last_robot_state_msg_ = *msg;
}


// Transform the Gazebo head IMU orientation into the head frame:
// TODO: add an imu frame into the config
Eigen::Quaterniond gazeboIMUToRobotOrientation(const drc::imu_t *msg){
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

void StereoOdom::fuseInterial(Eigen::Quaterniond imu_robotorientation,
                              int correction_frequency, int64_t utime){
  if (fusion_mode_==0){
    //    cout << "got IMU measurement - not incorporating them\n";
    return;
  }
  
  if (!pose_initialized_){
    if((fusion_mode_ ==1) ||(fusion_mode_ ==2) ){
      Eigen::Isometry3d init_pose;
      init_pose.setIdentity();
      init_pose.translation() << 0,0,0;
      init_pose.rotate( imu_robotorientation );
      estimator_->setHeadPose(init_pose);
      pose_initialized_ = true;
      cout << "got first IMU measurement\n";
      return;
    }
  }
  
  if((fusion_mode_ ==2) ||(fusion_mode_ ==3) ){
    if (1==1){//imu_counter_== correction_frequency){
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
        std::cout << "local_to_head: " << ss2.str() << " | "<< 
          ypr[0]*180/M_PI << " " << ypr[1]*180/M_PI << " " << ypr[2]*180/M_PI << "\n";        
      }
        
      double ypr_imu[3];
      quat_to_euler( imu_robotorientation , 
                      ypr_imu[0], ypr_imu[1], ypr_imu[2]);
      if (verbose){
        std::cout <<  ypr_imu[0]*180/M_PI << " " << ypr_imu[1]*180/M_PI << " " << ypr_imu[2]*180/M_PI << " imuypr\n";        
        cout << "IMU correction | pitch roll | was: "
            << ypr[1]*180/M_PI << " " << ypr[2]*180/M_PI << " | now: "
            << ypr_imu[1]*180/M_PI << " " << ypr_imu[2]*180/M_PI << "\n";
      }
      
      
      // NBNBNBNBNB was: // pitch and roll only:
      //Eigen::Quaterniond revised_local_to_head_quat = euler_to_quat( ypr[0], ypr_imu[1], ypr_imu[2]);             
      // ypr:
      Eigen::Quaterniond revised_local_to_head_quat = imu_robotorientation;
      ///////////////////////////////////////
      Eigen::Isometry3d revised_local_to_head;
      revised_local_to_head.setIdentity();
      revised_local_to_head.translation() = local_to_head.translation();
      revised_local_to_head.rotate(revised_local_to_head_quat);
      
      if (verbose){
        std::stringstream ss4;
        print_Isometry3d(revised_local_to_head, ss4);
        quat_to_euler(  Eigen::Quaterniond(revised_local_to_head.rotation()) , ypr[0], ypr[1], ypr[2]);
        std::cout << "local_revhead: " << ss4.str() << " | "<< 
          ypr[0]*180/M_PI << " " << ypr[1]*180/M_PI << " " << ypr[2]*180/M_PI << "\n";        
      }
      estimator_->setHeadPose(revised_local_to_head);
      //estimator_->publishUpdate(utime); // now always done in publishUpdateRobotState
    }
    if (imu_counter_ > correction_frequency) { imu_counter_ =0; }
    imu_counter_++;
  }
  
  estimator_->publishUpdateRobotState(&last_robot_state_msg_);
  
}

void StereoOdom::microstrainHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  microstrain::ins_t* msg){
  Eigen::Quaterniond imu_robotorientation;
  imu_robotorientation =microstrainIMUToRobotOrientation(msg);
  int correction_frequency=100;
  fuseInterial(imu_robotorientation, correction_frequency, msg->utime);
}

void StereoOdom::gazeboHeadIMUHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  drc::imu_t* msg){
  Eigen::Quaterniond imu_robotorientation;
  imu_robotorientation =gazeboIMUToRobotOrientation(msg);
  //int correction_frequency=1000;
  int correction_frequency=1; // once every 1/1000 of second
  fuseInterial(imu_robotorientation, correction_frequency, msg->utime);  
}



void StereoOdom::gazeboBodyIMUHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  drc::imu_t* msg){
  
  // NB: I convert to YPR as thats what the estimator assumes
  estimator_->setBodyRotRateImu( Eigen::Vector3d( msg->angular_velocity[2],  msg->angular_velocity[1],  msg->angular_velocity[0] )  );
  
  /// Repeat conversion and ublish as pose for plotting externally:
  Eigen::Quaterniond imu_orientation;
  imu_orientation =gazeboIMUToRobotOrientation(msg);
  Eigen::Vector3d rpy_rate_world;
  Eigen::Isometry3d imu_nullpose= Eigen::Isometry3d( imu_orientation );
  // Correct transformation from Dehann:
  rpy_rate_world = imu_nullpose.linear() * ( Eigen::Vector3d( msg->angular_velocity[0],  msg->angular_velocity[1],  msg->angular_velocity[2] ));
  //std::cout << rpy_rate_world.transpose() <<" rpy_rate_world\n";
  
  // Previous stuff from me - doesnt work
  /*
    Eigen::Vector3d ypr;
    quat_to_euler( imu_orientation , ypr(0), ypr(1), ypr(2) );

    Eigen::Vector3d ypr_rate( msg->angular_velocity[2],  msg->angular_velocity[1],  msg->angular_velocity[0] );
    Eigen::Quaterniond ypr_rate_quat= euler_to_quat( ypr_rate(0), ypr_rate(1), ypr_rate(2) );
    
    Eigen::Quaterniond applied_rotation = imu_orientation * ypr_rate_quat;
    Eigen::Vector3d applied_ypr;
    quat_to_euler( applied_rotation , applied_ypr(0), applied_ypr(1), applied_ypr(2));

    Eigen::Vector3d ypr_rate_world = applied_ypr - ypr ;

    std::cout << ypr(0) << " " << ypr(1) << " " << ypr(2) <<" orient\n";
    std::cout << ypr_rate(0) << " " << ypr_rate(1) << " " << ypr_rate(2) <<" velocity\n";
    std::cout << applied_ypr(0) << " " << applied_ypr(1) << " " << applied_ypr(2) <<" applied\n";
    std::cout << ypr_rate_world(0) << " " << ypr_rate_world(1) << " " << ypr_rate_world(2) <<" world velocity\n\n";
*/
    
    // publish as pose for plotting externally:
    bot_core::pose_t imu_pose;
    imu_pose.utime = msg->utime;
    imu_pose.pos[0] = 0;
    imu_pose.pos[1] = 0;
    imu_pose.pos[2] = 0;
    imu_pose.orientation[0] = imu_orientation.w();
    imu_pose.orientation[1] = imu_orientation.x();
    imu_pose.orientation[2] = imu_orientation.y();
    imu_pose.orientation[3] = imu_orientation.z();
    imu_pose.vel[0]=0;
    imu_pose.vel[1]=0;
    imu_pose.vel[2]=0;
    imu_pose.rotation_rate[0]=rpy_rate_world(0);
    imu_pose.rotation_rate[1]=rpy_rate_world(1);
    imu_pose.rotation_rate[2]=rpy_rate_world(2);
/*    imu_pose.rotation_rate[0]=ypr_rate_world(2);
    imu_pose.rotation_rate[1]=ypr_rate_world(1);
    imu_pose.rotation_rate[2]=ypr_rate_world(0); */
    imu_pose.accel[0]=0;
    imu_pose.accel[1]=0;
    imu_pose.accel[2]=0;  
    lcm_->publish("POSE_IMU_WORLD", &imu_pose);                
  
}



int main(int argc, char **argv){
  ConciseArgs parser(argc, argv, "fovision-odometry");
  int fusion_mode=0;
  string camera_config = "CAMERA";
  string output_extension = "";
  parser.add(fusion_mode, "i", "fusion_mode", "0 none, 1 at init, 2 every second, 3 init from gt, then every second");
  parser.add(camera_config, "c", "camera_config", "Camera Config block to use: CAMERA, stereo, stereo_with_letterbox");
  parser.add(output_extension, "o", "output_extension", "Extension to pose channels (e.g. '_VO' ");
  parser.add(zheight_clamp, "z", "zheight_clamp", "Clamp to GT height");
  parser.parse();
  cout << fusion_mode << " is fusion_mode\n"; 
  cout << camera_config << " is camera_config\n"; 
  cout << zheight_clamp << " is zheight_clamp\n"; 
  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  StereoOdom fo= StereoOdom(lcm, fusion_mode,camera_config, output_extension);    
  while(0 == lcm->handle());
}
