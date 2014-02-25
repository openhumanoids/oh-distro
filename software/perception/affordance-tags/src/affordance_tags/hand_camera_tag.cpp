// DRC-tags
// - detects tags (using april tags)
// - pairs detections with affordances in a library
// - updates the affordances in the affordances in the store
//
// TODO: read library from file
// TODO: add rate limiting (process is at about 15Hz)

// pre-sept: Tag36h11
// post sept: Tag16h5
//

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>

#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <opencv2/opencv.hpp>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <pointcloud_tools/pointcloud_lcm.hpp> // unpack lidar to xyz
#include <lcmtypes/bot_core.hpp>

#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>

#include <ConciseArgs>

#include <affordance/AffordancePlusState.h>

using namespace std;
using namespace Eigen;
using namespace affordance;
using namespace boost::assign; // bring 'operator+()' into scope

class Tags{
  public:
    Tags(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_);
    
    ~Tags(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool verbose_;
    std::string camera_frame_, camera_channel_;
    int width_, height_;
    double fx_, fy_, cx_, cy_;
    
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    void processTag(int64_t utime_in);

    BotParam* botparam_;
    bot::frames* botframes_cpp_;
    
    pointcloud_vis* pc_vis_;
    pointcloud_lcm* pc_lcm_;
    image_io_utils*  imgutils_;     
    uint8_t* img_buf_;
    
    AprilTags::TagDetector* tag_detector_;
};

Tags::Tags(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_):
    lcm_(lcm_), verbose_(verbose_){
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_cpp_ = new bot::frames( lcm_ , botparam_ );
 
  camera_channel_ = "CAMERARHAND";
  camera_frame_ = "CAMERARHAND";
  
  // subscribe but only keep a queue of one
  lcm::Subscription* sub = lcm_->subscribe( camera_channel_ ,&Tags::imageHandler,this);
  sub->setQueueCapacity(1);  
  
  std::string left_str = "cameras."+camera_frame_+".intrinsic_cal";
  width_ = bot_param_get_int_or_fail(botparam_, (left_str+".width").c_str());
  height_ = bot_param_get_int_or_fail(botparam_,(left_str+".height").c_str());
  double vals[10];
  bot_param_get_double_array_or_fail(botparam_, (left_str+".pinhole").c_str(), vals, 5);
  fx_ = vals[0];
  fy_ = vals[1];
  cx_ = vals[3];
  cy_ = vals[4];
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60000,"Tag Detections",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60001,"Camera Pose",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60002,"Hand Face Pose",5,1) );
   
  tag_detector_ = new AprilTags::TagDetector (AprilTags::tagCodes16h5);
  img_buf_= (uint8_t*) malloc(3* width_  *  height_);
  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), width_, height_); // extra space for stereo tasks
}
    
// draw April tag detection on actual image
// NB: because the conversion was skipped R and B will be switched...
void draw_detection(cv::Mat& image, const AprilTags::TagDetection& detection) {
  // use corner points detected by line intersection
  std::pair<float, float> p1 = detection.p[0];
  std::pair<float, float> p2 = detection.p[1];
  std::pair<float, float> p3 = detection.p[2];
  std::pair<float, float> p4 = detection.p[3];

  // plot outline
  cv::line(image, cv::Point2f(p1.first, p1.second), cv::Point2f(p2.first, p2.second), cv::Scalar(255,0,0,0) );
  cv::line(image, cv::Point2f(p2.first, p2.second), cv::Point2f(p3.first, p3.second), cv::Scalar(0,255,0,0) );
  cv::line(image, cv::Point2f(p3.first, p3.second), cv::Point2f(p4.first, p4.second), cv::Scalar(0,0,255,0) );
  cv::line(image, cv::Point2f(p4.first, p4.second), cv::Point2f(p1.first, p1.second), cv::Scalar(255,0,255,0) );

  // mark center
  cv::circle(image, cv::Point2f(detection.cxy.first, detection.cxy.second), 8, cv::Scalar(0,0,255,0), 2);

  // print ID
  std::ostringstream strSt;
  strSt << "#" << detection.id;
  cv::putText(image, strSt.str(), cv::Point2f(detection.cxy.first + 10, detection.cxy.second + 10),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
}

std::string printPose(Eigen::Isometry3d pose){
  Eigen::Vector3d t(pose.translation());
  Eigen::Quaterniond r(pose.rotation());
  double rpy[3];
  quat_to_euler(r, rpy[0], rpy[1], rpy[2]);
  
  std::stringstream ss;
  ss <<t[0]<<", "<<t[1]<<", "<<t[2]<<", " 
       <<r.w()<<", "<<r.x()<<", "<<r.y()<<", "<<r.z() << ", "
       << rpy[0] <<", "<< rpy[1] <<", "<< rpy[2];
  return ss.str();
}


void Tags::imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg){
  imgutils_->decodeImageToRGB((msg),  img_buf_ );
  processTag(msg->utime);
}

void Tags::processTag(int64_t utime_in){
  
  cv::Mat img = cv::Mat::zeros(height_, width_,CV_8UC3);
  img.data = img_buf_;
  //cv::Mat img = cv::imdecode(cv::Mat(msg->images[0].data), -1);
  
  // I'm assuming incoming data has LCM RGB convention
  // to avoid color re-conversions, skip RGB2BGR conversion here:
  cv::Mat gray;
  cv::cvtColor(img, gray, CV_RGB2GRAY); 
  std::vector<AprilTags::TagDetection> detections = tag_detector_->extractTags(gray); // find the tags!
  
  if (verbose_) cout << detections.size() << " tags detected:" << endl;
  std::vector < Isometry3dTime > detected_posesT;
  
  for (int i=0; i< detections.size(); i++) {
    cout << "  Id: " << detections[i].id << " -- " << "  Hamming distance: " << detections[i].hammingDistance << endl;
    // also highlight in the image
    if (verbose_) draw_detection(img, detections[i]);
   
    // Get the relative transform, match with affordance and publish updated affordance:
    Eigen::Matrix4d T = detections[i].getRelativeTransform( 0.053, fx_, fy_, cx_, cy_);  
    Eigen::Isometry3d local_to_camera;
    botframes_cpp_->get_trans_with_utime( camera_frame_ , "local", utime_in, local_to_camera);    
    Eigen::Isometry3d tag_pose = local_to_camera*Eigen::Isometry3d(T)   ;
    tag_pose.rotate( Eigen::Quaterniond(  euler_to_quat( 0 ,  (M_PI) ,  0 )  ) );

    
    if (verbose_){
      Isometry3dTime cameraT = Isometry3dTime(utime_in, local_to_camera);
      pc_vis_->pose_to_lcm_from_list(60001, cameraT);
      
      Isometry3dTime poseT = Isometry3dTime(utime_in, tag_pose);
      detected_posesT.push_back( poseT);
      std::cout << "detected: " << detections[i].id << "\n";
      
      Eigen::Isometry3d local_to_rhand_face;
      botframes_cpp_->get_trans_with_utime( "RHAND_FACE" , "local", utime_in, local_to_rhand_face);    
      Isometry3dTime hand_faceT = Isometry3dTime(utime_in, local_to_rhand_face);
      pc_vis_->pose_to_lcm_from_list(60002, hand_faceT);
      
      std::cout << printPose(local_to_camera) << " camera in world frame\n";
      std::cout << printPose(tag_pose) << " tag in world frame\n";
      std::cout << printPose(local_to_rhand_face) << " hand face in world frame\n";
    }    
    
  }
  
  if (verbose_){
    pc_vis_->pose_collection_to_lcm_from_list(60000, detected_posesT);
    imgutils_->sendImage( img.data, utime_in, width_, height_, 3, "CAMERA_TAGS");
  }
}


int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "drc-tags");
  bool verbose=FALSE;
  parser.add(verbose, "v", "verbose", "Verbosity");
  parser.parse();
  cout << verbose << " is verbose\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Tags app(lcm,verbose);
  cout << "Ready to find tags" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
