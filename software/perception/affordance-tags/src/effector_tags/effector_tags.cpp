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
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>

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
using namespace boost;

// Details about a pairing between and tag and an affordance:
class TagDetails { 
public:
  TagDetails(int id, Eigen::Isometry3d pose ):
    id_ (id), active_(false), pose_(pose){};
  int id_; // 
  bool active_; // has this object been seen yet
  Eigen::Isometry3d pose_; 
};

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
    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg);   
    void utimeHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::utime_t* msg);   

    void getTagList();
    bool processTag();
    void publishPalmGoal(Eigen::Isometry3d local_to_hand);

    BotParam* botparam_;
    bot::frames* botframes_cpp_;
   
    pointcloud_vis* pc_vis_;
    pointcloud_lcm* pc_lcm_;
    image_io_utils*  imgutils_;     
    uint8_t* img_buf_;
    
    AprilTags::TagDetector* tag_detector_;
    std::map<int,TagDetails> tags_; // map of tag details    

    int current_id_; // main currently active id (-1 if haven't found anything)
    Eigen::Isometry3d local_to_camera_; // the best estimate of the robot pose

    bool use_left_hand_;
    int current_utime_;
};

Tags::Tags(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_):
    lcm_(lcm_), verbose_(verbose_){
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_cpp_ = new bot::frames( lcm_ , botparam_ );

  camera_channel_ = "CAMERA";
  camera_frame_ = "CAMERA_LEFT";
  lcm_->subscribe( "WEBCAM" ,&Tags::imageHandler,this);


  width_ = 960;
  height_ = 720;
  fx_ = 830;//vals[0];
  fy_ = 830;//vals[1];
  cx_ = 480;//vals[3];
  cy_ = 360;//vals[4];
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(30000,"Pose Hand",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(40000,"Pose Camera [final]",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(50000,"Poses Prior",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60000,"Poses Detected",5,1) );
   

  tag_detector_ = new AprilTags::TagDetector (AprilTags::tagCodes16h5);
  img_buf_= (uint8_t*) malloc(3* width_  *  height_);
  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), width_, height_); // extra space for stereo tasks
  
  getTagList();
  current_id_=-1;

  use_left_hand_ =true;
}
    
void Tags::getTagList(){

  // TODO: read this information from a config file:
  /// NB: uids count from 1  
  /// cylinder constructor: length, radius, mass, uid, map id, position  
  
  // aff = 0 is reserved .... dont use it here

  ifstream fileinput ("tag_poses.txt");
  if (fileinput.is_open()){
    string message;
    while ( getline (fileinput,message) ) { // for each line
      cout << message << endl;

      vector<string> tokens;
      boost::split(tokens, message, boost::is_any_of(","));
      vector<double> values;
      BOOST_FOREACH(string t, tokens){
        values.push_back(lexical_cast<double>(t));
      }

      int tag_id =lexical_cast<int>( values[0] );
      Eigen::Isometry3d tag_pose;
      tag_pose.setIdentity();
      tag_pose.translation()  << values[1], values[2], values[3];
      Eigen::Quaterniond quat = Eigen::Quaterniond(values[4], values[5], values[6], values[7] );
      tag_pose.rotate(quat);
      tags_.insert( make_pair( tag_id , TagDetails( tag_id , tag_pose ) ) ); 
    }
  }
  fileinput.close();

  std::vector < Isometry3dTime > prior_posesT;
  std::vector< int64_t > prior_utimes;
  std::vector< string > prior_ids;

  for (int i=0; i< tags_.size(); i++) {

    TagDetails td = tags_.find(i)->second ;
    Isometry3dTime poseT = Isometry3dTime( td.id_ , td.pose_ );
    prior_posesT.push_back( poseT);
    prior_utimes.push_back( td.id_ );

    std::stringstream this_id;
    this_id << td.id_;
    prior_ids.push_back( this_id.str() );
  }

  pc_vis_->pose_collection_to_lcm_from_list(50000, prior_posesT);
  pc_vis_->text_collection_to_lcm(50001, 50000, "Frames [Labels]", prior_ids, prior_utimes );   

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


void Tags::utimeHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::utime_t* msg){
  current_utime_ = msg->utime;
}


int counter =0;
void Tags::imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg){
  // NB: camview doesn't do timestamps properly

  counter++;
  if (counter % 3 == 0){
    //std::cout <<msg->utime << " process\n";

    imgutils_->decodeImageToRGB((msg),  img_buf_ );
    bool successfully_updated_pose = processTag();

    Eigen::Isometry3d utorso_to_hand;
    utorso_to_hand.setIdentity();
    utorso_to_hand.translation().x() = 0.5;
    utorso_to_hand.translation().y() = 0.35;
    utorso_to_hand.translation().z() = 0.0;

    utorso_to_hand.translation().x()  += 2*local_to_camera_.translation().x() ;
    utorso_to_hand.translation().y()  += 2*local_to_camera_.translation().y() ;
    utorso_to_hand.translation().z()  += 2*local_to_camera_.translation().z() ;

    // Rotate the orientation from camera frame to palm frame:
    // red along the fingers, green towards the thumb joint blue out of the palm
    Eigen::Isometry3d camera_orientation;
    camera_orientation.setIdentity();
    camera_orientation.rotate( Eigen::Quaterniond(local_to_camera_.rotation() ) ); 
    Eigen::Isometry3d camera_to_hand_orientation = Eigen::Isometry3d::Identity();
    camera_to_hand_orientation.rotate( euler_to_quat( -90*M_PI/180, -90*M_PI/180, 0*M_PI/180 ) );  
    camera_orientation  = camera_orientation*camera_to_hand_orientation;
    utorso_to_hand.rotate(  Eigen::Quaterniond(camera_orientation.rotation()  ) ) ; 

    Eigen::Isometry3d local_to_utorso, local_to_hand;
    botframes_cpp_->get_trans_with_utime( "utorso" , "local", current_utime_, local_to_utorso);    
    local_to_hand = local_to_utorso*utorso_to_hand;
    Isometry3dTime local_to_handT(0, local_to_hand );
    pc_vis_->pose_to_lcm_from_list(30000, local_to_handT  );


    if (successfully_updated_pose){
      publishPalmGoal(local_to_hand);
    }



  }
}


// Returns true if the camera pose was updated
bool Tags::processTag(){
  int64_t utime_in = current_utime_;// 0;//msg->utime;
  
  cv::Mat img = cv::Mat::zeros(height_, width_,CV_8UC3);
  img.data = img_buf_;
  
  // I'm assuming incoming data has LCM RGB convention
  // to avoid color re-conversions, skip RGB2BGR conversion here:
  cv::Mat gray;
  cv::cvtColor(img, gray, CV_RGB2GRAY); 
  std::vector<AprilTags::TagDetection> detections = tag_detector_->extractTags(gray); // find the tags!
  
  // if (verbose_) cout << detections.size() << " tags detected:" << endl;

  std::vector < Isometry3dTime > detected_posesT;
  std::vector< int64_t > detected_utimes;
  std::vector< string > detected_ids_strings;
  std::vector< int > detected_ids;
  for (int i=0; i< detections.size(); i++) {

    // also highlight in the image
    if (verbose_) draw_detection(img, detections[i]);
    
    if (detections[i].id < 12){
      // Get the relative transform, match with affordance and publish updated affordance:
      Eigen::Matrix4d T = detections[i].getRelativeTransform( 0.072, fx_, fy_, cx_, cy_);  
      Eigen::Isometry3d camera_to_tag = Eigen::Isometry3d(T);
      
      TagDetails this_tag = tags_.find( detections[i].id )->second;
      Eigen::Isometry3d local_to_tag = this_tag.pose_;
      Eigen::Isometry3d this_local_to_camera = local_to_tag *camera_to_tag.inverse(); // ... for this tag
      
      if (verbose_){
        Isometry3dTime poseT = Isometry3dTime(utime_in + detections[i].id,this_local_to_camera);
        detected_posesT.push_back( poseT);
        detected_utimes.push_back( utime_in + detections[i].id );

        detected_ids.push_back( detections[i].id );
        std::stringstream this_id;
        this_id << detections[i].id;
        detected_ids_strings.push_back( this_id.str() );
        // std::cout << "detected: " << detections[i].id << "\n";
      }
    }
  }

  if (verbose_){
    if ( detected_posesT.size() > 0 ){
      pc_vis_->pose_collection_to_lcm_from_list(60000, detected_posesT);
      pc_vis_->text_collection_to_lcm(60001, 60000, "Frames [Labels]", detected_ids_strings, detected_utimes );   
      imgutils_->sendImage( img.data, utime_in, width_, height_, 3, "CAMERA_TAGS"); //"CAMERA_TAGS"
    }
  }


  /// Update the actual camera pose:
  
  // 1. Cant update if not tags were found:
  if (detected_ids.size() == 0){
    std::cout << "No valid tags found\n";
    return false;
  }

  // 2. If we found the previous tag, then stick with it:
  bool found_current_id = false;
  for (int i=0; i< detected_ids.size(); i++) {
    if( detected_ids[i] == current_id_){
      found_current_id = true;
      local_to_camera_ = detected_posesT[i].pose ;
      std::cout << "Tag Update ["<< current_id_ << "]\n";
      break;
    }
  }

  // 3. if we didnt find the previous tag, use the first detection (arbitrary)
  if ( found_current_id == false ){
    local_to_camera_ = detected_posesT[0].pose ;
    current_id_ = detected_ids[0];
    std::cout << "Switching to Tag ["<< current_id_ << "]\n";
  }

  // Draw the camera pose
  Isometry3dTime local_to_cameraT(utime_in, local_to_camera_ );
  pc_vis_->pose_to_lcm_from_list(40000, local_to_cameraT  );
  std::vector< int64_t > local_to_camera_utimes;
  local_to_camera_utimes.push_back(utime_in);
  std::vector< string > local_to_camera_labels;
  std::stringstream local_to_camera_label;
  local_to_camera_label << "Cam: " << local_to_cameraT.pose.translation().z() ;
  local_to_camera_labels.push_back( local_to_camera_label.str());
  pc_vis_->text_collection_to_lcm(40001, 40000, "Current Camer Positions", local_to_camera_labels, local_to_camera_utimes );   

  return true;
}


// copied from ee_driver:
void Tags::publishPalmGoal(Eigen::Isometry3d local_to_hand){


  drc::ee_goal_t msg;
  msg.utime = bot_timestamp_now();

  msg.ee_goal_pos.translation.x = local_to_hand.translation().x();
  msg.ee_goal_pos.translation.y = local_to_hand.translation().y();
  msg.ee_goal_pos.translation.z = local_to_hand.translation().z();
  
  Eigen::Quaterniond q = Eigen::Quaterniond(local_to_hand.rotation());
  msg.ee_goal_pos.rotation.w = q.w();
  msg.ee_goal_pos.rotation.x = q.x();
  msg.ee_goal_pos.rotation.y = q.y();
  msg.ee_goal_pos.rotation.z = q.z();
  msg.num_chain_joints = 0;

  if (!use_left_hand_){
    lcm_->publish("LEFT_PALM_GOAL_CLEAR", &msg);
    lcm_->publish("RIGHT_PALM_GOAL", &msg);
  }else {
    lcm_->publish("LEFT_PALM_GOAL", &msg);
    lcm_->publish("RIGHT_PALM_GOAL_CLEAR", &msg);
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

