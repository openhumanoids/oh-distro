// Listen to features, images and registeration triggers
// - when features received, find corresponding image from buffer of images and pair
// - when trigger is received, these two become the reference
// - for subsequent feature message, match the features and image with the reference and publish.

/*
The head is moving around - swaying over and back
- a trigger requests a sweep of scans @ utimeA - the last 100 scans, you bundle them off to the user.
- the user fits affordances to determine the transform from head at utimeA and afforance (utimeA->affordance)

Thereafter I register the head back to the image at utimeA (i.e. determine a current-to-previous transform). 
- This is by aligning directly back to the frame at utimeA - not VO
- when the affordances are fitted and returned they are projected onto this pose (at utimeB)
- the transform to an affordance is: utimeB->utimeA->affordance

If at any time an alignment to utimeA is not possible, we do VO until it is.
- in that time, utimeC, the transform to an affordance then:
- utimeC -> utimeB->utimeA->affordance
- where utimeC->utimeB is openloop VO
*/

#include <iostream>
#include <Eigen/Dense>
#include <deque>

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>

#include <dirent.h>


#include "registeration.hpp"

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

#include <lcm/lcm-cpp.hpp>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include <lcmtypes/drc_lcmtypes.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/motion_estimate.hpp>


#define DO_TIMING_PROFILE TRUE
#include <ConciseArgs>


using namespace std;
using namespace Eigen;
using namespace cv;
using namespace boost;
using namespace boost::assign; // bring 'operator+()' into scope

class RegApp{
  public:
    RegApp(boost::shared_ptr<lcm::LCM> &publish_lcm, std::string camera_);
    
    ~RegApp(){
    }
    
    void doReg();
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void featuresCurrentHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  reg::features_t* msg);
    void featuresReferenceHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  reg::features_t* msg);
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    void registerCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::map_params_t* msg);
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg);

    std::vector<ImageFeature> getFeaturesFromLCM(const  reg::features_t* msg, Eigen::Isometry3d &pose);
    std::string camera_;

    pointcloud_vis* pc_vis_;
    
    Reg::Ptr reg;
    std::vector<ImageFeature> cur_features_;
    bot_core::image_t cur_image_;
    Eigen::Isometry3d cur_pose_;
    
    std::vector<ImageFeature> ref_features_;
    bot_core::image_t ref_image_;
    Eigen::Isometry3d ref_pose_;
    
    int registeration_mode_ ; 
    // mode: 0 listening for trigger, 1 listening for refer feats, 2 listening for current features 
    
    BotParam* botparam_;
    BotFrames* botframes_;

    deque< bot_core::image_t  > * image_queue_;
    deque< bot_core::planar_lidar_t  > * laser_queue_;
    pcl::PointCloud<PointXYZRGB>::Ptr accum_cloud;
};



RegApp::RegApp(boost::shared_ptr<lcm::LCM> &lcm_, std::string camera_):          
    lcm_(lcm_),camera_(camera_), registeration_mode_(0){

  botparam_ = bot_param_new_from_server( lcm_->getUnderlyingLCM() , 0);
  botframes_= bot_frames_get_global( lcm_->getUnderlyingLCM() , botparam_);


  lcm_->subscribe("FEATURES_REF",&RegApp::featuresReferenceHandler,this);  
  lcm_->subscribe("FEATURES_CUR",&RegApp::featuresCurrentHandler,this);  
  lcm_->subscribe(camera_ ,&RegApp::imageHandler,this);  
  lcm_->subscribe("ROTATING_SCAN",&RegApp::lidarHandler,this);  

  
  // In progress:
  lcm_->subscribe("MAP_CREATE",&RegApp::registerCommandHandler,this);  

  reg = Reg::Ptr (new Reg (lcm_));
  
  image_queue_ = new deque< bot_core::image_t > ();
  laser_queue_ = new deque< bot_core::planar_lidar_t > ();

  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(70000,"Reg Pose",4,1) );

  // Cloud attached to Reg Pose
  float colors_0f[] ={1.0,0.0,0.0};
  vector <float> colors_0;
  colors_0.assign(colors_0f,colors_0f+4*sizeof(float));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(70001,"Null Pose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(70002,"Reg Pt Cloud"     ,1,1, 70001,1,colors_0));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(70003,"Reg Pt Cloud - Sensor"     ,1,1, 70000,1,colors_0));


  float colors_1f[] ={0.6,0.0,0.6};
  vector <float> colors_1;
  colors_1.assign(colors_1f,colors_1f+4*sizeof(float));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(70100,"Revised Pose",4,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(70101,"Reg Pt Cloud - Sensor Revised"     ,1,1, 70100,0,colors_1));


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accum_cloud_clf (new pcl::PointCloud<pcl::PointXYZRGB> ());
  accum_cloud = accum_cloud_clf;
}


// Classless method to convert queue to cloud on demand:
void deque_to_cloud(deque<bot_core::planar_lidar_t > * laser_queue, 
                               BotFrames* botframes, 
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &accum_cloud){
  cout <<"deque of size " << laser_queue->size() <<"to cloud\n";
  accum_cloud->points.clear(); 
 
  for (int i=0; i <laser_queue->size() ; i++){
    bot_core::planar_lidar_t curr_msg =  laser_queue->at(i);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    double maxRange = 2.0;//just the area right in front of the user    29.7;
    double validBeamAngles[] ={-2.5,2.5}; // all of it both sides
    convertLidar(curr_msg.ranges, curr_msg.nranges, curr_msg.rad0,
      curr_msg.radstep, lidar_cloud, maxRange,
      validBeamAngles[0], validBeamAngles[1]);

    for (size_t j=0; j< lidar_cloud->points.size() ; j++){
       pcl::PointXYZRGB pt = lidar_cloud->points[j]; 
       float range = sqrt( pow(pt.x,2) + pow(pt.y,2) + pow(pt.z,2) );
       if (range< 2.0){
         float rgb[3];
         jet_rgb( (range - 0.5)/1.5, rgb); // map 0:2 to range -1/3:1   [i.e. 0.5->2 becomes 0->1]
         pt.r = rgb[0]*255.0; pt.g = rgb[1]*255.0; pt.b = rgb[2]*255.0;
       }else{
         pt.r = 255.0; pt.g = 0.0; pt.b = 0.0;
       }
       lidar_cloud->points[j] = pt;
    }
     
    BotTrans transform;
    bot_frames_get_trans_with_utime(botframes, "ROTATING_SCAN", "local",
                                          curr_msg.utime, &transform);  

    // Fixed rigid transform
    Eigen::Isometry3d local_to_lidar;
    
    Eigen::Quaterniond quat = Eigen::Quaterniond(transform.rot_quat[0], transform.rot_quat[1],transform.rot_quat[2],transform.rot_quat[3]);
    local_to_lidar.setIdentity();
    local_to_lidar.translation()  << transform.trans_vec[0], transform.trans_vec[1], transform.trans_vec[2];
    local_to_lidar.rotate(quat);

    Eigen::Isometry3f pose_f = Isometry_d2f(local_to_lidar);
    Eigen::Quaternionf pose_quat(pose_f.rotation());
    pcl::transformPointCloud (*lidar_cloud, *lidar_cloud,
        pose_f.translation(), pose_quat); // !! modifies lidar_cloud
    (*accum_cloud) += (*lidar_cloud);      
  }
}


void RegApp::doReg(){
  // grey assumed:
  Mat ref_img = Mat::zeros( ref_image_.height , ref_image_.width ,CV_8UC1); // h,w
  ref_img.data = ref_image_.data.data();
  Mat cur_img = Mat::zeros( cur_image_.height , cur_image_.width ,CV_8UC1); // h,w
  cur_img.data = cur_image_.data.data();
    
  //imwrite("ref.png",ref_img);
  //imwrite("cur.png",cur_img);
      
  FrameMatchPtr match(new FrameMatch());
  reg->align_images(ref_img, cur_img, ref_features_, cur_features_, ref_image_.utime, cur_image_.utime , match);

  if (match->status == pose_estimator::SUCCESS){
    Eigen::Quaterniond delta_quat = Eigen::Quaterniond(match->delta.rotation());
    cout << "\nsuccess: matched ref at "        
       <<ref_image_.utime <<" (" << ref_features_.size() << "f) to cur at "
       <<cur_image_.utime <<" " << cur_features_.size() << "f) | inliers: "<< match->n_inliers <<"\n";

  Eigen::Isometry3d rev_pose_revised = cur_pose_* match->delta.inverse();

  std::stringstream ss2;
  ss2 << "Ref: "; print_Isometry3d(ref_pose_, ss2); ss2 << "\n";
  ss2 << "Cur: "; print_Isometry3d(cur_pose_, ss2); ss2 << "\n";
  ss2 << "DLTA "; print_Isometry3d(match->delta, ss2); ss2 << "\n";
  ss2 << "New: "; print_Isometry3d(rev_pose_revised, ss2); ss2 << "\n";
  std::cout << ss2.str() << "\n";

  Isometry3dTime revised_poseT = Isometry3dTime(ref_image_.utime, rev_pose_revised );
  pc_vis_->pose_to_lcm_from_list(70100, revised_poseT);
  pc_vis_->ptcld_to_lcm_from_list(70101, *accum_cloud, ref_image_.utime, ref_image_.utime);



  }else{
    cout << "f";
    //cout << "failure: no match old frame at "
    //   <<ref_image_.utime <<" (" << ref_features_.size() << "f) to "
    //   <<cur_image_.utime <<" " << cur_features_.size() << "f)\n";
  } 
}




std::vector<ImageFeature> RegApp::getFeaturesFromLCM(const  reg::features_t* msg, Eigen::Isometry3d &pose){
  std::vector<ImageFeature> features;
    
  for (int i =0; i < msg->nfeatures; i++){ 
    ImageFeature f;
    
    f.track_id = msg->features[i].track_id;
    f.uv[0] = msg->features[i].uv[0];
    f.uv[1] = msg->features[i].uv[1];
    f.base_uv[0] = msg->features[i].base_uv[0];
    f.base_uv[1] = msg->features[i].base_uv[1];
    f.uvd[0] = msg->features[i].uvd[0];
    f.uvd[1] = msg->features[i].uvd[1];
    f.uvd[2] = msg->features[i].uvd[2];
    f.xyz[0] = msg->features[i].xyz[0];
    f.xyz[1] = msg->features[i].xyz[1];
    f.xyz[2] = msg->features[i].xyz[2];
    f.xyzw[0] = msg->features[i].xyzw[0];
    f.xyzw[1] = msg->features[i].xyzw[1];
    f.xyzw[2] = msg->features[i].xyzw[2];
    f.xyzw[3] = msg->features[i].xyzw[3];
    // color left out for now
    
    pose.setIdentity();
    pose.translation() << msg->pos[0], msg->pos[1], msg->pos[2];
    Eigen::Quaterniond m(msg->quat[0],msg->quat[1],msg->quat[2],msg->quat[3]);
    pose.rotate(m);
    /*
    cout << line << " is line\n";
    cout << "i: " << i <<"\n";
    cout << "f.track_id: " << f.track_id <<"\n";
    cout << "f.uv: " << f.uv[0] << " "<< f.uv[1] <<"\n";
    cout << "f.base_uv: " << f.base_uv[0] << " "<< f.base_uv[1] <<"\n";
    cout << "f.uvd: " << f.uvd[0] << " "<< f.uvd[1]<< " "<< f.uvd[2]<<"\n";
    cout << "f.xyz: " << f.xyz[0] << " "<< f.xyz[1]<< " "<< f.xyz[2]<<"\n";
    cout << "f.xyzw: " << f.xyzw[0] << " "<< f.xyzw[1]<< " "<< f.xyzw[2]<< " "<< f.xyzw[3]<<"\n";
    cout << "f.color: " << (int)f.color[0] << " "<< (int)f.color[1] << " "<< (int)f.color[2] <<"\n";
      */

    features.push_back(f);
  }
  
  //cout << "in: " << msg->nfeatures << " | extracted: "<< features.size() << "\n"; 
  return features;  
}

void RegApp::featuresCurrentHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  reg::features_t* msg){
  if (registeration_mode_!= 2){
    return;
  }
  //cout << "\n\ngot current features @ "<< msg->utime<<"\n";
  
  // now find the corresponding image and pair it
  bool feat_image_matched_ = false;
  for(int i=0; i < image_queue_->size(); i++){
    //cout << image_queue_->at(i).utime << " " << i << "\n";
    if (msg->utime == image_queue_->at(i).utime){
      cur_features_ = getFeaturesFromLCM(msg,cur_pose_);
      cur_image_ = image_queue_->at(i);
      //cout << "image and features matched: " << i << " " << image_queue_->at(i).utime << " ("<< cur_features_.size()<<"f)\n"; 
      feat_image_matched_ = true;

      break;
    }
  }
  
  // If the registartation have been activated, do it live:
  if (feat_image_matched_ ){
    doReg();
  }else{
   cout << "didn't find a matching current image - make image deque bigger\n";  // not fatal... just continue
  }
}


void RegApp::featuresReferenceHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  reg::features_t* msg){
  
  if (registeration_mode_ != 1){ // ignore features if we havent just been asked to track to them
    return;
  }

  cout << "\n\ngot reference features @ "<< msg->utime<<"\n";

  // now find the corresponding image and pair it
  bool feat_image_matched_ = false;
  for(int i=0; i < image_queue_->size(); i++){
    //cout << image_queue_->at(i).utime << " " << i << "\n";
    if (msg->utime == image_queue_->at(i).utime){
      cur_features_ = getFeaturesFromLCM(msg,cur_pose_);
      cur_image_ = image_queue_->at(i);
      cout << "image and features matched: " << i << " " << image_queue_->at(i).utime << " ("<< cur_features_.size()<<"f)\n"; 
      feat_image_matched_ = true;

      break;
    }
  }
  
  if (!feat_image_matched_){
     cout <<"didn't find a matching reference image - make image deque bigger\n";
     exit(-1);
  } 

  ref_features_ = cur_features_;
  ref_image_ = cur_image_;
  ref_pose_ = cur_pose_;

  // set the reference image
  cout << "Will now register to " << ref_features_.size() << " features\n";
  
  // Send the pose we will try to register back to:
  Isometry3dTime ref_poseT = Isometry3dTime(ref_image_.utime, ref_pose_);
  pc_vis_->pose_to_lcm_from_list(70000, ref_poseT);

  deque_to_cloud(laser_queue_, botframes_, accum_cloud);

  Isometry3dTime null_poseT = Isometry3dTime(ref_image_.utime, Eigen::Isometry3d::Identity()  );
  pc_vis_->pose_to_lcm_from_list(70001, null_poseT);
  pc_vis_->ptcld_to_lcm_from_list(70002, *accum_cloud, ref_image_.utime, ref_image_.utime);

  std::stringstream ss2;
  print_Isometry3d(ref_pose_, ss2);
  std::cout << "Received refpose: " << ss2.str() << "\n";

  Eigen::Isometry3f pose_f = Isometry_d2f(ref_pose_.inverse() );
  Eigen::Quaternionf pose_quat(pose_f.rotation());
  pcl::transformPointCloud (*accum_cloud, *accum_cloud,
      pose_f.translation(), pose_quat); // !! modifies accum_cloud
  pc_vis_->ptcld_to_lcm_from_list(70003, *accum_cloud, ref_image_.utime, ref_image_.utime);

  registeration_mode_ = 2;
}



void RegApp::imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg){
  //cout << "got image @ "<< msg->utime <<"\n";
  
  // Keep a buffer/deque of left images - should only require a few images
  bot_core::image_t msg_cpy = *msg;
  image_queue_->push_back(  msg_cpy );
  
  //cout << "image_queue_ size: " << image_queue_->size() << "\n";
  if( image_queue_->size() > 10){
    image_queue_->pop_front();
  }
}


void RegApp::registerCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::map_params_t* msg){
  // TODO: add check that we've seen last_features
  cout << "got command\n";
  
  registeration_mode_ = 1;
}


void RegApp::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
  bot_core::planar_lidar_t msg_cpy = *msg;
  laser_queue_->push_back(  msg_cpy );

  while (laser_queue_->size() > 100){
    laser_queue_->pop_front();
  }
  
  //if (newmap_requested){
  //  deque_to_cloud();
  //  newmap_requested=0;
  //}
}


int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "registeration-app");
  string camera="CAMLCM_IMAGE_GRAY_LEFT";
  parser.add(camera, "c", "camera", "Camera channel");
  parser.parse();
  cout << camera << " is camera\n"; 
  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  RegApp app(lcm, camera);
  cout << "registeration is ready" << endl << endl;
  while(0 == lcm->handle());
  return 0;
}
