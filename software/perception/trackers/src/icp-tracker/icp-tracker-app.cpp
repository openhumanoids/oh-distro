//~/Desktop/drill_clouds$ drc-icp-tracker -o a_moved.pcd -n origi/sweep_cloud_305798000.pcd 
// work in progress - v1


//drc-icp-tracker -a a_moved.pcd -b d.pcd 
/*
capture a sweep
- with BB around previous position, clip the incoming point cloud
- (and possibly with position of a plane, remove the plane)
- run icp
*/

#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <ConciseArgs>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

#include <trackers/major-plane-detect.hpp>
#include <trackers/icp-tracker.hpp>
#include <affordance/AffordanceUtils.hpp>

using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string lidar_channel_, int affordance_id);
    
    ~Pass(){
    }    
    
    void init_tracking(bool use_plane_tracker, int plane_affordance_id);    
    
  private:
    
    boost::shared_ptr<lcm::LCM> lcm_;
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg);   
    std::string lidar_channel_;

    void affordancePlusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::affordance_plus_collection_t* msg);
    
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;

    bool got_affs_;    
    
    int counter_;
    pointcloud_vis* pc_vis_;
    
    Eigen::Isometry3d head_to_local_;
    
    // Tracker Configuration:
    int affordance_id_;
    MajorPlane* major_plane_;
    int plane_affordance_id_;
    bool use_plane_tracker_;    
    
    // Plane Tracking Variables:
    Eigen::Isometry3d plane_pose_ ;
    pcl::ModelCoefficients::Ptr plane_coeffs_;
    bool plane_pose_set_;

    ICPTracker* icp_tracker_;

    Isometry3dTime null_poseT_;    


    Eigen::Isometry3d object_pose_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_bb_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_;
    AffordanceUtils affutils;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string lidar_channel_, int affordance_id_): 
    lcm_(lcm_), lidar_channel_(lidar_channel_), affordance_id_(affordance_id_),
    null_poseT_(0, Eigen::Isometry3d::Identity()){

  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  lcm_->subscribe( lidar_channel_ ,&Pass::lidarHandler,this);
  lcm_->subscribe("AFFORDANCE_PLUS_COLLECTION",&Pass::affordancePlusHandler,this);  


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );

  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  float colors_b[] ={1.0,0.0,1.0};
  std::vector<float> colors_v;
  colors_v.assign(colors_b,colors_b+4*sizeof(float));

  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"[ICPApp] Pose - Null",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"[ICPApp] New Sweep"     ,1,1, 1000,1, {0,0,1}));
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1010,"[ICPApp] Pose - Previous",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1011,"[ICPApp] Aff Cloud [at Prev]"     ,1,1, 1010,1, {1,0,0}));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1012,"[ICPApp] Bounding Box [Prev]"     ,4,1, 1010,1, {1,0,0}));
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1020,"[ICPApp] Updated Pose",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1021,"[ICPApp] Aff Cloud [at Updated]"     ,1,1, 1020,1, {0,0.6,0}));

  
  counter_=0;
  got_affs_ = false;
  // Plane Detection:
  major_plane_ = new MajorPlane( lcm_, 1); // last variable defines verbosity
  plane_pose_.setIdentity();
  plane_pose_set_ = false;

  
  icp_tracker_ = new ICPTracker(lcm_, 0);
  pcl::ModelCoefficients::Ptr plane_coeffs_ptr_(new pcl::ModelCoefficients ());
  plane_coeffs_ = plane_coeffs_ptr_;
  
}


void Pass::init_tracking(bool use_plane_tracker, int plane_affordance_id){
  use_plane_tracker_ = use_plane_tracker;
  plane_affordance_id_ = plane_affordance_id;
}


void Pass::lidarHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_core::planar_lidar_t* msg){
  counter_++;
  if (counter_%30 ==0){ cout << counter_ << " | " << msg->utime << "\n";   }  

  // Update the head poses
  botframes_cpp_->get_trans_with_utime( botframes_ , "head", "local"  , msg->utime, head_to_local_);
  
  // Ask the maps collector for a plane:
  // true if we got a new sweep, make a box 5x5x5m centered around the robot's head
  // cout << "head_to_local_: " << head_to_local_.translation() << "\n";
  if (major_plane_->getSweep(head_to_local_.cast<float>().translation() ,  Eigen::Vector3f( 3., 3., 3.)) ){ 
    //plane_pose_set_ = major_plane_->trackPlane(plane_pose_, msg->utime);  
    //std::vector<float> plane_coeffs_ = major_plane_->getPlaneCoeffs();  
    
    cout << *plane_coeffs_ << " plane\n";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud = major_plane_->getSweepCloudWithoutPlane(0.03,
          plane_coeffs_);
    
    
    
    
    cout << "Got a new sweep with "<< new_cloud->points.size() <<" points\n";
    pc_vis_->pose_to_lcm_from_list(1000, null_poseT_);
    pc_vis_->ptcld_to_lcm_from_list(1001, *new_cloud, null_poseT_.utime, null_poseT_.utime);
    
    Isometry3dTime previous_object_poseT = Isometry3dTime(0, object_pose_); 
    pc_vis_->pose_to_lcm_from_list(1010, previous_object_poseT);
    pc_vis_->ptcld_to_lcm_from_list(1011, *object_cloud_, previous_object_poseT.utime, previous_object_poseT.utime);
    pc_vis_->ptcld_to_lcm_from_list(1012, *object_bb_cloud_,previous_object_poseT.utime, previous_object_poseT.utime);  

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_copy (new pcl::PointCloud<pcl::PointXYZRGB> ());
    object_cloud_copy = object_cloud_;
    icp_tracker_->doICPTracker( object_cloud_copy, new_cloud, object_pose_ );
    object_pose_ = icp_tracker_->getUpdatedPose();
    //std::cout << object_pose_.translation().transpose() << " new xyz\n";
    //std::cout << object_pose_.rotation() << " new rot\n";
    
    Isometry3dTime   updated_object_pose_T =  Isometry3dTime( 0, object_pose_);
    pc_vis_->pose_to_lcm_from_list(1020, updated_object_pose_T);
    pc_vis_->ptcld_to_lcm_from_list(1021, *object_cloud_, updated_object_pose_T.utime, updated_object_pose_T.utime);  
  }
}





std::vector<float> affordanceToPlane(std::vector<string> param_names, std::vector<double> params ){
  // Ridiculously hacky way of converting from plane affordance to plane coeffs.
  // the x-direction of the plane pose is along the axis - hence this
  
  std::map<string,double> am;
  for (size_t j=0; j< param_names.size(); j++){
    am[ param_names[j] ] = params[j];
  }
  Eigen::Quaterniond quat = euler_to_quat( am.find("yaw")->second , am.find("pitch")->second , am.find("roll")->second );             
  Eigen::Isometry3d transform;
  transform.setIdentity();
  transform.translation()  << am.find("x")->second , am.find("y")->second, am.find("z")->second;
  transform.rotate(quat);  

  Eigen::Isometry3d ztransform;
  ztransform.setIdentity();
  ztransform.translation()  << 0 ,0, 1; // determine a point 1m in the z direction... use this as the normal
  ztransform = transform*ztransform;
  float a =(float) ztransform.translation().x() -  transform.translation().x();
  float b =(float) ztransform.translation().y() -  transform.translation().y();
  float c =(float) ztransform.translation().z() -  transform.translation().z();
  float d = - (a*am.find("x")->second + b*am.find("y")->second + c*am.find("z")->second);
  
  /*
  cout << "pitch : " << 180.*am.find("pitch")->second/M_PI << "\n";
  cout << "yaw   : " << 180.*am.find("yaw")->second/M_PI << "\n";
  cout << "roll   : " << 180.*am.find("roll")->second/M_PI << "\n";   
  obj_cfg oconfig = obj_cfg(1251000,"Tracker | Affordance Pose Z",5,1);
  Isometry3dTime reinit_poseT = Isometry3dTime ( 0, ztransform );
  pc_vis_->pose_to_lcm(oconfig,reinit_poseT);
  */

  std::vector<float> plane = { a,b,c,d};
  return plane;
}

Eigen::Vector4f affordanceToCentroid(std::vector<string> param_names, std::vector<double> params ){
  std::map<string,double> am;
  for (size_t j=0; j< param_names.size(); j++){
    am[ param_names[j] ] = params[j];
  }

  Eigen::Vector4f plane_centroid( am.find("x")->second, am.find("y")->second , 
        am.find("z")->second, 0); // last element held at zero
  return plane_centroid;
}




void Pass::affordancePlusHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::affordance_plus_collection_t* msg){
  if  ( !got_affs_ ) {
    cout << "got affs\n";
    drc::affordance_plus_t a = msg->affs_plus[affordance_id_];
    object_pose_ = affutils.getPose( a.aff.param_names, a.aff.params );

    Isometry3dTime pT =  Isometry3dTime( 0, object_pose_);
    pc_vis_->pose_to_lcm_from_list(1010, pT);
    
    object_bb_cloud_ = affutils.getBoundingBoxCloud(a.aff.bounding_xyz, a.aff.bounding_rpy, a.aff.bounding_lwh);
    pc_vis_->ptcld_to_lcm_from_list(1012, *object_bb_cloud_,pT.utime, pT.utime);  
    
    Eigen::Vector3f boundbox_lower_left = -0.5* Eigen::Vector3f( a.aff.bounding_lwh[0], a.aff.bounding_lwh[1], a.aff.bounding_lwh[2]);
    Eigen::Vector3f boundbox_upper_right = 0.5* Eigen::Vector3f( a.aff.bounding_lwh[0], a.aff.bounding_lwh[1], a.aff.bounding_lwh[2]);
    icp_tracker_->setBoundingBox (boundbox_lower_left, boundbox_upper_right);

    
    /*
    // Read Point Cloud and Define Pose and Bounding Box:
    std::string hd_cloud_filename = "/home/mfallon/drc/software/perception/trackers/data_non_in_svn/drill_clouds/affordance_version/drill.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ( hd_cloud_filename, *object_cloud_) == -1){ // load the file
      cout << "Couldn't read " << hd_cloud_filename << ", quitting\n";
      exit(-1);
    } 
    cout << "read HD cloud with " << object_cloud_->points.size() << " points\n";  
    //Eigen::Vector3f boundbox_lower_left = Eigen::Vector3f( -0.15, -0.07, -0.135);
    //Eigen::Vector3f boundbox_upper_right = Eigen::Vector3f( 0.15, 0.18,  0.2);
    icp_tracker_->drawBoundingBox( object_pose_.cast<float>() );    
    */
    
    
    // TODO: convert a mesh affordance to a point cloud if required
    object_cloud_ = affutils.getCloudFromAffordance(a.points);
    pc_vis_->ptcld_to_lcm_from_list(1011, *object_cloud_, pT.utime, pT.utime);
    
    
    
    ///// PLANE CONSTRAINT
    drc::affordance_plus_t a_plane = msg->affs_plus[plane_affordance_id_];
    
    std::vector<float> p_coeffs = affordanceToPlane(a_plane.aff.param_names, a_plane.aff.params );
    Eigen::Vector4f p_centroid = affordanceToCentroid(a_plane.aff.param_names, a_plane.aff.params  );
    major_plane_->setPlane(p_coeffs, p_centroid);    
    
    cout << p_coeffs[0] << " " << p_coeffs[1] << " " << p_coeffs[2] << " " << p_coeffs[3] << "\n";
    plane_coeffs_->values = p_coeffs;
    cout << *plane_coeffs_ << " plane in\n";
    Eigen::Isometry3d plane_pose = major_plane_->determinePlanePose(plane_coeffs_, p_centroid);
    obj_cfg oconfig2 = obj_cfg(1351000,"Tracker | Affordance Plane",5,1);
    Isometry3dTime plane_poseT = Isometry3dTime ( 0, plane_pose );
    pc_vis_->pose_to_lcm(oconfig2,plane_poseT);
    
  }
  got_affs_ =true;  
}


int main(int argc, char ** argv) {
  string lidar_channel = "SCAN";
  int affordance_id=0;
  bool use_plane_tracker =false;  
  int plane_affordance_id=0;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(lidar_channel, "i", "lidar_channel","lidar_channel");
  opt.add(affordance_id, "a", "affordance_id","Affordance ID");
  opt.add(use_plane_tracker, "p", "use_plane_tracker","Use Plane Tracker");
  opt.add(plane_affordance_id, "l", "plane_affordance_id","Plane Affordance ID");
  opt.parse();
  std::cout << "lidar_channel: " << lidar_channel << "\n";    
  std::cout << "affordance_id: " << affordance_id << "\n";    
  std::cout << "use_plane_tracker: " << use_plane_tracker << "\n";    
  std::cout << "plane_affordance_id: " << plane_affordance_id << "\n";    

  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm, lidar_channel, affordance_id);
  app.init_tracking(use_plane_tracker, plane_affordance_id);

  
  cout << "Plane Tracker ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
