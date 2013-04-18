// Need to better define the plane affordance: is it a box or is it a plane
// Need to ensure that affordance-to-planecoeffs is stable 

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
#include <affordance/AffordanceUtils.hpp>

using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string lidar_channel_, int tracker_instance_id_, int verbose_);
    
    ~Pass(){
    }    
    
    void initiate_tracking(int plane_affordance_id);    
    //void affordancetoplane();
  private:
    int verbose_;
    boost::shared_ptr<lcm::LCM> lcm_;
    std::string lidar_channel_;
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg);   
    void affordancePlusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::affordance_plus_collection_t* msg);
    void trackerCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::tracker_command_t* msg);
    
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;

    void publishUpdatedAffordance();
    
    // unique id of this tracker instance. Plane=0, otherwise above that
    int tracker_instance_id_;

    int counter_;
    pointcloud_vis* pc_vis_;
    
    Eigen::Isometry3d head_to_local_;
    

    // Tracker State Engine:
    bool tracker_initiated_;
    bool got_initial_affordance_;    

    // Tracker Configuration:
    int plane_affordance_id_;
    MajorPlane* major_plane_;
    
    // Plane Tracking Variables:
    Eigen::Isometry3d plane_pose_ ;
    bool plane_pose_set_;
    drc::affordance_plus_t last_tracked_affp_msg_;
    
    AffordanceUtils affutils;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string lidar_channel_, int tracker_instance_id_, int verbose_): 
    lcm_(lcm_), lidar_channel_(lidar_channel_), tracker_instance_id_(tracker_instance_id_), verbose_(verbose_){

  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  lcm_->subscribe( lidar_channel_ ,&Pass::lidarHandler,this);
  lcm_->subscribe("AFFORDANCE_PLUS_COLLECTION",&Pass::affordancePlusHandler,this);  
  lcm_->subscribe("TRACKER_COMMAND",&Pass::trackerCommandHandler,this);  
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );

  counter_=0;
  got_initial_affordance_ = false;
  // Plane Detection:
  major_plane_ = new MajorPlane( lcm_, verbose_); // last variable defines verbosity
  plane_pose_.setIdentity();
  plane_pose_set_ = false;
  
  plane_affordance_id_ = -1;
  tracker_initiated_ = false; // by default start uninitiated
}

void Pass::publishUpdatedAffordance(){
  last_tracked_affp_msg_.aff.aff_store_control =  drc::affordance_t::UPDATE;
  //affutils.setXYZRPYFromPlane(last_tracked_affp_msg_.aff.param_names, last_tracked_affp_msg_.aff.params, major_plane_->getPlaneCoeffs(), plane_pose_.translation() );
  affutils.setXYZRPYFromPlane(last_tracked_affp_msg_.aff.origin_xyz, last_tracked_affp_msg_.aff.origin_rpy, 
                              major_plane_->getPlaneCoeffs(), plane_pose_.translation() );
  
  // Update the affordance hull:
  //std::cout << "publishUpdatedAffordance AFFORDANCE update: points\n";
  
  std::vector< std::vector<float> > points = major_plane_->getPlaneHull();
  last_tracked_affp_msg_.points = points;
  last_tracked_affp_msg_.npoints = points.size();
  // add triangles to message (duplicates what Paul has in segmentation gui:
  last_tracked_affp_msg_.ntriangles = points.size()-2;
  last_tracked_affp_msg_.triangles.resize(last_tracked_affp_msg_.ntriangles);
  for(int i=0; i<last_tracked_affp_msg_.ntriangles; i++){
    Eigen::Vector3i triangle(0,i+1,i+2);
    last_tracked_affp_msg_.triangles[i].resize(3);
    for(int j=0;j<3;j++) last_tracked_affp_msg_.triangles[i][j] = triangle[j];
  }
  lcm_->publish("AFFORDANCE_PLUS_TRACK", &last_tracked_affp_msg_);
  //lcm_->publish("AFFORDANCE_TRACK", &last_tracked_affp_msg_.aff);
}

void Pass::lidarHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_core::planar_lidar_t* msg){
  counter_++;
  if (counter_%30 ==0){ cout << counter_ << " | " << msg->utime << "\n";   }  

  if (!tracker_initiated_ || !got_initial_affordance_){
    // haven't been initated or have got bootstrapping affordances, quit ...
    return;
  }
  
  // Update the head poses
  botframes_cpp_->get_trans_with_utime( botframes_ , "head", "local"  , msg->utime, head_to_local_);
  
  // Ask the maps collector for a plane:
  // true if we got a new sweep, make a box 2x2x2m centered around the robot's head
  // cout << "head_to_local_: " << head_to_local_.translation() << "\n";
  if (major_plane_->getSweep(head_to_local_.cast<float>().translation() ,  Eigen::Vector3f( 2., 2., 2.)) ){ 
    plane_pose_set_ = major_plane_->trackPlane(plane_pose_, msg->utime);  
    publishUpdatedAffordance();
    cout << "Plane Update published\n";
  }
}


void Pass::affordancePlusHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::affordance_plus_collection_t* msg){
  if (!tracker_initiated_){
    // If we haven't told tracker what do to, then ignore affordances
    return; 
  }

  
  std::vector<int> uids;
  for (size_t i=0; i<msg->affs_plus.size(); i++){
    uids.push_back( msg->affs_plus[i].aff.uid );
  }
  int aff_iter = std::distance( uids.begin(), std::find( uids.begin(), uids.end(), plane_affordance_id_ ) );
  if (aff_iter == msg->affs_plus.size()){
     cout << "Error: Plane Affordance UID " <<  plane_affordance_id_ << " could not be found\n";
     return;
  }  
  
  // bootstrap tracker with required affordances:
  // Theres alot of grandfathered stuff here...
  if  ( !got_initial_affordance_ ) {
    cout << "got affs\n";
    drc::affordance_plus_t a = msg->affs_plus[aff_iter];
    std::vector<float> p_coeffs;
    Eigen::Vector3d p_centroid_3d;
    affutils.setPlaneFromXYZYPR(a.aff.origin_xyz, a.aff.origin_rpy, p_coeffs, p_centroid_3d );
    Eigen::Vector4f p_centroid = Eigen::Vector4f( p_centroid_3d(0), p_centroid_3d(1), p_centroid_3d(2), 0);
    major_plane_->setPlane(p_coeffs, p_centroid);    
    last_tracked_affp_msg_ = a;
    
    cout << "about to start tracking plane: " << p_coeffs[0] << " " << p_coeffs[1] << " " << p_coeffs[2] << " " << p_coeffs[3] << "\n";
    if (verbose_ >=1){
      pcl::ModelCoefficients::Ptr new_p_coeffs(new pcl::ModelCoefficients ());
      new_p_coeffs->values = p_coeffs;
      Eigen::Isometry3d p = major_plane_->determinePlanePose(new_p_coeffs, p_centroid);
      obj_cfg oconfig2 = obj_cfg(1351000,"Tracker | Affordance plane",5,1);
      Isometry3dTime pt = Isometry3dTime( 0, p );
      pc_vis_->pose_to_lcm(oconfig2,pt);
    }
  }
  got_initial_affordance_ =true;  
  
}

void Pass::initiate_tracking(int plane_affordance_id){
  plane_affordance_id_ = plane_affordance_id;
  tracker_initiated_ = true;
  cout << "Initiating Tracker on Affordance " << plane_affordance_id_ << endl;
}

void Pass::trackerCommandHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::tracker_command_t* msg){
  cout << "got tracker command\n";
  
  if ( msg->tracker_id != tracker_instance_id_){
    cout << "my tracker id is ["<< (int) tracker_instance_id_ 
         <<"] but command is for ["<< (int) msg->tracker_id <<"]. Ignoring command\n";
         return;
  }
  
  if (msg->tracker_type== drc::tracker_command_t::STOP){
    tracker_initiated_ = false;
    got_initial_affordance_ = false;
    cout << "Halting Tracker on command\n";
    return;
  }else if(msg->tracker_type== drc::tracker_command_t::PLANE){ 
    // only if the tracker type is 1 (PLANE) will I track
    // I'm only a plane after all!
    initiate_tracking(msg->uid);
  }else{
    cout << "Tracker Type not understood or not plane ["<< ((int)msg->tracker_type)  <<"]\n";
  }
}

int main(int argc, char ** argv) {
  string lidar_channel = "SCAN";
  int plane_affordance_id=-1; // by default dont track anything
  int tracker_instance_id=0; // id of this tracker instance. Plane Tracker==0, otherwise above that
  int verbose=0;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(lidar_channel, "i", "lidar_channel","lidar_channel");
  opt.add(plane_affordance_id, "l", "plane_affordance_id","Plane Affordance ID");
  opt.add(tracker_instance_id, "t", "tracker","Tracker ID");
  opt.add(verbose, "v", "verbosity","0 none, 1 little, 2 debug");
  opt.parse();
  std::cout << "lidar_channel: " << lidar_channel << "\n";    
  std::cout << "plane_affordance_id: " << plane_affordance_id << "\n";    
  std::cout << "tracker_instance_id: " << tracker_instance_id << "\n";    
  std::cout << "verbose: " << verbose << "\n";    

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm, lidar_channel, tracker_instance_id, verbose);
  
  if (plane_affordance_id>=0){
    // if a valid id - initiate tracker on launch
    app.initiate_tracking(plane_affordance_id);
  }else{
    cout << "Starting Plane Tracker Uninitiated\n";
  }
  
  cout << "Plane Tracker ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
