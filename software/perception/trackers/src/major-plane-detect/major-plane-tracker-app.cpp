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

using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string lidar_channel_, int plane_affordance_id);
    
    ~Pass(){
    }    
    
  private:
    
    boost::shared_ptr<lcm::LCM> lcm_;
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg);   
    std::string lidar_channel_;

    void affordanceHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::affordance_collection_t* msg);
    
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;

    bool got_affs_;    
    
    int counter_;
    pointcloud_vis* pc_vis_;
    
    Eigen::Isometry3d head_to_local_;
    
    // Tracker Configuration:
    int plane_affordance_id_;
    MajorPlane* major_plane_;
    
    // Plane Tracking Variables:
    Eigen::Isometry3d plane_pose_ ;
    bool plane_pose_set_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string lidar_channel_, int plane_affordance_id_): 
    lcm_(lcm_), lidar_channel_(lidar_channel_), plane_affordance_id_(plane_affordance_id_){

  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  lcm_->subscribe( lidar_channel_ ,&Pass::lidarHandler,this);
  lcm_->subscribe("AFFORDANCE_COLLECTION",&Pass::affordanceHandler,this);  
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );

  counter_=0;
  got_affs_ = false;
  // Plane Detection:
  major_plane_ = new MajorPlane( lcm_, 2); // last variable defines verbosity
  plane_pose_.setIdentity();
  plane_pose_set_ = false;
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
    plane_pose_set_ = major_plane_->trackPlane(plane_pose_, msg->utime);  
  }
}


Eigen::Isometry3d affordanceToIsometry3d(std::vector<string> param_names, std::vector<double> params ){
  std::map<string,double> am;
  for (size_t j=0; j< param_names.size(); j++){
    am[ param_names[j] ] = params[j];
  }
  Eigen::Quaterniond quat = euler_to_quat( am.find("yaw")->second , am.find("pitch")->second , am.find("roll")->second );             
  Eigen::Isometry3d transform;
  transform.setIdentity();
  transform.translation()  << am.find("x")->second , am.find("y")->second, am.find("z")->second;
  transform.rotate(quat);  
  return transform;
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


void Pass::affordanceHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::affordance_collection_t* msg){
  if  ( !got_affs_ ) {
    cout << "got affs\n";
    
    std::vector<float> p_coeffs = affordanceToPlane(msg->affs[plane_affordance_id_].param_names, msg->affs[plane_affordance_id_].params );
    Eigen::Vector4f p_centroid = affordanceToCentroid(msg->affs[plane_affordance_id_].param_names, msg->affs[plane_affordance_id_].params  );
    major_plane_->setPlane(p_coeffs, p_centroid);    
    
    cout << "About to start tracking plane: " << p_coeffs[0] << " " << p_coeffs[1] << " " << p_coeffs[2] << " " << p_coeffs[3] << "\n";
    pcl::ModelCoefficients::Ptr new_p_coeffs(new pcl::ModelCoefficients ());
    new_p_coeffs->values = p_coeffs;
    Eigen::Isometry3d p = major_plane_->determinePlanePose(new_p_coeffs, p_centroid);
    obj_cfg oconfig2 = obj_cfg(1351000,"Tracker | Affordance Plane",5,1);
    Isometry3dTime pT = Isometry3dTime ( 0, p );
    pc_vis_->pose_to_lcm(oconfig2,pT);
  }
  got_affs_ =true;  
}


int main(int argc, char ** argv) {
  string lidar_channel = "SCAN";
  int plane_affordance_id=0;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(lidar_channel, "i", "lidar_channel","lidar_channel");
  opt.add(plane_affordance_id, "l", "plane_affordance_id","Plane Affordance ID");
  opt.parse();
  std::cout << "lidar_channel: " << lidar_channel << "\n";    
  std::cout << "plane_affordance_id: " << plane_affordance_id << "\n";    

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm, lidar_channel, plane_affordance_id);
  
  cout << "Plane Tracker ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
