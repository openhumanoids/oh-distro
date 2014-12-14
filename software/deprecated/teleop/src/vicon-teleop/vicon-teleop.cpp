// Module to use raw Vicon data to produce end effector goals for the DRC robot 
// cpda, mfallon, jan 2013
#include <stdio.h>
#include <GL/gl.h>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>

#include <lcm/lcm-cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc_lcmtypes.hpp"

#include <lcmtypes/vicon.hpp>

//#include <drc_utils/Clock.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <ConciseArgs>

using namespace Eigen;

using namespace std;
using namespace boost::assign; // bring 'operator+()' into scope

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string output_type_, 
         double human_to_robot_scale_factor);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool verbose_;
    std::string output_type_;
    
    void sendRobotPlan( );
    void collectPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::plan_collect_t* msg);
    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);   
    void publishEndEffectorGoal(Isometry3dTime body_to_segment, std::string channel, std::string ee_name);
    void viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  vicon::body_t* msg);   
    
    bot_lcmgl_t* lcmgl_;
    
    pointcloud_vis* pc_vis_;
    int vis_counter_; // used for visualization
    int printf_counter_; // used for terminal feedback
    
    Isometry3dTime human_world_to_backT_, human_world_to_leftT_, human_world_to_rightT_;
    
    
    bool first_rstate_received_;
    
    int64_t dummy_utime_;

    Eigen::Isometry3d human_world_to_body_;    
    
    Eigen::Isometry3d world_to_robot_body_;    
    
    Eigen::Vector3d human_to_robot_scale_;
    
    
    // A vector of robot plans (intentionally using drc type)
    std::vector< drc::robot_state_t > robot_plan_;
    // Number of states for a full plan - keep to about 20-30 for now
    int n_plan_samples_;
    // Time between plan samples (This is the time simulated by gazebo)
    double plan_sample_period_; 
    int64_t last_plan_utime_;
    bool collect_plan_; // has the user told us to collect a plan?
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string output_type_, 
         double human_to_robot_scale_factor):
    human_world_to_backT_(0, Eigen::Isometry3d::Identity()),
    human_world_to_leftT_(0, Eigen::Isometry3d::Identity()),
    human_world_to_rightT_(0, Eigen::Isometry3d::Identity()),
    lcm_(lcm_), verbose_(verbose_), output_type_(output_type_){
  
  lcmgl_= bot_lcmgl_init(lcm_->getUnderlyingLCM(), "teleop-4markers");
  lcm_->subscribe("VICON_GET_PLAN",&Pass::collectPlanHandler,this);  
  lcm_->subscribe("EST_ROBOT_STATE",&Pass::robotStateHandler,this);  
  //drc::Clock::instance()->setLcm(lcm_);

  lcm_->subscribe( "VICON_DRC.*" ,&Pass::viconHandler,this);

  float colors_a[] ={1.0,0.0,0.0};
  vector <float> colors_v;
  colors_v.assign(colors_a,colors_a+4*sizeof(float));
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(70000,"World to Left",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(70001,"World to Right",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(70002,"Waist to Left",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(70003,"Waist to Right",5,1) );

  pc_vis_->obj_cfg_list.push_back( obj_cfg(70004,"Left EE Goal",4,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(70005,"Right EE Goal",4,1) );
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(70006,"World to Waist [Human]",5,1) );

  
  human_world_to_body_.setIdentity();
  
  double yaw = 180.0;
  Matrix3d m;
  m = AngleAxisd (  yaw*M_PI/180.0   , Vector3d::UnitZ ())
    * AngleAxisd (0, Vector3d::UnitY ())
    * AngleAxisd (0, Vector3d::UnitX ());  
  human_world_to_body_ *= m;
  
  human_world_to_body_.translation().x() = 0.77;
  human_world_to_body_.translation().y() = -1.08;
  human_world_to_body_.translation().z() = 0.66;

  vis_counter_ =0;
  printf_counter_ =0;

  // Currently the human_to_robot_scale is a single number in x,y,z:  
  human_to_robot_scale_= Eigen::Vector3d(human_to_robot_scale_factor,human_to_robot_scale_factor,human_to_robot_scale_factor);
  world_to_robot_body_.setIdentity();
  
  first_rstate_received_=false;
  
  // Nominal values:
  plan_sample_period_ = 0.1;
  n_plan_samples_ = 20;
  // Last time we stored a plan
  last_plan_utime_=0;
  
  dummy_utime_=0;
  collect_plan_= false;
  cout << "Finished setting up\n";  
}




void Pass::publishEndEffectorGoal(Isometry3dTime body_to_segment , std::string channel, std::string ee_name) //KDL::Frame &T_body_ee, 
{
  drc::ee_goal_t goalmsg;
  goalmsg.robot_name = "atlas";
  goalmsg.ee_name = ee_name;
  goalmsg.root_name = "pelvis";
  double x,y,z,w;

  goalmsg.ee_goal_pos.translation.x = body_to_segment.pose.translation().x();
  goalmsg.ee_goal_pos.translation.y = body_to_segment.pose.translation().y();
  goalmsg.ee_goal_pos.translation.z = body_to_segment.pose.translation().z();

  Eigen::Quaterniond r(body_to_segment.pose.rotation());
  goalmsg.ee_goal_pos.rotation.x = r.x();
  goalmsg.ee_goal_pos.rotation.y = r.y();
  goalmsg.ee_goal_pos.rotation.z = r.z();
  goalmsg.ee_goal_pos.rotation.w = r.w();

  goalmsg.ee_goal_twist.linear_velocity.x = 0.0;
  goalmsg.ee_goal_twist.linear_velocity.y = 0.0;
  goalmsg.ee_goal_twist.linear_velocity.z = 0.0;
  goalmsg.ee_goal_twist.angular_velocity.x = 0.0;
  goalmsg.ee_goal_twist.angular_velocity.y = 0.0;
  goalmsg.ee_goal_twist.angular_velocity.z = 0.0;

  goalmsg.num_chain_joints  = 6;
  // No specified posture bias
  goalmsg.use_posture_bias  = false;
  goalmsg.joint_posture_bias.resize(goalmsg.num_chain_joints);
  goalmsg.chain_joint_names.resize(goalmsg.num_chain_joints);
  for(int i = 0; i < goalmsg.num_chain_joints; i++){
      goalmsg.joint_posture_bias[i]=0;
      goalmsg.chain_joint_names[i]= "dummy_joint_names";
  }

  // Publish the message
  goalmsg.halt_ee_controller = false;

  lcm_->publish(channel, &goalmsg);
};



void Pass::viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  vicon::body_t* msg){
  if ((msg->trans[0] ==0)  && (msg->trans[1] ==0) ){ // ignore null return
    return;
  }
  
  if (verbose_){
    // Visualise the human's fixed body/pelvis position
    Isometry3dTime human_world_to_bodyT = Isometry3dTime( msg->utime , human_world_to_body_ );
    pc_vis_->pose_to_lcm_from_list(70006, human_world_to_bodyT);    
  }
  
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()  << msg->trans[0], msg->trans[1], msg->trans[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->quat[0], msg->quat[1], msg->quat[2], msg->quat[3] );
  pose.rotate(quat);   
  Isometry3dTime segment_poseT = Isometry3dTime( msg->utime , pose );
 
  if (channel.compare( "VICON_DRC_RightHand" ) == 0){
    human_world_to_rightT_= segment_poseT;
  }else if (channel.compare( "VICON_DRC_LeftHand" ) == 0){
    human_world_to_leftT_= segment_poseT;
    return;
  } else if (channel.compare( "VICON_DRC_Back" ) == 0){
    human_world_to_backT_= segment_poseT;
    return;
  }
  // We will only transmit ANYTHING when a valide right hand marker is detected
  // TODO: refactor to transmit individual marks, if required:

  if (verbose_){
    pc_vis_->pose_to_lcm_from_list(70000, human_world_to_leftT_);
	  pc_vis_->pose_to_lcm_from_list(70001, human_world_to_rightT_);
  }
  
  // 1. Find segments relative to the person's body
  human_world_to_leftT_.pose = human_world_to_body_.inverse() * human_world_to_leftT_.pose ;
  human_world_to_rightT_.pose = human_world_to_body_.inverse() * human_world_to_rightT_.pose ;

  // 2. Scale the end effector:
  human_world_to_leftT_.pose.translation().x() *= human_to_robot_scale_(0);
  human_world_to_leftT_.pose.translation().y() *= human_to_robot_scale_(1);
  human_world_to_leftT_.pose.translation().z() *= human_to_robot_scale_(2);
  
  human_world_to_rightT_.pose.translation().x() *= human_to_robot_scale_(0);
  human_world_to_rightT_.pose.translation().y() *= human_to_robot_scale_(1);
  human_world_to_rightT_.pose.translation().z() *= human_to_robot_scale_(2);
  
  if (verbose_){
    pc_vis_->pose_to_lcm_from_list(70002, human_world_to_leftT_);
    pc_vis_->pose_to_lcm_from_list(70003, human_world_to_rightT_);    
  }
  
  if (!first_rstate_received_){
    cout << "No Robot State yet. will not publish End Effector Goal\n";
    return; 
  }
  
  // 7. Determine the end effector goals and publish them to the controllers:
  if ((output_type_.compare( "left" ) == 0) || (output_type_.compare( "both" ) == 0) ) {
    publishEndEffectorGoal( human_world_to_leftT_,  "L_HAND_GOAL","l_hand");
    human_world_to_leftT_.pose = world_to_robot_body_*human_world_to_leftT_.pose;
    pc_vis_->pose_to_lcm_from_list(70004, human_world_to_leftT_);
  }
    
  if ((output_type_.compare( "right" ) == 0) || (output_type_.compare( "both" ) == 0) ) {
    publishEndEffectorGoal( human_world_to_rightT_,  "R_HAND_GOAL","r_hand");
    human_world_to_rightT_.pose = world_to_robot_body_*human_world_to_rightT_.pose;
    pc_vis_->pose_to_lcm_from_list(70005, human_world_to_rightT_);
  }
}


void Pass::sendRobotPlan( ){
  drc::robot_plan_t plan_msg;
  plan_msg.utime = robot_plan_[  robot_plan_.size() -1 ].utime ;//drc::Clock::instance()->getCurrentTime();
  plan_msg.robot_name =   robot_plan_[0].robot_name;
  plan_msg.num_states = robot_plan_.size();
  plan_msg.plan = robot_plan_;
  lcm_->publish("CANDIDATE_ROBOT_PLAN", &plan_msg);
  robot_plan_.clear(); // clear it out
  collect_plan_ = false;
  cout << "Sent CANDIDATE_ROBOT_PLAN msg\n";
}


void Pass::robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  if (first_rstate_received_==false){
    cout << "got first Robot State @ " << msg->utime << "\n";
    first_rstate_received_=true;
  }
  
  // Extract World to Body TF:
  world_to_robot_body_.setIdentity();
  world_to_robot_body_.translation()  << msg->origin_position.translation.x, msg->origin_position.translation.y, msg->origin_position.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->origin_position.rotation.w, msg->origin_position.rotation.x, 
                                               msg->origin_position.rotation.y, msg->origin_position.rotation.z);
  world_to_robot_body_.rotate(quat);
  
  if (collect_plan_){
    if (  msg->utime - (plan_sample_period_ *1E6) > last_plan_utime_ ){
      last_plan_utime_ = msg->utime;
      if ( robot_plan_.size() < n_plan_samples_ ){
        drc::robot_state_t msgcopy = *msg;
        robot_plan_.push_back( msgcopy);
     }
      if (robot_plan_.size() == n_plan_samples_ ){
        sendRobotPlan();
      }
    }
  }
}

// External command telling this program to save a history of states:
void Pass::collectPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::plan_collect_t* msg){
  n_plan_samples_ = msg->n_plan_samples;
  plan_sample_period_ = msg->sample_period;
  // TODO: insert type here
  
  // Start collecting:
  collect_plan_ = true;
  std::cout << "Starting to collect plan...\n";
}

int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool verbose=FALSE;
  string output_type="both";
  double scale=1.3;
  parser.add(verbose, "v", "verbose", "Verbosity");
  parser.add(output_type, "o", "output_type", "Command: left right both");
  parser.add(scale, "s", "scale", "Human to Robot Scale Factor");
  parser.parse();
  cout << verbose << " is verbose\n";
  cout << output_type << " is output_type\n";
  cout << scale << " is scale\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm, verbose,  output_type,  scale);
  cout << "Ready to process incoming LCM" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
