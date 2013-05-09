// Tool to recreate EST_ROBOT_STATE from  minimal version on the base station
#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include <lcmtypes/bot_core.hpp>
#include <boost/shared_ptr.hpp>
#include <ConciseArgs>
#include <bot_param/param_client.h>

using namespace std;

class StatePub
{
public:
  StatePub(boost::shared_ptr<lcm::LCM> &lcm_, std::string output_channel_);
  ~StatePub() {}
  boost::shared_ptr<lcm::LCM> lcm_;
  
  void handleMinimalRobotStateMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::minimal_robot_state_t * msg);
  void handleImageMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const bot_core::image_t* msg);
  void sendPose(drc::position_3d_t &origin, int64_t utime, std::string channel);

private:
  std::string output_channel_;
  std::vector <string> joint_names_;

  BotParam* botparam_;  
};

StatePub::StatePub(boost::shared_ptr<lcm::LCM> &lcm_, std::string output_channel_):
    lcm_(lcm_), output_channel_(output_channel_){
  lcm_->subscribe("EST_ROBOT_STATE_MINIMAL", &StatePub::handleMinimalRobotStateMsg, this);
  lcm_->subscribe("CAMERALEFT_COMPRESSED", &StatePub::handleImageMsg, this);
  

  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  std::string joint_names_root = "joint_names";
  char joint_names_key[512];  
  sprintf(joint_names_key, "network.%s",joint_names_root.c_str() );

  // Read the keys:  
  char **fnames = bot_param_get_str_array_alloc(botparam_, joint_names_key );
  std::vector <string> channels;
  for (int j = 0; fnames && fnames[j]!=NULL; j++) {
    joint_names_.push_back( fnames[j]  );
  } 

  std::cout << joint_names_key << " has been read\n";
  cout << "Assumed joint order:\n";
  for (size_t j = 0; j < joint_names_.size(); j++) {
    cout << j << " " << joint_names_[j] <<"\n";
  } 
}

void StatePub::handleImageMsg(const lcm::ReceiveBuffer* rbuf,
  const std::string& chan, const bot_core::image_t * msg){

  lcm_->publish( "CAMERALEFT", msg);
}

void StatePub::sendPose(drc::position_3d_t &origin, int64_t utime, std::string channel){
  bot_core::pose_t pose_msg;
  pose_msg.utime = utime;
  pose_msg.pos[0] = origin.translation.x;
  pose_msg.pos[1] = origin.translation.y;
  pose_msg.pos[2] = origin.translation.z;
  pose_msg.orientation[0] = origin.rotation.w;
  pose_msg.orientation[1] = origin.rotation.x;
  pose_msg.orientation[2] = origin.rotation.y;
  pose_msg.orientation[3] = origin.rotation.z;
  lcm_->publish(channel, &pose_msg);
}

void StatePub::handleMinimalRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
  const std::string& chan, const drc::minimal_robot_state_t * msg){

  /// Fill in the contents we know:
  drc::robot_state_t msgout;
  msgout.utime = msg->utime;
  msgout.robot_name = "atlas";
  msgout.origin_position = msg->origin_position;
  msgout.num_joints = msg->num_joints;
  msgout.joint_name = joint_names_;
  msgout.joint_position = msg->joint_position;
  
  /// THE REST OF THE MESSAGE IS CREATED FOR ZERO VALUES:  
  std::vector<float> joint_zeros;
  joint_zeros.assign ( joint_names_.size(),0); 
  drc::joint_covariance_t j_cov;
  j_cov.variance = 0;
  std::vector<drc::joint_covariance_t> joint_cov_zeros;
  joint_cov_zeros.assign ( joint_names_.size(),j_cov);  
  msgout.origin_twist.linear_velocity.x =0;
  msgout.origin_twist.linear_velocity.y =0;
  msgout.origin_twist.linear_velocity.z =0;
  msgout.origin_twist.angular_velocity.x =0;
  msgout.origin_twist.angular_velocity.y =0;
  msgout.origin_twist.angular_velocity.z =0;  
  for(int i = 0; i < 6; i++)  {
    for(int j = 0; j < 6; j++) {
      msgout.origin_cov.position_cov[i][j] = 0;
      msgout.origin_cov.twist_cov[i][j] = 0;
    }
  }
  msgout.joint_velocity = joint_zeros;
  msgout.measured_effort = joint_zeros;
  msgout.joint_cov = joint_cov_zeros;
  
  drc::contact_state_t cs;
  cs.num_contacts =0; /// THIS MIGHT NEED TO BE FILLED IN EVENTUALLY 
  msgout.contacts = cs;
  /////////////////////////////////////////////////////////
  
  
  lcm_->publish( output_channel_, &msgout);        
  sendPose(msgout.origin_position, msgout.utime, "POSE_BODY");

  drc::utime_t utime_out;
  utime_out.utime = msgout.utime;
  lcm_->publish( "ROBOT_UTIME", &utime_out);        

  cout << "Minimal message received and republished "<< msg->utime<<"\n";
}

int main (int argc, char ** argv){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  string output_channel="TRUE_ROBOT_STATE"; // publishing TRUE_ROBOT_STATE is kind of hacky - its makes it easier for j2f and ersp to work
  string role = "robot";
  parser.add(output_channel, "g", "output_channel", "Output reconstututed message as");
  parser.add(role, "r", "role","Role - robot or base [assumed]");  
  parser.parse();
  cout << "output_channel is: " << output_channel << "\n"; 
  std::cout << "role: " << role << "\n";
  
  string lcm_url="";
  std::string role_upper;
  for(short i = 0; i < role.size(); ++i)
     role_upper+= (std::toupper(role[i]));
  if((role.compare("robot") == 0) || (role.compare("base") == 0) ){
    for(short i = 0; i < role_upper.size(); ++i)
       role_upper[i] = (std::toupper(role_upper[i]));
    string env_variable_name = string("LCM_URL_DRC_" + role_upper); 
    char* env_variable;
    env_variable = getenv (env_variable_name.c_str());
    if (env_variable!=NULL){
      //printf ("The env_variable is: %s\n",env_variable);      
      lcm_url = string(env_variable);
    }else{
      std::cout << env_variable_name << " environment variable has not been set ["<< lcm_url <<"]\n";     
      exit(-1);
    }
  }else{
    std::cout << "Role not understood, choose: robot or base\n";
    return 1;
  }   
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM(lcm_url) );
  if(!lcm->good())
    return 1;

  StatePub app(lcm, output_channel);
  cout << "Minimal to Est ready"<< endl;
  while(0 == lcm->handle());
  return 0;
}
