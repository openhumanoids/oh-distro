
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <lcm/lcm-cpp.hpp>


#include <lcmtypes/drc/atlas_state_t.hpp>
#include <lcmtypes/drc/robot_state_t.hpp>


#include <drc_utils/joint_utils.hpp>

using namespace std;
///////////////////////////////////////////////////////////////
class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_);

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    JointUtils joint_utils_;

    void atlasHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg);

};

App::App(boost::shared_ptr<lcm::LCM> &lcm_):lcm_(lcm_){

  lcm::Subscription* sub0 = lcm_->subscribe("ATLAS_STATE",&App::atlasHandler,this);
  sub0->setQueueCapacity(1);

}



std::vector<float> joint_position_prev;
int64_t utime_prev;

void App::atlasHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                              const  drc::atlas_state_t* msg){

  std::vector<float> num_velocity = msg->joint_position;
  if ( utime_prev != 0){
    for (size_t i=0; i <  msg->joint_position.size(); i++){
      num_velocity[i] = (msg->joint_position[i] - joint_position_prev[i] )/ ((msg->utime- utime_prev)*1E-6);
    }
  }

  joint_position_prev = msg->joint_position; 
  utime_prev = msg->utime;  

  drc::robot_state_t robot_state_msg;
  robot_state_msg.utime = msg->utime;
  // Pelvis Pose:
  robot_state_msg.pose.translation.x =0;
  robot_state_msg.pose.translation.y =0;
  robot_state_msg.pose.translation.z =0;
  robot_state_msg.pose.rotation.w = 1;
  robot_state_msg.pose.rotation.x = 0;
  robot_state_msg.pose.rotation.y = 0;
  robot_state_msg.pose.rotation.z = 0;
  robot_state_msg.twist.linear_velocity.x  = 0;
  robot_state_msg.twist.linear_velocity.y  = 0;
  robot_state_msg.twist.linear_velocity.z  = 0;
  robot_state_msg.twist.angular_velocity.x = 0;
  robot_state_msg.twist.angular_velocity.y = 0;
  robot_state_msg.twist.angular_velocity.z = 0;  
  
  for (size_t i = 0; i < msg->joint_position.size(); i++)  {
//    robot_state_msg.joint_name.push_back( atlas_joints_.name[i] );
    robot_state_msg.joint_position.push_back( msg->joint_position[i] );
    robot_state_msg.joint_velocity.push_back( num_velocity[i]);
    robot_state_msg.joint_effort.push_back( num_velocity[i] );
  }  
  robot_state_msg.joint_name = joint_utils_.atlas_joint_names;  

  robot_state_msg.num_joints = robot_state_msg.joint_position.size();
  
  lcm_->publish("EST_ROBOT_STATE_ECHO", &robot_state_msg);  
}


int main(int argc, char ** argv) {

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App app(lcm);
  std::cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
