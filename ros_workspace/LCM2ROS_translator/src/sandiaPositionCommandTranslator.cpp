#include <ros/ros.h>
#include <cstdlib>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <std_msgs/Float64.h>
#include <map>
#include <string>

using namespace std;

class PositionCommandHandler{
  private:
  
    std::map<std::string, ros::Publisher> pub_map;
    ros::NodeHandle position_command_node;
    
  public:
    PositionCommandHandler(ros::NodeHandle &node): position_command_node(node) {
       
      pub_map.insert(std::make_pair("right_f0_j0",position_command_node.advertise<std_msgs::Float64>("/right_f0_j0_position_controller/command",10)));
      pub_map.insert(std::make_pair("right_f0_j1",position_command_node.advertise<std_msgs::Float64>("/right_f0_j1_position_controller/command",10)));
      pub_map.insert(std::make_pair("right_f0_j2",position_command_node.advertise<std_msgs::Float64>("/right_f0_j2_position_controller/command",10)));
      pub_map.insert(std::make_pair("right_f1_j0",position_command_node.advertise<std_msgs::Float64>("/right_f1_j0_position_controller/command",10)));
      pub_map.insert(std::make_pair("right_f1_j1",position_command_node.advertise<std_msgs::Float64>("/right_f1_j1_position_controller/command",10)));
      pub_map.insert(std::make_pair("right_f1_j2",position_command_node.advertise<std_msgs::Float64>("/right_f1_j2_position_controller/command",10)));

      pub_map.insert(std::make_pair("right_f2_j0",position_command_node.advertise<std_msgs::Float64>("/right_f2_j0_position_controller/command",10)));
      pub_map.insert(std::make_pair("right_f2_j1",position_command_node.advertise<std_msgs::Float64>("/right_f2_j1_position_controller/command",10)));
      pub_map.insert(std::make_pair("right_f2_j2",position_command_node.advertise<std_msgs::Float64>("/right_f2_j2_position_controller/command",10)));
      pub_map.insert(std::make_pair("right_f3_j0",position_command_node.advertise<std_msgs::Float64>("/right_f3_j0_position_controller/command",10)));
      pub_map.insert(std::make_pair("right_f3_j1",position_command_node.advertise<std_msgs::Float64>("/right_f3_j1_position_controller/command",10)));
      pub_map.insert(std::make_pair("right_f3_j2",position_command_node.advertise<std_msgs::Float64>("/right_f3_j2_position_controller/command",10)));

      pub_map.insert(std::make_pair("real_base_x",position_command_node.advertise<std_msgs::Float64>("/real_base_x_position_controller/command",10)));
      pub_map.insert(std::make_pair("real_base_y",position_command_node.advertise<std_msgs::Float64>("/real_base_y_position_controller/command",10)));
      pub_map.insert(std::make_pair("real_base_z",position_command_node.advertise<std_msgs::Float64>("/real_base_z_position_controller/command",10)));
      pub_map.insert(std::make_pair("real_base_roll",position_command_node.advertise<std_msgs::Float64>("/real_base_roll_position_controller/command",10)));
      pub_map.insert(std::make_pair("real_base_pitch",position_command_node.advertise<std_msgs::Float64>("/real_base_pitch_position_controller/command",10)));
      pub_map.insert(std::make_pair("real_base_yaw",position_command_node.advertise<std_msgs::Float64>("/real_base_yaw_position_controller/command",10)));
    }
    ~PositionCommandHandler() {}
  
    void position_command_callback(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::joint_angles_t* msg)//what is rbuf and channel here?
    {
      std_msgs::Float64 position_command_msg;
      for (int i=0; i<msg->num_joints; i++) {
          // should we instead publish all joints at the same time?
          position_command_msg.data = msg->joint_position[i];
        if(ros::ok()) {
          pub_map[msg->joint_name[i]].publish(position_command_msg);
          //cout<<"publishing" << msg->joint_name[i] << endl;
        }
      }
      //ros::spinOnce(); // required?
    }  
};

int main(int argc,char** argv) {
  ros::init(argc,argv,"position_command_publisher",ros::init_options::NoSigintHandler);
    ros::NodeHandle position_command_node;

  lcm::LCM listener;
  if(!listener.good())
    return 1;
  PositionCommandHandler handlerObject(position_command_node);
  
  //LCM subscription
  listener.subscribe("JOINT_POSITION_CMDS",&PositionCommandHandler::position_command_callback,&handlerObject);

  while(0 == listener.handle());
  return 0;
}


