#include <ros/ros.h>
#include <cstdlib>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <std_msgs/Float64.h>
#include <map>
#include <string>

class PositionCommandHandler{
  private:
  
    std::map<std::string, ros::Publisher> pub_map;
    ros::NodeHandle position_command_node;
    
  public:
    PositionCommandHandler(ros::NodeHandle &node): position_command_node(node) {
       
      pub_map.insert(std::make_pair("l_arm_elx",position_command_node.advertise<std_msgs::Float64>("/l_arm_elx_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_arm_ely",position_command_node.advertise<std_msgs::Float64>("/l_arm_ely_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_arm_mwx",position_command_node.advertise<std_msgs::Float64>("/l_arm_mwx_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_arm_shx",position_command_node.advertise<std_msgs::Float64>("/l_arm_shx_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_arm_usy",position_command_node.advertise<std_msgs::Float64>("/l_arm_usy_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_arm_uwy",position_command_node.advertise<std_msgs::Float64>("/l_arm_uwy_position_controller/command",10)));

      pub_map.insert(std::make_pair("r_arm_elx",position_command_node.advertise<std_msgs::Float64>("/r_arm_elx_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_arm_ely",position_command_node.advertise<std_msgs::Float64>("/r_arm_ely_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_arm_mwx",position_command_node.advertise<std_msgs::Float64>("/r_arm_mwx_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_arm_shx",position_command_node.advertise<std_msgs::Float64>("/r_arm_shx_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_arm_usy",position_command_node.advertise<std_msgs::Float64>("/r_arm_usy_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_arm_uwy",position_command_node.advertise<std_msgs::Float64>("/r_arm_uwy_position_controller/command",10)));

      pub_map.insert(std::make_pair("l_leg_kny",position_command_node.advertise<std_msgs::Float64>("/l_leg_kny_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_leg_lax",position_command_node.advertise<std_msgs::Float64>("/l_leg_lax_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_leg_lhy",position_command_node.advertise<std_msgs::Float64>("/l_leg_lhy_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_leg_mhx",position_command_node.advertise<std_msgs::Float64>("/l_leg_mhx_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_leg_uay",position_command_node.advertise<std_msgs::Float64>("/l_leg_uay_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_leg_uhz",position_command_node.advertise<std_msgs::Float64>("/l_leg_uhz_position_controller/command",10)));

      pub_map.insert(std::make_pair("r_leg_kny",position_command_node.advertise<std_msgs::Float64>("/r_leg_kny_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_leg_lax",position_command_node.advertise<std_msgs::Float64>("/r_leg_lax_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_leg_lhy",position_command_node.advertise<std_msgs::Float64>("/r_leg_lhy_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_leg_mhx",position_command_node.advertise<std_msgs::Float64>("/r_leg_mhx_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_leg_uay",position_command_node.advertise<std_msgs::Float64>("/r_leg_uay_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_leg_uhz",position_command_node.advertise<std_msgs::Float64>("/r_leg_uhz_position_controller/command",10)));

      pub_map.insert(std::make_pair("neck_ay",position_command_node.advertise<std_msgs::Float64>("/neck_ay_position_controller/command",10)));
      pub_map.insert(std::make_pair("back_lbz",position_command_node.advertise<std_msgs::Float64>("/back_lbz_position_controller/command",10)));
      pub_map.insert(std::make_pair("back_mby",position_command_node.advertise<std_msgs::Float64>("/back_mby_position_controller/command",10)));
      pub_map.insert(std::make_pair("back_ubx",position_command_node.advertise<std_msgs::Float64>("/back_ubx_position_controller/command",10)));
  
      pub_map.insert(std::make_pair("r_f0_j0",position_command_node.advertise<std_msgs::Float64>("/r_f0_j0_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_f0_j1",position_command_node.advertise<std_msgs::Float64>("/r_f0_j1_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_f0_j2",position_command_node.advertise<std_msgs::Float64>("/r_f0_j2_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_f1_j0",position_command_node.advertise<std_msgs::Float64>("/r_f1_j0_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_f1_j1",position_command_node.advertise<std_msgs::Float64>("/r_f1_j1_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_f1_j2",position_command_node.advertise<std_msgs::Float64>("/r_f1_j2_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_f2_j0",position_command_node.advertise<std_msgs::Float64>("/r_f2_j0_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_f2_j1",position_command_node.advertise<std_msgs::Float64>("/r_f2_j1_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_f2_j2",position_command_node.advertise<std_msgs::Float64>("/r_f2_j2_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_f3_j0",position_command_node.advertise<std_msgs::Float64>("/r_f3_j0_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_f3_j1",position_command_node.advertise<std_msgs::Float64>("/r_f3_j1_position_controller/command",10)));
      pub_map.insert(std::make_pair("r_f3_j2",position_command_node.advertise<std_msgs::Float64>("/r_f3_j2_position_controller/command",10)));
       
      pub_map.insert(std::make_pair("l_f0_j0",position_command_node.advertise<std_msgs::Float64>("/l_f0_j0_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_f0_j1",position_command_node.advertise<std_msgs::Float64>("/l_f0_j1_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_f0_j2",position_command_node.advertise<std_msgs::Float64>("/l_f0_j2_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_f1_j0",position_command_node.advertise<std_msgs::Float64>("/l_f1_j0_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_f1_j1",position_command_node.advertise<std_msgs::Float64>("/l_f1_j1_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_f1_j2",position_command_node.advertise<std_msgs::Float64>("/l_f1_j2_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_f2_j0",position_command_node.advertise<std_msgs::Float64>("/l_f2_j0_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_f2_j1",position_command_node.advertise<std_msgs::Float64>("/l_f2_j1_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_f2_j2",position_command_node.advertise<std_msgs::Float64>("/l_f2_j2_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_f3_j0",position_command_node.advertise<std_msgs::Float64>("/l_f3_j0_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_f3_j1",position_command_node.advertise<std_msgs::Float64>("/l_f3_j1_position_controller/command",10)));
      pub_map.insert(std::make_pair("l_f3_j2",position_command_node.advertise<std_msgs::Float64>("/l_f3_j2_position_controller/command",10)));
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


