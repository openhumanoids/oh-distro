#include <ros/ros.h>
#include <cstdlib>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>
#include <map>
#include <string>

/**
	A simple translator that listens for a robot state message on the SET_ROBOT_CONFIG channel and sets the robot's pose, joint positions,
	and PID controller setpoints accordingly. If pausePhysics is set to true, the gazebo simulation will be paused after the configuration 
	is reset.
**/

class ConfigurationCommandHandler{
	private:
	
		std::map<std::string, ros::Publisher> setpoint_map;
		ros::Publisher pose_pub;
		ros::Publisher joint_pub;
		ros::NodeHandle config_command_node;
		
		ros::ServiceClient client;
		bool pausePhysics;
	
		
	public:
		ConfigurationCommandHandler(ros::NodeHandle &node): config_command_node(node) {
	 		
	 		pose_pub = config_command_node.advertise<geometry_msgs::Pose>("/atlas/set_pose",10);
	 		joint_pub = config_command_node.advertise<sensor_msgs::JointState>("/atlas/configuration",10);
	 		
			setpoint_map.insert(std::make_pair("l_arm_elx",config_command_node.advertise<std_msgs::Float64>("/l_arm_elx_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("l_arm_ely",config_command_node.advertise<std_msgs::Float64>("/l_arm_ely_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("l_arm_mwx",config_command_node.advertise<std_msgs::Float64>("/l_arm_mwx_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("l_arm_shx",config_command_node.advertise<std_msgs::Float64>("/l_arm_shx_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("l_arm_usy",config_command_node.advertise<std_msgs::Float64>("/l_arm_usy_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("l_arm_uwy",config_command_node.advertise<std_msgs::Float64>("/l_arm_uwy_position_controller/command",10)));

			setpoint_map.insert(std::make_pair("r_arm_elx",config_command_node.advertise<std_msgs::Float64>("/r_arm_elx_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("r_arm_ely",config_command_node.advertise<std_msgs::Float64>("/r_arm_ely_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("r_arm_mwx",config_command_node.advertise<std_msgs::Float64>("/r_arm_mwx_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("r_arm_shx",config_command_node.advertise<std_msgs::Float64>("/r_arm_shx_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("r_arm_usy",config_command_node.advertise<std_msgs::Float64>("/r_arm_usy_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("r_arm_uwy",config_command_node.advertise<std_msgs::Float64>("/r_arm_uwy_position_controller/command",10)));

			setpoint_map.insert(std::make_pair("l_leg_kny",config_command_node.advertise<std_msgs::Float64>("/l_leg_kny_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("l_leg_lax",config_command_node.advertise<std_msgs::Float64>("/l_leg_lax_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("l_leg_lhy",config_command_node.advertise<std_msgs::Float64>("/l_leg_lhy_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("l_leg_mhx",config_command_node.advertise<std_msgs::Float64>("/l_leg_mhx_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("l_leg_uay",config_command_node.advertise<std_msgs::Float64>("/l_leg_uay_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("l_leg_uhz",config_command_node.advertise<std_msgs::Float64>("/l_leg_uhz_position_controller/command",10)));

			setpoint_map.insert(std::make_pair("r_leg_kny",config_command_node.advertise<std_msgs::Float64>("/r_leg_kny_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("r_leg_lax",config_command_node.advertise<std_msgs::Float64>("/r_leg_lax_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("r_leg_lhy",config_command_node.advertise<std_msgs::Float64>("/r_leg_lhy_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("r_leg_mhx",config_command_node.advertise<std_msgs::Float64>("/r_leg_mhx_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("r_leg_uay",config_command_node.advertise<std_msgs::Float64>("/r_leg_uay_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("r_leg_uhz",config_command_node.advertise<std_msgs::Float64>("/r_leg_uhz_position_controller/command",10)));

			setpoint_map.insert(std::make_pair("neck_ay",config_command_node.advertise<std_msgs::Float64>("/neck_ay_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("back_lbz",config_command_node.advertise<std_msgs::Float64>("/back_lbz_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("back_mby",config_command_node.advertise<std_msgs::Float64>("/back_mby_position_controller/command",10)));
			setpoint_map.insert(std::make_pair("back_ubx",config_command_node.advertise<std_msgs::Float64>("/back_ubx_position_controller/command",10)));
	
			pausePhysics = false;
			client = config_command_node.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
		}
		~ConfigurationCommandHandler() {}
	
		void configuration_command_callback(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::robot_state_t* msg)
		{
			// set robot pose		
			geometry_msgs::Pose pose_msg;
			pose_msg.position = geometry_msgs::Point();
			pose_msg.position.x = msg->origin_position.translation.x;
			pose_msg.position.y = msg->origin_position.translation.y;
			pose_msg.position.z = msg->origin_position.translation.z;
			
			pose_msg.orientation = geometry_msgs::Quaternion();
			pose_msg.orientation.x = msg->origin_position.rotation.x;
			pose_msg.orientation.y = msg->origin_position.rotation.y;
			pose_msg.orientation.z = msg->origin_position.rotation.z;
			pose_msg.orientation.w = msg->origin_position.rotation.w;
			
			// set robot joint config
			sensor_msgs::JointState joint_msg;
			for (int i=0; i<msg->num_joints; i++) {
			    joint_msg.name.push_back(msg->joint_name[i]);
			    joint_msg.position.push_back(msg->joint_position[i]);
			}
			
			if(ros::ok()) {
				joint_pub.publish(joint_msg);			
				pose_pub.publish(pose_msg);			
			}			
			
			// set PID goals
			std_msgs::Float64 position_command_msg;
			for (int i=0; i<msg->num_joints; i++) {
			    // should we instead publish all joints at the same time?
			    position_command_msg.data = msg->joint_position[i];
				if(ros::ok()) {
					setpoint_map[msg->joint_name[i]].publish(position_command_msg);
				}
			}
			
			// pause gazebo
			if(pausePhysics && ros::ok()) {
		  		std_srvs::Empty srv;
		  		if (!client.call(srv)) {
					ROS_ERROR("Failed to pause gazebo.");
		  		}	
			}
		}
};	

int main(int argc,char** argv) {
	ros::init(argc,argv,"config_command_publisher",ros::init_options::NoSigintHandler);
    ros::NodeHandle config_command_node;

	lcm::LCM listener;
	if(!listener.good())
		return 1;
	ConfigurationCommandHandler handlerObject(config_command_node);
	
	//LCM subscription
	listener.subscribe("SET_ROBOT_CONFIG",&ConfigurationCommandHandler::configuration_command_callback,&handlerObject);

	while(0 == listener.handle());
	return 0;
}


