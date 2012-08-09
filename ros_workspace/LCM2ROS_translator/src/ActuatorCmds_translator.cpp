#include <ros/ros.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <urdf/model.h>
#include <urdf_interface/joint.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <atlas_gazebo_msgs/RobotState.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <atlas_gazebo_msgs/ActuatorCmd.h>

double getTime_now()
{
	struct timeval tv;
	gettimeofday (&tv,NULL);
	return (int64_t) tv.tv_sec*1000000+tv.tv_usec;
}

class ActuatorCmdHandler{
	private:
		ros::Publisher actuator_cmd_pub;
		ros::NodeHandle actuator_cmd_node;
	public:
	ActuatorCmdHandler(ros::NodeHandle &node): actuator_cmd_node(node)	
	{
	 actuator_cmd_pub = actuator_cmd_node.advertise<atlas_gazebo_msgs::ActuatorCmd>("actuator_cmd",10);
	}
	~ActuatorCmdHandler() {}
	
	void actuator_cmd_Callback(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::actuator_cmd_t* msg)//what is rbuf and channel here?
	{
		atlas_gazebo_msgs::ActuatorCmd actuator_cmd_msg;
                long t = msg->timestamp*1000; // from usec to nsec
		actuator_cmd_msg.header.stamp.fromNSec(t);
		actuator_cmd_msg.robot_name = msg->robot_name;
		for(std::vector<int>::size_type i=0;i!=msg->joint_name.size();i++){//So varaibles in lcm message are all in vector type
			actuator_cmd_msg.joint_name.push_back(msg->joint_name.at(i));
			actuator_cmd_msg.joint_effort.push_back(msg->joint_effort.at(i));
			actuator_cmd_msg.duration.push_back(msg->duration.at(i));
		}
		if(ros::ok())
		{
			actuator_cmd_pub.publish(actuator_cmd_msg);
			//ros::spinOnce(); // required?
		}

	}
};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"actuator_cmd_publisher",ros::init_options::NoSigintHandler);
        ros::NodeHandle actuator_cmd_node;

	lcm::LCM listener;
	if(!listener.good())
             return 1;
	ActuatorCmdHandler handlerObject(actuator_cmd_node);
	
	//LCM subscription
	listener.subscribe("ACTUATOR_CMDS",&ActuatorCmdHandler::actuator_cmd_Callback,&handlerObject);

	while(0 == listener.handle());
	return 0;
}



// 	lcm::LCM talker;
// 	if(!talker.good())
// 		return 1;
// 	drc::actuator_cmd_t actuator_cmd;
// 	actuator_cmd.timestamp = 0;
// 	actuator_cmd.robot_name = "wheeled_atlas";
// 	actuator_cmd.num_joints = 2;
// 	actuator_cmd.joint_name.push_back("LShoulderRoll");
// 	actuator_cmd.joint_name.push_back("RShoulderRoll");
// 	actuator_cmd.joint_effort.push_back(12.0);
// 	actuator_cmd.joint_effort.push_back(-12.0);
// 	actuator_cmd.duration.push_back(0.1);
// 	actuator_cmd.duration.push_back(0.1);
// // 	lcm::LCM listener;
// 	if(!listener.good())
//              return 1;
// 	ActuatorCmdHandler handler;
// 	listener.subscribe("ACTUATOR_CMDS",&ActuatorCmdHandler::actuator_cmd_Callback,&handler);
// 	while(true)
// 	{
// 	        talker.publish("ACTUATOR_CMDS",&actuator_cmd); 
// 		listener.handle();
// 	}
// 	return 0;