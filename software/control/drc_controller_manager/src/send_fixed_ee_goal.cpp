// file: send_message.c
//
// LCM example program.

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <sys/time.h>
#include <time.h>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"


namespace drc_control
{
 class FixedGoalPublisher {
  public:
    FixedGoalPublisher(boost::shared_ptr<lcm::LCM> &lcm,drc::ee_goal_t &goal,std::string &channel): _lcm(lcm),_goal(goal),_channel(channel)
    {

     _lcm->publish(_channel, &_goal); 
      
    }
    ~FixedGoalPublisher()
    {
      _goal.halt_ee_controller = true;
      _lcm->publish(_channel, &_goal); 
    }
 private:
   boost::shared_ptr<lcm::LCM> _lcm;
   drc::ee_goal_t _goal;
   std::string _channel;
    
  };
  
}

double getTime_now()
{
	struct timeval tv;
	gettimeofday (&tv,NULL);
	return (int64_t) tv.tv_sec*1000000+tv.tv_usec;
};

int
main(int argc, char ** argv)
{
   boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM); 
   if(!lcm->good())
   {
      std::cerr << "\nLCM Not Good" << std::endl;
      return -1;
   }   

    drc::ee_goal_t my_goal;
 
    my_goal.utime = getTime_now();
    my_goal.robot_name = "wheeled_atlas";
    my_goal.ee_name = "RWristRoll_link";
    my_goal.root_name = "base";

    

    int goalid = 0;
    std::cout << argc << std::endl;
    if(argc>1){
     std::cout <<  "selecting  goal no: " << argv[1] << std::endl;
       goalid = atoi(argv[1]);
    }
    
    if(goalid ==2){
	my_goal.ee_goal_pos.translation.x = 0.1335;
	my_goal.ee_goal_pos.translation.y =  -0.241013;
	my_goal.ee_goal_pos.translation.z = 1.17883;

	my_goal.ee_goal_pos.rotation.x = -0.821902;
	my_goal.ee_goal_pos.rotation.y = 0.0375802;
	my_goal.ee_goal_pos.rotation.z = 0.396713;
	my_goal.ee_goal_pos.rotation.w = 0.407044;
    }
    else if (goalid ==1){
	
    my_goal.ee_goal_pos.translation.x = 0.151708;
    my_goal.ee_goal_pos.translation.y = -0.241059;
    my_goal.ee_goal_pos.translation.z = 0.353141;

    my_goal.ee_goal_pos.rotation.x = 0;
    my_goal.ee_goal_pos.rotation.y = -0.258691;
    my_goal.ee_goal_pos.rotation.z = 0;
    my_goal.ee_goal_pos.rotation.w = 0.96596;
    }
    else {
	
	my_goal.ee_goal_pos.translation.x = 0;
	my_goal.ee_goal_pos.translation.y = -0.241059;
	my_goal.ee_goal_pos.translation.z = 0.312513;

	my_goal.ee_goal_pos.rotation.x = 0;
	my_goal.ee_goal_pos.rotation.y = 0;
	my_goal.ee_goal_pos.rotation.z = 0;
	my_goal.ee_goal_pos.rotation.w = 1;
    }

//     
    my_goal.ee_goal_twist.linear_velocity.x = 0.0;
    my_goal.ee_goal_twist.linear_velocity.y = 0.0;
    my_goal.ee_goal_twist.linear_velocity.z = 0.0;
    my_goal.ee_goal_twist.angular_velocity.x = 0.0;
    my_goal.ee_goal_twist.angular_velocity.y = 0.0;
    my_goal.ee_goal_twist.angular_velocity.z = 0.0;
    my_goal.num_chain_joints  = 6;
    
    // No specified posture bias
    my_goal.use_posture_bias  = false;
    my_goal.joint_posture_bias.resize(my_goal.num_chain_joints);
    my_goal.chain_joint_names.resize(my_goal.num_chain_joints);
    for(int i = 0; i < my_goal.num_chain_joints; i++){
       my_goal.joint_posture_bias[i]=0;
       my_goal.chain_joint_names[i]= "dummy_joint_names";
       // my_goal.joint_posture_bias.push_back(0);
	// std::string dummy = "dummy_joint_names";
	//my_goal.chain_joint_names.push_back(dummy);
    }
    
    my_goal.halt_ee_controller = false;
    std::string channel = "RWRISTROLL_LINK_GOAL";
    drc_control::FixedGoalPublisher goalPublisher(lcm,my_goal,channel);
    while(true){
      std::cout << "Press any key to destruct fixed goal publisher and exit: " << std::endl;
      std::string key;
      std::cin >> key;
      std::cout << "you pressed: " << key << std::endl;
      break;
    }; 

    return 0;
}
/* struct ee_goal_t
{
	int64_t utime;
	string robot_name;
	string ee_name;
	string root_name;
	position_3d_t ee_goal_pos;
	twist_t ee_goal_twist;
	int32_t num_chain_joints;
	boolean use_posture_bias;
	double joint_posture_bias [num_chain_joints];
	string chain_joint_names [num_chain_joints];
        boolean halt_ee_controller;
}


link_name : LWristYaw_link
translation  : 
	 .x  : 0.151708
	 .y  : 0.241059
	 .z  : 0.353141
quaternion
	 .x  : 0
	 .y  : -0.258691
	 .z  : 0
	 .w  : 0.96596*/
