// This file is not robot generic, it is customized for a given robot. 


#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <kdl/tree.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "kdl_parser/kdl_parser.hpp"
#include "pd_chain_control/chain_controller.hpp"
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

namespace drc_control{
#define NUM_OF_ARM_JOINTS 6

 class RobotModel {
  public:
    std::string robot_name;
    std::string urdf_xml_string; 
    urdf::Model urdf_robot_model;

  };

 void onUrdfMessage(const lcm::ReceiveBuffer* rbuf, 
		    const std::string& channel, 
		    const  drc::robot_urdf_t* msg, 
		    boost::shared_ptr<drc_control::RobotModel> robot) 
 {
   // Received robot urdf string. Store it internally and get all available joints.
   robot->robot_name      = msg->robot_name;
   robot->urdf_xml_string = msg->urdf_xml_string;
   std::cout<<"Received urdf_xml_string of robot ["<<msg->robot_name <<"], storing it internally as a param"<<std::endl;

   // Get a urdf Model from the xml string and get all the joint names.
  if (!robot->urdf_robot_model.initString(msg->urdf_xml_string))
  {std::cerr << "ERROR: Could not generate robot model" << std::endl;}
  
 } // end onUrdfMessage

 class ControllerManager 
 {
    public:
        ControllerManager(std::string &robot_name,
			  boost::shared_ptr<lcm::LCM> &lcm,
			  urdf::Model &robot_model);
       ~ControllerManager() {}

    private:
       boost::shared_ptr<lcm::LCM> _lcm;
       boost::shared_ptr<pd_chain_control::ChainController<NUM_OF_ARM_JOINTS> > left_arm_controller;
       boost::shared_ptr<pd_chain_control::ChainController<NUM_OF_ARM_JOINTS> > right_arm_controller;

       std::string _robot_name;
       urdf::Model _urdf_robot_model;
       KDL::Tree _tree;
       std::vector<std::string> joint_names_; 
       std::map<std::string, double> jointpos_in,jointvel_in;
      int64_t latest_robotstatemsg_timestamp;

 // message callbacks 

// On receiving robot state, update controllers
   void handleRobotStateMsgAndUpdateControllers(const lcm::ReceiveBuffer* rbuf,
			 const std::string& chan, 
			 const drc::robot_state_t* msg)						 
  {
     int64_t  temp = (msg->utime-latest_robotstatemsg_timestamp); //from usec to sec
     double dt = (0.000001*temp); // rate at which robot state is arriving.

  // std::cout << "robot state msg received :" << (double)(0.001*temp)<< "msec" << std::endl;
   
     latest_robotstatemsg_timestamp = msg->utime;

    // call a routine that calculates the transforms the joint_state_t* msg.
     /* std::map<std::string, double> jointpos_in,jointvel_in;
  
    for (uint i=0; i< (uint) msg->num_joints; i++){ //cast to uint to suppress compiler warning
      jointpos_in.insert(make_pair(msg->joint_name[i], msg->joint_position[i]));
      jointvel_in.insert(make_pair(msg->joint_name[i], msg->joint_velocity[i]));
    }*/  

      // update jointpos_in and jointvel_in
     for (uint i=0; i< (uint) msg->num_joints; i++){
      jointpos_in[msg->joint_name[i]] = msg->joint_position[i]; 
      jointvel_in[msg->joint_name[i]] = msg->joint_velocity[i]; 
     }
    // Only update jointpos_in and jointvel_in;
    
 //call controller update routine.
  if(this->left_arm_controller->isRunning()) 
    this->left_arm_controller->update(jointpos_in, jointvel_in,dt); 
  } 

 }; //end Class control manager


   //constructor
   ControllerManager::ControllerManager(std::string &robot_name,
					boost::shared_ptr<lcm::LCM> &lcm,
					urdf::Model &robot_model)
   : _robot_name(robot_name),_lcm(lcm), _urdf_robot_model(robot_model) 
   {
    //lcm ok?
     if(!lcm->good())
      {
	std::cerr << "\nLCM Not Good: ControllerManager constructor" << std::endl;
	return;
      }     
     
     
     // initialize member variables
      latest_robotstatemsg_timestamp = 0;
 
    typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > joints_mapType;
    for( joints_mapType::const_iterator it = _urdf_robot_model.joints_.begin(); it!=_urdf_robot_model.joints_.end(); it++)
    { 
	  if(it->second->type!=6) // All joints that not of the type FIXED.
               this->joint_names_.push_back(it->first);
    }
    for (uint i=0; i< (uint) joint_names_.size(); i++){ //cast to uint to suppress compiler warning
      jointpos_in.insert(make_pair(joint_names_[i], 0));
      jointvel_in.insert(make_pair(joint_names_[i], 0));
    }
   
      // Parse KDL tree
	if (!kdl_parser::treeFromUrdfModel(_urdf_robot_model,_tree)){
          std::cerr << "ERROR: Failed to extract kdl tree from xml robot description"<<std::endl; 
          return;
	}    
	

     //==================== SPAWN CONTROLLERS
	std::string tip_link_name,root_link_name; // get it from command message.

	tip_link_name = "LWristRoll_link";
	if((robot_name == "wheeled_atlas")||(robot_name == "hovering_atlas"))
	{
	 root_link_name = "base";
	}
	else if (robot_name =="atlas") {
	 root_link_name = "BackRoll_link";
	}
	else {
	  std::cerr << "ERROR: Unknown Robot "<< std::endl;
	  return;
	}

	KDL::Chain kdl_chain;
	if (!_tree.getChain(root_link_name,tip_link_name, kdl_chain)){
	  std::cerr << "ERROR: Failed to extract kdl tree from xml robot description"<<std::endl; 
	  return;
	}
	//std::cout << " Joints: " << kdl_chain.getNrOfJoints() << " Links: " << kdl_chain.getNrOfSegments() <<std::endl;

        int NrOfJoints = (int) kdl_chain.getNrOfJoints();
	// Create controller objects
	if(NrOfJoints!=NUM_OF_ARM_JOINTS){
          std::cerr <<" NUM_OF_ARM_JOINTS does not match the the joints number is specified kdl_chain" <<  std::endl;
          return;
	}
	// TODO: Automatically parse channel name
	std::string channel = "LWRISTROLL_LINK_GOAL";// end effector name in CAPS followed by _GOAL
	//pd_chain_control::ChainController<NUM_OF_ARM_JOINTS> left_arm_controller(lcm,channel,kdl_chain);

	
        this->left_arm_controller = boost::shared_ptr<pd_chain_control::ChainController<NUM_OF_ARM_JOINTS> >(new pd_chain_control::ChainController<NUM_OF_ARM_JOINTS>(this->_lcm,channel,kdl_chain,_robot_name,_urdf_robot_model));
	
	
	tip_link_name = "RWristRoll_link";
	if (!_tree.getChain(root_link_name,tip_link_name, kdl_chain)){
	  std::cerr << "ERROR: Failed to extract kdl tree from xml robot description"<<std::endl; 
	  return;
	}
	 NrOfJoints = (int) kdl_chain.getNrOfJoints();
	 if(NrOfJoints!=NUM_OF_ARM_JOINTS){
          std::cerr <<" NUM_OF_ARM_JOINTS does not match the the joints number is specified kdl_chain" <<  std::endl;
          return;
	}
	channel = "RWRISTROLL_LINK_GOAL";
        this->right_arm_controller = boost::shared_ptr<pd_chain_control::ChainController<NUM_OF_ARM_JOINTS> >(new pd_chain_control::ChainController<NUM_OF_ARM_JOINTS>(this->_lcm,channel,kdl_chain,_robot_name,_urdf_robot_model));

  // create a robot_state subscription.
  this->_lcm->subscribe("EST_ROBOT_STATE", &drc_control::ControllerManager::handleRobotStateMsgAndUpdateControllers, this);
   } // end constructor

   
}//end namespace


// =============================================================
//		            MAIN LOOP
// =============================================================
int main(int argc, char ** argv)
{
 
   boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM); 
   if(!lcm->good())
   {
      std::cerr << "\nLCM Not Good" << std::endl;
      return -1;
   }   
   
   boost::shared_ptr<drc_control::RobotModel> robot(new drc_control::RobotModel);

   lcm::Subscription* robot_model_subcription_;
   robot_model_subcription_ =lcm->subscribeFunction("ROBOT_MODEL", drc_control::onUrdfMessage, robot);

   while(lcm->handle()==-1);// wait for one message, wait until you get a success.

  // Unsubscribe. Stop listening to ROBOT_MODEL.
   lcm->unsubscribe(robot_model_subcription_); 

  // CREATE A CONTROLLER MANAGER OBJECT AND PASS IT URDF MODEL AND lcm pointer.
  drc_control::ControllerManager controllerManagerObject(robot->robot_name,lcm,robot->urdf_robot_model);

   while(0 == lcm->handle());  
   return 0;
}


