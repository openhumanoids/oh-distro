// file: robot_state_publisher.cpp
// This file links to tf_publisher.cpp which publishes a ROS style transform tree 
// upon receipt of a joint_state_t message.
// The tf_publisher code us based robot_state_publisher package in ROS, adapted to work within lcm+pods.
// The tf_publisher sents out a tf tree with frame id and child frame id information.

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <kdl/tree.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "robot_state_publisher/tf_publisher.hpp"
#include "kdl_parser/kdl_parser.hpp"

namespace robot_state_publisher {

 class RobotModel {
  public:
    lcm::LCM lcm;
    std::string urdf_xml_string; 
    std::vector<std::string> joint_names_; 
    //KDL::Tree tree;
  };


 class JointStatesHandler 
 {
    public:
        JointStatesHandler(const KDL::Tree& tree): tf_publisher_(tree) {}
        ~JointStatesHandler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const drc::joint_state_t* msg)
        {

		// call a routine that calculates the transforms given the joint_state_t* msg.
		std::map<std::string, double> joint_positions;
    		for (unsigned int i=0; i< msg->num_joints; i++)
      			joint_positions.insert(make_pair(msg->joint_name[i], msg->position[i]));  

 		// publishes ROS style tf tree on JOINT_TRANSFORMS channel.  	        
  		tf_publisher_.publishTransforms(joint_positions, msg->timestamp);
		tf_publisher_.publishFixedTransforms( msg->timestamp); // publish transforms of fixed joints
        }

  	//lcm::LCM lcm;
        tf_publisher::TfPublisher tf_publisher_;
 };

 void onMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_urdf_t* msg, RobotModel*  robot) {
   // Received robot urdf string. Store it internally and get all available joints.
    robot->urdf_xml_string = msg->urdf_xml_string;
    std::cout<<"Received urdf_xml_string, storing it internally as a param"<<std::endl;

   // Get a urdf Model from the xml string and get all the joint names.
  urdf::Model robot_model; 
  if (!robot_model.initString( msg->urdf_xml_string))
  {std::cerr << "ERROR: Could not generate robot model" << std::endl;}

  typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > joints_mapType;
  for( joints_mapType::const_iterator it = robot_model.joints_.begin(); it!=robot_model.joints_.end(); it++)
  { 
	  if(it->second->type!=6) // All joints that not of the type FIXED.
               robot->joint_names_.push_back(it->first);
  }

 } // end onMessage

}



int
main(int argc, char ** argv)
{

  // Subscribe to Robot_Model. Parse xml and create a KDL tree internally  
   robot_state_publisher::RobotModel* robot = new robot_state_publisher::RobotModel;

   lcm::Subscription* robot_model_subcription_;
   robot_model_subcription_ = robot->lcm.subscribeFunction("ROBOT_MODEL", robot_state_publisher::onMessage, robot);

   while(robot->lcm.handle()==-1);// wait for one message, wait until you get a success.

  // Unsubscribe.
   robot->lcm.unsubscribe(robot_model_subcription_); // Stop listening to ROBOT_MODEL.

   std::cout<< "Number of Joints: " << robot->joint_names_.size() <<std::endl;



  // Subscribe to JOINT_STATES.
  lcm::LCM lcm;
  if(!lcm.good())
        return 1;
  
  KDL::Tree tree;
  // Parse KDL tree and pass as an argument into JOINT_STATES handler object cosntructor.
  if (!kdl_parser::treeFromString(robot->urdf_xml_string,tree)){
    std::cerr << "ERROR: Failed to extract kdl tree from xml robot description"<<std::endl; 
    return -1;
  }

  robot_state_publisher::JointStatesHandler handlerObject(tree);
  lcm.subscribe("JOINT_STATES", &robot_state_publisher::JointStatesHandler::handleMessage, &handlerObject);
  while(0 == lcm.handle());

   // Handler Objecthas an lcm object that publishes to JOINT_TRANSFORMS. 
  // It  instantiates a ROS style Tf_Publisher Object (contains frame_id and child_frame id information along with transforms) .

  // Need a separate check_treeflsolverposfull_recursive.cpp that subscribes to a robot_model. 
  // Subscribes to JOINT_STATES and computes global transforms in global frame for each joint


    delete robot; 
    return 0;
}
