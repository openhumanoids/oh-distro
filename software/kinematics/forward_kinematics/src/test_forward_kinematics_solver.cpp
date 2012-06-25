// file: test_forward_kinematics_solver.cpp
// This file links to treefksolverposfull_recursive.cpp thats provides a routine to solve forward kinematics 
// upon receipt of a joint_state_t message.
// for the whole kinematic tree. KDL lib functions only provide an interface to query global position between
// a specified root and a tip segment.


#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <kdl/tree.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "robot_state_publisher/treefksolverposfull_recursive.hpp"
#include "kdl_parser/kdl_parser.hpp"

namespace test_forward_kinematics_solver {

 class RobotModel {
  public:
    lcm::LCM lcm;
    std::string urdf_xml_string; 
    std::vector<std::string> joint_names_;

  };


 class JointStatesHandler 
 {
    public:
        JointStatesHandler(const KDL::Tree& tree): fksolver(tree) {}
        ~JointStatesHandler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const drc::joint_state_t* msg)
        {

		// call a routine that calculates the transforms the joint_state_t* msg.
		std::map<std::string, double> jointpos_in;
    		for (unsigned int i=0; i< msg->num_joints; i++)
      			jointpos_in.insert(make_pair(msg->joint_name[i], msg->position[i]));  

		 std::map<std::string, drc::transform_t > cartpos_out;
		  
		  // Calculate forward position kinematics
		  bool kinematics_status;
		  bool flatten_tree=true; // determines the absolute transforms with respect to robot origin. Otherwise returns relative transforms between joints. 
		
		  kinematics_status = fksolver.JntToCart(jointpos_in,cartpos_out,flatten_tree);
		  
		  if(kinematics_status>=0){
		      std::cout << "Success!" <<std::endl;
		  }else{
		      
		    std::cerr << "Error: could not calculate forward kinematics!" <<std::endl;
		  }
		  
		  for( std::map<std::string, drc::transform_t>::const_iterator it = cartpos_out.begin(); it!=cartpos_out.end(); it++)
		  { 
		    drc::joint_transform_t state;		    
		    state.timestamp = 0;
		    state.joint_name = it->first;
		    state.tf.translation = it->second.translation;
		    state.tf.rotation = it->second.rotation;
		    
		    //DO SOMETHING WITH THE TRANSFORMS HERE
		    //print transforms		    
		    std::cout << "timestamp  : " << state.timestamp << std::endl;
		    std::cout << "joint_name : " << state.joint_name << std::endl;	
		    std::cout << "translation  : " << std::endl;
		    std::cout << "\t .x  : " << state.tf.translation.x << std::endl;
		    std::cout << "\t .y  : " << state.tf.translation.y << std::endl;
		    std::cout << "\t .z  : " << state.tf.translation.z << std::endl;
		    std::cout << "quaternion" << std::endl;
		    std::cout << "\t .x  : " << state.tf.rotation.x << std::endl;
		    std::cout << "\t .y  : " << state.tf.rotation.y << std::endl;
		    std::cout << "\t .z  : " << state.tf.rotation.z << std::endl;
		    std::cout << "\t .w  : " << state.tf.rotation.w << std::endl;
		    
		    
		    
		  }
        } // end handleMessage

  	//lcm::LCM lcm;
	KDL::TreeFkSolverPosFull_recursive fksolver;
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

} // end namespace



int main(int argc, char ** argv)
{

  // Subscribe to Robot_Model. Parse xml and create a KDL tree internally  
   test_forward_kinematics_solver::RobotModel* robot = new test_forward_kinematics_solver::RobotModel;

   lcm::Subscription* robot_model_subcription_;
   robot_model_subcription_ = robot->lcm.subscribeFunction("ROBOT_MODEL", test_forward_kinematics_solver::onMessage, robot);

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

  // Subscribes to JOINT_STATES and computes global transforms in global frame for each joint
  test_forward_kinematics_solver::JointStatesHandler handlerObject(tree);
  lcm.subscribe("JOINT_STATES", &test_forward_kinematics_solver::JointStatesHandler::handleMessage, &handlerObject);
  while(0 == lcm.handle());


  delete robot; 
  return 0;
}
