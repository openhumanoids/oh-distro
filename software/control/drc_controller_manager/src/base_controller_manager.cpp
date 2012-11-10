
#include <iostream>
#include <algorithm>
#include <vector>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "reactive_navigation_2d/nav_controller.hpp"
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

namespace drc_control{

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

 class BaseControllerManager 
 {
    public:
        BaseControllerManager(std::string &robot_name,
			  boost::shared_ptr<lcm::LCM> &lcm,
			  urdf::Model &robot_model);
       ~BaseControllerManager();
      bool toggle;
    private:
       boost::shared_ptr<lcm::LCM> _lcm;
       boost::shared_ptr<nav_control::NavController> nav_controller;
       
       std::string _robot_name;
       urdf::Model _urdf_robot_model;


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

    
    if (this->nav_controller->isRunning()) {
      this->nav_controller->update(*msg,dt); 
    } 
    
  }


 }; //end Class Base controller manager


   //constructor
   BaseControllerManager::BaseControllerManager(std::string &robot_name,
					boost::shared_ptr<lcm::LCM> &lcm,
					urdf::Model &robot_model)
   : _robot_name(robot_name),_lcm(lcm), _urdf_robot_model(robot_model) 
   {
    //lcm ok?
     if(!lcm->good())
      {
	std::cerr << "\nLCM Not Good: BaseControllerManager constructor" << std::endl;
	return;
      }    

     // initialize member variables
      latest_robotstatemsg_timestamp = 0;

     //==================== SPAWN NAV CONTROLLER

	std::string channel ="NAV_GOAL";
	this->nav_controller = boost::shared_ptr<nav_control::NavController>(new nav_control::NavController(this->_lcm,channel,_robot_name,_urdf_robot_model));
	
	
  // create a robot_state subscription.
  this->_lcm->subscribe("EST_ROBOT_STATE", &drc_control::BaseControllerManager::handleRobotStateMsgAndUpdateControllers, this);
   } // end constructor
 
 
 
  BaseControllerManager::~BaseControllerManager(){ }// end destructor
  
  
   
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
  drc_control::BaseControllerManager BaseControllerManagerObject(robot->robot_name,lcm,robot->urdf_robot_model);

   while(0 == lcm->handle());  
   return 0;
}


