#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include <algorithm>
#include <vector>
#include <sys/time.h>
#include <time.h>

using namespace std;
using namespace boost;

class Handler
{
private:
  boost::shared_ptr<lcm::LCM> _lcm;  
	drc::contact_state_t contacts; // cache for contact state, updated asynchronously.
	std::vector<int64_t> last_contacts_timestamps;
public:
   Handler(boost::shared_ptr<lcm::LCM> &lcm):_lcm(lcm) {
   
    if(!lcm->good())      {
        	cerr << "\nLCM Not Good: Handler in est_robot_state_publisher" << endl;	return;
        }
 
    //lcm->subscribe("MEAS_JOINT_ANGLES", &Handler::handleJointAnglesMsg, this);  //Subscribes to MEAS_JOINT_ANGLES 
    lcm->subscribe("TRUE_ROBOT_STATE", &Handler::handleRobotStateMsg, this); //

// Creating individual subscriptions for the foot sensors.
// CHANNELS: LFOOT_TOE_IN/OUT_CONTACT_STATE, LFOOT_HEEL_IN/OUT_CONTACT_STATE
// CHANNELS: RFOOT_TOE_IN/OUT_CONTACT_STATE, RFOOT_HEEL_IN/OUT_CONTACT_STATE
lcm->subscribe("LFOOT_TOE_IN_CONTACT_STATE", &Handler::update_LFootToeIn_ContactStateMsg, this); 
 lcm->subscribe("LFOOT_TOE_OUT_CONTACT_STATE", &Handler::update_LFootToeOut_ContactStateMsg, this); 
 lcm->subscribe("LFOOT_HEEL_IN_CONTACT_STATE", &Handler::update_LFootHeelIn_ContactStateMsg, this); 
 lcm->subscribe("LFOOT_HEEL_OUT_CONTACT_STATE", &Handler::update_LFootHeelOut_ContactStateMsg, this); 
 lcm->subscribe("RFOOT_TOE_IN_CONTACT_STATE", &Handler::update_RFootToeIn_ContactStateMsg, this); 
 lcm->subscribe("RFOOT_TOE_OUT_CONTACT_STATE", &Handler::update_RFootToeOut_ContactStateMsg, this); 
 lcm->subscribe("RFOOT_HEEL_IN_CONTACT_STATE", &Handler::update_RFootHeelIn_ContactStateMsg, this); 
 lcm->subscribe("RFOOT_HEEL_OUT_CONTACT_STATE", &Handler::update_RFootHeelOut_ContactStateMsg, this); 

   
   // initialize contact states
      contacts.num_contacts =8;
      contacts.id.push_back("LFootToeIn");
      contacts.id.push_back("LFootToeOut");
      contacts.id.push_back("LFootHeelIn");
      contacts.id.push_back("LFootHeelOut");
      contacts.id.push_back("RFootToeIn");
      contacts.id.push_back("RFootToeOut");
      contacts.id.push_back("RFootHeelIn");
      contacts.id.push_back("RFootHeelOut");
       int64_t t = getTime_now();
      for (unsigned int i=0; i< contacts.num_contacts; i++){
          contacts.inContact.push_back(0);
          drc::vector_3d_t f_zero;
          f_zero.x = 0;f_zero.y = 0;f_zero.z = 0;
          contacts.contactForce.push_back(f_zero);
          last_contacts_timestamps.push_back(t);
      }
   };
	~Handler() {};

 private:	
  
	void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const drc::robot_state_t * const_msg)
	{
	 //Collate latest contact_state_t messages into the robot_state msg.
	  drc::robot_state_t msg = *const_msg;
	  collateContactSensors(&msg);
		_lcm->publish("EST_ROBOT_STATE", &msg);
	}
//==================================================================================================
  void collateContactSensors(drc::robot_state_t *msg)
  {

    std::vector<std::string>::const_iterator found;
    //found = msg->contacts.id.find("LFootToeIn");
    found = std::find (msg->contacts.id.begin(), msg->contacts.id.end(), "LFootToeIn");
    if (found != msg->contacts.id.end()) {
       unsigned int index = found - msg->contacts.id.begin();
       //std::cout<< "LFootToeIn: " << index <<std::endl;
       msg->contacts.inContact[index] =  contacts.inContact[0];
       msg->contacts.contactForce[index] =contacts.contactForce[0];
        
    } else {   
       std::cout << "ERROR: no LFootToeIn in RobotState Msg" << std::endl;
    }
    
    found = std::find (msg->contacts.id.begin(), msg->contacts.id.end(), "LFootToeOut");
    if (found != msg->contacts.id.end()) {
        unsigned int index = found - msg->contacts.id.begin();

        msg->contacts.inContact[index] =  contacts.inContact[1];
        msg->contacts.contactForce[index] =contacts.contactForce[1];
        
    } else {   
       std::cout << "ERROR: no LFootToeOut in RobotState Msg" << std::endl;
    }
    
    found = std::find (msg->contacts.id.begin(), msg->contacts.id.end(), "LFootHeelIn");
    if (found != msg->contacts.id.end()) {
        unsigned int index = found - msg->contacts.id.begin();

        msg->contacts.inContact[index] =  contacts.inContact[2];
        msg->contacts.contactForce[index] =contacts.contactForce[2];
        
    } else {   
       std::cout << "ERROR: no LFootHeelIn in RobotState Msg" << std::endl;
    }
    
    found = std::find (msg->contacts.id.begin(), msg->contacts.id.end(), "LFootHeelOut");
    if (found != msg->contacts.id.end()) {
        unsigned int index = found - msg->contacts.id.begin();
        msg->contacts.inContact[index] =  contacts.inContact[3];
        msg->contacts.contactForce[index] =contacts.contactForce[3];
        
    } else {   
       std::cout << "ERROR: no LFootHeelOut in RobotState Msg" << std::endl;
    }
    
    found = std::find (msg->contacts.id.begin(), msg->contacts.id.end(), "RFootToeIn");
    if (found != msg->contacts.id.end()) {
      unsigned int index = found - msg->contacts.id.begin();
      msg->contacts.inContact[index] =  contacts.inContact[4];
      msg->contacts.contactForce[index] =contacts.contactForce[4];
        
    } else {   
       std::cout << "ERROR: no RFootToeIn in RobotState Msg" << std::endl;
    }
    
    found = std::find (msg->contacts.id.begin(), msg->contacts.id.end(), "RFootToeOut");
    if (found != msg->contacts.id.end()) {
        unsigned int index = found - msg->contacts.id.begin();
        msg->contacts.inContact[index] =  contacts.inContact[5];
        msg->contacts.contactForce[index] =contacts.contactForce[5];
        
    } else {   
       std::cout << "ERROR: no RFootToeOut in RobotState Msg" << std::endl;
    }
    
    found = std::find (msg->contacts.id.begin(), msg->contacts.id.end(), "RFootHeelIn");
    if (found != msg->contacts.id.end()) {
        unsigned int index = found - msg->contacts.id.begin();
        msg->contacts.inContact[index] =  contacts.inContact[6];
        msg->contacts.contactForce[index] =contacts.contactForce[6];
        
    } else {   
       std::cout << "ERROR: no RFootHeelIn in RobotState Msg" << std::endl;
    }
    
    found = std::find (msg->contacts.id.begin(), msg->contacts.id.end(), "RFootHeelOut");
    if (found != msg->contacts.id.end()) {
        unsigned int index = found - msg->contacts.id.begin();
        msg->contacts.inContact[index] =  contacts.inContact[7];
        msg->contacts.contactForce[index] = contacts.contactForce[7];
        
    } else {   
       std::cout << "ERROR: no RFootHeelOut in RobotState Msg" << std::endl;
    }     

  };
//==================================================================================================
  
  
	void update_LFootToeIn_ContactStateMsg(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const drc::contact_state_stamped_t * const_msg)
	{
	   if (const_msg->num_contacts > 1){
	      std::cout << "ERROR:contact_state_t  has "<< const_msg->num_contacts <<" contacts,  handleContactStateMsg_LFootToeIn expects a single contact" << std::endl;
	   }
   std::vector<std::string>::const_iterator found;
    found = std::find (contacts.id.begin(), contacts.id.end(), "LFootToeIn");    
    if (found != contacts.id.end()) {
         unsigned int index = found - contacts.id.begin();
        last_contacts_timestamps[index] = const_msg->utime;
        contacts.inContact[index] =   const_msg->inContact[0];
        contacts.contactForce[index] = const_msg->contactForce[0];
     } 
	};
//==================================================================================================
	
	void update_LFootToeOut_ContactStateMsg(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const drc::contact_state_stamped_t * const_msg)
	{
	   if (const_msg->num_contacts > 1){
	      std::cout << "ERROR:contact_state_t  has "<< const_msg->num_contacts <<" contacts,  handleContactStateMsg_LFootToeOut expects a single contact" << std::endl;
	   }
	   
     std::vector<std::string>::const_iterator found;
     found = std::find (contacts.id.begin(), contacts.id.end(), "LFootToeOut"); 
     if (found != contacts.id.end()) {
        unsigned int index = found - contacts.id.begin();
        last_contacts_timestamps[index] = const_msg->utime;
        contacts.inContact[index] =   const_msg->inContact[0];
        contacts.contactForce[index] = const_msg->contactForce[0];
     } 
	};
//==================================================================================================
	
	void update_LFootHeelIn_ContactStateMsg(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const drc::contact_state_stamped_t * const_msg)
	{
	   // const_msg->utime;
	   //const_msg_robotname;
	
	   if (const_msg->num_contacts > 1){
	      std::cout << "ERROR:contact_state_t  has "<< const_msg->num_contacts <<" contacts,  handleContactStateMsg_LFootHeelIn expects a single contact" << std::endl;
	   }
	   
     std::vector<std::string>::const_iterator found;
     found = std::find (contacts.id.begin(), contacts.id.end(), "LFootHeelIn"); 
     if (found != contacts.id.end()) {
        unsigned int index = found - contacts.id.begin();
        last_contacts_timestamps[index] = const_msg->utime;
        contacts.inContact[index] =   const_msg->inContact[0];
        contacts.contactForce[index] = const_msg->contactForce[0];
     } 
	};
//==================================================================================================
	
	void update_LFootHeelOut_ContactStateMsg(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const drc::contact_state_stamped_t * const_msg)
	{
	   if (const_msg->num_contacts > 1){
	      std::cout << "ERROR:contact_state_t  has "<< const_msg->num_contacts <<" contacts,  handleContactStateMsg_LFootHeelOut expects a single contact" << std::endl;
	   }
	   
     std::vector<std::string>::const_iterator found;
     found = std::find (contacts.id.begin(), contacts.id.end(), "LFootHeelOut"); 
     if (found != contacts.id.end()) {
        unsigned int index = found - contacts.id.begin();
        last_contacts_timestamps[index] = const_msg->utime;
        contacts.inContact[index] =   const_msg->inContact[0];
        contacts.contactForce[index] = const_msg->contactForce[0];
     } 
	};	
	
//==================================================================================================
	
	void update_RFootToeIn_ContactStateMsg(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const drc::contact_state_stamped_t * const_msg)
	{
	   if (const_msg->num_contacts > 1){
	      std::cout << "ERROR:contact_state_t  has "<< const_msg->num_contacts <<" contacts,  handleContactStateMsg_RFootToeIn expects a single contact" << std::endl;
	   }
	   
     std::vector<std::string>::const_iterator found;
     found = std::find (contacts.id.begin(), contacts.id.end(), "RFootToeIn"); 
     if (found != contacts.id.end()) {
        unsigned int index = found - contacts.id.begin();
        last_contacts_timestamps[index] = const_msg->utime;
        contacts.inContact[index] =   const_msg->inContact[0];
        contacts.contactForce[index] = const_msg->contactForce[0];
     } 
	};
	
//==================================================================================================
	
	void update_RFootToeOut_ContactStateMsg(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const drc::contact_state_stamped_t * const_msg)
	{
	   if (const_msg->num_contacts > 1){
	      std::cout << "ERROR:contact_state_t  has "<< const_msg->num_contacts <<" contacts,  handleContactStateMsg_RFootToeOut expects a single contact" << std::endl;
	   }
	   
     std::vector<std::string>::const_iterator found;
     found = std::find (contacts.id.begin(), contacts.id.end(), "RFootToeOut"); 
     if (found != contacts.id.end()) {
        unsigned int index = found - contacts.id.begin();
        last_contacts_timestamps[index] = const_msg->utime;
        contacts.inContact[index] =   const_msg->inContact[0];
        contacts.contactForce[index] = const_msg->contactForce[0];
     } 
	};
	
//==================================================================================================

	void update_RFootHeelIn_ContactStateMsg(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const drc::contact_state_stamped_t * const_msg)
	{
	   if (const_msg->num_contacts > 1){
	      std::cout << "ERROR:contact_state_t  has "<< const_msg->num_contacts <<" contacts,  handleContactStateMsg_RFootHeelIn expects a single contact" << std::endl;
	   }
	   
     std::vector<std::string>::const_iterator found;
     found = std::find (contacts.id.begin(), contacts.id.end(), "RFootHeelIn"); 
     if (found != contacts.id.end()) {
        unsigned int index = found - contacts.id.begin();
        last_contacts_timestamps[index] = const_msg->utime;
        contacts.inContact[index] =   const_msg->inContact[0];
        contacts.contactForce[index] = const_msg->contactForce[0];
     } 
	};
//==================================================================================================
	
	void update_RFootHeelOut_ContactStateMsg(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const drc::contact_state_stamped_t * const_msg)
	{
	   if (const_msg->num_contacts > 1){
	      std::cout << "ERROR:contact_state_t  has "<< const_msg->num_contacts <<" contacts,  handleContactStateMsg_RFootHeelOut expects a single contact" << std::endl;
	   }
	   
     std::vector<std::string>::const_iterator found;
     found = std::find (contacts.id.begin(), contacts.id.end(), "RFootHeelOut"); 
     if (found != contacts.id.end()) {
        unsigned int index = found - contacts.id.begin();
        last_contacts_timestamps[index] = const_msg->utime;
        contacts.inContact[index] =   const_msg->inContact[0];
        contacts.contactForce[index] = const_msg->contactForce[0];
     } 
	};
	
double getTime_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec; 
};
	
}; // end Class Definition
//==================================================================================================

int main (int argc, char ** argv)
{

 boost::shared_ptr<lcm::LCM> _lcm(new lcm::LCM);
 if(!_lcm->good())
		return 1;
		
  boost::shared_ptr<Handler> handlerObject(new Handler(_lcm));
 
 	while(0 == _lcm->handle());

	return 0;

}
//==================================================================================================



