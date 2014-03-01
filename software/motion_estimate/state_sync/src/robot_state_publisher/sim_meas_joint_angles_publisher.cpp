// file: sim_meas_joint_angles_publisher.cpp
//
// Publishes fake meas_joint_angles_t msgs on MEAS_JOINT_ANGLES  lcm channel.
// Subscribes to  TRUE_robot_state msgs published from gazebo and republishes
//  the joint angles with added noise. 
//  Use this process instead of meas_joint_angles_publisher when running with gazebo.


#include <iostream>
#include <stdint.h> 
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#include <urdf/model.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/joint_angles_t.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"

namespace sim_meas_joint_angles_publisher {


class TrueRobotStateHandler 
{
    public:
        ~TrueRobotStateHandler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const drc::robot_state_t* msg)
        {

               drc::joint_angles_t message;

		message.utime =msg->utime; 
		message.num_joints = msg->num_joints;		     	 

		for (std::vector<int>::size_type i = 0; i !=  msg -> joint_name.size(); i++)  {
	              double noise = 0; // replace with sampleGaussian(0,variance)
		      message.joint_name.push_back(msg->joint_name[i]);
		      message.joint_position.push_back(msg->joint_position[i]+noise);
		}

 		if(lcm.good())
 		lcm.publish("MEAS_JOINT_ANGLES", &message);

        }
    private:
         lcm::LCM lcm;
}; // end TrueRobotStateHandler


// Utility for adding noise
double sampleGaussian(double mu,double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  // see wikipedia
  double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
  // we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
} // end sampleGaussian

}//end namespace


int main(int argc, char ** argv)
{

    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    sim_meas_joint_angles_publisher::TrueRobotStateHandler handlerObject;
    lcm.subscribe("TRUE_ROBOT_STATE", &sim_meas_joint_angles_publisher::TrueRobotStateHandler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

   return 0;
 
}


