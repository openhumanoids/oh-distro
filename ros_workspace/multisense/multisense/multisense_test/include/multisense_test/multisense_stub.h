#ifndef _MULTISENSOR_TEST_MULTISENSOR_STUB_H_
#define _MULTISENSOR_TEST_MULTISENSOR_STUB_H_

#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include <LibSensorPodCommunications/SensorPodCommunications.h>

namespace multisense_test
{

// Emulates the actual multisensor hardware
class MultisenseStub
{
public:
  MultisenseStub(int dest_addr, int dest_port, int incoming_port);

protected:
  void rosCb(const boost::shared_ptr<const std_msgs::UInt8>& msg);

private:
  SensorPodCommunications comm_;

  const int dest_port_;
  const int dest_addr_;   //!< Destination IP after being processed by inet_addr()

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

}



#endif // _MULTISENSOR_TEST_MULTISENSOR_STUB_H_
