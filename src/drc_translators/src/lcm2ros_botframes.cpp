/*
This node publishes the camera position transform as a ROS transform 
*/
#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>
#include <tf/transform_broadcaster.h>

using namespace std;

class LCM2ROS{
  public:
    LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_);
    ~LCM2ROS() {}

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;
    //ros::Publisher rgbdCamera_transform_pub_;

    tf::TransformBroadcaster br;

    void rgbdCameraTransformHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::rigid_transform_t* msg);
};


LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_): lcm_(lcm_),nh_(nh_)
{
  lcm_->subscribe("BODY_TO_LWR_ARM_7_LINK",&LCM2ROS::rgbdCameraTransformHandler, this);
  //rgbdCamera_transform_pub_ = nh_.advertise<ipab_msgs::PlannerRequest>("/exotica/planner_request",10);
}

void LCM2ROS::rgbdCameraTransformHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::rigid_transform_t* msg) {
  /*tf::Transform camera_offset_transform;
  camera_offset_transform.setOrigin( tf::Vector3(0.12, -0.03,  0.06) );
  tf::Quaternion q;// = tf::Quaternion(0.01469347,  0.02815516,  0.724825, 0.68820063);
  q.setRPY(0, -1.57, 3.14);
  std::cout << "Q " << q.normalize().getAxis().getX() << " " << q.normalize().getAxis().getY() << " " << q.normalize().getAxis().getZ() << " " << q.normalize().getW() << std::endl;
  camera_offset_transform.setRotation(q);*/

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->trans[0], msg->trans[1], msg->trans[2]) );
  transform.setRotation( tf::Quaternion(msg->quat[1], msg->quat[2], msg->quat[3], msg->quat[0]) );
  // this aint perfect - need to tune
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "lwr_arm_7_link"));
}

int main(int argc,char** argv) {
  ros::init(argc,argv,"lcm2ros_botframes",ros::init_options::NoSigintHandler);
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  ros::NodeHandle nh;

  LCM2ROS handlerObject(lcm, nh);
  cout << "\nlcm2ros_botframes translator ready\n";
  ROS_ERROR("LCM2ROS Botframes Translator Ready");

  while(0 == lcm->handle());
  return 0;
}
