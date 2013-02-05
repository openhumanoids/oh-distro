#include <multisense_ros/laser.h>
#include <multisense_ros/laser_joint.h>
#include <multisense_ros/led.h>
#include <multisense_ros/camera.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "multisense_driver");

  ros::NodeHandle nh_private_("~");

  std::string dest_ip;
  nh_private_.param<std::string>("dest_ip", dest_ip, "10.10.72.52");

  multisense_driver::MultisenseDriver d(dest_ip);
  multisense_ros::Laser l(&d);
  multisense_ros::LaserJoint lj(&d);
  multisense_ros::Led led(&d);
  multisense_ros::Camera camera(&d);

  ros::spin();
  return 0;
}
