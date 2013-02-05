#include <multisense_ros/tools.h>


namespace multisense_ros
{

ros::Time convertTime(const crl::TimeStamp& t)
{
  return ros::Time(t.getSeconds(), t.getMicroSeconds()*1000);
}

}
