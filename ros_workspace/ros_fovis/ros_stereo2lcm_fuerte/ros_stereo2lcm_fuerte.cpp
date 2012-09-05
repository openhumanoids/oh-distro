#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& limage, const ImageConstPtr& rimage)
{
  // Solve all of perception here...
  std::cout << "here\n";
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;

  message_filters::Subscriber<Image>            left_sub(nh, "/wide_stereo/left/image_mono", 1);
  message_filters::Subscriber<Image>           right_sub(nh, "/wide_stereo/right/image_mono", 1);
  TimeSynchronizer<Image, Image> sync(left_sub, right_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
