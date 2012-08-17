// Minimal PR2 ROS-to-LCM translator for Stereo Images
//
// todo:
// support color and jpeg compression
//
// mfallon and hordurj

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <bot_core/bot_core.h>
//#include <bot_param/param_client.h>
//#include <bot_param/param_util.h>
#include <lcmtypes/bot_core.h>

int width =640; // hardcoded
int height =480; // hardcoded
uint8_t* s_data = new uint8_t [width*height*2]; // 2 grey scale images stacked

class App
{
public:
  App(const std::string & chan_in,
      const std::string & chan_out);
  ~App();
  void imageCb(const sensor_msgs::ImageConstPtr& l_image,
      const sensor_msgs::CameraInfoConstPtr& l_cam_info,
      const sensor_msgs::ImageConstPtr& r_image,
      const sensor_msgs::CameraInfoConstPtr& r_cam_info);

private:
  lcm_t* lcmref_ ;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;

  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;

  const std::string chan_in_,chan_out_;
};


App::App(const std::string & chan_in,
    const std::string & chan_out) :
    chan_in_(chan_in),
    chan_out_(chan_out),
    it_(nh_),
    sync_(10){
  ROS_INFO("Initializing");
  lcmref_ = lcm_create(NULL);

  std::string lim_string = chan_in_ + "/left/image_mono";
  std::string lin_string = chan_in_ + "/left/camera_info";
  std::string rim_string = chan_in_ + "/right/image_mono";
  std::string rin_string = chan_in_ + "/right/camera_info";

  l_image_sub_.subscribe(it_, ros::names::resolve( lim_string ), 3);
  l_info_sub_.subscribe(nh_, ros::names::resolve( lin_string ), 3);
  r_image_sub_.subscribe(it_, ros::names::resolve( rim_string ), 3);
  r_info_sub_.subscribe(nh_, ros::names::resolve( rin_string ), 3);

  sync_.connectInput(l_image_sub_, l_info_sub_, r_image_sub_, r_info_sub_);
  sync_.registerCallback( boost::bind(&App::imageCb, this, _1, _2, _3, _4) );
}

App::~App()
{
}

void App::imageCb(const sensor_msgs::ImageConstPtr& l_image,
    const sensor_msgs::CameraInfoConstPtr& l_cam_info,
    const sensor_msgs::ImageConstPtr& r_image,
    const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{
  ros::Time current_time = l_image->header.stamp;
  int64_t current_utime = (int64_t) floor(l_image->header.stamp.toSec()  * 1E6);

  namespace enc = sensor_msgs::image_encodings;

  cv_bridge::CvImageConstPtr left_ptr;
  cv_bridge::CvImageConstPtr right_ptr;

  try {
    left_ptr =  cv_bridge::toCvShare(l_image, enc::MONO8);
    right_ptr =  cv_bridge::toCvShare(r_image, enc::MONO8);
  }catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  bot_core_image_t left;
  left.utime =current_utime;
  left.width =l_image->width;
  left.height =2*l_image->height;
  left.row_stride =l_image->width;
  left.pixelformat =BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
  left.size =left.width*left.height;
  copy(l_image->data.begin(), l_image->data.end(), s_data);
  copy(r_image->data.begin(), r_image->data.end(), s_data + (l_image->width*l_image->height));
  left.data =s_data;
  left.nmetadata =0;
  left.metadata =NULL;
  bot_core_image_t_publish(lcmref_, chan_out_.c_str(), &left);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "App");

  //cv::imshow("Image", depth_ptr->image);
  //cv::waitKey(1);
  //cv::namedWindow("Image",1);

  std::cout << "ros to lcm stereo converter\n";
  std::cout << "Arguments: program_name inputROSchannel outputLCMchannel\n";
  std::cout << "ros-stereo2lcm wide_stereo WIDE_STEREO_IMAGE\n";

  std::cout << "    program: " << argv[ 0 ] << "\n";
  std::cout << "ROS channel: " << argv[ 1 ] << " [Input]\n";
  std::cout << "LCM channel: " << argv[ 2 ] << " [Output]\n";

  App vo(argv[ 1 ], argv[ 2 ]);
  ros::spin();
}
