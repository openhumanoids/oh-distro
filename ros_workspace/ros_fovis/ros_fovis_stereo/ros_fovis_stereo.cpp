// Minimal stereo odometry using FoVis in ROS
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
#include <fovis/fovis.hpp>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>

#include <lcmtypes/bot_core.h>

class ros_fovis_stereo
{
public:
  ros_fovis_stereo(const std::string & camera_name,
      const std::string & config_name);
  ~ros_fovis_stereo();
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

  image_geometry::StereoCameraModel cam_model_;
  bool calib_initialized_;

  const std::string camera_name_;

  ros::Publisher odom_pub_;
  tf::TransformBroadcaster odom_broadcaster_;

  fovis::StereoCalibration* scal_;

  void set_vo_option_int(fovis::VisualOdometryOptions & vo_opts,
      const std::string & option) const;
  void set_vo_option_double(fovis::VisualOdometryOptions & vo_opts,
      const std::string & option) const;
  void set_vo_option_boolean(fovis::VisualOdometryOptions & vo_opts,
      const std::string & option) const;

  Eigen::Isometry3d pose_;

  BotParam* bot_param_;
  fovis::VisualOdometry* odom_;
  fovis::StereoDepth* depth_producer_;
  fovis::StereoCalibration* load_calibration(const std::string & config_name);
};


void ros_fovis_stereo::set_vo_option_int(fovis::VisualOdometryOptions & vo_opts,
    const std::string & option) const
{
  int ival;
  std::string param = std::string("fovis.") + option;
  if (bot_param_get_int(bot_param_, param.c_str(), &ival) == 0)
    vo_opts[option] = boost::lexical_cast<std::string>(ival);
}

void ros_fovis_stereo::set_vo_option_double(fovis::VisualOdometryOptions & vo_opts,
    const std::string & option) const
{
  double dval;
  std::string param = std::string("fovis.") + option;
  if (bot_param_get_double(bot_param_, param.c_str(), &dval) == 0)
    vo_opts[option] = boost::lexical_cast<std::string>(dval);
}

void ros_fovis_stereo::set_vo_option_boolean(fovis::VisualOdometryOptions & vo_opts,
    const std::string & option) const
{
  int bval;
  std::string param = std::string("fovis.") + option;
  if (bot_param_get_boolean(bot_param_, param.c_str(), &bval) == 0)
    vo_opts[option] = boost::lexical_cast<std::string>((bval?"true":"false"));
}



ros_fovis_stereo::ros_fovis_stereo(const std::string & camera_name,
    const std::string & config_name) :
    camera_name_(camera_name),
    it_(nh_),
    sync_(10),
    scal_(load_calibration(config_name)),
    calib_initialized_(false),
    pose_(Eigen::Isometry3d::Identity()){
  ROS_INFO("Initialize stereo camera");
  lcmref_ = lcm_create(NULL);

  fovis::VisualOdometryOptions vo_opts = fovis::VisualOdometry::getDefaultOptions();

  set_vo_option_int(vo_opts, "feature-window-size");
  set_vo_option_int(vo_opts, "max-pyramid-level");
  set_vo_option_int(vo_opts, "min-pyramid-level");
  set_vo_option_int(vo_opts, "target-pixels-per-feature");
  set_vo_option_int(vo_opts, "fast-threshold");
  set_vo_option_double(vo_opts, "fast-threshold-adaptive-gain");
  set_vo_option_boolean(vo_opts, "use-adaptive-threshold");
  set_vo_option_boolean(vo_opts, "use-homography-initialization");
  set_vo_option_int(vo_opts, "ref-frame-change-threshold");

  // OdometryFrame
  set_vo_option_boolean(vo_opts, "use-bucketing");
  set_vo_option_int(vo_opts, "bucket-width");
  set_vo_option_int(vo_opts, "bucket-height");
  set_vo_option_int(vo_opts, "max-keypoints-per-bucket");
  set_vo_option_boolean(vo_opts, "use-image-normalization");

  // MotionEstimator
  set_vo_option_double(vo_opts, "inlier-max-reprojection-error");
  set_vo_option_double(vo_opts, "clique-inlier-threshold");
  set_vo_option_int(vo_opts, "min-features-for-estimate");
  set_vo_option_double(vo_opts, "max-mean-reprojection-error");
  set_vo_option_boolean(vo_opts, "use-subpixel-refinement");
  set_vo_option_int(vo_opts, "feature-search-window");
  set_vo_option_boolean(vo_opts, "update-target-features-with-refined");

  // StereoDepth
  set_vo_option_boolean(vo_opts, "stereo-require-mutual-match");
  set_vo_option_double(vo_opts, "stereo-max-dist-epipolar-line");
  set_vo_option_double(vo_opts, "stereo-max-refinement-displacement");
  set_vo_option_int(vo_opts, "stereo-max-disparity");

  l_image_sub_.subscribe(it_, ros::names::resolve("wide_stereo/left/image_rect"), 3);
  l_info_sub_.subscribe(nh_, ros::names::resolve("wide_stereo/left/camera_info"), 3);
  r_image_sub_.subscribe(it_, ros::names::resolve("wide_stereo/right/image_rect"), 3);
  r_info_sub_.subscribe(nh_, ros::names::resolve("wide_stereo/right/camera_info"), 3);

  sync_.connectInput(r_image_sub_, r_info_sub_, l_image_sub_, l_info_sub_);
  sync_.registerCallback( boost::bind(&ros_fovis_stereo::imageCb, this, _1, _2, _3, _4) );

  odom_ = new fovis::VisualOdometry(scal_->getLeftRectification(), vo_opts);
  depth_producer_ = new fovis::StereoDepth(scal_, vo_opts);

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("vo", 50);    
}

ros_fovis_stereo::~ros_fovis_stereo()
{
  delete odom_;
}

void ros_fovis_stereo::imageCb(const sensor_msgs::ImageConstPtr& l_image,
    const sensor_msgs::CameraInfoConstPtr& l_cam_info,
    const sensor_msgs::ImageConstPtr& r_image,
    const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{
  ros::Time current_time = l_image->header.stamp;

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

  int64_t utime = (int64_t) floor(current_time.toSec()  * 1E6);

  depth_producer_->setRightImage(right_ptr->image.data);
  odom_->processFrame(left_ptr->image.data, depth_producer_);

  Eigen::Isometry3d cam_to_local = odom_->getPose();

  Eigen::Isometry3d motion_estimate = odom_->getMotionEstimate();

  const Eigen::MatrixXd & motion_cov = odom_->getMotionEstimateCov();
  fovis::MotionEstimateStatusCode estimation_status = odom_->getMotionEstimateStatus();

  if (estimation_status != fovis::SUCCESS){
    std::cout << "vo fail\n";
    return;
  }
  if (std::isnan(motion_estimate.translation().x())) {
    std::cout << "vo fail [NaN]\n";
    return;
  }

  pose_ = pose_ * motion_estimate;
  cam_to_local = pose_;

  // rotate coordinate frame so that look vector is +X, and up is +Z
  Eigen::Matrix3d M;
  M <<  0,  0, 1,
      -1,  0, 0,
      0, -1, 0;

  cam_to_local = M * cam_to_local;
  Eigen::Vector3d translation(cam_to_local.translation());
  Eigen::Quaterniond rotation(cam_to_local.rotation());
  rotation = rotation * M.transpose();

  Eigen::Vector3d rpy = (cam_to_local.rotation()*M.transpose()).eulerAngles(0, 1, 2);

  std::cout << "xyz-rpy: " << translation(0) << " " << translation(1) << " " << translation(2) << " -- "
      << rpy(0)/M_PI*180.0 << " " << rpy(1)/M_PI*180.0 << " " << rpy(2)/M_PI*180.0 << " " << std::endl;

  // Publish the odometry
  tf::Quaternion r;
  r[0] = rotation.x();
  r[1] = rotation.y();
  r[2] = rotation.z();
  r[3] = rotation.w();

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "vo";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = translation(0);
  odom_trans.transform.translation.y = translation(1);
  odom_trans.transform.translation.z = translation(2);
  tf::quaternionTFToMsg(r, odom_trans.transform.rotation);

  //send the transform
  odom_broadcaster_.sendTransform(odom_trans);

  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "base_link";

  //set the position
  odom.pose.pose.position.x = translation(0);
  odom.pose.pose.position.y = translation(1);
  odom.pose.pose.position.z = translation(2);
  tf::quaternionTFToMsg(r, odom.pose.pose.orientation);

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = 0;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = 0;

  odom_pub_.publish(odom);

  // publish pose to LCM
  bot_core_pose_t pose_msg;
  memset(&pose_msg, 0, sizeof(pose_msg));
  pose_msg.utime = (int64_t) floor(current_time.toSec()  * 1E6);
  pose_msg.pos[0] = translation(0);
  pose_msg.pos[1] = translation(1);
  pose_msg.pos[2] = translation(2);  
  pose_msg.orientation[0] =  rotation.w();  
  pose_msg.orientation[1] =  rotation.x();  
  pose_msg.orientation[2] =  rotation.y();  
  pose_msg.orientation[3] =  rotation.z();  
  bot_core_pose_t_publish(lcmref_, "POSE", &pose_msg);

}


fovis::StereoCalibration* ros_fovis_stereo::load_calibration(const std::string & config_name)
{
  bot_param_ = bot_param_new_from_file(config_name.c_str());

  fovis::StereoCalibrationParameters sparams;
  std::string key_prefix_ = "cameras." + camera_name_;

  std::string key_prefix_str;
  fovis::CameraIntrinsicsParameters* params;

  for (int i=0; i < 2; ++i) {
    if (i == 0) {
      key_prefix_str = std::string(key_prefix_) + ".left";
      params = &(sparams.left_parameters);
    } else {
      key_prefix_str = std::string(key_prefix_) + ".right";
      params = &(sparams.right_parameters);
    }
    params->width = bot_param_get_int_or_fail(bot_param_, (key_prefix_str+".width").c_str());
    params->height = bot_param_get_int_or_fail(bot_param_,(key_prefix_str+".height").c_str());
    params->fx = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".fx").c_str());
    params->fy = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".fy").c_str());
    params->cx = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".cx").c_str());
    params->cy = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".cy").c_str());
    params->k1 = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".k1").c_str());
    params->k2 = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".k2").c_str());
    params->k3 = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".k3").c_str());
    params->p1 = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".p1").c_str());
    params->p2 = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".p2").c_str());
  }

  // We assume rotation is a rotation matrix
  double rotation[9], translation[3];
  bot_param_get_double_array_or_fail(bot_param_,
      (key_prefix_str+".rotation").c_str(),
      &rotation[0],
      9);
  bot_param_get_double_array_or_fail(bot_param_,
      (key_prefix_str+".translation").c_str(),
      &translation[0],
      3);

  bot_matrix_to_quat(rotation, sparams.right_to_left_rotation);
  std::copy(translation, translation+3, sparams.right_to_left_translation);

  return new fovis::StereoCalibration(sparams);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "ros_fovis_stereo");

  //cv::imshow("Image", depth_ptr->image);
  //cv::waitKey(1);
  //cv::namedWindow("Image",1);

  std::cout << "     program: " << argv[ 0 ] << "\n";
  std::cout << " config file: " << argv[ 1 ] << "\n";
  std::cout << "camera block: " << argv[ 2 ] << "\n";

  ros_fovis_stereo vo(argv[ 2 ], argv[ 1 ]);
//  ros_fovis_stereo vo("wide_stereo","/home/mfallon/drc/software/build/config/drc_robot.cfg");

  ros::spin();
}
