/*
 * ros_lcm.cpp
 *
 *  Created on: Feb 4, 2011
 *      Author: hordurj
 *
 *  A ROS to LCM bridge
 *
 */

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

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;

class vo_fovis
{
public:
    vo_fovis();
    ~vo_fovis();
    void imageCb(const sensor_msgs::ImageConstPtr& l_image,
                 const sensor_msgs::CameraInfoConstPtr& l_cam_info,
                 const sensor_msgs::ImageConstPtr& r_image,
                 const sensor_msgs::CameraInfoConstPtr& r_cam_info);

private:
    ros::NodeHandle nh_;
    tf::TransformListener* listener_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter rgb_image_sub_, depth_image_sub_;

    message_filters::Subscriber<sensor_msgs::CameraInfo> rgb_info_sub_, depth_info_sub_;
    message_filters::Synchronizer<MySyncPolicy> sync_;
//    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
//      sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster odom_broadcaster_;

    bool use_sync_;

    fovis::PrimeSenseCalibration* kcal_;
    fovis::VisualOdometry odom_;
    fovis::PrimeSenseDepth depth_producer_;
    fovis::DepthImage* depth_image_;
    float* depth_data_;
    Eigen::Isometry3d pose_;

    fovis::PrimeSenseCalibration* load_calibration();
};

vo_fovis::vo_fovis() : it_(nh_), sync_(MySyncPolicy(10)),
      kcal_(load_calibration()),
      odom_(kcal_->getRgbRectification(), 
        fovis::VisualOdometry::getDefaultOptions()),
      depth_producer_(kcal_), pose_(Eigen::Isometry3d::Identity()) // 3)
{
    listener_ = 0;
    use_sync_ = true;
 
    fovis::CameraIntrinsicsParameters rgb_params = kcal_->getParameters().rgb_params;
    depth_image_ = new fovis::DepthImage(rgb_params, rgb_params.width, rgb_params.height);
    depth_data_ = new float[rgb_params.width * rgb_params.height];
    if (use_sync_)
    {
        depth_image_sub_.subscribe(it_, "/camera/depth/image_raw", 1);
        depth_info_sub_.subscribe(nh_, "/camera/depth/camera_info", 1);
        rgb_image_sub_.subscribe(it_, "/camera/rgb/image_mono", 1);
        rgb_info_sub_.subscribe(nh_, "/camera/rgb/camera_info", 1);

        sync_.connectInput(depth_image_sub_, depth_info_sub_, rgb_image_sub_, rgb_info_sub_);
        sync_.registerCallback( boost::bind(&vo_fovis::imageCb, this, _1, _2, _3, _4) );
    } 

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("vo", 50);    
}

vo_fovis::~vo_fovis()
{
  delete depth_image_;
  delete [] depth_data_;
}

void vo_fovis::imageCb(const sensor_msgs::ImageConstPtr& depth_image,
                             const sensor_msgs::CameraInfoConstPtr& depth_cam_info,
                             const sensor_msgs::ImageConstPtr& rgb_image,
                             const sensor_msgs::CameraInfoConstPtr& rgb_cam_info)
{
  ros::Time current_time = rgb_image->header.stamp; //ros::Time::now();

  cv_bridge::CvImageConstPtr depth_ptr;
  cv_bridge::CvImageConstPtr rgb_ptr;
  try
  {
    depth_ptr = cv_bridge::toCvShare(depth_image,"16SC1");
    rgb_ptr = cv_bridge::toCvShare(rgb_image,"mono8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // convert to meters, and set unknown depth values to NAN
  const uint16_t* depth_image_ptr = reinterpret_cast<const uint16_t*> (depth_ptr->image.data);

  int num_depth_pixels = depth_image->width * depth_image->height;
  for(int i=0; i<num_depth_pixels; ++i) {
    uint16_t d = depth_image_ptr[i];
    if(d != 0) {
      depth_data_[i] = d * 1e-3;
    } else {
      depth_data_[i] = NAN;
    }
  }
  depth_image_->setDepthImage(depth_data_);
  //depth_producer_.setDisparityData(reinterpret_cast<const uint16_t*> (depth_ptr->image.data));
  //odom_.processFrame(rgb_ptr->image.data, &depth_producer_);
  
  odom_.processFrame(rgb_ptr->image.data, depth_image_);

  Eigen::Isometry3d cam_to_local = odom_.getPose();
  Eigen::Isometry3d motion_estimate = odom_.getMotionEstimate();
  const Eigen::MatrixXd & motion_cov = odom_.getMotionEstimateCov();
  fovis::MotionEstimateStatusCode estimation_status = odom_.getMotionEstimateStatus();

  if (estimation_status != fovis::SUCCESS) return;
  if (std::isnan(motion_estimate.translation().x())) return;

  pose_ = pose_ * motion_estimate;
  cam_to_local = pose_;

  // rotate coordinate frame so that look vector is +X, and down is +Z
  /*
  Eigen::Matrix3d M;
  M <<  0,  0, 1,
        1,  0, 0,
        0,  1, 0;
  */

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
}

fovis::PrimeSenseCalibration* vo_fovis::load_calibration()
{
  // make up an initial calibration
  // This is calibration information for a specific Kinect
  // @todo load calibration information from somewhere.

  fovis::PrimeSenseCalibrationParameters kparams;
  kparams.width = 640;
  kparams.height = 480;

  kparams.depth_params.width = kparams.width;
  kparams.depth_params.height = kparams.height;
  kparams.depth_params.fx = 576.09757860;
  kparams.depth_params.fy = kparams.depth_params.fx;
  kparams.depth_params.cx = 321.06398107;
  kparams.depth_params.cy = 242.97676897;

  kparams.rgb_params.width = kparams.width;
  kparams.rgb_params.height = kparams.height;
  kparams.rgb_params.fx = 576.09757860;
  kparams.rgb_params.fy = kparams.rgb_params.fx;
  kparams.rgb_params.cx = 321.06398107;
  kparams.rgb_params.cy = 242.97676897;

  kparams.shift_offset = 1079.4753;
  kparams.projector_depth_baseline = 0.07214;

  Eigen::Matrix3d R;
  R << 0.999999, -0.000796, 0.001256, 0.000739, 0.998970, 0.045368, -0.001291, -0.045367, 0.998970;
  kparams.depth_to_rgb_translation[0] = -0.015756;
  kparams.depth_to_rgb_translation[1] = -0.000923;
  kparams.depth_to_rgb_translation[2] =  0.002316;
  Eigen::Quaterniond Q(R);
  kparams.depth_to_rgb_quaternion[0] = Q.w();
  kparams.depth_to_rgb_quaternion[1] = Q.x();
  kparams.depth_to_rgb_quaternion[2] = Q.y();
  kparams.depth_to_rgb_quaternion[3] = Q.z();

  return new fovis::PrimeSenseCalibration(kparams);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vo_fovis");

    //cv::imshow("Image", depth_ptr->image);
    //cv::waitKey(1);
    //cv::namedWindow("Image",1);

    vo_fovis vo;

    ros::spin();
}

