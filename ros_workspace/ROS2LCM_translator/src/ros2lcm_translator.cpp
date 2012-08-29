// Selective ros2lcm translator
//
// As of August 2012:
// - tilting laser
// - base_laser
// - some tf poses  - as rigid_transform_t
// - odometery - as rigid_transform_t
// - RGB stills from wide stereo
// - Point clouds from wide stereo
//
// mfallon aug 2012
#include "cv.h"
#include "highgui.h"


#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/message_filter.h>

#include <lcm/lcm-cpp.hpp>
//#include <lcmtypes/bot_core.h>
#include <lcmtypes/bot_core.h>
#include <lcmtypes/drc_lcmtypes.h>
#include <lcmtypes/visualization.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>

#define VERBOSE false
using namespace std;
using namespace cv;

class App{
  public:
    App();
    ~App();

    lcm_t* get_lcmref(){return lcmref_;}

  private:
    lcm_t* lcmref_ ;

    ros::NodeHandle node_;
    tf::Transformer transformer;

    uint8_t* image_data;

    ros::Subscriber tilt_scan_sub_, base_scan_sub_, odom_sub_, tf_sub_;
    void odom_cb(const nav_msgs::OdometryConstPtr& msg);
    void tf_cb(const tf::tfMessageConstPtr& msg);
    void base_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);
    void tilt_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);
    ros::Subscriber clock_sub_,left_image_sub_,right_image_sub_,points_sub_;
    void clock_cb(const rosgraph_msgs::ClockConstPtr& msg);
    void left_image_cb(const sensor_msgs::ImageConstPtr& msg);
    void right_image_cb(const sensor_msgs::ImageConstPtr& msg);
    void points_cb(const sensor_msgs::PointCloud2ConstPtr& msg);

    void send_image(const sensor_msgs::ImageConstPtr& msg,string channel );
    void send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel );
    void send_rigid_transform(tf::StampedTransform& transform,string channel );
    void send_obj(tf::StampedTransform& transform, int pose_collection_id, int64_t pose_id, bool reset );
    void send_points(const sensor_msgs::PointCloud2ConstPtr& msg,string channel );

    int points_cb_counter; // temp used to skip frames of points
};

App::App(){
  lcmref_ = lcm_create(NULL);

  tf_sub_ = node_.subscribe(string("/tf"), 10, &App::tf_cb,this);
  odom_sub_ = node_.subscribe(string("/base_odometry/odom"), 10, &App::odom_cb,this);
  base_scan_sub_ = node_.subscribe(string("/base_scan"), 10, &App::base_scan_cb,this);
  tilt_scan_sub_ = node_.subscribe(string("/tilt_scan"), 10, &App::tilt_scan_cb,this);

  clock_sub_ = node_.subscribe(string("/clock"), 10, &App::clock_cb,this);
  left_image_sub_ = node_.subscribe(string("/wide_stereo/left/image_rect_color"), 10, &App::left_image_cb,this);
  right_image_sub_ = node_.subscribe(string("/wide_stereo/right/image_rect_color"), 10, &App::right_image_cb,this);
  points_sub_ = node_.subscribe(string("/wide_stereo/points2"), 10, &App::points_cb,this);
  points_cb_counter=0;
};

App::~App()  {
}


Eigen::Quaterniond euler_to_quat(double yaw, double pitch, double roll) {
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}

void quat_to_euler(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}

bool pcdXYZRGB_to_lcm(lcm_t *lcm, pcl::PointCloud<pcl::PointXYZRGB> &cloud,
    int64_t msg_time,int pose_collection_id, int64_t pose_id,
    bool reset){

  int npoints =cloud.points.size();
  //int pose_id = 0; // use timestamp

  int64_t point_lists_id = pose_id; // use timestamp
  int point_type = 1; // point
  string collection_name = "Temp";
  int cloud_collection = 1;

  vs_point3d_list_collection_t plist_coll;
  plist_coll.id = cloud_collection;
  plist_coll.name =(char*)   collection_name.c_str();
  plist_coll.type =point_type; // collection of points
  plist_coll.reset = reset;
  plist_coll.nlists = 1; // number of seperate sets of points
  vs_point3d_list_t plist[plist_coll.nlists];

  // loop here for many lists
  vs_point3d_list_t* this_plist = &(plist[0]);
  // 3.0: header
  this_plist->id =point_lists_id; //bot_timestamp_now();
  this_plist->collection = pose_collection_id;
  this_plist->element_id = pose_id;
  // 3.1: points/entries (rename)
  vs_point3d_t* points = new vs_point3d_t[npoints];
  this_plist->npoints = npoints;
  // 3.2: colors:
  vs_color_t* colors = new vs_color_t[npoints];
  this_plist->ncolors = npoints;
  // 3.3: normals and ids:
  this_plist->nnormals = 0;
  this_plist->normals = NULL;
  this_plist->npointids = 0;
  this_plist->pointids= NULL;

  for(int j=0; j<npoints; j++) {
      colors[j].r = cloud.points[j].r/255.0; // points_collection values range 0-1
      colors[j].g = cloud.points[j].g/255.0;
      colors[j].b = cloud.points[j].b/255.0;
      points[j].x = cloud.points[j].z;
      points[j].y = - cloud.points[j].x;
      points[j].z = - cloud.points[j].y;
  }
  this_plist->colors = colors;
  this_plist->points = points;
  plist_coll.point_lists = plist;
  vs_point3d_list_collection_t_publish(lcm,"POINTS_COLLECTION",&plist_coll);

  delete colors;
  delete points;
}


//TODO: properly fill these fields:
// seq, frame_id, is_bigendian, point_step, row_step, is_dense
void App::send_points(const sensor_msgs::PointCloud2ConstPtr& msg,string channel){

  drc_pointcloud2_t pc;
  pc.utime = (int64_t) floor(msg->header.stamp.toSec()  * 1E6);
  pc.height = msg->height;
  pc.width = msg->width;
  pc.nfields = msg->fields.size();
  drc_pointfield_t* fields = new drc_pointfield_t[pc.nfields];
  for (size_t i=0;i < msg->fields.size();i++){
    //cout << " field: " << msg->fields[i].name << " " << (int) msg->fields[i].datatype << "\n";
    fields[i].name = (char*) msg->fields[i].name.c_str();
    fields[i].offset = msg->fields[i].offset;
    fields[i].datatype = msg->fields[i].datatype;
    fields[i].count =msg->fields[i].count;
  }
  pc.fields = fields;
  // pc.nfields =0;
  // pc.fields = NULL;
  //  pc.data_nbytes = 0;
  //  pc.data = NULL;

  pc.data_nbytes = (int) msg->data.size();
  uint8_t* raw_data = new uint8_t [ pc.data_nbytes];
  copy(msg->data.begin(), msg->data.end(), raw_data);
  pc.data = raw_data;

  drc_pointcloud2_t_publish(lcmref_, channel.c_str() ,&pc);
  delete[] raw_data;
}




// typical contents:
// 307200 pts | 480 height | 640 width | xyz rgb 4 field | 4bytes per field
//  n_bytes: 4915200bytes: 640 x 480 x 4field x 1byte
// nan published when no xyz available
void App::points_cb(const sensor_msgs::PointCloud2ConstPtr& msg){
  points_cb_counter++;
  if (points_cb_counter %10 !=0){
    cout << "skip\n";
    return;
  }

  int64_t utime = (int64_t) floor(msg->header.stamp.toSec()  * 1E6);

  send_points(msg, "WIDE_STEREO_POINTS");

  if (1==1){//(VERBOSE){
    cout << "got points: " << utime << "\n";
    cout << "    height: " << msg->height << "\n";
    cout << "     width: " << msg->width  << "\n";
    cout << "point_step: " << msg->point_step << "\n";
    cout << "  row_step: " << msg->row_step << "\n";
    cout << "  is_dense: " << (int) msg->is_dense << "\n";
    cout << " bigendian: " << (int) msg->is_bigendian << "\n";
    cout << "  n_fields: " << msg->fields.size() << "\n";
    cout << "  n_points: " << msg->data.size() << "\n";
    for (size_t i=0;i < msg->fields.size();i++){
      cout << " field: "
          << msg->fields[i].name << " "
          << msg->fields[i].offset << " "
          << (int) msg->fields[i].datatype << " "
          << msg->fields[i].count << "\n";
    }
  }

  try {
    // Look up transformation:
    tf::StampedTransform transform;
    transformer.lookupTransform("/odom_combined", "/wide_stereo_link", msg->header.stamp , transform);

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg (*msg, cloud);

    //pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
    //std::cout << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

    if (VERBOSE) cout << "Cloud contains: " << cloud.points.size() << " points\n";

    // Transmit pose and object collection:
    int pose_collection_id = 0;
    int64_t pose_id = floor(transform.stamp_.toSec()  * 1E6); // which specific pose
    bool reset = true;
    send_obj(transform,pose_collection_id,pose_id, reset );
    pcdXYZRGB_to_lcm(lcmref_ , cloud,utime,pose_collection_id,pose_id,reset);
  }  catch (tf::TransformException ex)   {
     ROS_ERROR("lookupTransform failed: %s", ex.what());
  }



}

// typical contents: 640 x 480 images
// points: xyz
// channels: rgb u v
void App::left_image_cb(const sensor_msgs::ImageConstPtr& msg){
  send_image(msg, "WIDE_LEFT_RECT");
}
void App::right_image_cb(const sensor_msgs::ImageConstPtr& msg){
  send_image(msg, "WIDE_RIGHT_RECT");
}

// TODO: jpeg compression:
//       look into openni_utils/openni_ros2rgb for code
// TODO: pre-allocate image_data and retain for speed - when size is known
void App::send_image(const sensor_msgs::ImageConstPtr& msg,string channel ){
  image_data = new uint8_t [640*480*3];
  IplImage *imageBGR = cvCreateImage(cvSize( msg->width ,   msg->height), 8, 3);
  IplImage* imageRGB = cvCreateImage(cvSize( msg->width ,   msg->height), 8, 3);

  copy(msg->data.begin(), msg->data.end(), image_data);
  imageBGR->imageData = (char*) image_data;
  cvCvtColor( imageBGR, imageRGB, CV_BGR2RGB );
  Mat imageRGBmat(imageRGB);

  bot_core_image_t img;
  img.utime = (int64_t) floor(msg->header.stamp.toSec()  * 1E6);
  img.width = msg->width;
  img.height =msg->height;
  img.row_stride = 3*msg->width; // check this
  img.size = msg->width*msg->height*3;
  img.nmetadata=0;
  img.metadata=NULL;
  img.data=imageRGBmat.data;
  img.pixelformat =BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
  bot_core_image_t_publish(lcmref_,channel.c_str(), &img);

  if (VERBOSE){
    cout << "got: " << channel<< "\n";
    cout << msg->width << ", " << msg->height << ", "
        << msg->encoding << ", " << ((int) msg->is_bigendian) << ", " << msg->step << "\n";
    //640, 480, mono8, 0, 640
    cout << img.utime << "\n";
  }
  cvReleaseImage(&imageBGR);
  cvReleaseImage(&imageRGB);
  delete[] image_data;
}

// TODO: not published yet. Is it needed?
void App::clock_cb(const rosgraph_msgs::ClockConstPtr& msg){
  int64_t clock_utime = (int64_t) floor(msg->clock.toSec()  * 1E6);
  if (VERBOSE) cout << "got clock msg: " << clock_utime << "\n";
}

void App::tilt_scan_cb(const sensor_msgs::LaserScanConstPtr& msg){
  send_lidar(msg, "TILT_SCAN");
}

void App::base_scan_cb(const sensor_msgs::LaserScanConstPtr& msg){
  send_lidar(msg, "BASE_SCAN");
}

void App::send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel ){
  bot_core_planar_lidar_t scan_out;

  float range_line[msg->ranges.size()];
  for (size_t i=0; i < msg->ranges.size(); i++){
    range_line[i] = msg->ranges[i];
  }
  scan_out.utime = (int64_t) floor(msg->header.stamp.toSec()  * 1E6);
  scan_out.nranges =msg->ranges.size();
  scan_out.ranges=range_line;
  scan_out.nintensities=0;
  scan_out.intensities=NULL;
  scan_out.rad0 = msg->angle_min;
  scan_out.radstep = msg->angle_increment;
  bot_core_planar_lidar_t_publish(lcmref_,channel.c_str(),&scan_out);
  if (VERBOSE) cout << channel << ": "<< scan_out.utime << "\n";
}

void App::odom_cb(const nav_msgs::OdometryConstPtr& msg){
  if (VERBOSE) cout << "Odom\n";
  bot_core_pose_t pose;
  pose.utime = (int64_t) floor(msg->header.stamp.toSec()  * 1E6);
  pose.pos[0] = msg->pose.pose.position.x;
  pose.pos[1] = msg->pose.pose.position.y;
  pose.pos[2] = msg->pose.pose.position.z;
  pose.orientation[0] = msg->pose.pose.orientation.w;
  pose.orientation[1] = msg->pose.pose.orientation.x;
  pose.orientation[2] = msg->pose.pose.orientation.y;
  pose.orientation[3] = msg->pose.pose.orientation.z;
  bot_core_pose_t_publish(lcmref_,"POSE",&pose);
}

void App::tf_cb(const tf::tfMessageConstPtr& msg){
  //cout << "tf n: " << msg->transforms.size() << "\n";

  for (unsigned int i = 0; i < msg->transforms.size(); ++i){
    tf::StampedTransform transform;
    tf::transformStampedMsgToTF(msg->transforms[i], transform);

    try{
      transformer.setTransform(transform);
    }
    catch (tf::TransformException& ex){
      std::string temp = ex.what();
      ROS_ERROR("Failure to set received transform from %s to %s with error: %s\n",
          msg->transforms[i].child_frame_id.c_str(),
          msg->transforms[i].header.frame_id.c_str(), temp.c_str());
    }
  }

  // Lookup a transform
  try {
    tf::StampedTransform transform;
    tf::Quaternion t_quat;
    tf::Vector3 t;

    /////////////////////////
    // Future Faulty time stamp - do not republish for now:
    //transformer.lookupTransform("/base_link", "/base_laser_link", ros::Time(0), transform);
    //send_rigid_transform(transform, "BODY_TO_BASE_SCAN");

    transformer.lookupTransform("/base_link", "/laser_tilt_link", ros::Time(0), transform);
    send_rigid_transform(transform, "BODY_TO_TILT_SCAN");

    transformer.lookupTransform("/base_link", "/wide_stereo_link", ros::Time(0), transform);
    send_rigid_transform(transform, "BODY_TO_WIDE_STEREO");

    //int64_t current_utime = (int64_t)  floor(transform.stamp_.toSec()  * 1E6);
    //cout << current_utime << " is current_utime\n";
  }
  catch (tf::TransformException ex)
  {
     ROS_ERROR("lookupTransform failed: %s", ex.what());
  }
}

void App::send_rigid_transform(tf::StampedTransform& transform,string channel ){
  tf::Quaternion t_quat= transform.getRotation();
  tf::Vector3 t= transform.getOrigin();

  bot_core_rigid_transform_t tf_out;
  tf_out.utime = (int64_t)  floor(transform.stamp_.toSec()  * 1E6);
  tf_out.trans[0] =t.x();
  tf_out.trans[1] =t.y();
  tf_out.trans[2] =t.z();
  tf_out.quat[0] =t_quat.w();
  tf_out.quat[1] =t_quat.x();
  tf_out.quat[2] =t_quat.y();
  tf_out.quat[3] =t_quat.z();
  bot_core_rigid_transform_t_publish(lcmref_,channel.c_str(),&tf_out);

  if (VERBOSE){
    std::cout << channel << ": "<< tf_out.utime << "\n";
    std::cout << "                ->                 : " << t.x() << " " << t.y() << " " << t.z() << std::endl;
    std::cout << "       "  << t_quat.w() << " " << t_quat.x() << " " << t_quat.y() << " " << t_quat.z() << std::endl;
  }
}


void App::send_obj(tf::StampedTransform& transform,int pose_collection_id,int64_t pose_id, bool reset  ){
  tf::Quaternion t_quat= transform.getRotation();
  tf::Vector3 t= transform.getOrigin();

  Eigen::Quaterniond t_quat_eigen(t_quat[3],t_quat[0],t_quat[1],t_quat[2]);
  // tf::Quaternion contructor is xyzw
  // Eigen constructor is wxyz
  // bot_pose.orientation is wxyz
  double ypr[3];
  quat_to_euler(t_quat_eigen, ypr[0],ypr[1],ypr[2]) ;

  // Send a pose to hang the model on:
  int64_t initial_timestamp =-1;
  vs_obj_collection_t objs;
  objs.id = pose_collection_id;
  objs.name = (char*)  "Zero Pose"; // "Trajectory";
  objs.type = VS_OBJ_COLLECTION_T_AXIS3D; // a pose
  objs.reset = reset; // true will delete them from the viewer
  objs.nobjs = 1;
  vs_obj_t poses[objs.nobjs];
  poses[0].id = pose_id; // which specific pose
  poses[0].x = t.x();
  poses[0].y = t.y();
  poses[0].z = t.z();
  poses[0].yaw = ypr[0];
  poses[0].pitch = ypr[1];
  poses[0].roll = ypr[2];
  objs.objs = poses;
  vs_obj_collection_t_publish(lcmref_, "OBJ_COLLECTION", &objs);

  if (VERBOSE){
    std::cout << objs.name << " @ "<< pose_id << "\n";
    std::cout << "                ->                 : " << t.x() << " " << t.y() << " " << t.z() << std::endl;
    std::cout << "       "  << t_quat.w() << " " << t_quat.x() << " " << t_quat.y() << " " << t_quat.z() << std::endl;
  }
}


int main(int argc, char **argv){
  std::cout << "ros2lcm_translator launched\n";
  ros::init(argc, argv, "ros2lcm_translator");
  App *app = new App();
  ros::spin();
  return 0;
}
