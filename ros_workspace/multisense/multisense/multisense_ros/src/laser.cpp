/**
 * @file laser.cpp
 *
 * Copyright 2013
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * This software is free: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation,
 * version 3 of the License.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#include <multisense_ros/laser.h>
#include <multisense_ros/RawLidarData.h>
#include <angles/angles.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>

#include <arpa/inet.h>

using namespace crl::multisense;

namespace { // anonymous

bool computePostSpindleCal(const KDL::Tree&  tree,
                           const KDL::Frame& spindle_T_laser,
                           KDL::Frame&       post_spindle_cal)
{
    KDL::Chain chain;
    if (!tree.getChain("hokuyo_link", "head_hokuyo_frame", chain)) {
        ROS_ERROR("Error extracting post-spindle chain from KDL tree");
        return false;
    }

    if (chain.getNrOfJoints() != 0) {
        ROS_ERROR("Expected 0 joints in chain. Got %u\n", 
                  chain.getNrOfJoints());
        return false;
    }

    KDL::JntArray joint_pos(0);
    KDL::ChainFkSolverPos_recursive fksolver(chain);

    KDL::Frame scan_post_spindle_cal_T_hokuyo_head;
    if(fksolver.JntToCart(joint_pos, scan_post_spindle_cal_T_hokuyo_head) < 0) {
        ROS_ERROR("Error in FK for post-spindle calcs");
        return false;
    }

    KDL::Frame nominal_T_optical(KDL::Rotation::RPY(M_PI/2, 
                                                    -M_PI/2, 0.0).Inverse(), 
                                 KDL::Vector());
    post_spindle_cal = (nominal_T_optical * spindle_T_laser * 
                        (scan_post_spindle_cal_T_hokuyo_head * 
                         nominal_T_optical).Inverse());

    return true;
}

bool computePreSpindleCal(const KDL::Tree&  tree,
                          const KDL::Frame& camera_T_spindle_fixed,
                          KDL::Frame&       pre_spindle_cal)
{
    KDL::Chain head_spindle_chain;
    if (!tree.getChain("head", "pre_spindle", head_spindle_chain)) {
        ROS_ERROR("Error extracting head-spindle chain from KDL tree");
        return false;
    }

    KDL::Chain head_cam_chain;
    if (!tree.getChain("head", "left_camera_optical_frame", head_cam_chain)) {
        ROS_ERROR("Error extracting head-camera chain from KDL tree");
        return false;
    }

    if (head_cam_chain.getNrOfJoints() != 0) {
        ROS_ERROR("Expected 0 joints in head-camera chain. Got %u\n", 
                  head_cam_chain.getNrOfJoints());
        return false;
    }

    if (head_spindle_chain.getNrOfJoints() != 0) {
        ROS_ERROR("Expected 0 joints in head-spindle chain. Got %u\n", 
                  head_spindle_chain.getNrOfJoints());
        return false;
    }

    KDL::JntArray joint_pos(0);
    KDL::ChainFkSolverPos_recursive head_spindle_solver(head_spindle_chain);
    KDL::ChainFkSolverPos_recursive head_cam_solver(head_cam_chain);
    KDL::Frame head_T_pre_spindle;
    KDL::Frame head_T_cam_optical;
    if(head_spindle_solver.JntToCart(joint_pos, head_T_pre_spindle) < 0) {
        ROS_ERROR("Error in FK for head_spindle calcs");
        return false;
    }

    if(head_cam_solver.JntToCart(joint_pos, head_T_cam_optical) < 0) {
        ROS_ERROR("Error in FK for head_cam calcs");
        return false;
    }

    KDL::Frame pre_spindle_T_pre_spindle_rot(KDL::Rotation::RPY(M_PI/2, 
                                                                -M_PI/2, 0.0).Inverse(), 
                                             KDL::Vector());

    KDL::Frame cam_opt_T_pre_spindle = (head_T_cam_optical.Inverse() * 
                                        head_T_pre_spindle);

    pre_spindle_cal = (cam_opt_T_pre_spindle.Inverse()
                       * camera_T_spindle_fixed * 
                       pre_spindle_T_pre_spindle_rot.Inverse());

    return true;
}


bool computeCal(const std::string& robot_desc_string,
                const KDL::Frame&  spindle_T_laser, 
                const KDL::Frame&  camera_T_spindle_fixed,
                KDL::Frame&        pre_spindle_cal, 
                KDL::Frame&        post_spindle_cal)
{
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(robot_desc_string, tree)) {
        ROS_ERROR("Error parsing robot description using kdl_parser");
        return false;
    }

    bool success;
    success = computePostSpindleCal(tree, spindle_T_laser, post_spindle_cal);
    if (!success) {
        ROS_ERROR("Error calculating post spindle calibration");
        return false;
    }

    success = computePreSpindleCal(tree, camera_T_spindle_fixed, pre_spindle_cal);
    if (!success) {
        ROS_ERROR("Error calculating pre spindle calibration");
        return false;
    }

    return true;
}

KDL::Frame makeFrame(float T[4][4])
{
    KDL::Frame out;

    for (int i=0; i < 3; i++) {
        for (int j=0; j < 3; j++)
            out.M(i,j) = T[i][j];
    }

    out.p[0] = T[0][3];
    out.p[1] = T[1][3];
    out.p[2] = T[2][3];

    return out;
}

void getLaserCal(lidar::Calibration& c,
                 const std::string&  robot_desc,
                 KDL::Frame&         pre_spindle_cal,
                 KDL::Frame&         post_spindle_cal)
{   
    KDL::Frame spindle_T_laser = makeFrame(c.laserToSpindle).Inverse();
    KDL::Frame camera_T_spindle_fixed = makeFrame(c.cameraToSpindleFixed);

    bool success;
    success = computeCal(robot_desc,
                         spindle_T_laser, camera_T_spindle_fixed,
                         pre_spindle_cal, post_spindle_cal);
    if (!success)
        ROS_ERROR("Error computing lidar calibration.");
}

void pushCal(sensor_msgs::JointState& msg, 
	     const std::string&       name, 
	     double                   pos)
{
    msg.name.push_back(name);
    msg.position.push_back(pos);
    msg.velocity.push_back(0.0);
    msg.effort.push_back(0.0);
}

}; // anonymous

namespace multisense_ros {

const float Laser::EXPECTED_RATE = 40.0;

namespace { // anonymous

//
// Shims for c-style driver callbacks

void lCB(const lidar::Header&        header,
	 const lidar::RangeType*     rangesP,
	 const lidar::IntensityType* intensitiesP,
	 void*                       userDataP)
{
    reinterpret_cast<Laser*>(userDataP)->scanCallback(header, rangesP, intensitiesP);
}

void pCB(const lidar::Header&        header,
	 const lidar::RangeType*     rangesP,
	 const lidar::IntensityType* intensitiesP,
	 void*                       userDataP)
{
    reinterpret_cast<Laser*>(userDataP)->pointCloudCallback(header, rangesP, intensitiesP);
}

}; // anonymous

Laser::Laser(Channel* driver,
             const std::string& robot_desc)
    : driver_(driver),
      laser_diagnostics_(ros::NodeHandle(), "laser/diagnostics", EXPECTED_RATE),
      subscribers_(0)
{
    ros::NodeHandle nh("laser");

    //
    // Get frame_id

    nh.param("frame_id", frame_id_, std::string("/head_hokuyo_frame"));

    //
    // Stop lidar stream

    stop();

    //
    // Query calibration from sensor

    if (Status_Ok != driver->getLidarCalibration(lidar_cal_))
        ROS_WARN("could not query lidar calibration, using URDF defaults");
    else {

        //
        // Calibration for laser scan topic

        getLaserCal(lidar_cal_, robot_desc, scan_pre_spindle_cal_, scan_post_spindle_cal_);

        //
        // Calibration for point cloud topic

        pc_pre_spindle_cal_  = makeFrame(lidar_cal_.laserToSpindle);
        pc_post_spindle_cal_ = makeFrame(lidar_cal_.cameraToSpindleFixed);
    }

    //
    // Default joint message for LaserScan message

    js_msg_.name.push_back("hokuyo_joint");
    js_msg_.position.push_back(0.0);
    js_msg_.velocity.push_back(0.0);
    js_msg_.effort.push_back(0.0);

    double roll, pitch, yaw;
    scan_pre_spindle_cal_.M.GetRPY(roll, pitch, yaw);
    
    pushCal(js_msg_, "pre_spindle_cal_x_joint", scan_pre_spindle_cal_.p[0]);
    pushCal(js_msg_, "pre_spindle_cal_y_joint", scan_pre_spindle_cal_.p[1]);
    pushCal(js_msg_, "pre_spindle_cal_z_joint", scan_pre_spindle_cal_.p[2]);
    pushCal(js_msg_, "pre_spindle_cal_roll_joint",  roll);
    pushCal(js_msg_, "pre_spindle_cal_pitch_joint", pitch);
    pushCal(js_msg_, "pre_spindle_cal_yaw_joint",   yaw);

    scan_post_spindle_cal_.M.GetRPY(roll, pitch, yaw);
    pushCal(js_msg_, "post_spindle_cal_x_joint", scan_post_spindle_cal_.p[0]);
    pushCal(js_msg_, "post_spindle_cal_y_joint", scan_post_spindle_cal_.p[1]);
    pushCal(js_msg_, "post_spindle_cal_z_joint", scan_post_spindle_cal_.p[2]);
    pushCal(js_msg_, "post_spindle_cal_roll_joint",  roll);
    pushCal(js_msg_, "post_spindle_cal_pitch_joint", pitch);
    pushCal(js_msg_, "post_spindle_cal_yaw_joint",   yaw);

    //
    // Joint publisher

    ros::NodeHandle nh_js("laser_joint");
    js_pub_ = nh_js.advertise<sensor_msgs::JointState>("/joint_states", 40);

    //
    // Latched publisher for diagnostics

    js_diagnostics_pub_         = nh_js.advertise<JointDiagnostics>("diagnostics", 40, true);
    js_diagnostics_.joint_state = js_msg_;
    js_diagnostics_pub_.publish(js_diagnostics_);

    //
    // Create scan publisher

    scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan", 20,
                boost::bind(&Laser::subscribe, this),
                boost::bind(&Laser::unsubscribe, this));

    //
    // Create point cloud publisher

    point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points2", 5,
                       boost::bind(&Laser::subscribe, this),
                       boost::bind(&Laser::unsubscribe, this));
    
    //
    // Create calibration publisher

    ros::NodeHandle calibration_nh(nh, "calibration");
    raw_lidar_data_pub_  = calibration_nh.advertise<multisense_ros::RawLidarData>("raw_lidar_data", 20,
                           boost::bind(&Laser::subscribe, this),
                           boost::bind(&Laser::unsubscribe, this));

    //
    // Register callbacks, driver creates dedicated background thread for each

    driver_->addIsolatedCallback(lCB, this);
    driver_->addIsolatedCallback(pCB, this);

    // Set LCM components:
    if(!lcm_publish_.good()){
      std::cerr <<"ERROR: lcm is not good()" <<std::endl;
      ROS_ERROR("ERROR: lcm is not good()");
    }    
    
    for (size_t i=0; i < js_msg_.position.size() ; i++){
      lcm_state_.joint_name.push_back( js_msg_.name[i] );
      lcm_state_.joint_position.push_back( js_msg_.position[i] );
      lcm_state_.joint_velocity.push_back( js_msg_.velocity[i] );
      lcm_state_.joint_effort.push_back( js_msg_.effort[i] );
    }
    lcm_state_.num_joints = js_msg_.position.size();
}

Laser::~Laser()
{
    boost::mutex::scoped_lock lock(sub_lock_);
    stop();
}

void Laser::pointCloudCallback(const lidar::Header&        header,
                               const lidar::RangeType*     rangesP,
                               const lidar::IntensityType* intensitiesP)
{
    //
    // For diagnostics, only count here

    laser_diagnostics_.countStream();

    //
    // Get out if we have no work to do

    if (0 == point_cloud_pub_.getNumSubscribers())
        return;

    pointCloudPublish(header, rangesP, intensitiesP, false, true);
}

void Laser::pointCloudPublish(const lidar::Header&        header,
                               const lidar::RangeType*     rangesP,
                               const lidar::IntensityType* intensitiesP,
                               bool sendToLCM, bool sendToROS)   
{
    
    const uint32_t cloud_step = 16;

    point_cloud_.data.resize(cloud_step * header.pointCount);

    if (4 != point_cloud_.fields.size()) {

        point_cloud_.is_bigendian    = (htonl(1) == 1);
        point_cloud_.is_dense        = true;
        point_cloud_.point_step      = cloud_step;
        point_cloud_.height          = 1;
        point_cloud_.header.frame_id = "/left_camera_optical_frame";

        point_cloud_.fields.resize(4);
        point_cloud_.fields[0].name     = "x";
        point_cloud_.fields[0].offset   = 0;
        point_cloud_.fields[0].count    = 1;
        point_cloud_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        point_cloud_.fields[1].name     = "y";
        point_cloud_.fields[1].offset   = 4;
        point_cloud_.fields[1].count    = 1;
        point_cloud_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        point_cloud_.fields[2].name     = "z";
        point_cloud_.fields[2].offset   = 8;
        point_cloud_.fields[2].count    = 1;
        point_cloud_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        point_cloud_.fields[3].name     = "intensity";
        point_cloud_.fields[3].offset   = 12;
        point_cloud_.fields[3].count    = 1;
        point_cloud_.fields[3].datatype = sensor_msgs::PointField::UINT32;
    }

    point_cloud_.row_step     = header.pointCount;
    point_cloud_.width        = header.pointCount;
    point_cloud_.header.stamp = ros::Time::now();
    
    //
    // For convenience below

    uint8_t       *cloudP            = reinterpret_cast<uint8_t*>(&point_cloud_.data[0]);
    const uint32_t pointSize         = 3 * sizeof(float); // x, y, z
    const double   arcRadians        = 1e-6 * static_cast<double>(header.scanArc);
    const double   mirrorThetaStart  = -arcRadians / 2.0;
    const double   spindleAngleStart = angles::normalize_angle(1e-6 * static_cast<double>(header.spindleAngleStart));
    const double   spindleAngleEnd   = angles::normalize_angle(1e-6 * static_cast<double>(header.spindleAngleEnd));
    const double   spindleAngleRange = angles::normalize_angle(spindleAngleEnd - spindleAngleStart);

    // Vectors for LCM type:
    std::vector< float > x_vals, y_vals, z_vals;
    std::vector< int32_t > intensity;
    
    
    for(uint32_t i=0; i<header.pointCount; ++i, cloudP += cloud_step) {
        
        //
        // Percent through the scan arc

        const double percent = static_cast<double>(i) / static_cast<double>(header.pointCount - 1);

        //
        // The mirror angle for this point, invert for mirror motor direction

        const double mirrorTheta = -1.0 * (mirrorThetaStart + percent * arcRadians);

        //
        // The rotation about the spindle

        const double spindleTheta    = spindleAngleStart + percent * spindleAngleRange;
        const double cosSpindleTheta = std::cos(spindleTheta);
        const double sinSpindleTheta = std::sin(spindleTheta);

        const KDL::Rotation spindleFromMotor(cosSpindleTheta, -sinSpindleTheta, 0.0,
                                             sinSpindleTheta,  cosSpindleTheta, 0.0,
                                             0.0,              0.0,             1.0);
        //
        // The coordinate in left optical frame

        const double      rangeMeters = 1e-3 * static_cast<double>(rangesP[i]);  // from millimeters
        const KDL::Vector pointMotor  = (pc_pre_spindle_cal_ * 
                                         KDL::Vector(rangeMeters * -std::sin(mirrorTheta), 0.0,
                                                     rangeMeters *  std::cos(mirrorTheta)));
        const KDL::Vector pointCamera = pc_post_spindle_cal_ * (spindleFromMotor * pointMotor);
        
        //
        // Copy data to point cloud structure

        const float xyz[3] = {static_cast<float>(pointCamera.x()),
                              static_cast<float>(pointCamera.y()),
                              static_cast<float>(pointCamera.z())};
                        
        memcpy(cloudP, &(xyz[0]), pointSize);
        memcpy((cloudP + pointSize), &(intensitiesP[i]), sizeof(uint32_t));
        
        x_vals.push_back( xyz[0] );
        y_vals.push_back( xyz[1] );
        z_vals.push_back( xyz[2] );
        intensity.push_back(intensitiesP[i] );
    }

    if (sendToROS){
      point_cloud_pub_.publish(point_cloud_);
    }
    
    // Push the cloud to LCM
    if (sendToLCM){
      multisense::pointcloud_t lcm_cloud;
      int64_t utime_start =(int64_t) ros::Time::now().toNSec()/1000; // from nsec to usec
      lcm_cloud.utime = utime_start;
      lcm_cloud.x = x_vals;
      lcm_cloud.y = y_vals;
      lcm_cloud.z = z_vals;
      lcm_cloud.intensity = intensity;
      lcm_cloud.num_points =x_vals.size();
      lcm_publish_.publish("SCAN_CLOUD", &lcm_cloud);         
    }
}



void Laser::publishLCMTransforms(int64_t utime_out, int32_t spindleAngle){
  
  
    std::stringstream ss2;
    ss2 << "scan_pre_spindle_cal_\n" <<  scan_pre_spindle_cal_ ;
    ROS_ERROR("%s", ss2.str().c_str() );
  
    double roll, pitch, yaw;
    scan_pre_spindle_cal_.M.GetRPY(roll, pitch, yaw);
    
    ROS_ERROR("pre_spindle_cal_x_joint     %f", scan_pre_spindle_cal_.p[0]);
    ROS_ERROR("pre_spindle_cal_y_joint     %f", scan_pre_spindle_cal_.p[1]);
    ROS_ERROR("pre_spindle_cal_z_joint     %f", scan_pre_spindle_cal_.p[2]);
    ROS_ERROR("pre_spindle_cal_roll_joint  %f",  roll);
    ROS_ERROR("pre_spindle_cal_pitch_joint %f", pitch);
    ROS_ERROR("pre_spindle_cal_yaw_joint   %f",   yaw);
  
    KDL::Frame trans_frame, roll_frame, pitch_frame, yaw_frame;
    {
      trans_frame.p[0]= scan_pre_spindle_cal_.p[0];
      trans_frame.p[1]= scan_pre_spindle_cal_.p[1];
      trans_frame.p[2]= scan_pre_spindle_cal_.p[2];
      KDL::Rotation rot= KDL::Rotation::RPY(0, 0, 0 );
      trans_frame.M = rot;
      std::stringstream ss2;
      ss2 << "trans_frame\n" <<  trans_frame ;
      ROS_ERROR("%s", ss2.str().c_str() );  
    }    
    {
      roll_frame.p[0]= 0;
      roll_frame.p[1]= 0;
      roll_frame.p[2]= 0;
      KDL::Rotation rot= KDL::Rotation::RPY(roll, 0, 0 );
      roll_frame.M = rot;
      std::stringstream ss2;
      ss2 << "roll_frame\n" <<  roll_frame ;
      ROS_ERROR("%s", ss2.str().c_str() );  
    }
    {
      pitch_frame.p[0]= 0;
      pitch_frame.p[1]= 0;
      pitch_frame.p[2]= 0;
      KDL::Rotation rot= KDL::Rotation::RPY(0, pitch, 0 );
      pitch_frame.M = rot;
      std::stringstream ss2;
      ss2 << "pitch_frame\n" <<  pitch_frame ;
      ROS_ERROR("%s", ss2.str().c_str() );  
    }    
    
    {
      yaw_frame.p[0]= 0;
      yaw_frame.p[1]= 0;
      yaw_frame.p[2]= 0;
      KDL::Rotation rot= KDL::Rotation::RPY(0, 0, yaw );
      yaw_frame.M = rot;
      std::stringstream ss2;
      ss2 << "yaw_frame\n" <<  yaw_frame ;
      ROS_ERROR("%s", ss2.str().c_str() );  
    }    
    
    
    KDL::Frame final_frame = trans_frame* roll_frame*pitch_frame*yaw_frame;
    {
      std::stringstream ss2;
      ss2 << "final_frame\n" <<  final_frame ;
      ROS_ERROR("%s\n\n", ss2.str().c_str() );  
    }
  
    // camera-to-laser is defined by three transforms: 
    // (1) camera-to-pre-spindle (from calibration)
    // (2) sensed rotation 
    // (3) post-spindle-to-laser (from calibration)
    // + seem to require a unit rotation into our expected lidar frame

    double spindleAngleDouble = angles::normalize_angle(1e-6 * static_cast<double>(spindleAngle));
    double cosSpindleTheta = std::cos(spindleAngleDouble);
    double sinSpindleTheta = std::sin(spindleAngleDouble);    
    const KDL::Rotation spindleFromMotor(cosSpindleTheta, -sinSpindleTheta, 0.0,
                                          sinSpindleTheta,  cosSpindleTheta, 0.0,
                                          0.0,              0.0,             1.0);  
    KDL::Frame T_spindleFromMotor;
    T_spindleFromMotor.p[0]= 0;
    T_spindleFromMotor.p[1]= 0;
    T_spindleFromMotor.p[2]= 0;
    T_spindleFromMotor.M = spindleFromMotor;

    KDL::Frame pre_spindle_T_pre_spindle_rot(KDL::Rotation::RPY(-M_PI/2, -M_PI/2, 0.0) );
    
    KDL::Frame cam_to_laser = pc_post_spindle_cal_ * T_spindleFromMotor * pc_pre_spindle_cal_
                                    * pre_spindle_T_pre_spindle_rot;

    bot_core::rigid_transform_t cam_to_hokuyo_frame;
    cam_to_hokuyo_frame.utime = utime_out;
    cam_to_hokuyo_frame.trans[0] = cam_to_laser.p[0];
    cam_to_hokuyo_frame.trans[1] = cam_to_laser.p[1];
    cam_to_hokuyo_frame.trans[2] = cam_to_laser.p[2];
    cam_to_laser.M.GetQuaternion(cam_to_hokuyo_frame.quat[1], cam_to_hokuyo_frame.quat[2], cam_to_hokuyo_frame.quat[3], cam_to_hokuyo_frame.quat[0]);
    lcm_publish_.publish("CAMERA_TO_SCAN", &cam_to_hokuyo_frame);
}

void Laser::scanCallback(const lidar::Header&        header,
                         const lidar::RangeType*     rangesP,
                         const lidar::IntensityType* intensitiesP)
{
    //
    // Get out if we have no work to do 

    if (!(0 == (header.scanId % 40)         ||
          scan_pub_.getNumSubscribers() > 0 ||
          raw_lidar_data_pub_.getNumSubscribers() > 0))
        return;

    //
    // The URDF assumes that the laser straight up at a joint angle of 0 degrees

    const float offset = M_PI;

    const ros::Time start_absolute_time = ros::Time::now();
    const ros::Time start_relative_time = ros::Time(header.timeStartSeconds,
                                                    1000 * header.timeStartMicroSeconds);
    const ros::Time end_relative_time   = ros::Time(header.timeEndSeconds,
                                                    1000 * header.timeEndMicroSeconds);

    const float angle_start = static_cast<float>(header.spindleAngleStart) / 1000000.0f - offset;
    const float angle_end   = static_cast<float>(header.spindleAngleEnd) / 1000000.0f - offset;
    const float angle_diff  = angles::shortest_angular_distance(angle_start, angle_end);
    const float velocity    = angle_diff / (end_relative_time - start_relative_time).toSec();

    //
    // Publish joint state for beginning of scan

    js_msg_.header.frame_id = "";
    js_msg_.header.stamp    = start_absolute_time;
    js_msg_.position[0]     = angle_start;
    js_msg_.velocity[0]     = velocity;
    js_pub_.publish(js_msg_);

    int64_t utime_start =(int64_t) start_absolute_time.toNSec()/1000; // from nsec to usec
    lcm_state_.utime = utime_start;
    lcm_state_.joint_position[0]= angle_start;
    lcm_state_.joint_velocity[0]= velocity;
    lcm_publish_.publish("STATE_MULTISENSE", &lcm_state_);    
    publishLCMTransforms(utime_start, header.spindleAngleStart);
    
    //
    // Publish joint state for end of scan
    
    js_msg_.header.frame_id = "";
    ros::Time end_absolute_time = start_absolute_time + (end_relative_time - start_relative_time);
    js_msg_.header.stamp    = end_absolute_time;
    js_msg_.position[0]     = angle_end;
    js_msg_.velocity[0]     = velocity;
    js_pub_.publish(js_msg_);
    
    int64_t utime_end = (int64_t) end_absolute_time.toNSec()/1000; // from nsec to usec
    lcm_state_.utime = utime_end;
    lcm_state_.joint_position[0]= angle_end;
    lcm_state_.joint_velocity[0]= velocity;
    lcm_publish_.publish("MULTISENSE_STATE", &lcm_state_);        
    publishLCMTransforms(utime_end, header.spindleAngleEnd);

    //
    // Downsample diagnostics to 1 Hz

    if (header.scanId % 40 == 0) {
        js_diagnostics_.stamp        = start_absolute_time;
        js_diagnostics_.joint_state  = js_msg_;
        js_diagnostics_.current_rate = velocity;
        js_diagnostics_pub_.publish(js_diagnostics_);
    }
    
    
    const double arcRadians = 1e-6 * static_cast<double>(header.scanArc);
      
    // LCM:
    lcm_laser_msg_.utime = (int64_t) floor(start_absolute_time.toNSec()/1000);
    lcm_laser_msg_.ranges.resize( header.pointCount );
    lcm_laser_msg_.intensities.resize( header.pointCount );
    for (size_t i=0; i < header.pointCount; i++){
      lcm_laser_msg_.ranges[i]      = static_cast<float>(rangesP[i]) / 1000.0f; // from millimeters
      lcm_laser_msg_.intensities[i] = static_cast<float>(intensitiesP[i]);      // in device units
    }
    lcm_laser_msg_.nranges = header.pointCount;
    lcm_laser_msg_.nintensities=header.pointCount;
    lcm_laser_msg_.rad0 = -arcRadians / 2.0;
    lcm_laser_msg_.radstep = arcRadians / (header.pointCount - 1);
    lcm_publish_.publish("SCAN", &lcm_laser_msg_);
    
    pointCloudPublish(header, rangesP, intensitiesP, true, false);
    /////////////////////////////////////////////////////////////      
    
    if (scan_pub_.getNumSubscribers() > 0) {
        laser_msg_.header.frame_id = frame_id_;
        laser_msg_.header.stamp    = start_absolute_time;
        laser_msg_.scan_time       = (end_relative_time - start_relative_time).toSec();
        laser_msg_.time_increment  = laser_msg_.scan_time / header.pointCount;
        laser_msg_.angle_min       = -arcRadians / 2.0;
        laser_msg_.angle_max       = arcRadians / 2.0;
        laser_msg_.angle_increment = arcRadians / (header.pointCount - 1);
        laser_msg_.range_min       = 0.0;
        laser_msg_.range_max       = static_cast<double>(header.maxRange) / 1000.0;
        
        laser_msg_.ranges.resize(header.pointCount);
        laser_msg_.intensities.resize(header.pointCount);
        
        for (size_t i=0; i<header.pointCount; i++) {
            laser_msg_.ranges[i]      = static_cast<float>(rangesP[i]) / 1000.0f; // from millimeters
            laser_msg_.intensities[i] = static_cast<float>(intensitiesP[i]);      // in device units
        }

        scan_pub_.publish(laser_msg_);
    }

    if (raw_lidar_data_pub_.getNumSubscribers() > 0) {

        RawLidarData::Ptr ros_msg(new RawLidarData);

        ros_msg->scan_count  = header.scanId;
        ros_msg->time_start  = start_relative_time;
        ros_msg->time_end    = end_relative_time;
        ros_msg->angle_start = header.spindleAngleStart;
        ros_msg->angle_end   = header.spindleAngleEnd;

        ros_msg->distance.resize(header.pointCount);
        memcpy(&(ros_msg->distance[0]), 
               rangesP, 
               header.pointCount * sizeof(uint32_t));

        ros_msg->intensity.resize(header.pointCount);
        memcpy(&(ros_msg->intensity[0]), 
               intensitiesP, 
               header.pointCount * sizeof(uint32_t));

        raw_lidar_data_pub_.publish(ros_msg);
    }
}

void Laser::stop() 
{
    subscribers_ = 0;

    laser_diagnostics_.publish(SensorStatus::STOPPING);

    Status status = driver_->stopStreams(Source_Lidar_Scan);
    if (Status_Ok != status)
        ROS_ERROR("Failed to stop laser stream. Error code %d", status);
    else
        laser_diagnostics_.publish(SensorStatus::STOPPED);
}

void Laser::unsubscribe()
{
    boost::mutex::scoped_lock lock(sub_lock_);

    if (--subscribers_ > 0)
        return;

    stop();
}

void Laser::subscribe()
{
    boost::mutex::scoped_lock lock(sub_lock_);

    if (0 == subscribers_++) {

        laser_diagnostics_.publish(SensorStatus::STARTING);

        Status status = driver_->startStreams(Source_Lidar_Scan);
        if (Status_Ok == status)
            laser_diagnostics_.publish(SensorStatus::RUNNING);
        else
            ROS_ERROR("Failed to start laser. Error code %d", status);
    }
}
}
