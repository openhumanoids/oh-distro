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

#include "laser.h"
#include "angles.h"
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
        printf("Error extracting post-spindle chain from KDL tree");
        return false;
    }

    if (chain.getNrOfJoints() != 0) {
        printf("Expected 0 joints in chain. Got %u\n", 
                  chain.getNrOfJoints());
        return false;
    }

    KDL::JntArray joint_pos(0);
    KDL::ChainFkSolverPos_recursive fksolver(chain);

    KDL::Frame scan_post_spindle_cal_T_hokuyo_head;
    if(fksolver.JntToCart(joint_pos, scan_post_spindle_cal_T_hokuyo_head) < 0) {
        printf("Error in FK for post-spindle calcs");
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
        printf("Error extracting head-spindle chain from KDL tree");
        return false;
    }

    KDL::Chain head_cam_chain;
    if (!tree.getChain("head", "left_camera_optical_frame", head_cam_chain)) {
        printf("Error extracting head-camera chain from KDL tree");
        return false;
    }

    if (head_cam_chain.getNrOfJoints() != 0) {
        printf("Expected 0 joints in head-camera chain. Got %u\n", 
                  head_cam_chain.getNrOfJoints());
        return false;
    }

    if (head_spindle_chain.getNrOfJoints() != 0) {
        printf("Expected 0 joints in head-spindle chain. Got %u\n", 
                  head_spindle_chain.getNrOfJoints());
        return false;
    }

    KDL::JntArray joint_pos(0);
    KDL::ChainFkSolverPos_recursive head_spindle_solver(head_spindle_chain);
    KDL::ChainFkSolverPos_recursive head_cam_solver(head_cam_chain);
    KDL::Frame head_T_pre_spindle;
    KDL::Frame head_T_cam_optical;
    if(head_spindle_solver.JntToCart(joint_pos, head_T_pre_spindle) < 0) {
        printf("Error in FK for head_spindle calcs");
        return false;
    }

    if(head_cam_solver.JntToCart(joint_pos, head_T_cam_optical) < 0) {
        printf("Error in FK for head_cam calcs");
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
        printf("Error parsing robot description using kdl_parser");
        return false;
    }

    bool success;
    success = computePostSpindleCal(tree, spindle_T_laser, post_spindle_cal);
    if (!success) {
        printf("Error calculating post spindle calibration");
        return false;
    }

    success = computePreSpindleCal(tree, camera_T_spindle_fixed, pre_spindle_cal);
    if (!success) {
        printf("Error calculating pre spindle calibration");
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


void makeCRLFrame(KDL::Frame frame_in, lidar::Calibration& c)
{
    for (int i=0; i < 3; i++) {
        for (int j=0; j < 3; j++)
            c.cameraToSpindleFixed[i][j] = frame_in.M(i,j);
    }
    c.cameraToSpindleFixed[0][3] = frame_in.p[0];
    c.cameraToSpindleFixed[1][3] = frame_in.p[1];
    c.cameraToSpindleFixed[2][3] = frame_in.p[2];

}

void getLaserCal(lidar::Calibration& c,
                 const std::string&  robot_desc,
                 KDL::Frame&         pre_spindle_cal,
                 KDL::Frame&         post_spindle_cal)
{   
    KDL::Frame spindle_T_laser = makeFrame(c.laserToSpindle).Inverse();
    KDL::Frame camera_T_spindle_fixed = makeFrame(c.cameraToSpindleFixed);

    bool success;
    success = true;//computeCal(robot_desc,
                     //    spindle_T_laser, camera_T_spindle_fixed,
                       //  pre_spindle_cal, post_spindle_cal);
    if (!success)
        printf("Error computing lidar calibration.");
}

void pushCal_LCM(multisense::state_t& msg, 
	     const std::string&       name, 
	     double                   pos)
{
    msg.joint_name.push_back(name);
    msg.joint_position.push_back(pos);
    msg.joint_velocity.push_back(0.0);
    msg.joint_effort.push_back(0.0);
}

}; // anonymous

namespace multisense_ros {

const float Laser::EXPECTED_RATE = 40.0;

namespace { // anonymous

//
// Shims for c-style driver callbacks

void lCB(const lidar::Header&        header,
	 void*                       userDataP)
{
    reinterpret_cast<Laser*>(userDataP)->scanCallback(header);
}


}; // anonymous

Laser::Laser(Channel* driver,
             const std::string& robot_desc)
    : driver_(driver)
{

    //
    // Stop lidar stream

    stop();

    //
    // Query calibration from sensor
    Status cal_status = driver->getLidarCalibration(lidar_cal_);

    // Manual addjustment of calibration by mfallon: (july 2013):
    // KDL::Frame cal = makeFrame(lidar_cal_.cameraToSpindleFixed);
    //// + further down
    //// + further left
    //// + further forward
    // KDL::Frame mfallon_adjust = KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector( -0.06, 0.0, -0.008));  
    // cal = cal*mfallon_adjust;
    // makeCRLFrame(cal, lidar_cal_);

    if (Status_Ok != cal_status)
        printf("could not query lidar calibration, using URDF defaults");
    else {

        //
        // Calibration for laser scan topic

        getLaserCal(lidar_cal_, robot_desc, scan_pre_spindle_cal_, scan_post_spindle_cal_);

        //
        // Calibration for point cloud topic

        pc_post_spindle_cal_  = makeFrame(lidar_cal_.laserToSpindle);
        pc_pre_spindle_cal_ = makeFrame(lidar_cal_.cameraToSpindleFixed);        
    }

    subscribe();    

    // Register callbacks, driver creates dedicated background thread for each

    driver_->addIsolatedCallback(lCB, this);

    // Set LCM components:
    if(!lcm_publish_.good()){
      std::cerr <<"ERROR: lcm is not good()" <<std::endl;
      printf("ERROR: lcm is not good()");
    }    

    lcm_state_.joint_name.push_back("hokuyo_joint");
    lcm_state_.joint_position.push_back(0.0);
    lcm_state_.joint_velocity.push_back(0.0);
    lcm_state_.joint_effort.push_back(0.0);

    double roll, pitch, yaw;
    scan_pre_spindle_cal_.M.GetRPY(roll, pitch, yaw);
    pushCal_LCM(lcm_state_, "pre_spindle_cal_x_joint", scan_pre_spindle_cal_.p[0]);
    pushCal_LCM(lcm_state_, "pre_spindle_cal_y_joint", scan_pre_spindle_cal_.p[1]);
    pushCal_LCM(lcm_state_, "pre_spindle_cal_z_joint", scan_pre_spindle_cal_.p[2]);
    pushCal_LCM(lcm_state_, "pre_spindle_cal_roll_joint",  roll);
    pushCal_LCM(lcm_state_, "pre_spindle_cal_pitch_joint", pitch);
    pushCal_LCM(lcm_state_, "pre_spindle_cal_yaw_joint",   yaw);

    scan_post_spindle_cal_.M.GetRPY(roll, pitch, yaw);
    pushCal_LCM(lcm_state_, "post_spindle_cal_x_joint", scan_post_spindle_cal_.p[0]);
    pushCal_LCM(lcm_state_, "post_spindle_cal_y_joint", scan_post_spindle_cal_.p[1]);
    pushCal_LCM(lcm_state_, "post_spindle_cal_z_joint", scan_post_spindle_cal_.p[2]);
    pushCal_LCM(lcm_state_, "post_spindle_cal_roll_joint",  roll);
    pushCal_LCM(lcm_state_, "post_spindle_cal_pitch_joint", pitch);
    pushCal_LCM(lcm_state_, "post_spindle_cal_yaw_joint",   yaw);
    lcm_state_.num_joints = lcm_state_.joint_position.size();
}

Laser::~Laser()
{
    stop();
}




void Laser::publishLCMTransforms(int64_t utime_out, int32_t spindleAngle){

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
    
    bot_core::rigid_transform_t pre_to_post_frame;
    pre_to_post_frame.utime = utime_out;
    pre_to_post_frame.trans[0] = T_spindleFromMotor.p[0];
    pre_to_post_frame.trans[1] = T_spindleFromMotor.p[1];
    pre_to_post_frame.trans[2] = T_spindleFromMotor.p[2];
    T_spindleFromMotor.M.GetQuaternion(pre_to_post_frame.quat[1], pre_to_post_frame.quat[2], pre_to_post_frame.quat[3], pre_to_post_frame.quat[0]);
    lcm_publish_.publish("PRE_SPINDLE_TO_POST_SPINDLE", &pre_to_post_frame);
    

}

void Laser::scanCallback(const lidar::Header& header)
{

    //
    // Get out if we have no work to do 

//    if (!(0 == (header.scanId % 40)))
  //      return;

    //
    // The URDF assumes that the laser straight up at a joint angle of 0 degrees

    const float offset = M_PI;

    int64_t start_absolute_utime = header.timeStartSeconds*1E6 + header.timeStartMicroSeconds;
    int64_t end_absolute_utime = header.timeEndSeconds*1E6 + header.timeEndMicroSeconds;
    double scan_time = (end_absolute_utime - start_absolute_utime)*1E-6; // in sec

    const float angle_start = 1e-6 * static_cast<float>(header.spindleAngleStart) - offset;
    const float angle_end   = 1e-6 * static_cast<float>(header.spindleAngleEnd) - offset;
    const float angle_diff  = angles::shortest_angular_distance(angle_start, angle_end);
    const float velocity    = angle_diff / scan_time;

    //
    // Publish joint state for beginning of scan

    int64_t utime_start =(int64_t) start_absolute_utime;
    lcm_state_.utime = utime_start;
    lcm_state_.joint_position[0]= angle_start;
    lcm_state_.joint_velocity[0]= velocity;
    lcm_publish_.publish("MULTISENSE_STATE", &lcm_state_);    
    publishLCMTransforms(utime_start, header.spindleAngleStart);
    
    //
    // Publish joint state for end of scan
    
    int64_t utime_end = (int64_t) end_absolute_utime;
    lcm_state_.utime = utime_end;
    lcm_state_.joint_position[0]= angle_end;
    lcm_state_.joint_velocity[0]= velocity;
    lcm_publish_.publish("MULTISENSE_STATE", &lcm_state_);        
    publishLCMTransforms(utime_end, header.spindleAngleEnd);

    
    const double arcRadians = 1e-6 * static_cast<double>(header.scanArc);
      
    // LCM:
    lcm_laser_msg_.utime = (int64_t) end_absolute_utime; // was start_absolute_utime until april 2014
    lcm_laser_msg_.ranges.resize( header.pointCount );
    lcm_laser_msg_.intensities.resize( header.pointCount );
    for (size_t i=0; i < header.pointCount; i++){
      lcm_laser_msg_.ranges[i]      = static_cast<float>(header.rangesP[i]) / 1000.0f; // from millimeters
      lcm_laser_msg_.intensities[i] = static_cast<float>(header.intensitiesP[i]);      // in device units
    }
    lcm_laser_msg_.nranges = header.pointCount;
    lcm_laser_msg_.nintensities=header.pointCount;
    lcm_laser_msg_.rad0 = -arcRadians / 2.0;
    lcm_laser_msg_.radstep = arcRadians / (header.pointCount - 1);
    lcm_publish_.publish("SCAN", &lcm_laser_msg_);
    
    /////////////////////////////////////////////////////////////      
}

void Laser::stop() 
{

    //laser_diagnostics_.publish(SensorStatus::STOPPING);

    Status status = driver_->stopStreams(Source_Lidar_Scan);
    if (Status_Ok != status)
        printf("Failed to stop laser stream. Error code %d", status);
    //else
    //    laser_diagnostics_.publish(SensorStatus::STOPPED);
}

void Laser::unsubscribe()
{
    stop();
}

void Laser::subscribe()
{

        //laser_diagnostics_.publish(SensorStatus::STARTING);

        Status status = driver_->startStreams(Source_Lidar_Scan);
        if (Status_Ok == status){
            //laser_diagnostics_.publish(SensorStatus::RUNNING);
        }else{
            printf("Failed to start laser. Error code %d", status);
        }

}
}
