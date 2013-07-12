/**
 * @file laser.h
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

#ifndef MULTISENSE_ROS_LASER_H
#define MULTISENSE_ROS_LASER_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <multisense_ros/state_publisher.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <multisense_ros/JointDiagnostics.h>
#include <kdl/frames.hpp>

#include <MultiSenseChannel.hh>

namespace multisense_ros {

class Laser {
public:
    Laser(crl::multisense::Channel* driver,
          const std::string& robot_desc);
    ~Laser();

    void scanCallback(const crl::multisense::lidar::Header& header,
                      const crl::multisense::lidar::RangeType *rangesP,
                      const crl::multisense::lidar::IntensityType *intensitiesP);

    void pointCloudCallback(const crl::multisense::lidar::Header& header,
                            const crl::multisense::lidar::RangeType *rangesP,
                            const crl::multisense::lidar::IntensityType *intensitiesP);

    static const float EXPECTED_RATE;
    
private:

    //
    // Device stream control

    void subscribe();
    void unsubscribe();
    void stop();

    //
    // Calibration from sensor

    crl::multisense::lidar::Calibration lidar_cal_;
    KDL::Frame                          scan_pre_spindle_cal_;
    KDL::Frame                          scan_post_spindle_cal_;
    KDL::Frame                          pc_pre_spindle_cal_;
    KDL::Frame                          pc_post_spindle_cal_;

    //
    // Scan publishing

    crl::multisense::Channel*      driver_;
    ros::Publisher                 scan_pub_;
    std::string                    frame_id_;
    multisense_ros::StatePublisher laser_diagnostics_;

    //
    // Joint state publishing

    ros::Publisher          js_pub_;
    ros::Publisher          js_diagnostics_pub_;
    JointDiagnostics        js_diagnostics_;
    sensor_msgs::JointState js_msg_;    

    //
    // Raw data publishing

    ros::Publisher raw_lidar_data_pub_;
    ros::Publisher point_cloud_pub_;

    //
    // Keep around for efficiency

    sensor_msgs::LaserScan   laser_msg_;
    sensor_msgs::PointCloud2 point_cloud_;

    //
    // Subscriptions
    
    boost::mutex sub_lock_;
    int32_t      subscribers_;

}; // class

}// namespace


#endif
