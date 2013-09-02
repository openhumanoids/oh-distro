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
#include <kdl/frames.hpp>

#include <MultiSenseChannel.hh>

#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/multisense.hpp>
#include <lcm/lcm-cpp.hpp>

namespace multisense_ros {

class Laser {
public:
    Laser(crl::multisense::Channel* driver,
          const std::string& robot_desc);
    ~Laser();

    void scanCallback(const crl::multisense::lidar::Header& header,
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

    
    // LCM stuff:
    lcm::LCM lcm_publish_ ;
    bot_core::planar_lidar_t lcm_laser_msg_;    
    multisense::state_t lcm_state_;
    void publishLCMTransforms(int64_t utime_out, int32_t spindleAngle);

    // Subscriptions
    
    boost::mutex sub_lock_;

}; // class

}// namespace


#endif
