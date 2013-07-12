/**
 * @file state_publisher.cpp
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

#include <multisense_ros/state_publisher.h>

namespace multisense_ros {
  
const static int scan_counter_size = 3;

StatePublisher::StatePublisher(ros::NodeHandle nh, const std::string& topic, float expected_rate)
    : scan_counter_index_(0), expected_rate_(expected_rate)
{
    // initialize scan counter
    scan_counter_.resize(scan_counter_size, 0);
    setCounter(expected_rate_);

    //create a latched publisher for diagnostics
    diagnostics_pub_ = nh.advertise<multisense_ros::SensorDiagnostics>(topic, 20, true);

    //set the expected rate for diagnostics
    diagnostics_.desired_rate = 0.0;
    diagnostics_.current_rate = 0.0;
    diagnostics_.status.state = diagnostics_.status.STOPPED;

    // periodic publishing
    timer_ = nh.createTimer(ros::Duration(1.0), boost::bind(&StatePublisher::publishScanRate, this, _1));
}

void StatePublisher::setCounter(int expected_rate)
{
    for (int i=0; i<scan_counter_size; i++)
        scan_counter_[i] = expected_rate;
}

void StatePublisher::setExpectedRate(float expected_rate)
{
    boost::mutex::scoped_lock lock(mutex_);
    expected_rate_ = expected_rate;

    if (diagnostics_.status.state == diagnostics_.status.RUNNING)
    {
        diagnostics_.current_rate = expected_rate_;
        diagnostics_.desired_rate = expected_rate_;
        setCounter(expected_rate_);
    }
}

void StatePublisher::publishScanRate(const ros::TimerEvent& t)
{
    boost::mutex::scoped_lock lock(mutex_);
    if (ros::Time::now() - diagnostics_.status.last_state_change > ros::Duration(2.0))
    {
        int count = 0;
        for (int i=0; i<scan_counter_size; i++)
            count += scan_counter_[i];
        diagnostics_.current_rate = (float)count/(float)scan_counter_size;
    }
    scan_counter_index_ = (scan_counter_index_+1)%scan_counter_size;
    scan_counter_[scan_counter_index_] = 0;

    diagnostics_.stamp = ros::Time::now();
    diagnostics_pub_.publish(diagnostics_);
}

void StatePublisher::publish(const uint8_t state)
{
    boost::mutex::scoped_lock lock(mutex_);

    ros::Time now = ros::Time::now();
    diagnostics_.status.state = state;
    diagnostics_.stamp = now;
    diagnostics_.status.last_state_change = now;
    if (state == diagnostics_.status.STOPPED)
    {
        diagnostics_.current_rate = 0.0;
        diagnostics_.desired_rate = 0.0;
        setCounter(0);
    }
    else if (state == diagnostics_.status.RUNNING)
    {
        diagnostics_.current_rate = expected_rate_;
        diagnostics_.desired_rate = expected_rate_;
        setCounter(expected_rate_);
    }
    diagnostics_pub_.publish(diagnostics_);
}

void StatePublisher::countStream()
{
    boost::mutex::scoped_lock lock(mutex_);
    scan_counter_[scan_counter_index_]++;
}
} // multisense_ros
