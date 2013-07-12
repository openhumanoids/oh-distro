/**
 * @file stream_diagnostics.cpp
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

#include <multisense_diagnostics/stream_diagnostics.h>

namespace multisense_diagnostics {

StreamDiagnostics::StreamDiagnostics()
{
    ros::NodeHandle nh("stream_name");

    // diagnostics updater
    diagnostics_.setHardwareID("multisense");
    diagnostics_.add("Rate", boost::bind(&StreamDiagnostics::rateCB, this, _1));
    diagnostics_.add("State", boost::bind(&StreamDiagnostics::stateCB, this, _1));

    // subscribe to callbacks from stream
    stream_sub_ = nh.subscribe<multisense_ros::SensorDiagnostics>("diagnostics", 100,
                                                                  boost::bind(&StreamDiagnostics::streamCB, this, _1));

    // set ros timer at 1 hz
    timer_ = nh.createTimer(ros::Duration(1.0), boost::bind(&StreamDiagnostics::run, this, _1));
}

void StreamDiagnostics::streamCB(const multisense_ros::SensorDiagnosticsConstPtr& msg)
{
    stream_msg_ = msg;
    diagnostics_.force_update();
}

void StreamDiagnostics::rateCB(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    // rate
    double rate_error = fabs(stream_msg_->desired_rate/stream_msg_->current_rate - 1.0);

    stat.add("Expected streaming rate", stream_msg_->desired_rate);
    stat.add("Measured streaming rate", stream_msg_->current_rate);

    // global status
    if (rate_error > 0.2)
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Expected rate of sensor not reached");
    else if (rate_error > 0.1)
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Expected rate of sensor not reached");
    else
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Sensor is streaming at expected rate");
}

void StreamDiagnostics::stateCB(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    // timeout
    ros::Duration delay = ros::Time::now() - stream_msg_->status.last_state_change;

    // stream stream status
    if (stream_msg_->status.state == stream_msg_->status.RUNNING){
        stat.add("StreamDiagnostics StreamDiagnostics", "Running");
        delay = ros::Duration();
    }
    else if (stream_msg_->status.state == stream_msg_->status.STOPPED){
        stat.add("StreamDiagnostics StreamDiagnostics", "Stopped");
        delay = ros::Duration();
    }
    else if (stream_msg_->status.state == stream_msg_->status.STARTING)
        stat.add("StreamDiagnostics StreamDiagnostics", "Starting");
    else if (stream_msg_->status.state == stream_msg_->status.STOPPING)
        stat.add("StreamDiagnostics StreamDiagnostics", "Stopping");
    stat.add("Last state change to stream stream at time", delay.toSec());

    // global status
    if (delay > ros::Duration(5.0))
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "We've been waiting for more than 5 seconds for a state change");
    else if (delay > ros::Duration(2.5))
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "We've been waiting for more than 2.5 seconds for a state change");
    else
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "StreamDiagnostics status ok");
}

void StreamDiagnostics::run(const ros::TimerEvent& t)
{
    if (stream_msg_)
        diagnostics_.update();
}
} // multisense_diagnostics


int main(int argc, char** argv)
{
    ros::init(argc, argv, "stream_diagnostics");

    multisense_diagnostics::StreamDiagnostics stream;
    ros::spin();
}
