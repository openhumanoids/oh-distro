/**
 * @file joint_diagnostics.cpp
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

#include <multisense_diagnostics/joint_diagnostics.h>

namespace multisense_diagnostics {

JointDiagnostics::JointDiagnostics()
    :rate_error_(0.0)
{
    ros::NodeHandle nh("stream_name");

    // diagnostics updater
    diagnostics_.setHardwareID("multisense");
    diagnostics_.add("Rate", boost::bind(&JointDiagnostics::rateCB, this, _1));
    diagnostics_.add("State", boost::bind(&JointDiagnostics::stateCB, this, _1));

    // subscribe to callbacks from joint
    joint_sub_ = nh.subscribe<multisense_ros::JointDiagnostics>("diagnostics", 100,
                                                                boost::bind(&JointDiagnostics::jointCB, this, _1));

    // set ros timer at 1 hz
    timer_ = nh.createTimer(ros::Duration(1.0), boost::bind(&JointDiagnostics::run, this, _1));
}

void JointDiagnostics::jointCB(const multisense_ros::JointDiagnosticsConstPtr& msg)
{
    joint_msg_ = msg;
    diagnostics_.force_update();
}

void JointDiagnostics::rateCB(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    // rate
    rate_error_ = 0.8 * rate_error_ + 0.2*fabs(joint_msg_->desired_rate/joint_msg_->current_rate - 1.0);

    stat.add("Expected rotation rate", joint_msg_->desired_rate);
    stat.add("Measured rotation rate", joint_msg_->current_rate);

    // global status
    if (rate_error_ > 0.2)
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Expected rate of sensor not reached");
    else if (rate_error_ > 0.1)
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Expected rate of sensor not reached");
    else
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Sensor is rotating at expected rate");
}

void JointDiagnostics::stateCB(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    // state
    stat.add("Joint name", joint_msg_->joint_state.name[0]);
    stat.add("Joint position", joint_msg_->joint_state.position[0]);
    stat.add("Joint velocity", joint_msg_->joint_state.velocity[0]);
    stat.add("Joint effort", joint_msg_->joint_state.effort[0]);

    // global status
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Sensor state");
}

void JointDiagnostics::run(const ros::TimerEvent& t)
{
    if (joint_msg_)
        diagnostics_.update();
}
} // multisense_diagnostics

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_diagnostics");

    multisense_diagnostics::JointDiagnostics joint;
    ros::spin();
}
