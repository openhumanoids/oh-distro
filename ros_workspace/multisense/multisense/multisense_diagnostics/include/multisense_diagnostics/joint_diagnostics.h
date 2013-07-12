/**
 * @file joint_diagnostics.h
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

#ifndef MULTISENSE_DIAGNOSTICS_JOINT_DIAGNOSTICS_H
#define MULTISENSE_DIAGNOSTICS_JOINT_DIAGNOSTICS_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <multisense_ros/JointDiagnostics.h>
#include <diagnostic_updater/publisher.h>

namespace multisense_diagnostics {

class JointDiagnostics
{
public:
    JointDiagnostics();

private:
    // joint state callback
    void jointCB(const multisense_ros::JointDiagnosticsConstPtr& msg);

    // fill out diagnostics message
    void stateCB(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void rateCB(diagnostic_updater::DiagnosticStatusWrapper &stat);

    // periodic diagnostics publishing
    void run(const ros::TimerEvent& t);

    ros::Subscriber joint_sub_;
    ros::Timer timer_;
    diagnostic_updater::Updater diagnostics_;
    multisense_ros::JointDiagnosticsConstPtr joint_msg_;
    double rate_error_;

}; // class

}// namespace


#endif
