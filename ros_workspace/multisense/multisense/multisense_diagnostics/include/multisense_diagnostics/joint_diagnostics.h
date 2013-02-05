#ifndef MULTISENSE_DIAGNOSTICS_JOINT_DIAGNOSTICS_H
#define MULTISENSE_DIAGNOSTICS_JOINT_DIAGNOSTICS_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <multisense_ros/JointDiagnostics.h>
#include <diagnostic_updater/publisher.h>

namespace multisense_diagnostics
{

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

}; // class

}// namespace


#endif
