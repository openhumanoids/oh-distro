#ifndef MULTISENSE_DIAGNOSTICS_STREAM_DIAGNOSTICS_H
#define MULTISENSE_DIAGNOSTICS_STREAM_DIAGNOSTICS_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <multisense_ros/SensorDiagnostics.h>
#include <diagnostic_updater/publisher.h>

namespace multisense_diagnostics
{

class StreamDiagnostics
{
public:
  StreamDiagnostics();

private:
  // stream state callback
  void streamCB(const multisense_ros::SensorDiagnosticsConstPtr& msg);

  // fill out diagnostics message
  void stateCB(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void rateCB(diagnostic_updater::DiagnosticStatusWrapper &stat);

  // periodic diagnostics publishing
  void run(const ros::TimerEvent& t);

  ros::Subscriber stream_sub_;
  ros::Timer timer_;
  diagnostic_updater::Updater diagnostics_;
  multisense_ros::SensorDiagnosticsConstPtr stream_msg_;

}; // class

}// namespace


#endif
