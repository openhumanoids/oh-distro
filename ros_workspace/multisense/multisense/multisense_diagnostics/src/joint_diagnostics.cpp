#include <multisense_diagnostics/joint_diagnostics.h>


namespace multisense_diagnostics
{

  JointDiagnostics::JointDiagnostics()
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
    double rate_error = fabs(joint_msg_->desired_rate/joint_msg_->current_rate - 1.0);

    stat.add("Expected jointing rate", joint_msg_->desired_rate);
    stat.add("Measured jointing rate", joint_msg_->current_rate);

    // global status
    if (rate_error > 0.2)
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Expected rate of sensor not reached");
    else if (rate_error > 0.1)
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Expected rate of sensor not reached");
    else
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Sensor is jointing at expected rate");
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
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_diagnostics");

  multisense_diagnostics::JointDiagnostics joint;
  ros::spin();
}
