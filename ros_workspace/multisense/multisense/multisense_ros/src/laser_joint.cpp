#include <multisense_ros/laser_joint.h>
#include <angles/angles.h>


namespace multisense_ros
{

  LaserJoint::LaserJoint(multisense_driver::MultisenseDriver* driver)
    : driver_(driver)
  {
    // default message
    js_msg_.name.push_back("hokuyo_joint");
    js_msg_.position.push_back(0.0);
    js_msg_.velocity.push_back(0.0);
    js_msg_.effort.push_back(0.0);

    // Default LCM message:
    msg_out_.robot_name = "robot_name_holder"; // dont know if i can fill this
    msg_out_.origin_position.translation.x = 0;
    msg_out_.origin_position.translation.y = 0;
    msg_out_.origin_position.translation.z = 0;
    msg_out_.origin_position.rotation.x = 0;
    msg_out_.origin_position.rotation.y = 0;
    msg_out_.origin_position.rotation.z = 0;
    msg_out_.origin_position.rotation.w = 1;
 
    msg_out_.origin_twist.linear_velocity.x =0;
    msg_out_.origin_twist.linear_velocity.y =0;
    msg_out_.origin_twist.linear_velocity.z =0;
    msg_out_.origin_twist.angular_velocity.x =0;
    msg_out_.origin_twist.angular_velocity.y =0;
    msg_out_.origin_twist.angular_velocity.z =0;

    int i,j;
    for(i = 0; i < 6; i++)  {
      for(j = 0; j < 6; j++) {
        msg_out_.origin_cov.position_cov[i][j] = 0;
        msg_out_.origin_cov.twist_cov[i][j] = 0;
      }
    }
    msg_out_.num_joints = 1;

    drc::joint_covariance_t j_cov;
    j_cov.variance = 0;
    msg_out_.joint_name.push_back("hokuyo_joint");
    msg_out_.joint_position.push_back(0);
    msg_out_.joint_velocity.push_back(0);
    msg_out_.measured_effort.push_back(0);
    msg_out_.joint_cov.push_back( j_cov );

    // dummy ground contact states
    msg_out_.contacts.num_contacts =8;
    msg_out_.contacts.id.push_back("LFootToeIn");
    msg_out_.contacts.id.push_back("LFootToeOut");
    msg_out_.contacts.id.push_back("LFootHeelIn");
    msg_out_.contacts.id.push_back("LFootHeelOut");
    msg_out_.contacts.id.push_back("RFootToeIn");
    msg_out_.contacts.id.push_back("RFootToeOut");
    msg_out_.contacts.id.push_back("RFootHeelIn");
    msg_out_.contacts.id.push_back("RFootHeelOut");
    for (int i=0; i< msg_out_.contacts.num_contacts; i++){
      msg_out_.contacts.inContact.push_back(0);
      drc::vector_3d_t f_zero;
      f_zero.x = 0;f_zero.y = 0;f_zero.z = 0;
      msg_out_.contacts.contactForce.push_back(f_zero);
    }

    if(!lcm_publish_.good()){
      std::cerr <<"ERROR: lcm is not good()" <<std::endl;
    }

    // create publishers
    ros::NodeHandle nh("laser_joint");
    js_pub_  = nh.advertise<sensor_msgs::JointState>("/joint_states", 40);

    //latched publisher for diagnostics
    diagnostics_pub_ = nh.advertise<JointDiagnostics>("diagnostics", 40, true);
    diagnostics_.joint_state = js_msg_;
    diagnostics_pub_.publish(diagnostics_);

    // subscribe to driver callbacks
    scan_sub_ = driver->subscribe<LidarDataMessage>(boost::bind(&LaserJoint::scanCB, this, _1));

    // action server
    as_.reset(new SpindleAS(nh, "spindle_control", false));
    as_->registerGoalCallback(boost::bind(&LaserJoint::spindleControlGoal, this));
    as_->start();
  };


  void LaserJoint::spindleControlGoal()
  {
    LidarSetMotorMessage cmd;
    {
      boost::mutex::scoped_lock lock(as_mutex_);
      spindle_goal_ = as_->acceptNewGoal();

      cmd.rpm = spindle_goal_->rotational_speed * 60.0 / (2.0 * M_PI);
    }

    // command motor
    spindle_commander_.reset(new SpindleCommand(cmd, driver_, boost::bind(&LaserJoint::spindleControlResult, this, _1)));
    spindle_commander_->run();

  }


  void LaserJoint::spindleControlResult(const boost::shared_ptr<const LidarSetMotorAckMessage>& res)
  {
    boost::mutex::scoped_lock lock(as_mutex_);

    if (as_->isActive())
    {
      as_->setSucceeded();

      //update desired rate and publish diagnostics
      diagnostics_.stamp = ros::Time::now();
      diagnostics_.desired_rate = spindle_goal_->rotational_speed;
      diagnostics_.joint_state = js_msg_;
      diagnostics_pub_.publish(diagnostics_);
    }
  }

  void LaserJoint::scanCB(const boost::shared_ptr<const LidarDataMessage>& msg)
  {
    ROS_DEBUG("Publish joint state");

    float offset = 0.5;
    float angle_start = (float)msg->angleStart / 1000000.0 - offset;
    float angle_end = (float)msg->angleEnd / 1000000.0 - offset;

    ros::Time start_absolute_time = convertTime(msg->timeStart.getCurrentTime());
    ros::Time start_relative_time = convertTime(msg->timeStart);
    ros::Time end_relative_time = convertTime(msg->timeEnd);

    float angle_diff = angles::shortest_angular_distance(angle_end, angle_start);
    float velocity = angle_diff / (end_relative_time - start_relative_time).toSec();
    //printf("Angles: %f  %f --> %f\n", angle_start, angle_end, angle_diff);
    //printf("Time: %f  %f --> %f\n", start_relative_time.toSec(), end_relative_time.toSec(), (end_relative_time-start_relative_time).toSec());



    // publish joint state for beginning of scan
    js_msg_.header.frame_id = "";
    js_msg_.header.stamp = start_absolute_time;
    js_msg_.position[0] = angle_start;
    js_msg_.velocity[0] = velocity;
    js_pub_.publish(js_msg_);

    // LCM:
    msg_out_.utime = (int64_t) start_absolute_time.toNSec()/1000; // from nsec to usec
    msg_out_.joint_position[0]= angle_start;
    msg_out_.joint_velocity[0]= velocity;
    lcm_publish_.publish("TRUE_ROBOT_STATE", &msg_out_);

    // publish joint state for end of scan
    js_msg_.header.frame_id = "";
    ros::Time end_absolute_time = start_absolute_time + (end_relative_time - start_relative_time);
    js_msg_.header.stamp = end_absolute_time;
    js_msg_.position[0] = angle_end;
    js_msg_.velocity[0] = velocity;
    js_pub_.publish(js_msg_);

    // LCM:
    msg_out_.utime = (int64_t) end_absolute_time.toNSec()/1000; // from nsec to usec
    msg_out_.joint_position[0]= angle_end;
    msg_out_.joint_velocity[0]= velocity;
    lcm_publish_.publish("TRUE_ROBOT_STATE", &msg_out_);

    diagnostics_.stamp = ros::Time::now();
    diagnostics_.joint_state = js_msg_;
    diagnostics_.current_rate = velocity;

    // Downsample diagnostics to 1 Hz
    if (msg->scanCount % 40 == 0)
      diagnostics_pub_.publish(diagnostics_);
  }
}
