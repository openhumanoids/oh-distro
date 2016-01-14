/**

Plugin to feed core state and commands to and from the Valkyrie ros_control API.
Listens for torque commands, and feeds them to the robot.
Forwards IMU, force/torque, and joint state over LCM in appropriate status messages.

Runs at 500hz in the Valkyrie ros_control main loop as a plugin.

Significant reference to 
https://github.com/NASA-JSC-Robotics/valkyrie/wiki/Running-Controllers-on-Valkyrie

gizatt@mit.edu, 201601**

**/

#include <LCM2ROSControl.hpp>

namespace valkyrie_translator
{
   LCM2ROSControl::LCM2ROSControl()
   {
   }

   LCM2ROSControl::~LCM2ROSControl()
   {}

    bool LCM2ROSControl::initRequest(hardware_interface::RobotHW* robot_hw, 
                             ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh, 
                             std::set<std::string>& claimed_resources)
    {
        // check if construction finished cleanly
        if (state_ != CONSTRUCTED){
          ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
          return false;
        }

        // setup LCM (todo: move to constructor? how to propagate an error then?)
        lcm_ = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
        if (!lcm_->good())
        {
          std::cerr << "ERROR: lcm is not good()" << std::endl;
          return false;
        }
        lcm_->subscribe("NASA_COMMAND", &LCM2ROSControl::jointCommandHandler, this);
        
        // get a pointer to the effort interface
        hardware_interface::EffortJointInterface* effort_hw = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (!effort_hw)
        {
          ROS_ERROR("This controller requires a hardware interface of type hardware_interface::EffortJointInterface.");
          return false;
        }

        // return which resources are claimed by this controller
        effort_hw->clearClaims();
        const std::vector<std::string>& effortNames = effort_hw->getNames();
        for(unsigned int i=0; i<effortNames.size(); i++)
        {
          effortJointHandles[effortNames[i]] = effort_hw->getHandle(effortNames[i]);
          latest_commands[effortNames[i]] = drc::joint_command_t();
          latest_commands[effortNames[i]].joint_name = effortNames[i];
          latest_commands[effortNames[i]].position = 0.0;
          latest_commands[effortNames[i]].velocity = 0.0;
          latest_commands[effortNames[i]].effort = 0.0;
          // set gains to zero as well? 
        }

        auto effort_hw_claims = effort_hw->getClaims();
        claimed_resources.insert(effort_hw_claims.begin(), effort_hw_claims.end());
        effort_hw->clearClaims();

        // get a pointer to the imu interface
        hardware_interface::ImuSensorInterface* imu_hw = robot_hw->get<hardware_interface::ImuSensorInterface>();
        if (!imu_hw)
        {
          ROS_ERROR("This controller requires a hardware interface of type hardware_interface::ImuSensorInterface.");
          return false;
        }

        // return which resources are claimed by this controller
        imu_hw->clearClaims();
        const std::vector<std::string>& imuNames = imu_hw->getNames();
        for(unsigned int i=0; i<imuNames.size(); i++)
        {
             imuSensorHandles[imuNames[i]] = imu_hw->getHandle(imuNames[i]);
        }

        auto imu_hw_claims = imu_hw->getClaims();
        claimed_resources.insert(imu_hw_claims.begin(), imu_hw_claims.end());
        imu_hw->clearClaims();

        // get a pointer to the effort interface
        hardware_interface::ForceTorqueSensorInterface* forceTorque_hw = robot_hw->get<hardware_interface::ForceTorqueSensorInterface>();
        if (!forceTorque_hw)
        {
          ROS_ERROR("This controller requires a hardware interface of type hardware_interface::EffortJointInterface.");
          return false;
        }

        // return which resources are claimed by this controller
        forceTorque_hw->clearClaims();
        const std::vector<std::string>& forceTorqueNames = forceTorque_hw->getNames();
        for(unsigned int i=0; i<forceTorqueNames.size(); i++)
        {
             forceTorqueHandles[forceTorqueNames[i]] = forceTorque_hw->getHandle(forceTorqueNames[i]);
        }
        auto forceTorque_hw_claims = forceTorque_hw->getClaims();
        claimed_resources.insert(forceTorque_hw_claims.begin(), forceTorque_hw_claims.end());
        forceTorque_hw->clearClaims();

        // success
        state_ = INITIALIZED;
        ROS_INFO("LCM2ROSCONTROL ON\n");
        return true;
    }

   void LCM2ROSControl::starting(const ros::Time& time)
   {
      last_update = time;
   }

   void LCM2ROSControl::update(const ros::Time& time, const ros::Duration& period)
   {
      lcm_->handleTimeout(0);

      double dt = (time - last_update).toSec();
      last_update = time;
      int64_t utime = time.toSec() * 100000.;

      // push out the joint states for all joints we see advertised
      pronto::joint_state_t lcm_pose_msg;
      lcm_pose_msg.utime = utime;
      lcm_pose_msg.num_joints = effortJointHandles.size();
      lcm_pose_msg.joint_name.assign(effortJointHandles.size(), "");
      lcm_pose_msg.joint_position.assign(effortJointHandles.size(), 0.);
      lcm_pose_msg.joint_velocity.assign(effortJointHandles.size(), 0.);
      lcm_pose_msg.joint_effort.assign(effortJointHandles.size(), 0.);
      int i = 0;
      for(auto iter = effortJointHandles.begin(); iter != effortJointHandles.end(); iter++)
      {
          // see drc_joint_command_t.lcm for explanation of gains and
          // force calculation.
          double q = iter->second.getPosition();
          double qd = iter->second.getVelocity();
          double f = iter->second.getEffort();

          drc::joint_command_t command = latest_commands[iter->first];
          double command_effort = 
            command.k_q_p * ( command.position - q ) + 
            command.k_q_i * ( command.position - q ) * dt + 
            command.k_qd_p * ( command.velocity - qd) + 
            command.k_f_p * ( command.effort - f) + 
            command.ff_qd * ( qd ) + 
            command.ff_qd_d * ( command.velocity ) + 
            command.ff_f_d * ( command.effort ) + 
            command.ff_const;

          if (fabs(command_effort) < 100.){
            iter->second.setCommand(command_effort);
          } else{
            ROS_INFO("Dangerous latest_commands[%s]: %f\n", iter->first.c_str(), command_effort);
            iter->second.setCommand(0.0);
          }

          lcm_pose_msg.joint_position[i] = q;
          lcm_pose_msg.joint_velocity[i] = qd;
          // TODO: is this right? or should I just assign f?
          lcm_pose_msg.joint_effort[i] = iter->second.getEffort(); // is this commanded or measured?

          i++;
      }   
      lcm_->publish("NASA_STATE", &lcm_pose_msg);

      // push out the measurements for all imus we see advertised
      for (auto iter = imuSensorHandles.begin(); iter != imuSensorHandles.end(); iter ++){
        mav::ins_t lcm_imu_msg;
        //lcm_imu_msg.utime = utime;
        std::ostringstream imuchannel;
        imuchannel << "NASA_INS_" << iter->first;
        lcm_imu_msg.utime = utime;
        for (i=0; i<3; i++){
          lcm_imu_msg.quat[i]= iter->second.getOrientation()[i];
          lcm_imu_msg.gyro[i] = iter->second.getAngularVelocity()[i];
          lcm_imu_msg.accel[i] = iter->second.getLinearAcceleration()[i];
          lcm_imu_msg.mag[i] = 0.0;
        }
        lcm_imu_msg.quat[3] = iter->second.getOrientation()[3];
        lcm_imu_msg.pressure = 0.0;
        lcm_imu_msg.rel_alt = 0.0;

        lcm_->publish(imuchannel.str(), &lcm_imu_msg);
      }

      // push out the measurements for all ft's we see advertised
      drc::six_axis_force_torque_array_t lcm_ft_array_msg;
      lcm_ft_array_msg.utime = utime;
      lcm_ft_array_msg.num_sensors = forceTorqueHandles.size();
      lcm_ft_array_msg.names.resize(forceTorqueHandles.size());
      lcm_ft_array_msg.sensors.resize(forceTorqueHandles.size());
      i = 0;
      for (auto iter = forceTorqueHandles.begin(); iter != forceTorqueHandles.end(); iter ++){

        lcm_ft_array_msg.sensors[i].utime = utime;
        lcm_ft_array_msg.sensors[i].force[0] = iter->second.getForce()[0];
        lcm_ft_array_msg.sensors[i].force[1] = iter->second.getForce()[1];
        lcm_ft_array_msg.sensors[i].force[2] = iter->second.getForce()[2];
        lcm_ft_array_msg.sensors[i].moment[0] = iter->second.getTorque()[0];
        lcm_ft_array_msg.sensors[i].moment[1] = iter->second.getTorque()[1];
        lcm_ft_array_msg.sensors[i].moment[2] = iter->second.getTorque()[2];

        lcm_ft_array_msg.names[i] = iter->first;
        i++;
      }
      lcm_->publish("NASA_FORCE_TORQUE", &lcm_ft_array_msg);
      
   }

   void LCM2ROSControl::stopping(const ros::Time& time)
   {}

   void LCM2ROSControl::jointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                           const drc::robot_command_t* msg)
   {
      ROS_INFO("Got new setpoints\n");
      
      // TODO: zero non-mentioned joints for safety? 
      
      for(unsigned int i = 0; i < msg->num_joints; ++i){
        ROS_INFO("Joint %s ", msg->joint_commands[i].joint_name.c_str());
        auto search = latest_commands.find(msg->joint_commands[i].joint_name);
        if (search != latest_commands.end()) {
          ROS_INFO("found in keys");
          latest_commands[msg->joint_commands[i].joint_name] = msg->joint_commands[i];
        } else {
          ROS_INFO("had no match.");
        }
      }
   }
}
PLUGINLIB_EXPORT_CLASS(valkyrie_translator::LCM2ROSControl, controller_interface::ControllerBase)