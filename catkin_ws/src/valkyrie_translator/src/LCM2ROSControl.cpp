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
        lcm_->subscribe("VAL_TRANSLATOR_JOINT_COMMAND", &LCM2ROSControl::jointCommandHandler, this);
        
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
          buffer_current_positions[effortNames[i]] = 0.0;
          buffer_current_velocities[effortNames[i]] = 0.0;
          buffer_current_efforts[effortNames[i]] = 0.0;
          buffer_command_efforts[effortNames[i]] = 0.0;
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
      for(auto iter = effortJointHandles.begin(); iter != effortJointHandles.end(); iter++)
      {
          buffer_current_positions[iter->first] = iter->second.getPosition();
          buffer_current_velocities[iter->first] = iter->second.getVelocity();
          buffer_current_efforts[iter->first] = iter->second.getEffort();
      }
   }

   void LCM2ROSControl::update(const ros::Time& time, const ros::Duration& period)
   {
      lcm_->handleTimeout(0);

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
          if (fabs(buffer_command_efforts[iter->first]) < 1.){
            iter->second.setCommand(buffer_command_efforts[iter->first]);
          } else{
            ROS_INFO("Dangerous buffer_command_efforts[%s]: %f\n", iter->first.c_str(), buffer_command_efforts[iter->first]);
          }
          buffer_current_positions[iter->first] = iter->second.getPosition();
          buffer_current_velocities[iter->first] = iter->second.getVelocity();
          buffer_current_efforts[iter->first] = buffer_command_efforts[iter->first];

          lcm_pose_msg.joint_position[i] = buffer_current_positions[iter->first];
          lcm_pose_msg.joint_velocity[i] = buffer_current_velocities[iter->first];
          lcm_pose_msg.joint_effort[i] = buffer_current_efforts[iter->first];
          lcm_pose_msg.joint_name[i] = iter->first;

          i++;
      }   
      lcm_->publish("VAL_TRANSLATOR_JOINT_STATE", &lcm_pose_msg);

      // push out the measurements for all imus we see advertised
      for (auto iter = imuSensorHandles.begin(); iter != imuSensorHandles.end(); iter ++){
        pronto::force_torque_t lcm_imu_msg;
        //lcm_imu_msg.utime = utime;
        std::ostringstream imuchannel;
        imuchannel << "VAL_TRANSLATOR_IMU_" << iter->first;
        lcm_imu_msg.l_foot_force_z = iter->second.getOrientation()[0];
        lcm_imu_msg.l_foot_torque_x = iter->second.getOrientation()[1];
        lcm_imu_msg.l_foot_torque_y = iter->second.getOrientation()[2];
        lcm_imu_msg.r_foot_force_z = iter->second.getAngularVelocity()[0];
        lcm_imu_msg.r_foot_torque_x = iter->second.getAngularVelocity()[1];
        lcm_imu_msg.r_foot_torque_y = iter->second.getAngularVelocity()[2];
        lcm_imu_msg.l_hand_force[0] = iter->second.getLinearAcceleration()[0];
        lcm_imu_msg.l_hand_force[1] = iter->second.getLinearAcceleration()[1];
        lcm_imu_msg.l_hand_force[2] = iter->second.getLinearAcceleration()[2];
        lcm_->publish(imuchannel.str(), &lcm_imu_msg);
      }

      // push out the measurements for all ft's we see advertised
      pronto::six_axis_force_torque_array_t lcm_ft_array_msg;
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
      lcm_->publish("VAL_TRANSLATOR_FT_STATE", &lcm_ft_array_msg);
      
   }

   void LCM2ROSControl::stopping(const ros::Time& time)
   {}

   void LCM2ROSControl::jointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                           const pronto::joint_angles_t* msg)
   {
      ROS_INFO("Got new setpoints\n");
      for(unsigned int i = 0; i < msg->num_joints; ++i){
        ROS_INFO("Joint %s ", msg->joint_name[i].c_str());
        auto search = buffer_command_efforts.find(msg->joint_name[i]);
        if (search != buffer_command_efforts.end()) {
          ROS_INFO("found in keys, updating force to %f\n", msg->joint_position[i]);
          buffer_command_efforts[msg->joint_name[i]] = msg->joint_position[i];
        } else {
          ROS_INFO("had no match.");
        }
      }
   }
}
PLUGINLIB_EXPORT_CLASS(valkyrie_translator::LCM2ROSControl, controller_interface::ControllerBase)