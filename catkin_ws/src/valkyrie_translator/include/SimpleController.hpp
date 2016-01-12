#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/joint_state_t.hpp"
#include "lcmtypes/drc/joint_angles_t.hpp"

namespace valkyrie_translator
{
   class SimpleController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
   {
   public:
        SimpleController();
        virtual ~SimpleController();

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n);
        void starting(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time& time);


        void jointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                           const drc::joint_angles_t* msg);

   private:
        boost::shared_ptr<lcm::LCM> lcm_;

        std::vector<hardware_interface::JointHandle> effortJointHandles;
        std::vector<double> buffer_command_effort;
        std::vector<double> buffer_current_positions;
        std::vector<double> buffer_current_velocities;
        std::vector<double> buffer_current_efforts;
        std::vector<double> latest_setpoints;
   };
}
#endif