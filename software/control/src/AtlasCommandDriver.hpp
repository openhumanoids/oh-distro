#include <string>
#include "lcmtypes/bot_core/atlas_command_t.hpp"
#include "drake/systems/controllers/QPCommon.h"

class AtlasCommandDriver {
  private:
    int m_num_joints; 
    RobotJointIndexMap input_index_map;
    Eigen::VectorXi state_to_drake_input_map;
    bot_core::atlas_command_t msg;

  public:
    AtlasCommandDriver(JointNames *input_joint_names, std::vector<std::string> &state_coordinate_names);
    int dim(void) {
      return 3*m_num_joints;
    }
    void updateGains(AtlasHardwareGains *gains);

    bot_core::atlas_command_t* encode(double t, QPControllerOutput *qp_output, AtlasHardwareParams &params);
};
