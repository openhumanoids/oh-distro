#include <string>
#include "lcmtypes/drc/atlas_command_t.hpp"
#include "drake/QPCommon.h"

class AtlasCommandDriver {
  private:
    int m_num_joints; 
    RobotJointIndexMap joint_index_map;
    drc::atlas_command_t msg;

  public:
    AtlasCommandDriver(JointNames *input_joint_names);
    int dim(void) {
      return 3*m_num_joints;
    }
    void updateGains(AtlasHardwareGains *gains);

    drc::atlas_command_t* encode(double t, QPControllerOutput *qp_output);
    drc::atlas_command_t* encode(double t, QPControllerOutput *qp_output, AtlasHardwareGains *new_gains);
};
