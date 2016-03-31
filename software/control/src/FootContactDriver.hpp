#include "drc/foot_contact_estimate_t.hpp"
#include "drake/systems/controllers/QPCommon.h"

class FootContactDriver {
  private:
    int r_foot_id;
    int l_foot_id;

  public:
    FootContactDriver(RobotPropertyCache rpc);
    void decode(const drc::foot_contact_estimate_t *msg, Eigen::Matrix<bool, Eigen::Dynamic, 1> &contact_force_detected);
};
