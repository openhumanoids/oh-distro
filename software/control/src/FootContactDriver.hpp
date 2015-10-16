#include "drc/foot_contact_estimate_t.hpp"
#include "drake/QPCommon.h"

class FootContactDriver {
  private:
    BodyIdsCache m_body_ids;

  public:
    FootContactDriver(BodyIdsCache body_ids);
    void decode(const drc::foot_contact_estimate_t *msg, Eigen::Matrix<bool, Eigen::Dynamic, 1> &contact_force_detected);
};
