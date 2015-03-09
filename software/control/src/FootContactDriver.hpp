#include "drc/foot_contact_estimate_t.hpp"
#include "drake/QPCommon.h"

class FootContactDriver {
  private:
    BodyIdsCache m_body_ids;

  public:
    FootContactDriver(BodyIdsCache body_ids);
    void decode(drc::foot_contact_estimate_t *msg, Ref<VectorXd> &contact_force_detected);
};
