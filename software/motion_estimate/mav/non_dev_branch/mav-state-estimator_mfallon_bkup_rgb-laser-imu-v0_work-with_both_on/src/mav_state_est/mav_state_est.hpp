#ifndef MAV_STATE_ESTIMATOR_HPP_
#define MAV_STATE_ESTIMATOR_HPP_

#include "rbis.hpp"
#include "update_history.hpp"

class MavStateEstimator {
public:
  MavStateEstimator(RBISResetUpdate * init_state, int64_t history_span);
  ~MavStateEstimator();

  updateHistory::historyMapIterator unprocessed_updates_start;
  updateHistory history;

  int64_t utime_history_span;

  void addUpdate(RBISUpdateInterface * update, bool roll_forward = true);
  void getHeadState(RBIS & head_state, RBIM & head_cov);
  double getMeasurementsLogLikelihood();
  void EKFSmoothBackwardsPass(double dt);

};

#endif /* MAV_STATE_ESTIMATOR_HPP_ */
