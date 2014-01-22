#include "noise_id.hpp"

using namespace std;
using namespace Eigen;
using namespace eigen_utils;

namespace MavStateEst {

void sampleProcessForward(const std::list<RBIS> & truth_state_history, const RBIMList & truth_cov_history, double dt,
    double q_gyro, double q_accel, int N_window, std::list<RBIS> & state_errors, RBIMList & covs,
    std::list<RBIS> & rolled_states, RBIMList & rolled_covs)
{
  double q_gyro_bias = 0;
  double q_accel_bias = 0;

  std::list<RBIS>::const_iterator truth_state_it = truth_state_history.begin();
  std::list<RBIM>::const_iterator truth_cov_it = truth_cov_history.begin();
  while (true) {
    RBIS rolled_state = *truth_state_it;
//    RBIM rolled_covariance = RBIM::Zero();
    RBIM start_window_cov = *truth_cov_it;
    RBIM rolled_covariance = start_window_cov;
    for (int ii = 0; ii < N_window; ii++) {
      insUpdateCovariance(q_gyro, q_accel, q_gyro_bias, q_accel_bias, rolled_state, rolled_covariance, dt);
      insUpdateCovariance(0, 0, 0, 0, rolled_state, start_window_cov, dt);
      insUpdateState(truth_state_it->angularVelocity(), truth_state_it->acceleration(), dt, rolled_state);
      rolled_state.utime = truth_state_it->utime;
      rolled_states.push_back(rolled_state);
      rolled_covs.push_back(rolled_covariance - start_window_cov);

      truth_state_it++;
      truth_cov_it++;
      if (truth_state_it == truth_state_history.end())
        return;
    }

    rolled_state.subtractState(*truth_state_it);
    rolled_state.quatToChi();
    state_errors.push_back(rolled_state);
    covs.push_back(rolled_covariance - start_window_cov);
  }
}

double negLogLikelihood(const std::list<RBIS> & state_errors, const RBIMList & covs, const VectorXi & active_inds)
{
  RBIMList::const_iterator cov_it = covs.begin();
  std::list<RBIS>::const_iterator state_it = state_errors.begin();

  double neg_likelihood = 0;

  while (state_it != state_errors.end()) {
    MatrixXd cov_full = *cov_it;
    VectorXd error_full = state_it->vec;

    MatrixXd cov_active = selectBlockByIndices(cov_full, active_inds, active_inds);
    VectorXd error_active = selectRowsByIndices(error_full, active_inds);

    double log_like = loglike_normalized(error_active, VectorXd::Zero(error_active.rows()), cov_active);

    neg_likelihood -= log_like;
    state_it++;
    cov_it++;
  }
  return neg_likelihood;
}

void loadFilterHistory(const string & logFileName, const string & filterStateChannel, int64_t start_utime,
    int64_t end_utime, list<RBIS> & state_list, RBIMList & cov_list)
{

  state_list.clear();
  cov_list.clear();

  vector<mav::filter_state_t> filter_state_messages = lcm_utils::loadMsgsFromLog<mav::filter_state_t>(logFileName,
      filterStateChannel);

  for (int ii = 0; ii < filter_state_messages.size(); ii++) {
    if (filter_state_messages[ii].utime < start_utime)
      continue;
    if (filter_state_messages[ii].utime > end_utime)
      break;

    Map<const RBIM> cov_map(&(filter_state_messages[ii].cov[0]));

    cov_list.push_back(cov_map);
    state_list.push_back(RBIS(filter_state_messages[ii]));
  }
}

//
//int main(int argc, char** argv)
//{
//  string logfileName;
//  string outFileName = "noise_id_output.txt";
//  string filterStateChannel = "STATE_ESTIMATOR_POSE";
//  double q_gyro, q_accel;
//  int N_window = 1000;
//  int64_t start_utime = 0;
//  int64_t end_utime = std::numeric_limits<int64_t>::max();
//  char inds_mode = 'a';
//
//  ConciseArgs opt(argc, argv, "logFname outFname q_gyro q_accel");
//  opt.add(inds_mode, "i", "indices", "active indices, a=all");
//  opt.add(start_utime, "s", "start", "start utime");
//  opt.add(end_utime, "e", "end", "end utime");
//  opt.add(N_window, "w", "window", "window size in imu steps");
//  opt.add(outFileName, "o", "output", "output filename for likelihood");
//  opt.parse(logfileName, q_gyro, q_accel);
//
//  VectorXi active_inds;
//
//  switch (inds_mode) {
//  case 'a':
//    active_inds.resize(9);
//    active_inds.head(3) = RBIS::velocityInds();
//    active_inds.segment(3,3) = RBIS::chiInds();
//    active_inds.tail(3) = RBIS::positionInds();
//    break;
//  default:
//    fprintf(stderr, "unrecognized index specification %c\n", inds_mode);
//    opt.usage(true);
//  }
//
//  fprintf(stderr, "active indices for noise likelihood:\n");
//  eigen_dump(active_inds);
//
//  vector<mav::filter_state_t> filter_state_messages = lcm_utils::loadMsgsFromLog<mav::filter_state_t>(logfileName,
//      filterStateChannel);
//
//  list<RBIS> state_list = list<RBIS>();
//  RBIMList cov_list = RBIMList();
//
//  for (int ii = 0; ii < filter_state_messages.size(); ii++) {
//    if (filter_state_messages[ii].utime < start_utime)
//      continue;
//    if (filter_state_messages[ii].utime > end_utime)
//      break;
//
//    Map<const RBIM> cov_map(&(filter_state_messages[ii].cov[0]));
//
//    cov_list.push_back(cov_map);
//    state_list.push_back(RBIS(filter_state_messages[ii]));
//  }
//
//  list<RBIS> state_errors = list<RBIS>();
//  RBIMList error_covs = RBIMList();
//
//  sampleProcessForward(state_list, DT, q_gyro, q_accel, N_window, state_errors, error_covs);
//  double neglike = negLogLikelihood(state_errors, error_covs, active_inds);
//
//  return 0;
//}

}
