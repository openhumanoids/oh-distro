#ifndef NOISE_ID_HPP__
#define NOISE_ID_HPP__

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <lcm/lcm.h>
#include <lcm_utils/lcm_utils.hpp>
#include <mav_state_est/rbis.hpp>
#include <list>

#include <Eigen/Dense>
#include <lcmtypes/mav/filter_state_t.hpp>
#include <lcm_utils/lcm_utils.hpp>
#include <string>

namespace MavStateEst {

typedef std::list<RBIM, Eigen::aligned_allocator<RBIM> > RBIMList;

void sampleProcessForward(const std::list<RBIS> & truth_state_history, const RBIMList & truth_cov_history, double dt,
    double q_gyro, double q_accel, int N_window, std::list<RBIS> & state_errors, RBIMList & covs,
    std::list<RBIS> & rolled_states, RBIMList & rolled_covs);

double negLogLikelihood(const std::list<RBIS> & state_errors, const RBIMList & covs,
    const Eigen::VectorXi & active_inds);

void loadFilterHistory(const std::string & logFileName, const std::string & filterStateChannel, int64_t start_utime,
    int64_t end_utime, std::list<RBIS> & state_list, RBIMList & cov_list);

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
#endif

