#include <mav_state_est/noise_id.hpp>
#include <lcm_utils/lcm_utils.hpp>
#include <lcmtypes/mav/filter_state_t.hpp>
#include <lcmtypes/rigid_body/pose_t.hpp>
#include <ConciseArgs>

using namespace std;
using namespace Eigen;
using namespace MavStateEst;

#define DT .01


int main(int argc, char** argv)
{
  string logfileName;
  string outFileNameSuffix = "_rolled";
  string filterStateChannel = "STATE_ESTIMATOR_STATE";
  string rolledPoseChannel = "STATE_ESTIMATOR_POSE";
  string gtPoseChannel = "SIM_POSE_GT";
  double q_gyro = 0.5;
  double q_accel = .1;
  int N_window = 1000;
  int64_t start_utime = 0;
  int64_t end_utime = std::numeric_limits<int64_t>::max();
  char inds_mode = 'a';

  ConciseArgs opt(argc, argv, "logFname");
  opt.add(inds_mode, "i", "indices", "active indices, a=all");
  opt.add(start_utime, "s", "start", "start utime");
  opt.add(end_utime, "e", "end", "end utime");
  opt.add(N_window, "w", "window", "window size in imu steps");
  opt.add(outFileNameSuffix, "o", "output", "output filename for likelihood");
  opt.add(q_gyro, "g", "q_gyro", "gyro noise val (deg/sec)");
  opt.add(q_accel, "a", "q_accel", "accel noise val (m/sec^2)");
  opt.parse(logfileName);

  string outFileName = logfileName + outFileNameSuffix;

  fprintf(
      stderr,
      "rolling forward filter states from channel %s\n"
      "publishing rolled forward states on %s, poses on %s\n"
      "publishing gt poses on %s\n",
      filterStateChannel.c_str(), filterStateChannel.c_str(), rolledPoseChannel.c_str(), gtPoseChannel.c_str());

  double cov_gyro = bot_sq(bot_to_radians(q_gyro));
  double cov_accel = bot_sq(q_accel);

  VectorXi active_inds;

  switch (inds_mode) {
  case 'a':
    active_inds.resize(9);
    active_inds.head(3) = RBIS::velocityInds();
    active_inds.segment(3, 3) = RBIS::chiInds();
    active_inds.tail(3) = RBIS::positionInds();
    break;
  default:
    fprintf(stderr, "unrecognized index specification %c\n", inds_mode);
    opt.usage(true);
  }



  fprintf(stderr, "active indices for noise likelihood:\n");
  eigen_dump(active_inds);

  list<RBIS> state_list = list<RBIS>();
  RBIMList cov_list = RBIMList();
  loadFilterHistory(logfileName, filterStateChannel, 0, numeric_limits<int64_t>::max(), state_list, cov_list);
  fprintf(stderr, "loaded %d filter states\n", (int) state_list.size());

  list<RBIS> state_errors = list<RBIS>();
  RBIMList error_covs = RBIMList();
  list<RBIS> rolled_states = list<RBIS>();
  RBIMList rolled_covs = RBIMList();
  sampleProcessForward(state_list, cov_list, DT, cov_gyro, cov_accel, N_window, state_errors, error_covs, rolled_states, rolled_covs);
  fprintf(stderr, "sectioned into %d windows\n", (int) state_errors.size());

  lcm::LogFile out_log(outFileName, "w");
  if (!out_log.good()) {
    fprintf(stderr, "Error couldn't open log file %s\n", outFileName.c_str());
    exit(1);
  }

  RBIMList::iterator cov_it = cov_list.begin();
  RBIMList::iterator rolled_cov_it = rolled_covs.begin();
  list<RBIS>::iterator state_it = state_list.begin();
  list<RBIS>::iterator rolled_state_it = rolled_states.begin();

  fprintf(stderr, "writing to %s\n", outFileName.c_str());

  while (state_it != state_list.end()) {
    rigid_body::pose_t gt_pose = state_it->getPose();
    rigid_body::pose_t rolled_pose = rolled_state_it->getPose();
    mav::filter_state_t rolled_filter_state = rbisCreateFilterStateMessageCPP(*rolled_state_it, *rolled_cov_it);

    lcm_utils::logEventData event_data;

    event_data = lcm_utils::encodeToLogEventData(gtPoseChannel, gt_pose.utime, &gt_pose);
    out_log.writeEvent(&event_data.event);

    event_data = lcm_utils::encodeToLogEventData(rolledPoseChannel, rolled_pose.utime, &rolled_pose);
    out_log.writeEvent(&event_data.event);

    event_data = lcm_utils::encodeToLogEventData(filterStateChannel, rolled_filter_state.utime, &rolled_filter_state);
    out_log.writeEvent(&event_data.event);

    state_it++;
    cov_it++;
    rolled_state_it++;
    rolled_cov_it++;
  }

  fprintf(stderr, "all done!\n");

  return 0;
}

