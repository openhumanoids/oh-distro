#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/rigid_body/pose_t.hpp>
#include <lcmtypes/vicon/body_t.hpp>
#include <lcmtypes/mav/filter_state_t.hpp>

#include <mav_state_est/rbis.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_utils/eigen_utils.hpp>

#include <ConciseArgs>

using namespace std;
using namespace Eigen;
using namespace eigen_utils;

const string DEF_POSE_CHANNEL = "STATE_ESTIMATOR_POSE";
const string FILTER_STATE_CHANNEL = "STATE_ESTIMATOR_STATE";

/**
 * expects:
 *
 * vicon_only_state_estimator {
 *      double angular_rate_alpha; #dt/timeconstant
 *      double velocity_alpha; #dt/timeconstant
 *      double dt; #dt < 0, use utimes to compute dt
 * }
 */

class ViconOnlyStateEst {
public:
  int64_t last_utime;
  RigidBodyState last_state;

  lcm::LCM * lcm;

  BotParam * param;
  string pose_channel, state_channel;

  eigen_utils::ExponentialFilter<Eigen::Vector3d> body_velocity_filter;
  eigen_utils::ExponentialFilter<Eigen::Vector3d> body_angular_rate_filter;
  double dt;
  double angular_rate_timeconstant, velocity_timeconstant;

  ViconOnlyStateEst(int argc, char ** argv) :
      last_state(), body_velocity_filter(Eigen::Vector3d::Zero(), 1),
          body_angular_rate_filter(Eigen::Vector3d::Zero(), 1)
  {
    pose_channel = DEF_POSE_CHANNEL;
    string vicon_body;
    bool param_tune = false;

    ConciseArgs opt(argc, argv, "vicon_body");
    opt.add(pose_channel, "p", "pose", "pose channel to publish");
    opt.add(param_tune, "P", "param_tune", "allow online param tuning");
    opt.parse(vicon_body);

    string vicon_channel = "VICON_" + vicon_body;

    lcm = new lcm::LCM("");
    param = bot_param_new_from_server(lcm->getUnderlyingLCM(), 1);
    if (param_tune) {
      bot_param_add_update_subscriber(param, &loadParams, (void *) this);
      fprintf(stderr, "param tuning enabled\n");
    }
    loadParams(NULL, param, 0, (void *) this);

    last_utime = 0;

    fprintf(stderr, "subscribed to %s for vicon messages\n", vicon_channel.c_str());
    lcm->subscribe(vicon_channel, &ViconOnlyStateEst::viconHandler, this);
  }

  static void loadParams(BotParam * old_param, BotParam * new_param, int64_t utime, void * user)
  {
    ViconOnlyStateEst * self = (ViconOnlyStateEst *) user;
    self->angular_rate_timeconstant = bot_param_get_double_or_fail(new_param,
        "vicon_only_state_estimator.angular_rate_timeconstant");

    self->velocity_timeconstant = bot_param_get_double_or_fail(new_param,
        "vicon_only_state_estimator.velocity_timeconstant");

    self->dt = bot_param_get_double_or_fail(new_param, "vicon_only_state_estimator.dt");
  }

  void run()
  {
    while (true) {
      lcm->handle();
    }
  }

  bool isValid(const vicon::body_t * msg)
  {
    if ((Eigen::Map<const Eigen::Array3d>(msg->trans).abs() < 1e-5).all()) {
      fprintf(stderr, "vicon dropout\n");
      return false;
    }
    else if (msg->utime < last_utime) {
      fprintf(stderr, "vicon out order utimes, utime difference: %fd\n", msg->utime - last_utime);
      return false;
    }
    else {
      return true;
    }
  }

  void initialize(const vicon::body_t * msg)
  {
    last_state.position() = Map<const Vector3d>(msg->trans);
    eigen_utils::botDoubleToQuaternion(last_state.orientation(), msg->quat);
    last_utime = msg->utime;
  }

  bool isInitialized()
  {
    if (last_utime == 0) {
      return false;
    }
    else {
      return true;
    }
  }

  void viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
      const vicon::body_t * msg)
  {
    RigidBodyState cur_state;
    cur_state.position() = Map<const Vector3d>(msg->trans);
    eigen_utils::botDoubleToQuaternion(cur_state.orientation(), msg->quat);

    cur_state.utime = msg->utime;

    if (isValid(msg)) {
      if (isInitialized()) {
        double dt_used = dt;
        if (dt_used < 0) {
          dt_used = double(msg->utime - last_utime) / 1e6;
          dt_used = fmax(dt_used, .005);
        }
        RigidBodyState diff_state = cur_state;
        diff_state.subtractState(last_state);

        body_velocity_filter.setAlpha(dt_used / fmax(velocity_timeconstant, dt_used));
        body_velocity_filter.step(cur_state.orientation().inverse() * diff_state.position() / dt_used);

        AngleAxisd ang_ax_diff(diff_state.orientation());
        body_angular_rate_filter.setAlpha(dt_used / fmax(angular_rate_timeconstant, dt_used));
        body_angular_rate_filter.step(bot_mod2pi(ang_ax_diff.angle()) * ang_ax_diff.axis() / dt_used);

        last_state = cur_state;
        last_utime = cur_state.utime;
      }
      else {
        initialize(msg);
      }
    }
    else {
      fprintf(stderr, "vicon dropout\n");
    }

    cur_state.angularVelocity() = body_angular_rate_filter();
    cur_state.velocity() = body_velocity_filter();

    rigid_body::pose_t pose = cur_state.getPose();
    lcm->publish(pose_channel, &pose);

    RBIS rbis(cur_state);
    RBIM rbim = RBIM::Identity();
    mav::filter_state_t filter_state = rbisCreateFilterStateMessageCPP(rbis, rbim);
    lcm->publish(FILTER_STATE_CHANNEL, &filter_state);
  }
};

int main(int argc, char **argv)
{
  ViconOnlyStateEst vicon_state_est(argc, argv);
  vicon_state_est.run();

  return 0;
}
