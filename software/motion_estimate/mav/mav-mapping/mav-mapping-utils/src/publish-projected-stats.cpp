#include <bot_frames/bot_frames.h>
#include <laser_utils/laser_util.h>
#include <Eigen/Dense>
#include <eigen_utils/eigen_utils.hpp>
#include <octomap_utils/octomap_util.hpp>
#include <ConciseArgs>
#include <mav_state_est/gpf/laser_gpf_lib.hpp>
#include <path_util/path_util.h>

using namespace Eigen;
using namespace std;

class PubStats {
public:
  double floor_thresh;
  double ceil_thresh;

  BotFrames * frames;
  BotParam * param;
  lcm::LCM * lcm;
  LaserGPF * laser_gpf;
  RBIS state;
  RBIM cov;
  bool set_state;
  string pointChannel;

  PubStats(BotParam * _param, BotFrames * _frames, lcm_t * _lcm, const string _pointChannel) :
      frames(_frames), param(_param), pointChannel(_pointChannel)
  {
    lcm = new lcm::LCM(_lcm);
    set_state = false;
    this->laser_gpf = new LaserGPF(lcm->getUnderlyingLCM(), param, frames);
  }

  void filterStateHandler(const mav_filter_state_t * msg)
  {
    Map<const RBIM> cov_map(msg->cov);
    cov = cov_map;
    state = RBIS(msg);
    set_state = true;

  }

  void laserHandler(const bot_core_planar_lidar_t * msg)
  {
    if (!set_state)
      return;
    Vector3d laser_omega = state.angularVelocity();
    Vector3d laser_vel = state.velocity();
    laser_projected_scan * global_projected_scan = laser_create_projected_scan_from_planar_lidar_with_motion(
        laser_gpf->laser_projector,
        msg, "local", laser_omega.data(), laser_vel.data());

    if (global_projected_scan == NULL)
      return;

    laser_decimate_projected_scan(global_projected_scan, laser_gpf->beam_skip, laser_gpf->spatial_decimation_min,
        laser_gpf->spatial_decimation_max);

    MatrixXd point_mat = MatrixXd::Zero(4, global_projected_scan->npoints + 1); //plus one for utime

    for (int ii = 0; ii < point_mat.cols(); ii++) {
      if (global_projected_scan->point_status[ii] > laser_valid_projection)
        continue;

      point_mat(3, ii) = 1;
      point_mat(0, ii) = global_projected_scan->points[ii].x;
      point_mat(1, ii) = global_projected_scan->points[ii].y;
      point_mat(2, ii) = global_projected_scan->points[ii].z;

    }

    point_mat.rightCols(1).setConstant(double(msg->utime));

    eigen_utils::eigen_matrixxd_t point_msg = eigen_utils::toMatrixXdLcmMsg(point_mat);
    lcm->publish(pointChannel, &point_msg);

    laser_destroy_projected_scan(global_projected_scan);

  }

};

typedef struct {
  PubStats * pub_stats;
  lcm_t * lcm;
  BotParam * param;
  BotFrames * frames;
} app_t;

static void filter_state_handler(const lcm_recv_buf_t *rbuf, const char * channel,
    const mav_filter_state_t * msg, void * user)
{
  app_t * self = (app_t *) user;
  self->pub_stats->filterStateHandler(msg);
}

static void laser_handler(const lcm_recv_buf_t *rbuf, const char * channel,
    const bot_core_planar_lidar_t * msg, void * user)
{
  app_t * self = (app_t *) user;
  self->pub_stats->laserHandler(msg);
}

int main(int argc, char ** argv)
{
  string filterStateChannel = "STATE_ESTIMATOR_STATE";
  string laserChannel = "LASER";
  string pointChannel = "GLOBAL_LASER_POINTS";

  app_t * app = (app_t *) calloc(1, sizeof(app_t));
  app->lcm = lcm_create(NULL);
  char param_fname[1024];
  sprintf(param_fname, "%s/fixie.cfg", getConfigPath());
  fprintf(stderr, "Loading params from:\n %s\n", param_fname);
  app->param = bot_param_new_from_file(param_fname);
  app->frames = bot_frames_new(app->lcm, app->param);
  app->pub_stats = new PubStats(app->param, app->frames, app->lcm, pointChannel);

  mav_filter_state_t_subscribe(app->lcm, filterStateChannel.c_str(), filter_state_handler, (void*) app);
  bot_core_planar_lidar_t_subscribe(app->lcm, laserChannel.c_str(), laser_handler, (void*) app);

  while (1)
    lcm_handle(app->lcm);

  return 0;

}
