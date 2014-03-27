#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define __STDC_LIMIT_MACROS
#include <stdint.h>

#include <deque>

#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include <lcm/lcm.h>
#include <bot_lcmgl_client/lcmgl.h>

#include <lcmtypes/mav/filter_state_t.hpp>
#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <lcmtypes/mav/indexed_measurement_t.hpp>

#include <Eigen/Dense>
#include <eigen_utils/eigen_utils.hpp>
#include <Eigen/StdVector>

#include <mav_state_est/rbis.hpp>
#include <mav_state_est/gpf/laser_gpf_lib.hpp>

#include "rbis_gpf_update.hpp"

#include <ConciseArgs>

using namespace std;
using namespace Eigen;
using namespace eigen_utils;
using namespace MavStateEst;

class app_t {
public:
  app_t(int argc, char ** argv)
  {

    ConciseArgs opt(argc, argv);
    opt.parse();

    lcm_front = new MavStateEst::LCMFrontEnd("");
    counter = 0;
    downsample_factor = bot_param_get_int_or_fail(lcm_front->param, "state_estimator.laser_gpf.downsample_factor");

    laser_handler = new LaserGPFHandler(lcm_front->lcm_pub->getUnderlyingLCM(), lcm_front->param, lcm_front->frames);

    bot_gauss_rand_init(time(NULL)); //randomize for particles

    laser_queue = new deque<bot_core::planar_lidar_t *>();
    filter_state_queue = new deque<mav::filter_state_t *>();

    gpf = laser_handler->gpf;

    //setup threads
    lcm_data_mutex = g_mutex_new();
    lcm_data_cond = g_cond_new();
    processor_thread = g_thread_create(processing_func, (void *) this, false, NULL);

    //----------------------------------------------------------------------------

    lcm_front->lcm_recv->subscribe(laser_handler->laser_channel.c_str(), &app_t::laser_message_handler, this);
    lcm_front->lcm_recv->subscribe(lcm_front->filter_state_channel.c_str(), &app_t::filter_state_handler, this);

  }

  //--------------------LCM stuff-----------------------
  MavStateEst::LCMFrontEnd * lcm_front;

  LaserGPFHandler * laser_handler;

  //lcm reading thread stuff
  GThread * processor_thread;
  GMutex * lcm_data_mutex; //should be held when reading/writing message queues
  GCond * lcm_data_cond; //signals new lcm data
  deque<bot_core::planar_lidar_t *> * laser_queue;
  deque<mav::filter_state_t *> * filter_state_queue;
  int noDrop;
  //------------------------------------------------------

  int counter;
  int downsample_factor;

  LaserGPF *gpf;

  void filter_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
      const mav::filter_state_t * msg)
  {
    mav::filter_state_t * msg_copy = new mav::filter_state_t(*msg);
    g_mutex_lock(lcm_data_mutex);
    filter_state_queue->push_back(msg_copy);
    g_mutex_unlock(lcm_data_mutex);
    g_cond_broadcast(lcm_data_cond);
  }

  void laser_message_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
      const bot_core::planar_lidar_t * msg)
  {
    if (counter++ % downsample_factor != 0)
      return;

    bot_core::planar_lidar_t * msg_copy = new bot_core::planar_lidar_t(*msg);
    g_mutex_lock(lcm_data_mutex);
    if (!noDrop) {
      //clear the old messages
      while (!laser_queue->empty()) {
        bot_core::planar_lidar_t * msg_queued = laser_queue->front();
        delete msg_queued;
        laser_queue->pop_front();
      }
    }
    laser_queue->push_back(msg_copy);
    g_mutex_unlock(lcm_data_mutex);
    g_cond_broadcast(lcm_data_cond);
  }

  static void * processing_func(void * user)
  {
    app_t * app = (app_t *) user;

    g_mutex_lock(app->lcm_data_mutex);
    while (1) {
      if (app->laser_queue->empty() || app->filter_state_queue->empty()) {
        g_cond_wait(app->lcm_data_cond, app->lcm_data_mutex);
        continue;
      }

      bot_core::planar_lidar_t * laser_msg = app->laser_queue->front();
      app->laser_queue->pop_front();
      mav::filter_state_t * fs_msg = NULL;
      //keep going until the queue is empty, or the front is actually after the
      while (!app->filter_state_queue->empty() && app->filter_state_queue->front()->utime <= laser_msg->utime) {
        if (fs_msg != NULL) {
          delete fs_msg;
        }
        fs_msg = app->filter_state_queue->front();
        app->filter_state_queue->pop_front();
      }
      if (fs_msg == NULL) {
        fprintf(stderr, "WARNING: first filter state is after the laser message\n");
        continue;
      }

      g_mutex_unlock(app->lcm_data_mutex);

      Eigen::VectorXd z_effective;
      Eigen::MatrixXd R_effective;

      Map<const RBIM> cov(&fs_msg->cov[0]);
      Map<const RBIS::VectorNd> state_vec_map(&fs_msg->state[0]);
      Quaterniond quat;
      botDoubleToQuaternion(quat, fs_msg->quat);
      RBIS state(state_vec_map, quat);

      bool valid = app->gpf->getMeasurement(state, cov, laser_msg, z_effective, R_effective);
      if (valid) {
//      eigen_dump(state);
//      eigen_dump(cov);
//      eigen_dump(z_effective.transpose());
//      eigen_dump(R_effective);

        mav::indexed_measurement_t * gpf_msg = gpfCreateLCMmsgCPP(app->gpf->laser_gpf_measurement_indices, z_effective,
            R_effective);
        gpf_msg->utime = laser_msg->utime;
        gpf_msg->state_utime = fs_msg->utime;

        app->lcm_front->lcm_pub->publish(app->laser_handler->pub_channel.c_str(), gpf_msg);

        delete gpf_msg;
      }

      //destroy the local copies of the data
      delete laser_msg;
      delete fs_msg;

      //go back around loop, must hold lcm_data lock
      g_mutex_lock(app->lcm_data_mutex);
    }
    return NULL;
  }

};

int main(int argc, char **argv)
{

  app_t * app = new app_t(argc, argv);

  while (true) {
    int ret = app->lcm_front->lcm_recv->handle();
    if (ret != 0) {
      printf("log is done\n");
      break;
    }
  }

  //wait on the mutex until the queues are empty
  g_mutex_lock(app->lcm_data_mutex);
  g_cond_wait(app->lcm_data_cond, app->lcm_data_mutex);

  //todo: compute stats, cleanup, etc...
  printf("all_done!\n");
  return 0;
}

