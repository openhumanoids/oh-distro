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

#include <lcmtypes/mav_filter_state_t.h>

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

class app_t {
public:
  app_t(int argc, char ** argv)
  {
    lcm = bot_lcm_get_global(NULL);
    param = bot_param_get_global(lcm, 0);

    if (param == NULL) {
      exit(1);
    }
    frames = bot_frames_get_global(lcm, param);

    ConciseArgs opt(argc, argv);
    opt.parse();

    laser_handler = new LaserGPFHandler(lcm, param, frames);
    filter_state_channel = bot_param_get_str_or_fail(param, "state_estimator.filter_state_channel");

    bot_gauss_rand_init(time(NULL)); //randomize for particles

    laser_queue = new deque<kinect_frame_msg_t *>();
    filter_state_queue = new deque<mav_filter_state_t *>();

    gpf = laser_handler->gpf;

    //setup threads
    lcm_data_mutex = g_mutex_new();
    lcm_data_cond = g_cond_new();
    processor_thread = g_thread_create(processing_func, (void *) this, false, NULL);

    //----------------------------------------------------------------------------

    kinect_frame_msg_t_subscribe(lcm, laser_handler->laser_channel.c_str(), laser_message_handler, (void *) this);
    mav_filter_state_t_subscribe(lcm, filter_state_channel.c_str(), filter_state_handler, (void *) this);

  }

  //--------------------LCM stuff-----------------------
  lcm_t * lcm;
  BotParam * param;
  BotFrames * frames;

  string filter_state_channel;

  LaserGPFHandler * laser_handler;

  //lcm reading thread stuff
  GThread * processor_thread;
  GMutex * lcm_data_mutex; //should be held when reading/writing message queues
  GCond * lcm_data_cond; //signals new lcm data
  deque<kinect_frame_msg_t *> * laser_queue;
  deque<mav_filter_state_t *> * filter_state_queue;
  int noDrop;
  //------------------------------------------------------

  LaserGPF *gpf;



static void filter_state_handler(const lcm_recv_buf_t *rbuf, const char * channel, const mav_filter_state_t * msg,
    void * user)
{
  app_t * app = (app_t *) user;
  mav_filter_state_t * msg_copy = mav_filter_state_t_copy(msg);
  g_mutex_lock(app->lcm_data_mutex);
  app->filter_state_queue->push_back(msg_copy);
  g_mutex_unlock(app->lcm_data_mutex);
  g_cond_broadcast(app->lcm_data_cond);
}

static void laser_message_handler(const lcm_recv_buf_t *rbuf, const char * channel, const kinect_frame_msg_t * msg,
    void * user)
{
  app_t * app = (app_t *) user;
  if (app->laser_handler->counter++ % app->laser_handler->downsample_factor != 0)
    return;


  kinect_frame_msg_t * msg_copy = kinect_frame_msg_t_copy(msg);
  g_mutex_lock(app->lcm_data_mutex);
  if (!app->noDrop) {
    //clear the old messages
    while (!app->laser_queue->empty()) {
      kinect_frame_msg_t * msg = app->laser_queue->front();
      kinect_frame_msg_t_destroy(msg);
      app->laser_queue->pop_front();
    }
  }
  app->laser_queue->push_back(msg_copy);
  g_mutex_unlock(app->lcm_data_mutex);
  g_cond_broadcast(app->lcm_data_cond);
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

    kinect_frame_msg_t * laser_msg = app->laser_queue->front();
    app->laser_queue->pop_front();
    mav_filter_state_t * fs_msg = NULL;
    //keep going until the queue is empty, or the front is actually after the
    while (!app->filter_state_queue->empty() && app->filter_state_queue->front()->utime <= laser_msg->utime) {
      if (fs_msg != NULL) {
        mav_filter_state_t_destroy(fs_msg);
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

    Map<const RBIM> cov(fs_msg->cov);
    Map<const RBIS::VectorNd> state_vec_map(fs_msg->state);
    Quaterniond quat;
    botDoubleToQuaternion(quat, fs_msg->quat);
    RBIS state(state_vec_map, quat);

    bool valid = app->gpf->getMeasurement(state, cov, laser_msg, z_effective, R_effective);
    if (valid) {
//      eigen_dump(state);
//      eigen_dump(cov);
//      eigen_dump(z_effective.transpose());
//      eigen_dump(R_effective);

      mav_indexed_measurement_t * gpf_msg = gpfCreateLCMmsg(app->gpf->laser_gpf_measurement_indices, z_effective,
          R_effective);
      gpf_msg->utime = laser_msg->utime;
      gpf_msg->state_utime = fs_msg->utime;

      mav_indexed_measurement_t_publish(app->lcm, app->laser_handler->channel.c_str(), gpf_msg);
      mav_indexed_measurement_t_destroy(gpf_msg);
    }

    //destroy the local copies of the data
    kinect_frame_msg_t_destroy(laser_msg);
    mav_filter_state_t_destroy(fs_msg);

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
    int ret = lcm_handle(app->lcm);
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

