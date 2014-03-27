#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <path_util/path_util.h>
#include <eigen_utils/eigen_utils.hpp>
#include <lcm/lcm.h>

#include <lcmtypes/mav_pose_t.h>
#include <lcmtypes/mav_ins_t.h>
#include <lcmtypes/vicon_body_t.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_utils/eigen_utils.hpp>

#include <getopt.h>

#define DEF_INS_CHANNEL "MICROSTRAIN_INS"
#define DEF_POSE_CHANNEL "VICON_POSE"
#define DEF_VICON_CHANNEL "VICON_fixie"

#define RAD2DEG(X) (180.0/M_PI*(X))
#define DEG2RAD(X) (M_PI/180.0*(X))

using namespace std;
using namespace Eigen;
using namespace eigen_utils;

#define DT .01

typedef struct {
  lcm_t * lcm;
  BotParam * param;
  BotFrames * frames;

  BotTrans ins_to_body;

  Eigen::Vector3d omegab;
  Eigen::Vector3d accelb;

  char * pose_channel;
  char * ins_channel;
  char * vicon_channel;

//  BotTrans enu_to_ned;
//  BotTrans frd_to_flu;

} state_estimator_t;

static void ins_message_handler(const lcm_recv_buf_t *rbuf, const char * channel, const mav_ins_t * msg, void * user)
{
  state_estimator_t * self = (state_estimator_t *) user;

  double body_accel[3];
  double body_gyro[3];

  bot_trans_apply_vec(&self->ins_to_body, msg->accel, body_accel);
  bot_trans_apply_vec(&self->ins_to_body, msg->gyro, body_gyro);

  Map<Vector3d> accel(body_accel);
  Map<Vector3d> gyro(body_gyro);

  self->accelb = accel;
  self->omegab = gyro;

}

static void vicon_message_handler(const lcm_recv_buf_t *rbuf, const char * channel, const vicon_body_t * msg,
    void * user)
{
  state_estimator_t * self = (state_estimator_t *) user;

  BotTrans body_flu_to_enu;

  bot_trans_set_from_quat_trans(&body_flu_to_enu, msg->quat, msg->trans);

//  bot_trans_apply_trans_to(&self->enu_to_ned, &body_flu_to_enu, &body_flu_to_ned);
//  bot_trans_apply_trans_to(&body_flu_to_ned, &self->frd_to_flu, &body_to_enu);

  mav_pose_t pose;
  memcpy(pose.accel, self->accelb.data(), 3 * sizeof(double));
  memcpy(pose.rotation_rate, self->omegab.data(), 3 * sizeof(double));

  memcpy(pose.orientation, body_flu_to_enu.rot_quat, 4 * sizeof(double));
  memcpy(pose.pos, body_flu_to_enu.trans_vec, 3 * sizeof(double));
  pose.utime = msg->utime;

  mav_pose_t_publish(self->lcm, self->pose_channel, &pose);
}

static void usage(const char *progname)
{
  char *basename = g_path_get_basename(progname);
  printf(
      "Assumes that the vicon pose message contains the transform from a forward-left-up coordinate frame"
          " to a east-north-up coordinate frame. The appropriate forward-right-down, north-east-down transformations"
          " are made to publish an FRD-ENU fixie pose message\n"
          "Usage: %s [options]\n"
          "\n"
          "Options:\n"
          "\n"
          "    -h, --help                Shows this help text and exits\n"
          "    -i, --ins                 INS message channel to subscribe (default %s)\n"
          "    -v, --vicon               Vicon message channel to subscribe (default %s)\n"
          "    -p, --pose                POSE message channel to publish (default %s)\n"
          "\n", basename, DEF_INS_CHANNEL, DEF_VICON_CHANNEL, DEF_POSE_CHANNEL);
  free(basename);
  exit(1);
}

int main(int argc, char **argv)
{
  const char *optstring = "hi:v:p:";
  struct option long_opts[] = { { "help", no_argument, 0, 'h' },
      { "ins", required_argument, 0, 'i' },
      { "vicon", required_argument, 0, 'v' },
      { "pose", required_argument, 0, 'p' },
      { 0, 0, 0, 0 } };

  state_estimator_t * self = (state_estimator_t *) calloc(1, sizeof(state_estimator_t));

  //default settings
  self->ins_channel = strdup(DEF_INS_CHANNEL);
  self->pose_channel = strdup(DEF_POSE_CHANNEL);
  self->vicon_channel = strdup(DEF_VICON_CHANNEL);

  int c;
  while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
    switch (c) {
    case 'h':
      usage(argv[0]);
      break;
    case 'i':
      self->ins_channel = strdup(optarg);
      break;
    case 'v':
      {
        if (strncmp(optarg, "VICON", strlen("VICON")) == 0)
          self->vicon_channel = strdup(optarg);
        else
          self->vicon_channel = g_strdup_printf("VICON_%s", optarg);
      }
      break;
    case 'p':
      self->pose_channel = strdup(optarg);
      break;
    default:
      usage(argv[0]);
      break;
    }
  }

  if (optind < argc - 1) {
    usage(argv[0]);
  }

  self->lcm = bot_lcm_get_global(NULL);
  self->param = bot_param_get_global(self->lcm, 0);

  if (self->param == NULL) {
    char param_fname[1024];
    sprintf(param_fname, "%s/fixie.cfg", getConfigPath());
    fprintf(stderr, "Warning: No param server running, falling back to param file:\n %s\n", param_fname);
    self->param = bot_param_new_from_file(param_fname);
  }
  self->frames = bot_frames_get_global(self->lcm, self->param);

  bot_frames_get_trans(self->frames, "microstrain", "body", &self->ins_to_body);
//  bot_frames_get_trans(self->frames, "enu_global", "ned_global", &self->enu_to_ned);
//  bot_frames_get_trans(self->frames, "body_flu", "body", &self->frd_to_flu); //get the inverse
//  bot_trans_invert(&self->frd_to_flu);

//  mav_ins_t_subscribe(self->lcm, self->ins_channel, ins_message_handler, (void *) self);

  printf("Subscribing to vicon channel: %s\n", self->vicon_channel);
  vicon_body_t_subscribe(self->lcm, self->vicon_channel, vicon_message_handler, (void *) self);

  while (true) {
    lcm_handle(self->lcm);
  }

  return 0;
}
