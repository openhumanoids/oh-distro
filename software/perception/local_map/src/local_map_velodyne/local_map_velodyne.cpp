#include <stdio.h>
#include <inttypes.h>
//#include <gtk/gtk.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <getopt.h>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include <velodyne/velodyne.h>
#include <path_util/path_util.h>
#include <lcmtypes/bot_param_update_t.h>

#include "lcmtypes/senlcm_velodyne_t.h"
#include "lcmtypes/senlcm_velodyne_list_t.h"

#define GAMMA 0.1
#define PUBLISH_HZ 50
#define VELODYNE_DATA_CIRC_SIZE 1
#define SHORT_RANGE_FILTER 0.3


using namespace std;

typedef struct _xyzi_t {
  double xyz[3];
  double intensity;
} xyzi_t;

typedef struct _rate_t {
  double current_hz;
  int64_t last_tick;
  int64_t tick_count;
} rate_t;

enum {
  COLOR_Z,
  COLOR_INTENSITY,
  COLOR_NONE,
};


typedef struct _pose_data_t pose_data_t;
struct _pose_data_t {
  double pose[6];
  double motion[6];
  int64_t utime;
};

typedef struct _state_t {

  GMainLoop *mainloop;

  rate_t* capture_rate;

  lcm_t *lcm;
  BotParam *param;
  BotFrames *frames;

  int64_t last_collector_utime;

  int have_data;

  velodyne_calib_t *calib;
  velodyne_laser_return_collector_t *collector;
  //velodyne_state_t *vstate;

  bot_core_pose_t *bot_pose_last;

  BotPtrCircular   *velodyne_data_circ;


  GAsyncQueue *velodyne_message_queue;
  GThread *velodyne_work_thread;
  GMutex *mutex;
  int velodyne_work_thread_exit;

  int64_t last_publish_utime;

  int64_t last_msg_count;

  int64_t 	      last_velodyne_data_utime;
  int64_t           last_pose_utime;

} state_t;

static int process_velodyne (state_t *self, const senlcm_velodyne_t *v);


rate_t* rate_new()
        {
  rate_t* rt = (rate_t *) calloc(1, sizeof(rate_t));
  return rt;
        }

void rate_destroy(rate_t* rate)
{
  free(rate);
}

int rate_update(rate_t* rate)
{
  // check the current time
  int64_t c_utime = bot_timestamp_now();

  // compute the framerate if we were to publish an image
  int64_t dt = c_utime - rate->last_tick;

  double p_framerate = GAMMA * (1.0 * 1e6 / dt) + (1 - GAMMA) * rate->current_hz;

  // otherwise, update current_hz with a exponential moving average, and return 1
  rate->current_hz = p_framerate;
  rate->last_tick = c_utime;
  rate->tick_count++;
  return 1;
}


void write_pcd(vector <xyzi_t> &points,std::ofstream &fstream ){

  fstream << "# .PCD v.5 - Point Cloud Data file format" << endl;
  fstream << "FIELDS x y z intensity" << endl;
  fstream << "SIZE 4 4 4 4" << endl;
  fstream << "TYPE F F F F" << endl;
  fstream << "WIDTH " << points.size() << endl;
  fstream << "HEIGHT 1" << endl;
  fstream << "POINTS " << points.size() << endl;
  fstream << "DATA ascii" << endl;

  for (size_t i=0; i < points.size(); i++){
    xyzi_t lr = points[i];
    ostringstream temp0;
    temp0 << lr.xyz[0] <<" " << lr.xyz[1]<< " " << lr.xyz[2] << " "<< lr.intensity << endl;
    fstream << temp0.str();
  }
  fstream.close();

}


static void
on_pose_trigger(const lcm_recv_buf_t *buf, const char *channel,
    const bot_core_pose_t *msg, void *user) {
  state_t *self = (state_t *)user;

  cout << "got trigger\n";
  int size = bot_ptr_circular_size(self->velodyne_data_circ);
  cout << "queue size: " << size << "\n";

  vector < xyzi_t > points;

  int hist_len = VELODYNE_DATA_CIRC_SIZE;


  for (unsigned int cidx = 0;
      cidx < bot_ptr_circular_size(self->velodyne_data_circ) && cidx < hist_len;
      cidx++) {
    printf("cidx : %d\n", cidx);

    velodyne_laser_return_collection_t *lrc =(velodyne_laser_return_collection_t *) bot_ptr_circular_index(self->velodyne_data_circ, cidx);
    printf("abce\n");

    double sensor_to_local[12];
    // Working here on bot frames

    if (!bot_frames_get_trans_mat_3x4 (self->frames, "VELODYNE",
        "local",
        sensor_to_local)) {
      fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
      return;
    }

    printf("LASER returns : %d\n", lrc->num_lr);

    int chunk_size = 32;// * 12;

    for (int s = 0; s < lrc->num_lr; s++) {
      velodyne_laser_return_t *lr = &(lrc->laser_returns[s]);

      if(s % chunk_size == 0){
        //updated the sensor_to_local transform
        if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
            "local", lr->utime,
            sensor_to_local)) {
          fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
          return;
        }
      }

      //fprintf(stderr, "\t %d - %d : %f\n", lr->physical, lr->logical, lr->phi);
      //double local_xyz[3];

      if (lr->range > SHORT_RANGE_FILTER){
        xyzi_t point;
        bot_vector_affine_transform_3x4_3d (sensor_to_local, lr->xyz, point.xyz);
        point.intensity = lr->intensity;
        points.push_back(point);
      }
    }
  }


  std::ofstream fstream;
  fstream.open ("test.pcd"); // output is now xyz quat
  write_pcd(points,fstream );
  cout << "wrote output file, exiting\n";


}



static void
on_bot_pose (const lcm_recv_buf_t *buf, const char *channel,
    const bot_core_pose_t *msg, void *user) {
  state_t *self = (state_t *)user;

  //  g_mutex_lock (self->mutex);

  if (self->bot_pose_last)
    bot_core_pose_t_destroy (self->bot_pose_last);
  self->bot_pose_last = bot_core_pose_t_copy (msg);

  //  g_mutex_unlock (self->mutex);
}


void
circ_free_velodyne_data(void *user, void *p) {

  velodyne_laser_return_collection_t *lrc = (velodyne_laser_return_collection_t*) p;
  velodyne_free_laser_return_collection (lrc);
}

static void
on_velodyne_list(const lcm_recv_buf_t *rbuf, const char * channel,
    const senlcm_velodyne_list_t * msg, void * user)
{
  state_t *self = (state_t *)user;
  g_assert(self);


  static int64_t last_redraw_utime = 0;
  int64_t now = bot_timestamp_now();

  cout << msg->num_packets << " num packets\n";
  for (int i=0; i < msg->num_packets; i++)
    process_velodyne (self, &(msg->packets[i]));

  //  if ((now - last_redraw_utime) > MAX_REFRESH_RATE_USEC) {
  //    bot_viewer_request_redraw( self->viewer );
  //    last_redraw_utime = now;
  // }








  /*
  senlcm_velodyne_t *msg;

  int64_t now = bot_timestamp_now();

  // For each of the incomping packets
  for (int i=0; i < msgl->num_packets; i++){
//    cout << msgl->packets[i].utime << " recd\n";
    g_async_queue_push (self->velodyne_message_queue, senlcm_velodyne_t_copy (& (msgl->packets[i])));
  }
   */

  cout << " ====================== \n";

  //g_async_queue_push (self->velodyne_message_queue, senlcm_velodyne_t_copy (msg));
  return;

  //we are missing data packets
  //static int count = 0;
  //count++;

  // Is this a scan packet?
  /* if (msg->packet_type == SENLCM_VELODYNE_T_TYPE_DATA_PACKET) { */

  /*     int i_f = 0;  */

  /*     double ctheta = VELODYNE_GET_ROT_POS(msg->data, VELODYNE_DATA_FIRING_START(i_f)); */

  /*     static double test_handler_ctheta_prev = 0;  */

  /*     static double delta = 0; */

  /*     if(fabs(fabs(ctheta - test_handler_ctheta_prev) - delta) > 0.1 && fabs(fabs(ctheta - test_handler_ctheta_prev) - delta) < 4.0 ){ */
  /*         //fprintf(stderr,"%f Different Delta : %f, %f\n", msg->utime /1.0e6, fabs(ctheta - test_handler_ctheta_prev), delta); */
  /*     } */
  /*     delta = fabs(ctheta - test_handler_ctheta_prev); */

  /*     test_handler_ctheta_prev = ctheta; */

  /*     // Decode the data packet */
  /* 	fprintf (stdout, "*******************************************************\n"); */
  /*     velodyne_laser_return_collection_t *lrc = */
  /*         velodyne_decode_data_packet(self->calib, msg->data, msg->datalen, msg->utime); */

  /*     int ret = -1; */
  /*     //int ret = velodyne_collector_push_laser_returns (self->collector, lrc); */

  /*     velodyne_free_laser_return_collection (lrc); */

  /*     return; */

  /*     if (VELODYNE_COLLECTION_READY == ret) { */
  /*         //fprintf(stderr, "Outer count : %d\n", count);  */
  /*         //count = 0; */

  /*         rate_update(self->capture_rate); */

  /*         fprintf(stderr,"Data rate : %f\n", self->capture_rate->current_hz); */

  /*         velodyne_laser_return_collection_t *lrc = */
  /*             velodyne_collector_pull_collection (self->collector); */

  /*         // if enough time has elapsed since the last scan push it onto the circular buffer */

  /*             // memory leak city if this isnt here as soon as you increase the history spacing */
  /*         velodyne_free_laser_return_collection (lrc); */
  /*     } */
  /*     else if(VELODYNE_COLLECTION_READY == ret) { */
  /*         rate_update(self->capture_rate); */

  /*         fprintf(stderr,"Data rate : %f\n", self->capture_rate->current_hz); */

  /*         velodyne_laser_return_collection_t *lrc = */
  /*             velodyne_collector_pull_collection (self->collector); */

  /*         // if enough time has elapsed since the last scan push it onto the circular buffer */

  /*             // memory leak city if this isnt here as soon as you increase the history spacing */
  /*         velodyne_free_laser_return_collection (lrc); */
  /*     } */
  /* } */
}


// Processes 
static int
process_velodyne (state_t *self, const senlcm_velodyne_t *v)
{
  g_assert(self);

  int do_push_motion = 0; // only push motion data if we are starting a new collection or there is a new pose
  double hist_spc = 1000.0;//bot_gtk_param_widget_get_double (self->pw, PARAM_HISTORY_FREQUENCY);


  // Algorithm:
  // Decodes and pushes velodyne_t packets into the collector
  // When the collector is full it pushes the collector in its entirety into the circular buffer
  // The collector is a portion of a scan

  // Is this a scan packet?
  if (v->packet_type == SENLCM_VELODYNE_T_TYPE_DATA_PACKET) {

    velodyne_laser_return_collection_t *lrc =
        velodyne_decode_data_packet(self->calib, v->data, v->datalen, v->utime);

    int ret = velodyne_collector_push_laser_returns (self->collector, lrc);

    velodyne_free_laser_return_collection (lrc);

    if (VELODYNE_COLLECTION_READY == ret) {

      velodyne_laser_return_collection_t *lrc =
          velodyne_collector_pull_collection (self->collector);

      cout << "1\n";

      // if enough time has elapsed since the last scan push it onto the circular buffer
      if (abs (lrc->utime - self->last_velodyne_data_utime) > (int64_t)(1E6/hist_spc)) {
        cout << "2\n";

        bot_ptr_circular_add (self->velodyne_data_circ, lrc);
        self->last_velodyne_data_utime = lrc->utime;
      } else {
        // memory leak city if this isnt here as soon as you increase the history spacing
        velodyne_free_laser_return_collection (lrc);
      }

      //starting a new collection
      do_push_motion = 1;
    }
    else if(VELODYNE_COLLECTION_READY_LOW == ret) {
      fprintf(stderr,"Low packet - ignoring");

      velodyne_laser_return_collection_t *lrc =
          velodyne_collector_pull_collection (self->collector);

      velodyne_free_laser_return_collection (lrc);
    }
  }

  // Update the Velodyne's state information (pos, rpy, linear/angular velocity)
  if (do_push_motion) {

    if (!self->bot_pose_last)
      return 0;

    // push new motion onto collector
    velodyne_state_t state;

    state.utime = v->utime;

    // find sensor pose in local/world frame
    /*
     * double x_lr[6] = {self->pose->x, self->pose->y, self->pose->z,
     *                   self->pose->r, self->pose->p, self->pose->h};
     * double x_ls[6] = {0};
     * ssc_head2tail (x_ls, NULL, x_lr, self->x_vs);
     */

    BotTrans velodyne_to_local;
    bot_frames_get_trans_with_utime (self->frames, "VELODYNE", "local", v->utime, &velodyne_to_local);

    memcpy (state.xyz, velodyne_to_local.trans_vec, 3*sizeof(double));
    bot_quat_to_roll_pitch_yaw (velodyne_to_local.rot_quat, state.rph);

    // Compute translational velocity
    //
    // v_velodyne = v_bot + r x w
    BotTrans velodyne_to_body;
    bot_frames_get_trans (self->frames, "VELODYNE", "body", &velodyne_to_body);

    double v_velodyne[3];
    double r_body_to_velodyne_local[3];
    bot_quat_rotate_to (self->bot_pose_last->orientation, velodyne_to_body.trans_vec, r_body_to_velodyne_local);

    // vel_rot = r x w
    double vel_rot[3];
    bot_vector_cross_3d (r_body_to_velodyne_local, self->bot_pose_last->rotation_rate, vel_rot);

    bot_vector_add_3d (self->bot_pose_last->vel, vel_rot, state.xyz_dot);


    // Compute angular rotation rate
    memcpy (state.rph_dot, self->bot_pose_last->rotation_rate, 3*sizeof(double));

    //state.xyz[0] = x_ls[0]; state.xyz[1] = x_ls[1]; state.xyz[2] = x_ls[2];
    //state.rph[0] = x_ls[3]; state.rph[1] = x_ls[4]; state.rph[2] = x_ls[5];

    // move velocities and rates into sensor frame
    //double O_sv[9];
    //double rph_vs[3] = {self->x_vs[3], self->x_vs[4], self->x_vs[5]};
    //so3_rotxyz (O_sv, rph_vs);
    //double t_vs[3] = {self->x_vs[0], self->x_vs[1], self->x_vs[2]};
    ////uvw_sensor = O_sv * [uvw - skewsym(t_vs)*abc];

    // pose fields NEVER POPULATED
    //double abc[3] = {self->pose->a,
    //                 self->pose->b,
    //                 self->pose->c};
    //double uvw[3] = {self->pose->u,
    //                 self->pose->v,
    //                 self->pose->w};
    //double skewsym[9] = { 0,       -t_vs[2],  t_vs[1],
    //                      t_vs[2],  0,       -t_vs[0],
    //                     -t_vs[1],  t_vs[0],  0    };
    //GSLU_VECTOR_VIEW (uvw_sensor,3, {0});
    //gsl_vector_view abc_v = gsl_vector_view_array (abc, 3);
    //gsl_vector_view uvw_v = gsl_vector_view_array (uvw, 3);
    //gsl_matrix_view O_sv_v = gsl_matrix_view_array (O_sv, 3, 3);
    //gsl_matrix_view skewsym_v = gsl_matrix_view_array (skewsym, 3, 3);
    //skewsym(t_vs)*abc;
    //gslu_mv (&uvw_sensor.vector, &skewsym_v.matrix, &abc_v.vector); // uvw_sensor.vector is all zero since abc contains zeros
    //[uvw - skewsym(t_vs)*abc]
    //gsl_vector_sub (&uvw_v.vector, &uvw_sensor.vector);
    //uvw_sensor = O_sv * [uvw - skewsym(t_vs)*abc];
    //gslu_mv (&uvw_sensor.vector, &O_sv_v.matrix, &uvw_v.vector);

    // sensor frame rates
    //GSLU_VECTOR_VIEW (abc_sensor, 3, {0});
    //gsl_matrix_view O_vs_v = gsl_matrix_view_array (O_sv, 3, 3);
    //gsl_matrix_transpose (&O_vs_v.matrix);
    //gslu_mv (&abc_sensor.vector, &O_vs_v.matrix, &abc_v.vector);

    //rotate velodyne body velocities into local frame
    //double R_sl[9];
    //so3_rotxyz (R_sl, state.rph); //state.rph = rph_ls
    //gsl_matrix_view R_sl_v = gsl_matrix_view_array (R_sl, 3, 3);
    //GSLU_VECTOR_VIEW (xyz_dot_sensor,3, {0});
    //gslu_mv (&xyz_dot_sensor.vector, &R_sl_v.matrix, &uvw_sensor.vector);
    //memcpy (&(state.xyz_dot), xyz_dot_sensor.vector.data, 3*sizeof (double));

    // set euler rates
    //so3_body2euler (abc_sensor.vector.data, state.rph, state.rph_dot, NULL);

    velodyne_collector_push_state (self->collector, state);
    do_push_motion = 0;
  }

  return 1;
}




static int dropped_packets = 0;
static int64_t dropped_utime;

static void *
velodyne_work_thread (void *user)
{
  state_t *self = (state_t *) user;

  while (1) {

    int MAX_QUEUE_SIZE = 650;
    while (g_async_queue_length(self->velodyne_message_queue) > MAX_QUEUE_SIZE) {

      void *msg = g_async_queue_pop (self->velodyne_message_queue);

      // Check to see whether the thread should exit
      g_mutex_lock (self->mutex);
      if (self->velodyne_work_thread_exit) {
        fprintf (stdout, "got exit command\n");
        g_mutex_unlock (self->mutex);
        g_thread_exit (NULL);
      }
      g_mutex_unlock (self->mutex);

      // Don't exit. Keep going
      if (msg == &(self->velodyne_work_thread_exit))
        continue;

      senlcm_velodyne_t *v = (senlcm_velodyne_t *) msg;

      senlcm_velodyne_t_destroy(v);

      int64_t now = bot_timestamp_now();
      dropped_packets++;
      double dt = (now - dropped_utime) / 1000000.0;
      if (dt > 1 || dropped_packets % 1000 == 0) {
        fprintf(stderr, "WARNING: dropping velodyne packets (total dropped: %d)\n", dropped_packets);
        dropped_utime = now;
      }
    }

    void *msg = g_async_queue_pop (self->velodyne_message_queue);

    // Check to see whether the thread should exit
    g_mutex_lock (self->mutex);
    if (self->velodyne_work_thread_exit) {
      fprintf (stdout, "got exit command\n");
      g_mutex_unlock (self->mutex);
      g_thread_exit (NULL);
    }
    g_mutex_unlock (self->mutex);

    // Don't exit. Keep going
    senlcm_velodyne_t *v = (senlcm_velodyne_t *) msg;

    self->last_msg_count = v->utime;

    // Process a velodyne_t (chunk of scans)
    //cout << "about to process " << v->utime << "\n";
    //process_velodyne(self, v);
    senlcm_velodyne_t_destroy(v);
  }

  return NULL;
}

static void
usage(const char *progname)
{
  fprintf (stderr, "usage: %s [options]\n"
      "\n"
      "  -c, --config PATH      Location of config file\n"
      , g_path_get_basename(progname));
}

int
main(int argc, char ** argv)
{
  char *optstring = "hc:";
  int c;
  struct option long_opts[] = {
      {"help", no_argument, 0, 'h'},
      {"config", required_argument, 0, 'c'},
  };

  int exit_code = 0;
  char *config_file =NULL;// g_strdup("velodyne.cfg");

  while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0)
  {
    switch (c)
    {
    case 'c':
      free(config_file);
      config_file = g_strdup(optarg);
      break;
    case 'h':
    default:
      usage(argv[0]);
      return 1;
    }
  }

  state_t* self = new state_t();
  //  state_t *self = calloc(1,sizeof(state_t));
  self->lcm = lcm_create(NULL);//"udpm://239.255.76.67:7667?recv_buf_size=100000");
  if(!self->lcm)
    return 1;

  if (config_file){
    fprintf(stderr,"Reading velodyne config from file\n");
    char config_path[2048];
    sprintf(config_path, "%s/%s", getConfigPath(),config_file);
    printf("%s is config_path\n",config_path);
    self->param = bot_param_new_from_file(config_path);

    if(!self->param){
      fprintf(stderr, "Couldn't get bot param from file %s\n",
          config_path);
      return 0;
    }
  }else {


    self->param = bot_param_new_from_server(self->lcm, 0);
    if (self->param == NULL) {
      fprintf(stderr, "Couldn't get bot param from server.\n");
      return 1;
    }
  }

  self->frames = bot_frames_get_global (self->lcm, self->param);


  char *velodyne_model = bot_param_get_str_or_fail (self->param, "calibration.velodyne.model");
  char *calib_file = bot_param_get_str_or_fail (self->param, "calibration.velodyne.intrinsic_calib_file");
  char calib_file_path[2048];
  sprintf(calib_file_path, "%s/%s", getConfigPath(), calib_file);

  if (0 == strcmp (velodyne_model, VELODYNE_HDL_32E_MODEL_STR))
    self->calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_32E, calib_file_path);
  else if (0 == strcmp (velodyne_model, VELODYNE_HDL_64E_S1_MODEL_STR))
    self->calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_64E_S1, calib_file_path);
  else if (0 == strcmp (velodyne_model, VELODYNE_HDL_64E_S2_MODEL_STR))
    self->calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_64E_S2, calib_file_path);
  else
    fprintf (stderr, "ERROR: Unknown Velodyne model \'%s\'", velodyne_model);

  free (velodyne_model);
  free (calib_file);

  g_thread_init (NULL);

  //  self->collector = velodyne_laser_return_collector_create (1, 0, 2* M_PI); // full scan
  self->collector = velodyne_laser_return_collector_create (0, M_PI, 2* M_PI); // front facing pixels only

  // Create an asynchronous queue for messages
  self->velodyne_message_queue = g_async_queue_new ();

  self->velodyne_data_circ = bot_ptr_circular_new (VELODYNE_DATA_CIRC_SIZE,
      circ_free_velodyne_data, self);


  self->velodyne_work_thread = g_thread_create (velodyne_work_thread, self, TRUE, NULL);
  self->velodyne_work_thread_exit = 0;
  self->mutex = g_mutex_new();


  senlcm_velodyne_list_t_subscribe (self->lcm, "VELODYNE_LIST", on_velodyne_list, self);
  bot_core_pose_t_subscribe (self->lcm, "POSE", on_bot_pose, self);

  bot_core_pose_t_subscribe (self->lcm, "POSE_TRIGGER", on_pose_trigger, self);

  self->capture_rate = rate_new();

  bot_glib_mainloop_attach_lcm (self->lcm);

  self->mainloop = g_main_loop_new (NULL, FALSE);

  bot_signal_pipe_glib_quit_on_kill (self->mainloop);

  g_main_loop_run (self->mainloop);


  // Exiting
  fprintf (stderr, "Handler exiting,\n");

  // Stop the velodyne_work thread
  g_mutex_lock (self->mutex);
  self->velodyne_work_thread_exit = 1;
  g_mutex_unlock (self->mutex);
  g_async_queue_push (self->velodyne_message_queue,
      &(self->velodyne_work_thread_exit));


  g_thread_join (self->velodyne_work_thread);

  g_mutex_free (self->mutex);

  bot_glib_mainloop_detach_lcm (self->lcm);

  int num_freed = 0;
  for (senlcm_velodyne_t *msg = (senlcm_velodyne_t*)   g_async_queue_try_pop (self->velodyne_message_queue); 
      msg; msg = (senlcm_velodyne_t*) g_async_queue_try_pop (self->velodyne_message_queue) ) {


    if (msg ==  (senlcm_velodyne_t*) &(self->velodyne_work_thread_exit))
      continue;

    senlcm_velodyne_t_destroy(msg);
    num_freed++;
  }

  g_async_queue_unref (self->velodyne_message_queue);

  return 0;
}
