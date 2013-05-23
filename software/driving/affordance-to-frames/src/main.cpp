#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <glib.h>
#include <sys/time.h>
#include <getopt.h>
#include <string.h>
#include <unistd.h>

#ifdef __APPLE
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif


#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <lcmtypes/drc_affordance_collection_t.h>
#include <vector>
#include <algorithm> 

#define TIMER_PERIOD_MSEC 100

typedef struct _state_t {
    lcm_t *lcm;
    BotParam * param;
    BotFrames * frames;
    GMainLoop *mainloop; 
    drc_affordance_t *car_affordance;
    int verbose ;
    int simulate_manip_map;
  
} state_t;

void publish_frame_update(state_t *self);

static gboolean
status_timer (gpointer data)
{
    state_t *self = (state_t *) data;
    publish_frame_update(self);
    return TRUE;
}

static void usage (int argc, char **argv)
{
    fprintf (stdout, "Usage: %s [options]\n"
             "\n"
             " -v, --verbose    verbose output\n"
             " -s, --simulate-manipmap        Simulate Manip-map\n", 
             argv[0]);
}

int has_affordance_position_updated(drc_affordance_t *c_aff, drc_affordance_t *n_aff){
    if(c_aff->origin_xyz[0] != n_aff->origin_xyz[0] || 
       c_aff->origin_xyz[1] != n_aff->origin_xyz[1] ||
       c_aff->origin_xyz[2] != n_aff->origin_xyz[2] ||
       c_aff->origin_rpy[0] != n_aff->origin_rpy[0] ||
       c_aff->origin_rpy[1] != n_aff->origin_rpy[1] ||
       c_aff->origin_rpy[2] != n_aff->origin_rpy[2]){
        return 1;
    }
    return 0;
}

void publish_frame_update(state_t *self){
    if(self->car_affordance == NULL)
        return;
    BotTrans local_to_body;
    bot_frames_get_trans_with_utime(self->frames, bot_frames_get_root_name(self->frames), "body",
                                    self->car_affordance->utime, &local_to_body);

    //fprintf(stderr, "Local to Body : %f,%f,%f\n", local_to_body.trans_vec[0], local_to_body.trans_vec[1], 
    //local_to_body.trans_vec[2]);

    BotTrans car_to_local; 
    car_to_local.trans_vec[0] = self->car_affordance->origin_xyz[0];
    car_to_local.trans_vec[1] = self->car_affordance->origin_xyz[1];
    car_to_local.trans_vec[2] = self->car_affordance->origin_xyz[2];
  
    bot_roll_pitch_yaw_to_quat( self->car_affordance->origin_rpy, car_to_local.rot_quat);
  
    BotTrans car_to_body;
    bot_trans_apply_trans_to(&local_to_body, &car_to_local, &car_to_body);

    bot_core_rigid_transform_t car_to_body_tf;
    car_to_body_tf.utime = self->car_affordance->utime;

    memcpy (car_to_body_tf.trans, car_to_body.trans_vec, 3*sizeof(double));
    memcpy (car_to_body_tf.quat, car_to_body.rot_quat, 4*sizeof(double));

    bot_core_rigid_transform_t_publish (self->lcm, "CAR_TO_BODY", &car_to_body_tf);
  
    /*bot_core_rigid_transform_t global_to_local;
      global_to_local.utime = slam_pose.utime;
    
      BotTrans bt_global_to_local;

      // If we are solving for the pose, publish the SLAM pose and the GLOBAL_TO_LOCAL as identity
      if (self->solve_for_pose == TRUE) {
      bot_core_pose_t_publish(self->lcm, "POSE", &slam_pose);
        
      // Set the global-to-local transformation to the identity transformation
      bot_trans_set_identity( &bt_global_to_local);
      }
      else {
      // The transformation from GLOBAL to LOCAL is defined as
      // T_global-to-local = T_body-to-local * T_global-to-body
      //                   = T_body-to-local * inv(T_body-to-global)
      BotTrans bt_body_to_global;
      BotTrans bt_global_to_body;
      BotTrans bt_body_to_local;
 
      bot_trans_set_from_quat_trans (&bt_body_to_global, slam_pose.orientation, slam_pose.pos);
      bot_trans_set_from_quat_trans (&bt_body_to_local, self->last_bot_pose->orientation, self->last_bot_pose->pos);
      bot_trans_invert (&bt_body_to_global);
      bot_trans_copy (&bt_global_to_body, &bt_body_to_global);
      bot_trans_apply_trans_to (&bt_body_to_local, &bt_global_to_body, &bt_global_to_local);
      }
      memcpy (global_to_local.trans, bt_global_to_local.trans_vec, 3*sizeof(double));
      memcpy (global_to_local.quat, bt_global_to_local.rot_quat, 4*sizeof(double));


      int64_t utime_now = bot_timestamp_now();
      if ((utime_now - self->last_global_to_local_utime) > 1e6/PUBLISH_GLOBAL_TO_LOCAL_HZ) {
      bot_core_rigid_transform_t_publish (self->lcm, "GLOBAL_TO_LOCAL", &global_to_local);
      self->last_global_to_local_utime = utime_now;
      }*/

}

static void on_affordance_collection(const lcm_recv_buf_t * buf, const char *channel, 
                                     const drc_affordance_collection_t *msg, void *user){
    state_t *self = (state_t *) user;
    for(int i = 0; i < msg->naffs; i++){
        if(!strcmp("car", msg->affs[i].otdf_type)){
            if(self->car_affordance != NULL){
                //do a diff - publsih the frame update if they are different
                int different = has_affordance_position_updated(self->car_affordance, &msg->affs[i]);
                if(different){
                    drc_affordance_t_destroy(self->car_affordance);
                    self->car_affordance = drc_affordance_t_copy(&msg->affs[i]);      
                    publish_frame_update(self);
                    fprintf(stderr, "Updating frames\n");
                }
            }
            else{
                self->car_affordance = drc_affordance_t_copy(&msg->affs[i]);      
                publish_frame_update(self);
                fprintf(stderr, "Updating frames\n");
            }
        }
    }
}



int main (int argc, char **argv) {
   
    setlinebuf(stdout);

    state_t *self = (state_t *) calloc (1, sizeof (state_t));

    char *optstring = "vs";
    char c;
    struct option long_opts[] = {
        { "verbose", no_argument, 0, 'v'},
        { "simulate-manipmap", no_argument, 0, 's'},
        { 0, 0, 0, 0 }
    };

    int arc = 0;

    self->car_affordance = NULL;
    
    while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0){
        switch (c) {
        case 's':
            fprintf(stderr, "Simulating manip-map\n");
            self->simulate_manip_map = 1;
            break;
        case 'v':
            self->verbose = 1;
            break;
        default:
            usage (argc, argv);
            return 1;
        };
    }


    
    if(self->simulate_manip_map){
        //subscribe to manip map range commands
    }
    
    self->lcm = bot_lcm_get_global (NULL);
    if (!self->lcm) {
        fprintf (stderr, "Error getting LCM\n");
        //goto fail;
        return -1;
    }

    self->param = bot_param_new_from_server (self->lcm, 0);
    if (!self->param) {
        fprintf (stderr, "Error getting BotParam\n");
        //goto fail;
        return -1;
    }

    self->frames = bot_frames_new (self->lcm, self->param);
    if (!self->frames) {
        fprintf (stderr, "Error getting BotFrames\n");
        //goto fail;
        return -1;
    }

    drc_affordance_collection_t_subscribe(self->lcm, "AFFORDANCE_COLLECTION", on_affordance_collection, self);
    
    self->mainloop = g_main_loop_new (NULL, TRUE);
    
    g_timeout_add (TIMER_PERIOD_MSEC, status_timer, self);

    // Connect LCM to the mainloop
    bot_glib_mainloop_attach_lcm (self->lcm);

    g_main_loop_run (self->mainloop);

    return 1;
}
