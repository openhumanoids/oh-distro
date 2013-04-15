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
#include <bot_lcmgl_client/lcmgl.h>
#include <occ_map/PixelMap.hpp>

#include <lcmtypes/occ_map_pixel_map_t.h>
#include <lcmtypes/drc_driving_control_params_t.h>

#define DEFAULT_GOAL_DISTANCE 10.0

#define TIMER_PERIOD_MSEC 50

#define CAR_FRAME "body" // placeholder until we have the car frame


typedef struct _state_t {
    lcm_t *lcm;
    BotParam * param;
    BotFrames * frames;
    GMainLoop *mainloop; 
    guint controller_timer_id;

    occ_map_pixel_map_t *cost_map_last;
    
    bot_lcmgl_t *lcmgl_goal; 

    int have_valid_goal;
    double cur_goal[3];

    // Control parameters
    double goal_distance;
    double kp_steer;

    int verbose;
} state_t;


void draw_goal(state_t *self){
    if (self->lcmgl_goal) {
        bot_lcmgl_t *lcmgl = self->lcmgl_goal; 

        double xyz_car_car[] = {0, 0, 0};
        double xyz_car_local[3];

        bot_frames_transform_vec (self->frames, CAR_FRAME, "local", xyz_car_car, xyz_car_local);
        xyz_car_local[2] = 0.0;

        lcmglColor3f (0.0, 0.0, 1.0);
        lcmglCircle (xyz_car_local, self->goal_distance);
        //fprintf (stdout, "Drawing circle at xyz = [%.2f %.2f %.2f] with goal_distance = %.2f\n",
        //        xyz_car_local[0], xyz_car_local[2], xyz_car_local[2], self->goal_distance);

        bot_lcmgl_line_width(lcmgl, 5);
        lcmglColor3f(1.0, 0.0, 0.0);

        lcmglCircle(self->cur_goal, 0.6);
        bot_lcmgl_switch_buffer(self->lcmgl_goal);
    }  
}

void draw_goal_range (state_t *self) 
{
    if (self->lcmgl_goal) {
        double xyz_car_car[] = {0, 0, 0};
        double xyz_car_local[3];

        bot_frames_transform_vec (self->frames, CAR_FRAME, "local", xyz_car_car, xyz_car_local);
        xyz_car_local[2] = 0.0;

        bot_lcmgl_t *lcmgl = self->lcmgl_goal;
        lcmglColor3f (1.0, 0.0, 0.0);
        lcmglCircle (xyz_car_local, self->goal_distance);
        bot_lcmgl_switch_buffer (self->lcmgl_goal);
    }
}

static void
on_driving_control_params (const lcm_recv_buf_t *rbuf, const char *channel,
                           const drc_driving_control_params_t *msg, void *user)
{

    state_t *self = (state_t *) user;

    if (self->verbose) {
        fprintf (stdout, "Changing control parameters: lookahead_dist from %.2f --> %.2f\n",
                 self->goal_distance, msg->lookahead_distance);
        fprintf (stdout, "                             kp_steer from       %.2f --> %.2f\n",
                 self->kp_steer, msg->kp_steer);
    }

    self->goal_distance = msg->lookahead_distance;
    self->kp_steer = msg->kp_steer;
}


static void
on_terrain_dist_map (const lcm_recv_buf_t *rbuf, const char *channel,
                     const occ_map_pixel_map_t *msg, void *user)
{

    state_t *self = (state_t *) user;

    if (self->cost_map_last)
        occ_map_pixel_map_t_destroy (self->cost_map_last);

    self->cost_map_last = occ_map_pixel_map_t_copy (msg);

    return;

}

static int
find_goal (occ_map::FloatPixelMap *fmap, state_t *self)
{

    // Find the best goal
    int found_goal = 0;
    double max_reward = 0;
    double xy_goal[2] = {0, 0};
    double xyz_car_car[] = {0, 0, 0};
    double xyz_car_local[3];

    
    bot_frames_transform_vec (self->frames, CAR_FRAME, "local", xyz_car_car, xyz_car_local);
    double x_arc, y_arc;
    double angle_deg;
    for (int i=0; i<360; i++) {
        angle_deg = i;
        double xy_arc[2];
        xy_arc[0] = xyz_car_local[0] + self->goal_distance * cos (bot_to_radians (angle_deg));
        xy_arc[1] = xyz_car_local[1] + self->goal_distance * sin (bot_to_radians (angle_deg));

        if (!fmap->isInMap (xy_arc))
            continue;

        double val = fmap->readValue (xy_arc);
        if (val > max_reward) {
            found_goal = 1;
            xy_goal[0] = xy_arc[0];
            xy_goal[1] = xy_arc[1];
            max_reward = val;
        }
    }

    if (found_goal) {
        self->cur_goal[0] = xy_goal[0];
        self->cur_goal[1] = xy_goal[1];
        self->cur_goal[2] = 0;
    }

    return found_goal;
}

static void
perform_emergency_stop (state_t *self)
{

    fprintf (stdout, "Performing emergency stop!!!!\n");

}


static gboolean
on_controller_timer (gpointer data)
{

    state_t *self = (state_t *) data;

    if (!self->cost_map_last) {
        if (self->verbose)
            //fprintf (stdout, "No valid terrain classification map\n");
        
        return TRUE;
    }

    
    occ_map::FloatPixelMap *fmap = new occ_map::FloatPixelMap (self->cost_map_last);

    //draw_goal_range (self);    
    double xyz_goal[3];
    if (find_goal (fmap, self)) {
         draw_goal (self);
         self->have_valid_goal = 1;
    }
    else {
        if (self->verbose)
            fprintf (stdout, "Unable to find goal in PixMap\n");
    }
    
    // We should verify that current goal is valid
    if (self->have_valid_goal) {
        // This is where we check
    }

    if (self->have_valid_goal) {

        // Compute the steering command
        double xyz_goal_car[3];
        bot_frames_transform_vec (self->frames, "local", CAR_FRAME, self->cur_goal, xyz_goal_car);
        double heading_error = bot_fasttrig_atan2 (xyz_goal_car[1], xyz_goal_car[0]);
        double steering_input = self->kp_steer * heading_error;
        
        //if (self->verbose)
        //fprintf (stdout, "Steering control: Error = %.2f deg, steering_input = %.2f\n",
        //         heading_error, steering_input);
    }
    else
        perform_emergency_stop (self);

    return TRUE;
}




static void
driving_control_destroy (state_t *self)
{
    if (!self)
        return;

    if (self->cost_map_last)
        occ_map_pixel_map_t_destroy (self->cost_map_last);

    if (self->mainloop)
        g_main_loop_unref (self->mainloop);

    free (self);
}


static void usage (int argc, char **argv)
{
    fprintf (stdout, "Usage: %s [options]\n"
             "\n"
             " -v, --verbose    verbose output\n"
             "\n",
             argv[0]);
}


int main (int argc, char **argv) {
   
    setlinebuf(stdout);

    state_t *self = (state_t *) calloc (1, sizeof (state_t));
    self->goal_distance = DEFAULT_GOAL_DISTANCE;

    char *optstring = "vd:";
    char c;
    struct option long_opts[] = {
        { "verbose", no_argument, 0, 'v'},
        { "distance", required_argument, 0, 'd'},
        { 0, 0, 0, 0 }
    };

    while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0)
    {
        switch (c) {
        case 'd':
            self->goal_distance = strtod (optarg, NULL);
        case 'v':
            self->verbose = 1;
            break;
        default:
            usage (argc, argv);
            return 1;
        };
    }

    self->verbose = 1;

    self->lcm = bot_lcm_get_global (NULL);
    if (!self->lcm) {
        fprintf (stderr, "Error getting LCM\n");
        goto fail;
    }

    self->param = bot_param_new_from_server (self->lcm, 0);
    if (!self->param) {
        fprintf (stderr, "Error getting BotParam\n");
        goto fail;
    }

    self->frames = bot_frames_new (self->lcm, self->param);
    if (!self->frames) {
        fprintf (stderr, "Error getting BotFrames\n");
        goto fail;
    }

    
    // Get default parameters
    self->goal_distance = bot_param_get_double_or_fail (self->param, "driving.control.lookahead_distance_default");
    self->kp_steer = bot_param_get_double_or_fail (self->param, "driving.control.kp_steer_default");


    self->lcmgl_goal = bot_lcmgl_init (self->lcm, "DRIVING_GOAL");

    occ_map_pixel_map_t_subscribe (self->lcm, "TERRAIN_DIST_MAP", on_terrain_dist_map, self);
    drc_driving_control_params_t_subscribe (self->lcm, "DRIVING_CONTROL_PARAMS_DESIRED", on_driving_control_params, self);
    
    self->mainloop = g_main_loop_new (NULL, TRUE);

    // Control timer
    self->controller_timer_id = g_timeout_add (TIMER_PERIOD_MSEC, on_controller_timer, self);
    
    // Connect LCM to the mainloop
    bot_glib_mainloop_attach_lcm (self->lcm);

    g_main_loop_run (self->mainloop);

    return 1;

 fail:
    driving_control_destroy (self);
    return 0;
}
