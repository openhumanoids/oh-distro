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
#include <lcmtypes/drc_driving_control_cmd_t.h>
#include <lcmtypes/drc_utime_t.h>
#include <lcmtypes/drc_driving_cmd_t.h>
#include <vector>
#include <algorithm> 

#define DEFAULT_GOAL_DISTANCE 10.0

#define TIMER_PERIOD_MSEC 50

#define STOP_TIME_GAP_SEC 50.0
#define ACCELERATOR_TIME_GAP_SEC 2.5
#define TIMEOUT_THRESHOLD 1.0
#define MAX_THROTTLE 0.03
#define MAX_BRAKE    0.02

#define CAR_FRAME "body" // placeholder until we have the car frame

typedef enum {
    IDLE, MAP_TIMEOUT, DRIVING
} controller_state_t;

typedef struct{
    double xy[2];
    double score;
} pos_t;


typedef struct _state_t {
    lcm_t *lcm;
    BotParam * param;
    BotFrames * frames;
    GMainLoop *mainloop; 
    guint controller_timer_id;

    controller_state_t *curr_state;
  
    occ_map_pixel_map_t *cost_map_last;
    
    drc_driving_cmd_t *last_driving_cmd; 

    bot_lcmgl_t *lcmgl_goal; 
    bot_lcmgl_t *lcmgl_rays; 
    int64_t utime;

    int64_t time_applied_to_brake;
    
    int64_t time_applied_to_accelerate;

    double throttle_duration;
    double throttle_ratio;

    double drive_duration; 
    int64_t drive_start_time;

    double goal_distance;
    double kp_steer;

    int have_valid_goal;
    double cur_goal[3];
  
    int accelerator_utime; 
    int stop_utime;
    // Control parameters

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

static bool score_compare(const std::pair<int, pos_t>& lhs, const std::pair<int, pos_t>& rhs) { 
    return lhs.second.score > rhs.second.score;
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
on_utime (const lcm_recv_buf_t *rbuf, const char *channel,
	  const drc_utime_t *msg, void *user)
{
    state_t *self = (state_t *) user;
    self->utime = msg->utime;
  
}

static void
on_driving_command (const lcm_recv_buf_t *rbuf, const char *channel,
                    const drc_driving_cmd_t *msg, void *user)
{
    state_t *self = (state_t *) user;
    fprintf(stderr, "Drive Command received\n");

    if(self->last_driving_cmd){
        drc_driving_cmd_t_destroy(self->last_driving_cmd);
    }

    self->last_driving_cmd = drc_driving_cmd_t_copy(msg);

    self->drive_duration = msg->drive_duration; 
    self->kp_steer = msg->kp_steer;
    self->throttle_ratio = msg->throttle_ratio;
    self->throttle_duration = msg->throttle_duration;
    self->goal_distance = msg->lookahead_dist;
    self->drive_start_time = self->utime;
    self->time_applied_to_accelerate  = 0;
    
    
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

static int
find_goal_enhanced (occ_map::FloatPixelMap *fmap, state_t *self)
{
    occ_map::FloatPixelMap *cost_map = new occ_map::FloatPixelMap (fmap);

    //invert the values
    for(int i = 0; i < fmap->dimensions[0]; i++){
        for(int j = 0; j < fmap->dimensions[1]; j++){
            int ixy[2] = {i,j};
            float val = fmax(0.1, fmap->readValue(ixy));
            cost_map->writeValue(ixy, 1/val);
        }
    }
    
    // Find the best goal
    int found_goal = 0;
    //double max_reward = 10000;

    double min_cost = 10000;
    double xy_goal[2] = {0, 0};
    double xyz_car_car[] = {0, 0, 0};
    double xyz_car_local[3];

    BotTrans car_to_body; 
    car_to_body.trans_vec[0] = 0;
    car_to_body.trans_vec[1] = -0.3;
    car_to_body.trans_vec[2] = 0;
    
    double rpy[3] = {0};
    bot_roll_pitch_yaw_to_quat(rpy, car_to_body.rot_quat);

    BotTrans body_to_local;

    bot_frames_get_trans(self->frames, "body", "local", 
			 &body_to_local);

    BotTrans car_to_local; 
    bot_trans_apply_trans_to(&body_to_local, &car_to_body, &car_to_local);
    
    xyz_car_local[0] = car_to_local.trans_vec[0];
    xyz_car_local[1] = car_to_local.trans_vec[1];

    //fprintf(stderr, "Car Frame to Local : %f, %f\n", xyz_car_local[0], xyz_car_local[1]);

    //bot_frames_transform_vec (self->frames, CAR_FRAME, "local", xyz_car_car, xyz_car_local);

    //fprintf(stderr, "Car Frame to Local : %f, %f\n", xyz_car_local[0], xyz_car_local[1]);
    
    double x_arc, y_arc;
    double angle_deg;

    double min_dist = fmin(6.0, self->goal_distance);
    double max_dist = self->goal_distance;

    //we should actually ckeck 
    bot_lcmgl_t *lcmgl = self->lcmgl_rays; 
    lcmglColor3f (1.0, 0.0, 0.0);
    bot_lcmgl_line_width(lcmgl, 5);

    int skip = 2;

    for (int i=0; i<360; i+=skip) {
        //get the rays (starting from some distance onwards - to skip too close obstacles 

        angle_deg = i;
        double xy_arc[2];
        xy_arc[0] = xyz_car_local[0] + self->goal_distance * cos (bot_to_radians (angle_deg));
        xy_arc[1] = xyz_car_local[1] + self->goal_distance * sin (bot_to_radians (angle_deg));

        if (!cost_map->isInMap (xy_arc))
            continue;

        double val = cost_map->readValue (xy_arc);
        if (val < min_cost) {
            found_goal = 1;
            xy_goal[0] = xy_arc[0];
            xy_goal[1] = xy_arc[1];
            min_cost = val;
        }
    }

    
    
    int no_beams = 360.0 / skip + 1;

    std::vector<std::pair<int, pos_t> > scores;

    //std::map<int, pos_t> pos_map;

    int count = 0;

    for (int i=0; i<360; i+=skip) {
        count++;
        //get the rays (starting from some distance onwards - to skip too close obstacles 
        angle_deg = i;
        double xy_arc_min[2];
        xy_arc_min[0] = xyz_car_local[0] + min_dist * cos (bot_to_radians (angle_deg));
        xy_arc_min[1] = xyz_car_local[1] + min_dist * sin (bot_to_radians (angle_deg));
      
        double xy_arc_max[2];
        xy_arc_max[0] = xyz_car_local[0] + max_dist * cos (bot_to_radians (angle_deg));
        xy_arc_max[1] = xyz_car_local[1] + max_dist * sin (bot_to_radians (angle_deg));
      
        if (!cost_map->isInMap (xy_arc_min) || !cost_map->isInMap (xy_arc_max))
            continue;
      
        double collision_point[2];
      
        double threshold = 1/2.0;

        bool collision = cost_map->collisionCheck(xy_arc_min, xy_arc_max, threshold, collision_point); 
      
        double ray_dist = 0;

        bot_lcmgl_begin(lcmgl, GL_LINES);

        double goal_pos[2];

        if(collision){
            goal_pos[0] = collision_point[0];
            goal_pos[1] = collision_point[1];

            bot_lcmgl_vertex3f(lcmgl, xyz_car_local[0], xyz_car_local[1], 0);
            bot_lcmgl_vertex3f(lcmgl, collision_point[0], collision_point[1], 0);
            ray_dist = hypot(collision_point[0] -  xyz_car_local[0], collision_point[1] - xyz_car_local[1]);
        }
        else{
            goal_pos[0] = xy_arc_max[0];
            goal_pos[1] = xy_arc_max[1];

            bot_lcmgl_vertex3f(lcmgl, xyz_car_local[0], xyz_car_local[1], 0);
            bot_lcmgl_vertex3f(lcmgl, xy_arc_max[0], xy_arc_max[1], 0);
            ray_dist = hypot(xy_arc_max[0] -  xyz_car_local[0], xy_arc_max[1] - xyz_car_local[1]);
        }

        //calculate a score 
        
        double dist_from_goal = hypot(xy_goal[0] - goal_pos[0], xy_goal[1] - goal_pos[1]);

        //scores.push_back(std::make_pair<int, double>(i,dist_from_goal));
        pos_t pos;
        pos.xy[0] = goal_pos[0];
        pos.xy[1] = goal_pos[1];
        pos.score = dist_from_goal;
        scores.push_back(std::make_pair<int, pos_t>(i,pos));

        bot_lcmgl_end(lcmgl);

        //score each ray - to find the best one 
             
      
        //fprintf(stderr, "[%d] Dist : %f\n", i, ray_dist); 
    }

    //std::vector<std::pair<int, double> >::iterator it = std::max_element(scores.begin(), scores.end(), score_compare);
    std::vector<std::pair<int, pos_t> >::iterator it = std::max_element(scores.begin(), scores.end(), score_compare);
    fprintf(stderr, "Max Ind : %d => Score : %f => Pos [%f,%f]\n", it->first, it->second.score, it->second.xy[0], it->second.xy[1]);

    fprintf(stderr, "Goal : %f,%f\n", xy_goal[0], xy_goal[1]);

    fprintf(stderr, "Count  %d No beams : %d\n", count, no_beams);

    bot_lcmgl_switch_buffer(lcmgl);

    /**/

    if (found_goal) {
        /*self->cur_goal[0] = xy_goal[0];
        self->cur_goal[1] = xy_goal[1];
        self->cur_goal[2] = 0;*/
        self->cur_goal[0] = it->second.xy[0];
        self->cur_goal[1] = it->second.xy[1];
        self->cur_goal[2] = 0;
    }

    delete cost_map;

    return found_goal;
}

static void
perform_emergency_stop (state_t *self)
{

    fprintf (stdout, "Performing emergency stop!!!!\n");
    drc_driving_control_cmd_t msg;
    msg.utime = bot_timestamp_now();
    msg.type = DRC_DRIVING_CONTROL_CMD_T_TYPE_BRAKE; //_DELTA_STEERING ;
    msg.brake_value = 1.0;
    drc_driving_control_cmd_t_publish(self->lcm, "DRC_DRIVING_COMMAND", &msg);
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

    //we should check if the map is stale - because of some issue with the controller   

    if((self->utime - self->cost_map_last->utime)/1.0e6 > TIMEOUT_THRESHOLD){
        fprintf(stderr, "Error - Costmap too old - asking for EStop\n");
      
        drc_driving_control_cmd_t msg;
        msg.utime = bot_timestamp_now();
        msg.type = DRC_DRIVING_CONTROL_CMD_T_TYPE_BRAKE; //_DELTA_STEERING ;
        msg.brake_value = 1.0;
        drc_driving_control_cmd_t_publish(self->lcm, "DRC_DRIVING_COMMAND", &msg);

        return TRUE;
    }

    if(self->drive_duration < 0){
        //we are not required to drive - return TRUE
      
        return TRUE;
    }

    if((self->utime - self->drive_start_time)/1.0e6 > self->drive_duration){
        drc_driving_control_cmd_t msg;
        msg.utime = bot_timestamp_now();
        msg.type = DRC_DRIVING_CONTROL_CMD_T_TYPE_BRAKE; //_DELTA_STEERING ;
        msg.brake_value = 1.0;
        drc_driving_control_cmd_t_publish(self->lcm, "DRC_DRIVING_COMMAND", &msg);
        self->drive_duration = -1;
        return TRUE;
    }

    static int64_t last_utime = 0;
    
    fprintf(stderr, "Drive start time : %f\n", (self->utime - self->drive_start_time)/1.0e6);

    occ_map::FloatPixelMap *fmap = new occ_map::FloatPixelMap (self->cost_map_last);

    //draw_goal_range (self);    
    double xyz_goal[3];
    if (find_goal_enhanced (fmap, self)) {
        draw_goal (self);
        self->have_valid_goal = 1;
    }
    else {
        if (self->verbose)
            fprintf (stdout, "Unable to find goal in PixMap - Stopping\n");
        
        perform_emergency_stop(self);
        self->drive_duration = -1;
        return TRUE;
    }
    
    // We should verify that current goal is valid
    if (self->have_valid_goal) {
        // This is where we check
    }

    if (self->have_valid_goal) {
        // Compute the steering command
        double xyz_goal_car[3];
	bot_frames_transform_vec (self->frames, "local", "body", self->cur_goal, xyz_goal_car);

	BotTrans goal_to_body;
	goal_to_body.trans_vec[0] = xyz_goal_car[0];
	goal_to_body.trans_vec[1] = xyz_goal_car[1];
	goal_to_body.trans_vec[2] = 0;

	double rpy[3] = {0};
	bot_roll_pitch_yaw_to_quat(rpy, goal_to_body.rot_quat);
	
	BotTrans body_to_car; 
	body_to_car.trans_vec[0] = 0;
	body_to_car.trans_vec[1] = 0.3;
	body_to_car.trans_vec[2] = 0;

	bot_roll_pitch_yaw_to_quat(rpy, body_to_car.rot_quat);
	
	BotTrans goal_to_car; 
	bot_trans_apply_trans_to(&body_to_car, &goal_to_body, &goal_to_car);

        double heading_error = bot_fasttrig_atan2 (goal_to_car.trans_vec[1], goal_to_car.trans_vec[0]);  // xyz_goal_car[1], xyz_goal_car[0]);
        double steering_input = self->kp_steer * heading_error;

	double steering_angle_delta = steering_input;
	
	//angle at which accceleration should be down - this might be tough when actually controlling the car 
	//using the robot 
	double steering_no_angle;
	if(steering_angle_delta >= 0){
            steering_no_angle = bot_to_radians(4);
	}
	else{
            steering_no_angle = bot_to_radians(-4);
	}

	//lets assume a driving for a given time model 
	
	double throttle_ratio = (steering_no_angle - steering_angle_delta) / steering_no_angle;
	throttle_ratio = fmin(1.0, fmax(throttle_ratio, -2.0));
	double throttle_val = 0;
	double brake_val = 0;
	
	if((self->utime - self->drive_start_time)/1.0e6 < self->throttle_duration){
            throttle_val = self->throttle_ratio;//MAX_THROTTLE;
	}

	/*if(throttle_ratio >=0){
        //throttle_val = 0.04 + MAX_THROTTLE * throttle_ratio;
        if(throttle_ratio >0 && last_utime > 0)
        self->time_applied_to_accelerate += (self->utime - last_utime);
	}
	else{
        brake_val = 0; //MAX_BRAKE * fabs(throttle_ratio);
        if(last_utime > 0)
        self->time_applied_to_brake += (self->utime - last_utime);
        }*/
        double max_angle = 90;
        steering_input = fmax(bot_to_radians(-max_angle), fmin(bot_to_radians(max_angle), steering_angle_delta));

	fprintf (stdout, "Steering control: Error = %.2f deg, Signal = %.2f (deg) Throttle : %f Brake: %f\n",
		 bot_to_degrees(heading_error), bot_to_degrees(steering_input), 
		 throttle_val, brake_val);

	if(self->time_applied_to_brake > 0 && self->time_applied_to_accelerate)
            fprintf(stderr, "Time accelerating : %f, Time braking : %f\n", self->time_applied_to_accelerate / 1.0e6, 
                    self->time_applied_to_brake / 1.0e6);

	drc_driving_control_cmd_t msg;
	msg.utime = bot_timestamp_now();
	msg.type = DRC_DRIVING_CONTROL_CMD_T_TYPE_DRIVE; //_DELTA_STEERING ;
	msg.steering_angle =  steering_input;
	msg.throttle_value = throttle_val;
	msg.brake_value = brake_val;
	drc_driving_control_cmd_t_publish(self->lcm, "DRC_DRIVING_COMMAND", &msg);	
    }
    else
        perform_emergency_stop (self);

    last_utime = self->utime;

    delete fmap;

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
    self->accelerator_utime = -1;
    self->stop_utime = -1;

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

    self->last_driving_cmd = NULL;
    self->drive_duration = 0;
    // Get default parameters
    self->goal_distance = 10.0;//bot_param_get_double_or_fail (self->param, "driving.control.lookahead_distance_default");
    self->kp_steer = 5;//bot_param_get_double_or_fail (self->param, "driving.control.kp_steer_default");


    self->lcmgl_goal = bot_lcmgl_init (self->lcm, "DRIVING_GOAL");

    self->lcmgl_rays = bot_lcmgl_init (self->lcm, "DRIVING_RAYS");

    drc_utime_t_subscribe(self->lcm, "ROBOT_UTIME", on_utime, self);

    occ_map_pixel_map_t_subscribe (self->lcm, "TERRAIN_DIST_MAP", on_terrain_dist_map, self);
    drc_driving_control_params_t_subscribe (self->lcm, "DRIVING_CONTROL_PARAMS_DESIRED", on_driving_control_params, self);
    
    drc_driving_cmd_t_subscribe(self->lcm, "DRIVING_CONTROLLER_COMMAND", on_driving_command, self);

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
