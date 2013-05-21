// example usage:
// kmcl-ply-editor
//
//1. select pose
//2. look up all triangles within xm of pose
//3. find all unique colours inside that pose
//4. manual reassgn those colours to another r g b value from an array
//*********************************************************************
#include <sys/types.h>
#include <dirent.h>

#include <getopt.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <glib.h>
#include <glib-object.h>

#include <bot_core/bot_core.h>
#include <lcmtypes/drc_driving_control_cmd_t.h>
#include <lcmtypes/drc_driving_status_t.h>
#include <lcmtypes/drc_affordance_goal_t.h>
//#include <common/globals.h>
//#include <lcmtypes/mrtypes.h>

#include <cstdio> 

#include <vector>
#include <lcmtypes/visualization.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"


#include <ncurses.h>
#include <wchar.h>

#include <iostream>
#include <sstream>
using namespace std;

#define COLOR_PLAIN 1
#define COLOR_TITLE 2
#define COLOR_ERROR 3
#define COLOR_WARN  4
#define STEP_TESTING 1

ros::Publisher hand_wheel_pub;
ros::Publisher hand_brake_pub;
ros::Publisher gas_pedal_pub;
ros::Publisher brake_pedal_pub;
ros::Publisher direction_pub;
ros::Publisher key_pub;

ros::Publisher exit_pub;
ros::Publisher enter_pub;

//http://gazebosim.org/wiki/Tutorials/drcsim/2.2/VRC_Plugin_DRC_Vehicle

typedef struct  {
    WINDOW *w;
    int width, height;

    lcm_t* lcm;

    GMainLoop * mainloop;
    guint timer_id;

    string robot_name;
    double hand_wheel;
    double hand_brake;
    double gas_pedal;
    double brake_pedal;

    // DBW flags
    int dbw_hand_wheel;
    int dbw_hand_brake;
    int dbw_gas_pedal;
    int dbw_brake_pedal;
    int dbw_key;
    int dbw_transmission;

    int verbose;
    int direction;
    int key;

} state_t;



static int publish_gas_pedal(void *user_data){
    state_t* s = static_cast<state_t*>(user_data);

    //if (s->gas_pedal >=0.09) {    s->gas_pedal = 0.09; }
    if (s->gas_pedal <=0.0) {    s->gas_pedal = 0.0; }

    std_msgs::Float64 msg;
    msg.data = s->gas_pedal;
    gas_pedal_pub.publish(msg);
    return 0; 
}

static int publish_hand_brake(void *user_data){
    state_t* s = static_cast<state_t*>(user_data);

    if (s->hand_brake >=1.0) {    s->hand_brake = 1.0; }
    if (s->hand_brake <=0.0) {    s->hand_brake = 0.0; }

    std_msgs::Float64 msg;
    msg.data = s->hand_brake;
    hand_brake_pub.publish(msg);
    if(s->verbose)
        fprintf(stderr, "HB Called : %f\n", s->hand_brake);
    return 0; 
}

static int publish_hand_wheel(void *user_data){
    state_t* s = static_cast<state_t*>(user_data);

    if (s->hand_wheel >=7.0) {    s->hand_wheel = 7.0; }
    if (s->hand_wheel <=-7.0) {    s->hand_wheel = -7.0; }

    std_msgs::Float64 msg;
    msg.data = s->hand_wheel;
    hand_wheel_pub.publish(msg);
    return 0; 
}



static int publish_brake_pedal(void *user_data){
    state_t* s = static_cast<state_t*>(user_data);

    if (s->brake_pedal >=1.0) {    s->brake_pedal = 1.0; }
    if (s->brake_pedal <=0.0) {    s->brake_pedal = 0.0; }

    std_msgs::Float64 msg;
    msg.data = s->brake_pedal;
    brake_pedal_pub.publish(msg);
    return 0; 
}

/*
  The direction state and key state topics send an Int8 value. The direction state reports "1" for Forward, "0" for Neutral, and "-1" for Reverse. The key state reports "1" for On, "0" for Off, and "-1" for an error caused by turning the key to On when the direction switch is not in Neutral. Putting the direction switch back to neutral will restore the key state to "1". The key switch defaults to "On" and the direction to "Forward", but this may not be the case in future versions of the software or in the competition.

*/

static int publish_key(void *user_data){
    state_t* s = static_cast<state_t*>(user_data);
    std_msgs::Int8 msg;
    msg.data = (int8_t) s->key;
    key_pub.publish(msg);
    return 0; 
}

//this is incremental - it needs to be added to the state 
static int publish_direction(void *user_data){
    state_t* s = static_cast<state_t*>(user_data);
    std_msgs::Int8 msg;
    msg.data = s->direction;
    direction_pub.publish(msg);
    return 0; 
}

static int update_and_publish_gas_pedal(double new_val, state_t *s){
    if(new_val != s->gas_pedal){
        s->gas_pedal = new_val;
        return publish_gas_pedal(s);
    }
    return 0;
}

static int update_and_publish_hand_brake(double new_val, state_t *s){
    //if(new_val != s->hand_brake){
    s->hand_brake = new_val;
    return publish_hand_brake(s);
    //}
    //return 0;
}

/*
  static int update_to_and_publish_hand_wheel(double new_val, state_t *s){
  double delta = new_val - s->hand_wheel;
  
  if(delta >0){
  std_msgs::Float64 msg;

  msg.data = delta;
  hand_wheel_pub.publish(msg);

  //how long ?? 
  sleep(1);
  msg.data = 0;
  hand_wheel_pub.publish(msg);
    
  s->hand_wheel += delta;

  fprintf(stderr, "Delta : %f (deg)  Steering Angle : %f (deg) - %f (rad) \n", bot_to_degrees(delta), bot_to_degrees(s->hand_wheel), s->hand_wheel);    

  if (s->hand_wheel >=7.0) {    s->hand_wheel = 7.0; }
  if (s->hand_wheel <=-7.0) {    s->hand_wheel = -7.0; }
  }

  return 0;
  }

  static int update_and_publish_hand_wheel(double delta, state_t *s){
  if(delta >0){
  std_msgs::Float64 msg;
  msg.data = delta;
  hand_wheel_pub.publish(msg);

  //how long ?? 
  sleep(1);
  msg.data = 0;
  hand_wheel_pub.publish(msg);

  s->hand_wheel += delta;

  fprintf(stderr, "Delta : %f (deg)  Steering Angle : %f (deg) - %f (rad) \n", bot_to_degrees(delta), bot_to_degrees(s->hand_wheel), s->hand_wheel);

  if (s->hand_wheel >=7.0) {    s->hand_wheel = 7.0; }
  if (s->hand_wheel <=-7.0) {    s->hand_wheel = -7.0; }
  }

  return 0;
  }
*/

static int update_and_publish_hand_wheel_delta(double delta, state_t *s){
    s->hand_wheel += delta;
    if (s->hand_wheel >=7.0) {    s->hand_wheel = 7.0; }
    if (s->hand_wheel <=-7.0) {    s->hand_wheel = -7.0; }

    if (s->dbw_hand_wheel) {
        std_msgs::Float64 msg;
        
        msg.data = s->hand_wheel;
        hand_wheel_pub.publish(msg);
    }
    else {
        string dof_name_local[] = {"steering_joint"};
        drc_affordance_goal_t msg;
        msg.utime = bot_timestamp_now();
        msg.aff_type = (char *) "car";
        msg.aff_uid = 1;
        msg.num_dofs = 1;
        msg.dof_name = (char **) dof_name_local;
        msg.dof_value[0] = s->hand_wheel;

        drc_affordance_goal_t_publish (s->lcm, "DRIVING_MANIP_CMD", &msg);
    }

    if(s->verbose)
        fprintf(stderr, "Delta : %f (deg)  Steering Angle : %f (deg) - %f (rad) \n", bot_to_degrees(delta), bot_to_degrees(s->hand_wheel), s->hand_wheel);    

    return 0;
}

static int update_and_publish_hand_wheel(double new_val, state_t *s){
    double delta = new_val - s->hand_wheel;
    s->hand_wheel = new_val;
    if (s->hand_wheel >=7.0) {    s->hand_wheel = 7.0; }
    if (s->hand_wheel <=-7.0) {    s->hand_wheel = -7.0; }
    
    if (s->dbw_hand_wheel) {
        std_msgs::Float64 msg;
        msg.data = s->hand_wheel;
        hand_wheel_pub.publish(msg);
    }
    else {
        string dof_name_local[] = {"steering_joint"};
        drc_affordance_goal_t msg;
        msg.utime = bot_timestamp_now();
        msg.aff_type = (char *) "car";
        msg.aff_uid = 1;
        msg.num_dofs = 1;
        msg.dof_name = (char **) dof_name_local;
        msg.dof_value[0] = s->hand_wheel;

        drc_affordance_goal_t_publish (s->lcm, "DRIVING_MANIP_CMD", &msg);
    }
    if(s->verbose)
        fprintf(stderr, "Delta : %f (deg)  Steering Angle : %f (deg) - %f (rad) \n", bot_to_degrees(delta), bot_to_degrees(s->hand_wheel), s->hand_wheel);

    return 0;
}

static int update_and_publish_brake_pedal(double new_val, state_t *s){
    if(new_val != s->brake_pedal){
        s->brake_pedal = new_val;
        return publish_brake_pedal(s);
    }
    return 0;
}

static int update_and_publish_key(int new_val, state_t *s){
    if(new_val != s->key){
        s->key = new_val;
        return publish_key(s);
    }
    return 0;
}

static int update_and_publish_direction(int new_val, state_t *s){
    if(new_val != s->direction){
        s->direction = new_val;
        return publish_direction(s);
    }
    return 0;
}

static int publish_enter(void *user_data){
    state_t* s = static_cast<state_t*>(user_data);
    geometry_msgs::Pose msg;
    msg.position.x =0.0;
    msg.position.y =0.0;
    msg.position.z =0;
    msg.orientation.w =1.0;
    msg.orientation.x =0.0;
    msg.orientation.y =0.0;
    msg.orientation.z =0.0;
    enter_pub.publish(msg);
  
    //rostopic pub --once /drc_world/robot_exit_car geometry_msgs/Pose '{position: {x: 0.5, y: -3.5, z: 0}, //orientation: {w: 0.707, x: 0, y: 0, z: 0.707}}'
    return 0; 
}
static int publish_exit(void *user_data){
    state_t* s = static_cast<state_t*>(user_data);
    geometry_msgs::Pose msg;
    msg.position.x =0.5;
    msg.position.y =-3.5;
    msg.position.z =0;
    msg.orientation.w =0.707;
    msg.orientation.x =0.0;
    msg.orientation.y =0.0;
    msg.orientation.z =0.707;
    exit_pub.publish(msg);
  

    //  std_msgs::Int8 msg;
    //  msg.data = s->direction;
    //  direction_pub.publish(msg);
    return 0; 
}

void init_state(state_t *s){
    fprintf(stderr, "Initializing State\n");
    
    s->hand_wheel = 0;
    s->gas_pedal = 0;
    s->hand_brake = 0.05;
    s->brake_pedal = 0.0;
    s->key = 1.0;
    s->direction = 0.0;

    //publish these values 
    publish_hand_brake(s);
    sleep(1.0);
    s->hand_brake = 0.00;
    publish_hand_brake(s);
    sleep(1.0);
    publish_gas_pedal(s);
    sleep(1.0);
    update_and_publish_hand_wheel(0, s);
    sleep(1.0);
    publish_brake_pedal(s);
    sleep(1.0);
    publish_key(s);
    sleep(1.0);
    publish_direction(s);
    sleep(1.0);
    s->direction = 1.0;
    publish_direction(s);
}


void on_driving_cmd(const lcm_recv_buf_t *rbuf, const char * channel, const drc_driving_control_cmd_t * msg, void * user) {
    state_t *self = (state_t *) user;

    static int first = 1;
    /*if(first)
        init_state(self);
        
        first = 0;*/

    fprintf(stderr, "Command => Utime : %f\n", msg->utime/1.0e6);
    if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_START_CAR){
        fprintf(stderr, "Starting vehicle - this should be ignored - vehicle always on\n");
        //self->key = 1;
        //publish_key(user);
        self->direction = 0;
        publish_direction(self);
        update_and_publish_key(1, self);
        self->direction = 1;
        publish_direction(self);
    }
    else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_SWITCH_OFF_ENGINE){
        fprintf(stderr, "Switching off engine - this should be ignored - vehicle always on\n");
        update_and_publish_key(0, self);
    } 
    else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_GEAR_FORWARD){
        fprintf(stderr, "Putting gear forward\n");
        update_and_publish_direction(1, self);
    } 
    else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_GEAR_NEUTRAL){
        fprintf(stderr, "Putting gear neutral\n");
        update_and_publish_direction(0, self);
    } 
    else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_GEAR_REVERSE){
        fprintf(stderr, "Putting gear reverse\n");
        update_and_publish_direction(-1, self);
    } 
    else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_DRIVE){
        if(self->verbose)
            fprintf(stderr, "Driving - turning wheel heading (deg): %f Throttle : %f Brake : %f\n", bot_to_degrees(msg->steering_angle), msg->throttle_value,  msg->brake_value);
        update_and_publish_hand_brake(0.0, self);
        update_and_publish_brake_pedal(msg->brake_value, self);
        update_and_publish_hand_wheel(msg->steering_angle, self);
        update_and_publish_gas_pedal(msg->throttle_value, self);
    } 
    else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_DRIVE_DELTA_STEERING){
        if(self->verbose)
            fprintf(stderr, "Driving - turning wheel delta heading (deg): %f Throttle : %f Brake : %f \n", bot_to_degrees(msg->steering_angle + self->hand_wheel), msg->throttle_value, msg->brake_value);
        update_and_publish_hand_brake(0.0, self);
        //update_and_publish_brake_pedal(0.0, self);
        update_and_publish_hand_wheel_delta(msg->steering_angle, self);
        update_and_publish_brake_pedal(msg->brake_value, self);
        update_and_publish_gas_pedal(msg->throttle_value, self);
    } 
    else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_BRAKE){
        if(self->verbose)
            fprintf(stderr, "Applying brake %f\n", msg->brake_value); 
        //update_and_publish_hand_wheel(msg->steering_angle, self);
        update_and_publish_brake_pedal(msg->brake_value, self);
        update_and_publish_gas_pedal(0, self);
    } 
    else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_E_STOP){
        fprintf(stderr, "E Stopping Vehicle \n");
        update_and_publish_brake_pedal(1.0, self);
    }
    else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_HANDBRAKE_ON){
        fprintf(stderr, "Applying Handbrake \n");
        //update_and_publish_hand_brake(0.05, self);
        //update_and_publish_hand_brake(.3, self);
        update_and_publish_hand_brake(1.0, self);
        //update_and_publish_hand_brake(1.0, self);
        //update_and_publish_hand_brake(1.0, self);
    }
    else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_HANDBRAKE_OFF){
        fprintf(stderr, "Removing Handbrake\n");
        update_and_publish_hand_brake(0.00, self);
    }

    else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_START_SEQUENCE){
        fprintf(stderr, "Initializing start sequence\n");
        init_state(self);
        fprintf(stderr, "Done with the startup");
    }
}


static gboolean
on_timer (void * user)
{
    state_t* s = static_cast<state_t*>(user);
    //publish state
    drc_driving_status_t msg;
    msg.utime = bot_timestamp_now();
    msg.hand_wheel = s->hand_wheel;
    msg.hand_brake = s->hand_brake;
    msg.gas_pedal = s->gas_pedal;
    msg.brake_pedal = s->brake_pedal;
    msg.direction = s->direction;
    msg.key = s->key;

    drc_driving_status_t_publish(s->lcm, "DRC_DRIVING_STATUS", &msg);
  
    return TRUE;
}

static void usage (int argc, char **argv)
{
    fprintf (stdout, "Usage: %s [options]\n"
             "\n"
             " -D, --DBW                 Control all actuation using drive-by-wire\n"
             " -s, --steer_dbw           Control steering using drive-by-wire\n"
             " -b, --brake_dbw           Control brake pedal using drive-by-wire\n"
             " -g, --gas_dbw             Control gas pedal using drive-by-wire\n"
             " -p, --handbrake_dbw       Control handbrake using drive-by-wire\n"
             " -t, --transmission_dbw    Control transmission using drive-by-wire\n"
             " -k, --key_dbw             Control key using drive-by-wire\n"
             " -v, --verbose             Verbose output\n"
             " -h, --help                Print this help and exit\n"
             "\n",
             argv[0]);
}




int main(int argc, char *argv[])
{
    state_t* state = new state_t();
    state->lcm= lcm_create(NULL);

    char c;
    char *optstring = "vhDsbgpkt";
    struct option long_opts[] = {
        { "DBW", no_argument, 0, 'D'},
        { "steer_dbw", no_argument, 0, 's'},
        { "brake_dbw", no_argument, 0, 'b'},
        { "gas_dbw", no_argument, 0, 'g'},
        { "handbrake_dbw", no_argument, 0, 'p'},
        { "key_dbw", no_argument, 0, 'k'},
        { "transmission_dbw", no_argument, 0, 't'},
        { "verbose", no_argument, 0, 'v'},
        { "help", no_argument, 0, 'h'},
        { 0, 0, 0, 0 }
    };

    state->verbose = 0;
    
    while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0){
        switch (c) {
        case 'D':
            state->dbw_hand_wheel = 1;
            state->dbw_hand_brake = 1;
            state->dbw_gas_pedal = 1;
            state->dbw_brake_pedal = 1;
            state->dbw_key = 1;
            state->dbw_transmission = 1;
            break;
        case 's':
            state->dbw_hand_wheel = 1;
            break;
        case 'b':
            state->dbw_brake_pedal = 1;
            break;
        case 'g':
            state->dbw_gas_pedal = 1;
            break;
        case 'p':
            state->dbw_hand_brake = 1;
            break;
        case 'k':
            state->dbw_key = 1;
            break;
        case 't':
            state->dbw_transmission = 1;
            break;
        case 'v':
            fprintf(stderr, "Verbose\n");
            state->verbose = 1;
            break;
        default:
        case 'h':
            usage (argc, argv);
            return 1;
        }
    }

    if (state->dbw_hand_wheel)
        fprintf (stdout, "USING DRIVE-BY-WIRE TO CONTROL STEERING\n");
    if (state->dbw_hand_brake)
        fprintf (stdout, "USING DRIVE-BY-WIRE TO CONTROL HAND BRAKE\n");
    if (state->dbw_brake_pedal)
        fprintf (stdout, "USING DRIVE-BY-WIRE TO CONTROL BRAKE PEDAL\n");
    if (state->dbw_gas_pedal)
        fprintf (stdout, "USING DRIVE-BY-WIRE TO CONTROL GAS PEDAL\n");
    if (state->dbw_key)
        fprintf (stdout, "USING DRIVE-BY-WIRE TO CONTROL KEY\n");
    if (state->dbw_transmission)
        fprintf (stdout, "USING DRIVE-BY-WIRE TO CONTROL TRANSMISSION\n");

    state->robot_name = "drc_vehicle";
    ros::init(argc, argv, "keyboard_driving");
    ros::NodeHandle n;
    hand_wheel_pub = n.advertise<std_msgs::Float64>(state->robot_name +"/hand_wheel/cmd", 1000);
    hand_brake_pub = n.advertise<std_msgs::Float64>(state->robot_name +"/hand_brake/cmd", 1000);
    gas_pedal_pub = n.advertise<std_msgs::Float64>(state->robot_name +"/gas_pedal/cmd", 1000);
    brake_pedal_pub = n.advertise<std_msgs::Float64>(state->robot_name +"/brake_pedal/cmd", 1000);
    direction_pub = n.advertise<std_msgs::Int8>(state->robot_name +"/direction/cmd", 1000);
    key_pub = n.advertise<std_msgs::Int8>(state->robot_name +"/key/cmd", 1000);

    exit_pub = n.advertise<geometry_msgs::Pose>("/drc_world/robot_exit_car", 1000);
    enter_pub = n.advertise<geometry_msgs::Pose>("/drc_world/robot_enter_car", 1000);
  
    state->mainloop = g_main_loop_new (NULL, FALSE);
    bot_glib_mainloop_attach_lcm (state->lcm);
    bot_signal_pipe_glib_quit_on_kill (state->mainloop);

    //subscribe to control channel 
    drc_driving_control_cmd_t_subscribe(state->lcm, "DRC_DRIVING_COMMAND", on_driving_cmd, state);

    state->timer_id = g_timeout_add (100, on_timer, state);
    init_state(state);
    /* Watch stdin */
    /*GIOChannel * channel = g_io_channel_unix_new (0);
      g_io_add_watch (channel, G_IO_IN, on_input, state);
      state->timer_id = g_timeout_add (25, on_timer, state);

      state->w = initscr();
      start_color();
      cbreak();
      noecho();

      init_pair(COLOR_PLAIN, COLOR_WHITE, COLOR_BLACK);
      init_pair(COLOR_TITLE, COLOR_BLACK, COLOR_WHITE);
      init_pair(COLOR_WARN, COLOR_BLACK, COLOR_YELLOW);
      init_pair(COLOR_ERROR, COLOR_BLACK, COLOR_RED);*/

    g_main_loop_run (state->mainloop);
  
    endwin ();
    g_source_remove (state->timer_id);
    bot_glib_mainloop_detach_lcm (state->lcm);
    g_main_loop_unref (state->mainloop);
    state->mainloop = NULL;
    //globals_release_lcm (s->lc);
    free (state);
}
