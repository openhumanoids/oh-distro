/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include <string>
#include <pthread.h>

#include <bot_core/bot_core.h>
#include <lcmtypes/drc_driving_status_t.h>

using namespace std;

typedef struct  {
    lcm_t* lcm;
    
    GMainLoop * mainloop;
    pthread_t  publish_thread;
    pthread_mutex_t mutex;

    string robot_name;
    
    double hand_wheel;
    double hand_brake;
    double gas_pedal;
    double brake_pedal;
    
    int direction;
    int key;    
} state_t;

state_t* state;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

void hand_wheel_callback(const std_msgs::Float64::ConstPtr& msg)
{
    pthread_mutex_lock(&state->mutex);
    state->hand_wheel = msg->data;
    pthread_mutex_unlock(&state->mutex);
    //ROS_INFO("Hand Wheel: [%f]", msg->data);
}

void hand_brake_callback(const std_msgs::Float64::ConstPtr& msg)
{
    pthread_mutex_lock(&state->mutex);
    state->hand_brake = msg->data;
    pthread_mutex_unlock(&state->mutex);
    //ROS_INFO("Hand Brake: [%f]", msg->data);
}

void gas_pedal_callback(const std_msgs::Float64::ConstPtr& msg)
{
    pthread_mutex_lock(&state->mutex);
    state->gas_pedal = msg->data;
    pthread_mutex_unlock(&state->mutex);
    //ROS_INFO("Gas Pedal: [%f]", msg->data);
}

void brake_pedal_callback(const std_msgs::Float64::ConstPtr& msg)
{
    pthread_mutex_lock(&state->mutex);
    state->brake_pedal = msg->data;
    pthread_mutex_unlock(&state->mutex);
    //ROS_INFO("Brake Pedal: [%f]", msg->data);
}

void direction_callback(const std_msgs::Int8::ConstPtr& msg)
{
    pthread_mutex_lock(&state->mutex);
    state->direction = msg->data;
    pthread_mutex_unlock(&state->mutex);
    //ROS_INFO("Direction: [%f]", msg->data);
}

void key_callback(const std_msgs::Int8::ConstPtr& msg)
{
    pthread_mutex_lock(&state->mutex);
    state->key = msg->data;
    pthread_mutex_unlock(&state->mutex);
    //ROS_INFO("Key: [%f]", msg->data);
}


//pthread
static void *track_work_thread(void *user)
{
    state_t *s = (state_t*) user;

    printf("obstacles: track_work_thread()\n");

    while(1){
        pthread_mutex_lock(&state->mutex);

        fprintf(stderr, "Hand Brake : %f\n", state->hand_brake);
        fprintf(stderr, "Hand Wheel : %f\n", state->hand_wheel);
        fprintf(stderr, "Brake Pedal : %f\n", state->brake_pedal);
        fprintf(stderr, "Gas Pedal : %f\n", state->gas_pedal);
        fprintf(stderr, "Direction : %d\n", state->direction);
        fprintf(stderr, "Key : %d\n\n", state->key);

        drc_driving_status_t msg;
        msg.utime = bot_timestamp_now();
        msg.hand_wheel = state->hand_wheel;
        msg.hand_brake = state->hand_brake;
        msg.gas_pedal = state->gas_pedal;
        msg.brake_pedal = state->brake_pedal;
        msg.direction = state->direction;
        msg.key = state->key;

        drc_driving_status_t_publish(state->lcm, "DRC_DRIVING_GROUND_TRUTH_STATUS", &msg);
        
        pthread_mutex_unlock(&state->mutex);
        usleep(5000);
    }
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
    state = new state_t();
    state->lcm= lcm_create(NULL);
    
    state->robot_name = "drc_vehicle";
    ros::init(argc, argv, "listener");
    
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    ros::Subscriber sub_hw = n.subscribe(state->robot_name +"/hand_wheel/state", 1000, hand_wheel_callback);
    ros::Subscriber sub_hb = n.subscribe(state->robot_name +"/hand_brake/state", 1000, hand_brake_callback);
    ros::Subscriber sub_gp = n.subscribe(state->robot_name +"/gas_pedal/state", 1000, gas_pedal_callback);
    ros::Subscriber sub_bp = n.subscribe(state->robot_name +"/brake_pedal/state", 1000, brake_pedal_callback);
    ros::Subscriber sub_dir = n.subscribe(state->robot_name +"/direction/state", 1000, direction_callback);
    ros::Subscriber sub_key = n.subscribe(state->robot_name +"/key/state", 1000, key_callback);

    //state->timer_id = g_timeout_add (25, on_timer, state);
    pthread_mutex_init(&state->mutex, NULL);
    pthread_create(&state->publish_thread, NULL, track_work_thread, state);
// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
    //add a pthread?? - to  publish the state?? - or do we just publish as they come?
    
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
