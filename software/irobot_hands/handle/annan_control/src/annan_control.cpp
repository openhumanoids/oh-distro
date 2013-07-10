/**
 * \file annan_control.cpp
 *
 * control hand over tcp interface with annan's device
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

//#include "../../annanLib.hpp"
#include "../include/annan_control/annanLib.hpp"

#include "ros/ros.h"

#include "std_msgs/Empty.h"

#include "handle_msgs/HandleSensors.h"
#include "handle_msgs/Finger.h"
#include "handle_msgs/CableTension.h"
#include "handle_msgs/HandleControl.h"

#define ANNAN_DEVICE "/dev/ttyACM0"
//#define HAND_NAME "192.168.0.22" //"armH-palm-1"

// globals
ros::Publisher control_pub;
ros::Publisher calibrate_pub;
ros::Publisher control_pub2;
ros::Publisher calibrate_pub2;

// 7200 ticks = 2pi radians
//#define ENCODER_TICKS_TO_SPINDLE_RADIANS 8.72664625997e-4 // 2pi / 7200

void a_callback(const AnnanBoard& data)
{
    HandleCommand cmd = annan_to_cmd(data);
    if (cmd.calibrate)
    {
        if (!data.Button[1])
            calibrate_pub.publish(std_msgs::Empty());
        else
            calibrate_pub2.publish(std_msgs::Empty());
    }
    else if (cmd.anyValid())
    {
        handle_msgs::HandleControl msg;
        for (int i=0; i<5; i++)
        {
            switch (cmd.motorCommand[i].type)
            {
                case MOTOR_VELOCITY:
                    msg.type[i] = handle_msgs::HandleControl::VELOCITY;
                    break;
                case MOTOR_POSITION:
                    msg.type[i] = handle_msgs::HandleControl::POSITION;
                    break;
                case MOTOR_CURRENT:
                    msg.type[i] = handle_msgs::HandleControl::CURRENT;
                    break;
                case MOTOR_VOLTAGE:
                    msg.type[i] = handle_msgs::HandleControl::VOLTAGE;
                    break;
                default:
                    printf("UNKNOWN CONTROL TYPE\n");
                    return;
            }
            msg.value[i] = cmd.motorCommand[i].value; // * ENCODER_TICKS_TO_SPINDLE_RADIANS;
            msg.valid[i] = cmd.motorCommand[i].valid;
        }
        
        if (!data.Button[1])
            control_pub.publish(msg);
        else
            control_pub2.publish(msg);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "annan_control");
    ros::NodeHandle n;
    control_pub = n.advertise<handle_msgs::HandleControl>("/handle/control", 1);
    calibrate_pub = n.advertise<std_msgs::Empty>("/handle/calibrate", 1);
    control_pub2 = n.advertise<handle_msgs::HandleControl>("/handle/control2", 1);
    calibrate_pub2 = n.advertise<std_msgs::Empty>("/handle/calibrate2", 1);
    
    if (annan_connect(ANNAN_DEVICE) < 0)
    {
        printf("ERROR: cannot open annan device\n");
        return -1;
    }
    
    if (annan_start(a_callback) < 0)
    {
        printf("ERROR: cannot start thread\n");
        return -1;
    }
    
    printf("Press CTRL-C to exit\n");
    ros::spin();
    
    if (annan_stop() < 0)
    {
        printf("WARNING: cannot stop annan thread\n");
    };
    
    return 0;
};
