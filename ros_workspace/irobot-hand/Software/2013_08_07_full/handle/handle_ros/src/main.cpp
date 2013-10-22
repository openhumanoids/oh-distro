/**
 * \file main.cpp
 *
 * ROS wrapper for HANDLE
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#include <handle_lib/handleTcp.hpp>
#include <handle_lib/handleControl.hpp>

#include "ros/ros.h"

#include "std_msgs/Empty.h"

#include "handle_msgs/HandleSensors.h"
#include "handle_msgs/Finger.h"
#include "handle_msgs/CableTension.h"
#include "handle_msgs/HandleControl.h"

#define HAND_NAME "armH-palm-1"
#define HAND_PORT "3490"

// globals
ros::Publisher pub;
Dexter hand;

void h_callback(const HandPacket& msg)
{
    handle_msgs::HandleSensors newmsg;
    newmsg.header.stamp = ros::Time(msg.stamp.tv_sec, msg.stamp.tv_usec * 1000);
    
    // for (int i=0; i<2; i++)
    // {
    //     newmsg.cableTension[i].sensor1 = msg.data.cableTension[i].sensor1;
    //     newmsg.cableTension[i].sensor2 = msg.data.cableTension[i].sensor2;
    // }
        
    for (int i=0; i<3; i++)
    {
        newmsg.fingerTactile[i].distal.resize(10);
        newmsg.fingerTactile[i].proximal.resize(12);
        for (int j=0; j<10; j++)
            newmsg.fingerTactile[i].distal[j] = msg.data.fingerTactile[i].distal[j];
        for (int j=0; j<12; j++)
            newmsg.fingerTactile[i].proximal[j] = msg.data.fingerTactile[i].proximal[j];

        newmsg.fingerTactileTemp[i].distal.resize(10);
        newmsg.fingerTactileTemp[i].proximal.resize(12);
        for (int j=0; j<10; j++)
            newmsg.fingerTactileTemp[i].distal[j] = msg.data.fingerTactileTemp[i].distal[j];
        for (int j=0; j<12; j++)
            newmsg.fingerTactileTemp[i].proximal[j] = msg.data.fingerTactileTemp[i].proximal[j];
        
        newmsg.proximalJointAngle[i] = msg.data.proximalJointAngle[i];
        
        newmsg.distalJointAngle[i].distal.resize(4);
        newmsg.distalJointAngle[i].proximal.resize(4);
        for (int j=0; j<4; j++)
        {
            newmsg.distalJointAngle[i].distal[j] = msg.data.distalJointAngle[i].distal[j];
            newmsg.distalJointAngle[i].proximal[j] = msg.data.distalJointAngle[i].proximal[j];
        }
        
        // newmsg.fingerPVDF[i].distal.resize(3);
        // newmsg.fingerPVDF[i].proximal.resize(3);
        // for (int j=0; j<3; j++)
        // {
        //     newmsg.fingerPVDF[i].distal[j] = msg.data.fingerPVDF[i].distal[j];
        //     newmsg.fingerPVDF[i].proximal[j] = msg.data.fingerPVDF[i].proximal[j];
        // }

        newmsg.proximalAcceleration[i].x = msg.data.proximalAcceleration[i].x;
        newmsg.proximalAcceleration[i].y = msg.data.proximalAcceleration[i].y;
        newmsg.proximalAcceleration[i].z = msg.data.proximalAcceleration[i].z;
            
        newmsg.distalAcceleration[i].x = msg.data.distalAcceleration[i].x;
        newmsg.distalAcceleration[i].y = msg.data.distalAcceleration[i].y;
        newmsg.distalAcceleration[i].z = msg.data.distalAcceleration[i].z;
    }
        
    for (int i=0; i<4; i++)
    {
        newmsg.motorHallEncoder[i] = msg.data.motorHallEncoder[i];
        newmsg.motorWindingTemp[i] = msg.data.motorWindingTemp[i];
        newmsg.motorVelocity[i] = msg.data.motorVelocity[i];
        // newmsg.palmPVDF[i] = msg.data.palmPVDF[i];
    }
        
    for (int i=0; i<5; i++)
    {
        newmsg.motorHousingTemp[i] = msg.data.motorHousingTemp[i];
        newmsg.motorCurrent[i] = msg.data.motorCurrent[i];
    }
    
    for (int i=0; i<48; i++)
        newmsg.palmTactile[i] = msg.data.palmTactile[i];
    for (int i=0; i<48; i++)
        newmsg.palmTactileTemp[i] = msg.data.palmTactileTemp[i];
    
    newmsg.airTemp = msg.data.airTemp;
    // newmsg.volts33 = msg.data.voltage.volts33;
    // newmsg.volts12 = msg.data.voltage.volts12;
    // newmsg.volts48 = msg.data.voltage.volts48;
    newmsg.fingerSpread = msg.data.fingerSpread;

    pub.publish(newmsg);
};

void calibrate_callback(const std_msgs::EmptyConstPtr& msg)
{
    HandleCommand cmd;
    cmd.calibrate = true;
    hand.send(cmd);
};

void control_callback(const handle_msgs::HandleControlConstPtr& msg)
{
    HandleCommand cmd;
    for (int i=0; i<5; i++)
    {
        if (msg->valid[i])
        {
            switch (msg->type[i])
            {
                case handle_msgs::HandleControl::VELOCITY: 
                    cmd.motorCommand[i].type = MOTOR_VELOCITY; 
                    break;
                case handle_msgs::HandleControl::POSITION: 
                    cmd.motorCommand[i].type = MOTOR_POSITION; 
                    break;
                case handle_msgs::HandleControl::CURRENT: 
                    cmd.motorCommand[i].type = MOTOR_CURRENT; 
                    break;
                case handle_msgs::HandleControl::VOLTAGE: 
                    cmd.motorCommand[i].type = MOTOR_VOLTAGE; 
                    break;
                default:
                    printf("WARNING: UNKNOWN CONTROL TYPE: %d\n", msg->type[i]);
                    //return;
            }
            cmd.motorCommand[i].value = msg->value[i];
            cmd.motorCommand[i].valid = true;
        }
        else
            cmd.motorCommand[i].valid = false;
    }
    
    if (cmd.anyValid())
        hand.send(cmd);
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "handle_ros");
    ros::NodeHandle n;
    pub = n.advertise<handle_msgs::HandleSensors>("/handle/sensors/raw", 1);
    ros::Subscriber sub1 = n.subscribe("/handle/control", 1, control_callback);
    ros::Subscriber sub2 = n.subscribe("/handle/calibrate", 1, calibrate_callback);
    
    std::string hand_name = HAND_NAME;
    std::string hand_port = HAND_PORT;
    bool udp = false;
    
    ros::NodeHandle("~").getParam("hand_name", hand_name);
    ros::NodeHandle("~").getParam("hand_port", hand_port);
    ros::NodeHandle("~").getParam("hand_udp", udp);
    
    ROS_INFO("%s connecting to %s on %s port %s", 
             ros::this_node::getName().c_str(), 
             hand_name.c_str(), 
             udp ? "udp": "tcp",
             hand_port.c_str());
    
    if (hand.connect(hand_name.c_str(), hand_port.c_str(), udp) < 0)
    {
        printf("ERROR: cannot connect to hand '%s' on %s port %s\n", 
               hand_name.c_str(), 
               udp ? "UDP" : "TCP",
               hand_port.c_str());
        return -1;
    }
    
    if (hand.start(h_callback) < 0)
    {
        printf("ERROR: cannot start handler thread\n");
        return -1;  
    }
    
    printf("Press CTRL-C to exit\n");
    ros::spin();
    
    if (hand.stop() < 0)
    {
        printf("WARNING: cannot stop handler thread\n");
    };
    
    return 0;
};
