/**
 * \file gui.cpp
 *
 * Main GUI app for handle control.  
 * Can interface with Annan's controller board, or replace it.
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#include <QApplication>

#include <annan_control/annanLib.hpp>
#include "../include/gui_control/annanWrapper.hpp"
#include "../include/gui_control/controlform.hpp"
//#include "../../../HandleLib/handleTcp.hpp"

#include "ros/ros.h"

#include "std_msgs/Empty.h"

#include "handle_msgs/HandleSensors.h"
#include "handle_msgs/Finger.h"
#include "handle_msgs/CableTension.h"
#include "handle_msgs/HandleControl.h"

#define ANNAN_DEVICE "/dev/ttyACM0"
//#define HAND_NAME "192.168.0.22" //"armH-palm-1"

// 7200 ticks = 2pi radians
#define ENCODER_TICKS_TO_SPINDLE_RADIANS 8.72664625997e-4 // 2pi / 7200

// globals
bool annanConnected = false;
ros::Publisher control_pub;
ros::Publisher calibrate_pub;
AnnanWrapper a_wrapper;

void a_callback(const AnnanBoard& data)
{
    // got data from annan board
    
    // send data to hand
    HandleCommand cmd = annan_to_cmd(data);
    if (cmd.calibrate)
        calibrate_pub.publish(std_msgs::Empty());
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
            msg.value[i] = cmd.motorCommand[i].value * ENCODER_TICKS_TO_SPINDLE_RADIANS;
            msg.valid[i] = cmd.motorCommand[i].valid;
        }
        control_pub.publish(msg);
    }
    
    // update GUI
    if (annanConnected)
        a_wrapper.relay(data);
};


int main(int argc, char *argv[])
{
    qRegisterMetaType<AnnanBoard>("AnnanBoard");

    ros::init(argc, argv, "gui_control");
    ros::NodeHandle n;
    control_pub = n.advertise<handle_msgs::HandleControl>("/handle/control", 1);
    calibrate_pub = n.advertise<std_msgs::Empty>("/handle/calibrate", 1);

    QApplication app(argc, argv);

    if (annan_connect(ANNAN_DEVICE) < 0)
    {
        printf("WARNING: cannot open annan device\n");
        annanConnected = false;
    }
    else
    {
        annanConnected = true;
    }

    // gui
    ControlForm control(annanConnected, a_callback);
    
    if (annanConnected)
    {
        if (annan_start(a_callback) < 0)
        {
            printf("WARNING: cannot start serial thread\n");
            annanConnected = false;
        }
    }
    
    if (annanConnected)
    {
        // signals / slots for inter-thread communication
        QObject::connect(&a_wrapper, SIGNAL(updategui(AnnanBoard)),
                         &control, SLOT(setData(AnnanBoard)));//,
                         //Qt::QueuedConnection);
    }
    
    control.show();
    
    // block on GUI
    printf("Press CTRL-C to exit\n");
    app.exec();
    
    if (annanConnected)
    {
        if (annan_stop() < 0)
        {
            printf("WARNING: cannot stop annan thread\n");
        }
    }
    
    return 0;
}
