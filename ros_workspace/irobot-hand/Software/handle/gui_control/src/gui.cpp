/**
 * \file gui.cpp
 *
 * Main GUI app for handle control.  
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#include <signal.h>

#include <QApplication>

#include "../include/gui_control/controlform.hpp"

#include "ros/ros.h"

// Custom signal hander.  
// The default ros signal handler only calls ros::shutdown().
// We need to signal Qt that the user pressed CTRL-C.
QApplication* ptr = NULL;
void mySigintHandler(int sig)
{
    if (ptr)
        ptr->closeAllWindows();
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gui_control");
    
    QApplication app(argc, argv);
    ptr = &app;
    
    ControlForm control;

    // Must go AFTER first NodeHandle declaration.
    signal(SIGINT, mySigintHandler);
    
    control.show();
    
    printf("Press CTRL-C to exit\n");
    app.exec();
    
    // stop both hands
    control.stop();
    control.sendControl();
    control.right_hand = !control.right_hand;
    control.sendControl();
    
    return 0;
}
