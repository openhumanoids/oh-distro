#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "ros/ros.h"
#include "handle_msgs/HandleControl.h"
#include "std_msgs/Empty.h"

#include <signal.h>

#include <ncurses.h>


#define SPEED_INC 500
#define MAX_SPEED 10000
#define MIN_SPEED 500

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle n;
    ros::Publisher control_pub = n.advertise<handle_msgs::HandleControl>("/irobot_hands/r_hand/control", 1);
    ros::Publisher calibrate_pub = n.advertise<std_msgs::Empty>("/irobot_hands/r_hand/calibrate", 1);
    ros::Publisher control_pub2 = n.advertise<handle_msgs::HandleControl>("/irobot_hands/l_hand/control", 1);
    ros::Publisher calibrate_pub2 = n.advertise<std_msgs::Empty>("/irobot_hands/l_hand/calibrate", 1);

    bool righthand = true;
    int speed = 2000;
    
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);

    printw("Commands:\n");
    printw("Toggle hand:    'h'\n");
    printw("Increase speed: 'y'\n");
    printw("Decrease speed: 'n'\n");
    printw("Motor:   [F1] [F2] [F3] [F3A] [S]\n");
    printw("Forward:  'Q'  'W'  'E'  'R'  'T'\n");
    printw("Backward: 'Z'  'X'  'C'  'V   'B'\n");
    printw("------------------------------------\n");
    printw("Hand: %s\n", righthand ? "Right":"Left");
    printw("Speed: %d\n", speed);
    refresh();
    
    while (ros::ok())
    {
        handle_msgs::HandleControl msg;
        for (int i=0; i<5; i++)
        {
            msg.type[i] = handle_msgs::HandleControl::VELOCITY;
            msg.value[i] = 0;
            msg.valid[i] = true;
        }
        
        char ch = getch();
        if (ch != ERR) 
        {
            switch (ch)
            {
                case 'q':
                    msg.value[0] = speed;
                    break;
                case 'w':
                    msg.value[1] = speed;
                    break;
                case 'e':
                    msg.value[2] = speed;
                    break;
                case 'r':
                    msg.value[3] = speed;
                    break;
                case 't':
                    msg.value[4] = speed;
                    break;
                case 'z':
                    msg.value[0] = -speed;
                    break;
                case 'x':
                    msg.value[1] = -speed;
                    break;
                case 'c':
                    msg.value[2] = -speed;
                    break;
                case 'v':
                    msg.value[3] = -speed;
                    break;
                case 'b':
                    msg.value[4] = -speed;
                    break;

                case 'h':
                    righthand = !righthand;
                    move(8, 0);
                    printw("Hand: %s\n", righthand ? "Right":"Left");
                    refresh();
                    break;
                    
                case 'y':
                    speed += SPEED_INC;
                    if (speed > MAX_SPEED)
                        speed = MAX_SPEED;
                    move(9, 0);
                    printw("Speed: %d\n", speed);
                    refresh();
                    break;
                    
                case 'n':
                    speed -= SPEED_INC;
                    if (speed < MIN_SPEED)
                        speed = MIN_SPEED;
                    move(9, 0);
                    printw("Speed: %d\n", speed);
                    refresh();
                    break;
                    
                default:
                    break;
            }
        }
        
        if (righthand)
            control_pub.publish(msg);
        else
            control_pub2.publish(msg);
        
        usleep(50000);
    }
    
    endwin();
    printf("done\n");
}

