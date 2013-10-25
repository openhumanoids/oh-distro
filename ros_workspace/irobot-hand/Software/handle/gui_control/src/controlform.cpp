/**
 * \file controlform.cpp
 *
 * 
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/
#include <QtGui>

#include "../include/gui_control/controlform.hpp"

#include "std_msgs/Empty.h"

#include "handle_msgs/HandleSensors.h"
#include "handle_msgs/Finger.h"
#include "handle_msgs/HandleControl.h"

#define MAX_SLIDER 100
#define MAX_MOTOR_VELOCITY 12000
#define MAX_MOTOR_POSITION 10000
#define MAX_SPREAD_POSITION 850
//#define MAX_MOTOR_CURRENT 1000 
//#define MAX_MOTOR_VOLTAGE 43000  // RPM = 0.2863 * Voltage - 385.5
#define MAX_SPREAD_MOTOR_VELOCITY 4095

ControlForm::ControlForm(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    
    right_hand = true;
    
    // for convenience
    sliders[0] = ui.F1Slider;
    sliders[1] = ui.F2Slider;
    sliders[2] = ui.F3Slider;
    sliders[3] = ui.F4Slider;
    sliders[4] = ui.F5Slider;
    
    control_pub = node.advertise<handle_msgs::HandleControl>("/handle/control", 1);
    calibrate_pub = node.advertise<std_msgs::Empty>("/handle/calibrate", 1);
    control_pub2 = node.advertise<handle_msgs::HandleControl>("/handle/control2", 1);
    calibrate_pub2 = node.advertise<std_msgs::Empty>("/handle/calibrate2", 1);
    
    for (int i=0; i<5; i++)
        connect(sliders[i], SIGNAL(valueChanged(int)),
                this, SLOT(sendControl()));
}

void ControlForm::stop()
{
    ui.posButton->setChecked(false); // stop position mode
    ui.velButton->setChecked(true);  // start velocity mode
    for (int i=0; i<5; i++)
    {
        sliders[i]->blockSignals(true);
        sliders[i]->setValue(50);
        sliders[i]->blockSignals(false);
    }
}

void ControlForm::on_posButton_clicked() // position mode
{
    // NOTE: the checked state got toggled before entering this function so 
    // !checked() actually means the button was checked when the user pressed it.
    if (!ui.posButton->isChecked())
        return;
    
    ui.posButton->setChecked(true); // start position mode
    ui.velButton->setChecked(false); // stop velocity mode
    for (int i=0; i<5; i++)
    {
        sliders[i]->blockSignals(true);
        sliders[i]->setValue(0);
        sliders[i]->blockSignals(false);
    }
    
    sendControl();
}

void ControlForm::on_velButton_clicked() // velocity mode
{
    // NOTE: the checked state got toggled before entering this function so 
    // !checked() actually means the button was checked when the user pressed it.
    if (!ui.velButton->isChecked())
        return;
    
    stop();
    sendControl();
}

void ControlForm::on_hndButton_clicked() // switch hand publication
{
    // stop current hand
    stop();
    sendControl();

    // switch hands
    right_hand = !right_hand;
    if (right_hand)
        ui.hndButton->setText("Switch to Left");
    else
        ui.hndButton->setText("Switch to Right");
    
    // stop new hand
    stop();
    sendControl();
}

void ControlForm::on_calButton_clicked() // calibrate msg
{
    sendCalibrate();
}

void ControlForm::on_btmButton_clicked() // bottom
{
    for (int i=0; i<5; i++)
    {
        sliders[i]->blockSignals(true);
        sliders[i]->setValue(0);
        sliders[i]->blockSignals(false);
    }
    sendControl();
}

void ControlForm::on_mdlButton_clicked() // middle
{
    for (int i=0; i<5; i++)
    {
        sliders[i]->blockSignals(true);
        sliders[i]->setValue(50);
        sliders[i]->blockSignals(false);
    }
    
    sendControl();
}

void ControlForm::on_topButton_clicked() //top
{
    for (int i=0; i<5; i++)
    {
        sliders[i]->blockSignals(true);
        sliders[i]->setValue(100);
        sliders[i]->blockSignals(false);
    }
    
    sendControl();
}

void ControlForm::on_stpButton_clicked() // stop
{
    stop();
    sendControl();
}

void ControlForm::sendControl()
{
    handle_msgs::HandleControl msg;
    
    if (ui.posButton->isChecked())
    {
        for (int i=0; i<5; i++)
        {
            msg.type[i] = handle_msgs::HandleControl::POSITION;
            msg.valid[i] = true;
            
            // handle spread motor slightly differently
            if (i==4)
                msg.value[i] = sliders[i]->value() * (float)MAX_SPREAD_POSITION / (float)MAX_SLIDER;
            else
                msg.value[i] = sliders[i]->value() * (float)MAX_MOTOR_POSITION / (float)MAX_SLIDER;
        }
    }
    else if (ui.velButton->isChecked())
    {
        for (int i=0; i<5; i++)
        {
            msg.type[i] = handle_msgs::HandleControl::VELOCITY;
            msg.valid[i] = true;
            
            // handle spread motor slightly differently
            if (i==4)
                msg.value[i] = (sliders[i]->value()-MAX_SLIDER/2) * (float)MAX_SPREAD_MOTOR_VELOCITY / (MAX_SLIDER/2);
            else
                msg.value[i] = (sliders[i]->value()-MAX_SLIDER/2) * (float)MAX_MOTOR_VELOCITY / (MAX_SLIDER/2);
        }
    }
    else
    {
        // return before publishing
        return;
    }
    
    if (right_hand)
        control_pub.publish(msg);
    else
        control_pub2.publish(msg);
}

void ControlForm::sendCalibrate()
{
    if (right_hand)
        calibrate_pub.publish(std_msgs::Empty());
    else
        calibrate_pub2.publish(std_msgs::Empty());
}
