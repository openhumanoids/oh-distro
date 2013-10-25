/**
 * \file controlform.h
 *
 * 
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#ifndef CONTROLFORM_H
#define CONTROLFORM_H

#include "ros/ros.h"

#include "ui_controlform.h"
#include <QMainWindow>

class ControlForm : public QMainWindow
{
    Q_OBJECT

public:

    ControlForm(QWidget *parent = 0);

private slots:
    void on_posButton_clicked();
    void on_velButton_clicked();
    void on_hndButton_clicked();
    void on_calButton_clicked();

    void on_topButton_clicked();
    void on_mdlButton_clicked();
    void on_btmButton_clicked();
    void on_stpButton_clicked();
    
public slots:
    void sendControl();
    void sendCalibrate();
    void stop();

public:
    ros::NodeHandle node;
    
    ros::Publisher control_pub;
    ros::Publisher calibrate_pub;
    ros::Publisher control_pub2;
    ros::Publisher calibrate_pub2;
    
    bool right_hand;
    
    Ui::ControlForm ui;
    
    // for convenience
    QSlider* sliders[5];
};

#endif
