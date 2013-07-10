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

#include <annan_control/annanLib.hpp>
#include "ui_controlform.h"
#include <QTimer>

class ControlForm : public QMainWindow //Widget
{
    Q_OBJECT

public:

    /// Constructor
    //
    // \param useAnnan Set to true if Annan's device is plugged in
    //
    ControlForm(bool useAnnan, annan_cb_t acb); //QWidget *parent = 0);

    AnnanBoard toAnnanData() const;
                                  
public slots:
    void setData(AnnanBoard data);
    //void setData(int s0, int s1, int s2, int s3, int s4, bool b0, bool b1, bool b2, bool b3);

private slots:
    // void on_B0_released();
    // void on_B1_released();
    // void on_B2_released();
    // void on_B3_released();

    void on_UseSerial_released();
    
    void timeout();
    
    // void on_S0_sliderMoved(int value);
    
    void on_Aux0_released();
    void on_Aux2_released();
    void on_Aux4_released();
    void on_Aux1_released();
    
    // void on_inputSpinBox2_valueChanged(int value);

signals:
    void sendToHand(AnnanBoard data);
    
private:
    Ui::ControlForm ui;
    QTimer timer;
    annan_cb_t annan_cb;
};

#endif
