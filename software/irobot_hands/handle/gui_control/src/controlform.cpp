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

ControlForm::ControlForm(bool useAnnan, annan_cb_t acb)
    : QMainWindow()
{
    ui.setupUi(this);
    
    annan_cb = acb;
    
    if (!useAnnan)
    {
        ui.UseSerial->setChecked(false);
        ui.UseSerial->setDisabled(true);
    }
    
    on_UseSerial_released();

    connect(&timer, SIGNAL(timeout()), this, SLOT(timeout()));
    timer.start(40); //25 Hz
}

AnnanBoard ControlForm::toAnnanData() const
{
    AnnanBoard adata;
    adata.Slider[0] = ui.Slider0->value();
    adata.Slider[1] = ui.Slider1->value();
    adata.Slider[2] = ui.Slider2->value();
    adata.Slider[3] = ui.Slider3->value();
    adata.Slider[4] = ui.Slider4->value();
    adata.Button[0] = ui.Button0->isChecked();
    adata.Button[1] = ui.Button1->isChecked();
    adata.Button[2] = ui.Button2->isChecked();
    adata.Button[3] = ui.Button3->isChecked();
    return adata;
}

void ControlForm::timeout()
{
    if (!ui.UseSerial->isChecked())
    {
        annan_cb(toAnnanData());
    }
}

void ControlForm::on_Aux0_released() // bottom
{
    ui.Slider0->setValue(0);
    ui.Slider1->setValue(0);
    ui.Slider2->setValue(0);
    ui.Slider3->setValue(0);
    ui.Slider4->setValue(0);
}
void ControlForm::on_Aux2_released() // middle
{
    ui.Slider0->setValue(512);
    ui.Slider1->setValue(512);
    ui.Slider2->setValue(512);
    ui.Slider3->setValue(512);
    ui.Slider4->setValue(512);
}
void ControlForm::on_Aux4_released() //top
{
    ui.Slider0->setValue(1024);
    ui.Slider1->setValue(1024);
    ui.Slider2->setValue(1024);
    ui.Slider3->setValue(1024);
    ui.Slider4->setValue(1024);
}

void ControlForm::on_Aux1_released() // stop
{
    ui.Button3->setChecked(false); // stop position mode
    ui.Button0->setChecked(false); // stop calibrating
    ui.Button2->setChecked(true); // velocity mode
    ui.Slider0->setValue(512);
    ui.Slider1->setValue(512);
    ui.Slider2->setValue(512);
    ui.Slider3->setValue(512);
    ui.Slider4->setValue(512);
}

void ControlForm::on_UseSerial_released()
{
    ui.Button0->setEnabled(!ui.UseSerial->isChecked());
    ui.Button1->setEnabled(!ui.UseSerial->isChecked());
    ui.Button2->setEnabled(!ui.UseSerial->isChecked());
    ui.Button3->setEnabled(!ui.UseSerial->isChecked());
    
    ui.Slider0->setEnabled(!ui.UseSerial->isChecked());
    ui.Slider1->setEnabled(!ui.UseSerial->isChecked());
    ui.Slider2->setEnabled(!ui.UseSerial->isChecked());
    ui.Slider3->setEnabled(!ui.UseSerial->isChecked());
    ui.Slider4->setEnabled(!ui.UseSerial->isChecked());
}

void ControlForm::setData(AnnanBoard data)
{
    //if (ui.UseSerial->isChecked())
    {
        ui.Slider0->setValue(data.Slider[0]);
        ui.Slider1->setValue(data.Slider[1]);
        ui.Slider2->setValue(data.Slider[2]);
        ui.Slider3->setValue(data.Slider[3]);
        ui.Slider4->setValue(data.Slider[4]);
    
        ui.Button0->setChecked(data.Button[0]);
        ui.Button1->setChecked(data.Button[1]);
        ui.Button2->setChecked(data.Button[2]);
        ui.Button3->setChecked(data.Button[3]);
    }
}


