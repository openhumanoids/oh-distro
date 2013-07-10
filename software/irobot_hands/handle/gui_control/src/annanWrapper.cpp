/**
 * \file annanWrapper.cpp
 *
 * a QObject wrapper for annanLib
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#include "../include/gui_control/annanWrapper.hpp"
//#include <stdio.h>//printf

AnnanWrapper::AnnanWrapper(QObject* parent) :
    QObject(parent)
{

}

void AnnanWrapper::relay(AnnanBoard data)
{
    emit updategui(data);
}

