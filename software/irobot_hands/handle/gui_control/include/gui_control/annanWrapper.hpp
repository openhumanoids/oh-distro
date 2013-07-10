/**
 * \file annanWrapper.hpp
 *
 * a QObject wrapper for annanLib
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#ifndef ANNAN_WRAPPER_H
#define ANNAN_WRAPPER_H

#include <QObject>
#include <annan_control/annanLib.hpp>

class AnnanWrapper : public QObject
{
Q_OBJECT

public:
    AnnanWrapper(QObject* parent = 0);
    void relay(AnnanBoard data);
    
public slots:
    
signals:
    void updategui(AnnanBoard data);
};

#endif
