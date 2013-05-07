#pragma once

#include <QWidget>
#include <action_authoring/OffsetRelation.h>

namespace action_authoring 
{

class QtOffsetRelation
{
protected:
    OffsetRelationPtr _relation;
public:
    QtOffsetRelation(OffsetRelationPtr);
    QWidget * getWidget();
    void updateElementsFromState();

private slots:

    void updateStateFromElements();

}; // end class 

} // end namespace
