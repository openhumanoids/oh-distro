#pragma once

// QT includes
#include <QtGui>
#include <QWidget>
#include <QApplication>
// Custom QT widget
#include "togglepanel.h"

#include "../AtomicConstraint.h"

namespace action_authoring
{

    // Qt4 gui objects corresponding to the members of the superclass
    class Qt4Constraint : public QWidget {
    
    Q_OBJECT

    public:
	Qt4Constraint(AtomicConstraintPtr atomicConstraint);
        TogglePanel* getPanel(); 
	// read the state from the gui elements already referenced
	void readState();

	void setAffordances(std::vector<affordance::AffPtr> &allAffordances);
	void setJointNames(std::vector<std::string> &allJointNames);
	void getConstraint(AtomicConstraintPtr &constraint);
	
    protected:
	TogglePanel* _gui_panel;
        QLineEdit* _gui_name;
        QComboBox* _gui_robotJointType;
        QComboBox* _gui_constraintType;
        QComboBox* _gui_affordanceType;
	QSignalMapper* _signalMapper;

        AtomicConstraintPtr _constraint;

        // should be static
        std::vector<affordance::AffPtr> _allAffordances;
        std::vector<std::string> _allJointNames;

    private slots:
        void updateStateFromElements();
    
    };

    typedef boost::shared_ptr<Qt4Constraint> Qt4ConstraintPtr;
}
