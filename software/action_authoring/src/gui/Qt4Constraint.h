#ifndef QT4CONSTRAINT_H
#define QT4CONSTRAINT_H

// QT includes
#include <QtGui>
#include <QWidget>
#include <QApplication>
// Custom QT widget
#include "togglepanel.h"
#include "action_authoring/AffordanceRelation.h"
#include "action_authoring/Constraint.h"

namespace action_authoring
{

    // Qt4 gui objects corresponding to the members of the superclass
    class Qt4Constraint : public QWidget {
    
    Q_OBJECT

    public:
	Qt4Constraint(ConstraintPtr atomicConstraint);
	~Qt4Constraint();
        TogglePanel* getPanel();
	// read the state from the gui elements already referenced
	void updateElementsFromState();
	void setSelected(bool selected);

	void setAffordances(std::vector<affordance::AffPtr> &leftSideAffordances, 
			    std::vector<affordance::AffPtr> &rightSideAffordances);
	ConstraintPtr getConstraint();
	std::string getSelectedLinkName();
	
    protected:
	std::map<affordance::GlobalUID, int> _affordance1IndexMap;
	std::map<affordance::GlobalUID, int> _affordance2IndexMap;
	
	TogglePanel* _gui_panel;
        QLineEdit* _gui_name;
        QComboBox* _gui_robotJointType;
        QComboBox* _gui_constraintType;
        QComboBox* _gui_affordanceType;
        ConstraintPtr _constraint;

        // should be static
        std::vector<affordance::AffPtr> _leftSideAffordances;
        std::vector<affordance::AffPtr> _rightSideAffordances;

    signals:
	void activatedSignal();

    private slots:
        void updateStateFromElements();
        void setActive();
    
    };

    typedef boost::shared_ptr<Qt4Constraint> Qt4ConstraintPtr;
}

#endif //QT4CONSTRAINT_H
