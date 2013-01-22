#ifndef QT4CONSTRAINT_H
#define QT4CONSTRAINT_H

// QT includes
#include <QtGui>
#include <QWidget>
#include <QApplication>
// Custom QT widget
#include "togglepanel.h"
#include "action_authoring/AtomicConstraint.h"
#include "action_authoring/ConstraintMacro.h"

namespace action_authoring
{

    // Qt4 gui objects corresponding to the members of the superclass
    class Qt4ConstraintMacro : public QWidget {
    
    Q_OBJECT

    public:
	Qt4ConstraintMacro(ConstraintMacroPtr atomicConstraintMacro);
	~Qt4ConstraintMacro();
        TogglePanel* getPanel();
	// read the state from the gui elements already referenced
	void updateElementsFromState();
	void setSelected(bool selected);

	void setAffordances(std::vector<affordance::AffConstPtr> &leftSideAffordances, 
			    std::vector<affordance::AffConstPtr> &rightSideAffordances);
	ConstraintMacroPtr getConstraintMacro();
	std::string getSelectedLinkName();
	
    private:
	std::map<affordance::GlobalUID, int> _affordance1IndexMap;
	std::map<affordance::GlobalUID, int> _affordance2IndexMap;
	
	TogglePanel* _gui_panel;
        QLineEdit* _gui_name;
        QComboBox* _gui_robotJointType;
        QComboBox* _gui_constraintType;
        QComboBox* _gui_affordanceType;
        ConstraintMacroPtr _constraint;

        // should be static
        std::vector<affordance::AffConstPtr> _leftSideAffordances;
        std::vector<affordance::AffConstPtr> _rightSideAffordances;

    signals:
	void activatedSignal(Qt4ConstraintMacro*);

    private slots:
        void updateStateFromElements();
        void setActive();
    
    };

    typedef boost::shared_ptr<Qt4ConstraintMacro> Qt4ConstraintMacroPtr;
}

#endif //QT4CONSTRAINT_H
