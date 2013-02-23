#ifndef QT4CONSTRAINT_H
#define QT4CONSTRAINT_H

// QT includes
#include <QtGui>
#include <QWidget>
#include <QApplication>
// Custom QT widget
#include "../qt4_widgets/TogglePanel.h"
#include <action_authoring/ConstraintMacro.h>
#include <action_authoring/RelationState.h>
#include <action_authoring/ManipulationRelation.h>
#include <action_authoring/OffsetRelation.h>
#include <action_authoring/PointContactRelation.h>

namespace action_authoring
{

// Qt4 gui objects corresponding to the members of the superclass
class Qt4ConstraintMacro : public QWidget
{

    Q_OBJECT

public:
    Qt4ConstraintMacro(ConstraintMacroPtr atomicConstraintMacro, int constraintIndex);
    ~Qt4ConstraintMacro();
    TogglePanel *getPanel();
    // read the state from the gui elements already referenced
    void updateElementsFromState();
    void setSelected(bool selected);

    void setModelObjects(std::vector<affordance::AffConstPtr> &affordances,
                         std::vector<affordance::ManipulatorStateConstPtr> &manipulators);
    ConstraintMacroPtr getConstraintMacro();
    std::string getSelectedLinkName();
    bool isInitialized();
    void setConstraintIndex(int constraintIndex);
    int getConstraintIndex()
    {
        return _constraintIndex;
    }
    void setActiveExternal();
    std::string getModePrompt();

private:
    std::map<affordance::GlobalUID, int> _affordance1IndexMap;
    std::map<affordance::GlobalUID, int> _affordance2IndexMap;

    int _constraintIndex;
    bool _initialized;
    bool _updatingElementsFromState;
    TogglePanel *_gui_panel;
    QLineEdit *_gui_name;
    QComboBox *_gui_robotJointType;
    QComboBox *_gui_constraintType;
    QComboBox *_gui_affordanceType;
    QDoubleSpinBox *_gui_time_lower_bound;
    QDoubleSpinBox *_gui_time_upper_bound;
    ConstraintMacroPtr _constraint;

    // should be static
    std::vector<affordance::AffConstPtr> _affordances;
    std::vector<affordance::ManipulatorStateConstPtr> _manipulators;

signals:
    void activatedSignal(Qt4ConstraintMacro *);

private slots:
    void updateStateFromElements();
    void setActive();

};

typedef boost::shared_ptr<Qt4ConstraintMacro> Qt4ConstraintMacroPtr;
}

#endif //QT4CONSTRAINT_H
