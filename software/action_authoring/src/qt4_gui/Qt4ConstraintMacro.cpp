#include "Qt4ConstraintMacro.h"
#include <action_authoring/ManipulationRelation.h>
#include <iostream>
#include <sstream>

using namespace action_authoring;

Qt4ConstraintMacro::
Qt4ConstraintMacro(ConstraintMacroPtr constraint, int constraintIndex) : 
				     _gui_panel(new TogglePanel(this, "test", true)),
				     _gui_name(new QLineEdit()),
				     _gui_robotJointType(new QComboBox()),
				     _gui_constraintType(new QComboBox()),
				     _gui_affordanceType(new QComboBox()),
				     _gui_time_lower_bound(new QDoubleSpinBox()),
				     _gui_time_upper_bound(new QDoubleSpinBox())
{
    // constructor
    _constraint = constraint;
    _gui_time_lower_bound->setSuffix(" sec");
    _gui_time_upper_bound->setSuffix(" sec");
    _initialized = false;
    _constraintIndex = constraintIndex;
    _updatingElementsFromState = false;
}

Qt4ConstraintMacro::
~Qt4ConstraintMacro() {
    //TODO; hack
    std::cout << "destructive destructor destructing" << std::endl;
    _gui_panel->setParent(NULL);
    //delete _gui_panel;
}

std::string
Qt4ConstraintMacro::
getSelectedLinkName() {
    return _gui_robotJointType->currentText().toStdString();
}

void
Qt4ConstraintMacro::
setConstraintIndex(int constraintIndex) {
    _constraintIndex = constraintIndex;
    updateElementsFromState();
}

bool
Qt4ConstraintMacro::
isInitialized() {
    return _initialized;
}

TogglePanel* 
Qt4ConstraintMacro::
getPanel() {
//    QString waypointTitle = QString::fromStdString(_constraint->getName());
//    std::cout << "title is " << _constraint->getName() << std::endl;
    if (_initialized) {
	return _gui_panel;
    }
    
    QGroupBox* groupBox = new QGroupBox();
    QPushButton* editButton = new QPushButton(QString::fromUtf8("edit"));

    // update the global maps
    //_all_panels[waypoint_constraint->getName()] = outputPanel;
    //_all_robot_link_combos[waypoint_constraint->getName()] = radio1;

    _gui_constraintType->insertItem(0, "point to point");
    _gui_constraintType->insertItem(0, "3D Offset");
    _gui_constraintType->insertItem(0, "tangent to");
    _gui_constraintType->insertItem(0, "normal to");
    _gui_constraintType->insertItem(0, "near");
    _gui_constraintType->insertItem(0, "surface touches");
    _gui_constraintType->insertItem(0, "grasps");

    QVBoxLayout* vbox = new QVBoxLayout;
    QHBoxLayout* top_line_hbox = new QHBoxLayout();
    QWidget* top_line_container = new QWidget();
    top_line_hbox->addWidget(_gui_name);
    top_line_hbox->addWidget(new QLabel("lower bound: "));
    top_line_hbox->addWidget(_gui_time_lower_bound);
    top_line_hbox->addWidget(new QLabel("upper bound: "));
    top_line_hbox->addWidget(_gui_time_upper_bound);
    top_line_hbox->addWidget(new QPushButton("click to bind"));
    top_line_container->setLayout(top_line_hbox);
    vbox->addWidget(top_line_container);
    
    QHBoxLayout* hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel("robot"));
    hbox->addWidget(_gui_robotJointType);
    hbox->addWidget(new QLabel("relation"));
    hbox->addWidget(_gui_constraintType);
    hbox->addWidget(new QLabel("affordance"));
    hbox->addWidget(_gui_affordanceType);
    hbox->addWidget(editButton);
    hbox->addStretch(1);
    QWidget* boxcontainer = new QWidget();
    boxcontainer->setLayout(hbox);
    vbox->addWidget(boxcontainer);

    // previous constraints checkboxes
    TogglePanel* checkboxGroup = new TogglePanel(_gui_panel, "constraints still in effect", false);
    QGroupBox* checkboxGroupBox = new QGroupBox();
    QHBoxLayout* checkboxGroupLayout = new QHBoxLayout();
    for (int i = 0; i < _constraintIndex; i++) {
	QCheckBox* q1 = new QCheckBox(QString("previous constraint %1").arg(i));
	q1->setChecked(true);
	checkboxGroupLayout->addWidget(q1);
    }
    checkboxGroupBox->setLayout(checkboxGroupLayout);
    checkboxGroup->addWidget(checkboxGroupBox);
    vbox->addWidget(checkboxGroup);

    groupBox->setLayout(vbox);

    // MUST go before the QT connections have been made 
    updateElementsFromState();
    
    connect(_gui_time_lower_bound, SIGNAL(valueChanged(double)), this, SLOT(updateStateFromElements()));
    connect(_gui_time_upper_bound, SIGNAL(valueChanged(double)), this, SLOT(updateStateFromElements()));

    connect(_gui_name, SIGNAL(textChanged(QString)), this, SLOT(updateStateFromElements()));
    connect(_gui_robotJointType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateStateFromElements()));
    connect(_gui_constraintType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateStateFromElements()));
    connect(_gui_affordanceType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateStateFromElements()));
    connect(editButton, SIGNAL(released()), this, SLOT(setActive()));

    _gui_panel->addWidget(groupBox);
    _initialized = true;
    return _gui_panel;
}

void
Qt4ConstraintMacro::
setModelObjects(std::vector<affordance::AffConstPtr> &affordances,
		std::vector<affordance::ManipulatorStateConstPtr> &manipulators) {
    _affordances = affordances;
    _manipulators = manipulators;
    updateElementsFromState();
}

ConstraintMacroPtr
Qt4ConstraintMacro::
getConstraintMacro() {
    return _constraint;
}

// todo; very primitive; need affordance UID type
void
Qt4ConstraintMacro::
updateStateFromElements() {
    if (_updatingElementsFromState)  
	return;

    _constraint->setName(_gui_name->text().toStdString());
    _gui_panel->setTitle(QString("[%1] ").arg(_constraintIndex) + QString::fromStdString(_constraint->getName()));

    _constraint->setTimeLowerBound(_gui_time_lower_bound->value());
    _constraint->setTimeUpperBound(_gui_time_upper_bound->value());

    if (_gui_robotJointType->currentIndex() >= 0) 
	_constraint->getAtomicConstraint()->setManipulator(_manipulators[_gui_robotJointType->currentIndex()]);

    if (_gui_affordanceType->currentIndex() >= 0) 
	_constraint->getAtomicConstraint()->setAffordance(_affordances[_gui_affordanceType->currentIndex()]);

    setActive();
}


void 
Qt4ConstraintMacro::
setActiveExternal() {
    setActive();
}


void 
Qt4ConstraintMacro::
setActive() {
    emit activatedSignal(this);
}

void 
Qt4ConstraintMacro::
setSelected(bool selected) {
    _gui_panel->setSelected(selected);
}

void
Qt4ConstraintMacro::
updateElementsFromState() {
    // if we don't block signals, we willl overwrite state when we are
    // rebuilding the GUI elements because the values will change and
    // automatically trigger state updates
    _updatingElementsFromState = true;
    _gui_robotJointType->blockSignals(true);
    _gui_affordanceType->blockSignals(true);
    _gui_constraintType->blockSignals(true);

    _gui_name->setText(QString::fromStdString(_constraint->getName()));
    _gui_panel->setTitle(QString("[%1] ").arg(_constraintIndex) + QString::fromStdString(_constraint->getName()));

    _gui_robotJointType->clear();

    _gui_time_lower_bound->setValue(_constraint->getTimeLowerBound());
    _gui_time_upper_bound->setValue(_constraint->getTimeUpperBound());

    // re-initialize the maps
    _affordance1IndexMap.clear();
    _affordance2IndexMap.clear();

    if (_manipulators.size() > 0) {
	// update the left side combo box
	_gui_robotJointType->clear();
	for (int i = 0; i < (int)_manipulators.size(); i++) {
	    _gui_robotJointType->insertItem(i, QString::fromStdString(_manipulators[i]->getName()));
	    _affordance1IndexMap[_manipulators[i]->getGlobalUniqueId()] = i;
	}

	// select the correct joint name
	std::map<affordance::GlobalUID, int>::const_iterator it = _affordance1IndexMap.find(
	    _constraint->getAtomicConstraint()->getManipulator()->getGlobalUniqueId());
	if (it!=_affordance1IndexMap.end()) {
	    _gui_robotJointType->setCurrentIndex(it->second);
	    //std::cout << "found LH affordance iterator: " << it->second << std::endl;
	}
    }

    if (_affordances.size() > 0) {
	// update the right side combo box
	_gui_affordanceType->clear();
	for (int i = 0; i < (int)_affordances.size(); i++) {
	    if (_affordances[i] != NULL) {
		_gui_affordanceType->insertItem(i, QString::fromStdString(_affordances[i]->getName()));
		_affordance2IndexMap[_affordances[i]->getGlobalUniqueId()] = i;
	    } else {
		std::cout << "affordance " << i << " was NULL" << std::endl;
		// todo: throw exception
	    }
	}

	// select the current affordance
	std::map<affordance::GlobalUID, int>::const_iterator it2 = _affordance2IndexMap.find(
	    _constraint->getAtomicConstraint()->getAffordance()->getGlobalUniqueId());
	if (it2 != _affordance2IndexMap.end()) {
	    _gui_affordanceType->setCurrentIndex(it2->second);
//	    std::cout << "found RH index ((" << it2->second << ")): " << " for GUID " <<
//		_constraint->getAtomicConstraint()->getAffordance()->getGUIDAsString() << std::endl;
	}
    }

    _gui_robotJointType->blockSignals(false);
    _gui_affordanceType->blockSignals(false);
    _gui_constraintType->blockSignals(false);
    _updatingElementsFromState = false;
}

std::string
Qt4ConstraintMacro::
getModePrompt() {
    std::stringstream ss;
    ss << "<b>constraint " << _constraintIndex << "</b>:" <<
	_constraint->getAtomicConstraint()->getRelationState()->getState() << 
	"<br/><b>prompt: </b>" << _constraint->getAtomicConstraint()->getRelationState()->getPrompt();
    return ss.str();
}
