#include "Qt4Constraint.h"

#include <iostream>

using namespace action_authoring;

Qt4Constraint::
Qt4Constraint(AtomicConstraintPtr constraint) : _gui_name(new QLineEdit()),
				     _gui_robotJointType(new QComboBox()),
				     _gui_constraintType(new QComboBox()),
				     _gui_affordanceType(new QComboBox()),
				     _signalMapper(new QSignalMapper()),
				     _gui_panel(new TogglePanel(this, "test"))
{
    // constructor
    _constraint = constraint;
}

std::string
Qt4Constraint::
getSelectedLinkName() {
    return _gui_robotJointType->currentText().toStdString();
}

TogglePanel* 
Qt4Constraint::
getPanel() {
    QString waypointTitle = QString::fromStdString(_constraint->getName());
    std::cout << "title is " << _constraint->getName() << std::endl;
    _gui_panel->setTitle(waypointTitle);

    QGroupBox* groupBox = new QGroupBox();
    QPushButton* editButton = new QPushButton(QString::fromUtf8("edit"));

    // update the global maps
    //_all_panels[waypoint_constraint->getName()] = outputPanel;
    //_all_robot_link_combos[waypoint_constraint->getName()] = radio1;

    _gui_constraintType->insertItem(0, "3D Offset");
    _gui_constraintType->insertItem(0, "tangent to");
    _gui_constraintType->insertItem(0, "normal to");
    _gui_constraintType->insertItem(0, "near");
    _gui_constraintType->insertItem(0, "point touches");
    _gui_constraintType->insertItem(0, "surface touches");
    _gui_constraintType->insertItem(0, "grasps");

    QVBoxLayout* vbox = new QVBoxLayout;
    _gui_name->setText(QString::fromStdString(_constraint->getName()));
    vbox->addWidget(_gui_name);
    
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
    
    _signalMapper->setMapping(_gui_robotJointType, waypointTitle);
    _signalMapper->setMapping(editButton, waypointTitle);

    connect(_gui_name, SIGNAL(textChanged(QString)), this, SLOT(updateStateFromElements()));    
    connect(_gui_robotJointType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateStateFromElements()));
    connect(_gui_constraintType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateStateFromElements()));
    connect(_gui_affordanceType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateStateFromElements()));

    connect(editButton, SIGNAL(released()), this, SLOT(setActive()));

//    connect(_gui_robotJointType, SIGNAL(currentIndexChanged(int)), _signalMapper, SLOT (map()));
//    connect(_signalMapper, SIGNAL(mapped(QString)), SLOT(handleRobotLinkChange(QString)));

//    connect(editButton, SIGNAL(released()), _signalMapper, SLOT (map()));
//    connect(_signalMapper, SIGNAL(mapped(QString)), SLOT(setSelectedAction(QString)));

    groupBox->setLayout(vbox);
    _gui_panel->addWidget(groupBox);
    return _gui_panel;
}

void
Qt4Constraint::
setAffordances(std::vector<affordance::AffPtr> &allAffordances) {
    // set the protected member
    _allAffordances = allAffordances;
    // update the combo box
    for (int i = 0; i < _allAffordances.size(); i++) {
        _gui_affordanceType->insertItem(0, QString::fromStdString(_allAffordances[i]->getName()));
    }
}

void
Qt4Constraint::
setJointNames(std::vector<std::string> &allJointNames) {
    // set the protected member
    _allJointNames = allJointNames;

    for (int i = 0; i < _allJointNames.size(); i++) {
        _gui_robotJointType->insertItem(0, QString::fromStdString(_allJointNames[i]));
    }
}

AtomicConstraintPtr
Qt4Constraint::
getConstraint() {
    return _constraint;
}

// todo; very primitive; need affordance UID type
void
Qt4Constraint::
updateStateFromElements() {
    _constraint->setName(_gui_name->text().toStdString());
    _gui_panel->setTitle(QString::fromStdString(_constraint->getName()));

    // find the robot joint affordance by ID and update the constraint appropriately
    std::string currentJointName = getSelectedLinkName();
    for (int i = 0; i < _allJointNames.size(); i++) {
	if (_allJointNames[i].compare(currentJointName) == 0) {
	    std::cout << "found joint name " << _allJointNames[i] << std::endl;
	    //_constraint->setAffordance1(_allAffordances[i]);
	    break;
	}
    }

    // find the robot joint affordance by ID and update the constraint appropriately
    std::string currentAffordanceName = _gui_affordanceType->currentText().toStdString();
    for (int i = 0; i < _allAffordances.size(); i++) {
	if (_allAffordances[i]->getName().compare(currentAffordanceName)) {
	    _constraint->setAffordance2(_allAffordances[i]);
	    break;
	}
    }
    setActive();
}

void 
Qt4Constraint::
setActive() {
    emit activatedSignal();
}

void 
Qt4Constraint::
setSelected(bool selected) {
    _gui_panel->setSelected(selected);
}
