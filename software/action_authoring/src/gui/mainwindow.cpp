#include "mainwindow.h"

#include <state/state_gfe.h>
#include "affordance/AffordanceState.h"

using namespace KDL;
using namespace Eigen;
using namespace opengl;
using namespace state;
using namespace collision;
using namespace action_authoring;
using namespace affordance;
using namespace boost;

//using namespace collision_detection;

/*
 * Filler method to populate affordance and constraint lists until we get proper
 * data sources set up.
 */ 
void MainWindow::demoPopulateConstraints()
{
 /*    TODO : read from server
    AtomicConstraintPtr rfoot_gas   = AtomicConstraintPtr(new AtomicConstraint("Gas Pedal Constraint", rfoot, gas,
    													  AtomicConstraint::NORMAL));
    AtomicConstraintPtr lfoot_brake = AtomicConstraintPtr(new AtomicConstraint("Brake Pedal Constraint", lfoot, brake,
    													  AtomicConstraint::TANGENT));
    AtomicConstraintPtr rhand_wheel = AtomicConstraintPtr(new AtomicConstraint("Right Hand Wheel Constraint", rhand, wheel,
    													AtomicConstraint::TANGENT));
    AtomicConstraintPtr lhand_wheel = AtomicConstraintPtr(new AtomicConstraint("Left Hand Wheel Constraint", lhand, wheel,
    													   AtomicConstraint::TANGENT));

    _authoringState._all_constraints.push_back(rfoot_gas);
    _authoringState._all_constraints.push_back(lfoot_brake);
    _authoringState._all_constraints.push_back(rhand_wheel);
    _authoringState._all_constraints.push_back(lhand_wheel);
    */
}

/*
 * Create a waypoint entry GUI element
 */
TogglePanel* MainWindow::
createWaypointGUI(AtomicConstraintPtr waypoint_constraint, std::vector<std::string> joint_names)
{
    QString waypointTitle = QString::fromStdString(waypoint_constraint->getName());
    TogglePanel* tp = new TogglePanel(this, waypointTitle);

    QGroupBox* groupBox = new QGroupBox();
    QLineEdit* name = new QLineEdit(waypointTitle);
    QPushButton* editButton = new QPushButton(QString::fromUtf8("edit"));
    QComboBox* radio1 = new QComboBox();
    QComboBox* radio2 = new QComboBox();
    QComboBox* radio3 = new QComboBox();

    
    vector<AffPtr> affordances;
    _worldState.affordances.getAllAffordances(affordances);

    for (int i = 0; i < affordances.size(); i++)
      {
        radio3->insertItem(0, QString::fromStdString(affordances[i]->getName()));
      }
    
    for (int i = 0; i < joint_names.size(); i++) 
      {
        radio1->insertItem(0, QString::fromStdString(joint_names[i]));
    }

    // update the global maps
    _all_panels[waypoint_constraint->getName()] = tp;
    _all_robot_link_combos[waypoint_constraint->getName()] = radio1;

    radio2->insertItem(0, "3D Offset");
    radio2->insertItem(0, "tangent to");
    radio2->insertItem(0, "normal to");
    radio2->insertItem(0, "near");
    radio2->insertItem(0, "point touches");
    radio2->insertItem(0, "surface touches");
    radio2->insertItem(0, "grasps");

    //radio1->setChecked(true);

    QVBoxLayout* vbox = new QVBoxLayout;
    vbox->addWidget(name);
    
    QHBoxLayout* hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel("robot"));
    hbox->addWidget(radio1);
    hbox->addWidget(new QLabel("relation"));
    hbox->addWidget(radio2);
    hbox->addWidget(new QLabel("affordance"));
    hbox->addWidget(radio3);
    hbox->addWidget(editButton);
    hbox->addStretch(1);
    QWidget* boxcontainer = new QWidget();
    boxcontainer->setLayout(hbox);
    vbox->addWidget(boxcontainer);
    
//    _signalMapper->setMapping(name, waypointTitle);
    _signalMapper->setMapping(radio1, waypointTitle);
    _signalMapper->setMapping(editButton, waypointTitle);
    
//    connect(name, SIGNAL(textChanged(QString)), this, SLOT(map()));
//    connect(_signalMapper, SIGNAL(mapped(QString)), SLOT(updateWaypointName(QString)));

    connect(radio1, SIGNAL(currentIndexChanged(int)), _signalMapper, SLOT (map()));
    connect(_signalMapper, SIGNAL(mapped(QString)), SLOT(handleRobotLinkChange(QString)));

    connect(editButton, SIGNAL(released()), _signalMapper, SLOT (map()));
    connect(_signalMapper, SIGNAL(mapped(QString)), SLOT(setSelectedAction(QString)));

    groupBox->setLayout(vbox);
    tp->addWidget(groupBox);
    return tp;
}

MainWindow::MainWindow(const shared_ptr<lcm::LCM> &theLcm, QWidget* parent)
	: _widget_opengl(),
	  _worldState(theLcm)
{
    // setup the OpenGL 6scene
	_worldState.state_gfe.from_urdf();
	_worldState.colorRobot.set(_worldState.state_gfe);

    // build collision objects
	/*todo  construct collision objects from the affordances
	_collision_object_box = new Collision_Object_Box("box1", Vector3f(0.25, 0.25, 0.25), Vector3f(1.0, 0.0, 0.0), Vector4f(1.0, 0.0, 0.0, 0.0));
    _collision_object_cylinder = new Collision_Object_Cylinder("cylinder1", 0.25, 0.25, Vector3f(0.0, 1.0, 0.0), Vector4f(1.0, 0.0, 0.0, 0.0));
    _collision_object_sphere = new Collision_Object_Sphere("sphere1", 0.125, Vector3f(-0.5, -0.5, 0.0), Vector4f(1.0, 0.0, 0.0, 0.0));
    //_collision_object_gfe("robot1");
*/
    // read the joints from the robot state
    std::vector<std::string> joint_names;
    std::map< std::string, State_GFE_Joint > joints = _worldState.state_gfe.joints();
    for (std::map< std::string, State_GFE_Joint >::const_iterator it = joints.begin(); it != joints.end(); it++) {
	const State_GFE_Joint& state_gfe_joint = it->second;
	joint_names.push_back(state_gfe_joint.id());
    }

    /*todo
    _widget_opengl.opengl_scene().add_object(_opengl_object_box);
    _widget_opengl.opengl_scene().add_object(_opengl_object_cylinder);
    _widget_opengl.opengl_scene().add_object(_opengl_object_sphere);
*/
    _widget_opengl.opengl_scene().add_object(_worldState.colorRobot);

    /*todo
    _widget_opengl.add_object_with_collision(_collision_object_box);
    _widget_opengl.add_object_with_collision(_collision_object_cylinder);
    _widget_opengl.add_object_with_collision(_collision_object_sphere);
    	*/
    //_widget_opengl.add_object_with_collision(_collision_object_gfe);


    _signalMapper = new QSignalMapper(this);

    QVBoxLayout* layout = new QVBoxLayout();
    layout->setMargin(0);

    QSplitter* splitter = new QSplitter();
    QGroupBox* leftside = new QGroupBox();
    QVBoxLayout* vbox = new QVBoxLayout();

    QWidget* topmenugroup = new QWidget();
    QHBoxLayout* topmenu = new QHBoxLayout();
    topmenu->addWidget(new QLabel("Action: "));
    topmenu->addWidget(new QLineEdit("Ingress"));
    QPushButton* savebutton = new QPushButton("Save");
    topmenu->addWidget(savebutton);
    topmenu->addSpacerItem(new QSpacerItem(100, 0));
    QPushButton* loaddiff = new QPushButton("load a different action");
    topmenu->addWidget(loaddiff);
    topmenugroup->setLayout(topmenu);
    vbox->addWidget(topmenugroup);

    //joint_names = getJointNames("/home/drc/drc/software/models/mit_gazebo_models/mit_robot/model.urdf");

    demoPopulateConstraints();
    for(std::vector<int>::size_type i = 0; i != _authoringState._all_constraints.size(); i++) {
	TogglePanel* tp = createWaypointGUI(_authoringState._all_constraints[i], joint_names);
	vbox->addWidget(tp);
    }
    

    QVBoxLayout* rightsidelayout = new QVBoxLayout();
    QGroupBox* rightside = new QGroupBox();
    _jointSlider = new QSlider( Qt::Horizontal, this );
    rightsidelayout->addWidget(&_widget_opengl );
    rightsidelayout->addWidget(_jointSlider);
    rightside->setLayout(rightsidelayout);

    splitter->addWidget( leftside);
    splitter->addWidget(rightside);

    layout->addWidget(splitter);
    QGroupBox* toolbarButtons = new QGroupBox(""); //controls
    QHBoxLayout* toolbarButtonsLayout = new QHBoxLayout();
    toolbarButtonsLayout->addWidget(new QPushButton("delete"));
    toolbarButtonsLayout->addSpacing(200);
    toolbarButtonsLayout->addWidget(new QPushButton("move up"));
    toolbarButtonsLayout->addWidget(new QPushButton("move down"));
    toolbarButtons->setLayout(toolbarButtonsLayout);
    vbox->addWidget(toolbarButtons);
    vbox->addStretch(1);
    leftside->setLayout(vbox);

    // fix the borders on the group boxes
    this->setStyleSheet("QGroupBox { border: 1px solid gray; border-radius: 3px; padding: 5px; } "
			"QGroupBox::title { background-color: transparent; "
			"subcontrol-position: top left; /* position at the top left*/ "
			"padding:2 5px; } ");

    this->setLayout(layout);

    // wire up the buttons
    connect(savebutton, SIGNAL(released()), this, SLOT(handleSaveAction()));
    connect(loaddiff, SIGNAL(released()), this, SLOT(handleLoadAction()));
    connect(_jointSlider, SIGNAL(valueChanged(int)), this, SLOT(updateJoint(int)));
}

MainWindow::~MainWindow()
{

}

void
MainWindow::
handleLoadAction() {
 QString fileName = QFileDialog::getOpenFileName(this,
     tr("Open Action"), "", tr("Action XML Files (*.xml)"));
}

void
MainWindow::
handleSaveAction() {
 QString fileName = QFileDialog::getSaveFileName(this,
     tr("Save Action"), "", tr("Action XML Files (*.xml)"));  
}

void
MainWindow::
handleDeleteWaypoint() {

}

void
MainWindow::
handleMoveUp() {

}

void
MainWindow::
handleMoveDown() {

}

void
MainWindow::
handleToggleExpandContract() {

}

void 
MainWindow::
updateJoint(int value) {
    if (_selectedJointName != "") {
	std::cout << "joint " << _selectedJointName << " set to " <<  _jointSlider->value() / 100.0 << std::endl;
	_worldState.state_gfe.joint(_selectedJointName).set_position( _jointSlider->value() / 100.0 );
	_worldState.colorRobot.set(_worldState.state_gfe);
	_widget_opengl.update(); //_opengl_widget.update();
	//update();
    }
}

/*
 * change the functionality of the slider to actuate currently selected link
 */
void 
MainWindow::
handleRobotLinkChange(QString waypointName) {
    // start by getting the name of the link currently selected
    QComboBox* selected_combo = _all_robot_link_combos.find(waypointName.toStdString())->second;
    // find the selected text
    _selectedJointName = selected_combo->currentText().toStdString();  //itemData(selected_combo->currentIndex());
    _worldState.colorRobot.setSelectedJoint(_selectedJointName);
    _widget_opengl.update(); //_opengl_widget.update();
}

void 
MainWindow::
setSelectedAction(QString waypointName) {
    TogglePanel* selected_panel = _all_panels.find(waypointName.toStdString())->second;
    std::map<std::string, TogglePanel*>::iterator iter;
    for (iter = _all_panels.begin(); iter != _all_panels.end(); iter++) {
        TogglePanel* other_panel = iter->second;
	other_panel->setSelected(false);
    }
    selected_panel->setSelected(true);
}
