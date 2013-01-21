#include "mainwindow.h"

#include <state/state_gfe.h>
#include "affordance/OpenGL_Affordance.h"
#include <vector>
using namespace std;
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
void MainWindow::demoPopulateConstraintMacros()
{
    AffPtr rhand = AffPtr(new AffordanceState("Right Hand", 235, 38252));
    AffPtr lhand = AffPtr(new AffordanceState("Left Hand",  852365807, 3723));
    AffPtr rfoot = AffPtr(new AffordanceState("Right Foot", 2, 51));
    AffPtr lfoot = AffPtr(new AffordanceState("Left Foot", 32872, 380));
    AffPtr wheel = AffPtr(new AffordanceState("Steering Wheel", 3287, 273));
    AffPtr gas   = AffPtr(new AffordanceState("Gas Pedal", 3202, 328720));
    AffPtr brake = AffPtr(new AffordanceState("Brake Pedal", 32802, 23093));

    AffPtr sphere 	= AffPtr(new AffordanceState("Pink Sphere", 3280, 328));
    AffPtr box 		= AffPtr(new AffordanceState("Yellow Box", 3280235, 3828));
    AffPtr cylinder = AffPtr(new AffordanceState("Blue Cylinder", 23802, 83));

    _worldState.affordances.push_back(rhand);
    _worldState.affordances.push_back(lhand);
    _worldState.affordances.push_back(rfoot);
    _worldState.affordances.push_back(lfoot);
    _worldState.affordances.push_back(wheel);
    _worldState.affordances.push_back(gas);
    _worldState.affordances.push_back(brake);

    _worldState.affordances.push_back(sphere);
    _worldState.affordances.push_back(box);
    _worldState.affordances.push_back(cylinder);
    
    AtomicConstraintPtr rfoot_gas_relation (new AtomicConstraint(rfoot, gas, AtomicConstraint::NORMAL));
    AtomicConstraintPtr lfoot_brake_relation(new AtomicConstraint(lfoot, brake, AtomicConstraint::TANGENT));
    AtomicConstraintPtr rhand_wheel_relation(new AtomicConstraint(rhand, wheel, AtomicConstraint::TANGENT));
    AtomicConstraintPtr lhand_wheel_relation(new AtomicConstraint(lhand, wheel, AtomicConstraint::TANGENT));

    ConstraintMacroPtr rfoot_gas  (new ConstraintMacro("Right Foot to Gas Pedal", rfoot_gas_relation));
    ConstraintMacroPtr lfoot_brake (new ConstraintMacro("Left Foot to Brake Pedal", lfoot_brake_relation));                                                                                            
    ConstraintMacroPtr rhand_wheel (new ConstraintMacro("Right Hand To Wheel", rhand_wheel_relation));
    ConstraintMacroPtr lhand_wheel (new ConstraintMacro("Left Hand To Wheel", lhand_wheel_relation));

    _authoringState._all_gui_constraints.push_back((Qt4ConstraintMacroPtr)new Qt4ConstraintMacro(rfoot_gas));
    _authoringState._all_gui_constraints.push_back((Qt4ConstraintMacroPtr)new Qt4ConstraintMacro(lfoot_brake));
    _authoringState._all_gui_constraints.push_back((Qt4ConstraintMacroPtr)new Qt4ConstraintMacro(rhand_wheel));
    _authoringState._all_gui_constraints.push_back((Qt4ConstraintMacroPtr)new Qt4ConstraintMacro(lhand_wheel));
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
    for (std::map< std::string, State_GFE_Joint >::const_iterator it = joints.begin(); it != joints.end(); it++)
    {
    	const State_GFE_Joint& state_gfe_joint = it->second;
    	joint_names.push_back(state_gfe_joint.id());
    }


    for(uint i = 0; i < _worldState.affordances.size(); i++)
      {
    	AffConstPtr next = _worldState.affordances[i];
    	if (next->_otdf_id == AffordanceState::CYLINDER ||
    		next->_otdf_id == AffordanceState::BOX ||
    		next->_otdf_id == AffordanceState::SPHERE)
    	{
    		OpenGL_Affordance *asGlAff = new OpenGL_Affordance(*next); //todo : get rid of this memory leak
    		_widget_opengl.opengl_scene().add_object(*asGlAff);
    	}
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
    QWidget* leftside = new QWidget();
    QVBoxLayout* vbox = new QVBoxLayout();

    QWidget* topmenugroup = new QWidget();
    QHBoxLayout* topmenu = new QHBoxLayout();
    topmenu->addWidget(new QLabel("Action: "));
    topmenu->addWidget(new QLineEdit("Ingress"));
    QLabel* actionTypeLabel = new QLabel("acts on: ");
    topmenu->addWidget(actionTypeLabel);
    QComboBox* actionType = new QComboBox();
    actionType->insertItem(0, "Vehicle");
    actionType->insertItem(0, "Door");
    actionType->insertItem(0, "Ladder");
    actionType->insertItem(0, "Table");
    topmenu->addWidget(actionType);
    QPushButton* savebutton = new QPushButton("Save");
    topmenu->addWidget(savebutton);
    topmenu->addSpacerItem(new QSpacerItem(100, 0));
    QPushButton* loaddiff = new QPushButton("load a different action");
    topmenu->addWidget(loaddiff);
    topmenugroup->setLayout(topmenu);
    vbox->addWidget(topmenugroup);

    //joint_names = getJointNames("/home/drc/drc/software/models/mit_gazebo_models/mit_robot/model.urdf");
    // load the constraints
    //std::vector<AffPtr> _all_affordances =
    _constraint_vbox = new QVBoxLayout(); 
    QWidget* constraintWrapper = new QWidget();
    constraintWrapper->setLayout(_constraint_vbox);
    vbox->addWidget(constraintWrapper);
    demoPopulateConstraintMacros();
    makeGUIFromConstraintMacros();

    QGroupBox* mediaControls = new QGroupBox();
    QHBoxLayout* mediaControlsLayout = new QHBoxLayout();
    QPushButton* fbwd = new QPushButton();
    QPushButton* bwd = new QPushButton();
    QPushButton* play = new QPushButton();
    QPushButton* fwd = new QPushButton();
    QPushButton* ffwd = new QPushButton();

    // see http://www.qtcentre.org/wiki/index.php?title=Embedded_resources
    QPixmap pixmap1(":/trolltech/styles/commonstyle/images/media-skip-backward-32.png");
    fbwd->setIcon(QIcon(pixmap1));
    fbwd->setIconSize(pixmap1.rect().size());

    QPixmap pixmap2(":/trolltech/styles/commonstyle/images/media-seek-backward-32.png");
    bwd->setIcon(QIcon(pixmap2));
    bwd->setIconSize(pixmap2.rect().size());

    QPixmap pixmap3(":/trolltech/styles/commonstyle/images/media-pause-32.png");
    play->setIcon(QIcon(pixmap3));
    play->setIconSize(pixmap3.rect().size());

    QPixmap pixmap4(":/trolltech/styles/commonstyle/images/media-seek-forward-32.png");
    fwd->setIcon(QIcon(pixmap4));
    fwd->setIconSize(pixmap4.rect().size());

    QPixmap pixmap5(":/trolltech/styles/commonstyle/images/media-skip-forward-32.png");
    ffwd->setIcon(QIcon(pixmap5));
    ffwd->setIconSize(pixmap5.rect().size());

    mediaControlsLayout->addSpacerItem(new QSpacerItem(100, 0));
    mediaControlsLayout->addWidget(fbwd);
    mediaControlsLayout->addWidget(bwd);
    mediaControlsLayout->addWidget(play);
    mediaControlsLayout->addWidget(fwd);
    mediaControlsLayout->addWidget(ffwd);
    mediaControlsLayout->addSpacerItem(new QSpacerItem(100, 0));
    mediaControls->setLayout(mediaControlsLayout);
    play->resize(play->width() * 2, play->height());

    QGroupBox* widgetWrapper = new QGroupBox();
    widgetWrapper->setStyleSheet("QGroupBox { border: 1px solid gray; border-radius: 0px; padding: 0px; margin: 0px; background-color: black; }");
    QVBoxLayout* widgetWrapperLayout = new QVBoxLayout();
    widgetWrapperLayout->setSpacing(0);
    widgetWrapperLayout->setMargin(0);
    widgetWrapperLayout->addWidget(&_widget_opengl);
    widgetWrapperLayout->addWidget(mediaControls);
    widgetWrapper->setLayout(widgetWrapperLayout);

    QVBoxLayout* rightsidelayout = new QVBoxLayout();
    QWidget* rightside = new QWidget();
    _jointSlider = new QSlider( Qt::Horizontal, this );
    _jointNameLabel = new QLabel();
    DefaultValueSlider* scrubber = new DefaultValueSlider( Qt::Horizontal, this );
    rightsidelayout->addWidget(_jointNameLabel);
    rightsidelayout->addWidget(_jointSlider);
    rightsidelayout->addWidget(widgetWrapper);
    rightsidelayout->addWidget(scrubber );
//    rightside->setStyleSheet("QGroupBox { border: 1px solid black; border-radius: 3px; padding: 0px; background-color: black; } ");
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
    toolbarButtonsLayout->addSpacing(100);
    toolbarButtonsLayout->addWidget(new QPushButton("+ add constraint"));
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

 DatabaseManager* dbm = new DatabaseManager(fileName.toStdString());
 dbm->parseFile();

  std::vector<ConstraintMacroPtr> revivedConstraintMacros;
  dbm->getConstraintMacros(revivedConstraintMacros);

//  for (int i = 0; i < _authoringState._all_gui_constraints.size(); i++) {
//      delete _authoringState._all_gui_constraints[i].get();
//  }

  _authoringState._all_gui_constraints.clear();
  for (int i = 0; i < revivedConstraintMacros.size(); i++) {
      _authoringState._all_gui_constraints.push_back((Qt4ConstraintMacroPtr)new Qt4ConstraintMacro(revivedConstraintMacros[i]));
  }
  makeGUIFromConstraintMacros();

}

void
MainWindow::
handleSaveAction() {
 QString fileName = QFileDialog::getSaveFileName(this,
     tr("Save Action"), "", tr("Action XML Files (*.xml)"));  

 vector<ConstraintMacroConstPtr> all_constraints;
 for (int i = 0; i < _authoringState._all_gui_constraints.size(); i++) {
     all_constraints.push_back(_authoringState._all_gui_constraints[i]->getConstraintMacro());
 }

 DatabaseManager* dbm = new DatabaseManager(fileName.toStdString());
 dbm->store(_worldState.affordances, all_constraints);
 
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

std::string 
MainWindow::
getSelectedJointName() {
    if (_authoringState._selected_gui_constraint != NULL)
	return _authoringState._selected_gui_constraint->getSelectedLinkName(); 
    return "";
}

void 
MainWindow::
updateJoint(int value) {
    std::string selectedJointName = getSelectedJointName();
    if (selectedJointName != "") {
	std::cout << "joint " << selectedJointName << " set to " <<  _jointSlider->value() / 100.0 << std::endl;
	_worldState.state_gfe.joint(selectedJointName).set_position( _jointSlider->value() / 100.0 );
	_worldState.colorRobot.set(_worldState.state_gfe);
	_widget_opengl.update();
    }
}

/*
 * change the functionality of the slider to actuate currently selected link
 */
void 
MainWindow::
handleRobotLinkChange() {
    std::string selectedJointName = getSelectedJointName();
    _worldState.colorRobot.setSelectedJoint(selectedJointName);
    _widget_opengl.update();
    _jointNameLabel->setText(QString::fromStdString(selectedJointName));
}

void 
MainWindow::
setSelectedAction(QString activator) { 
    std::string activator_str = activator.toStdString();
    for (std::vector<int>::size_type i = 0; i != _authoringState._all_gui_constraints.size(); i++) {
	if (activator_str.compare(_authoringState._all_gui_constraints[i]->getConstraintMacro()->getName()) == 0) {
	    _authoringState._all_gui_constraints[i]->setSelected(true);
	    _authoringState._selected_gui_constraint = _authoringState._all_gui_constraints[i];
	} else {
	    _authoringState._all_gui_constraints[i]->setSelected(false);
	}
    }
/*
    handleRobotLinkChange();
*/
}

void
MainWindow::
makeGUIFromConstraintMacros() {
    // Get the toggle panels from the Qt4ConstraintMacro objects and populate the gui
    for(std::vector<int>::size_type i = 0; i != _authoringState._all_gui_constraints.size(); i++) 
      {
	_authoringState._all_gui_constraints[i]->setAffordances(_worldState.affordances, _worldState.affordances);
	
	TogglePanel* tp = _authoringState._all_gui_constraints[i]->getPanel();
	// todo: currently using constraint name as UID
	_signalMapper->setMapping(_authoringState._all_gui_constraints[i].get(), 
				  QString::fromStdString(_authoringState._all_gui_constraints[i]->getConstraintMacro()->getName()));
	connect(_authoringState._all_gui_constraints[i].get(), SIGNAL(activatedSignal()), _signalMapper, SLOT(map()));
	connect(_signalMapper, SIGNAL(mapped(QString)), this, SLOT(setSelectedAction(QString)));
	
	_constraint_vbox->addWidget(tp);
    }
}
