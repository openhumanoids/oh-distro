#include "MainWindow.h"

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
void MainWindow::handleAffordancesChanged()
{
  //------------------------REDRAW
  //clear the scene.  todo : is there a better way to handle 
  //memory management thru boost pointers?  opengl_scene doesn't use boost
  if (_worldState.glObjects.size() != _worldState.collisionObjs.size())
    throw InvalidStateException("glObjects and collisionObjs should have the same size");

  _widget_opengl.opengl_scene().clear_objects();  
  for(uint i = 0; i < _worldState.glObjects.size(); i++)
    {
      delete _worldState.glObjects[i];
      delete _worldState.collisionObjs[i];
    }
      
  _worldState.glObjects.clear();
  _worldState.collisionObjs.clear();

  for(uint i = 0; i < _worldState.affordances.size(); i++)
    {
      AffConstPtr next = _worldState.affordances[i];
      if (next->_otdf_id == AffordanceState::CYLINDER ||
	  next->_otdf_id == AffordanceState::BOX ||
	  next->_otdf_id == AffordanceState::SPHERE)
    	{
	  OpenGL_Affordance *asGlAff = new OpenGL_Affordance(next); 
	  _widget_opengl.opengl_scene().add_object(*asGlAff);
	  _worldState.glObjects.push_back(asGlAff);

	  // Create CollisionObject_Affordances, add to scene, and add to _worldState.glObjects
	  Collision_Object* collision_object_affordance;
	  KDL::Frame f = next->getFrame();
	  double q1, q2, q3, q4;
	  f.M.GetQuaternion(q1, q2, q3, q4);

	  if (next->_otdf_id == AffordanceState::BOX) {
	      collision_object_affordance = new Collision_Object_Box(next->getName(), 
		     Vector3f(next->length(), next->width(), next->height()), 
		     Vector3f(f.p.x(), f.p.y(), f.p.z()), Vector4f(q1, q2, q3, q4));
	  }
	  if (next->_otdf_id == AffordanceState::CYLINDER) {
	      collision_object_affordance = new Collision_Object_Cylinder(next->getName(), 
	             next->radius(), next->length(),
		     Vector3f(f.p.x(), f.p.y(), f.p.z()), Vector4f(q1, q2, q3, q4));
	  }
	  if (next->_otdf_id == AffordanceState::SPHERE) {
	      collision_object_affordance = new Collision_Object_Sphere(next->getName(),
		     next->radius(),
		     Vector3f(f.p.x(), f.p.y(), f.p.z()), Vector4f(q1, q2, q3, q4));
	  }
	  _widget_opengl.add_collision_object(collision_object_affordance);
	  _worldState.collisionObjs.push_back(collision_object_affordance);
    	}
    }


  //----------add robot and vehicle
  _widget_opengl.opengl_scene().add_object(_worldState.colorRobot); //add robot

  connect(&_widget_opengl, SIGNAL(raycastCallback(std::string)),
	  this, SLOT(selectedOpenGLObjectChanged(std::string)));

  _worldState.colorVehicle = new opengl::OpenGL_Object_DAE("vehicle", 
    "/home/drc/drc/software/models/mit_gazebo_models/" "mit_golf_cart/meshes/new_golf_cart.dae");
  _widget_opengl.opengl_scene().add_object(*_worldState.colorVehicle); //add vehicle

  _widget_opengl.update();
  //_widget_opengl.add_object_with_collision(_collision_object_gfe);

  //----------handle constraint macros 
  unordered_map<string, AffConstPtr> nameToAffMap;
  for(vector<AffConstPtr>::iterator iter = _worldState.affordances.begin();
      iter != _worldState.affordances.end();
      ++iter)
    {
      nameToAffMap[(*iter)->getName()] = *iter;
    }
 
  //todo: this is just demo code
/*
  AffConstPtr rfoot = nameToAffMap["Right Foot"];
  AffConstPtr lfoot = nameToAffMap["Left Foot"];
  AffConstPtr rhand = nameToAffMap["Right Hand"];
  AffConstPtr lhand = nameToAffMap["Left Hand"];

  AffConstPtr gas = nameToAffMap["Gas Pedal"];
  AffConstPtr brake = nameToAffMap["Brake Pedal"];
  AffConstPtr wheel = nameToAffMap["Steering Wheel"];

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
*/
  rebuildGUIFromState(_authoringState, _worldState);
}

MainWindow::MainWindow(const shared_ptr<lcm::LCM> &theLcm, QWidget* parent)
	: _widget_opengl(),
	  _worldState(theLcm),
	  _constraint_container(new QWidget()),
	  _constraint_vbox(new QVBoxLayout())
{
  // setup the OpenGL scene
  _worldState.state_gfe.from_urdf();
  _worldState.colorRobot.set(_worldState.state_gfe);
  
  //setup timer to look for affordance changes
  QTimer *timer = new QTimer;
  connect(timer, SIGNAL(timeout()), 
	  this, SLOT(affordanceUpdateCheck()));
  timer->start(1000); //1Hz  

    // read the joints from the robot state
    // create the manipulators from the robot's joints

    std::vector<std::string> joint_names;
    std::map< std::string, State_GFE_Joint > joints = _worldState.state_gfe.joints();
    for (std::map< std::string, State_GFE_Joint >::const_iterator it = joints.begin(); it != joints.end(); it++)
    {
    	const State_GFE_Joint& state_gfe_joint = it->second;
    	joint_names.push_back(state_gfe_joint.id());
	std::string id = state_gfe_joint.id();
	_worldState.manipulators.push_back((ManipulatorStateConstPtr)new ManipulatorState(id));
    }
  
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

    _constraint_container->setLayout(_constraint_vbox);
    vbox->addWidget(_constraint_container);

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
    _scrubber = new DefaultValueSlider( Qt::Horizontal, this );
    rightsidelayout->addWidget(_jointNameLabel);
    rightsidelayout->addWidget(_jointSlider);
    rightsidelayout->addWidget(widgetWrapper);
    rightsidelayout->addWidget(_scrubber);
//    rightside->setStyleSheet("QGroupBox { border: 1px solid black; border-radius: 3px; padding: 0px; background-color: black; } ");
    rightside->setLayout(rightsidelayout);

    splitter->addWidget( leftside);
    splitter->addWidget(rightside);

    layout->addWidget(splitter);
    QGroupBox* toolbarButtons = new QGroupBox(""); //controls
    QHBoxLayout* toolbarButtonsLayout = new QHBoxLayout();
    QPushButton* deletebutton = new QPushButton("delete");
    QPushButton* moveupbutton = new QPushButton("move up");
    QPushButton* movedownbutton = new QPushButton("move down");
    QPushButton* addconstraintbutton = new QPushButton("+ add constraint");
    toolbarButtonsLayout->addWidget(deletebutton);
    toolbarButtonsLayout->addSpacing(200);
    toolbarButtonsLayout->addWidget(moveupbutton);
    toolbarButtonsLayout->addWidget(movedownbutton);
    toolbarButtonsLayout->addSpacing(100);
    toolbarButtonsLayout->addWidget(addconstraintbutton);
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
    connect(deletebutton, SIGNAL(released()), this, SLOT(handleDeleteConstraint()));
    connect(moveupbutton, SIGNAL(released()), this, SLOT(handleMoveUp()));
    connect(movedownbutton, SIGNAL(released()), this, SLOT(handleMoveDown()));
    connect(addconstraintbutton, SIGNAL(released()), this, SLOT(handleAddConstraint()));

    connect(savebutton, SIGNAL(released()), this, SLOT(handleSaveAction()));
    connect(loaddiff, SIGNAL(released()), this, SLOT(handleLoadAction()));
    connect(_jointSlider, SIGNAL(valueChanged(int)), this, SLOT(updateJoint(int)));
}

MainWindow::~MainWindow()
{

}

void MainWindow::handleLoadAction() 
{
  QString fileName = QFileDialog::getOpenFileName(this,
      tr("Open Action"), "", tr("Action XML Files (*.xml)"));
  
  if (fileName.toStdString() == "") 
      return;

/*
  std::cout << "database manager" << std::endl;
  DatabaseManager* dbm = new DatabaseManager(fileName.toStdString()); //todo : Memory leak
  dbm->parseFile();
  std::cout << "file parsed successfully" << std::endl;
  std::vector<ConstraintMacroPtr> revivedConstraintMacros;
  dbm->getConstraintMacros(revivedConstraintMacros);
  std::cout << "constraint macros parsed successfully" << std::endl;

  //  for (int i = 0; i < _authoringState._all_gui_constraints.size(); i++) {
  //      delete _authoringState._all_gui_constraints[i].get();
  //  }
  
  // delete the children of the constraint box

  _authoringState._all_gui_constraints.clear();
  for (int i = 0; i < revivedConstraintMacros.size(); i++) 
  {
      _authoringState._all_gui_constraints.push_back((Qt4ConstraintMacroPtr)new Qt4ConstraintMacro(revivedConstraintMacros[i])); 
  }
  rebuildGUIFromState(_authoringState, _worldState);
*/
}

void
MainWindow::
handleSaveAction() {
 QString fileName = QFileDialog::getSaveFileName(this,
     tr("Save Action"), "", tr("Action XML Files (*.xml)"));  

 if (fileName.toStdString() == "") 
     return;
/*
 vector<ConstraintMacroConstPtr> all_constraints;
 for (int i = 0; i < _authoringState._all_gui_constraints.size(); i++) {
     all_constraints.push_back(_authoringState._all_gui_constraints[i]->getConstraintMacro());
 }

 DatabaseManager* dbm = new DatabaseManager(fileName.toStdString());
 dbm->store(_worldState.affordances, all_constraints);
*/ 
}

int
MainWindow::
getSelectedGUIConstraintIndex() {
    if (_authoringState._selected_gui_constraint == NULL)
	return -1;

    std::vector<Qt4ConstraintMacroPtr> &constraints = _authoringState._all_gui_constraints;
    std::vector<Qt4ConstraintMacroPtr>::iterator it = std::find(
	constraints.begin(), constraints.end(), 
	_authoringState._selected_gui_constraint);
    if (it != constraints.end()) {
	int i = it - constraints.begin();
	return i;
    } else {
	return -1;
    }
}

void
MainWindow::
handleDeleteConstraint() {
    int i = getSelectedGUIConstraintIndex();
    if (i > 0) {
	// delete the qt4 widget
	// TODO re-enable
	//_authoringState._all_gui_constraints.erase(_authoringState._all_gui_constraints[i]);
	rebuildGUIFromState(_authoringState, _worldState);
    } 
}

void
MainWindow::
handleAddConstraint() {
    if (_worldState.affordances.size() == 0 || _worldState.manipulators.size() == 0) {
	QMessageBox msgBox;
	msgBox.setText("Need at least one affordance and manipulator to add a constraint");
	msgBox.exec();
	return;
    }

    AffConstPtr left = _worldState.affordances[0];
    ManipulatorStateConstPtr manip = _worldState.manipulators[0];

    RelationStatePtr relstate(new RelationState(RelationState::UNDEFINED));
    ManRelPtr new_relation(new ManipulationRelation(left, manip, relstate));

    AtomicConstraintPtr new_constraint(new AtomicConstraint(new_relation));
    ConstraintMacroPtr rfoot_gas  (new ConstraintMacro("Untitled", new_constraint));

    _authoringState._all_gui_constraints.push_back((Qt4ConstraintMacroPtr)new Qt4ConstraintMacro(rfoot_gas));
    rebuildGUIFromState(_authoringState, _worldState);
}

void
MainWindow::
moveQt4Constraint(bool up) {
    int i = getSelectedGUIConstraintIndex();
    std::vector<Qt4ConstraintMacroPtr> &constraints = _authoringState._all_gui_constraints;
    if (up && i > 0 || (! up) && i < constraints.size() - 1) {
	std::swap(constraints[i], constraints[i + (up ? -1 : 1)]);
	rebuildGUIFromState(_authoringState, _worldState);
    }
}

void
MainWindow::
handleMoveUp() {
    moveQt4Constraint(true);
}

void
MainWindow::
handleMoveDown() {
    moveQt4Constraint(false);
}

void 
MainWindow::
updateJoint(int value) {
/*    std::string selectedJointName = getSelectedJointName();
    if (selectedJointName != "") {
	std::cout << "joint " << selectedJointName << " set to " <<  _jointSlider->value() / 100.0 << std::endl;
	_worldState.state_gfe.joint(selectedJointName).set_position( _jointSlider->value() / 100.0 );
	_worldState.colorRobot.set(_worldState.state_gfe);
	_widget_opengl.update();
    }
*/
}

/*
 * change the functionality of the slider to actuate currently selected link
 */
void 
MainWindow::
handleSelectedAffordanceChange() {
/*
    std::string selectedJointName = getSelectedJointName();
    _worldState.colorRobot.setSelectedJoint(selectedJointName);
    _widget_opengl.update();
    _jointNameLabel->setText(QString::fromStdString(selectedJointName));
*/
}

void 
MainWindow::
updateScrubber() {
    // update the scrubber
    double sum = 0;
    vector<double> lengths;
    _scrubber->clearTicks();
    for (std::vector<int>::size_type i = 0; i != _authoringState._all_gui_constraints.size(); i++) {
	double max = _authoringState._all_gui_constraints[i]->getConstraintMacro()->getTimeUpperBound();
	lengths.push_back(max);
	sum += max;
    }
    for (int i = 0; i < lengths.size(); i++) {
	_scrubber->addTick(lengths[i] / sum);
    }
}

void 
MainWindow::
setSelectedAction(Qt4ConstraintMacro* activator) { 
//    std::cout << "activated !!! " << std::endl;
    for (std::vector<int>::size_type i = 0; i != _authoringState._all_gui_constraints.size(); i++) {
	if (activator == _authoringState._all_gui_constraints[i].get()) {
	    _authoringState._all_gui_constraints[i]->setSelected(true);
	    _authoringState._selected_gui_constraint = _authoringState._all_gui_constraints[i];
	} else {
	    _authoringState._all_gui_constraints[i]->setSelected(false);
	}
    }
    updateScrubber();
/*
    handleRobotLinkChange();
*/
}

/* 
 * Get the toggle panels from the Qt4ConstraintMacro objects and populate the gui
 */
void
MainWindow::
rebuildGUIFromState(AuthoringState &state, WorldStateView &worldState) {
    // delete all of the current constraint's toggle panel widgets
    qDeleteAll(_constraint_container->findChildren<QWidget*>());

    for(std::vector<int>::size_type i = 0; i != state._all_gui_constraints.size(); i++) 
    {
	state._all_gui_constraints[i]->setModelObjects(worldState.affordances, worldState.manipulators);
	TogglePanel* tp = state._all_gui_constraints[i]->getPanel();
	connect(state._all_gui_constraints[i].get(),
		SIGNAL(activatedSignal(Qt4ConstraintMacro*)), 
		this, SLOT(setSelectedAction(Qt4ConstraintMacro*)));
	_constraint_vbox->addWidget(tp);
    }
    
}

//===========world state affordance updating
void MainWindow::affordanceUpdateCheck()
{
  int origSize = _worldState.affordances.size();
  _worldState.affServerWrapper.getAllAffordances(_worldState.affordances);
  if (_worldState.affordances.size() == origSize) //todo : use a better check than size (like "==" on each affordane if the sizes are equal )
    return;
  
  cout << "\n\n\n size of _worldState.affordances changed \n\n" << endl;
  handleAffordancesChanged(); 
}

/* select the OpenGL object corresponding to this affordance by highlighting it
 * in the GUI. Connected to the raycastCallbick signal from the OpenGL widget pane.
 */
void MainWindow::selectedOpenGLObjectChanged(const std::string &modelName) 
{
    std::cout << "intersected affordance: " << affordanceName << std::endl;

    for(uint i = 0; i < _worldState.glObjects.size(); i++)
    {
      if (_worldState.glObjects[i].id() == modelName)
	_worldState.glObjects[i].setHighlighted(true);

      /*
      //see if the next object is an affordance
      OpenGL_Object *nextObj = _worldState.glObjects[i];
      if (dynamic_cast<OpenGL_Affordance*>(nextObj) == NULL)
	continue;
      
      OpenGL_Affordance *glAff = (OpenGL_Affordance*) nextObj;
      glAff->setHighlighted(glAff->getAffordance()->getName() == affordanceName);
      }*/

    _widget_opengl.update();
    return;
}
