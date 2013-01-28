#include "MainWindow.h"
#include <affordance/OpenGL_Manipulator.h>

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

string RandomString(int len)
{
   string str = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
   int pos;
   while(str.size() != len) {
    pos = ((rand() % (str.size() - 1)));
    str.erase (pos, 1);
   }
   return str;
}

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

  // TODO resolve path
  _worldState.colorVehicle = new opengl::OpenGL_Object_DAE("vehicle", 
    "~/drc/software/models/mit_gazebo_models/" "mit_golf_cart/meshes/new_golf_cart.dae");
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

  AffConstPtr rfoot = nameToAffMap["Right Foot"];
  AffConstPtr lfoot = nameToAffMap["Left Foot"];
  AffConstPtr rhand = nameToAffMap["Right Hand"];
  AffConstPtr lhand = nameToAffMap["Left Hand"];

  AffConstPtr gas = nameToAffMap["Gas Pedal"];
  AffConstPtr brake = nameToAffMap["Brake Pedal"];
  AffConstPtr wheel = nameToAffMap["Steering Wheel"];

  ManipulatorStateConstPtr manip = _worldState.manipulators[0];
  RelationStatePtr relstate(new RelationState(RelationState::UNDEFINED));

  AtomicConstraintPtr rfoot_gas_relation (new ManipulationRelation(rfoot, manip, relstate));
  AtomicConstraintPtr lfoot_brake_relation(new ManipulationRelation(lfoot, manip, relstate));
  AtomicConstraintPtr rhand_wheel_relation(new ManipulationRelation(rhand, manip, relstate));
  AtomicConstraintPtr lhand_wheel_relation(new ManipulationRelation(lhand, manip, relstate));
  
  ConstraintMacroPtr rfoot_gas  (new ConstraintMacro("Right Foot to Gas Pedal", rfoot_gas_relation));
  ConstraintMacroPtr lfoot_brake (new ConstraintMacro("Left Foot to Brake Pedal", lfoot_brake_relation)); 
  ConstraintMacroPtr rhand_wheel (new ConstraintMacro("Right Hand To Wheel", rhand_wheel_relation));
  ConstraintMacroPtr lhand_wheel (new ConstraintMacro("Left Hand To Wheel", lhand_wheel_relation));
  
  _authoringState._all_gui_constraints.push_back((Qt4ConstraintMacroPtr)new Qt4ConstraintMacro(rfoot_gas, 0));
  _authoringState._all_gui_constraints.push_back((Qt4ConstraintMacroPtr)new Qt4ConstraintMacro(lfoot_brake, 1));
  _authoringState._all_gui_constraints.push_back((Qt4ConstraintMacroPtr)new Qt4ConstraintMacro(rhand_wheel, 2));
  _authoringState._all_gui_constraints.push_back((Qt4ConstraintMacroPtr)new Qt4ConstraintMacro(lhand_wheel, 3));

  rebuildGUIFromState(_authoringState, _worldState);
}

MainWindow::MainWindow(const shared_ptr<lcm::LCM> &theLcm, QWidget* parent)
	: _widget_opengl(),
	  _worldState(theLcm, "/mit_gazebo_models/mit_robot_drake/model_minimal_contact_ros.urdf"),
	  _constraint_container(new QWidget()),
	  _constraint_vbox(new QVBoxLayout())
{
    // setup the OpenGL scene
//  _worldState.state_gfe.from_urdf("/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf");
    _worldState.state_gfe.from_urdf("/mit_gazebo_models/mit_robot_drake/model_minimal_contact_ros.urdf");
    _worldState.colorRobot.set(_worldState.state_gfe);
  
    //setup timer to look for affordance changes
    QTimer *timer = new QTimer;
    connect(timer, SIGNAL(timeout()), 
	    this, SLOT(affordanceUpdateCheck()));
    timer->start(1000); //1Hz  

    // read the joints from the robot state
    // create the manipulators from the robot's joints

    std::vector<std::string> link_names;
    std::map< std::string, State_GFE_Joint > joints = _worldState.state_gfe.joints();
    for (std::map< std::string, State_GFE_Joint >::const_iterator it = joints.begin(); it != joints.end(); it++)
    {
    	const State_GFE_Joint& state_gfe_joint = it->second;
	std::string id = state_gfe_joint.id();
	shared_ptr<const urdf::Link> link = _worldState.colorRobot.getLinkFromJointName(id);
    	link_names.push_back(link->name);

	// TODO: constructor that works on link not on the link name
	ManipulatorStateConstPtr manipulator(new ManipulatorState(link, 
								  GlobalUID(rand(), rand()))); //todo guid
	_worldState.manipulators.push_back(manipulator);

	OpenGL_Manipulator *asGlMan = new OpenGL_Manipulator(manipulator); 
	_widget_opengl.opengl_scene().add_object(*asGlMan);
	_worldState.glObjects.push_back(asGlMan);
	_worldState.collisionObjs.push_back(new Collision_Object_Sphere("todo: fix this name",
									0.01,
									Vector3f(0,0,0),
									Vector4f(0,0,0,1)));
	// update the selected manipulator
	//	std::string manName = _authoringState._selected_gui_constraint->
	//	getConstraintMacro()->getAtomicConstraint()->getRelation()->getManipulator()->getName();
	
	//todo Mike start todo
	
	//todo: make a collision object
	/*	Collision_Object* collision_object_manipulator;
	collision_object_manipulator = new Collision_Object_Sphere(RandomString(10),
								   0.05, Vector3f(f.p.x(), f.p.y(), f.p.z()), Vector4f(0, 0, 0, 0));
	_widget_opengl.add_collision_object(collision_object_manipulator);
		_worldState.collisionObjs.push_back(collision_object_manipulator);
		// todo : end of move to manipulator state
		*/
    }
    
    this->setWindowTitle("Action Authoring Interface");

    QVBoxLayout* layout = new QVBoxLayout();
    layout->setMargin(0);

    QSplitter* splitter = new QSplitter();
    QWidget* leftside = new QWidget();
    QVBoxLayout* vbox = new QVBoxLayout();

    QToolBar *toolbar = new QToolBar("main toolbar");
    toolbar->addWidget(new QLabel("Action: "));
    toolbar->addWidget(new QLineEdit("Ingress"));
    QLabel* actionTypeLabel = new QLabel(" acts on: ");
    toolbar->addWidget(actionTypeLabel);
    QComboBox* actionType = new QComboBox();
    actionType->insertItem(0, "Vehicle");
    actionType->insertItem(0, "Door");
    actionType->insertItem(0, "Ladder");
    actionType->insertItem(0, "Table");
    toolbar->addWidget(actionType);
    toolbar->addSeparator();
    QPushButton* savebutton = new QPushButton("Save Action");
    toolbar->addWidget(savebutton);
    QPushButton* loaddiff = new QPushButton("Load Action");
    toolbar->addWidget(loaddiff);



    vbox->addWidget(toolbar);

//    _constraint_vbox->setSizeConstraint(QLayout::SetMinAndMaxSize);
//    _constraint_vbox->setMargin(0);
    _constraint_vbox->setAlignment(Qt::AlignTop);
//_constraint_container->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );
    _constraint_container->setLayout(_constraint_vbox);
    QScrollArea *area = new QScrollArea();
//    area->setBackgroundRole(QPalette::Dark);
    area->setWidgetResizable(true);
    area->setWidget(_constraint_container);
    area->setMinimumSize( QSize(900, 700 ) );
    area->setAlignment(Qt::AlignTop);
//    vbox->addWidget(_constraint_container);
    vbox->addWidget(area);


    QToolBar *constraint_toolbar = new QToolBar("main toolbar");
    QPushButton* deletebutton = new QPushButton("delete");
    _moveUpButton = new QPushButton("move up");
    _moveDownButton = new QPushButton("move down");
    QPushButton* addconstraintbutton = new QPushButton("+ add constraint");
    constraint_toolbar->addWidget(deletebutton);
    constraint_toolbar->addSeparator();
    constraint_toolbar->addWidget(_moveUpButton);
    constraint_toolbar->addWidget(_moveDownButton);
    constraint_toolbar->addSeparator();
    constraint_toolbar->addWidget(addconstraintbutton);
    vbox->addStretch(1);
    vbox->addWidget(constraint_toolbar);


    QGroupBox* mediaControls = new QGroupBox();
    QHBoxLayout* mediaControlsLayout = new QHBoxLayout();
    _fbwd = new QPushButton();
    _bwd = new QPushButton();
    _play = new QPushButton();
    _fwd = new QPushButton();
    _ffwd = new QPushButton();

    // see http://www.qtcentre.org/wiki/index.php?title=Embedded_resources
    QPixmap pixmap1(":/trolltech/styles/commonstyle/images/media-skip-backward-32.png");
    _fbwd->setIcon(QIcon(pixmap1));
    _fbwd->setIconSize(pixmap1.rect().size());

    QPixmap pixmap2(":/trolltech/styles/commonstyle/images/media-seek-backward-32.png");
    _bwd->setIcon(QIcon(pixmap2));
    _bwd->setIconSize(pixmap2.rect().size());

    QPixmap pixmap3(":/trolltech/styles/commonstyle/images/media-play-32.png");
    _play->setIcon(QIcon(pixmap3));
    _play->setIconSize(pixmap3.rect().size());
    _isPlaying = true;

    QPixmap pixmap4(":/trolltech/styles/commonstyle/images/media-seek-forward-32.png");
    _fwd->setIcon(QIcon(pixmap4));
    _fwd->setIconSize(pixmap4.rect().size());

    QPixmap pixmap5(":/trolltech/styles/commonstyle/images/media-skip-forward-32.png");
    _ffwd->setIcon(QIcon(pixmap5));
    _ffwd->setIconSize(pixmap5.rect().size());

    mediaControlsLayout->addSpacerItem(new QSpacerItem(100, 0));
    mediaControlsLayout->addWidget(_fbwd);
    mediaControlsLayout->addWidget(_bwd);
    mediaControlsLayout->addWidget(_play);
    mediaControlsLayout->addWidget(_fwd);
    mediaControlsLayout->addWidget(_ffwd);
    mediaControlsLayout->addSpacerItem(new QSpacerItem(100, 0));
    mediaControls->setLayout(mediaControlsLayout);
    _play->resize(_play->width() * 2, _play->height());

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
    vbox->addStretch(1);
    leftside->setLayout(vbox);

    // fix the borders on the group boxes
    this->setStyleSheet("QGroupBox { border: 1px solid gray; border-radius: 3px; padding: 5px; } "
			"QGroupBox::title { background-color: transparent; "
			"subcontrol-position: top left; /* position at the top left*/ "
			"padding:2 5px; } ");

    this->setLayout(layout);

    // wire up the buttons
    connect(_play, SIGNAL(released()), this, SLOT(mediaPlay()));
    connect(_ffwd, SIGNAL(released()), this, SLOT(mediaFastForward()));
    connect(_fbwd, SIGNAL(released()), this, SLOT(mediaFastBackward()));

    connect(deletebutton, SIGNAL(released()), this, SLOT(handleDeleteConstraint()));
    connect(_moveUpButton, SIGNAL(released()), this, SLOT(handleMoveUp()));
    connect(_moveDownButton, SIGNAL(released()), this, SLOT(handleMoveDown()));
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

  std::vector<ConstraintMacroPtr> revivedConstraintMacros;
  std::vector<AffConstPtr> revivedAffordances;

  DatabaseManager::retrieve(fileName.toStdString(), revivedAffordances, revivedConstraintMacros);

  _worldState.affordances.clear();

  for (int i = 0; i < revivedAffordances.size(); i++ ) {
    _worldState.affordances.push_back(revivedAffordances[i]);
  }

  _authoringState._all_gui_constraints.clear();

  for (int i = 0; i < revivedConstraintMacros.size(); i++) 
  {
      _authoringState._all_gui_constraints.push_back((Qt4ConstraintMacroPtr)new Qt4ConstraintMacro(revivedConstraintMacros[i], i)); 
  }

  rebuildGUIFromState(_authoringState, _worldState);

}

void
MainWindow::
handleSaveAction() {
 QString fileName = QFileDialog::getSaveFileName(this,
     tr("Save Action"), "", tr("Action XML Files (*.xml)"));  

 if (fileName.toStdString() == "") 
     return;

 vector<ConstraintMacroPtr> all_constraints;
 for (int i = 0; i < _authoringState._all_gui_constraints.size(); i++) {
     all_constraints.push_back(_authoringState._all_gui_constraints[i]->getConstraintMacro());
 }

 DatabaseManager::store(fileName.toStdString(), _worldState.affordances, all_constraints);

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
    if (i >= 0) {
	_authoringState._all_gui_constraints.erase(_authoringState._all_gui_constraints.begin() + i);
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
    AtomicConstraintPtr new_constraint(new ManipulationRelation(left, manip, relstate));

    ConstraintMacroPtr rfoot_gas  (new ConstraintMacro("Untitled" + RandomString(5), new_constraint));

    // compute the number
    int index = _authoringState._all_gui_constraints.size();
    _authoringState._all_gui_constraints.push_back((Qt4ConstraintMacroPtr)new Qt4ConstraintMacro(rfoot_gas, index));
    rebuildGUIFromState(_authoringState, _worldState);
}

void
MainWindow::
moveQt4Constraint(bool up) {
    int i = getSelectedGUIConstraintIndex();
    std::vector<Qt4ConstraintMacroPtr> &constraints = _authoringState._all_gui_constraints;
    if (up && i > 0 || (! up) && i < constraints.size() - 1) {
	int j = i + (up ? -1 : 1);
	std::cout << "swapping " << i << " with " << j << std::endl;
	//std::swap(constraints[i], constraints[i + (up ? i - 1 : i + 1)]);
	//rebuildGUIFromState(_authoringState, _worldState);
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
    double sum = 0.0;
    vector<double> lengths;
    _scrubber->clearTicks();
    for (std::vector<int>::size_type i = 0; i != _authoringState._all_gui_constraints.size(); i++) {
	double max = _authoringState._all_gui_constraints[i]->getConstraintMacro()->getTimeUpperBound();
	lengths.push_back(sum + max);
	sum += max;
    }
    for (int i = 0; i < lengths.size(); i++) {
	_scrubber->addTick(lengths[i] / sum);
    }
    _scrubber->setSelectedRangeIndex(getSelectedGUIConstraintIndex());
    _scrubber->update();
}

void 
MainWindow::
setSelectedAction(Qt4ConstraintMacro* activator) { 
//    std::cout << "activated !!! " << std::endl;
    int selected_index = -1;
    for (std::vector<int>::size_type i = 0; i != _authoringState._all_gui_constraints.size(); i++) {
	if (activator == _authoringState._all_gui_constraints[i].get()) {
	    _authoringState._all_gui_constraints[i]->setSelected(true);
	    _authoringState._selected_gui_constraint = _authoringState._all_gui_constraints[i];
	    selected_index = i;
	} else {
	    _authoringState._all_gui_constraints[i]->setSelected(false);
	}
    }
    _moveUpButton->setEnabled(selected_index != 0);
    _moveDownButton->setEnabled(selected_index != _authoringState._all_gui_constraints.size() - 1);
    _fbwd->setEnabled(selected_index != 0);
    _ffwd->setEnabled(selected_index != _authoringState._all_gui_constraints.size() - 1);

    updateScrubber();
}

/* 
 * Get the toggle panels from the Qt4ConstraintMacro objects and populate the gui
 */
void
MainWindow::
rebuildGUIFromState(AuthoringState &state, WorldStateView &worldState) {
    // delete all of the current constraint's toggle panel widgets
//    std::cout << "deleting rendered children " << std::endl;
//    qDeleteAll(_constraint_container->findChildren<QWidget*>());
    std::cout << "starting loop " << std::endl;

    for(std::vector<int>::size_type i = 0; i != state._all_gui_constraints.size(); i++) 
    {
	state._all_gui_constraints[i]->setModelObjects(worldState.affordances, worldState.manipulators);
	if (! state._all_gui_constraints[i]->isInitialized()) { // have we already added this tp?
	    TogglePanel* tp = state._all_gui_constraints[i]->getPanel();
	    // TODO: the connect() call here appears to be quadratic in the
	    // number of previously connected signals
	    connect(state._all_gui_constraints[i].get(),
		    SIGNAL(activatedSignal(Qt4ConstraintMacro*)), 
		    this, SLOT(setSelectedAction(Qt4ConstraintMacro*)));
//		    Qt::UniqueConnection);

	    std::cout << "adding constraint #" << i << std::endl;
	    _constraint_vbox->addWidget(tp);
	}
    }
}

/*
 * world state affordance updating
 */
void
MainWindow::
affordanceUpdateCheck()
{
  int origSize = _worldState.affordances.size();
  _worldState.affServerWrapper.getAllAffordances(_worldState.affordances);
  if (_worldState.affordances.size() == origSize) //todo : use a better check than size (like "==" on each affordane if the sizes are equal )
    return;
  
  cout << "\n\n\n size of _worldState.affordances changed \n\n" << endl;
  handleAffordancesChanged(); 
}

/*
 * Select the OpenGL object corresponding to this affordance by highlighting it
 * in the GUI. Connected to the raycastCallbick signal from the OpenGL widget pane.
 */
void
MainWindow::
selectedOpenGLObjectChanged(const std::string &modelName) 
{
    std::cout << "intersected affordance: " << modelName << std::endl;

    for(uint i = 0; i < _worldState.glObjects.size(); i++)
    {
	if (_worldState.glObjects[i]->id() == modelName)
	    _worldState.glObjects[i]->setHighlighted(true);
	else 
	    _worldState.glObjects[i]->setHighlighted(false);
    }
    _widget_opengl.update();
    return;
}


void
MainWindow::
mediaFastForward() {
    int i = getSelectedGUIConstraintIndex();
    _authoringState._all_gui_constraints[i + 1]->setActiveExternal();
}

void
MainWindow::
mediaFastBackward() {
    int i = getSelectedGUIConstraintIndex();
    _authoringState._all_gui_constraints[i - 1]->setActiveExternal();
}

void
MainWindow::mediaPlay() {
    if (_isPlaying) {
	QPixmap pixmap3(":/trolltech/styles/commonstyle/images/media-pause-32.png");
	_play->setIcon(QIcon(pixmap3));
	_play->setIconSize(pixmap3.rect().size());
    } else {
	QPixmap pixmap3(":/trolltech/styles/commonstyle/images/media-play-32.png");
	_play->setIcon(QIcon(pixmap3));
	_play->setIconSize(pixmap3.rect().size());
    }
    _isPlaying = ! _isPlaying;
}
