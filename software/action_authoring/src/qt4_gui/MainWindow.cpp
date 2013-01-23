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
	  OpenGL_Affordance *asGlAff = new OpenGL_Affordance(*next); 
	  _widget_opengl.opengl_scene().add_object(*asGlAff);
	  _worldState.glObjects.push_back(asGlAff);

	  // Create CollisionObject_Affordances, add to scene, and add to _worldState.glObjects
	  Collision_Object* collision_object_affordance;
	  KDL::Frame f;
	  next->getFrame(f);
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

  _widget_opengl.opengl_scene().add_object(_worldState.colorRobot); //add robot

  connect(&_widget_opengl, SIGNAL(raycastCallback(std::string)),
	  this, SLOT(selectedOpenGLObjectChanged(std::string)));
//  _widget_opengl.set_raycast_callback((void (*)(std::string))(&MainWindow::selectedOpenGLObjectChanged));

//  _worldState.colorVehicle = new opengl::OpenGL_Object_DAE("vehicle", "drc/software/models/mit_gazebo_models/" "mit_golf_cart/meshes/no_wheels.dae"); //mit_golf_cart/meshes/model.dae");
//  _worldState.colorVehicle = new OpenGL_Object_DAE( "object-object-dae", 
//    "/usr/local/share/drcsim-1.3/models/golf_cart/meshes/no_wheels.dae");
//	"/home/drc/drc/software/models/mit_gazebo_models/mit_wheeled_robot/meshes/head.dae"); //mit_robot/meshes/utorso.dae"); //mit_golf_cart/meshes/no_wheels.dae");
//  _widget_opengl.opengl_scene().add_object(*_worldState.colorVehicle); //add vehicle

  _widget_opengl.update();
  //_widget_opengl.add_object_with_collision(_collision_object_gfe);

  //=========collision objects

  /*todo

  // build collision objects
	/*todo  construct collision objects from the affordances
	_collision_object_box = new Collision_Object_Box("box1", Vector3f(0.25, 0.25, 0.25), Vector3f(1.0, 0.0, 0.0), Vector4f(1.0, 0.0, 0.0, 0.0));
    _collision_object_cylinder = new Collision_Object_Cylinder("cylinder1", 0.25, 0.25, Vector3f(0.0, 1.0, 0.0), Vector4f(1.0, 0.0, 0.0, 0.0));
    _collision_object_sphere = new Collision_Object_Sphere("sphere1", 0.125, Vector3f(-0.5, -0.5, 0.0), Vector4f(1.0, 0.0, 0.0, 0.0));
    //_collision_object_gfe("robot1");
*/


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

  makeGUIFromConstraintMacros();
}

MainWindow::MainWindow(const shared_ptr<lcm::LCM> &theLcm, QWidget* parent)
	: _widget_opengl(),
	  _worldState(theLcm)
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
    std::vector<std::string> joint_names;
    std::map< std::string, State_GFE_Joint > joints = _worldState.state_gfe.joints();
    for (std::map< std::string, State_GFE_Joint >::const_iterator it = joints.begin(); it != joints.end(); it++)
    {
    	const State_GFE_Joint& state_gfe_joint = it->second;
    	joint_names.push_back(state_gfe_joint.id());
    }
  
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

void MainWindow::handleLoadAction() 
{
  QString fileName = QFileDialog::getOpenFileName(this,
						  tr("Open Action"), "", tr("Action XML Files (*.xml)"));
  
  DatabaseManager* dbm = new DatabaseManager(fileName.toStdString()); //todo : Memory leak
  dbm->parseFile();
  
  std::vector<ConstraintMacroPtr> revivedConstraintMacros;
  dbm->getConstraintMacros(revivedConstraintMacros);
  
  //  for (int i = 0; i < _authoringState._all_gui_constraints.size(); i++) {
  //      delete _authoringState._all_gui_constraints[i].get();
  //  }
  
  _authoringState._all_gui_constraints.clear();
  for (int i = 0; i < revivedConstraintMacros.size(); i++) 
    {
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
handleSelectedAffordanceChange() {
    std::string selectedJointName = getSelectedJointName();
    _worldState.colorRobot.setSelectedJoint(selectedJointName);
    _widget_opengl.update();
    _jointNameLabel->setText(QString::fromStdString(selectedJointName));
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
/*
    handleRobotLinkChange();
*/
}

void
MainWindow::
makeGUIFromConstraintMacros() {
    std::cout << "making GUI..." << std::endl;
    // Get the toggle panels from the Qt4ConstraintMacro objects and populate the gui
    for(std::vector<int>::size_type i = 0; i != _authoringState._all_gui_constraints.size(); i++) 
      {
	_authoringState._all_gui_constraints[i]->setAffordances(_worldState.affordances, _worldState.affordances);
	TogglePanel* tp = _authoringState._all_gui_constraints[i]->getPanel();
	connect(_authoringState._all_gui_constraints[i].get(),
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

void
MainWindow::
selectedOpenGLObjectChanged(std::string affordanceName) {
    std::cout << "intersected affordance: " << affordanceName << std::endl;
    for(uint i = 0; i < _worldState.glObjects.size(); i++)
    {
	AffordanceState& selectedAff = ((OpenGL_Affordance*)_worldState.glObjects[i])->_affordance;
//	std::cout << "affordances" << selectedAff.getName() << (selectedAff.getName().compare(affordanceName) != 0) << std::endl;
	// select the openGL object corresponding to this affordance by highlighting it
	((OpenGL_Affordance*)_worldState.glObjects[i])->setHighlighted(selectedAff.getName().compare(affordanceName) == 0);
    }
    _widget_opengl.update();
    return;
}
