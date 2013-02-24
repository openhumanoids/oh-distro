#include "MainWindow.h"
#include <affordance/OpenGL_Manipulator.h>
#include <affordance/Collision_Object_Affordance.h>
#include <affordance/Collision_Object_Manipulator.h>

using namespace std;
using namespace opengl;
using namespace state;
using namespace collision;
using namespace action_authoring;
using namespace affordance;

#define PLAN_ACTION_MESSAGE_CHANNEL "action_authoring_plan_action_request"


//using namespace collision_detection;

string RandomString(int len)
{
    string str = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    int pos;

    while ((int)str.size() != len)
    {
        pos = ((rand() % (str.size() - 1)));
        str.erase(pos, 1);
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
    {
        throw InvalidStateException("glObjects and collisionObjs should have the same size");
    }

    std::cout << " flag1" << std::endl;

    _widget_opengl.opengl_scene().clear_objects();

    for (uint i = 0; i < _worldState.glObjects.size(); i++)
    {
        delete _worldState.glObjects[i];
        delete _worldState.collisionObjs[i];
    }

    std::cout << " flag2" << std::endl;

    _worldState.glObjects.clear();
    _worldState.collisionObjs.clear();
    std::cout << " flag2d" << std::endl;
    for (uint i = 0; i < _worldState.affordances.size(); i++)
    {
        AffConstPtr next = _worldState.affordances[i];
    std::cout << " flag2c" << std::endl;
        if (!Collision_Object_Affordance::isSupported(next))
        {
            cout << "\n Collision_Object_Affordance doesn't support " << next->getName() << endl;
            continue;
        }

        if (!OpenGL_Affordance::isSupported(next))
        {
            cout << "\n OpenGL_Affordance doesn't support " << next->getName() << endl;
            continue;
        }
    std::cout << " flag2b" << std::endl;
    std::cout << " next " << next->getName() << std::endl;
        //opengl
        OpenGL_Affordance *asGlAff = new OpenGL_Affordance(next);
    std::cout << " flag2b-1" << std::endl;
        _widget_opengl.opengl_scene().add_object(*asGlAff);
    std::cout << " flag2b-2" << std::endl;
        _worldState.glObjects.push_back(asGlAff);
    std::cout << " flag2a" << std::endl;
        //collisions:  Create CollisionObject_Affordances, add to scene, and add to _worldState.glObjects
        Collision_Object *collision_object_affordance = new Collision_Object_Affordance(next);
        _widget_opengl.add_collision_object(collision_object_affordance);
        _worldState.collisionObjs.push_back(collision_object_affordance);
    }

    std::cout << " flag3" << std::endl;

    //----------handle constraint macros
    unordered_map<string, AffConstPtr> nameToAffMap;

    for (vector<AffConstPtr>::iterator iter = _worldState.affordances.begin();
            iter != _worldState.affordances.end();
            ++iter)
    {
        nameToAffMap[(*iter)->getName()] = *iter;
    }

    std::cout << " flag4" << std::endl;

    createManipulators();
    updateFlyingManipulators();

    _widget_opengl.opengl_scene().add_object(point_contact_axis);
    //_worldState.glObjects.push_back(&point_contact_axis);
    _widget_opengl.opengl_scene().add_object(point_contact_axis2);
    //_worldState.glObjects.push_back(&point_contact_axis2);

    //----------add robot and vehicle
    _widget_opengl.opengl_scene().add_object(_worldState.colorRobot); //add robot

        std::cout << " flag5" << std::endl;

    //  connect(&_widget_opengl, SIGNAL(raycastPointIntersectCallback(Eigen::Vector3f)),
    //	  this, SLOT(mainRaycastCallback(Eigen::Vector3f)));

    // TODO resolve path
    //_worldState.colorVehicle = new opengl::OpenGL_Object_DAE("vehicle",
    //"/home/drc/drc/software/models/mit_gazebo_models/" "mit_golf_cart/meshes/new_golf_cart.dae");
    //_widget_opengl.opengl_scene().add_object(*_worldState.colorVehicle); //add vehicle

    _widget_opengl.update();
    //_widget_opengl.add_object_with_collision(_collision_object_gfe);

}

MainWindow::MainWindow(const shared_ptr<lcm::LCM> &theLcm, QWidget *parent)
    : _widget_opengl(),
      _constraint_container(new QWidget()),
      _constraint_vbox(new QVBoxLayout()),
      _worldState(theLcm, "/mit_gazebo_models/mit_robot_drake/model_minimal_contact_ros.urdf")

{
    _theLcm = theLcm;
	_theLcm->subscribe("ACTION_AUTHORING_IK_ROBOT_STATE",
                       &MainWindow::updateRobotState, this);

    //point_contact_axis = new OpenGL_Object_Coordinate_Axis();
    //point_contact_axis2 = new OpenGL_Object_Coordinate_Axis();

    // setup the OpenGL scene
    _worldState.state_gfe.from_urdf("/mit_gazebo_models/mit_robot_drake/model_minimal_contact_ros.urdf");
    _worldState.colorRobot.set(_worldState.state_gfe);

    _widget_opengl.setMinimumHeight(100);
    _widget_opengl.setMinimumWidth(500);
    _widget_opengl.setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    connect(&_widget_opengl, SIGNAL(raycastCallback(std::string, Eigen::Vector3f)),
            this, SLOT(selectedOpenGLObjectChanged(std::string, Eigen::Vector3f)));

    //setup timer to look for affordance changes
    QTimer *timer = new QTimer;
    connect(timer, SIGNAL(timeout()),
            this, SLOT(affordanceUpdateCheck()));
    timer->start(1000); //1Hz

    _scrubberTimer = new QTimer;
    connect(_scrubberTimer, SIGNAL(timeout()), this, SLOT(nextKeyFrame()));

    this->setWindowTitle("Action Authoring Interface");

    QVBoxLayout *layout = new QVBoxLayout();
    layout->setMargin(0);

    QSplitter *splitter = new QSplitter();
    QWidget *leftside = new QWidget();
    QVBoxLayout *vbox = new QVBoxLayout();

    QToolBar *toolbar = new QToolBar("main toolbar");
    toolbar->addWidget(new QLabel("Action: "));
    _actionName = new QLineEdit("Ingress");
    toolbar->addWidget(_actionName);
    QLabel *actionTypeLabel = new QLabel(" acts on: ");
    toolbar->addWidget(actionTypeLabel);
    QComboBox *actionType = new QComboBox();
    actionType->insertItem(0, "Vehicle");
    actionType->insertItem(0, "Door");
    actionType->insertItem(0, "Ladder");
    actionType->insertItem(0, "Table");
    toolbar->addWidget(actionType);
    toolbar->addSeparator();
    QPushButton *savebutton = new QPushButton("Save Action");
    savebutton->setIcon(QApplication::style()->standardIcon(QStyle::SP_DialogSaveButton));
    toolbar->addWidget(savebutton);
    QPushButton *loaddiff = new QPushButton("Load Action");
    loaddiff->setIcon(QApplication::style()->standardIcon(QStyle::SP_DialogOpenButton));
    toolbar->addWidget(loaddiff);
    QPushButton *planbutton = new QPushButton("Publish For Planning");
    planbutton->setIcon(QApplication::style()->standardIcon(QStyle::SP_DriveNetIcon));
    toolbar->addWidget(planbutton);


    vbox->addWidget(toolbar);

    //    _constraint_vbox->setSizeConstraint(QLayout::SetMinAndMaxSize);
    //    _constraint_vbox->setMargin(0);
    _constraint_vbox->setAlignment(Qt::AlignTop);
    _constraint_container->setLayout(_constraint_vbox);
    QScrollArea *area = new QScrollArea();
    //    area->setBackgroundRole(QPalette::Dark);
    area->setWidgetResizable(true);
    area->setWidget(_constraint_container);
    area->setMinimumSize(QSize(725, 600));
    area->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    area->setAlignment(Qt::AlignTop);
    //    vbox->addWidget(_constraint_container);
    vbox->addWidget(area);
    vbox->setStretchFactor(area, 1000);

    QToolBar *constraint_toolbar = new QToolBar("main toolbar");
    QPushButton *deletebutton = new QPushButton("delete");
    deletebutton->setIcon(QApplication::style()->standardIcon(QStyle::SP_TrashIcon));
    _moveUpButton = new QPushButton("move up");
    //_moveUpButton->setIcon(QApplication::style()->standardIcon(QStyle::SP_ArrowUp));
    _moveDownButton = new QPushButton("move down");
    //_moveDownButton->setIcon(QApplication::style()->standardIcon(QStyle::SP_ArrowDown));
    QPushButton *addconstraintbutton = new QPushButton("+ add constraint");
    //    addconstraintbutton->setIcon(QApplication::style()->standardIcon(QStyle::SP_FileDialogNewFolder));
    constraint_toolbar->addWidget(deletebutton);
    constraint_toolbar->addSeparator();
    constraint_toolbar->addWidget(_moveUpButton);
    constraint_toolbar->addWidget(_moveDownButton);
    constraint_toolbar->addSeparator();
    constraint_toolbar->addWidget(addconstraintbutton);
    constraint_toolbar->addSeparator();
    vbox->addStretch(1);
    vbox->addWidget(constraint_toolbar);

    QGroupBox *mediaControls = new QGroupBox();
    QHBoxLayout *mediaControlsLayout = new QHBoxLayout();
    _fbwd = new QPushButton();
    _bwd = new QPushButton();
    _play = new QPushButton();
    _fwd = new QPushButton();
    _ffwd = new QPushButton();

    // see http://www.qtcentre.org/wiki/index.php?title=Embedded_resources
    QPixmap pixmap1(":/trolltech/styles/commonstyle/images/media-skip-backward-32.png");
    _fbwd->setEnabled(false);
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
    _ffwd->setEnabled(false);
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

    _segmentedButton = new QtSegmentControl();
    _segmentedButton->setCount(2);
    _segmentedButton->setSegmentText(0, tr("Authoring"));
    _segmentedButton->setSegmentText(1, tr("Live"));
    _segmentedButton->setSelectionBehavior(QtSegmentControl::SelectOne);

    QWidget *_liveWidgets = new QWidget(this);
    //    QHBoxLayout* metaActionLayout = new QHBoxLayout();
    //    QPushButton* addAffordanceButton = new QPushButton("Copy Selected Affordance for Authoring");
    //    metaActionLayout->addWidget(addAffordanceButton);
    //    QPushButton* bindButton = new QPushButton("Bind To...");
    //    metaActionLayout->addWidget(bindButton);
    //    _liveWidgets->setLayout(metaActionLayout);

    QWidget *_authoringWidgets = new QWidget(this);
    //    QHBoxLayout* authoringWidgetsLayout = new QHBoxLayout();
    //    QPushButton* something = new QPushButton("authoring view button");
    //    metaActionLayout->addWidget(something);
    //    _authoringWidgets->setLayout(authoringWidgetsLayout);

    QGroupBox *widgetWrapper = new QGroupBox();
    widgetWrapper->setStyleSheet("QGroupBox { border: 1px solid gray; border-radius: 0px; padding: 0px; margin: 0px; background-color: black; }");
    QVBoxLayout *widgetWrapperLayout = new QVBoxLayout();
    widgetWrapperLayout->setSpacing(0);
    widgetWrapperLayout->setMargin(0);
    widgetWrapperLayout->addWidget(&_widget_opengl);
    widgetWrapperLayout->addWidget(mediaControls);
    widgetWrapper->setLayout(widgetWrapperLayout);

    QVBoxLayout *rightsidelayout = new QVBoxLayout();
    QWidget *rightside = new QWidget();
    _actionDescLabel = new QLabel();
    _actionDescLabel->setTextFormat(Qt::RichText);
    _scrubber = new DefaultValueSlider(Qt::Horizontal, this);
    _scrubber->setRange(0, 1000);
    rightsidelayout->addWidget(_actionDescLabel);
    //    rightsidelayout->addWidget(_jointSlider);
    rightsidelayout->addWidget(_segmentedButton);
    rightsidelayout->addWidget(_liveWidgets);
    rightsidelayout->addWidget(_authoringWidgets);
    rightsidelayout->addWidget(widgetWrapper);
    rightsidelayout->addWidget(_scrubber);
    rightside->setLayout(rightsidelayout);

    splitter->addWidget(leftside);
    splitter->addWidget(rightside);
    splitter->setStretchFactor(1, 1000);

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
    connect(planbutton, SIGNAL(released()), this, SLOT(publishActionToLCM()));
    connect(_segmentedButton, SIGNAL(segmentSelected(int)), this, SLOT(changeMode()));

    connect(_play, SIGNAL(released()), this, SLOT(mediaPlay()));
    connect(_ffwd, SIGNAL(released()), this, SLOT(mediaFastForward()));
    connect(_fbwd, SIGNAL(released()), this, SLOT(mediaFastBackward()));

    connect(deletebutton, SIGNAL(released()), this, SLOT(handleDeleteConstraint()));
    connect(_moveUpButton, SIGNAL(released()), this, SLOT(handleMoveUp()));
    connect(_moveDownButton, SIGNAL(released()), this, SLOT(handleMoveDown()));
    connect(addconstraintbutton, SIGNAL(released()), this, SLOT(handleAddConstraint()));

    connect(savebutton, SIGNAL(released()), this, SLOT(handleSaveAction()));
    connect(loaddiff, SIGNAL(released()), this, SLOT(handleLoadAction()));
}

MainWindow::~MainWindow()
{

}

void MainWindow::handleLoadAction()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                       tr("Open Action"), "", tr("Action XML Files (*.xml)"));

    if (fileName.toStdString() == "")
    {
        return;
    }

    std::vector<ConstraintMacroPtr> revivedConstraintMacros;
    std::vector<AffConstPtr> revivedAffordances;

#ifdef DATABASE
    DatabaseManager::retrieve(fileName.toStdString(), revivedAffordances, revivedConstraintMacros);

    printf("done retrieving.\n");
    _worldState.affordances.clear();


    printf("done clearing world state affordances.\n");

    for (int i = 0; i < (int)revivedAffordances.size(); i++)
    {
        _worldState.affordances.push_back(revivedAffordances[i]);
    }

    _authoringState._all_gui_constraints.clear();
    printf("done clearing authoring state gui constraints.\n");

    for (int i = 0; i < (int)revivedConstraintMacros.size(); i++)
    {
        printf("making Qt4ConstraintMacroPtr %i\n", i);
        _authoringState._all_gui_constraints.push_back((Qt4ConstraintMacroPtr)new Qt4ConstraintMacro(revivedConstraintMacros[i], i));
    }

    printf("Now calling rebuild gui from state.");
    rebuildGUIFromState(_authoringState, _worldState);

#endif
}

void
MainWindow::
handleSaveAction()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                       tr("Save Action"), _actionName->text() + ".xml", tr("Action XML Files (*.xml)"));

    if (fileName.toStdString() == "")
    {
        return;
    }

#ifdef DATABASE
    vector<ConstraintMacroPtr> all_constraints;

    for (int i = 0; i < (int)_authoringState._all_gui_constraints.size(); i++)
    {
        all_constraints.push_back(_authoringState._all_gui_constraints[i]->getConstraintMacro());
    }

    // todo: hack
    for (uint i = 0; i < _authoringState._all_gui_constraints.size(); i++)
    {
        cout << "gui constraint i " << all_constraints[i]->getAtomicConstraint()->getAffordance().get() << endl;

        for (uint j = 0; j < _worldState.affordances.size(); j++)
        {
            cout << "......vs affordance j " << _worldState.affordances[j].get() << endl;

            if (_worldState.affordances[j] == all_constraints[i]->getAtomicConstraint()->getAffordance())
            {
                //	 if (_worldState.affordances[j]->getGUIDAsString() == all_constraints[i]->getAtomicConstraint()->getAffordance()->getGUIDAsString()) {
                cout << "matched i, j " << i << ", " << j << endl;
                all_constraints[i]->getAtomicConstraint()->setAffordance(_worldState.affordances[j]);
            }
        }
    }

    /* for (uint j = 0; j < _worldState.affordances.size(); j++) {
         cout << "......vs affordance j " << j << " : " << (*_worldState.affordances[j] << endl;
         //if (_worldState.affordances[j] == all_constraints[i]->getAtomicConstraint()->getAffordance()) {
     }
    */

    DatabaseManager::store(fileName.toStdString(), _worldState.affordances, all_constraints);


#endif //DATABASE
}


int
MainWindow::
getSelectedGUIConstraintIndex()
{
    if (_authoringState._selected_gui_constraint == NULL)
    {
        return -1;
    }

    std::vector<Qt4ConstraintMacroPtr> &constraints = _authoringState._all_gui_constraints;
    std::vector<Qt4ConstraintMacroPtr>::iterator it = std::find(
                constraints.begin(), constraints.end(),
                _authoringState._selected_gui_constraint);

    if (it != constraints.end())
    {
        int i = it - constraints.begin();
        return i;
    }
    else
    {
        return -1;
    }
}

void
MainWindow::
handleDeleteConstraint()
{
    int i = getSelectedGUIConstraintIndex();

    if (i >= 0)
    {
        _authoringState._all_gui_constraints.erase(_authoringState._all_gui_constraints.begin() + i);
        rebuildGUIFromState(_authoringState, _worldState);
    }
}

void
MainWindow::
handleAddConstraint()
{
    if (_worldState.affordances.size() == 0 || _worldState.manipulators.size() == 0)
    {
        QMessageBox msgBox;
        msgBox.setText("Need at least one affordance and manipulator to add a constraint");
        msgBox.exec();
        return;
    }

    AffConstPtr left = _worldState.affordances[0];
    ManipulatorStateConstPtr manip = _worldState.manipulators[0];

    PointContactRelationPtr relstate(new PointContactRelation());
    AtomicConstraintPtr new_atomic(new ManipulationRelation(left, manip, relstate));

    ConstraintMacroPtr new_constraint(new ConstraintMacro("Untitled" + RandomString(5), new_atomic));

    // compute the number
    int index = _authoringState._all_gui_constraints.size();
    _authoringState._all_gui_constraints.push_back((Qt4ConstraintMacroPtr)new Qt4ConstraintMacro(new_constraint, index));
    rebuildGUIFromState(_authoringState, _worldState);
}

void
MainWindow::
moveQt4Constraint(bool up)
{
    int i = getSelectedGUIConstraintIndex();
    std::vector<Qt4ConstraintMacroPtr> &constraints = _authoringState._all_gui_constraints;

    if ((up && i > 0) || ((! up) && (i < (int)constraints.size() - 1)))
    {
        int j = i + (up ? -1 : 1);
        std::cout << "swapping " << i << " with " << j << std::endl;
        //std::swap(constraints[i], constraints[i + (up ? i - 1 : i + 1)]);
        //rebuildGUIFromState(_authoringState, _worldState);
    }
}

void
MainWindow::
handleMoveUp()
{
    moveQt4Constraint(true);
}

void
MainWindow::
handleMoveDown()
{
    moveQt4Constraint(false);
}

void
MainWindow::
updateScrubber()
{
    // update the scrubber
    _timeSum = 0.0;
    vector<double> lengths;
    _scrubber->clearTicks();

    for (std::vector<int>::size_type i = 0; i != _authoringState._all_gui_constraints.size(); i++)
    {
        double max = _authoringState._all_gui_constraints[i]->getConstraintMacro()->getTimeUpperBound();
        lengths.push_back(_timeSum + max);
        _timeSum += max;
    }

    for (int i = 0; i < (int)lengths.size(); i++)
    {
        _scrubber->addTick(lengths[i] / _timeSum);
    }

    _scrubber->setSelectedRangeIndex(getSelectedGUIConstraintIndex());
    _scrubber->update();
}

/*
 * This function is connected to the activated() signal of all of the
 * Qt4ConstraintMacros. It is called whenever a piece of constraint state
 * is modified in the GUI.
 */
void
MainWindow::
setSelectedAction(Qt4ConstraintMacro *activator)
{
    int selected_index = -1;
    bool noChange = false;

    for (std::vector<int>::size_type i = 0; i != _authoringState._all_gui_constraints.size(); i++)
    {
        if (activator == _authoringState._all_gui_constraints[i].get())
        {
            _authoringState._all_gui_constraints[i]->setSelected(true);

            if (_authoringState._selected_gui_constraint == _authoringState._all_gui_constraints[i])
            {
                noChange = true;
            }
            else
            {
                _authoringState._selected_gui_constraint = _authoringState._all_gui_constraints[i];
            }

            selected_index = i;
        }
        else
        {
            _authoringState._all_gui_constraints[i]->setSelected(false);
        }
    }

    if (! noChange)
    {
        // enable or disable media/move buttons
        _moveUpButton->setEnabled(selected_index != 0);
        _moveDownButton->setEnabled(selected_index != (int)_authoringState._all_gui_constraints.size() - 1);
        _fbwd->setEnabled(selected_index != 0);
        _ffwd->setEnabled(selected_index != (int)_authoringState._all_gui_constraints.size() - 1);

    }

    // update the scrubber tick lines
    updateScrubber();

    // highlight the affordance
    selectedOpenGLObjectChanged(_authoringState._selected_gui_constraint->getConstraintMacro()
                                ->getAtomicConstraint()->getAffordance()->getGUIDAsString());

    // highlight the manipulator
    _worldState.colorRobot.setSelectedLink(_authoringState._selected_gui_constraint->getConstraintMacro()
                                           ->getAtomicConstraint()->getManipulator()->getName());

    // update any flying manipulators
    updateFlyingManipulators();

    updatePointVisualizer();

    // prompt to set relation state
    std::cout << "getting mode states" << std::endl;
    _actionDescLabel->setText(QString::fromStdString(_authoringState._selected_gui_constraint->getModePrompt()));
    std::cout << "done getting mode states" << std::endl;

}

/*
 * Get the toggle panels from the Qt4ConstraintMacro objects and populate the gui
 */
void
MainWindow::
rebuildGUIFromState(AuthoringState &state, WorldStateView &worldState)
{
    // delete all of the current constraint's toggle panel widgets
    //    std::cout << "deleting rendered children " << std::endl;
    //    qDeleteAll(_constraint_container->findChildren<QWidget*>());

    for (std::vector<int>::size_type i = 0; i != state._all_gui_constraints.size(); i++)
    {
        state._all_gui_constraints[i]->setModelObjects(worldState.affordances, worldState.manipulators);

        if (! state._all_gui_constraints[i]->isInitialized())   // have we already added this tp?
        {
            TogglePanel *tp = state._all_gui_constraints[i]->getPanel();
            // TODO: the connect() call here appears to be quadratic in the
            // number of previously connected signals
            connect(state._all_gui_constraints[i].get(),
                    SIGNAL(activatedSignal(Qt4ConstraintMacro *)),
                    this, SLOT(setSelectedAction(Qt4ConstraintMacro *)));
            //		    Qt::UniqueConnection);
            //	    std::cout << "adding constraint #" << i << std::endl;
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

    if ((int)_worldState.affordances.size() == origSize) //todo : use a better check than size (like "==" on each affordance if the sizes are equal )
    {
        return;
    }

    cout << "\n\n\n size of _worldState.affordances changed \n\n" << endl;
    handleAffordancesChanged();
}

void
MainWindow::
selectedOpenGLObjectChanged(const std::string &modelGUID)
{
    selectedOpenGLObjectChanged(modelGUID, Eigen::Vector3f(0, 0, 0));
}


/*
 * Select the OpenGL object corresponding to this affordance by highlighting it
 * in the GUI. Connected to the raycastCallback signal from the OpenGL widget pane.
 */
void
MainWindow::
selectedOpenGLObjectChanged(const std::string &modelGUID, Eigen::Vector3f hitPoint)
{
    //cout << " hit " << modelGUID << " at point " << hitPoint.x() << ", " << hitPoint.y() << ", " << hitPoint.z() << endl;

    // highlight the object in the GUI
    for (uint i = 0; i < _worldState.glObjects.size(); i++)
    {
        if (_worldState.glObjects[i]->id() == modelGUID)
        {
            _worldState.glObjects[i]->setHighlighted(true);
        }
        else
        {
            _worldState.glObjects[i]->setHighlighted(false);
        }
    }

    _widget_opengl.update();

    if (_authoringState._selected_gui_constraint == NULL)
    {
        return;
    }

    // set the selected manipulator or affordance in the currently selected constraint pane
    // begin by getting the ModelState object from the selected openGL object
    // TODO: slow; use map objectsToModels
    bool wasAffordance = false;

    for (int i = 0; i < (int)_worldState.affordances.size(); i++)
    {
        if (_worldState.affordances[i]->getGUIDAsString() == modelGUID)
        {
            //std::cout << " setting affordance" << std::endl;
            _authoringState._selected_gui_constraint->getConstraintMacro()->getAtomicConstraint()->setAffordance(_worldState.affordances[i]);
            _authoringState._selected_gui_constraint->updateElementsFromState();
            wasAffordance = true;
            break;
        }
    }

    if (! wasAffordance)
    {
        for (int i = 0; i < (int)_worldState.manipulators.size(); i++)
        {
            if (_worldState.manipulators[i]->getGUIDAsString() == modelGUID)
            {
                //std::cout << " setting manipulator" << std::endl;
                _authoringState._selected_gui_constraint->getConstraintMacro()->getAtomicConstraint()->setManipulator(_worldState.manipulators[i]);
                _authoringState._selected_gui_constraint->updateElementsFromState();
                break;
            }
        }
    }

    if (_authoringState._selected_gui_constraint != NULL && ! (hitPoint.x() == 0 && hitPoint.y() == 0 && hitPoint.z() == 0))
    {
        // set the contact point for the relation
        RelationStatePtr rel = _authoringState._selected_gui_constraint->getConstraintMacro()->getAtomicConstraint()->getRelationState();

        if (rel->getRelationType() == RelationState::POINT_CONTACT)
        {
            PointContactRelationPtr pc = boost::static_pointer_cast<PointContactRelation>(rel);
            if (wasAffordance)
            {
                pc->setPoint2(hitPoint);
            }
            else
            {
                pc->setPoint1(hitPoint);
            }
            updatePointVisualizer();
        }
    }

    // prompt to set relation state
    _actionDescLabel->setText(QString::fromStdString(_authoringState._selected_gui_constraint->getModePrompt()));

    return;
}


void
MainWindow::
mediaFastForward()
{
    int i = getSelectedGUIConstraintIndex();
    _authoringState._all_gui_constraints[i + 1]->setActiveExternal();
}

void
MainWindow::
mediaFastBackward()
{
    int i = getSelectedGUIConstraintIndex();
    _authoringState._all_gui_constraints[i - 1]->setActiveExternal();
}

void
MainWindow::
mediaPlay()
{
    if (_isPlaying)
    {
        QPixmap pixmap3(":/trolltech/styles/commonstyle/images/media-pause-32.png");
        _play->setIcon(QIcon(pixmap3));
        _play->setIconSize(pixmap3.rect().size());
        _scrubberTimer->start(50); // 20 HZ
    }
    else
    {
        QPixmap pixmap3(":/trolltech/styles/commonstyle/images/media-play-32.png");
        _play->setIcon(QIcon(pixmap3));
        _play->setIconSize(pixmap3.rect().size());
        _scrubberTimer->stop();
    }

    _isPlaying = ! _isPlaying;
}

void
MainWindow::
nextKeyFrame()
{
    if (_scrubber->value() == _scrubber->maximum())
    {
        _scrubberTimer->stop();
    }
    else
    {
        _scrubber->setValue(_scrubber->value() + 1);
    }

    double fraction = (double)_scrubber->value() / (double)_scrubber->maximum();
    double acc = 0.0;

    // round fraction to the nearest constraint and select it
    for (std::vector<int>::size_type i = 0; i != _authoringState._all_gui_constraints.size(); i++)
    {
        acc += _authoringState._all_gui_constraints[i]->getConstraintMacro()->getTimeUpperBound();

        if ((acc / _timeSum) > fraction)
        {
            setSelectedAction(_authoringState._all_gui_constraints[i].get()); // todo ugly; have to because of qt
            break;
        }
    }
}

void
MainWindow::
publishActionToLCM()
{
    std::vector<Qt4ConstraintMacroPtr> all_gui_constraints = _authoringState._all_gui_constraints;
    std::vector<drc::contact_goal_t> contact_goals;

    for (int i = 0; i < (int) all_gui_constraints.size(); i++)
    {
        std::vector<drc::contact_goal_t> constraint_contact_goals = all_gui_constraints[i]->getConstraintMacro()->toLCM();

        for (int j = 0; j < (int) constraint_contact_goals.size(); j++)
        {
            contact_goals.push_back(constraint_contact_goals[j]);
        }
    }

    drc::action_sequence_t actionSequence;

    // TODO: get the actual name and the proper time
    actionSequence.utime = 0;
    actionSequence.robot_name = "foo bar robot";

    actionSequence.num_contact_goals = (int) contact_goals.size();
    actionSequence.contact_goals = contact_goals;

    _theLcm->publish(PLAN_ACTION_MESSAGE_CHANNEL, &actionSequence);
}

void
MainWindow::
changeMode()
{
    cout << "hi htere" << endl;

    if (_segmentedButton->isSegmentSelected(0))   // authoring view
    {
        _authoringWidgets->show();
        //_liveWidgets->hide();
    }
    else if (_segmentedButton->isSegmentSelected(1))   // live view
    {
        //_authoringWidgets->hide();
        //_liveWidgets->show();
    }

    cout << "done there" << endl;
}


void
MainWindow::
updateFlyingManipulators()
{
    // for each relation, add an appropriate "flying" manipulator
    for (uint i = 0; i < _worldState.manipulators.size(); i++)
    {
        string man_link_name = _worldState.manipulators[i]->getName();
        // TODO (mfleder, ine) here is : to pass the manipulator through
        // the currently selected *relation* and render the outcome

        // find any relations that reference this link. for each relation, spawn
        // a new flying affordance.
        for (std::vector<int>::size_type i = 0; i != _authoringState._all_gui_constraints.size(); i++)
        {
            RelationStatePtr rel = _authoringState._all_gui_constraints[i]->getConstraintMacro()->getAtomicConstraint()->getRelationState();
            string constraint_link_name = _authoringState._all_gui_constraints[i]->getConstraintMacro()->getAtomicConstraint()->getManipulator()->getName();

            if (constraint_link_name == man_link_name && rel->getRelationType() == RelationState::OFFSET) 
            {
                // TODO: mfleder, ine
/*
                cout << "ITS AN OFFSET " << endl;
                //KDL::Frame shifted_frame = _worldState.manipulators[i]->getLinkFrame();
                //shifted_frame.p = KDL::Vector(shifted_frame.p + rel->getTranslation());
                //shifted_frame = KDL::Vector(shifted_frame.p.x() + 0.25, shifted_frame.p.y(), shifted_frame.p.z());
                // TODO: this is a hack, we assume that the object is an OpenGL_Object_DAE. 
                // that's not necessarily true! It could be any subclass of OpenGL_Object
                OpenGL_Object_DAE* flying_link = new OpenGL_Object_DAE(
                    *((OpenGL_Object_DAE*)_worldState.colorRobot.getOpenGLObjectForLink(man_link_name)));
                OffsetRelationPtr offset = boost::dynamic_pointer_cast<OffsetRelation>(rel);
                //flying_link->set_transform(offset->getFrame());

                _widget_opengl.opengl_scene().add_object(*flying_link);
                _worldState.glObjects.push_back(flying_link);
*/
            }
        }
    }
}

void
MainWindow::
updateRobotState(const lcm::ReceiveBuffer* rbuf, 
                 const std::string& channel,
                 const drc::robot_state_t *new_robot_state)
{
    _worldState.state_gfe.from_lcm(*new_robot_state);
    _worldState.colorRobot.set(_worldState.state_gfe);
    // seems redundant but is necessary to get pose to work - don't remove!
    _worldState.colorRobot.set(*new_robot_state);
    // TODO: re-enable
    //handleAffordancesChanged();
}

void
MainWindow::
updatePointVisualizer() 
{
    // TODO: This should probably be abstracted into a class called
    // OpenGL_PointContactRelation or something
    RelationStatePtr rel = _authoringState._selected_gui_constraint->getConstraintMacro()->getAtomicConstraint()->getRelationState();

    // update the coordinate axis visualizer
    if (rel->getRelationType() == RelationState::POINT_CONTACT)
    {
        PointContactRelationPtr prel = boost::dynamic_pointer_cast<PointContactRelation>(rel);        

        KDL::Frame trans(KDL::Rotation::Quaternion(0, 0, 0, 1.0), KDL::Vector(prel->getPoint1().x(), prel->getPoint1().y(), prel->getPoint1().z()));
        point_contact_axis.set_transform(trans);

        KDL::Frame trans2(KDL::Rotation::Quaternion(0, 0, 0, 1.0), KDL::Vector(prel->getPoint2().x(), prel->getPoint2().y(), prel->getPoint2().z()));
        point_contact_axis2.set_transform(trans2);
    }
}

// create the manipulators from the robot's joints
// TODO : pull out worldState and robot
void
MainWindow::
createManipulators()
{
    // read the joints from the robot state
    std::map< std::string, State_GFE_Joint > joints = _worldState.state_gfe.joints();

    for (std::map< std::string, State_GFE_Joint >::const_iterator it = joints.begin(); it != joints.end(); it++)
    {
        const State_GFE_Joint &state_gfe_joint = it->second;
        std::string id = state_gfe_joint.id();
        shared_ptr<const urdf::Link> link = _worldState.colorRobot.getLinkFromJointName(id);

        ManipulatorStateConstPtr manipulator(new ManipulatorState(link,
                                             _worldState.colorRobot.getKinematicsModel().link(link->name),
                                             GlobalUID(rand(), rand()))); //todo guid

        _worldState.manipulators.push_back(manipulator);

        OpenGL_Manipulator *asGlMan = new OpenGL_Manipulator(manipulator);
        _widget_opengl.opengl_scene().add_object(*asGlMan);
        _worldState.glObjects.push_back(asGlMan);

        Collision_Object_Manipulator *cObjManip = new Collision_Object_Manipulator(manipulator);

        if (cObjManip->isSupported(manipulator))
        {
            _widget_opengl.add_collision_object(cObjManip);
            _worldState.collisionObjs.push_back(cObjManip);
        }
    }
}
