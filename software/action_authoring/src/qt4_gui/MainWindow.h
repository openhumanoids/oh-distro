#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// QT includes
#include <QtGui>
#include <QWidget>
#include <QApplication>
#include <QSplitter>
#include <QToolBar>
// Custom QT widgets
#include "../qt4_widgets/TogglePanel.h"
#include "../qt4_widgets/CustomSlider.h"
#include "../qt4_widgets/segmentedbutton/qtsegmentcontrol.h"
#include "Qt4ConstraintMacro.h"

// OpenGL includes
#include "opengl/opengl_object.h"
#include "opengl/opengl_object_dae.h"
#include "opengl/opengl_object_coordinate_axis.h"

// The following are now subclassed
//#include "qt4/qt4_widget_opengl.h"
//#include "opengl/opengl_object_gfe.h"

// Collision stuff
#include <collision/collision_object_box.h>
#include <collision/collision_object_sphere.h>
#include <collision/collision_object_cylinder.h>
#include <collision/collision_object_gfe.h>

// Affordances
#include <affordance/ModelState.h>
#include <affordance/AffordanceState.h>
#include <affordance/AffordanceUpWrapper.h>
#include <affordance/OpenGL_Affordance.h>
#include <affordance/OpenGL_Manipulator.h>
#include <affordance/Collision_Object_Affordance.h>
#include <affordance/Collision_Object_Manipulator.h>

// Local includes
//#include <action_authoring/AtomicConstraintMacro.h>
#include "../opengl/ColorRobot.h"
#include "../opengl/SelectableOpenGLWidget.h"
#include <action_authoring/ManipulationConstraint.h>
#include <action_authoring/AtomicConstraint.h>
#include <action_authoring/ConstraintMacro.h>
#include <action_authoring/RelationState.h>
#include <action_authoring/PointContactRelation.h>
#include <action_authoring/OffsetRelation.h>

// File handling
#include "UtilityFile.h"

// GUI helpers
#include "GUIManipulators.h"

//TODO : CHANGE TO ADD LOAD FUNCTIONALITY
#define DATABASE 1
#ifdef DATABASE
#include <action_authoring/DatabaseManager.h>
#endif

namespace action_authoring
{

//===============WORLD STATE
class InvalidStateException : public std::runtime_error
{
public:
    InvalidStateException(const std::string &msg) : std::runtime_error(msg) {}
};

/**Represents the read-only state of the world and objects used for rendering that state*/
struct WorldStateView
{
    affordance::AffordanceUpWrapper affServerWrapper;  //used for reading affordances from the affordance server
    std::vector<affordance::AffConstPtr> affordances; //latest affordances read from the wrapper
    std::vector<affordance::ManipulatorStateConstPtr> manipulators; //robot manipulators
    state::State_GFE state_gfe; //robot state
    robot_opengl::ColorRobot colorRobot; //subclasses OpenGL_Object_GFE. used for coloring the robot
    opengl::OpenGL_Object_DAE *colorVehicle; // TODO : special case for demo
    std::vector<opengl::OpenGL_Object *> glObjects; //objects for rendering

    std::map<opengl::OpenGL_Object *, affordance::ModelStateConstPtr> objectsToModels; //map to identify clicked objects
    std::vector<collision::Collision_Object *> collisionObjs; //collision objects corresponding to glObjects

    /**initializes all the fields in the struct*/
    WorldStateView(const boost::shared_ptr<lcm::LCM> &theLcm, std::string urdf_filename)
        : affServerWrapper(theLcm), colorRobot(urdf_filename)
    { }

};

//==========AUTHORING STATE


/**represents the state of the authoring gui*/
typedef struct
{
    /**sorted sequence of actomic constraints being authored*/
    std::vector<Qt4ConstraintMacroPtr> _all_gui_constraints;
    Qt4ConstraintMacroPtr _selected_gui_constraint;
    std::string _selected_affordance_guid;
    std::string _selected_manipulator_guid;
} AuthoringState;

//===========MAIN WINDOW

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(const boost::shared_ptr<lcm::LCM> &theLcm, QWidget *parent = 0);
    boost::shared_ptr<TogglePanel> createWaypointGUI(Qt4ConstraintMacroPtr waypoint_constraint,
            std::vector<std::string> joint_names);
    ~MainWindow();

    //=================gui state
private:
    boost::shared_ptr<lcm::LCM> _theLcm;
    QTimer *_scrubberTimer;
    QTimer *_affordanceUpdateTimer;
    QLineEdit *_actionName;
    QComboBox *_filesList;
    DefaultValueSlider *_scrubber;
    QLabel *_actionDescLabel;
    double _timeSum; // total length of time for all robot actions to complete. sum of all upperTimeBounds
    robot_opengl::SelectableOpenGLWidget _widget_opengl;
    QWidget *_constraint_container;
    QVBoxLayout *_constraint_vbox;
    QPushButton *_moveUpButton;
    QPushButton *_moveDownButton;
    QPushButton *_ffwd;
    QPushButton *_fwd;
    QPushButton *_play;
    bool _isPlaying;
    QPushButton *_bwd;
    QPushButton *_fbwd;
    QtSegmentControl *_segmentedButton;
    QWidget *_liveWidgets;
    QWidget *_authoringWidgets;

    // TODO: temporary
    opengl::OpenGL_Object_Coordinate_Axis point_contact_axis;
    opengl::OpenGL_Object_Coordinate_Axis point_contact_axis2;

    //================world state and authoring state
private:
    WorldStateView _worldState; //view of the world state
    AuthoringState _authoringState; //state of authoring

private:
    void rebuildGUIFromState(AuthoringState &state, WorldStateView &worldState);
    void handleAffordancesChanged(); //only called if the affordances have changed
    int getSelectedGUIConstraintIndex();
    void moveQt4Constraint(bool up);
    void updateScrubber();
    void updateFlyingManipulators();
    void updateRobotState(const lcm::ReceiveBuffer* rbuf, 
                          const std::string& channel,
                          const drc::robot_state_t *robot_state);
    void updatePointVisualizer();
    void updateFilesListComboBox();

protected:
    virtual void  keyPressEvent(QKeyEvent *event);

private slots:
    void affordanceUpdateCheck(); //called to see if should update _worldState.affordances

    void handleLoadAction();
    void handleLoadAction(std::string);
    void handleLoadActionCombo();
    void handleSaveAction();
    void handleDeleteConstraint();
    void handleAddConstraint();
    void handleMoveUp();
    void handleMoveDown();
    void setSelectedAction(Qt4ConstraintMacro *activator);
    void selectedOpenGLObjectChanged(const std::string &modelGUID);
    void selectedOpenGLObjectChanged(const std::string &modelName, Eigen::Vector3f hitPoint);
    void mediaFastForward();
    void mediaFastBackward();
    void mediaPlay();
    void nextKeyFrame();
    void publishActionToLCM();
    void changeMode();
};

} //namespace action_authoring

#endif // MAINWINDOW_H
