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
#include "QtPointContactRelation.h"

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
#include <action_authoring/AffordanceManipMap.h>

// File handling
#include "UtilityFile.h"

// GUI helpers
#include "GUIManipulators.h"

//TODO : CHANGE TO ADD LOAD FUNCTIONALITY
//#define DATABASE 
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
  class WorldStateView : public AffordanceManipMap
{
 public:
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
      : affServerWrapper(theLcm), colorRobot(kinematics::Kinematics_Model_GFE::urdf_filename_to_xml_string(getModelsPath() + urdf_filename))
    { 
      std::cout << "\n urdf_filename = " << urdf_filename << std::endl;
    }

  //====================AffordanceManipMap interface
 public:
    virtual affordance::AffConstPtr getAffordance(const affordance::GlobalUID &affordanceUID) const
      {
         for (uint i = 0; i < affordances.size(); i++)
          {
            if (affordances[i]->getGlobalUniqueId() == affordanceUID)
              return affordances[i];
          }

        throw std::runtime_error("WorldStateView: affordance not found w/ the given id");
      }
    
    virtual affordance::ManipulatorStateConstPtr getManipulator(const affordance::GlobalUID &manipulatorUID) const
      {
        for (uint i = 0; i < manipulators.size(); i++)
          {
            if (manipulators[i]->getGlobalUniqueId() == manipulatorUID)
              return manipulators[i];
          }
        throw std::runtime_error("WorldStateView: manipulator not found w/ the given id");
      }

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
    QtPointContactRelationPtr _selected_pcr_gui;
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
    double _MaxEndTime; // the length of time for all robot actions to complete. max of all upperTimeBounds
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


    int _newConstraintCounter;

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
                          const drc::robot_state_constraint_checked_t *robot_state);
    void updatePointVisualizer();
    void updateFilesListComboBox();
    void setP2PFromCurrConstraint(); //set point2Point from curr selection

protected:
    virtual void keyPressEvent(QKeyEvent *event);

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
    void mediaForward();
    void mediaBackward();
    void nextKeyFrame();
    void requestMotionPlan();
    void changeMode();

    action_authoring::PointContactRelationPtr getCurrentPCR();
    void updatePointContactRelation();

    void getContactGoals(std::vector<drc::contact_goal_t> *contact_goals);
    void requestIKSolution();
    void handleScrubberChange();
};

} //namespace action_authoring

#endif // MAINWINDOW_H
