#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// QT includes
#include <QtGui>
#include <QWidget>
#include <QApplication>
#include <QSplitter>
// Custom QT widgets
#include "togglepanel.h"
#include "customslider.h"
#include "Qt4ConstraintMacro.h"

// OpenGL includes
#include "opengl/opengl_object.h"

// The following are now subclassed
//#include "qt4/qt4_widget_opengl.h"
//#include "opengl/opengl_object_gfe.h"

// Collision stuff
#include <collision/collision_object_box.h>
#include <collision/collision_object_sphere.h>
#include <collision/collision_object_cylinder.h>
#include <collision/collision_object_gfe.h>

// Affordances
#include "affordance/AffordanceState.h"
#include "affordance/AffordanceUpWrapper.h"

// Local includes
//#include "action_authoring/AtomicConstraintMacro.h"
#include "../opengl/ColorRobot.h"
#include "../opengl/SelectableOpenGLWidget.h"
#include "action_authoring/AtomicConstraint.h"
#include "action_authoring/ConstraintMacro.h"
#include "action_authoring/DatabaseManager.h"

namespace action_authoring
{

/**Represents the read-only state of the world and objects used for rendering that state*/
struct WorldStateView
{

  affordance::AffordanceUpWrapper affServerWrapper;  //used for reading affordances from the affordance server
  std::vector<affordance::AffPtr> affordances; //latest affordances read from the wrapper
  state::State_GFE state_gfe; //robot state
  robot_opengl::ColorRobot colorRobot; //subclasses OpenGL_Object_GFE. used for coloring the robot
  //std::vector<collision::Collision_Object> collisionObjs;
  
  /**initializes all the fields in the struct*/
WorldStateView(const boost::shared_ptr<lcm::LCM> &theLcm)
 : affServerWrapper(theLcm), colorRobot()
  { }
  
};
 

/**represents the state of the authoring gui*/
typedef struct
{
	/**sorted sequence of actomic constraints being authored*/
        std::vector<Qt4ConstraintMacroPtr> _all_gui_constraints;
        Qt4ConstraintMacroPtr _selected_gui_constraint;
} AuthoringState;

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(const boost::shared_ptr<lcm::LCM> &theLcm, QWidget *parent = 0);
    boost::shared_ptr<TogglePanel> createWaypointGUI(Qt4ConstraintMacroPtr waypoint_constraint,
				   std::vector<std::string> joint_names);
    std::vector<std::string> getJointNames(std::string urdf_xml_filename);
    void demoPopulate();
    ~MainWindow();

    //=================gui state
 private: 
    QSlider* _jointSlider;
    QLabel * _jointNameLabel;
    QSignalMapper* _signalMapper;
    robot_opengl::SelectableOpenGLWidget _widget_opengl;
    QVBoxLayout* _constraint_vbox;

    //================world state and authoring state
private:
    WorldStateView _worldState; //view of the world state
    AuthoringState _authoringState; //state of authoring
    //todo: move this into another class
    std::vector<std::string> getJointNames(std::string urdf_xml_filename) const;

private:
    void demoPopulateConstraintMacros(); //todo : remove this
    std::string getSelectedJointName();
    void makeGUIFromConstraintMacros();

    //private signals:
    //    void 

private slots:
    void updateJoint(int value);
    void handleLoadAction();
    void handleSaveAction();
    void handleDeleteWaypoint();
    void handleMoveUp();
    void handleMoveDown();
    void handleRobotLinkChange();
    void setSelectedAction(QString activator);
};

} //namespace action_authoring

#endif // MAINWINDOW_H
