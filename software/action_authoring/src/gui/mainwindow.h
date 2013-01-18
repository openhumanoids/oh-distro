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
#include "Qt4Constraint.h"

// OpenGL includes
#include "opengl/opengl_object_box.h"
#include "opengl/opengl_object_cylinder.h"
#include "opengl/opengl_object_sphere.h"

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
#include "../AtomicConstraint.h"
#include "../ConstraintSequence.h"
#include "../opengl/ColorRobot.h"
#include "../opengl/SelectableOpenGLWidget.h"


namespace action_authoring
{

/**Represents the read-only state of the world and objects used for rendering that state*/
struct WorldStateView
{
	//----------world state fields
	/**used for reading affordances from the affordance server*/
	affordance::AffordanceUpWrapper affordances;
  
    /**robot state*/
    state::State_GFE state_gfe;

    /**used for coloring the robot*/
    robot_opengl::ColorRobot colorRobot; //subclasses OpenGL_Object_GFE
  
  /*
    collision::Collision_Object_Sphere* _collision_object_sphere;
    collision::Collision_Object_GFE* _collision_object_gfe;
  */

    //-------------
    /**initializes all the fields in the struct*/
    WorldStateView(const boost::shared_ptr<lcm::LCM> &theLcm)
  	  : affordances(theLcm), colorRobot()
    { }

};


/**represents the state of the authoring gui*/
typedef struct
{
	/**sorted sequence of actomic constraints being authored*/
        std::vector<Qt4ConstraintPtr> _all_gui_constraints;
        Qt4ConstraintPtr _selected_gui_constraint;
} AuthoringState;

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(const boost::shared_ptr<lcm::LCM> &theLcm, QWidget *parent = 0);
    boost::shared_ptr<TogglePanel> createWaypointGUI(Qt4ConstraintPtr waypoint_constraint,
				   std::vector<std::string> joint_names);
    std::vector<std::string> getJointNames(std::string urdf_xml_filename);
    void demoPopulate();
    ~MainWindow();

protected:
    QSlider* _jointSlider;
    QLabel * _jointNameLabel;
    state::State_GFE _state_gfe;
    QSignalMapper* _signalMapper;
    robot_opengl::SelectableOpenGLWidget _widget_opengl;

    std::vector<affordance::AffPtr> _all_affordances;

    //================world state and authoring state
private:
    WorldStateView _worldState; //view of the world state
    AuthoringState _authoringState; //state of authoring

    //todo: move this into another class
    std::vector<std::string> getJointNames(std::string urdf_xml_filename) const;

private:
    void demoPopulateConstraints(); //todo : remove this
    std::string getSelectedJointName();

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
