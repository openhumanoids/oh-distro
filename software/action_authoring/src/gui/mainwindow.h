#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// QT includes
#include <QtGui>
#include <QWidget>
#include <QApplication>
#include <QSplitter>
#include <QTreeView>
#include <QListView>
#include <QTableView>
#include <QtGui>
#include <QStandardItemModel>

// Custom QT widget
#include "togglepanel.h"

// OpenGL includes
#include "opengl/opengl_object_box.h"
#include "opengl/opengl_object_cylinder.h"
#include "opengl/opengl_object_sphere.h"

// The following are now subclassed
//#include "qt4/qt4_widget_opengl.h"

// Collision stuff
#include <collision/collision_object_box.h>
#include <collision/collision_object_sphere.h>
#include <collision/collision_object_cylinder.h>
#include <collision/collision_object_gfe.h>

// affordances
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
	std::vector<AtomicConstraintPtr> _all_constraints;
} AuthoringState;



/**Main GUI Window*/
class MainWindow : public QWidget
{
    Q_OBJECT

public:
    //--------------constructor
    explicit MainWindow(const boost::shared_ptr<lcm::LCM> &theLcm, QWidget *parent = 0);
    TogglePanel* createWaypointGUI(AtomicConstraintPtr waypoint_constraint,
				 	 	 	 	   std::vector<std::string> joint_names);
    ~MainWindow();

    //------------qt stuff
protected:
    std::map<std::string, TogglePanel*> _all_panels;
    std::map<std::string, QComboBox*> _all_robot_link_combos;
    std::string _selectedJointName;
    QSlider* _jointSlider;

    QSignalMapper* _signalMapper;
    robot_opengl::SelectableOpenGLWidget _widget_opengl;

    //qt4::Qt4_Widget_OpenGL _qt4_widget_opengl;

    //================world state and authoring state
private:
    WorldStateView _worldState; //view of the world state
    AuthoringState _authoringState; //state of authoring

    //todo: move this into another class
    std::vector<std::string> getJointNames(std::string urdf_xml_filename) const;

private:
    void demoPopulateConstraints(); //todo : remove this


//================slots
private slots:
    void updateJoint(int value);
    void handleLoadAction();
    void handleSaveAction();
    void handleDeleteWaypoint();
    void handleMoveUp();
    void handleMoveDown();
    void handleToggleExpandContract();
    void handleRobotLinkChange(QString waypointName);
    void setSelectedAction(QString waypointName);
};

} //namespace action_authoring

#endif // MAINWINDOW_H
