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
//#include "opengl/opengl_object_gfe.h"

// Collision stuff
#include <collision/collision_object_box.h>
#include <collision/collision_object_sphere.h>
#include <collision/collision_object_cylinder.h>
#include <collision/collision_object_gfe.h>

// Local includes
#include "../AtomicConstraint.h"
#include "../ConstraintSequence.h"
#include "../opengl/ColorRobot.h"
#include "../opengl/SelectableOpenGLWidget.h"


namespace action_authoring
{


class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    TogglePanel* createWaypointGUI(AtomicConstraintPtr waypoint_constraint,
				 	 	 	 	   std::vector<std::string> joint_names);
    std::vector<std::string> getJointNames(std::string urdf_xml_filename);
    void demoPopulate();
    ~MainWindow();

protected:
    std::map<std::string, TogglePanel*> _all_panels;
    std::map<std::string, QComboBox*> _all_robot_link_combos;
    std::string _selectedJointName;
    QSlider* _jointSlider;
    state::State_GFE _state_gfe;
    QSignalMapper* _signalMapper;
//qt4::Qt4_Widget_OpenGL _qt4_widget_opengl;
    robot_opengl::SelectableOpenGLWidget _widget_opengl;

    opengl::OpenGL_Object_Box _opengl_object_box;
    opengl::OpenGL_Object_Cylinder _opengl_object_cylinder;
    opengl::OpenGL_Object_Sphere _opengl_object_sphere;
    robot_opengl::ColorRobot _colorRobot;

    collision::Collision_Object_Box* _collision_object_box;
    collision::Collision_Object_Cylinder* _collision_object_cylinder;
    collision::Collision_Object_Sphere* _collision_object_sphere;
//    collision::Collision_Object_GFE* _collision_object_gfe;

//    opengl::OpenGL_Object_GFE _opengl_object_gfe;

    std::vector<affordance::AffPtr> _all_affordances;
    std::vector<AtomicConstraintPtr> _all_constraints;

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
