#ifndef AUTHORING_QT4_WIDGET_OPENGL_AUTHORING_H
#define AUTHORING_QT4_WIDGET_OPENGL_AUTHORING_H

#include <iostream>
#include <string>
#include <vector>

#include <qt4/qt4_widget_opengl.h>

#include <authoring/constraint.h>
#include <authoring/constraint_task_space_region.h>
#include <affordance/AffordanceState.h>
#include <opengl/opengl_object_gfe.h>
#include <opengl/opengl_object_trajectory_gfe.h>
#include <opengl/opengl_object_coordinate_axis.h>
#include <state/state_gfe.h>

#include <authoring/opengl_object_affordance_collection.h>
#include <authoring/opengl_object_constraint_sequence.h>
#include <authoring/qt4_widget_constraint_editor.h>

#include <collision/collision_object_container_coordinate_axis.h>
//#include <authoring/interface_handler_authoring.h>

namespace authoring {

    enum adjust_axes_t {
        ADJUST_NONE = -1,
        ADJUST_TX = 0,
        ADJUST_TY = 1,
        ADJUST_TZ = 2,
        ADJUST_RR = 3,
        ADJUST_RP = 4,
        ADJUST_RY = 5,
        ADJUST_PLUS = 100,
        ADJUST_MINUS = 101
    };
    enum adjust_dir_t {

    };

  class Qt4_Widget_OpenGL_Authoring : public qt4::Qt4_Widget_OpenGL {
    Q_OBJECT
  public:
    Qt4_Widget_OpenGL_Authoring( const std::string& xmlString, QWidget * parent = 0 );
    ~Qt4_Widget_OpenGL_Authoring();
    Qt4_Widget_OpenGL_Authoring( const Qt4_Widget_OpenGL_Authoring& other );
    Qt4_Widget_OpenGL_Authoring& operator=( const Qt4_Widget_OpenGL_Authoring& other );

  signals:
    void update_constraint( const Constraint_Task_Space_Region & cnst );
    void select_constraint( const QString& id, select_class_t select_class );
    void publish_constraints( float ik_time_of_interest );

  public slots:
    void set_affordance_collection( const std::vector< affordance::AffordanceState >& affordanceCollection );
    void update_opengl_object_affordance_collection( std::vector< affordance::AffordanceState >& affordanceCollection );
    void update_opengl_object_affordance_collection_ghost( std::vector< affordance::AffordanceState >& affordanceCollection );
    void update_opengl_object_constraint_sequence( const Constraint_Sequence& constraintSequence );
    void update_opengl_object_robot_plan( std::vector< state::State_GFE >& robotPlan );
    void update_opengl_object_gfe( state::State_GFE& stateGFE );
    void update_opengl_object_gfe_ghost( state::State_GFE& stateGFE );
    void update_opengl_object_gfe_selected_links( const std::vector<std::string>& linkNames );
    void update_opengl_object_robot_plan_current_index( int currentIndex );
    void update_opengl_object_robot_plan_visible_current_index( int visibleCurrentIndex );
    void update_opengl_object_robot_plan_visible_trajectory( int visibleTrajectory );
    void update_opengl_object_robot_plan_visible_trajectory_wrist( int visibleTrajectoryWrist );
    void update_opengl_object_robot_plan_visible_initial_state( int visibleTrajectoryWrist );
    void highlight_constraint( const QString& id, highlight_class_t highlight_class, bool highlight );
    void highlight_constraints( const std::vector< std::string >& ids, highlight_class_t highlight_class );

    void highlight_child( const QString& id, const QString& child, bool highlight );

    void set_collision_to_constraints( void );
    void set_collision_to_axes( void );
    void bind_axes_to_constraint( Constraint_Task_Space_Region * cnst, bool unbind_if_duplicate = false );
    void unbind_axes_from_constraint( Constraint_Task_Space_Region * cnst );

    //virtual void add_collision_object( collision::Collision_Object& collisionObject );
    //virtual Interface_Handler_Authoring& interface_handler( void );

    virtual void mouseIdleEvent( void );
    virtual void mouseMoveEvent( QMouseEvent * event );
    virtual void mousePressEvent( QMouseEvent * event );
    virtual void mouseReleaseEvent( QMouseEvent * event );

    KDL::Vector get_approx_closest_approach_on_line_from_mouse_coords( 
        GLdouble * modelview_matrix, GLdouble * projection_matrix, GLint * viewport,
        double x, double y, KDL::Vector line_origin, KDL::Vector line_dir);
    KDL::Vector get_intersection_on_plane_from_mouse_coords( 
        GLdouble * modelview_matrix, GLdouble * projection_matrix, GLint * viewport, 
        double x, double y, KDL::Vector plane_origin, KDL::Vector plane_normal);

  protected slots:
    void _timer_update_callback( void );

  protected:
    std::vector< affordance::AffordanceState > _affordance_collection;
    OpenGL_Object_Affordance_Collection _opengl_object_affordance_collection;
    OpenGL_Object_Affordance_Collection _opengl_object_affordance_collection_ghost;
    OpenGL_Object_Constraint_Sequence _opengl_object_constraint_sequence;
    opengl::OpenGL_Object_Trajectory_GFE _opengl_object_robot_plan;
    opengl::OpenGL_Object_GFE _opengl_object_gfe;
    opengl::OpenGL_Object_GFE _opengl_object_gfe_ghost;
    // axes used for manip -- only one set exists, and is bound to one constraint being
    // manip'd at once
    opengl::OpenGL_Object_Coordinate_Axis _opengl_object_xyz_axis;
    opengl::OpenGL_Object_Coordinate_Axis _opengl_object_rpy_axis;
    collision::Collision_Object_Container_Coordinate_Axis _collision_object_container_xyz_axis;
    collision::Collision_Object_Container_Coordinate_Axis _collision_object_container_rpy_axis;
    Constraint_Task_Space_Region * _bound_constraint_task_space_region;
    KDL::Frame _bound_constraint_midpoint;
    QTimer* _timer_update;

    // Whether mouse is captured, overriding camera movement
    bool _mouse_captured;
    // Pos at capture
    KDL::Vector2 _mouse_capture_pos;
    KDL::Vector _mouse_capture_last_point;
    // Which axis is being adjusted
    adjust_axes_t _adjust_axis;
    adjust_axes_t _adjust_dir;

  private:


  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_OpenGL_Authoring& other );
}

#endif /* AUTHORING_QT4_WIDGET_OPENGL_AUTHORING_H */
