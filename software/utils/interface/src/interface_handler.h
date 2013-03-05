#ifndef INTERFACE_INTERFACE_HANDLER_H
#define INTERFACE_INTERFACE_HANDLER_H

#include <iostream>
#include <GL/gl.h>

#include <kdl/frames.hpp>

#include <collision/collision_detector.h>
#include <collision/collision_object.h>

namespace interface {
  typedef enum {
    MOUSE_BUTTON_NONE,
    MOUSE_BUTTON_LEFT,
    MOUSE_BUTTON_RIGHT,
    MOUSE_BUTTON_MIDDLE,
    NUM_MOUSE_BUTTONS
  } mouse_button_t;

  typedef enum {
    MOUSE_EVENT_MOVE,
    MOUSE_EVENT_CLICK,
    MOUSE_EVENT_DOUBLE_CLICK,
    MOUSE_EVENT_RELEASE,
    NUM_MOUSE_EVENTS
  } mouse_event_t;

  class Interface_Handler {
  public:
    Interface_Handler();
    ~Interface_Handler();
    Interface_Handler( const Interface_Handler& other );
    Interface_Handler& operator=( const Interface_Handler& other );

    void set_eye_position( KDL::Vector eyePosition );
    void set_modelview_matrix( GLdouble modelviewMatrix[] );
    void set_projection_matrix( GLdouble projectionMatrix[] );
    void set_viewport( GLint viewport[] );

    void add_collision_object( collision::Collision_Object& collisionObject );

    void mouse_event( mouse_event_t mouseEvent, mouse_button_t mouseButton, unsigned int mouseX, unsigned int mouseY );
    void mouse_event_idle( void );
    collision::Collision_Detector& collision_detector( void );
    collision::Collision_Object* intersected_object( void );

  protected:
    void _mouse_event_move( mouse_button_t mouseButton );
    void _mouse_event_click( mouse_button_t mouseButton );
    void _mouse_event_double_click( mouse_button_t mouseButton );
    void _mouse_event_release( mouse_button_t mouseButton );

    GLdouble  _modelview_matrix[ 16 ];
    GLdouble  _projection_matrix[ 16 ];
    GLint     _viewport[ 4 ];
  
    KDL::Vector _mouse_position;
    KDL::Vector _eye_position;
    
    collision::Collision_Detector               _collision_detector;
    std::vector< collision::Collision_Object* > _collision_objects;
    collision::Collision_Object*                _intersected_object;

    unsigned int _num_event_idle_intersections;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Interface_Handler& other );
}

#endif /* INTERFACE_INTERFACE_HANDLER_H */
