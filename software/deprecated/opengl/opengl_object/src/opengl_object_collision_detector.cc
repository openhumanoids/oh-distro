#include <GL/gl.h>

#include "opengl/opengl_object_collision_detector.h"

using namespace std;
using namespace Eigen;
using namespace collision;
using namespace opengl;

OpenGL_Object_Collision_Detector::
OpenGL_Object_Collision_Detector() : _collision_detector( NULL ),
                                      _opengl_object_cylinder(),
                                      _opengl_object_sphere(){
  _opengl_object_sphere.set_color( Eigen::Vector4f( 1.0, 1.0, 0.0, 1.0 ) );
}

OpenGL_Object_Collision_Detector::
~OpenGL_Object_Collision_Detector() {

}

OpenGL_Object_Collision_Detector::
OpenGL_Object_Collision_Detector( const OpenGL_Object_Collision_Detector& other ) : _collision_detector( other._collision_detector ),
                                                                                    _opengl_object_cylinder( other._opengl_object_cylinder ),
                                                                                    _opengl_object_sphere( other._opengl_object_sphere ){

}

OpenGL_Object_Collision_Detector&
OpenGL_Object_Collision_Detector::
operator=( const OpenGL_Object_Collision_Detector& other ) {
  _id = other._id;
  _visible = other._visible;
  _color = other._color;
  _transparency = other._transparency;
  _transform = other._transform;
  _offset = other._offset;
  _opengl_object_cylinder = other._opengl_object_cylinder;
  _opengl_object_sphere = other._opengl_object_sphere;
  return (*this);
}

void
OpenGL_Object_Collision_Detector::
set( Collision_Detector& collisionDetector ){
  _collision_detector = &collisionDetector;
  return;
}

void
OpenGL_Object_Collision_Detector::
draw( void ){
  if( visible() ){
    glColor4f( 1.0, 1.0, 0.0, 1.0 );
    glPolygonMode(GL_FRONT, GL_LINE);
    glPolygonMode(GL_BACK, GL_LINE);
    glDisable( GL_LIGHTING );
    glDisable( GL_COLOR_MATERIAL );
    if( _collision_detector != NULL ){
      btCollisionObjectArray& collision_object_array = _collision_detector->bt_collision_world().getCollisionObjectArray();
      for( unsigned int i = 0; i < collision_object_array.size(); i++ ){
        btCollisionObject * collision_object = collision_object_array[ i ];
        glPushMatrix();
        btTransform& transform = collision_object->getWorldTransform();
        btVector3 translation = transform.getOrigin();
        btQuaternion quaternion = transform.getRotation();
        glTranslatef( translation.x(), translation.y(), translation.z() );
        glRotatef(quaternion.getAngle()*180.0/M_PI,quaternion.getAxis().x(),quaternion.getAxis().y(),quaternion.getAxis().z());
        _draw_collision_shape( collision_object->getCollisionShape() );
        glPopMatrix();
      }
    }
    glEnable( GL_LIGHTING );
    glEnable( GL_COLOR_MATERIAL );
    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_FILL);
  }
  return;
}

void
OpenGL_Object_Collision_Detector::
_draw_collision_shape( btCollisionShape* collisionShape ){
  btVector3 edge_a, edge_b;
  btBoxShape * bt_box_shape = NULL;
  btConvexHullShape * bt_convex_hull_shape = NULL;
  btCompoundShape * bt_compound_shape = NULL;
  switch( collisionShape->getShapeType() ){
  case ( BOX_SHAPE_PROXYTYPE ):
    bt_box_shape = dynamic_cast< btBoxShape* >( collisionShape );
    glBegin( GL_LINES );
    for( unsigned int j = 0; j < bt_box_shape->getNumEdges(); j++ ){
      bt_box_shape->getEdge( j, edge_a, edge_b );
      glVertex3f( edge_a.x(), edge_a.y(), edge_a.z() );
      glVertex3f( edge_b.x(), edge_b.y(), edge_b.z() );
    }
    glEnd();
    glPointSize(5.0);
    glBegin(GL_LINES);
    for( unsigned int j = 0; j < bt_box_shape->getNumVertices(); j++ ){
      bt_box_shape->getVertex( j, edge_a );
      glVertex3f( edge_a.x(), edge_a.y(), edge_a.z() );
    }
    glEnd();
    glPointSize(1.0);
    break;
  case ( CONVEX_HULL_SHAPE_PROXYTYPE ):
    bt_convex_hull_shape = dynamic_cast< btConvexHullShape* >( collisionShape );
    glBegin( GL_LINES );
    for( unsigned int j = 0; j < bt_convex_hull_shape->getNumEdges(); j++ ){
      bt_convex_hull_shape->getEdge( j, edge_a, edge_b );
      glVertex3f( edge_a.x(), edge_a.y(), edge_a.z() );
      glVertex3f( edge_b.x(), edge_b.y(), edge_b.z() );
    }
    glEnd();
    glPointSize(5.0);
    glBegin(GL_LINES);
    for( unsigned int j = 0; j < bt_convex_hull_shape->getNumVertices(); j++ ){
      bt_convex_hull_shape->getVertex( j, edge_a );
      glVertex3f( edge_a.x(), edge_a.y(), edge_a.z() );
    }
    glEnd();
    glPointSize(1.0);
    break;
  case ( SPHERE_SHAPE_PROXYTYPE ):
    _opengl_object_sphere.set( dynamic_cast< btSphereShape* >( collisionShape )->getRadius() );
    _opengl_object_sphere.draw( Vector3f( 1.0, 1.0, 0.0 ) );
    break;
  case ( CYLINDER_SHAPE_PROXYTYPE ):
    glPushMatrix();
    switch( dynamic_cast< btCylinderShape* >( collisionShape )->getUpAxis() ){
    case ( 0 ):
      _opengl_object_cylinder.set( Vector2f( dynamic_cast< btCylinderShape* >( collisionShape )->getRadius(),
                                             dynamic_cast< btConvexInternalShape* >( collisionShape )->getImplicitShapeDimensions().x() * 2 ) );
      break;
    case ( 1 ):
      glRotatef( 90.0, 1.0, 0.0, 0.0 );
      _opengl_object_cylinder.set( Vector2f( dynamic_cast< btCylinderShape* >( collisionShape )->getRadius(),
                                             dynamic_cast< btConvexInternalShape* >( collisionShape )->getImplicitShapeDimensions().y() * 2 ) );
      break;
    case ( 2 ):
      _opengl_object_cylinder.set( Vector2f( dynamic_cast< btCylinderShape* >( collisionShape )->getRadius(),
                                             dynamic_cast< btConvexInternalShape* >( collisionShape )->getImplicitShapeDimensions().z() * 2 ) );
      break;
    default:
      break;
    }
    _opengl_object_cylinder.draw( Vector3f( 1.0, 1.0, 0.0 ) );
    glPopMatrix();
    break;
  case ( CONE_SHAPE_PROXYTYPE ):
    cout << "not drawing cone" << endl;
    break;  
  case ( COMPOUND_SHAPE_PROXYTYPE ):
    bt_compound_shape = dynamic_cast< btCompoundShape* >( collisionShape );
    for( unsigned int i = 0; i < bt_compound_shape->getNumChildShapes(); i++ ){
      glPushMatrix();
      btTransform& transform = bt_compound_shape->getChildTransform( i );
      btVector3 translation = transform.getOrigin();
      btQuaternion quaternion = transform.getRotation();
      glTranslatef( translation.x(), translation.y(), translation.z() );
      glRotatef(quaternion.getAngle()*180.0/M_PI,quaternion.getAxis().x(),quaternion.getAxis().y(),quaternion.getAxis().z());
      _draw_collision_shape( bt_compound_shape->getChildShape( i ) );
      glPopMatrix();
    }
    break;
  default:
    cout << "could not draw collision shape" << endl;
    break;
  }
  return;
} 

namespace opengl {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Object_Collision_Detector& other ) {
    return out;
  }

}
