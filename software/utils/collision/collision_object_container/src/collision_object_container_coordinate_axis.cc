#include <collision/collision_object_container_coordinate_axis.h>

using namespace std;
using namespace Eigen;
using namespace KDL;
using namespace collision;

/**
 * Collision_Object_Container_Coordinate_Axis
 * class constructor with axis, id, dimension, position, and orientation arguments
 */
Collision_Object_Container_Coordinate_Axis::
Collision_Object_Container_Coordinate_Axis( 
                      bool drawTranslationAxes,
                      bool drawRotationAxes,
                      string id,
                      Vector3f dims,
                      Vector3f position,
                      Vector4f orientation,
                      double min_scale) : Collision_Object( id ),
          _min_scale( min_scale ),
          _scale( _min_scale ),
          _draw_translation_axes( drawTranslationAxes ),
          _draw_rotation_axes( drawRotationAxes ),
          _collision_ptx( new Collision_Object_Cylinder( id + "+tx", _scale/20.0, _scale ) ),
          _collision_pty( new Collision_Object_Cylinder( id + "+ty", _scale/20.0, _scale ) ),
          _collision_ptz( new Collision_Object_Cylinder( id + "+tz", _scale/20.0, _scale ) ),
          _collision_mtx( new Collision_Object_Cylinder( id + "-tx", _scale/20.0, _scale ) ),
          _collision_mty( new Collision_Object_Cylinder( id + "-ty", _scale/20.0, _scale ) ),
          _collision_mtz( new Collision_Object_Cylinder( id + "-tz", _scale/20.0, _scale ) ),
          _collision_rr( new Collision_Object_Torus( id + "+rr", _scale/40.0, _scale/1.5 ) ),
          _collision_rp( new Collision_Object_Torus( id + "+rp", _scale/40.0, _scale/1.5 ) ),
          _collision_ry( new Collision_Object_Torus( id + "+ry", _scale/40.0, _scale/1.5 ) ),
          _xdir( 1.0, 0.0, 0.0),
          _ydir( 0.0, 1.0, 0.0),
          _zdir( 0.0, 0.0, 1.0)
          {
  set_transform( position, orientation, 1.0 );
}


/**
 * Collision_Object_Container_Coordinate_Axis
 * class constructor with id, dimension, position, and orientation arguments
 */   //TORUS DOESN'T SUPPORT THIS YET.
/*
Collision_Object_Container_Coordinate_Axis::
Collision_Object_Container_Coordinate_Axis( 
                      bool drawTranslationAxes,
                      bool drawRotationAxes,
                      string id,
                      Vector3f dims,
                      const Frame& offset,
                      const Frame& transform ) : Collision_Object( id, true, offset ),
          _scale( 0.1 ),
          _draw_translation_axes( drawTranslationAxes ),
          _draw_rotation_axes( drawRotationAxes ),
          _collision_tx( id + "-tx", _scale/20.0, _scale, offset),
          _collision_ty( id + "-ty", _scale/20.0, _scale, offset),  
          _collision_tz( id + "-tz", _scale/20.0, _scale, offset),
          _collision_rr( id + "-rr", _scale/40.0, _scale/1.5, offset ),
          _collision_rp( id + "-rp", _scale/40.0, _scale/1.5, offset ),
          _collision_ry( id + "-ry", _scale/40.0, _scale/1.5, offset )  
                                               {
  set_transform( transform );
}
*/

/**
 * Collision_Object_Container_Coordinate_Axis
 * copy constructor 
 */ /*
Collision_Object_Container_Coordinate_Axis::
Collision_Object_Container_Coordinate_Axis( const Collision_Object_Container_Coordinate_Axis& other ) : 
          Collision_Object( other )                           {
  _draw_rotation_axes = other._draw_rotation_axes;
  _draw_translation_axes = other._draw_translation_axes;
  _scale = other._scale;
  _collision_tx = new Collision_Object_Cylinder( other._collision_tx );
  _collision_ty = new Collision_Object_Cylinder( other._collision_ty );
  _collision_tz = new Collision_Object_Cylinder( other._collision_tz );
  _collision_rr = new Collision_Object_Torus( other._collision_rr );
  _collision_rp = new Collision_Object_Torus( other._collision_rp );
  _collision_ry = new Collision_Object_Torus( other._collision_ry );
} */

/**
 * ~Collision_Object_Container_Coordinate_Axis
 * class destructor
 */
Collision_Object_Container_Coordinate_Axis::
~Collision_Object_Container_Coordinate_Axis(){

}

/** 
 * set_transform
 * sets the world-frame position and orientation of the collision object
 */
void
Collision_Object_Container_Coordinate_Axis::
set_transform( const Vector3f position,
                const Vector4f orientation, double new_scale ){
  // update scale
  // TODO: free old objects!! maybe hold actual objects and not pointers in
  //  the class? Not doing that for now, as copy constructors / assignment operators
  //  for cylinder and torus not operational.
  if (new_scale < _min_scale)
    _scale = _min_scale;
  else
    _scale = new_scale;

  // %TODO: figure out why I can't delete these and not leak memory :'(
  // (problem with the destructor, maybe?)
  _collision_ptx = new Collision_Object_Cylinder( _id + "+tx", _scale/20.0, _scale );
  _collision_pty = new Collision_Object_Cylinder( _id + "+ty", _scale/20.0, _scale );
  _collision_ptz = new Collision_Object_Cylinder( _id + "+tz", _scale/20.0, _scale );
  _collision_mtx = new Collision_Object_Cylinder( _id + "-tx", _scale/20.0, _scale );
  _collision_mty = new Collision_Object_Cylinder( _id + "-ty", _scale/20.0, _scale );
  _collision_mtz = new Collision_Object_Cylinder( _id + "-tz", _scale/20.0, _scale );
  _collision_rr = new Collision_Object_Torus( _id + "-rr", _scale/1.50, _scale/40.0 );
  _collision_rp = new Collision_Object_Torus( _id + "-rp", _scale/1.50, _scale/40.0 );
  _collision_ry = new Collision_Object_Torus( _id + "-ry", _scale/1.50, _scale/40.0 );

  // shift into correct place
  set_transform( position, orientation );
  return;
}

void
Collision_Object_Container_Coordinate_Axis::
set_transform( const Frame& transform, double new_scale ){
  // update scale
  if (new_scale < _min_scale)
    _scale = _min_scale;
  else
    _scale = new_scale;
  
  // %TODO: figure out why I can't delete these and not leak memory :'(
  // (problem with the destructor, maybe?)
  _collision_ptx = new Collision_Object_Cylinder( _id + "+tx", _scale/20.0, _scale );
  _collision_pty = new Collision_Object_Cylinder( _id + "+ty", _scale/20.0, _scale );
  _collision_ptz = new Collision_Object_Cylinder( _id + "+tz", _scale/20.0, _scale );
  _collision_mtx = new Collision_Object_Cylinder( _id + "-tx", _scale/20.0, _scale );
  _collision_mty = new Collision_Object_Cylinder( _id + "-ty", _scale/20.0, _scale );
  _collision_mtz = new Collision_Object_Cylinder( _id + "-tz", _scale/20.0, _scale );
  _collision_rr = new Collision_Object_Torus( _id + "-rr", _scale/1.50, _scale/40.0 );
  _collision_rp = new Collision_Object_Torus( _id + "-rp", _scale/1.50, _scale/40.0 );
  _collision_ry = new Collision_Object_Torus( _id + "-ry", _scale/1.50, _scale/40.0 );

  // shift into correct place
  Frame origin = transform * _offset;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qs = 0.0;
  origin.M.GetQuaternion( qx, qy, qz, qs );
  Vector3f position(origin.p.x(), origin.p.y(), origin.p.z());
  Vector4f orientation(qx, qy, qz, qs);
 // cout << "P: " << position << "|| O: " << orientation << "||\n";
  set_transform( position, orientation );
  return;
}

static Vector4f
vector4f_from_vector3f(Vector3f le3f){
  return Vector4f(le3f[0], le3f[1], le3f[2], 0.0 );
}

void
Collision_Object_Container_Coordinate_Axis::
set_transform( const Vector3f position,
                const Vector4f orientation ){
  // shift into correct place
  assert(_collision_ptx && _collision_pty && collision_ptz &&
         _collision_mtx && _collision_mty && collision_mtz &&
         _collision_rr && _collision_rp && collision_ry );

  Quaternion <float, AutoAlign> ori(orientation[1], -orientation[2], orientation[3], orientation[0]);
  ori = ori.inverse();
  Quaternion <float, AutoAlign> orix(AngleAxis<float>(M_PI/2, Vector3f(1.0, 0.0, 0.0) ));
  Quaternion <float, AutoAlign> oriy(AngleAxis<float>(M_PI/2, Vector3f(0.0, 1.0, 0.0) ));
  //Quaternion <float, AutoAlign> oriz(AngleAxis<float>(M_PI/2, Vector3f(0.0, 0.0, 1.0) ));

  _collision_ptx->set_transform( position + ori._transformVector(Vector3f(-_scale/2.0, 0.0, 0.0)), (ori*oriy).coeffs());
  _collision_mty->set_transform( position + ori._transformVector(Vector3f(0.0, -_scale/2.0, 0.0)), (ori*orix).coeffs());
  _collision_mtz->set_transform( position + ori._transformVector(Vector3f(0.0, 0.0, _scale/2.0)), (ori).coeffs());
  _collision_mtx->set_transform( position - ori._transformVector(Vector3f(-_scale/2.0, 0.0, 0.0)), (ori*oriy.inverse()).coeffs());
  _collision_pty->set_transform( position - ori._transformVector(Vector3f(0.0, -_scale/2.0, 0.0)), (ori*orix.inverse()).coeffs());
  _collision_ptz->set_transform( position - ori._transformVector(Vector3f(0.0, 0.0, _scale/2.0)), (ori*oriy*oriy).coeffs());
  _collision_rr->set_transform( position, (ori*oriy).coeffs() );
  _collision_rp->set_transform( position, (ori*orix).coeffs() );
  _collision_ry->set_transform( position, (ori).coeffs() );

  // And save directional vectors; signs have been adjusted to make things work out
  //  and function correctly. Be careful about changing these, as they affect both
  //  translation and rotation...
  _xdir = (ori)._transformVector(Vector3f(-1.0, 0.0, 0.0));
  _ydir = (ori)._transformVector(Vector3f(0.0, 1.0, 0.0));
  _zdir = (ori)._transformVector(Vector3f(0.0, 0.0, -1.0));

  return;
}

void
Collision_Object_Container_Coordinate_Axis::
set_transform( const Frame& transform ){
  // shift into correct place
  Frame origin = transform * _offset;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qs = 0.0;
  origin.M.GetQuaternion( qx, qy, qz, qs );
  Vector3f position(origin.p.x(), origin.p.y(), origin.p.z());
  Vector4f orientation(qx, qy, qz, qs);
 // cout << "P: " << position << "|| O: " << orientation << "||\n";
  set_transform( position, orientation );
  return;
}

/***
 * get_axis_dir
 * Returns a normalized vector in the direction of the specified
 * axis. 0 (or X_AXIS) for x axis, up to 2 (Z_AXIS) for z axis.
 * Returns as a KDL vector. (This is employed primarily as a helper
 * in qt4_widget_opengl_authoring.)
 */
Vector
Collision_Object_Container_Coordinate_Axis::
get_axis_direction( axis_num_t axis_number ){
  switch (axis_number){
    case X_AXIS:
      return Vector(_xdir[0], _xdir[1], _xdir[2]);
      break;
    case Y_AXIS:
      return Vector(_ydir[0], _ydir[1], _ydir[2]);
      break;
    case Z_AXIS:
      return Vector(_zdir[0], _zdir[1], _zdir[2]);
      break;
    default:
      printf("Wtf did you call get_axis_dir with? It didn't make sense.\n");
      exit(1);
      break; 
  }
}

void
Collision_Object_Container_Coordinate_Axis::
add_to_collision_detector( Collision_Detector * detector ){
  if (_draw_translation_axes){
    detector->add_collision_object(_collision_ptx);
    detector->add_collision_object(_collision_pty);
    detector->add_collision_object(_collision_ptz);
    detector->add_collision_object(_collision_mtx);
    detector->add_collision_object(_collision_mty);
    detector->add_collision_object(_collision_mtz);
  }
  if (_draw_rotation_axes){
    detector->add_collision_object(_collision_rr);
    detector->add_collision_object(_collision_rp);
    detector->add_collision_object(_collision_ry);
  }
}

void
Collision_Object_Container_Coordinate_Axis::
remove_from_collision_detector( Collision_Detector * detector ){
  if (_draw_translation_axes){
    detector->clear_collision_object(_collision_ptx);
    detector->clear_collision_object(_collision_pty);
    detector->clear_collision_object(_collision_ptz);    
    detector->clear_collision_object(_collision_mtx);
    detector->clear_collision_object(_collision_mty);
    detector->clear_collision_object(_collision_mtz);
  }
  if (_draw_rotation_axes){
    detector->clear_collision_object(_collision_rr);
    detector->clear_collision_object(_collision_rp);
    detector->clear_collision_object(_collision_ry);
  }
}

// not correct yet; should be fixed up to suite this container class
namespace collision {
  ostream&
  operator<<( ostream& out,
              const Collision_Object_Container_Coordinate_Axis& other ){
    out << "id:{" << other.id().c_str() << "} ";
    out << "bt_collision_objects[" << other.bt_collision_objects().size() << "]:{";
    for( unsigned int i = 0; i < other.bt_collision_objects().size(); i++ ){
      if( other.bt_collision_objects()[ i ]->getBroadphaseHandle() != NULL ){
        out << other.bt_collision_objects()[ i ]->getBroadphaseHandle()->getUid();
      } else {
        out << "N/A";
      }
      out << ":{pos:(" << other.bt_collision_objects()[ i ]->getWorldTransform().getOrigin().x() << "," << other.bt_collision_objects()[ i ]->getWorldTransform().getOrigin().y() << "," << other.bt_collision_objects()[ i ]->getWorldTransform().getOrigin().z() << "),(" << other.bt_collision_objects()[ i ]->getWorldTransform().getRotation().getX() << "," << other.bt_collision_objects()[ i ]->getWorldTransform().getRotation().getY() << "," << other.bt_collision_objects()[ i ]->getWorldTransform().getRotation().getZ() << "," << other.bt_collision_objects()[ i ]->getWorldTransform().getRotation().getW() << ")}";
      if( i != ( other.bt_collision_objects().size() - 1 ) ){
        out << ",";
      }
    }
    out << "}";
    return out;
  }
}
