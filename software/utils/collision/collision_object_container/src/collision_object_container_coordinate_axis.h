/**
 * @file collision_object_container_coordinate_axis.h
 * @author Gregory Izatt
 * @namespace collision
 * 
 * @section DESCRIPTION
 * 
 * This class is a container for collision objects and not
 * a legitimate collision object itself! Don't try to add this
 * to a collision detector or behavior is undefined.
 * 
 * A class used to describe a set of coordinate axes for 
 * manipulation (parallel of the opengl_object_coordinate_axis
 * class). The constructor accepts a pair of boolean values
 * representing whether this collision model includes the
 * translation / rotation axes (or both). This model actually
 * represents a collection of other collision bodies with id's
 * indicating what translation/rotation axis they are part of.
 * IDs are assigned as the passed ID plus specifiers -tx, -ty, -tz,
 * -rr, -ry, -rz for translation axis and rotation axis respec.
 */

#ifndef COLLISION_COLLISION_OBJECT_CONTAINER_COORDINATE_AXIS_H
#define COLLISION_COLLISION_OBJECT_CONTAINER_COORDINATE_AXIS_H

#include <iostream>
#include <string>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>
#include <kdl/tree.hpp>

#include <collision/collision_object.h>
#include <collision/collision_object_cylinder.h>
#include <collision/collision_object_torus.h>
#include <collision/collision_detector.h>

namespace collision {

  enum axis_num_t{
    X_AXIS = 0,
    Y_AXIS = 1,
    Z_AXIS = 2
  };

  class Collision_Object_Container_Coordinate_Axis : public Collision_Object {
  public:
    Collision_Object_Container_Coordinate_Axis( bool drawTranslationAxes, bool drawRotationAxes,
      std::string id, Eigen::Vector3f dims = Eigen::Vector3f( 1.0, 1.0, 1.0 ), 
      Eigen::Vector3f position = Eigen::Vector3f( 0.0, 0.0, 0.0 ), 
      Eigen::Vector4f orientation = Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ),
      double min_scale = 0.3 );
    /*
    Collision_Object_Coordinate_Axis( bool drawTranslationAxes, bool drawRotationAxes,
      std::string id, Eigen::Vector3f dims, const KDL::Frame& offset, 
      const KDL::Frame& transform = KDL::Frame::Identity() );
    */
    //Collision_Object_Container_Coordinate_Axis( const Collision_Object_Container_Coordinate_Axis& other );
    ~Collision_Object_Container_Coordinate_Axis();

    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation, double new_scale );
    virtual void set_transform( const KDL::Frame& transform, double new_scale );
    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation);
    virtual void set_transform( const KDL::Frame& transform );
    virtual KDL::Vector get_axis_direction( axis_num_t axis_number );

    void add_to_collision_detector( Collision_Detector * detector );
    void remove_from_collision_detector( Collision_Detector * detector );
    
  protected:
    
    // note: this could be made more accurate by using specialized "arrow"
    // collision objects instead of cylinders for x/y/z translation axes,
    // since that's what they're rendered as (i.e. they have arrow heads)
    Collision_Object_Cylinder *_collision_ptx;
    Collision_Object_Cylinder *_collision_pty;
    Collision_Object_Cylinder *_collision_ptz;
    Collision_Object_Cylinder *_collision_mtx;
    Collision_Object_Cylinder *_collision_mty;
    Collision_Object_Cylinder *_collision_mtz;
    Collision_Object_Torus    *_collision_rr;    
    Collision_Object_Torus    *_collision_rp;
    Collision_Object_Torus    *_collision_ry;

    // Save normalized vectors in the cardinal directions for utility
    Eigen::Vector3f _xdir;
    Eigen::Vector3f _ydir;
    Eigen::Vector3f _zdir;

    bool              _draw_translation_axes;
    bool              _draw_rotation_axes;
    double            _scale;
    double            _min_scale;
  private:

  };
  std::ostream& operator<<( std::ostream& out, const Collision_Object_Container_Coordinate_Axis& other );
}

#endif /* COLLISION_COLLISION_OBJECT_CONTAINER_COORDINATE_AXIS_H */
