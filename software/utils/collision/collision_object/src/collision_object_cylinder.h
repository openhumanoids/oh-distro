#ifndef COLLISION_COLLISION_OBJECT_CYLINDER_H
#define COLLISION_COLLISION_OBJECT_CYLINDER_H

#include <iostream>
#include <string>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>

#include <collision/collision_object.h>

namespace collision {
  class Collision_Object_Cylinder : public Collision_Object {
  public:
    Collision_Object_Cylinder( std::string id = "N/A", double radius = 1.0, double height = 0.1, Eigen::Vector3f position = Eigen::Vector3f(), Eigen::Vector4f orientation = Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) );
    Collision_Object_Cylinder( std::string id, double radius, double height, const KDL::Frame& offset, const KDL::Frame& transform = KDL::Frame::Identity() );
    ~Collision_Object_Cylinder();

    virtual Eigen::Vector3f position( void )const ;
    virtual Eigen::Vector4f orientation( void )const;

    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );
    virtual void set_transform( const KDL::Frame& transform );  

    virtual std::vector< btCollisionObject* > bt_collision_objects( void );    
    virtual std::vector< const btCollisionObject* > bt_collision_objects( void )const;

  protected:
    btCollisionObject       _bt_collision_object;
    btCylinderShapeZ        _bt_cylinder_shape;
  private:

  };
}

#endif /* COLLISION_COLLISION_OBJECT_CYLINDER_H */
