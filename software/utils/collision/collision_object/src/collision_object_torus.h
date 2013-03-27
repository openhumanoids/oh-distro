#ifndef COLLISION_COLLISION_OBJECT_TORUS_H
#define COLLISION_COLLISION_OBJECT_TORUS_H

#include <iostream>
#include <string>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>

#include <collision/collision_object.h>

namespace collision {
  class Collision_Object_Torus : public Collision_Object {
  public:
    Collision_Object_Torus( std::string id = "N/A", double majorRadius = 1.0, double minorRadius = 0.1, Eigen::Vector3f position = Eigen::Vector3f(), Eigen::Vector4f orientation = Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) );
    ~Collision_Object_Torus();

    virtual Eigen::Vector3f position( void )const ;
    virtual Eigen::Vector4f orientation( void )const;
    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );
    virtual void set_transform( const KDL::Frame& transform );

    virtual std::vector< btCollisionObject* > bt_collision_objects( void );    
    virtual std::vector< const btCollisionObject* > bt_collision_objects( void )const;

  protected:
    btCollisionObject       _bt_collision_object;
    btCompoundShape         _bt_torus_shape;
    btCylinderShape         _bt_cylinder_shape_1;
    btCylinderShape         _bt_cylinder_shape_2;
    btCylinderShape         _bt_cylinder_shape_3;
    btCylinderShape         _bt_cylinder_shape_4;
    btCylinderShape         _bt_cylinder_shape_5;
    btCylinderShape         _bt_cylinder_shape_6;
    btCylinderShape         _bt_cylinder_shape_7;
    btCylinderShape         _bt_cylinder_shape_8;
    btCylinderShape         _bt_cylinder_shape_9;
    btCylinderShape         _bt_cylinder_shape_10;
    btCylinderShape         _bt_cylinder_shape_11;
    btCylinderShape         _bt_cylinder_shape_12;
    btCylinderShape         _bt_cylinder_shape_13;
    btCylinderShape         _bt_cylinder_shape_14;
    btCylinderShape         _bt_cylinder_shape_15;
    btCylinderShape         _bt_cylinder_shape_16;
  private:

  };
}

#endif /* COLLISION_COLLISION_OBJECT_TORUS_H */
