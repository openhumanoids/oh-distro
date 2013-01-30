#ifndef COLLISION_OBJECT_MANIPULATOR_H
#define COLLISION_OBJECT_MANIPULATOR_H

#include <collision/collision_object_cylinder.h>
#include <collision/collision_object_sphere.h>
#include <collision/collision_object_box.h>

#include <affordance/ManipulatorState.h>

namespace affordance
{
  class Collision_Object_Manipulator : public collision::Collision_Object
    {
      //------------------fields
    private:
      /**underlying manipulator state*/
      affordance::ManipulatorStateConstPtr _manipulator;
      
      //---collection of collision objects
      std::vector<collision::Collision_Object*> _cObjs;
      
      //-------------constructor/destructor
    public:
      Collision_Object_Manipulator(affordance::ManipulatorStateConstPtr manipulator);
      virtual ~Collision_Object_Manipulator();
      
      //Collision_Object interface
      virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );
      virtual Eigen::Vector3f position( void ) const;
      virtual Eigen::Vector4f orientation( void ) const;
      virtual std::vector< btCollisionObject* > bt_collision_objects( void );    
      virtual std::vector< const btCollisionObject* > bt_collision_objects( void )const;
      
      //---------useful methods
    public:
      static bool isSupported(affordance::ManipulatorStateConstPtr m); //check if we support collision objects for this type of manipulator
      
    }; //class Collision_Object_Manipulator   
  
} //namespace affordance


#endif // COLLISION_OBJECT_MANIPULATOR
