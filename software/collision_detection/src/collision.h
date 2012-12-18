/**
 * @file collision_object.h
 * @author Thomas Howard
 * @namespace collision_detection
 * 
 * @section DESCRIPTION
 * 
 * A class used to generally describe attributes of
 *   a collision object used by the Collision_Detector
 *   class. 
 */

#ifndef COLLISION_DETECTION_COLLISION_H
#define COLLISION_DETECTION_COLLISION_H

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <btBulletCollisionCommon.h>

namespace collision_detection {
  class Collision {
  public:
    Collision( std::string firstId = "N/A", std::string secondId = "N/A" );
    Collision( const Collision& other );
    Collision& operator=( const Collision& other );
    ~Collision();

    void set_first_id( std::string firstId );
    void set_second_id( std::string secondId );
    void add_contact_point( Eigen::Vector3f contactPoint );
    void clear_contact_points( void );

    std::string first_id( void )const;
    std::string second_id( void )const;
    std::vector< Eigen::Vector3f > contact_points( void )const;

  protected:
    std::string _first_id;
    std::string _second_id;
    std::vector< Eigen::Vector3f > _contact_points;
  private:

  };
  std::ostream& operator<<( std::ostream& out, const Collision& other );
}

#endif /* COLLISION_DETECTION_COLLISION_H */
