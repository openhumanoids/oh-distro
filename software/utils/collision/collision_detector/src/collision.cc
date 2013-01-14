#include <collision/collision.h>

using namespace std;
using namespace Eigen;
using namespace collision;

/**
 * Collision
 * class constructor
 */
Collision::
Collision( string firstId,
            string secondId ) : _first_id( firstId ),
                                _second_id( secondId ),
                                _contact_points(){

}

/**
 * Collision 
 * copy constructor
 */
Collision::
Collision( const Collision& other ) : _first_id( other._first_id ),
                                      _second_id( other._second_id ),
                                      _contact_points( other._contact_points ){

}

/**
 * operator=
 * assignment operator
 */
Collision&
Collision::
operator=( const Collision& other ) {
  _first_id = other._first_id;
  _second_id = other._second_id;
  _contact_points = other._contact_points;
  return (*this);
}

/**
 * ~Collision
 * class destructor
 */
Collision::
~Collision(){

}

/**
 * set_first_id
 * sets the id of the first contact body
 */
void
Collision::
set_first_id( string firstId ){
  _first_id = firstId;
  return;
}

/** 
 * set_second_id
 * sets the id of the second contact body
 */
void
Collision::
set_second_id( string secondId ){
  _second_id = secondId;
  return;
}

/** 
 * add_contact_point
 * adds a contact point to the _contact_points vector
 */
void
Collision::
add_contact_point( Vector3f contactPoint ){
  _contact_points.push_back( contactPoint );
  return;
}

/**
 * clear_contact_points
 * clears the _contact_points vector
 */
void
Collision::
clear_contact_points( void ){
  _contact_points.clear();
}

/**
 * first_id
 * returns the first id
 */
string
Collision::
first_id( void )const{
  return _first_id;
}

/**
 * second_id
 * returns the second id
 */
string
Collision::
second_id( void )const{
  return _second_id;
}

/**
 * contact_points
 * returns a vector of contact points
 */
vector< Vector3f >
Collision::
contact_points( void )const{
  return _contact_points;
}

/**
 * operator<<
 * ostream operator
 */
namespace collision {
  ostream&
  operator<<( ostream& out,
              const Collision& other ){
    out << "first_id:{" << other.first_id().c_str() << "} ";
    out << "second_id:{" << other.second_id().c_str() << "} ";
    vector< Vector3f > contact_points = other.contact_points();
    out << "contact points[" << contact_points.size() << "]:{";
    for( unsigned int i = 0; i < contact_points.size(); i++ ){
      out << "(" << contact_points[ i ].x() << "," << contact_points[ i ].y() << "," << contact_points[ i ].z() << ")";
      if( i != ( contact_points.size() - 1 ) ){
        out << ",";
      }
    }
    out << "}";
    return out;
  }
}
