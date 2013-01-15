#include <vector>
#include <fstream>
#include <boost/shared_ptr.hpp>

#include <path_util/path_util.h>
#include <urdf_interface/joint.h>

#include <state/state_gfe.h>

using namespace std;
using namespace boost;
using namespace KDL;
using namespace urdf;
using namespace drc;
using namespace state;


/**
 * State_GFE
 * class constructor
 */
State_GFE::
State_GFE() : _id( "gfe" ),
              _time( 0 ),
              _pose() {

}

/**
 * ~State_GFE
 * class destructor
 */
State_GFE::
~State_GFE() {

}

/** 
 * State_GFE
 * class copy constructor
 */
State_GFE::
State_GFE( const State_GFE& other ) : _id( other._id ),
                                      _time( other._time ),
                                      _pose( other._pose ){
  
}

/**
 * operator=
 * class assignment operator
 */
State_GFE&
State_GFE::
operator=( const State_GFE& other ){
  _id = other._id;
  _time = other._time;
  _pose = other._pose;
  return ( *this );
}

/**
 * from_xml_string
 * constructs the joints from an xml string representing the contents of a urdf file
 */
bool
State_GFE::
from_xml_string( string xmlString ){
  Model model;
  if( !model.initString( xmlString ) ){
    cout << "could not initialize xml_string" << endl;
    return false;
  }
 
  _joints.clear();
  for ( std::map< std::string, boost::shared_ptr< urdf::Joint > >::const_iterator it = model.joints_.begin(); it != model.joints_.end(); it++ ){
    State_GFE_Joint joint( it->first );
    _joints.insert( make_pair( joint.id(), joint ) );
  }

  return true;
}

/**
 * from_urdf
 * constructs the joints from an urdf file
 */
bool
State_GFE::
from_urdf( string urdfFilename ){
  string filename = getModelsPath() + urdfFilename;
  string xml_string;
  ifstream xml_file( filename.c_str(), fstream::in );
  if( xml_file.is_open() ){
    while( xml_file.good() ){
      string line;
      getline( xml_file, line );
      xml_string += ( line + "\n" );
    }
    xml_file.close();
  } else {
    cout << "could not read urdf: " << urdfFilename.c_str() << endl;
    return false;
  }
  return from_xml_string( xml_string );
}

/**
 * from_lcm
 * constructs the joints from a drc::robot_state_t message
 */
bool
State_GFE::
from_lcm( const robot_state_t& robotState ){
  _joints.clear();

  _pose.p[ 0 ] = robotState.origin_position.translation.x;
  _pose.p[ 1 ] = robotState.origin_position.translation.y;
  _pose.p[ 2 ] = robotState.origin_position.translation.z;
  _pose.M = KDL::Rotation::Quaternion( robotState.origin_position.rotation.x, robotState.origin_position.rotation.y, robotState.origin_position.rotation.z, robotState.origin_position.rotation.w );
  for( unsigned int i = 0; i < robotState.num_joints; i++ ){
    State_GFE_Joint joint( robotState.joint_name[ i ],
                            robotState.utime,
                              robotState.joint_position[ i ],
                            robotState.joint_velocity[ i ],
                            robotState.measured_effort[ i ] );
    _joints.insert( make_pair( joint.id(), joint ) );
  }
  return true;
}

/**
 * to_lcm
 * constructs an drc::robot_state_t message from the contents of the State_GF#
 */
void
State_GFE::
to_lcm( robot_state_t& robotState )const{
  robotState.origin_position.translation.x = _pose.p[ 0 ];
  robotState.origin_position.translation.y = _pose.p[ 1 ];
  robotState.origin_position.translation.z = _pose.p[ 2 ];
  _pose.M.GetQuaternion( robotState.origin_position.rotation.x, robotState.origin_position.rotation.y, robotState.origin_position.rotation.z, robotState.origin_position.rotation.w );
  return;
}

/** 
 * set_id
 * sets the id of the state
 */
void
State_GFE::
set_id( string id ){
  _id = id;
  return;
}

/**
 * set_time
 * sets the time of the state 
 */
void
State_GFE::
set_time( unsigned long long time ){
  _time = time;
  for ( std::map< std::string, State_GFE_Joint >::iterator it = _joints.begin(); it != _joints.end(); it++ ){
    it->second.set_time( time );
  }
  return;
}

/**
 * set_pose
 * sets the pose of the state
 */
void
State_GFE::
set_pose( const Frame& pose ){
  _pose = pose;
  return;
}

/** 
 * id
 * returns the id of the state 
 */
string
State_GFE::
id( void )const{
  return _id;
}

/**
 * time
 * returns the time of the state 
 */
unsigned long long
State_GFE::
time( void )const{
  return _time;
}

/**
 * pose
 * returns the pose of the state 
 */
Frame
State_GFE::
pose( void )const{
  return _pose;
}

/**
 * joints
 * returns a map of joints 
 */
map< string, State_GFE_Joint >
State_GFE::
joints( void )const{
  return _joints;
}

/**
 * joint_angle
 * returns a map of joint angles
 */
map< string, double >
State_GFE::
joint_angles( void )const{
  map< string, double > joint_angles;
  for( map< string, State_GFE_Joint >::const_iterator it = _joints.begin(); it != _joints.end(); it++ ){
    joint_angles.insert( make_pair( it->first, it->second.position() ) );
  }   
  return joint_angles;
}

/**
 * joint
 * returns a reference to a specific joint based on the id argument 
 */
State_GFE_Joint&
State_GFE::
joint( string id ){
  map< string, State_GFE_Joint >::iterator it = _joints.find( id );
  return  it->second;
}

/**
 * joint
 * returns a const reference to a specific joint based on the id argument
 */
const State_GFE_Joint&
State_GFE::
joint( string id )const{
  map< string, State_GFE_Joint >::const_iterator it = _joints.find( id );
  return it->second;
}

/**
 * operator<<
 * ostream operator
 */
namespace state {
  ostream&
  operator<<( ostream& out,
              const State_GFE& other ){
    out << "id:{" << other.id() << "} ";
    out << "time:{" << other.time() << "} ";
    Frame pose = other.pose();
    out << "r:{" << pose.p[0] << "," << pose.p[1] << "," << pose.p[2] << "} ";
    double qx, qy, qz, qw;
    pose.M.GetQuaternion( qx, qy, qz, qw );
    out << "q:{(" << qx << "," << qy << "," << qz << ")," << qw << "} ";
    map< string, State_GFE_Joint > joints = other.joints();
    out << "jid[" << joints.size() << "]:{";
    for ( std::map< std::string, State_GFE_Joint >::const_iterator it = joints.begin(); it != joints.end(); it++ ){
      out << it->second.id();
      if( it != joints.end() ){
        out << ",";
      }
    }
    out << "} jp[" << joints.size() << "]:{";
    for ( std::map< std::string, State_GFE_Joint >::const_iterator it = joints.begin(); it != joints.end(); it++ ){
      out << it->second.position();
      if( it != joints.end() ){
        out << ",";
      }
    }
    out << "}";
    return out;
  }
}
