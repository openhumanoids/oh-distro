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

State_GFE::
State_GFE() : _id( "gfe" ),
              _time( 0 ),
              _pose() {

}

State_GFE::
~State_GFE() {

}

State_GFE::
State_GFE( const State_GFE& other ) : _id( other._id ),
                                      _time( other._time ),
                                      _pose( other._pose ){
  
}

State_GFE&
State_GFE::
operator=( const State_GFE& other ){
  _id = other._id;
  _time = other._time;
  _pose = other._pose;
  return ( *this );
}

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

  Model model;
  if( !model.initString( xml_string ) ){
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

void
State_GFE::
to_lcm( robot_state_t& robotState )const{
  robotState.origin_position.translation.x = _pose.p[ 0 ];
  robotState.origin_position.translation.y = _pose.p[ 1 ];
  robotState.origin_position.translation.z = _pose.p[ 2 ];
  _pose.M.GetQuaternion( robotState.origin_position.rotation.x, robotState.origin_position.rotation.y, robotState.origin_position.rotation.z, robotState.origin_position.rotation.w );
  return;
}

void
State_GFE::
set_id( string id ){
  _id = id;
  return;
}

void
State_GFE::
set_time( unsigned long long time ){
  _time = time;
  for ( std::map< std::string, State_GFE_Joint >::iterator it = _joints.begin(); it != _joints.end(); it++ ){
    it->second.set_time( time );
  }
  return;
}

void
State_GFE::
set_pose( const Frame& pose ){
  _pose = pose;
  return;
}

string
State_GFE::
id( void )const{
  return _id;
}

unsigned long long
State_GFE::
time( void )const{
  return _time;
}

Frame
State_GFE::
pose( void )const{
  return _pose;
}

map< string, State_GFE_Joint >
State_GFE::
joints( void )const{
  return _joints;
}

map< string, double >
State_GFE::
joint_angles( void )const{
  map< string, double > joint_angles;
  for( map< string, State_GFE_Joint >::const_iterator it = _joints.begin(); it != _joints.end(); it++ ){
    joint_angles.insert( make_pair( it->first, it->second.position() ) );
  }   
  return joint_angles;
}

State_GFE_Joint&
State_GFE::
joint( string id ){
  map< string, State_GFE_Joint >::iterator it = _joints.find( id );
  return  it->second;
}

const State_GFE_Joint&
State_GFE::
joint( string id )const{
  map< string, State_GFE_Joint >::const_iterator it = _joints.find( id );
  return it->second;
}

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
