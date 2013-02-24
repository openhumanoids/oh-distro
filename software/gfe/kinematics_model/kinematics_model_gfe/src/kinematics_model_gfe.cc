#include <fstream>
#include <kdl_parser/kdl_parser.hpp>

#include <path_util/path_util.h>
#include <kinematics_model/kinematics_model.h>
#include <kinematics_model/kinematics_model_gfe.h>

using namespace std;
using namespace boost;
using namespace urdf;
using namespace KDL;
using namespace kdl_parser;
using namespace drc;
using namespace state;
using namespace kinematics_model;

/** 
 * Kinematics_Model
 * class constructor
 */
Kinematics_Model_GFE::
Kinematics_Model_GFE() : _model(),
                          _tree(),
                          _fk_solver( NULL ),
                          _world_to_body(),
                          _joint_frames(){
  if( !load_urdf( getModelsPath() + string( "/mit_gazebo_models/mit_robot/model.urdf" ) ) ){
    cout << "could not load urdf" << endl;
  }
}

/** 
 * Kinematics_Model
 * class constructor that loads from an xml file
 */
Kinematics_Model_GFE::
Kinematics_Model_GFE( string urdfFilename ) : _model(),
                                            _tree(),
                                            _fk_solver( NULL ),
                                            _world_to_body(),
                                            _joint_frames(){
  if( !load_urdf( getModelsPath() + urdfFilename ) ) {
    cout << "could not load urdf " << endl;
  }
}

/** 
 * Kinematics_Model_GFE
 * copy constructor
 */
Kinematics_Model_GFE::
Kinematics_Model_GFE( const Kinematics_Model_GFE& other ) : _model( other._model ),
                                                            _tree( other._tree),
                                                            _fk_solver( new TreeFkSolverPos_recursive( _tree ) ),
                                                            _world_to_body( other._world_to_body ),
                                                            _joint_frames( other._joint_frames ){
}

/**
 * ~Kinematics_Model_GFE
 * class destructor
 */
Kinematics_Model_GFE::
~Kinematics_Model_GFE(){
  if( _fk_solver != NULL ){
    delete _fk_solver;
    _fk_solver = NULL;
  } 
}

/**
 * load_xml_string
 * loads a model from a xml string
 */
bool
Kinematics_Model_GFE::
load_xml_string( string xmlString ){
  if( !_model.initString( xmlString ) ){
    return false;
  }
  if( !treeFromString( xmlString, _tree ) ){
    return false;
  }

  _fk_solver = new TreeFkSolverPos_recursive( _tree );

  return true;
}

/**
 * load_urdf
 * loads a model from a urdf file
 */
bool
Kinematics_Model_GFE::
load_urdf( string urdfFilename ){
  string xml_string;
  fstream xml_file( urdfFilename.c_str(), fstream::in );
  if( xml_file.is_open() ){
    while( xml_file.good() ){
      string line;
      getline( xml_file, line );
      xml_string += ( line + "\n" );
    }
    xml_file.close();
  } else {
    return false;
  }
  
  return load_xml_string( xml_string );
}

/**
 * set
 * updates the robot state in the kinematics model
 */
void
Kinematics_Model_GFE::
set( const drc::robot_state_t& robotState ){
  Kinematics_Model::drc_position_3d_t_to_kdl_frame( robotState.origin_position, _world_to_body );
 
  _joint_frames.clear();
  _joint_frames.insert( make_pair( _model.getRoot()->name, KDL::Frame::Identity() ) );

  map< string, double > joint_angles;
  for( unsigned int i = 0; i < robotState.num_joints; i++ ){
    joint_angles.insert( make_pair( robotState.joint_name[ i ], robotState.joint_position[ i ] ) );
  }

  _update_joint( joint_angles, _world_to_body, _tree.getRootSegment() );

  for( map< string, Frame >::iterator it = _joint_frames.begin(); it != _joint_frames.end(); it++ ){
    it->second = _world_to_body * it->second;
  }

  return;
}

void
Kinematics_Model_GFE::
set( State_GFE& stateGFE ){
  _joint_frames.clear();
  _joint_frames.insert( make_pair( _model.getRoot()->name, KDL::Frame::Identity() ) );
  
  map< string, double > joint_angles = stateGFE.joint_angles();
  _update_joint( joint_angles, _world_to_body, _tree.getRootSegment() );

  for( map< string, Frame >::iterator it = _joint_frames.begin(); it != _joint_frames.end(); it++ ){
    it->second = _world_to_body * it->second;
  }
  return;
}

/**
 * model
 * returns a const reference to the urdf::Model
 */
const Model&
Kinematics_Model_GFE::
model( void )const{
  return _model;
}

/**
 * tree
 * returns a const reference to the KDL::Tree
 */
const Tree&
Kinematics_Model_GFE::
tree( void )const{
  return _tree;
}

/**
 * link
 * returns a KDL::Frame with the name linkName
 */
Frame
Kinematics_Model_GFE::
link( string linkName )const{
  map< string, Frame >::const_iterator joint_frame_iterator = _joint_frames.find( linkName );
  if( joint_frame_iterator != _joint_frames.end() ){
    return joint_frame_iterator->second;
  } else {
    cout << "could not find frame " << linkName << endl;
    return Frame();
  }
}

/**
 * _update_joint
 * recursively updates the KDL::Frame map
 */
void
Kinematics_Model_GFE::
_update_joint( map< string, double >& jointAngles,
                    Frame& parentFrame,
                    const SegmentMap::const_iterator jointIterator ){
  KDL::Frame joint_frame;

  if( jointIterator->second.segment.getJoint().getType() != KDL::Joint::None ){
    map< string, double >::const_iterator this_joint = jointAngles.find( jointIterator->second.segment.getJoint().getName() );
    if( this_joint == jointAngles.end() ){
      cout << "could not find joint " << jointIterator->second.segment.getJoint().getName() << endl;
    } else {
      joint_frame = parentFrame * jointIterator->second.segment.pose( this_joint->second );
    }
    _joint_frames.insert( make_pair( jointIterator->first, joint_frame ) );
  }

  for( vector< SegmentMap::const_iterator >::const_iterator child = jointIterator->second.children.begin(); child != jointIterator->second.children.end(); child++ ){
    _update_joint( jointAngles, joint_frame, *child );
  }
  return;
}

/**
 * operator<<
 * ostream operator
 */
namespace kinematics_model {
  ostream&
  operator<<( ostream& out,
              const Kinematics_Model_GFE& other ){
    return out;
  }
} 
