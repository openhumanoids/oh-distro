#include <fstream>
#include <kdl_parser/kdl_parser.hpp>

#include <path_util/path_util.h>
#include <kinematics/kinematics_model.h>
#include <kinematics/kinematics_model_urdf.h>

using namespace std;
using namespace boost;
using namespace urdf;
using namespace KDL;
using namespace kdl_parser;
using namespace drc;
using namespace kinematics;

/** 
 * Kinematics_Model
 * class constructor
 */
Kinematics_Model_URDF::
Kinematics_Model_URDF() : _model(),
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
Kinematics_Model_URDF::
Kinematics_Model_URDF( string xmlString ) : _model(),
                                            _tree(),
                                            _fk_solver( NULL ),
                                            _world_to_body(),
                                            _joint_frames(){
  if( !load_xml_string( xmlString ) ){
    cout << "could not load xml string" << endl;
  } 
}

/** 
 * Kinematics_Model_URDF
 * copy constructor
 */
Kinematics_Model_URDF::
Kinematics_Model_URDF( const Kinematics_Model_URDF& other ) : _model( other._model ),
                                                            _tree( other._tree),
                                                            _fk_solver( new TreeFkSolverPos_recursive( _tree ) ),
                                                            _world_to_body( other._world_to_body ),
                                                            _joint_frames( other._joint_frames ){
}

/**
 * ~Kinematics_Model_URDF
 * class destructor
 */
Kinematics_Model_URDF::
~Kinematics_Model_URDF(){
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
Kinematics_Model_URDF::
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
Kinematics_Model_URDF::
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
 * model
 * returns a const reference to the urdf::Model
 */
const Model&
Kinematics_Model_URDF::
model( void )const{
  return _model;
}

/**
 * tree
 * returns a const reference to the KDL::Tree
 */
const Tree&
Kinematics_Model_URDF::
tree( void )const{
  return _tree;
}

/**
 * link
 * returns a KDL::Frame with the name linkName
 */
Frame
Kinematics_Model_URDF::
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
Kinematics_Model_URDF::
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
namespace kinematics {
  ostream&
  operator<<( ostream& out,
              const Kinematics_Model_URDF& other ){
    return out;
  }
} 
