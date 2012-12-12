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
using namespace kinematics_model;

/** 
 * Kinematics_Model
 * class constructor
 */
Kinematics_Model_GFE::
Kinematics_Model_GFE() : _urdf_filename( getModelsPath() + string( "/mit_gazebo_models/mit_robot_PnC/model.sdf" ) ),
                          _model(),
                          _tree(),
                          _fk_solver( NULL ),
                          _world_to_body(),
                          _joint_angles(){
  if( !load_urdf( _urdf_filename ) ){
    cout << "could not load urdf" << endl;
  }
}

/** 
 * Kinematics_Model
 * class constructor that takes the urdf filename
 */
Kinematics_Model_GFE::
Kinematics_Model_GFE( std::string urdfFilename ) : _urdf_filename( urdfFilename ),
                                                    _model(),
                                                    _tree(),
                                                    _fk_solver( NULL ),
                                                    _world_to_body(),
                                                    _joint_angles(){
  if( !load_urdf( _urdf_filename ) ){
    cout << "could not load urdf" << endl;
  } 
}

/** 
 * Kinematics_Model_GFE
 * copy constructor
 */
Kinematics_Model_GFE::
Kinematics_Model_GFE( const Kinematics_Model_GFE& other ) : _urdf_filename( other._urdf_filename ),
                                                            _model(),
                                                            _tree(),
                                                            _fk_solver( NULL ),
                                                            _world_to_body(),
                                                            _joint_angles(){
  if( !load_urdf( _urdf_filename ) ){
    cout << "could not load urdf" << endl;
  }
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
 * load_urdf
 * loads a model from a urdf file
 */
bool
Kinematics_Model_GFE::
load_urdf( std::string urdfFilename ){
  string xml_string;
  fstream xml_file( urdfFilename.c_str(), fstream::in );
  if( xml_file.is_open() ){
    while( xml_file.good() ){
      string line;
      getline( xml_file, line );
      xml_string += ( line + "\n" );
    }
    xml_file.close();
    cout << "file " << urdfFilename.c_str() << " parsed successfully" << endl;
  }

  if( !_model.initString( xml_string ) ){
    cout << "could not initialize string" << endl;
    return false;
  }
  if( !treeFromString( xml_string, _tree ) ){
    cout << "could not load tree from string" << endl;
    return false;
  }
  
  _fk_solver = new TreeFkSolverPos_recursive( _tree );

  return true;
}

/**
 * set
 * updates the robot state in the kinematics model
 */
void
Kinematics_Model_GFE::
set( drc::robot_state_t& robotState ){
  Kinematics_Model::drc_position_3d_t_to_kdl_frame( robotState.origin_position, _world_to_body );
  
  _joint_angles.resize( robotState.num_joints );
  for( unsigned int i = 0; i < robotState.num_joints; i++ ){
    _joint_angles( i ) = robotState.joint_position[ i ];
  }

  return;
}

/**
 * urdf_filename
 * returns a copy of the urdf filename
 */
string
Kinematics_Model_GFE::
urdf_filename( void )const{
  return _urdf_filename;
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
 * returns a KDL::Frame with the name linkID
 */
Frame
Kinematics_Model_GFE::
link( string linkID )const{
  Frame frame;
  if( _fk_solver != NULL ){
    Frame body_to_link;
    _fk_solver->JntToCart( _joint_angles, body_to_link, linkID );
    frame = _world_to_body * body_to_link;
  }
  return frame;
}

/**
 * operator<<
 * ostream operator
 */
namespace kinematics_model {
  ostream&
  operator<<( ostream& out,
              const Kinematics_Model_GFE& other ){
    out << "joints[" << other.tree().getNrOfJoints() << "]:{";
    out << "} ";
    out << "links[" << other.tree().getNrOfSegments() << "]:{";
    out << "} ";
    return out;
  }
} 
