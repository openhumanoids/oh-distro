#include <fstream>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <state/state_gfe_arm.h>
#include <state/state_gfe_leg.h>
#include <path_util/path_util.h>
#include <kinematics/kinematics_model.h>
#include <kinematics/kinematics_model_gfe.h>
using namespace std;
using namespace boost;
using namespace urdf;
using namespace KDL;
using namespace kdl_parser;
using namespace drc;
using namespace state;
using namespace kinematics;

/** 
 * Kinematics_Model
 * class constructor
 */
Kinematics_Model_GFE::
Kinematics_Model_GFE( double eps,
                      unsigned int maxIterations ) : _model(),
                          _tree(),
                          _fk_solver( NULL ),
                          _iksolverpos_left_arm( NULL ),
                          _iksolverpos_right_arm( NULL ),
                          _iksolverpos_left_leg( NULL ),
                          _iksolverpos_right_leg( NULL ),
                          _world_to_body(),
                          _link_frames(),
                          _left_arm_chain(),
                          _right_arm_chain(),
                          _left_leg_chain(),
                          _right_leg_chain(),
                          _min_joint_limits_left_arm( 3 + NUM_STATE_GFE_ARM_JOINTS ),
                          _max_joint_limits_left_arm( 3 + NUM_STATE_GFE_ARM_JOINTS ),
                          _min_joint_limits_right_arm( 3 + NUM_STATE_GFE_ARM_JOINTS ),
                          _max_joint_limits_right_arm( 3 + NUM_STATE_GFE_ARM_JOINTS ),
                          _min_joint_limits_left_leg( NUM_STATE_GFE_LEG_JOINTS ),
                          _max_joint_limits_left_leg( NUM_STATE_GFE_LEG_JOINTS ),
                          _min_joint_limits_right_leg( NUM_STATE_GFE_LEG_JOINTS ),
                          _max_joint_limits_right_leg( NUM_STATE_GFE_LEG_JOINTS ){
  if( !load_urdf( getModelsPath() + string( "/mit_gazebo_models/mit_robot/model.urdf" ), eps, maxIterations ) ){
    cout << "could not load urdf" << endl;
  }
}

/** 
 * Kinematics_Model
 * class constructor that loads from an xml file
 */
Kinematics_Model_GFE::
Kinematics_Model_GFE( string xmlString, double eps, unsigned int maxIterations ) : _model(),
                                            _tree(),
                                            _fk_solver( NULL ),
                                            _iksolverpos_left_arm( NULL ),
                                            _iksolverpos_right_arm( NULL ),
                                            _iksolverpos_left_leg( NULL ),
                                            _iksolverpos_right_leg( NULL ),
                                            _world_to_body(),
                                            _link_frames(),
                                            _left_arm_chain(),
                                            _right_arm_chain(),
                                            _left_leg_chain(),
                                            _right_leg_chain(),
                                            _min_joint_limits_left_arm( 3 + NUM_STATE_GFE_ARM_JOINTS ),
                                            _max_joint_limits_left_arm( 3 + NUM_STATE_GFE_ARM_JOINTS ),
                                            _min_joint_limits_right_arm( 3 + NUM_STATE_GFE_ARM_JOINTS ),
                                            _max_joint_limits_right_arm( 3 + NUM_STATE_GFE_ARM_JOINTS ),
                                            _min_joint_limits_left_leg( NUM_STATE_GFE_LEG_JOINTS ),
                                            _max_joint_limits_left_leg( NUM_STATE_GFE_LEG_JOINTS ),
                                            _min_joint_limits_right_leg( NUM_STATE_GFE_LEG_JOINTS ),
                                            _max_joint_limits_right_leg( NUM_STATE_GFE_LEG_JOINTS ){
  if( !load_xml_string( xmlString, eps, maxIterations ) ) {
    cout << "could not load xml string " << endl;
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
                                                            _link_frames( other._link_frames ),
                                                            _left_arm_chain( other._left_arm_chain ),
                                                            _right_arm_chain( other._right_arm_chain ),
                                                            _left_leg_chain( other._left_leg_chain ),
                                                            _right_leg_chain( other._right_leg_chain ),
                                                            _min_joint_limits_left_arm( other._min_joint_limits_left_arm ),
                                                            _max_joint_limits_left_arm( other._max_joint_limits_left_arm ),
                                                            _min_joint_limits_right_arm( other._min_joint_limits_right_arm ),
                                                            _max_joint_limits_right_arm( other._max_joint_limits_right_arm ),
                                                            _min_joint_limits_left_leg( other._min_joint_limits_left_leg ),
                                                            _max_joint_limits_left_leg( other._max_joint_limits_left_leg ),
                                                            _min_joint_limits_right_leg( other._min_joint_limits_right_leg ),
                                                            _max_joint_limits_right_leg( other._max_joint_limits_right_leg ){
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
 * inverse_kinematics_left_arm
 * solves for the left arm state given a target pose
 */
bool
Kinematics_Model_GFE::
inverse_kinematics_left_arm( const State_GFE& robotState,
                              const Frame& pelvisToHandPose,
                              State_GFE& solution ){
  JntArray in_left_arm_angles( 3 + NUM_STATE_GFE_ARM_JOINTS );
  JntArray out_left_arm_angles( 3 + NUM_STATE_GFE_ARM_JOINTS );
 
  for( unsigned int i = 0; i < ( 3 + NUM_STATE_GFE_ARM_JOINTS ); i++ ){
    in_left_arm_angles( i ) = 0.5 * ( _min_joint_limits_left_arm( i ) + _max_joint_limits_left_arm( i ) );
  } 

  if( _iksolverpos_left_arm != NULL ){
    if( _iksolverpos_left_arm->CartToJnt( in_left_arm_angles, pelvisToHandPose, out_left_arm_angles ) < 0 ){
      return false;
    }
  } else {
    return false;
  }

  for( unsigned int i = 0; i < ( 3 + NUM_STATE_GFE_ARM_JOINTS ); i++ ){
    if( ( out_left_arm_angles( i ) < _min_joint_limits_left_arm( i ) ) || ( out_left_arm_angles( i ) > _max_joint_limits_left_arm( i )  ) ){
      return false;
    }
  }

  solution.joint( STATE_GFE_BACK_LBZ_JOINT ).set_position( out_left_arm_angles( 0 ) );
  solution.joint( STATE_GFE_BACK_MBY_JOINT ).set_position( out_left_arm_angles( 1 ) );
  solution.joint( STATE_GFE_BACK_UBX_JOINT ).set_position( out_left_arm_angles( 2 ) );
  for( unsigned int i = 0; i < NUM_STATE_GFE_ARM_JOINTS; i++ ){
    solution.left_arm().joint( ( state_gfe_arm_joint_t )( i ) ).set_position( out_left_arm_angles( 3 + i ) );
  }

  return true;
}

/**
 * inverse_kinematics_right_arm
 * solves for the right arm state given a target pose
 */
bool
Kinematics_Model_GFE::
inverse_kinematics_right_arm( const State_GFE& robotState,
                              const Frame& pelvisToHandPose,
                              State_GFE& solution ){
  JntArray in_right_arm_angles( 3 + NUM_STATE_GFE_ARM_JOINTS );
  JntArray out_right_arm_angles( 3 + NUM_STATE_GFE_ARM_JOINTS );

  for( unsigned int i = 0; i < ( 3 + NUM_STATE_GFE_ARM_JOINTS ); i++ ){
    in_right_arm_angles( i ) = 0.5 * ( _min_joint_limits_right_arm( i ) + _max_joint_limits_right_arm( i ) );
  }

  if( _iksolverpos_right_arm != NULL ){
    if( _iksolverpos_right_arm->CartToJnt( in_right_arm_angles, pelvisToHandPose, out_right_arm_angles ) < 0 ){
      return false;
    } 
  } else {
    return false;
  }

  for( unsigned int i = 0; i < ( 3 + NUM_STATE_GFE_ARM_JOINTS ); i++ ){
    if( ( out_right_arm_angles( i ) < _min_joint_limits_right_arm( i ) ) || ( out_right_arm_angles( i ) > _max_joint_limits_right_arm( i )  ) ){
      return false;
    } 
  }
  
  solution.joint( STATE_GFE_BACK_LBZ_JOINT ).set_position( out_right_arm_angles( 0 ) );
  solution.joint( STATE_GFE_BACK_MBY_JOINT ).set_position( out_right_arm_angles( 1 ) );
  solution.joint( STATE_GFE_BACK_UBX_JOINT ).set_position( out_right_arm_angles( 2 ) );
  for( unsigned int i = 0; i < NUM_STATE_GFE_ARM_JOINTS; i++ ){
    solution.right_arm().joint( ( state_gfe_arm_joint_t )( i ) ).set_position( out_right_arm_angles( 3 + i ) );
  }

  return true;
}

/**
 * inverse_kinematics_left_leg
 * solves for the left leg state given a target pose
 */
bool
Kinematics_Model_GFE::
inverse_kinematics_left_leg( const State_GFE& robotState,
                              const Frame& pelvisToFootPose,
                              State_GFE& solution ){
  JntArray in_left_leg_angles( NUM_STATE_GFE_LEG_JOINTS );
  JntArray out_left_leg_angles( NUM_STATE_GFE_LEG_JOINTS );

  for( unsigned int i = 0; i < NUM_STATE_GFE_LEG_JOINTS; i++ ){
    in_left_leg_angles( i ) = 0.5 * ( _min_joint_limits_left_leg( i ) + _max_joint_limits_left_leg( i ) );
  }

  if( _iksolverpos_left_leg != NULL ){
    if( _iksolverpos_left_leg->CartToJnt( in_left_leg_angles, pelvisToFootPose, out_left_leg_angles ) < 0 ){
      return false;
    }
  } else {
    return false;
  }

  for( unsigned int i = 0; i < NUM_STATE_GFE_LEG_JOINTS; i++ ){
    if( ( out_left_leg_angles( i ) < _min_joint_limits_left_leg( i ) ) || ( out_left_leg_angles( i ) > _max_joint_limits_left_leg( i )  ) ){
      return false;
    }
  }

  for( unsigned int i = 0; i < NUM_STATE_GFE_LEG_JOINTS; i++ ){
    solution.left_leg().joint( ( state_gfe_leg_joint_t )( i ) ).set_position( out_left_leg_angles( i ) );
  }

  return true;
}

/**
 * inverse_kinematics_right_leg
 * solves for the right leg state given a target pose
 */
bool
Kinematics_Model_GFE::
inverse_kinematics_right_leg( const State_GFE& robotState,
                              const Frame& pelvisToFootPose,
                              State_GFE& solution ){
  JntArray in_right_leg_angles( NUM_STATE_GFE_LEG_JOINTS );
  JntArray out_right_leg_angles( NUM_STATE_GFE_LEG_JOINTS );

  for( unsigned int i = 0; i < NUM_STATE_GFE_LEG_JOINTS; i++ ){
    in_right_leg_angles( i ) = 0.5 * ( _min_joint_limits_right_leg( i ) + _max_joint_limits_right_leg( i ) );
  }

  if( _iksolverpos_right_leg != NULL ){
    if( _iksolverpos_right_leg->CartToJnt( in_right_leg_angles, pelvisToFootPose, out_right_leg_angles ) < 0 ){
      return false;
    }
  } else {
    return false;
  }

  for( unsigned int i = 0; i < NUM_STATE_GFE_LEG_JOINTS; i++ ){
    if( ( out_right_leg_angles( i ) < _min_joint_limits_right_leg( i ) ) || ( out_right_leg_angles( i ) > _max_joint_limits_right_leg( i )  ) ){
      return false;
    }
  }

  for( unsigned int i = 0; i < NUM_STATE_GFE_LEG_JOINTS; i++ ){
    solution.right_leg().joint( ( state_gfe_leg_joint_t )( i ) ).set_position( out_right_leg_angles( 3 + i ) );
  }

  return true;
}

string
Kinematics_Model_GFE::
urdf_filename_to_xml_string( string urdfFilename ){
  string xml_string;
  fstream xml_file( urdfFilename.c_str(), fstream::in );
  if( xml_file.is_open() ){
    while( xml_file.good() ){
      string line;
      getline( xml_file, line );
      xml_string += ( line + "\n" );
    }
    xml_file.close();
  }
  return xml_string;
}	

/**
 * load_xml_string
 * loads a model from a xml string
 */
bool
Kinematics_Model_GFE::
load_xml_string( string xmlString,
                  double eps,
                  unsigned int maxIterations ){
  if( !_model.initString( xmlString ) ){
    return false;
  }
  if( !treeFromString( xmlString, _tree ) ){
    return false;
  }

  _fk_solver = new TreeFkSolverPos_recursive( _tree );

  if( !_tree.getChain( "pelvis", "l_hand", _left_arm_chain ) ){
    cout << "could not get left arm chain" << endl;
    return false;
  } else {
    _iksolverpos_left_arm = new ChainIkSolverPos_LMA( _left_arm_chain, eps, maxIterations );
  }

  if( !_tree.getChain( "pelvis", "r_hand", _right_arm_chain ) ){
    cout << "could not get left arm chain" << endl;
    return false;
  } else {
    _iksolverpos_right_arm = new ChainIkSolverPos_LMA( _right_arm_chain, eps, maxIterations );
  }

  if( !_tree.getChain( "pelvis", "l_foot", _left_leg_chain ) ){
    cout << "could not get left foot chain" << endl;
    return false;
  } else {
    _iksolverpos_left_leg = new ChainIkSolverPos_LMA( _left_leg_chain, eps, maxIterations );
  }
  
  if( !_tree.getChain( "pelvis", "r_foot", _right_leg_chain ) ){
    cout << "could not get right foot chain" << endl;
    return false;
  } else {
    _iksolverpos_right_leg = new ChainIkSolverPos_LMA( _right_leg_chain, eps, maxIterations );
  }


  _min_joint_limits_left_arm( 0 ) = _model.getJoint( "back_lbz" )->limits->lower;
  _min_joint_limits_left_arm( 1 ) = _model.getJoint( "back_mby" )->limits->lower;
  _min_joint_limits_left_arm( 2 ) = _model.getJoint( "back_ubx" )->limits->lower;

  _min_joint_limits_left_arm( 3 + STATE_GFE_ARM_USY_JOINT ) = _model.getJoint( "l_arm_usy" )->limits->lower;
  _min_joint_limits_left_arm( 3 + STATE_GFE_ARM_SHX_JOINT ) = _model.getJoint( "l_arm_shx" )->limits->lower;
  _min_joint_limits_left_arm( 3 + STATE_GFE_ARM_ELY_JOINT ) = _model.getJoint( "l_arm_ely" )->limits->lower;
  _min_joint_limits_left_arm( 3 + STATE_GFE_ARM_ELX_JOINT ) = _model.getJoint( "l_arm_elx" )->limits->lower;
  _min_joint_limits_left_arm( 3 + STATE_GFE_ARM_UWY_JOINT ) = _model.getJoint( "l_arm_uwy" )->limits->lower;
  _min_joint_limits_left_arm( 3 + STATE_GFE_ARM_MWX_JOINT ) = _model.getJoint( "l_arm_mwx" )->limits->lower;

  _max_joint_limits_left_arm( 0 ) = _model.getJoint( "back_lbz" )->limits->upper;
  _max_joint_limits_left_arm( 1 ) = _model.getJoint( "back_mby" )->limits->upper;
  _max_joint_limits_left_arm( 2 ) = _model.getJoint( "back_ubx" )->limits->upper;

  _max_joint_limits_left_arm( 3 + STATE_GFE_ARM_USY_JOINT ) = _model.getJoint( "l_arm_usy" )->limits->upper;
  _max_joint_limits_left_arm( 3 + STATE_GFE_ARM_SHX_JOINT ) = _model.getJoint( "l_arm_shx" )->limits->upper;
  _max_joint_limits_left_arm( 3 + STATE_GFE_ARM_ELY_JOINT ) = _model.getJoint( "l_arm_ely" )->limits->upper;
  _max_joint_limits_left_arm( 3 + STATE_GFE_ARM_ELX_JOINT ) = _model.getJoint( "l_arm_elx" )->limits->upper;
  _max_joint_limits_left_arm( 3 + STATE_GFE_ARM_UWY_JOINT ) = _model.getJoint( "l_arm_uwy" )->limits->upper;
  _max_joint_limits_left_arm( 3 + STATE_GFE_ARM_MWX_JOINT ) = _model.getJoint( "l_arm_mwx" )->limits->upper;  

  _min_joint_limits_right_arm( 0 ) = _model.getJoint( "back_lbz" )->limits->lower;
  _min_joint_limits_right_arm( 1 ) = _model.getJoint( "back_mby" )->limits->lower;
  _min_joint_limits_right_arm( 2 ) = _model.getJoint( "back_ubx" )->limits->lower;

  _min_joint_limits_right_arm( 3 + STATE_GFE_ARM_USY_JOINT ) = _model.getJoint( "r_arm_usy" )->limits->lower;
  _min_joint_limits_right_arm( 3 + STATE_GFE_ARM_SHX_JOINT ) = _model.getJoint( "r_arm_shx" )->limits->lower;
  _min_joint_limits_right_arm( 3 + STATE_GFE_ARM_ELY_JOINT ) = _model.getJoint( "r_arm_ely" )->limits->lower;
  _min_joint_limits_right_arm( 3 + STATE_GFE_ARM_ELX_JOINT ) = _model.getJoint( "r_arm_elx" )->limits->lower;
  _min_joint_limits_right_arm( 3 + STATE_GFE_ARM_UWY_JOINT ) = _model.getJoint( "r_arm_uwy" )->limits->lower;
  _min_joint_limits_right_arm( 3 + STATE_GFE_ARM_MWX_JOINT ) = _model.getJoint( "r_arm_mwx" )->limits->lower;

  _max_joint_limits_right_arm( 0 ) = _model.getJoint( "back_lbz" )->limits->upper;
  _max_joint_limits_right_arm( 1 ) = _model.getJoint( "back_mby" )->limits->upper;
  _max_joint_limits_right_arm( 2 ) = _model.getJoint( "back_ubx" )->limits->upper;

  _max_joint_limits_right_arm( 3 + STATE_GFE_ARM_USY_JOINT ) = _model.getJoint( "r_arm_usy" )->limits->upper;
  _max_joint_limits_right_arm( 3 + STATE_GFE_ARM_SHX_JOINT ) = _model.getJoint( "r_arm_shx" )->limits->upper;
  _max_joint_limits_right_arm( 3 + STATE_GFE_ARM_ELY_JOINT ) = _model.getJoint( "r_arm_ely" )->limits->upper;
  _max_joint_limits_right_arm( 3 + STATE_GFE_ARM_ELX_JOINT ) = _model.getJoint( "r_arm_elx" )->limits->upper;
  _max_joint_limits_right_arm( 3 + STATE_GFE_ARM_UWY_JOINT ) = _model.getJoint( "r_arm_uwy" )->limits->upper;
  _max_joint_limits_right_arm( 3 + STATE_GFE_ARM_MWX_JOINT ) = _model.getJoint( "r_arm_mwx" )->limits->upper;

  _min_joint_limits_left_leg( STATE_GFE_LEG_KNY_JOINT ) = _model.getJoint( "l_leg_kny" )->limits->lower;
  _min_joint_limits_left_leg( STATE_GFE_LEG_LAX_JOINT ) = _model.getJoint( "l_leg_lax" )->limits->lower;
  _min_joint_limits_left_leg( STATE_GFE_LEG_LHY_JOINT ) = _model.getJoint( "l_leg_lhy" )->limits->lower;
  _min_joint_limits_left_leg( STATE_GFE_LEG_MHX_JOINT ) = _model.getJoint( "l_leg_mhx" )->limits->lower;
  _min_joint_limits_left_leg( STATE_GFE_LEG_UAY_JOINT ) = _model.getJoint( "l_leg_uay" )->limits->lower;
  _min_joint_limits_left_leg( STATE_GFE_LEG_UHZ_JOINT ) = _model.getJoint( "l_leg_uhz" )->limits->lower;

  _max_joint_limits_left_leg( STATE_GFE_LEG_KNY_JOINT ) = _model.getJoint( "l_leg_kny" )->limits->upper;
  _max_joint_limits_left_leg( STATE_GFE_LEG_LAX_JOINT ) = _model.getJoint( "l_leg_lax" )->limits->upper;
  _max_joint_limits_left_leg( STATE_GFE_LEG_LHY_JOINT ) = _model.getJoint( "l_leg_lhy" )->limits->upper;
  _max_joint_limits_left_leg( STATE_GFE_LEG_MHX_JOINT ) = _model.getJoint( "l_leg_mhx" )->limits->upper;
  _max_joint_limits_left_leg( STATE_GFE_LEG_UAY_JOINT ) = _model.getJoint( "l_leg_uay" )->limits->upper;
  _max_joint_limits_left_leg( STATE_GFE_LEG_UHZ_JOINT ) = _model.getJoint( "l_leg_uhz" )->limits->upper;

  _min_joint_limits_right_leg( STATE_GFE_LEG_KNY_JOINT ) = _model.getJoint( "r_leg_kny" )->limits->lower;
  _min_joint_limits_right_leg( STATE_GFE_LEG_LAX_JOINT ) = _model.getJoint( "r_leg_lax" )->limits->lower;
  _min_joint_limits_right_leg( STATE_GFE_LEG_LHY_JOINT ) = _model.getJoint( "r_leg_lhy" )->limits->lower;
  _min_joint_limits_right_leg( STATE_GFE_LEG_MHX_JOINT ) = _model.getJoint( "r_leg_mhx" )->limits->lower;
  _min_joint_limits_right_leg( STATE_GFE_LEG_UAY_JOINT ) = _model.getJoint( "r_leg_uay" )->limits->lower;
  _min_joint_limits_right_leg( STATE_GFE_LEG_UHZ_JOINT ) = _model.getJoint( "r_leg_uhz" )->limits->lower;

  _max_joint_limits_right_leg( STATE_GFE_LEG_KNY_JOINT ) = _model.getJoint( "r_leg_kny" )->limits->upper;
  _max_joint_limits_right_leg( STATE_GFE_LEG_LAX_JOINT ) = _model.getJoint( "r_leg_lax" )->limits->upper;
  _max_joint_limits_right_leg( STATE_GFE_LEG_LHY_JOINT ) = _model.getJoint( "r_leg_lhy" )->limits->upper;
  _max_joint_limits_right_leg( STATE_GFE_LEG_MHX_JOINT ) = _model.getJoint( "r_leg_mhx" )->limits->upper;
  _max_joint_limits_right_leg( STATE_GFE_LEG_UAY_JOINT ) = _model.getJoint( "r_leg_uay" )->limits->upper;
  _max_joint_limits_right_leg( STATE_GFE_LEG_UHZ_JOINT ) = _model.getJoint( "r_leg_uhz" )->limits->upper;

  return true;
}

/**
 * load_urdf
 * loads a model from a urdf file
 */
bool
Kinematics_Model_GFE::
load_urdf( string urdfFilename,
            double eps,
            unsigned int maxIterations ){
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
  
  return load_xml_string( xml_string, eps, maxIterations );
}

/**
 * set
 * updates the robot state in the kinematics model
 */
void
Kinematics_Model_GFE::
set( const drc::robot_state_t& robotState ){
  Kinematics_Model::drc_position_3d_t_to_kdl_frame( robotState.origin_position, _world_to_body );
 
  _link_frames.clear();

  map< string, double > joint_angles;
  for( unsigned int i = 0; i < robotState.num_joints; i++ ){
    joint_angles.insert( make_pair( robotState.joint_name[ i ], robotState.joint_position[ i ] ) );
  }

  _update_joint( joint_angles, _world_to_body, _tree.getRootSegment() );

  return;
}

/**
 * set
 * updates the robot state in the kinematics model using the state::State_GFE class
 */
void
Kinematics_Model_GFE::
set( State_GFE& stateGFE ){
  _link_frames.clear();
 
  _world_to_body = stateGFE.pose();
 
  map< string, double > joint_angles = stateGFE.joint_angles();
  _update_joint( joint_angles, _world_to_body, _tree.getRootSegment() );

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
  map< string, Frame >::const_iterator joint_frame_iterator = _link_frames.find( linkName );
  if( joint_frame_iterator != _link_frames.end() ){
    return joint_frame_iterator->second;
  } else {
    cout << "could not find frame " << linkName << endl;
    return Frame();
  }
}

/**
 * link_frames
 * returns a map of link frames
 */
map< string, Frame >
Kinematics_Model_GFE::
link_frames( void )const{
  return _link_frames;
}

/**
 * min_joint_limits_left_arm
 * returns the JntArry that contains the min joint angles for the left arm
 */
const JntArray&
Kinematics_Model_GFE::
min_joint_limits_left_arm( void )const{
  return _min_joint_limits_left_arm;
}

/**
 * max_joint_limits_left_arm
 * returns the JntArry that contains the max joint angles for the left arm
 */
const JntArray&
Kinematics_Model_GFE::
max_joint_limits_left_arm( void )const{
  return _max_joint_limits_left_arm;
}

/**
 * min_joint_limits_right_arm
 * returns the JntArry that contains the min joint angles for the right arm
 */
const JntArray&
Kinematics_Model_GFE::
min_joint_limits_right_arm( void )const{
  return _min_joint_limits_right_arm;
}

/**
 * max_joint_limits_right_arm
 * returns the JntArry that contains the max joint angles for the right arm
 */
const JntArray&
Kinematics_Model_GFE::
max_joint_limits_right_arm( void )const{
  return _max_joint_limits_right_arm;
}

/**
 * min_joint_limits_left_leg
 * returns the JntArry that contains the min joint angles for the left leg
 */
const JntArray&
Kinematics_Model_GFE::
min_joint_limits_left_leg( void )const{
  return _min_joint_limits_left_leg;
}

/**
 * max_joint_limits_left_leg
 * returns the JntArry that contains the max joint angles for the left leg
 */
const JntArray&
Kinematics_Model_GFE::
max_joint_limits_left_leg( void )const{
  return _max_joint_limits_left_leg;
}

/**
 * min_joint_limits_right_leg
 * returns the JntArry that contains the min joint angles for the right leg
 */
const JntArray&
Kinematics_Model_GFE::
min_joint_limits_right_leg( void )const{
  return _min_joint_limits_right_leg;
}

/**
 * max_joint_limits_right_leg
 * returns the JntArry that contains the max joint angles for the right leg
 */
const JntArray&
Kinematics_Model_GFE::
max_joint_limits_right_leg( void )const{
  return _max_joint_limits_right_leg;
}

/**
 * left_arm_chain
 * returns a const reference to the left arm chain
 */
const Chain&
Kinematics_Model_GFE::
left_arm_chain( void )const{
  return _left_arm_chain;
}

/**
 * right_arm_chain
 * returns a const reference to the right arm chain
 */
const Chain&
Kinematics_Model_GFE::
right_arm_chain( void )const{
  return _right_arm_chain;
}

/**
 * left_leg_chain
 * returns a const reference to the left leg chain
 */
const Chain&
Kinematics_Model_GFE::
left_leg_chain( void )const{
  return _left_leg_chain;
}

/**
 * right_leg_chain
 * returns a const reference to the right leg chain
 */
const Chain&
Kinematics_Model_GFE::
right_leg_chain( void )const{
  return _right_leg_chain;
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
    _link_frames.insert( make_pair( jointIterator->first, joint_frame ) );
  } else {
    joint_frame = parentFrame * jointIterator->second.segment.pose( 0.0 );
    _link_frames.insert( make_pair( jointIterator->first, joint_frame ) );
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
              const Kinematics_Model_GFE& other ){
    out << endl;
    // display the left arm chain
    out << "  l_arm_chain: ";
    for( unsigned int i = 0; i < other.left_arm_chain().getNrOfSegments(); i++ ){
      out << other.left_arm_chain().getSegment( i ).getJoint().getName();
      out << "(" << other.left_arm_chain().getSegment( i ).getInertia().getMass() << "kg)";
      if( i != ( other.left_arm_chain().getNrOfSegments() - 1 ) ){
        out << ",";
      }
    }
    out << endl;
    // display the left arm limits
    out << "  l_arm_limits: ";
    for( unsigned int i = 0; i < NUM_STATE_GFE_ARM_JOINTS; i++ ){
      out << "("  << other.min_joint_limits_left_arm()( i ) << "," << other.max_joint_limits_left_arm()( i ) << ")";
      if( i != ( NUM_STATE_GFE_ARM_JOINTS - 1 ) ){
        out << ",";
      }
    }
    out << endl;
    // display the right arm chain
    out << "  r_arm_chain: ";
    for( unsigned int i = 0; i < other.right_arm_chain().getNrOfSegments(); i++ ){
      out << other.right_arm_chain().getSegment( i ).getJoint().getName();
      out << "(" << other.right_arm_chain().getSegment( i ).getInertia().getMass() << "kg)";
      if( i != ( other.right_arm_chain().getNrOfSegments() - 1 ) ){
        out << ",";
      }
    }
    out << endl;
    // display the right arm limits
    out << "  r_arm_limits: ";
    for( unsigned int i = 0; i < NUM_STATE_GFE_ARM_JOINTS; i++ ){
      out << "("  << other.min_joint_limits_right_arm()( i ) << "," << other.max_joint_limits_right_arm()( i ) << ")";
      if( i != ( NUM_STATE_GFE_ARM_JOINTS - 1 ) ){
        out << ",";
      }
    }
    out << endl;
    // display the left leg chain
    out << "  l_leg_chain: ";
    for( unsigned int i = 0; i < other.left_leg_chain().getNrOfSegments(); i++ ){
      out << other.left_leg_chain().getSegment( i ).getJoint().getName();
      out << "(" << other.left_leg_chain().getSegment( i ).getInertia().getMass() << "kg)";
      if( i != ( other.left_leg_chain().getNrOfSegments() - 1 ) ){
        out << ",";
      }
    }
    out << endl;
    // display the left leg limits
    out << "  l_leg_limits: ";
    for( unsigned int i = 0; i < NUM_STATE_GFE_LEG_JOINTS; i++ ){
      out << "("  << other.min_joint_limits_left_leg()( i ) << "," << other.max_joint_limits_left_leg()( i ) << ")";
      if( i != ( NUM_STATE_GFE_LEG_JOINTS - 1 ) ){
        out << ",";
      }
    }
    out << endl;
    // display the right leg chain
    out << "  r_leg_chain: ";
    for( unsigned int i = 0; i < other.right_leg_chain().getNrOfSegments(); i++ ){
      out << other.right_leg_chain().getSegment( i ).getJoint().getName();
      out << "(" << other.right_leg_chain().getSegment( i ).getInertia().getMass() << "kg)";
      if( i != ( other.right_leg_chain().getNrOfSegments() - 1 ) ){
        out << ",";
      }
    }
    out << endl;
    // display the right leg limits
    out << "  r_leg_limits: ";
    for( unsigned int i = 0; i < NUM_STATE_GFE_LEG_JOINTS; i++ ){
      out << "("  << other.min_joint_limits_right_leg()( i ) << "," << other.max_joint_limits_right_leg()( i ) << ")";
      if( i != ( NUM_STATE_GFE_LEG_JOINTS - 1 ) ){
        out << ",";
      }
    }
    out << endl;
    return out;
  }
} 
