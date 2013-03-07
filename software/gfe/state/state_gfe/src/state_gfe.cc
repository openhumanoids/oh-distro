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
State_GFE() : State( "gfe" ),
              _pose(),
              _left_arm( "l_arm_" ),
              _right_arm( "r_arm_" ),
              _left_hand ( "left_" ),
              _right_hand( "right_" ),
              _left_leg( "l_leg_" ),
              _right_leg( "r_leg_" ),
              _joints() {
  _joints[ STATE_GFE_BACK_LBZ_JOINT ].set_id( "back_lbz" );
  _joints[ STATE_GFE_BACK_MBY_JOINT ].set_id( "back_mby" );
  _joints[ STATE_GFE_BACK_UBX_JOINT ].set_id( "back_ubx" );
  _joints[ STATE_GFE_NECK_AY_JOINT ].set_id( "neck_ay" );
  _joints[ STATE_GFE_HEAD_IMU_JOINT ].set_id( "head_imu_joint" );
  _joints[ STATE_GFE_HOKUYO_JOINT ].set_id( "hokuyo_joint" );
  _joints[ STATE_GFE_IMU_JOINT ].set_id( "imu_joint" );
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
State_GFE( const State_GFE& other ) : State( other ),
                                      _pose( other._pose ),
                                      _left_arm( other._left_arm ),
                                      _right_arm( other._right_arm ),
                                      _left_hand( other._left_hand ),
                                      _right_hand( other._right_hand ),
                                      _left_leg( other._left_leg ),
                                      _right_leg( other._right_leg ),
                                      _joints(){
  for( unsigned int i = 0; i < NUM_STATE_GFE_JOINTS; i++ ){
    _joints[ i ] = other._joints[ i ];
  }
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
  _left_arm = other._left_arm;
  _right_arm = other._right_arm;
  _left_hand = other._left_hand;
  _right_hand = other._right_hand;
  _left_leg = other._left_leg;
  _right_leg = other._right_leg;
  for( unsigned int i = 0; i < NUM_STATE_GFE_JOINTS; i++ ){
    _joints[ i ] = other._joints[ i ];
  }
  return ( *this );
}

/**
 * interpolate
 * interpolates between the first and second states at the given time
 */
State_GFE
State_GFE::
interpolate( const State_GFE& first,
              const State_GFE& second,
              unsigned long long time ){
  std::cout << "in interpolate" << std::endl;
  std::cout << "first " << first << endl;
  std::cout << "second " << second << endl;
  std::cout << "time: " << time << endl;
  State_GFE state;
  state.set_id( first.id() );
  state.set_time( time );
  state.set_pose( first.pose() );
  state.left_arm() = State_GFE_Arm::interpolate( first.left_arm(), second.left_arm(), time );
  state.right_arm() = State_GFE_Arm::interpolate( first.right_arm(), second.right_arm(), time );
  state.left_hand() = State_GFE_Hand::interpolate( first.left_hand(), second.left_hand(), time );
  state.right_hand() = State_GFE_Hand::interpolate( first.right_hand(), second.right_hand(), time );
  state.left_leg() = State_GFE_Leg::interpolate( first.left_leg(), second.left_leg(), time );
  state.right_leg() = State_GFE_Leg::interpolate( first.right_leg(), second.right_leg(), time );
  for( unsigned int i = 0; i < NUM_STATE_GFE_JOINTS; i++ ){
    state.joint( ( state_gfe_joint_t )( i ) ) = State_GFE_Joint::interpolate( first.joint( ( state_gfe_joint_t )( i ) ), second.joint( ( state_gfe_joint_t )( i ) ), time );
  }
  std::cout << "state" << state << endl;
  return state;
} 

/**
 * from_lcm
 * constructs the joints from a drc::robot_state_t message reference
 */
bool
State_GFE::
from_lcm( const robot_state_t& robotState ){
  return from_lcm( &robotState );
}

/**
 * from_lcm
 * constructs the joints from a drc::robot_state_t message pointer
 */
bool
State_GFE::
from_lcm( const robot_state_t* robotState ){
  if( robotState == NULL ){
    return false;
  } else {
    _time = robotState->utime;
    _pose.p[ 0 ] = robotState->origin_position.translation.x;
    _pose.p[ 1 ] = robotState->origin_position.translation.y;
    _pose.p[ 2 ] = robotState->origin_position.translation.z;
    _pose.M = KDL::Rotation::Quaternion( robotState->origin_position.rotation.x, robotState->origin_position.rotation.y, robotState->origin_position.rotation.z, robotState->origin_position.rotation.w );

    for( unsigned int i = 0; i < robotState->num_joints; i++ ){ 
      State_GFE_Joint& current_joint = joint( robotState->joint_name[i] );
      current_joint.set_time( robotState->utime );
      if( robotState->joint_position[i] > M_PI ){
        current_joint.set_position( robotState->joint_position[i] - 2.0 * M_PI );
      } else if ( robotState->joint_position[i] < -M_PI ){
        current_joint.set_position( robotState->joint_position[i] + 2.0 * M_PI );
      } else {
        current_joint.set_position( robotState->joint_position[i] );
      }
      current_joint.set_velocity( robotState->joint_velocity[i] );
      current_joint.set_measured_effort( robotState->measured_effort[i] );
    }
    return true;
  }
}

/**
 * to_lcm
 * constructs an drc::robot_state_t message from the contents of the State_GF#
 */
void
State_GFE::
to_lcm( robot_state_t* robotState )const{
  robotState->utime = _time;
  robotState->origin_position.translation.x = _pose.p[ 0 ];
  robotState->origin_position.translation.y = _pose.p[ 1 ];
  robotState->origin_position.translation.z = _pose.p[ 2 ];
  _pose.M.GetQuaternion( robotState->origin_position.rotation.x, robotState->origin_position.rotation.y, robotState->origin_position.rotation.z, robotState->origin_position.rotation.w );
  robotState->num_joints = 53;
  robotState->joint_name.resize( robotState->num_joints );
  robotState->joint_name[0] = "back_lbz";
  robotState->joint_name[1] = "back_mby";
  robotState->joint_name[2] = "back_ubx";
  robotState->joint_name[3] = "imu_joint";
  robotState->joint_name[4] = "l_arm_elx";
  robotState->joint_name[5] = "l_arm_ely";
  robotState->joint_name[6] = "l_arm_mwx";
  robotState->joint_name[7] = "l_arm_shx";
  robotState->joint_name[8] = "l_arm_usy";
  robotState->joint_name[9] = "l_arm_uwy";
  robotState->joint_name[10] = "l_leg_kny";
  robotState->joint_name[11] = "l_leg_lax";
  robotState->joint_name[12] = "l_leg_lhy";
  robotState->joint_name[13] = "l_leg_mhx";
  robotState->joint_name[14] = "l_leg_uay";
  robotState->joint_name[15] = "l_leg_uhz";
  robotState->joint_name[16] = "neck_ay";
  robotState->joint_name[17] = "r_arm_elx";
  robotState->joint_name[18] = "r_arm_ely";
  robotState->joint_name[19] = "r_arm_mwx";
  robotState->joint_name[20] = "r_arm_shx";
  robotState->joint_name[21] = "r_arm_usy";
  robotState->joint_name[22] = "r_arm_uwy";
  robotState->joint_name[23] = "r_leg_kny";
  robotState->joint_name[24] = "r_leg_lax";
  robotState->joint_name[25] = "r_leg_lhy";
  robotState->joint_name[26] = "r_leg_mhx";
  robotState->joint_name[27] = "r_leg_uay";
  robotState->joint_name[28] = "r_leg_uhz";
  robotState->joint_name[29] = "left_f0_j0";
  robotState->joint_name[30] = "left_f0_j1";
  robotState->joint_name[31] = "left_f0_j2";
  robotState->joint_name[32] = "left_f1_j0";
  robotState->joint_name[33] = "left_f1_j1";
  robotState->joint_name[34] = "left_f1_j2";
  robotState->joint_name[35] = "left_f2_j0";
  robotState->joint_name[36] = "left_f2_j1";
  robotState->joint_name[37] = "left_f2_j2";
  robotState->joint_name[38] = "left_f3_j0";
  robotState->joint_name[39] = "left_f3_j1";
  robotState->joint_name[40] = "left_f3_j2";
  robotState->joint_name[41] = "right_f0_j0";
  robotState->joint_name[42] = "right_f0_j1";
  robotState->joint_name[43] = "right_f0_j2";
  robotState->joint_name[44] = "right_f1_j0";
  robotState->joint_name[45] = "right_f1_j1";
  robotState->joint_name[46] = "right_f1_j2";
  robotState->joint_name[47] = "right_f2_j0";
  robotState->joint_name[48] = "right_f2_j1";
  robotState->joint_name[49] = "right_f2_j2";
  robotState->joint_name[50] = "right_f3_j0";
  robotState->joint_name[51] = "right_f3_j1";
  robotState->joint_name[52] = "right_f3_j2";
  robotState->joint_position.resize( robotState->num_joints );
  robotState->joint_velocity.resize( robotState->num_joints );
  robotState->measured_effort.resize( robotState->num_joints );
  robotState->joint_cov.resize( robotState->num_joints );
  for( unsigned int i = 0; i < robotState->num_joints; i++ ){
    const State_GFE_Joint& current_joint = joint( robotState->joint_name[i] );
    robotState->joint_position[ i ] = current_joint.position();
    robotState->joint_velocity[ i ] = current_joint.velocity();
    robotState->measured_effort[ i ] = current_joint.measured_effort();
  }
  robotState->contacts.num_contacts = 0;
  robotState->contacts.id.clear(); 
  robotState->contacts.contact_torque.clear(); 
  robotState->contacts.contact_force.clear(); 
  return;
}

/**
 * from_urdf
 * loads the state from the URDF
 */
bool
State_GFE::
from_urdf( string filename ){
  return true;
}

/**
 * set_time
 * sets the time of the state 
 */
void
State_GFE::
set_time( unsigned long long time ){
  _time = time;
  _left_arm.set_time( time );
  _right_arm.set_time( time );
  _left_leg.set_time( time );
  _right_leg.set_time( time );
  for( unsigned int i = 0; i < NUM_STATE_GFE_JOINTS; i++ ){
    _joints[ i ].set_time( time );
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
  map< string, State_GFE_Joint > joints;
  for (uint i = 0; i < NUM_STATE_GFE_JOINTS; i++) {
      joints[_joints[i].id()] = _joints[i];
  }
  return joints;
}

/**
 * joint_angle
 * returns a map of joint angles
 */
map< string, double >
State_GFE::
joint_angles( void )const{
  map< string, double > joint_angles;
  for( unsigned int i = 0; i < NUM_STATE_GFE_JOINTS; i++ ){
    joint_angles.insert( make_pair( _joints[ i ].id(), _joints[ i ].position() ) );
  }
  for( unsigned int i = 0; i < NUM_STATE_GFE_ARM_JOINTS; i++ ){
    joint_angles.insert( make_pair( _left_arm.joint( ( state_gfe_arm_joint_t )( i ) ).id(), _left_arm.joint( ( state_gfe_arm_joint_t )( i ) ).position() ) );
    joint_angles.insert( make_pair( _right_arm.joint( ( state_gfe_arm_joint_t )( i ) ).id(), _right_arm.joint( ( state_gfe_arm_joint_t )( i ) ).position() ) );
  }
  for( unsigned int i = 0; i < NUM_STATE_GFE_HAND_FINGERS; i++ ){
    for( unsigned int j = 0; j < NUM_STATE_GFE_FINGER_JOINTS; j++ ){
      joint_angles.insert( make_pair( _left_hand.finger( ( state_gfe_finger_t )( i ) ).joint( ( state_gfe_finger_joint_t )( j ) ).id(), _left_hand.finger( ( state_gfe_finger_t )( i ) ).joint( ( state_gfe_finger_joint_t )( j ) ).position() ) );
      joint_angles.insert( make_pair( _right_hand.finger( ( state_gfe_finger_t )( i ) ).joint( ( state_gfe_finger_joint_t )( j ) ).id(), _right_hand.finger( ( state_gfe_finger_t )( i ) ).joint( ( state_gfe_finger_joint_t )( j ) ).position() ) );
    }
  }
  for( unsigned int i = 0; i < NUM_STATE_GFE_LEG_JOINTS; i++ ){
    joint_angles.insert( make_pair( _left_leg.joint( ( state_gfe_leg_joint_t )( i ) ).id(), _left_leg.joint( ( state_gfe_leg_joint_t )( i ) ).position() ) );
    joint_angles.insert( make_pair( _right_leg.joint( ( state_gfe_leg_joint_t )( i ) ).id(), _right_leg.joint( ( state_gfe_leg_joint_t )( i ) ).position() ) );
  }

  return joint_angles;
}

State_GFE_Arm&
State_GFE::
left_arm( void ){
  return _left_arm;
}

const State_GFE_Arm&
State_GFE::
left_arm( void )const{
  return _left_arm;
}

State_GFE_Arm&
State_GFE::
right_arm( void ){
  return _right_arm;
}

const State_GFE_Arm&
State_GFE::
right_arm( void )const{
  return _right_arm;
}

State_GFE_Hand&
State_GFE::
left_hand( void ){
  return _left_hand;
}

const State_GFE_Hand&
State_GFE::
left_hand( void )const{
  return _left_hand;
}

State_GFE_Hand&
State_GFE::
right_hand( void ){
  return _right_hand;
}

const State_GFE_Hand&
State_GFE::
right_hand( void )const{
  return _right_hand;
}

State_GFE_Leg&
State_GFE::
left_leg( void ){
  return _left_leg;
}

const State_GFE_Leg&
State_GFE::
left_leg( void )const{
  return _left_leg;
}

State_GFE_Leg&
State_GFE::
right_leg( void ){
  return _right_leg;
}

const State_GFE_Leg&
State_GFE::
right_leg( void )const{
  return _right_leg;
}

State_GFE_Joint&
State_GFE::
joint( state_gfe_joint_t joint ){
  return _joints[ joint ];
}

const State_GFE_Joint&
State_GFE::
joint( state_gfe_joint_t joint )const{
  return _joints[ joint ];
}

/**
 * joint
 * returns a reference to a specific joint based on the id argument 
 */
State_GFE_Joint&
State_GFE::
joint( string id ){
/*
  map< string, State_GFE_Joint >::iterator it = _joints.find( id );
  return  it->second;
*/
  if( id == "back_lbz" ){
    return _joints[ STATE_GFE_BACK_LBZ_JOINT ];
  } else if( id == "back_mby" ){
    return _joints[ STATE_GFE_BACK_MBY_JOINT ];
  } else if( id == "back_ubx" ){
    return _joints[ STATE_GFE_BACK_UBX_JOINT ];
  } else if( id == "neck_ay" ){
    return _joints[ STATE_GFE_NECK_AY_JOINT ];
  } else if( id == "head_imu_joint" ){
    return _joints[ STATE_GFE_HEAD_IMU_JOINT ];
  } else if( id == "hokuyo_joint" ){
    return _joints[ STATE_GFE_HOKUYO_JOINT ];
  } else if( id == "imu_joint" ){
    return _joints[ STATE_GFE_IMU_JOINT ];
  } else if( id == "l_arm_elx" ){
    return _left_arm.joint( STATE_GFE_ARM_ELX_JOINT );
  } else if ( id == "l_arm_ely" ){
    return _left_arm.joint( STATE_GFE_ARM_ELY_JOINT );
  } else if ( id == "l_arm_mwx" ){
    return _left_arm.joint( STATE_GFE_ARM_MWX_JOINT );
  } else if ( id == "l_arm_shx" ){
    return _left_arm.joint( STATE_GFE_ARM_SHX_JOINT );
  } else if ( id == "l_arm_usy" ){
    return _left_arm.joint( STATE_GFE_ARM_USY_JOINT );
  } else if ( id == "l_arm_uwy" ){
    return _left_arm.joint( STATE_GFE_ARM_UWY_JOINT );
  } else if ( id == "r_arm_elx" ){
    return _right_arm.joint( STATE_GFE_ARM_ELX_JOINT );
  } else if ( id == "r_arm_ely" ){
    return _right_arm.joint( STATE_GFE_ARM_ELY_JOINT );
  } else if ( id == "r_arm_mwx" ){
    return _right_arm.joint( STATE_GFE_ARM_MWX_JOINT );
  } else if ( id == "r_arm_shx" ){
    return _right_arm.joint( STATE_GFE_ARM_SHX_JOINT );
  } else if ( id == "r_arm_usy" ){
    return _right_arm.joint( STATE_GFE_ARM_USY_JOINT );
  } else if ( id == "r_arm_uwy" ){
    return _right_arm.joint( STATE_GFE_ARM_UWY_JOINT );
  } else if ( id == "left_f0_j0" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_0 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "left_f0_j1" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_0 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "left_f0_j2" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_0 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "left_f1_j0" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_1 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "left_f1_j1" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_1 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "left_f1_j2" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_1 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "left_f2_j0" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_2 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "left_f2_j1" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_2 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "left_f2_j2" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_2 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "left_f3_j0" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_3 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "left_f3_j1" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_3 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "left_f3_j2" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_3 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "right_f0_j0" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_0 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "right_f0_j1" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_0 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "right_f0_j2" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_0 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "right_f1_j0" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_1 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "right_f1_j1" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_1 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "right_f1_j2" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_1 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "right_f2_j0" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_2 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "right_f2_j1" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_2 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "right_f2_j2" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_2 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "right_f3_j0" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_3 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "right_f3_j1" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_3 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "right_f3_j2" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_3 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "l_leg_kny" ){
    return _left_leg.joint( STATE_GFE_LEG_KNY_JOINT );
  } else if ( id == "l_leg_lax" ){
    return _left_leg.joint( STATE_GFE_LEG_LAX_JOINT );
  } else if ( id == "l_leg_lhy" ){
    return _left_leg.joint( STATE_GFE_LEG_LHY_JOINT );
  } else if ( id == "l_leg_mhx" ){
    return _left_leg.joint( STATE_GFE_LEG_MHX_JOINT );
  } else if ( id == "l_leg_uay" ){
    return _left_leg.joint( STATE_GFE_LEG_UAY_JOINT );
  } else if ( id == "l_leg_uhz" ){
    return _left_leg.joint( STATE_GFE_LEG_UHZ_JOINT );
  } else if ( id == "r_leg_kny" ){
    return _right_leg.joint( STATE_GFE_LEG_KNY_JOINT );
  } else if ( id == "r_leg_lax" ){
    return _right_leg.joint( STATE_GFE_LEG_LAX_JOINT );
  } else if ( id == "r_leg_lhy" ){
    return _right_leg.joint( STATE_GFE_LEG_LHY_JOINT );
  } else if ( id == "r_leg_mhx" ){
    return _right_leg.joint( STATE_GFE_LEG_MHX_JOINT );
  } else if ( id == "r_leg_uay" ){
    return _right_leg.joint( STATE_GFE_LEG_UAY_JOINT );
  } else if ( id == "r_leg_uhz" ){
    return _right_leg.joint( STATE_GFE_LEG_UHZ_JOINT );
  }
}

/**
 * joint
 * returns a const reference to a specific joint based on the id argument
 */
const State_GFE_Joint&
State_GFE::
joint( string id )const
{
/*
  map< string, State_GFE_Joint >::const_iterator it = _joints.find( id );
  return it->second;
*/
  if( id == "back_lbz" ){
    return _joints[ STATE_GFE_BACK_LBZ_JOINT ];
  } else if( id == "back_mby" ){
    return _joints[ STATE_GFE_BACK_MBY_JOINT ];
  } else if( id == "back_ubx" ){
    return _joints[ STATE_GFE_BACK_UBX_JOINT ];
  } else if( id == "neck_ay" ){
    return _joints[ STATE_GFE_NECK_AY_JOINT ];
  } else if( id == "head_imu_joint" ){
    return _joints[ STATE_GFE_HEAD_IMU_JOINT ];
  } else if( id == "hokuyo_joint" ){
    return _joints[ STATE_GFE_HOKUYO_JOINT ];
  } else if( id == "imu_joint" ){
    return _joints[ STATE_GFE_IMU_JOINT ];
  } else if( id == "l_arm_elx" ){
    return _left_arm.joint( STATE_GFE_ARM_ELX_JOINT );
  } else if ( id == "l_arm_ely" ){
    return _left_arm.joint( STATE_GFE_ARM_ELY_JOINT );
  } else if ( id == "l_arm_mwx" ){
    return _left_arm.joint( STATE_GFE_ARM_MWX_JOINT );
  } else if ( id == "l_arm_shx" ){
    return _left_arm.joint( STATE_GFE_ARM_SHX_JOINT );
  } else if ( id == "l_arm_usy" ){
    return _left_arm.joint( STATE_GFE_ARM_USY_JOINT );
  } else if ( id == "l_arm_uwy" ){
    return _left_arm.joint( STATE_GFE_ARM_UWY_JOINT );
  } else if ( id == "r_arm_elx" ){
    return _right_arm.joint( STATE_GFE_ARM_ELX_JOINT );
  } else if ( id == "r_arm_ely" ){
    return _right_arm.joint( STATE_GFE_ARM_ELY_JOINT );
  } else if ( id == "r_arm_mwx" ){
    return _right_arm.joint( STATE_GFE_ARM_MWX_JOINT );
  } else if ( id == "r_arm_shx" ){
    return _right_arm.joint( STATE_GFE_ARM_SHX_JOINT );
  } else if ( id == "r_arm_usy" ){
    return _right_arm.joint( STATE_GFE_ARM_USY_JOINT );
  } else if ( id == "r_arm_uwy" ){
    return _right_arm.joint( STATE_GFE_ARM_UWY_JOINT );
  } else if ( id == "left_f0_j0" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_0 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "left_f0_j1" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_0 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "left_f0_j2" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_0 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "left_f1_j0" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_1 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "left_f1_j1" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_1 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "left_f1_j2" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_1 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "left_f2_j0" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_2 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "left_f2_j1" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_2 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "left_f2_j2" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_2 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "left_f3_j0" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_3 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "left_f3_j1" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_3 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "left_f3_j2" ){
    return _left_hand.finger( STATE_GFE_HAND_FINGER_3 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "right_f0_j0" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_0 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "right_f0_j1" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_0 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "right_f0_j2" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_0 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "right_f1_j0" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_1 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "right_f1_j1" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_1 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "right_f1_j2" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_1 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "right_f2_j0" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_2 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "right_f2_j1" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_2 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "right_f2_j2" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_2 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "right_f3_j0" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_3 ).joint( STATE_GFE_FINGER_JOINT_0 );
  } else if ( id == "right_f3_j1" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_3 ).joint( STATE_GFE_FINGER_JOINT_1 );
  } else if ( id == "right_f3_j2" ){
    return _right_hand.finger( STATE_GFE_HAND_FINGER_3 ).joint( STATE_GFE_FINGER_JOINT_2 );
  } else if ( id == "l_leg_kny" ){
    return _left_leg.joint( STATE_GFE_LEG_KNY_JOINT );
  } else if ( id == "l_leg_lax" ){
    return _left_leg.joint( STATE_GFE_LEG_LAX_JOINT );
  } else if ( id == "l_leg_lhy" ){
    return _left_leg.joint( STATE_GFE_LEG_LHY_JOINT );
  } else if ( id == "l_leg_mhx" ){
    return _left_leg.joint( STATE_GFE_LEG_MHX_JOINT );
  } else if ( id == "l_leg_uay" ){
    return _left_leg.joint( STATE_GFE_LEG_UAY_JOINT );
  } else if ( id == "l_leg_uhz" ){
    return _left_leg.joint( STATE_GFE_LEG_UHZ_JOINT );
  } else if ( id == "r_leg_kny" ){
    return _right_leg.joint( STATE_GFE_LEG_KNY_JOINT );
  } else if ( id == "r_leg_lax" ){
    return _right_leg.joint( STATE_GFE_LEG_LAX_JOINT );
  } else if ( id == "r_leg_lhy" ){
    return _right_leg.joint( STATE_GFE_LEG_LHY_JOINT );
  } else if ( id == "r_leg_mhx" ){
    return _right_leg.joint( STATE_GFE_LEG_MHX_JOINT );
  } else if ( id == "r_leg_uay" ){
    return _right_leg.joint( STATE_GFE_LEG_UAY_JOINT );
  } else if ( id == "r_leg_uhz" ){
    return _right_leg.joint( STATE_GFE_LEG_UHZ_JOINT );
  }
 
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
    for( unsigned int i = 0; i < NUM_STATE_GFE_JOINTS; i++ ){
      out << other.joint( ( state_gfe_joint_t )( i ) ) << " ";
    }
    out << endl;
    out << " left_arm: " << other.left_arm();
    out << " right_arm: " << other.right_arm();
    out << " left_hand: " << other.left_hand();
    out << " right_hand: " << other.right_hand();
/*
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
*/
    return out;
  }
}
