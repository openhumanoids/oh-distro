#ifndef STATE_GFE_STATE_GFE_H
#define STATE_GFE_STATE_GFE_H

#include <iostream>
#include <string>

#include <kdl/tree.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <urdf/model.h>

#include <state/state.h>
#include <state/state_gfe_joint.h>
#include <state/state_gfe_arm.h>
#include <state/state_gfe_leg.h>

namespace state {
  typedef enum {
    STATE_GFE_BACK_LBZ_JOINT,
    STATE_GFE_BACK_MBY_JOINT,
    STATE_GFE_BACK_UBX_JOINT,
    STATE_GFE_NECK_AY_JOINT,
    STATE_GFE_HEAD_IMU_JOINT,
    STATE_GFE_HOKUYO_JOINT,
    STATE_GFE_IMU_JOINT,
    NUM_STATE_GFE_JOINTS
  } state_gfe_joint_t;

  class State_GFE : public State {
  public:
    State_GFE();
    ~State_GFE();
    State_GFE( const State_GFE& other );
    State_GFE& operator=( const State_GFE& other );

    static State_GFE interpolate( const State_GFE& first, const State_GFE& second, unsigned long long time );
 
    bool from_lcm( const drc::robot_state_t& robotState ); 
    bool from_lcm( const drc::robot_state_t* robotState );
    void to_lcm( drc::robot_state_t* robotState )const;

    bool from_urdf( std::string filename );

    virtual void set_time( unsigned long long time );
    void set_pose( const KDL::Frame& pose );

    KDL::Frame pose( void )const;
    std::map< std::string, State_GFE_Joint > joints( void )const;;
    std::map< std::string, double > joint_angles( void )const; 
    State_GFE_Arm& left_arm( void );
    const State_GFE_Arm& left_arm( void )const;
    State_GFE_Arm& right_arm( void );
    const State_GFE_Arm& right_arm( void )const;
    State_GFE_Hand& left_hand( void );
    const State_GFE_Hand& left_hand( void )const;
    State_GFE_Hand& right_hand( void );
    const State_GFE_Hand& right_hand( void )const;
    State_GFE_Leg& left_leg( void );
    const State_GFE_Leg& left_leg( void )const;
    State_GFE_Leg& right_leg( void );
    const State_GFE_Leg& right_leg( void )const;
    State_GFE_Joint& joint( state_gfe_joint_t joint );
    const State_GFE_Joint& joint( state_gfe_joint_t joint )const;
    State_GFE_Joint& joint( std::string id );
    const State_GFE_Joint& joint( std::string id )const;
 
  protected:
    KDL::Frame _pose;
    State_GFE_Arm _left_arm;
    State_GFE_Arm _right_arm;
    State_GFE_Hand _left_hand;
    State_GFE_Hand _right_hand;
    State_GFE_Leg _left_leg;
    State_GFE_Leg _right_leg;
    State_GFE_Joint _joints[ NUM_STATE_GFE_JOINTS ];

  private:
    
  };
  std::ostream& operator<<( std::ostream& out, const State_GFE& other );
}

#endif /* STATE_GFE_STATE_GFE_H */
