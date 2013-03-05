#ifndef STATE_STATE_GFE_FINGER_H
#define STATE_STATE_GFE_FINGER_H

#include <iostream>

#include <state/state.h>
#include <state/state_gfe_joint.h>

namespace state {
  typedef enum {
    STATE_GFE_FINGER_JOINT_0,
    STATE_GFE_FINGER_JOINT_1,
    STATE_GFE_FINGER_JOINT_2,
    NUM_STATE_GFE_FINGER_JOINTS
  } state_gfe_finger_joint_t;

  class State_GFE_Finger : public State {
  public:
    State_GFE_Finger( std::string jointIdPrefix = "f0_" );
    ~State_GFE_Finger();
    State_GFE_Finger( const State_GFE_Finger& other );
    State_GFE_Finger& operator=( const State_GFE_Finger& other );

    static State_GFE_Finger interpolate( const State_GFE_Finger& first, const State_GFE_Finger& second, unsigned long long time );

    virtual void set_id( std::string id );
    virtual void set_time( unsigned long long time );

    State_GFE_Joint& joint( state_gfe_finger_joint_t joint );
    const State_GFE_Joint& joint( state_gfe_finger_joint_t joint )const;

  protected:
    State_GFE_Joint _joints[ NUM_STATE_GFE_FINGER_JOINTS ];
  private:

  };
  std::ostream& operator<<( std::ostream& out, const State_GFE_Finger& other );
}

#endif /* STATE_STATE_GFE_FINGER_H */
