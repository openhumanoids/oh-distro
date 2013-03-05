#ifndef STATE_STATE_GFE_HAND_H
#define STATE_STATE_GFE_HAND_H

#include <iostream>

#include <state/state.h>
#include <state/state_gfe_finger.h>

namespace state {
  typedef enum {
    STATE_GFE_HAND_FINGER_0,
    STATE_GFE_HAND_FINGER_1,
    STATE_GFE_HAND_FINGER_2,
    STATE_GFE_HAND_FINGER_3,
    NUM_STATE_GFE_HAND_FINGERS
  } state_gfe_finger_t;

  class State_GFE_Hand : public State {
  public:
    State_GFE_Hand( std::string jointIdPrefix = "left_" );
    ~State_GFE_Hand();
    State_GFE_Hand( const State_GFE_Hand& other );
    State_GFE_Hand& operator=( const State_GFE_Hand& other );

    static State_GFE_Hand interpolate( const State_GFE_Hand& first, const State_GFE_Hand& second, unsigned long long time );

    virtual void set_time( unsigned long long time );
  
    State_GFE_Finger& finger( state_gfe_finger_t finger );
    const State_GFE_Finger& finger( state_gfe_finger_t finger )const;

  protected:
    State_GFE_Finger _fingers[ NUM_STATE_GFE_HAND_FINGERS ];

  private:

  };
  std::ostream& operator<<( std::ostream& out, const State_GFE_Hand& other );
}

#endif /* STATE_STATE_GFE_HAND_H */
