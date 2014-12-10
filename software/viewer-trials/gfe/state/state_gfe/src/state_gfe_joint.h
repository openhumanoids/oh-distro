#ifndef STATE_GFE_STATE_GFE_JOINT_H
#define STATE_GFE_STATE_GFE_JOINT_H

#include <iostream>
#include <string>

#include <state/state.h>

namespace state {
  class State_GFE_Joint : public State {
  public:
    State_GFE_Joint( std::string id = "gfe-joint", unsigned long long time = 0, double position = 0.0, double velocity = 0.0, double effort = 0.0 );
    ~State_GFE_Joint();
    State_GFE_Joint( const State_GFE_Joint& other );
    State_GFE_Joint& operator=( const State_GFE_Joint& other );

    static State_GFE_Joint interpolate( const State_GFE_Joint& first, const State_GFE_Joint& second, unsigned long long time );

    void set_position( double position );
    void set_velocity( double velocity );
    void set_effort( double effort );

    double position( void )const;
    double velocity( void )const;
    double effort( void )const; 
 
  protected:
    double _position;
    double _velocity;
    double _effort;

  private:
    
  };
  std::ostream& operator<<( std::ostream& out, const State_GFE_Joint& other );
}

#endif /* STATE_GFE_STATE_GFE_JOINT_H */
