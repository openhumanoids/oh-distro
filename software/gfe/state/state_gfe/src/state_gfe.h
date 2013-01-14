#ifndef STATE_GFE_STATE_GFE_H
#define STATE_GFE_STATE_GFE_H

#include <iostream>
#include <string>

#include <kdl/tree.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <urdf/model.h>

#include <state/state_gfe_joint.h>

namespace state {
  class State_GFE {
  public:
    State_GFE();
    ~State_GFE();
    State_GFE( const State_GFE& other );
    State_GFE& operator=( const State_GFE& other );

    bool from_urdf( std::string urdfFilename = "/mit_gazebo_models/mit_robot_PnC/model.urdf" );
    bool from_lcm( const drc::robot_state_t& robotState );
    void to_lcm( drc::robot_state_t& robotState )const;

    void set_id( std::string id );
    void set_time( unsigned long long time );
    void set_pose( const KDL::Frame& pose );

    std::string id( void )const;
    unsigned long long time( void )const;
    KDL::Frame pose( void )const;
    std::map< std::string, State_GFE_Joint > joints( void )const;
    std::map< std::string, double > joint_angles( void )const; 
    State_GFE_Joint& joint( std::string id );
    const State_GFE_Joint& joint( std::string id )const;
 
  protected:
    std::string _id;
    unsigned long long _time;
    KDL::Frame _pose;
    std::map< std::string, State_GFE_Joint > _joints;

  private:
    
  };
  std::ostream& operator<<( std::ostream& out, const State_GFE& other );
}

#endif /* STATE_GFE_STATE_GFE_H */
