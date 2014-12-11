#ifndef STATE_STATE_H
#define STATE_STATE_H

#include <iostream>
#include <string>

namespace state {
  class State {
  public:
    State( std::string id = "N/A", unsigned long long time = 0 );
    ~State();
    State( const State& other );
    State& operator=( const State& other );

    virtual void set_id( std::string id );
    virtual void set_time( unsigned long long time );

    std::string id( void )const;
    unsigned long long time( void )const;

  protected:
    std::string _id;
    unsigned long long _time;
  
  private:

  };
  std::ostream& operator<<( std::ostream& out, const State& other );
}

#endif /* STATE_STATE_H */
