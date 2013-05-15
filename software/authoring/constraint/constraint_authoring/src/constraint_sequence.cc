#include <string.h>
#include <string>

#include "authoring/constraint_task_space_region.h"
#include "authoring/constraint_sequence.h"

using namespace std;
using namespace boost;
using namespace lcm;
using namespace urdf;
using namespace drc;
using namespace state;
using namespace affordance;
using namespace authoring;

Constraint_Sequence::
Constraint_Sequence() : _constraints(),
                        _q0() {

}

Constraint_Sequence::
~Constraint_Sequence() {

}

Constraint_Sequence::
Constraint_Sequence( const Constraint_Sequence& other ) : _constraints( other._constraints ),
                                                          _q0( other._q0 ) {

}

Constraint_Sequence&
Constraint_Sequence::
operator=( const Constraint_Sequence& other ) {
  _constraints = other._constraints;
  _q0 = other._q0;
  return (*this);
}

void
Constraint_Sequence::
load( string filename, 
      vector< affordance::AffordanceState >& affordanceCollection ){
  LogFile lFileReader( filename, "r" );
  const LogEvent *eventFromFile = lFileReader.readNextEvent();
  action_sequence_t msg;
  msg.decode( eventFromFile->data,0,eventFromFile->datalen );
  from_msg( msg, affordanceCollection );
  return;
}

void
Constraint_Sequence::
save( string filename ){
  action_sequence_t msg;
  to_msg( msg );
  void * buffer = malloc(msg.getEncodedSize());
  msg.encode(buffer,0,msg.getEncodedSize());
  LogEvent logEvent;
  logEvent.eventnum = 0;
  logEvent.timestamp = 0;
  logEvent.channel = "action_sequence_gui_io";
  logEvent.datalen = msg.getEncodedSize();
  logEvent.data = buffer;
  LogFile lFileWriter( filename.c_str(), "w" );
  lFileWriter.writeEvent( &logEvent );
  free( buffer );
  return;
}

void
Constraint_Sequence::
to_msg( action_sequence_t& msg ){ 
  msg.utime = 0;
  msg.num_contact_goals = 0;
  msg.contact_goals.clear();
  msg.robot_name = "atlas";
  _q0.to_lcm( &msg.q0 );
  for( vector< Constraint* >::iterator it = _constraints.begin(); it != _constraints.end(); it++ ){
    if( (*it) != NULL ){
      (*it)->add_to_drc_action_sequence_t( msg );
    }
  }
  return;
}

void
Constraint_Sequence::
from_msg( const action_sequence_t& msg,
          vector< AffordanceState >& affordanceCollection ){
  _q0.from_lcm( &msg.q0 );
  _constraints.clear();
  for( unsigned int i = 0; i < ( msg.num_contact_goals/2 ); i++ ){
    char buffer[ 80 ];
    sprintf( buffer, "C%03d", i );
    Constraint_Task_Space_Region* constraint = new Constraint_Task_Space_Region( buffer, true, msg.contact_goals[2*i].lower_bound_completion_time, msg.contact_goals[2*i].upper_bound_completion_time, pair< string, string >( msg.contact_goals[2*i].object_1_name, msg.contact_goals[2*i].object_1_contact_grp ) );
    if( msg.contact_goals[2*i].contact_type == contact_goal_t::ON_GROUND_PLANE ){
      constraint->contact_type() = CONSTRAINT_TASK_SPACE_REGION_ON_GROUND_PLANE_CONTACT_TYPE; 
    } else if ( msg.contact_goals[2*i].contact_type == contact_goal_t::POINT_CONTACT ){
      constraint->contact_type() = CONSTRAINT_TASK_SPACE_REGION_POINT_CONTACT_CONTACT_TYPE;
    } else if ( msg.contact_goals[2*i].contact_type == contact_goal_t::FORCE_CLOSURE ){
      constraint->contact_type() = CONSTRAINT_TASK_SPACE_REGION_FORCE_CLOSURE_CONTACT_TYPE;
    }
    constraint->ranges()[0].first = msg.contact_goals[2*i].x_offset;
    constraint->ranges()[1].first = msg.contact_goals[2*i].y_offset;
    constraint->ranges()[2].first = msg.contact_goals[2*i].z_offset;
    constraint->ranges()[0].second = msg.contact_goals[2*i+1].x_offset;
    constraint->ranges()[1].second = msg.contact_goals[2*i+1].y_offset;
    constraint->ranges()[2].second = msg.contact_goals[2*i+1].z_offset;
    for( vector< AffordanceState >::iterator it = affordanceCollection.begin(); it != affordanceCollection.end(); it++ ){
      if( it->getName() == msg.contact_goals[2*i].object_2_name ){
        constraint->child() = &(*it);
      }
    }
    _constraints.push_back( constraint );
  }
  return;
}

void
Constraint_Sequence::
print_msg( const action_sequence_t& msg ){
  cout << "utime:{" << msg.utime << "} ";
  cout << "robot_name:{" << msg.robot_name << "} ";
  cout << "contact_goals[" << msg.num_contact_goals << "]:{";
  for( vector< contact_goal_t >::const_iterator it = msg.contact_goals.begin(); it != msg.contact_goals.end(); it++ ){
    cout << "{(" << it->lower_bound_completion_time << "," << it->upper_bound_completion_time << "),";
    cout << "(" << it->object_1_name << "," << it->object_1_contact_grp << "),";
    cout << "(" << it->object_2_name << "," << it->object_2_contact_grp << "),";
    if( it->contact_type == contact_goal_t::ON_GROUND_PLANE ){
      cout << "(ON_GROUND_PLANE),";
    } else if ( it->contact_type == contact_goal_t::POINT_CONTACT ){
      cout << "(POINT_CONTACT),";
    } else if ( it->contact_type == contact_goal_t::FORCE_CLOSURE ){
      cout << "(FORCE_CLOSURE),";
    }
    if( it->x_relation == contact_goal_t::REL_EQUAL ){
      cout << "(x=" << it->x_offset << "),";
    } else if( it->x_relation == contact_goal_t::REL_LESS_THAN ){
      cout << "(x<" << it->x_offset << "),";
    } else if ( it->x_relation == contact_goal_t::REL_GREATER_THAN ){
      cout << "(x>" << it->x_offset << "),";
    }
    if( it->y_relation == contact_goal_t::REL_EQUAL ){
      cout << "(y=" << it->y_offset << "),";
    } else if( it->y_relation == contact_goal_t::REL_LESS_THAN ){
      cout << "(y<" << it->y_offset << "),";
    } else if ( it->y_relation == contact_goal_t::REL_GREATER_THAN ){
      cout << "(y>" << it->y_offset << "),";
    }
    if( it->z_relation == contact_goal_t::REL_EQUAL ){
      cout << "(z=" << it->z_offset << ")";
    } else if( it->z_relation == contact_goal_t::REL_LESS_THAN ){
      cout << "(z<" << it->z_offset << ")";
    } else if ( it->z_relation == contact_goal_t::REL_GREATER_THAN ){
      cout << "(z>" << it->z_offset << ")";
    }
    cout << "}";
  }
  cout << "}" << endl;
  return;
}

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const Constraint_Sequence& other ) {
    return out;
  }

}
