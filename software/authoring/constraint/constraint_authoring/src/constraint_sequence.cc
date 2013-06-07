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
Constraint_Sequence( unsigned int numConstraints ) : _constraints( numConstraints ),
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
from_xml( const string& filename ){
  xmlDoc * doc = NULL;
  doc = xmlReadFile( filename.c_str(), NULL, 0 );
  from_xml( xmlDocGetRootElement( doc ) );
  xmlFreeDoc( doc );
  return;
}

void
Constraint_Sequence::
from_xml( xmlNodePtr root ){
  for( vector< Constraint_Task_Space_Region >::iterator it = _constraints.begin(); it != _constraints.end(); it++ ){
    it->active() = false;
  }

  vector< Constraint_Task_Space_Region >::iterator it = _constraints.begin();
  if( root->type == XML_ELEMENT_NODE ){
    xmlNodePtr l1 = NULL;
    for( l1 = root->children; l1; l1 = l1->next ){
      if( l1->type == XML_ELEMENT_NODE ){
        if( xmlStrcmp( l1->name, ( const xmlChar* )( "constraint_task_space_region" ) ) == 0 ){
          cout << "found constraint task space region" << endl;
          it->from_xml( l1 );
          it++;
          if( it == _constraints.end() ){
            return;
          }
        }         
      }
    }
  }
  return;
}

void
Constraint_Sequence::
to_xml( const string& filename )const{
  ofstream out;
  out.open( filename.c_str() );
  to_xml( out );
  out.close();
  return;
}

void
Constraint_Sequence::
to_xml( ofstream& out,
        unsigned int indent )const{
  out << string( indent, ' ' ) << "<constraint_sequence>" << endl;
  for( vector< Constraint_Task_Space_Region >::const_iterator it = _constraints.begin(); it != _constraints.end(); it++ ){
    if( it->active() ){
      it->to_xml( out, indent + 2 );
    }   
  }
  out << string( indent, ' ' ) << "</constraint_sequence>" << endl;
  return;
}

void
Constraint_Sequence::
to_msg( action_sequence_t& msg, 
        vector< AffordanceState >& affordanceCollection ){ 
  msg.utime = 0;
  msg.num_contact_goals = 0;
  msg.contact_goals.clear();
  msg.robot_name = "atlas";
  _q0.to_lcm( &msg.q0 );
  for( vector< Constraint_Task_Space_Region >::iterator it = _constraints.begin(); it != _constraints.end(); it++ ){
    cout << it->id() << ": " << it->active() << endl;
    if( it->active() ) {
      it->add_to_drc_action_sequence_t( msg, affordanceCollection );
    }
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
    if( it->contact_type == contact_goal_t::WITHIN_REGION ){
      cout << "(WITHIN_REGION),";
    } else if ( it->contact_type == contact_goal_t::SUPPORTED_WITHIN_REGION ){
      cout << "(SUPPORTED_WITHIN_REGION),";
    } else {
      cout << "(UNKNOWN),";
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
