#include <boost/algorithm/string.hpp>
#include "authoring/constraint_task_space_region.h"

using namespace std;
using namespace boost;
using namespace KDL;
using namespace drc;
using namespace urdf;
using namespace affordance;
using namespace authoring;

Constraint_Task_Space_Region::
Constraint_Task_Space_Region( const string& id,
                              bool active,
                              bool visible,
                              double start,
                              double end,
                              const string& metadata,
                              const vector< string >& parents,
                              const string& child,
                              relation_type_t relationType,
                              contact_type_t contactType,
                              const Frame& offset ) : Constraint( id, active, visible, start, end, metadata ),
                                                              _relation_type( relationType ),
                                                              _contact_type( contactType ), 
                                                              _ranges( NUM_CONSTRAINT_TASK_SPACE_REGION_RANGES, pair< bool, double >( true, 0.0 ) ),
                                                              _parents( parents ),
                                                              _child( child ),
                                                              _offset( offset ) {

}

Constraint_Task_Space_Region::
~Constraint_Task_Space_Region() {

}

Constraint_Task_Space_Region::
Constraint_Task_Space_Region( const Constraint_Task_Space_Region& other ) : Constraint( other ),
                                                                            _relation_type( other._relation_type ),
                                                                            _contact_type( other._contact_type ),
                                                                            _ranges( other._ranges ),
                                                                            _parents( other._parents ),
                                                                            _child( other._child ),
                                                                            _offset( other._offset ) {

}

Constraint_Task_Space_Region&
Constraint_Task_Space_Region::
operator=( const Constraint_Task_Space_Region& other ) {
  _id = other._id;
  _active = other._active;
  _visible = other._visible;
  _start = other._start;
  _end = other._end;
  _metadata = other._metadata;
  _contact_type = other._contact_type;
  _ranges = other._ranges;
  _parents = other._parents;
  _child = other._child;
  _offset = other._offset;
  return (*this);
}

string
Constraint_Task_Space_Region::
range_index_t_to_std_string( range_index_t rangeIndex ){
  switch( rangeIndex ){
  case ( CONSTRAINT_TASK_SPACE_REGION_X_MIN_RANGE ):
    return "X_MIN";
    break;
  case ( CONSTRAINT_TASK_SPACE_REGION_X_MAX_RANGE ):
    return "X_MAX";
    break;
  case ( CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE ):
    return "Y_MIN";
    break;
  case ( CONSTRAINT_TASK_SPACE_REGION_Y_MAX_RANGE ):
    return "Y_MAX";
    break;
  case ( CONSTRAINT_TASK_SPACE_REGION_Z_MIN_RANGE ):
    return "Z_MIN";
    break;
  case ( CONSTRAINT_TASK_SPACE_REGION_Z_MAX_RANGE ):
    return "Z_MAX";
    break;
  case ( NUM_CONSTRAINT_TASK_SPACE_REGION_RANGES ):
  default:
    return "UNKNOWN";
    break;
  }
}

string
Constraint_Task_Space_Region::
relation_type_t_to_std_string( relation_type_t relationType ){
  switch( relationType ){
  case ( CONSTRAINT_TASK_SPACE_REGION_ORIGIN_RELATION_TYPE ):
    return "ORIGIN";
    break;
  case ( CONSTRAINT_TASK_SPACE_REGION_AFFORDANCE_RELATION_TYPE ):
    return "AFFORDANCE";
    break;
  case ( CONSTRAINT_TASK_SPACE_REGION_AFFORDANCE_PLUS_PLANAR_PELVIS_RELATION_TYPE ):
    return "AFFORDANCE_PLUS_PLANAR_PELVIS";
    break;
  case ( NUM_CONSTRAINT_TASK_SPACE_REGION_RELATION_TYPES ):
  default:
    return "UNKNOWN";
    break;
  }
}

string
Constraint_Task_Space_Region::
contact_type_t_to_std_string( contact_type_t contactType ){
  switch( contactType ){
  case ( CONSTRAINT_TASK_SPACE_REGION_WITHIN_REGION_CONTACT_TYPE ):
    return "WITHIN_REGION";
    break;
  case ( CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE ):
    return "SUPPORTED_WITHIN_REGION";
    break;
  case ( NUM_CONSTRAINT_TASK_SPACE_REGION_CONTACT_TYPES ):
  default:
    return "UNKNOWN";
    break;
  }
}

void
Constraint_Task_Space_Region::
from_xml( xmlNodePtr root ){
  if( root->type == XML_ELEMENT_NODE ){
    xmlChar * tmp = xmlGetProp( root, ( const xmlChar* )( "id" ) );
    if( tmp != NULL ){
      _id = ( char* )( tmp );
      xmlFree( tmp );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "active" ) );
    if( tmp != NULL ){
      string active_string = ( char* )( tmp );
      _active = ( active_string == "T" );
      xmlFree( tmp );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "visible" ) );
    if( tmp != NULL ){
      string visible_string = ( char* )( tmp );
      _visible = ( visible_string == "T" );
      xmlFree( tmp );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "start" ) );
    if( tmp != NULL ){
      string start_string = ( char* )( tmp );
      _start = strtof( start_string.c_str(), NULL );
      xmlFree( tmp );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "end" ) );
    if( tmp != NULL ){
      string end_string = ( char* )( tmp );
      _end = strtof( end_string.c_str(), NULL );
      xmlFree( tmp );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "metadata" ) );
    if( tmp != NULL ){
      _metadata = ( char* )( tmp );
      xmlFree( tmp );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "relation_type" ) );
    if( tmp != NULL ){
      string relation_type_string = ( char* )( tmp );
      for( unsigned int i = 0; i < NUM_CONSTRAINT_TASK_SPACE_REGION_RELATION_TYPES; i++ ){
        if( relation_type_string == relation_type_t_to_std_string( ( relation_type_t )( i ) ) ){
          _relation_type = ( relation_type_t )( i );
        }
      }
      xmlFree( tmp );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "contact_type" ) );
    if( tmp != NULL ){
      string contact_type_string = ( char* )( tmp );
      for( unsigned int i = 0; i < NUM_CONSTRAINT_TASK_SPACE_REGION_CONTACT_TYPES; i++ ){
        if( contact_type_string == contact_type_t_to_std_string( ( contact_type_t )( i ) ) ){
          _contact_type = ( contact_type_t )( i );
        }
      }
      xmlFree( tmp );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "parents" ) );
    if( tmp != NULL ){
      string parents_string = ( char* )( tmp );
      boost::split( _parents, parents_string, boost::is_any_of( "," ) ); 
      xmlFree( tmp );
    } 
    tmp = xmlGetProp( root, ( const xmlChar* )( "child" ) );
    if( tmp != NULL ){
      _child = ( char* )( tmp );
      xmlFree( tmp );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "offset" ) );
    if( tmp != NULL ){
      string offset_string = ( char* )( tmp );
      vector< string > offset_strings;
      boost::split( offset_strings, offset_string, boost::is_any_of( "," ) );
      _offset.p = Vector( strtof( offset_strings[0].c_str(), NULL ), strtof( offset_strings[1].c_str(), NULL ), strtof( offset_strings[2].c_str(), NULL ) );
      xmlFree( tmp );
    }
    xmlNodePtr l1 = NULL;
    for( l1 = root->children; l1; l1 = l1->next ){
      if( l1->type == XML_ELEMENT_NODE ){
        range_index_t range_index;
        if( xmlStrcmp( l1->name, ( const xmlChar* )( "range" ) ) == 0 ){
          tmp = xmlGetProp( l1, ( const xmlChar* )( "index" ) );
          if( tmp != NULL ){
            string range_index_string = ( char* )( tmp );
            for( unsigned int i = 0; i < NUM_CONSTRAINT_TASK_SPACE_REGION_RANGES; i++ ){
              if( range_index_string == range_index_t_to_std_string( ( range_index_t )( i ) ) ){  
                range_index = ( range_index_t )( i );
              }
            }
            xmlFree( tmp );
          }
          tmp = xmlGetProp( l1, ( const xmlChar* )( "active" ) );
          if( tmp != NULL ){
            string active_string = ( char* )( tmp );
            _ranges[ range_index ].first = ( active_string == "T" ? true : false );
            xmlFree( tmp );
          }
          tmp = xmlGetProp( l1, ( const xmlChar* )( "value" ) );
          if( tmp != NULL ){
            string value_string = ( char* )( tmp );
            _ranges[ range_index ].second = strtof( value_string.c_str(), NULL );
            xmlFree( tmp );
          }
        }
      }
    }
  }
  return;
}

void
Constraint_Task_Space_Region::
to_xml( ofstream& out,
        unsigned int indent )const{
  string parents_string = "";
  for( unsigned int i = 0; i < _parents.size(); i++ ){
    parents_string.append( _parents[ i ] );
    if( i != ( _parents.size() - 1 ) ){
      parents_string.append( "," );
    }
  }

  out << string( indent, ' ' ) << "<constraint_task_space_region id=\"" << _id << "\" active=\"" << ( _active ? "T" : "F" ) << "\" visible=\"" << ( _visible ? "T" : "F" ) << "\" start=\"" << _start << "\" end=\"" << _end << "\" metadata=\"" << _metadata << "\" relation_type=\"" << relation_type_t_to_std_string( _relation_type ) << "\" contact_type=\"" << contact_type_t_to_std_string( _contact_type ) << "\" parents=\"" << parents_string << "\" child=\"" << _child << "\" offset=\"" << _offset.p[0] << "," << _offset.p[1] << "," << _offset.p[2] << "\">" << endl;
  for( unsigned int i = 0; i < NUM_CONSTRAINT_TASK_SPACE_REGION_RANGES; i++ ){
    out << string( indent + 2, ' ' ) << "<range index=\"" << range_index_t_to_std_string( ( range_index_t )( i ) ) << "\" active=\"" << ( _ranges[ i ].first ? "T" : "F" ) << "\" value=\"" << _ranges[ i ].second << "\"/>" << endl;
  }
  out << string( indent, ' ' ) << "</constraint_task_space_region>" << endl;
  return;
}

void 
Constraint_Task_Space_Region::
add_to_drc_action_sequence_t( action_sequence_t& actionSequence, 
                              vector< AffordanceState >& affordanceCollection ){

  AffordanceState * child = NULL;
  for( vector< AffordanceState >::iterator it = affordanceCollection.begin(); it != affordanceCollection.end(); it++ ){
    if( it->getName() == _child ){
      child = &(*it);
    }
  }

  if( child == NULL ){
    cout << "could not add_to_drc_action_sequence_t because child == NULL" << endl;
    return;
  } else {
    string affordance_link_name = "NA";
    string affordance_group_name = "default";
    child->splitNameIntoLinkGroup( affordance_link_name, affordance_group_name );
    for( vector< string >::iterator it_parent = _parents.begin(); it_parent != _parents.end(); it_parent++ ){
      vector< string > parent_strings;
      boost::split( parent_strings, (*it_parent ), boost::is_any_of( "-" ) );
      if( parent_strings.size() != 2 ){
        cout << "could not add_to_drc_action_sequence_t because parent name not splitable" << endl;
        return;
      } else {  
        Frame target_frame = Frame( KDL::Rotation::RPY( child->getRPY().x(), child->getRPY().y(), child->getRPY().z() ), KDL::Vector( child->getXYZ().x(), child->getXYZ().y(), child->getXYZ().z() ) ) * _offset;
        string parent_link_name = parent_strings[ 0 ];
        string parent_group_name = parent_strings[ 1 ];
        for( unsigned int i = 0; i < ( NUM_CONSTRAINT_TASK_SPACE_REGION_RANGES/3 ); i++ ){
          if( _active ){
            actionSequence.num_contact_goals++;
            contact_goal_t contact_goal;
            actionSequence.contact_goals.push_back( contact_goal );
            actionSequence.contact_goals.back().utime = 0;
            actionSequence.contact_goals.back().object_1_name = parent_link_name;
            actionSequence.contact_goals.back().object_1_contact_grp = parent_group_name;
            actionSequence.contact_goals.back().object_2_name = affordance_link_name;
            actionSequence.contact_goals.back().object_2_contact_grp = affordance_group_name;
            actionSequence.contact_goals.back().lower_bound_completion_time = _start;
            actionSequence.contact_goals.back().upper_bound_completion_time = _end;
            actionSequence.contact_goals.back().contact_type = _contact_type;
            actionSequence.contact_goals.back().target_pt.x = target_frame.p[0];
            actionSequence.contact_goals.back().target_pt.y = target_frame.p[1];
            actionSequence.contact_goals.back().target_pt.z = target_frame.p[2];
            if( i == 0 ){
              actionSequence.contact_goals.back().x_offset = ( _ranges[ CONSTRAINT_TASK_SPACE_REGION_X_MIN_RANGE ].first ? _ranges[ CONSTRAINT_TASK_SPACE_REGION_X_MIN_RANGE ].second : -1000.0 );
              actionSequence.contact_goals.back().y_offset = ( _ranges[ CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE ].first ? _ranges[ CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE ].second : -1000.0 );
              actionSequence.contact_goals.back().z_offset = ( _ranges[ CONSTRAINT_TASK_SPACE_REGION_Z_MIN_RANGE ].first ? _ranges[ CONSTRAINT_TASK_SPACE_REGION_Z_MIN_RANGE ].second : -1000.0 );
              actionSequence.contact_goals.back().x_relation = contact_goal_t::REL_GREATER_THAN;
              actionSequence.contact_goals.back().y_relation = contact_goal_t::REL_GREATER_THAN;
              actionSequence.contact_goals.back().z_relation = contact_goal_t::REL_GREATER_THAN;
            } else if ( i == 1 ){
              actionSequence.contact_goals.back().x_offset = ( _ranges[ CONSTRAINT_TASK_SPACE_REGION_X_MAX_RANGE ].first ? _ranges[ CONSTRAINT_TASK_SPACE_REGION_X_MAX_RANGE ].second : 1000.0 );
              actionSequence.contact_goals.back().y_offset = ( _ranges[ CONSTRAINT_TASK_SPACE_REGION_Y_MAX_RANGE ].first ? _ranges[ CONSTRAINT_TASK_SPACE_REGION_Y_MAX_RANGE ].second : 1000.0 );
              actionSequence.contact_goals.back().z_offset = ( _ranges[ CONSTRAINT_TASK_SPACE_REGION_Z_MAX_RANGE ].first ? _ranges[ CONSTRAINT_TASK_SPACE_REGION_Z_MAX_RANGE ].second : 1000.0 );
              actionSequence.contact_goals.back().x_relation = contact_goal_t::REL_LESS_THAN;
              actionSequence.contact_goals.back().y_relation = contact_goal_t::REL_LESS_THAN;
              actionSequence.contact_goals.back().z_relation = contact_goal_t::REL_LESS_THAN;
            }
          }
        } 
      }
    }
  }
  return;
}

string
Constraint_Task_Space_Region::
description( void )const{
  string description( "P:{" );
  for( unsigned int i = 0; i < _parents.size(); i++ ){
    description.append( _parents[ i ] );
    if( i != ( _parents.size() - 1 ) ){
      description.append( "," );
    } else {
      description.append( "} " );
    }
  } 
  description.append( "C:\"" + _child + "\" " );
  description.append( "CT:\"" + contact_type_t_to_std_string( _contact_type ) + "\" " );
  return description;
}

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const Constraint_Task_Space_Region& other ) {
    out << "id: " << other.id() << " ";
    out << "start: " << other.start() << " ";
    out << "end: " << other.end() << " ";
    out << "ranges[" << other.ranges().size() << "]: {";
    for( unsigned int i = 0; i < other.ranges().size(); i++ ){
      out << "(" << ( other.ranges()[i].first ? "T" : "F" ) << "," << other.ranges()[i].second << ")";
      if( i != ( other.ranges().size() - 1 ) ){
        out << ",";
      }
    }
    out << "}";
    return out;
  }
}
