#include <boost/algorithm/string.hpp>
#include "authoring/constraint_task_space_region.h"

using namespace std;
using namespace boost;
using namespace drc;
using namespace urdf;
using namespace affordance;
using namespace authoring;

Constraint_Task_Space_Region::
Constraint_Task_Space_Region( const string& id,
                              bool active,
                              double start,
                              double end,
                              const string& parent,
                              AffordanceState* child,
                              contact_type_t contactType ) : Constraint( id, active, start, end ),
                                                              _contact_type( contactType ), 
                                                              _ranges( NUM_CONSTRAINT_TASK_SPACE_REGION_RANGES, pair< bool, double >( true, 0.0 ) ),
                                                              _parents(),
                                                              _child( child ),
                                                              _offset() {
  _parents.push_back( parent );
}

Constraint_Task_Space_Region::
~Constraint_Task_Space_Region() {

}

Constraint_Task_Space_Region::
Constraint_Task_Space_Region( const Constraint_Task_Space_Region& other ) : Constraint( other ),
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
  _start = other._start;
  _end = other._end;
  _contact_type = other._contact_type;
  _ranges = other._ranges;
  _parents = other._parents;
  _child = other._child;
  _offset = other._offset;
  return (*this);
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
  default:
    return "UNKNOWN";
    break;
  }
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

  string affordance_link_name = "N/A";
  string affordance_group_name = "N/A";
  if( _child == NULL ){
    cout << "could not export constraint " << _id << " to xml because child is NULL" << endl;
  } else {
    _child->splitNameIntoLinkGroup( affordance_link_name, affordance_group_name ); 
    string child_string = affordance_link_name + '-' + affordance_group_name;

    out << string( indent, ' ' ) << "<constraint_task_space_region id=\"" << _id << "\" active=\"" << ( _active ? "T" : "F" ) << "\" start=\"" << _start << "\" end=\"" << _end << "\" contact_type=\"" << contact_type_t_to_std_string( _contact_type ) << "\" parents=\"" << parents_string << "\" child=\"" << child_string << "\">" << endl;
    out << string( indent + 2, ' ' ) << "<xmin active=\"" << ( _ranges[ CONSTRAINT_TASK_SPACE_REGION_X_MIN_RANGE ].first ? "T" : "F" ) << "\" value=\"" << _ranges[ CONSTRAINT_TASK_SPACE_REGION_X_MIN_RANGE ].second << "\"/>" << endl; 
    out << string( indent + 2, ' ' ) << "<xmax active=\"" << ( _ranges[ CONSTRAINT_TASK_SPACE_REGION_X_MAX_RANGE ].first ? "T" : "F" ) << "\" value=\"" << _ranges[ CONSTRAINT_TASK_SPACE_REGION_X_MAX_RANGE ].second << "\"/>" << endl; 
    out << string( indent + 2, ' ' ) << "<ymin active=\"" << ( _ranges[ CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE ].first ? "T" : "F" ) << "\" value=\"" << _ranges[ CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE ].second << "\"/>" << endl; 
    out << string( indent + 2, ' ' ) << "<ymax active=\"" << ( _ranges[ CONSTRAINT_TASK_SPACE_REGION_Y_MAX_RANGE ].first ? "T" : "F" ) << "\" value=\"" << _ranges[ CONSTRAINT_TASK_SPACE_REGION_Y_MAX_RANGE ].second << "\"/>" << endl; 
    out << string( indent + 2, ' ' ) << "<zmin active=\"" << ( _ranges[ CONSTRAINT_TASK_SPACE_REGION_Z_MIN_RANGE ].first ? "T" : "F" ) << "\" value=\"" << _ranges[ CONSTRAINT_TASK_SPACE_REGION_Z_MIN_RANGE ].second << "\"/>" << endl; 
    out << string( indent + 2, ' ' ) << "<zmax active=\"" << ( _ranges[ CONSTRAINT_TASK_SPACE_REGION_Z_MAX_RANGE ].first ? "T" : "F" ) << "\" value=\"" << _ranges[ CONSTRAINT_TASK_SPACE_REGION_Z_MAX_RANGE ].second << "\"/>" << endl; 
    out << string( indent, ' ' ) << "</constraint_task_space_region>" << endl;
  }
  return;
}

void 
Constraint_Task_Space_Region::
add_to_drc_action_sequence_t( drc::action_sequence_t& actionSequence ){
  if( _child == NULL ){
    cout << "could not add_to_drc_action_sequence_t because child == NULL" << endl;
    return;
  } else {
    string affordance_link_name = "N/A";
    string affordance_group_name = "N/A";
    _child->splitNameIntoLinkGroup( affordance_link_name, affordance_group_name );
    for( vector< string >::iterator it_parent = _parents.begin(); it_parent != _parents.end(); it_parent++ ){
      vector< string > parent_strings;
      boost::split( parent_strings, (*it_parent ), boost::is_any_of( "-" ) );
      if( parent_strings.size() != 2 ){
        cout << "could not add_to_drc_action_sequence_t because parent name not splitable" << endl;
        return;
      } else {  
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
            actionSequence.contact_goals.back().target_pt.x = _child->getXYZ().x();
            actionSequence.contact_goals.back().target_pt.y = _child->getXYZ().y();
            actionSequence.contact_goals.back().target_pt.z = _child->getXYZ().z();
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
/*
  actionSequence.num_contact_goals += 2;
  contact_goal_t contact_goal;
  actionSequence.contact_goals.push_back( contact_goal );  
  actionSequence.contact_goals.back().utime = 0;
  actionSequence.contact_goals.back().object_1_name = _parent.first;
  actionSequence.contact_goals.back().object_1_contact_grp = _parent.second;

  if( _child != NULL )
    {
      //see if the the child name has any slashes: link/group/item
      string linkName,groupObjNames;
      _child->splitNameIntoLinkGroup(linkName, groupObjNames);
      actionSequence.contact_goals.back().object_2_name = linkName;
      actionSequence.contact_goals.back().object_2_contact_grp = groupObjNames;
    } 
  else 
    {
      cout << "\n\n child was null\n" << endl;
      actionSequence.contact_goals.back().object_2_name = "N/A";
      actionSequence.contact_goals.back().object_2_contact_grp = "N/A";
    }

  actionSequence.contact_goals.back().lower_bound_completion_time = _start;
  actionSequence.contact_goals.back().upper_bound_completion_time = _end;
  actionSequence.contact_goals.back().contact_type = _contact_type;
  if( _child != NULL ){
    actionSequence.contact_goals.back().target_pt.x = _child->getXYZ().x();
    actionSequence.contact_goals.back().target_pt.y = _child->getXYZ().y();
    actionSequence.contact_goals.back().target_pt.z = _child->getXYZ().z();
  } else {
    actionSequence.contact_goals.back().target_pt.x = 0.0;
    actionSequence.contact_goals.back().target_pt.y = 0.0;
    actionSequence.contact_goals.back().target_pt.z = 0.0;
  }
  actionSequence.contact_goals.back().x_offset = _ranges[ 0 ].first;
  actionSequence.contact_goals.back().y_offset = _ranges[ 1 ].first;
  actionSequence.contact_goals.back().z_offset = _ranges[ 2 ].first;
  actionSequence.contact_goals.back().x_relation = contact_goal_t::REL_GREATER_THAN;
  actionSequence.contact_goals.back().y_relation = contact_goal_t::REL_GREATER_THAN;
  actionSequence.contact_goals.back().z_relation = contact_goal_t::REL_GREATER_THAN;
  actionSequence.contact_goals.push_back( contact_goal ); 
  actionSequence.contact_goals.back().utime = 0;
  actionSequence.contact_goals.back().object_1_name = _parent.first;
  actionSequence.contact_goals.back().object_1_contact_grp = _parent.second;
  if( _child != NULL )
    {
      string linkName,groupObjNames;
      _child->splitNameIntoLinkGroup(linkName, groupObjNames);
      actionSequence.contact_goals.back().object_2_name = linkName;
      actionSequence.contact_goals.back().object_2_contact_grp = groupObjNames;
    } 
  else 
    {
      actionSequence.contact_goals.back().object_2_name = "N/A";
      actionSequence.contact_goals.back().object_2_contact_grp = "N/A";
    }
  actionSequence.contact_goals.back().lower_bound_completion_time = _start;
  actionSequence.contact_goals.back().upper_bound_completion_time = _end;
  actionSequence.contact_goals.back().contact_type = _contact_type;
  if( _child != NULL ){
    actionSequence.contact_goals.back().target_pt.x = _child->getXYZ().x();
    actionSequence.contact_goals.back().target_pt.y = _child->getXYZ().y();
    actionSequence.contact_goals.back().target_pt.z = _child->getXYZ().z();
  } else {
    actionSequence.contact_goals.back().target_pt.x = 0.0;
    actionSequence.contact_goals.back().target_pt.y = 0.0;
    actionSequence.contact_goals.back().target_pt.z = 0.0;
  }
  actionSequence.contact_goals.back().x_offset = _ranges[ 0 ].second;
  actionSequence.contact_goals.back().y_offset = _ranges[ 1 ].second;
  actionSequence.contact_goals.back().z_offset = _ranges[ 2 ].second;
  actionSequence.contact_goals.back().x_relation = contact_goal_t::REL_LESS_THAN;
  actionSequence.contact_goals.back().y_relation = contact_goal_t::REL_LESS_THAN;
  actionSequence.contact_goals.back().z_relation = contact_goal_t::REL_LESS_THAN;
*/ 
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
  if( _child != NULL ){
    description.append( "C:\"" + _child->getName() + "\" " );
  } else {
    description.append( "C:\"N/A\" " );
  }
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
    out << "} ";
    out << "offset[6]: (" << other.offset().p[0] << "," << other.offset().p[1] << "," << other.offset().p[2] << ") ";
    return out;
  }
}
