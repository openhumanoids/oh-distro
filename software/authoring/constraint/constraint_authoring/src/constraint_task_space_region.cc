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
                              const pair< string, string >& parent,
                              AffordanceState* child ) : Constraint( id, active, start, end ),
                                                          _contact_type( CONSTRAINT_TASK_SPACE_REGION_WITHIN_REGION_CONTACT_TYPE ), 
                                                          _ranges( 6 ),
                                                          _parent( parent ),
                                                          _child( child ),
                                                          _parent_to_constraint(),
                                                          _child_to_constraint() {

}

Constraint_Task_Space_Region::
~Constraint_Task_Space_Region() {

}

Constraint_Task_Space_Region::
Constraint_Task_Space_Region( const Constraint_Task_Space_Region& other ) : Constraint( other ),
                                                                            _contact_type( other._contact_type ),
                                                                            _ranges( other._ranges ),
                                                                            _parent_to_constraint( other._parent_to_constraint ),
                                                                            _child_to_constraint( other._child_to_constraint ) {

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
  _parent_to_constraint = other._parent_to_constraint;
  _child_to_constraint = other._child_to_constraint;
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

//split affName by slashes.  should be of the form: linkName/groupName/objName
//we lump groupName/objName into 1 string
void getLinkGroupObjSplits(const string &affName, string &linkName, string &groupObjNames)
{
     vector<string> nameSplit = affordance::ToString::split(affName, '/');
     linkName = nameSplit[0];
     if (nameSplit.size() == 1)
       {
         cout << "\n\n\n name didn't have a slash\n\n" << endl;
         groupObjNames = "N/A";
         return;
       }

     groupObjNames = "";
     for(uint i = 1; i < nameSplit.size(); i++)
       {
         groupObjNames += nameSplit[i];
         if (i < nameSplit.size() - 1)
           groupObjNames += "/";
       }
}

void 
Constraint_Task_Space_Region::
add_to_drc_action_sequence_t( drc::action_sequence_t& actionSequence ){
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
      getLinkGroupObjSplits(_child->getName(), linkName, groupObjNames);
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
      getLinkGroupObjSplits(_child->getName(), linkName, groupObjNames);
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
  return;
}

string
Constraint_Task_Space_Region::
description( void )const{
  string description;
  description.append( "P:\"" + _parent.first + "-" + _parent.second + "\" " );
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
    out << "range[6]: {";
    for( unsigned int i = 0; i < 6; i++ ){
      out << "(" << other.ranges()[i].first << "," << other.ranges()[i].second << ")";
      if( i != 5 ){
        out << ",";
      }
    }
    out << "} ";
    out << "parent_to_constraint[6]: (" << other.parent_to_constraint().p[0] << "," << other.parent_to_constraint().p[1] << "," << other.parent_to_constraint().p[2] << ") ";
    out << "child_to_constraint[6]: (" << other.child_to_constraint().p[0] << "," << other.child_to_constraint().p[1] << "," << other.child_to_constraint().p[2] << ") ";
    return out;
  }
}
