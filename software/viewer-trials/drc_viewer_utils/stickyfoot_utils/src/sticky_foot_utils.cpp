
#include "sticky_foot_utils.hpp"

using namespace std;
namespace visualization_utils
{
  bool is_sticky_foot_condition_active(const StickyFootStruc &foot_struc,boost::shared_ptr<visualization_utils::AffordanceCollectionManager>  &affCollectionManager)
  { 
    if(foot_struc.is_conditional)
    {
      object_instance_map_type_::iterator obj_it = affCollectionManager->_objects.find(string(foot_struc.object_name));
      double current_dof_pos, current_dof_vel;
      obj_it->second._otdf_instance->getJointState (foot_struc.conditioned_parent_joint_name, current_dof_pos, current_dof_vel);
      bool cond=false;
      if(foot_struc.cond_type==foot_struc.GT)
         cond =  (current_dof_pos >= foot_struc.conditioned_parent_joint_val);
      else if (foot_struc.cond_type==foot_struc.LT)
         cond =  (current_dof_pos <= foot_struc.conditioned_parent_joint_val);
        
      return cond;
    }
    else {
      return true;
    }
  }
}//end_namespace



