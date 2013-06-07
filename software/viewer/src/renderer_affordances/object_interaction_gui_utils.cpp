#include "object_interaction_gui_utils.hpp"
#include <tinyxml.h>

void renderer_affordances_gui_utils::store_sticky_hand(BotGtkParamWidget *pw, const char *name,void *user,bool unstore)
{

  RendererAffordances *self = (RendererAffordances*) user;

  typedef map<string, StickyHandStruc > sticky_hands_map_type_;
  sticky_hands_map_type_::iterator hand_it = self->sticky_hands.find(self->stickyhand_selection);

  typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
  object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(string(hand_it->second.object_name));

  // get otdf name
  std::string otdf_models_path = std::string(getModelsPath()) + "/otdf/"; 
  std::string filepath =  otdf_models_path + obj_it->second.otdf_type +".otdf";

  // get variables to create grasp_seed xml
  GraspSeed graspSeed;
  graspSeed.appType = GraspSeed::HAND;
  KDL::Frame& geo = hand_it->second.T_geometry_hand;
  graspSeed.xyz[0] = geo.p.x(), graspSeed.xyz[1] = geo.p.y(), graspSeed.xyz[2] = geo.p.z();
  geo.M.GetRPY(graspSeed.rpy[0],graspSeed.rpy[1],graspSeed.rpy[2]);
  graspSeed.geometry_name = hand_it->second.geometry_name;
  graspSeed.grasp_type = hand_it->second.hand_type;
  graspSeed.joint_names = hand_it->second.joint_name;
  graspSeed.joint_positions = hand_it->second.joint_position;
  if(unstore) graspSeed.unstoreFromOtdf(filepath);
  else        graspSeed.writeToOtdf(filepath);

}  

void renderer_affordances_gui_utils::store_sticky_feet(BotGtkParamWidget *pw, const char *name,void *user,bool unstore)
{

  RendererAffordances *self = (RendererAffordances*) user;

  typedef map<string, StickyFootStruc > sticky_feet_map_type_;
  sticky_feet_map_type_::iterator feet_it = self->sticky_feet.find(self->stickyfoot_selection);

  typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
  object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(string(feet_it->second.object_name));

  // get otdf name
  std::string otdf_models_path = std::string(getModelsPath()) + "/otdf/"; 
  std::string filepath =  otdf_models_path + obj_it->second.otdf_type +".otdf";

  // get variables to create grasp_seed xml
  GraspSeed graspSeed;
  graspSeed.appType = GraspSeed::FOOT;
  KDL::Frame& geo = feet_it->second.T_geometry_foot;
  graspSeed.xyz[0] = geo.p.x(), graspSeed.xyz[1] = geo.p.y(), graspSeed.xyz[2] = geo.p.z();
  geo.M.GetRPY(graspSeed.rpy[0],graspSeed.rpy[1],graspSeed.rpy[2]);
  graspSeed.geometry_name = feet_it->second.geometry_name;
  graspSeed.grasp_type = feet_it->second.foot_type;
  graspSeed.joint_names = feet_it->second.joint_name;
  graspSeed.joint_positions = feet_it->second.joint_position;
  if(unstore) graspSeed.unstoreFromOtdf(filepath);
  else        graspSeed.writeToOtdf(filepath);

}  

