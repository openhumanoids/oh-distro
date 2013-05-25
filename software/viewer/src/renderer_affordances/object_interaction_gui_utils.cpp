#include "object_interaction_gui_utils.hpp"
#include <tinyxml.h>

void renderer_affordances_gui_utils::store_sticky_hand(BotGtkParamWidget *pw, const char *name,void *user)
{

  RendererAffordances *self = (RendererAffordances*) user;

  typedef map<string, StickyHandStruc > sticky_hands_map_type_;
  sticky_hands_map_type_::iterator hand_it = self->sticky_hands.find(self->stickyhand_selection);

  typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
  object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(string(hand_it->second.object_name));

  // load xml
  std::string otdf_models_path = std::string(getModelsPath()) + "/otdf/"; 
  std::string filepath =  otdf_models_path + obj_it->second.otdf_type +".otdf";
  TiXmlDocument doc(filepath);
	bool rc = doc.LoadFile();
  if(!rc) {
    cout << "store_sticky_hand failed on loading: " << filepath << endl;
    return;
  }
  TiXmlElement* object = doc.FirstChildElement("object");
  if(!object){
    cout << "store_sticky_hand failed on finding object: " << filepath << endl;
    return;
  }

  // get variables to create grasp_seed xml
  KDL::Frame& geo = hand_it->second.T_geometry_hand;
  double x = geo.p.x(), y = geo.p.y(), z = geo.p.z();
  double r,p,yaw;
  geo.M.GetRPY(r,p,yaw);

  // create grasp_seed xml
  // TODO scale pose to the size of the object
  // TODO use pose from before opt
  stringstream grasp_seed;
  grasp_seed << "<grasp_seed>" << endl;
  grasp_seed << "\t<parent name=\"" << "TODO" << "\" />" << endl;
  grasp_seed << "\t<relative_pose rpy=\"" << r << " " << p << " " << yaw << "\"";
  grasp_seed << " xyz=\"" << x << " " << y << " " << z << "\" />" << endl;
  grasp_seed << "\t<grasp_type type=\"" << hand_it->second.hand_type << "\" />" << endl;
  grasp_seed << "\t<state num_joints=\""<< hand_it->second.joint_position.size() << "\" joint_positions=\"";
  for(int i=0;i<hand_it->second.joint_position.size();i++) grasp_seed << hand_it->second.joint_position[i] << " ";
  grasp_seed << "\" />" << endl;
  grasp_seed << "</grasp_seed>" << endl;
  TiXmlDocument grasp_seed_doc;
  rc = grasp_seed_doc.Parse(grasp_seed.str().c_str());
  if(!rc){
    cout << "store_sticky_hand failed on parsing grasp_seed: " << grasp_seed.str() << endl;
    cout << "Desc: " << grasp_seed_doc.ErrorDesc() << " Line: " << grasp_seed_doc.ErrorRow() << endl;
    return;
  }

  // insert into otdf xml
  TiXmlElement* grasp_seed_ele = grasp_seed_doc.RootElement();
  TiXmlNode* node = object->InsertEndChild(*grasp_seed_ele);
  if(!node){
    cout << "store_sticky_hand failed on InsertEndChild\n";
    return;
  }
  rc = doc.SaveFile(); 
  if(!rc){
    cout << "store_sticky_hand failed on SaveFile.\n";
    return;
  }

  /*
     <grasp_seed>
        <parent name="link_name" />  
        <relative_pose rpy="0 0 0" xyz="0 0 0"/>
        <grasp_type type=1 />
        <state num_joints = 9 joint_positions="1 2 3 4 5 6 7.3 4.5 1.2"/>  
     </grasp_seed>

     <footstep_seed>
        <parent name="link_name" />  
        <relative_pose rpy="0 0 0" xyz="0 0 0"/>
        <footstep_type type=1 />   
     </footstep_seed>

  */  
}  

