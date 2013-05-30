#include <otdf_interface/GraspSeed.h>
#include <math.h>

using namespace std;  

///////////////////////////////////////////////////////////////////////
void GraspSeed::setFromXml(TiXmlElement* grasp_seed_xml){

  TiXmlElement* geometry = grasp_seed_xml->FirstChildElement("geometry");
  if(geometry) {
    geometry_name = geometry->Attribute("name");
  }else std::cout << "Error parsing grasp_seed parent\n";
  
  TiXmlElement* relative_pose_ele = grasp_seed_xml->FirstChildElement("relative_pose");
  if(relative_pose_ele) {
    std::stringstream xyzSS(relative_pose_ele->Attribute("xyz"));
    xyzSS >> xyz[0] >> xyz[1] >> xyz[2];
    std::stringstream rpySS(relative_pose_ele->Attribute("rpy"));
    rpySS >> rpy[0] >> rpy[1] >> rpy[2];
  }else std::cout << "Error parsing grasp_seed relative_pose\n";

  TiXmlElement* grasp_type_ele = grasp_seed_xml->FirstChildElement("grasp_type");
  if(grasp_type_ele) {
    grasp_type = atoi(grasp_type_ele->Attribute("type"));
  }else std::cout << "Error parsing grasp_seed grasp_type\n";

  TiXmlElement* state = grasp_seed_xml->FirstChildElement("state");
  if(state) {
    int numJoints = atoi(state->Attribute("num_joints"));
    joint_names.resize(numJoints);    
    joint_positions.resize(numJoints);    
    std::stringstream joint_names_ss(state->Attribute("joint_names"));
    std::stringstream joint_positions_ss(state->Attribute("joint_positions"));
    for(int i=0;i<numJoints;i++) {
      joint_names_ss >> joint_names[i];
      joint_positions_ss >> joint_positions[i];
    }
  }else std::cout << "Error parsing grasp_seed state\n";
}

/////////////////////////////////////////////////////////////////////////////////////////
void GraspSeed::writeToOtdf(const std::string& otdf_file){
  TiXmlDocument doc(otdf_file);
  bool rc = doc.LoadFile();
  if(!rc) {
    std::cout << "store_sticky_hand failed on loading: " << otdf_file << std::endl;
    return;
  }
  TiXmlElement* object = doc.FirstChildElement("object");
  if(!object){
    std::cout << "store_sticky_hand failed on finding object: " << otdf_file << std::endl;
    return;
  }
  
  // create grasp_seed xml
  // TODO scale pose to the size of the object
  // TODO use pose from before opt
  std::stringstream grasp_seed;
  grasp_seed << "<grasp_seed>" << std::endl;
  grasp_seed << "\t<geometry name=\"" << geometry_name << "\" />" << std::endl;
  grasp_seed << "\t<relative_pose rpy=\"" << rpy[0] << " " << rpy[1] << " " << rpy[2] << "\"";
  grasp_seed << " xyz=\"" << xyz[0] << " " << xyz[1] << " " << xyz[2] << "\" />" << std::endl;
  grasp_seed << "\t<grasp_type type=\"" << grasp_type << "\" />" << std::endl;
  grasp_seed << "\t<state num_joints=\""<< joint_positions.size();
  grasp_seed << "\" joint_names=\"";
  for(int i=0;i<joint_names.size();i++) grasp_seed << joint_names[i] << " ";
  grasp_seed << "\" joint_positions=\"";
  for(int i=0;i<joint_positions.size();i++) grasp_seed << joint_positions[i] << " ";
  grasp_seed << "\" />" << std::endl;
  grasp_seed << "</grasp_seed>" << std::endl;
  TiXmlDocument grasp_seed_doc;
  rc = grasp_seed_doc.Parse(grasp_seed.str().c_str());
  if(!rc){
    std::cout << "store_sticky_hand failed on parsing grasp_seed: " << grasp_seed.str() << std::endl;
    std::cout << "Desc: " << grasp_seed_doc.ErrorDesc() << " Line: " << grasp_seed_doc.ErrorRow() << std::endl;
    return;
  }

  // insert into otdf xml
  TiXmlElement* grasp_seed_ele = grasp_seed_doc.RootElement();
  TiXmlNode* node = object->InsertEndChild(*grasp_seed_ele);
  if(!node){
    std::cout << "store_sticky_hand failed on InsertEndChild\n";
    return;
  }
  rc = doc.SaveFile(); 
  if(!rc){
    std::cout << "store_sticky_hand failed on SaveFile.\n";
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

/////////////////////////////////////////////////////////////////////////////////////////
void GraspSeed::clearAllFromOtdf(const std::string& otdf_file){
  TiXmlDocument doc(otdf_file);
  bool rc = doc.LoadFile();
  if(!rc) {
    std::cout << "store_sticky_hand failed on loading: " << otdf_file << std::endl;
    return;
  }
  TiXmlElement* object = doc.FirstChildElement("object");
  if(!object){
    std::cout << "store_sticky_hand failed on finding object: " << otdf_file << std::endl;
    return;
  }

  // for each grasp_seed, remove from object
  while(true){    
    TiXmlElement* ele = object->FirstChildElement("grasp_seed");
    if(!ele) break;
    object->RemoveChild(ele);
  }

  doc.SaveFile();
}


/////////////////////////////////////////////////////////////////////////////////////////
void GraspSeed::unstoreFromOtdf(const std::string& otdf_file){
  TiXmlDocument doc(otdf_file);
  bool rc = doc.LoadFile();
  if(!rc) {
    std::cout << "store_sticky_hand failed on loading: " << otdf_file << std::endl;
    return;
  }
  TiXmlElement* object = doc.FirstChildElement("object");
  if(!object){
    std::cout << "store_sticky_hand failed on finding object: " << otdf_file << std::endl;
    return;
  }

  // for each grasp_seed, parse and compare.  If matches this, append to toRemove
  vector<TiXmlElement*> toRemove;
  for (TiXmlElement* grasp_it = object->FirstChildElement("grasp_seed"); grasp_it; grasp_it = grasp_it->NextSiblingElement("grasp_seed")) {
    GraspSeed readSeed;
    readSeed.setFromXml(grasp_it);
    cout << readSeed.geometry_name << endl;
    if(readSeed==*this) toRemove.push_back(grasp_it);
  }

  cout << "Unstoring: " << toRemove.size() << endl;

  // remove all elements of toRemove from otdf and write to file
  if(!toRemove.empty()){
    for(int i=0;i<toRemove.size();i++) object->RemoveChild(toRemove[i]);
    doc.SaveFile();
  }
}

// smart double comparison within epsilon
static bool dblsame(double x, double y){
  return fabs(x-y)<1e-3;
}

bool GraspSeed::operator==(const GraspSeed& other){
  if(geometry_name!=other.geometry_name) return false;
  if(!dblsame(xyz[0],other.xyz[0])) return false;
  if(!dblsame(xyz[1],other.xyz[1])) return false;
  if(!dblsame(xyz[2],other.xyz[2])) return false;
  if(!dblsame(rpy[0],other.rpy[0])) return false;
  if(!dblsame(rpy[1],other.rpy[1])) return false;
  if(!dblsame(rpy[2],other.rpy[2])) return false;
  if(grasp_type!=other.grasp_type) return false;
  if(joint_names!=other.joint_names) return false;
  if(joint_positions.size()!=other.joint_positions.size()) return false;
  for(int i=0;i<joint_positions.size();i++){
    if(!dblsame(joint_positions[i],other.joint_positions[i])) return false;
  }

  return true;
}

