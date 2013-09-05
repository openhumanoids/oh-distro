#include <otdf_interface/PlanSeed.h>
#include <math.h>
#include <stdio.h>

using namespace std;  

///////////////////////////////////////////////////////////////////////
void PlanSeed::loadFromOTDF(const std::string& otdf_file,const std::string& plan_xml_dir,const std::string& plan_name)
{
  TiXmlDocument doc(otdf_file);
  bool rc = doc.LoadFile();
  if(!rc) {
    std::cout << "failed on loading: " << otdf_file << std::endl;
    return;
  }
  
  TiXmlElement* object = doc.FirstChildElement("object");
  if(!object){
    std::cout << "store_plan_seed failed on finding object: " << otdf_file << std::endl;
    return;
  }

 std::string otdf_type = std::string(object->Attribute("name"));
 std::string otdf_seed_file = plan_xml_dir +otdf_type +"_plan_seeds.xml"; 
 TiXmlDocument xmldoc(otdf_seed_file);
  bool rc2 = xmldoc.LoadFile();
  if(!rc2) {
    std::cout << "failed to load plan storage xml file: " << otdf_seed_file << std::endl;
    return;
  }// end if
  
  for (TiXmlElement* plan_it = xmldoc.FirstChildElement("Plan"); plan_it; plan_it = plan_it->NextSiblingElement("Plan")) 
  {
    std::string plan_seed_name = std::string(plan_it->Attribute("name"));
    if(plan_seed_name==plan_name)
    {
      setFromXml(plan_it);
      break;
    }
  }// end for

}

///////////////////////////////////////////////////////////////////////
void PlanSeed::setFromXml(TiXmlElement* plan_xml)
{

 // sets the following data structures from xml
 // std::string plan_ref; // Name of the xml file that contains the plan 
 // std::string plan_type;
 // std::vector<std::string> stateframe_ids;
 // std::vector< std::vector<double> > stateframe_values;
 // std::vector<std::string> graspframe_ids;
 // std::vector< std::vector<double> > graspframe_values;

  plan_ref= plan_xml->Attribute("name");
  plan_type= plan_xml->Attribute("type");

  TiXmlElement * frame_ids = plan_xml->FirstChildElement("Frame_ids"); 
  int num_values;
  std::stringstream val_ss(frame_ids->Attribute("num_values"));
  val_ss >>num_values;
  
  stateframe_ids.clear();
  std::stringstream values_ss(frame_ids->Attribute("values"));
  for(int i=0;i<num_values;i++) 
  {
    string temp;
    values_ss >> temp;
    stateframe_ids.push_back(temp);
  }

  stateframe_values.clear();
  for (TiXmlElement* frame_it = plan_xml->FirstChildElement("Frame"); frame_it; frame_it = frame_it->NextSiblingElement("Frame")) 
  {
    int num_values;
    std::stringstream val_ss(frame_ids->Attribute("num_values"));
    val_ss >>num_values;
  
    std::stringstream values_ss(frame_it->Attribute("values"));
    std::vector<double> framevals;
    for(int i=0;i<num_values;i++) 
    {
      double temp;
      values_ss >> temp;
      framevals.push_back(temp);
    } 
    stateframe_values.push_back(framevals);
  }// end for


  TiXmlElement * gframe_ids = plan_xml->FirstChildElement("GraspFrame_ids"); 
  std::stringstream val_ss2(gframe_ids->Attribute("num_values"));
  val_ss2 >>num_values;
 

  graspframe_ids.clear();
  std::stringstream values_ss2(gframe_ids->Attribute("values"));
  for(int i=0;i<num_values;i++) 
  {
    string temp;
    values_ss2 >> temp;
    graspframe_ids.push_back(temp);
  }
  
  graspframe_values.clear();
  for (TiXmlElement* gframe_it = plan_xml->FirstChildElement("GraspFrame"); gframe_it; gframe_it = gframe_it->NextSiblingElement("GraspFrame")) 
  {
    int num_values;
    std::stringstream val_ss(gframe_it->Attribute("num_values"));
    val_ss >>num_values;
    std::stringstream values_ss(gframe_it->Attribute("values"));
    std::vector<double> gframevals;
    for(int i=0;i<num_values;i++) 
    {
      double temp;
      values_ss >> temp;
      gframevals.push_back(temp);
    } 
    graspframe_values.push_back(gframevals);
  }// end for     

}// end method setFromXml

/////////////////////////////////////////////////////////////////////////////////////////
void PlanSeed::writeToOtdf(const std::string& otdf_file){
  TiXmlDocument doc(otdf_file);
  bool rc = doc.LoadFile();
  if(!rc) {
    std::cout << "store_plan_seed failed on loading: " << otdf_file << std::endl;
    return;
  }
  TiXmlElement* object = doc.FirstChildElement("object");
  if(!object){
    std::cout << "store_plan_seed failed on finding object: " << otdf_file << std::endl;
    return;
  }
  
  // create plan_seed xml
  std::stringstream plan_seed;
  plan_seed << "<plan_seed name=\"" << plan_ref << "\" />" << std::endl;
  TiXmlDocument plan_seed_doc;
  rc = plan_seed_doc.Parse(plan_seed.str().c_str());
  if(!rc){
    std::cout << "store_plan_seed failed on parsing plan_seed: " << plan_seed.str() << std::endl;
    std::cout << "Desc: " << plan_seed_doc.ErrorDesc() << " Line: " << plan_seed_doc.ErrorRow() << std::endl;
    return;
  }

  // insert into otdf xml
  TiXmlElement* plan_seed_ele = plan_seed_doc.RootElement();
  TiXmlNode* node = object->InsertEndChild(*plan_seed_ele);
  if(!node){
    std::cout << "store_plan_seed failed on InsertEndChild\n";
    return;
  }
  rc = doc.SaveFile(); 
  if(!rc){
    std::cout << "store_plan_seed failed on SaveFile.\n";
    return;
  }

}

/////////////////////////////////////////////////////////////////////////////////////////

void PlanSeed::writePlanToXMLFile(const std::string& otdf_type,const std::string& file_path)
{
  std::string file_name = file_path +otdf_type +"_plan_seeds.xml";
  TiXmlDocument doc(file_name);
  bool rc = doc.LoadFile();
  if(!rc) {
     std::cout << "plan_seeds.xml does not exist for : " << otdf_type << std::endl;
    std::cout << "creating a new one..."<< std::endl;
	  TiXmlDocument doc;
    createOrAppendPlanXML(file_name,doc,true);
	  return;
  }
  createOrAppendPlanXML(file_name,doc,false);
}

/////////////////////////////////////////////////////////////////////////////////////////
void PlanSeed::createOrAppendPlanXML(const std::string& file_name,TiXmlDocument &doc,bool create)
{
  TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
  TiXmlElement * element = new TiXmlElement( "Plan" );
    element->SetAttribute("name", plan_ref);
    element->SetAttribute("type", plan_type);
    
    {  
      TiXmlElement * frame_ids = new TiXmlElement( "Frame_ids" );
      frame_ids->SetAttribute("num_values", stateframe_ids.size());
      std::stringstream id_values;
      for(size_t i=0;i<stateframe_ids.size();i++)
        id_values << stateframe_ids[i] << " ";
      frame_ids->SetAttribute("values", id_values.str()); 
      element->LinkEndChild(frame_ids);   
    }

    for(size_t j=0;j<stateframe_values.size();j++)
    {
      std::vector<double> temp =  stateframe_values[j];
      TiXmlElement * frame = new TiXmlElement( "Frame" );
      frame->SetAttribute("num_values", temp.size());
      std::stringstream id_values;
      for(size_t i=0;i<temp.size();i++)
        id_values << temp[i] << " ";
      frame->SetAttribute("values", id_values.str()); 
      element->LinkEndChild(frame);  
    }

    {
      TiXmlElement * gframe_ids = new TiXmlElement( "GraspFrame_ids" );
      gframe_ids->SetAttribute("num_values", graspframe_ids.size());
      std::stringstream id_values;
      for(size_t i=0;i<graspframe_ids.size();i++)
        id_values << graspframe_ids[i] << " ";
      gframe_ids->SetAttribute("values", id_values.str()); 
      element->LinkEndChild(gframe_ids);   
    }

    for(size_t j=0;j<graspframe_values.size();j++){
      std::vector<double> temp =  graspframe_values[j];
      TiXmlElement * gframe = new TiXmlElement( "GraspFrame" );
      gframe->SetAttribute("num_values", temp.size());
      std::stringstream id_values;
      for(size_t i=0;i<temp.size();i++)
        id_values << temp[i] << " ";
      gframe->SetAttribute("values", id_values.str()); 
      element->LinkEndChild(gframe);  
    }
  
  if(create)
    doc.LinkEndChild( decl );
  doc.LinkEndChild( element );
  doc.SaveFile(file_name);
}


/////////////////////////////////////////////////////////////////////////////////////////
void PlanSeed::clearAllFromOtdf(const std::string& otdf_file,const std::string& plan_xml_dir){
  TiXmlDocument doc(otdf_file);
  bool rc = doc.LoadFile();
  if(!rc) {
    std::cout << "store_plan_seed failed on loading: " << otdf_file << std::endl;
    return;
  }
  TiXmlElement* object = doc.FirstChildElement("object");
  if(!object){
    std::cout << "store_plan_seed failed on finding object: " << otdf_file << std::endl;
    return;
  }
  
  // for each plan_seed, remove from object
  while(true){    
    TiXmlElement* ele = object->FirstChildElement("plan_seed");
    if(!ele) break;
    std::string plan_seed_name= std::string(ele->Attribute("name"));
    //cout << (plan_seed_name) << endl; 
    object->RemoveChild(ele);
  }
  doc.SaveFile();

 std::string otdf_type = std::string(object->Attribute("name"));
 std::string otdf_seed_file = plan_xml_dir +otdf_type +"_plan_seeds.xml"; 
 TiXmlDocument xmldoc(otdf_seed_file);
  bool rc2 = xmldoc.LoadFile();

  // for each plan_seed, remove from object
  if(rc2){
    while(true){    
      TiXmlElement* ele = xmldoc.FirstChildElement("Plan");
      if(!ele) break;
      xmldoc.RemoveChild(ele);
    }
    xmldoc.SaveFile();
  }
}


/////////////////////////////////////////////////////////////////////////////////////////
void PlanSeed::unstoreFromOtdf(const std::string& otdf_file,const std::string& plan_xml_dir,const std::string& plan_name){
  TiXmlDocument doc(otdf_file);
  bool rc = doc.LoadFile();
  if(!rc) {
    std::cout << "unstore_plan_seed failed on loading: " << otdf_file << std::endl;
    return;
  }
  TiXmlElement* object = doc.FirstChildElement("object");
  if(!object){
    std::cout << "unstore_plan_seed failed on finding object: " << otdf_file << std::endl;
    return;
  }
  
   std::string otdf_type = std::string(object->Attribute("name"));
   std::string otdf_seed_file = plan_xml_dir +otdf_type +"_plan_seeds.xml"; 
   TiXmlDocument xmldoc(otdf_seed_file);
    bool rc2 = xmldoc.LoadFile();
    if(!rc2) {
      std::cout << "unstore_plan_seed failed to load plan storage xml file: " << otdf_seed_file << std::endl;
      return;
    }
    

  // for each plan_seed, parse and compare.  If matches this, append to toRemove
  vector<TiXmlElement*> toRemove;
  vector<TiXmlElement*> toRemoveInXMLFile;
  for (TiXmlElement* plan_it = object->FirstChildElement("plan_seed"); plan_it; plan_it = plan_it->NextSiblingElement("plan_seed")) {
    std::string plan_seed_name = std::string(plan_it->Attribute("name"));
    if(plan_seed_name==plan_name) {
      toRemove.push_back(plan_it);
    }
  }
  for (TiXmlElement* plan_it = xmldoc.FirstChildElement("Plan"); plan_it; plan_it = plan_it->NextSiblingElement("Plan")) {
    std::string plan_seed_name = std::string(plan_it->Attribute("name"));
    if(plan_seed_name==plan_name) {
      toRemoveInXMLFile.push_back(plan_it);
    }
  }
  cout << "Unstoring: " << toRemove.size() << endl;

  // remove all elements of toRemove from otdf and write to file
  if(!toRemove.empty()){
    for(int i=0;i<toRemove.size();i++)
    { 
      object->RemoveChild(toRemove[i]);
    }
    doc.SaveFile();    
  }
  if(!toRemoveInXMLFile.empty()){
    for(int i=0;i<toRemoveInXMLFile.size();i++)  { 
      xmldoc.RemoveChild(toRemoveInXMLFile[i]);
    }
    xmldoc.SaveFile();    
  } 
}
/////////////////////////////////////////////////////////////////////////////////////////
void PlanSeed::getList(const std::string& otdf_file,std::vector<std::string> &seed_list)
{
  seed_list.clear();
  TiXmlDocument doc(otdf_file);
  bool rc = doc.LoadFile();
  if(!rc) {
    std::cout << "store_plan_seed failed on loading: " << otdf_file << std::endl;
    return;
  }
  TiXmlElement* object = doc.FirstChildElement("object");
  if(!object){
    std::cout << "store_plan_seed failed on finding object: " << otdf_file << std::endl;
    return;
  }
  for (TiXmlElement* plan_it = object->FirstChildElement("plan_seed"); plan_it; plan_it = plan_it->NextSiblingElement("plan_seed")) {
    std::string seed_name = std::string(plan_it->Attribute("name"));
    seed_list.push_back(seed_name);
  }
}


