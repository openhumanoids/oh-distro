#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <tinyxml.h>

struct PlanSeed{

  std::string plan_ref; // Name of the xml file that contains the plan 
  std::string plan_type;
  std::vector<std::string> stateframe_ids;
  std::vector< std::vector<double> > stateframe_values;
  std::vector<std::string> graspframe_ids;
  std::vector< std::vector<double> > graspframe_values;

  // populate this with <plan_seed> from an otdf file
  void loadFromOTDF(const std::string& otdf_file,const std::string& plan_xml_dir,const std::string& plan_name);
  void setFromXml(TiXmlElement* plan_seed_xml);

  // create a <plan_seed> in given otdf_file
  void writeToOtdf(const std::string& otdf_file);
  
  //create a separate plan xml file.
  void writePlanToXMLFile(const std::string& otdf_type,const std::string& file_path);
  void createOrAppendPlanXML(const std::string& file_name,TiXmlDocument &doc,bool create);
  // remove matching <plan_seed> from otdf_file
  static void unstoreFromOtdf(const std::string& otdf_file,const std::string& plan_xml_dir,const std::string& plan_name);
 
  // remove all <plan_seed>s from otdf_file
  static void clearAllFromOtdf(const std::string& otdf_file,const std::string& plan_xml_dir);

  static void getList(const std::string& otdf_file,std::vector<std::string> &seed_list);

};

