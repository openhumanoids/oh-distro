#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <tinyxml.h>

struct PoseSeed{

  std::string pose_ref; // Name of the xml file that contains the pose 
  //std::string pose_type;
  std::vector<std::string> stateframe_ids;
  std::vector< std::vector<double> > stateframe_values;
  //std::vector<std::string> graspframe_ids;
  //std::vector< std::vector<double> > graspframe_values;

  // populate this with <pose_seed> from an otdf file
  void loadFromOTDF(const std::string& otdf_file,const std::string& pose_xml_dir,const std::string& pose_name);
  void setFromXml(TiXmlElement* pose_seed_xml);

  // create a <pose_seed> in given otdf_file
  void writeToOtdf(const std::string& otdf_file);
  
  //create a separate pose xml file.
  void writePoseToXMLFile(const std::string& otdf_type,const std::string& file_path);
  void createOrAppendPoseXML(const std::string& file_name,TiXmlDocument &doc,bool create);
  // remove matching <pose_seed> from otdf_file
  static void unstoreFromOtdf(const std::string& otdf_file,const std::string& pose_xml_dir,const std::string& pose_name);
 
  // remove all <pose_seed>s from otdf_file
  static void clearAllFromOtdf(const std::string& otdf_file,const std::string& pose_xml_dir);

  static void getList(const std::string& otdf_file,std::vector<std::string> &seed_list);

};

