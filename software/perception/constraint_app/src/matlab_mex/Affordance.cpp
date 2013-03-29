#include "Affordance.h"

Affordance::Affordance(std::ofstream& log, const drc_affordance_t& msg) : m_log(log)
{
  const std::string filename(msg.otdf_type);
  
  // load the OTDF file
  std::string xml_string;
  if ( !otdf::get_xml_string_from_file(filename, xml_string) ) {
    m_log << "unable to get_xml_string_from_file " << filename << std::endl;
    return;
  }

  //convert OTDF to URDF
  boost::shared_ptr<otdf::ModelInterface> otdf_instance(otdf::parseOTDF(xml_string));
  std::string urdf_xml_string(otdf::convertObjectInstanceToCompliantURDFstring(otdf_instance));
  
  // Parse KDL tree
  KDL::Tree tree;
  if ( !kdl_parser::treeFromString(urdf_xml_string,tree) ) {
    m_log << "ERROR: Failed to extract kdl tree from "  << otdf_instance->name_ 
	  << " otdf object instance "<< std::endl; 
  }
  
  m_log << "created KDL object with " << tree.getNrOfJoints() << " joints and " 
	<< tree.getNrOfSegments() << " segments" << std::endl;
  PrintKdlTree(tree, tree.getRootSegment()->second, 0);
  
}

void Affordance::PrintKdlTree(const KDL::Tree& tree, 
			      const KDL::TreeElement& segment,
			      unsigned int depth) 
{
  for ( unsigned int i = 0; i < depth; i++ ) {
    m_log << " ";
  }
  m_log << segment.segment.getName() 
	<< ", joint " << segment.segment.getJoint().getName() 
	<< " of type " << segment.segment.getJoint().getTypeName()
	<< std::endl;
  for ( unsigned int i = 0; i < segment.children.size(); i++ ) {
    PrintKdlTree(tree, segment.children[i]->second, depth+1);
  }
}
