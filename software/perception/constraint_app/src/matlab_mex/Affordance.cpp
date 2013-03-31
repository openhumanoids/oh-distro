#include "Affordance.h"
#include <otdf_lcm_utils/otdf_lcm_utils.h>
#include "ConstraintApp.h"

Affordance::Affordance(std::ofstream& log, const drc::affordance_t& msg) : m_log(log)
{
  /*
  std::string urdf_xml_string;
  if ( !otdf::AffordanceLcmMsgToUrdfString(msg, urdf_xml_string) ) {
    m_log << "ERROR: failed to process new affordance" << std::endl;
    return;
  }
  */

  const std::string filename(msg.otdf_type);
  
  // load the OTDF file
  std::string xml_string;
  if ( !otdf::get_xml_string_from_file(filename, xml_string) ) {
    m_log << "unable to get_xml_string_from_file " << filename << std::endl;
    return;
  }

  //convert OTDF to URDF
  boost::shared_ptr<otdf::ModelInterface> otdf_instance(otdf::parseOTDF(xml_string));
  
  for(int i=0;i<msg.nparams;i++){
    otdf_instance->setParam(msg.param_names[i],msg.params[i]); 
    m_log << msg.param_names[i] << "<=" << msg.params[i] << std::endl;
  }
  for(int i=0;i<msg.nstates;i++){
    double pos, vel;
    pos = msg.states[i];
    vel =0;
    otdf_instance->setJointState(msg.state_names[i],pos,vel); 
    m_log << msg.state_names[i] << "<=" << pos << std::endl;
  }
  otdf_instance->update();

  std::string urdf_xml_string(otdf::convertObjectInstanceToCompliantURDFstring(otdf_instance));

  // Parse KDL tree
  if ( !kdl_parser::treeFromString(urdf_xml_string, m_tree) ) {
    m_log << "ERROR: Failed to extract kdl tree from " << msg.otdf_type << ", uid=" << msg.uid
	  << " otdf object instance "<< std::endl; 
    return;
  }

  UpdateJointNames();
  UpdateStateFromMsg(msg);

  //print some debug info on the new affordance
  m_log << "xml string: " << std::endl 
	<< urdf_xml_string << std::endl;

  m_log << "Joint names and numbers: " << std::endl;
  PrintJointNames();
  
  m_log << "created KDL object with " << m_tree.getNrOfJoints() << " joints and " 
	<< m_tree.getNrOfSegments() << " segments" << std::endl;
  PrintKdlTree(m_tree, m_tree.getRootSegment()->second, 0);

  PrintState();
  
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
	<< ", joint #" << segment.q_nr
	<< std::endl;
  for ( unsigned int i = 0; i < segment.children.size(); i++ ) {
    PrintKdlTree(tree, segment.children[i]->second, depth+1);
  }
}

void Affordance::PrintJointNames()
{
  for ( JointMultiIndex::nth_index<0>::type::iterator iter = m_joints.begin();
	iter != m_joints.end(); ++iter ) {
    m_log << iter->name << " #" << iter->num << ", value=" << iter->value << std::endl;
  }
}

void Affordance::UpdateJointNames(const KDL::TreeElement* element /*=NULL*/)
{
  if ( !element ) {
    m_joints.clear();
    element = &m_tree.getRootSegment()->second;
  }

  if ( element->segment.getJoint().getType() != KDL::Joint::None ) {
    m_joints.insert(Joint(element->segment.getJoint().getName(), element->q_nr));
  }

  for ( unsigned int i = 0; i < element->children.size(); i++ ) {
    UpdateJointNames(&element->children[i]->second);
  }

}

void Affordance::GetState(std::vector<double>& state) const
{
  state = ConstraintApp::FrameToVector(m_basepose);

  for ( JointMultiIndex::nth_index<0>::type::const_iterator iter = m_joints.begin();
	iter != m_joints.end(); ++iter ) {
    state.push_back(iter->value);
  }
}

void Affordance::UpdateStateFromMsg(const drc::affordance_t& msg) 
{
  ConstraintApp::OptionalKDLFrame msgFrame(ConstraintApp::GetFrameFromParams(&msg));
  if ( !msgFrame ) {
    m_log << "ERROR: Affordance::UpdateStateFromMsg unable to recover frame from message" << std::endl;
    return;
  }
  m_basepose = *msgFrame;

  typedef std::map<std::string, double> ValueMap;
  ValueMap values;
  for ( int i=0; i<msg.nstates; i++ ) {
    values.insert(std::pair<std::string, double>(msg.state_names[i], msg.states[i]));
  }

  for ( JointMultiIndex::nth_index<0>::type::iterator iter = m_joints.begin();
	iter != m_joints.end(); ++iter ) {
    ValueMap::iterator match = values.find(iter->name);
    if ( match == values.end() ) {
      m_log << "ERROR: unable to update joint " << iter->name << std::endl;
      return;
    }
    Joint thisJoint = *iter;
    thisJoint.value = match->second;
    m_joints.replace(iter, thisJoint);
  }
}

void Affordance::PrintState()
{
  std::vector<double> state;
  GetState(state);
  for ( int i = 0; i < state.size(); i++ ) {
    m_log << state[i] << ", ";
  }
  m_log << std::endl;
}
