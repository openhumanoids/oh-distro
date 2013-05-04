#include "Affordance.h"
#include <otdf_lcm_utils/otdf_lcm_utils.h>
#include "ConstraintApp.h"
#include <kdl/treefksolverpos_recursive.hpp>

Affordance::Affordance(const std::string& filename, std::ostream& log /*= std::cout*/) : m_log(log) 
{
  // load the OTDF file
  std::string xml_string;
  if ( !otdf::get_xml_string_from_file(filename, xml_string) ) {
    m_log << "unable to get_xml_string_from_file " << filename << std::endl;
    return;
  }

  //convert OTDF to URDF
  boost::shared_ptr<otdf::ModelInterface> otdf_instance(otdf::parseOTDF(xml_string));
  otdf_instance->update();

  std::string urdf_xml_string(otdf::convertObjectInstanceToCompliantURDFstring(otdf_instance));

  // Parse KDL tree
  if ( !kdl_parser::treeFromString(urdf_xml_string, m_tree) ) {
    m_log << "ERROR: Failed to extract kdl tree from " << filename 
	  << " otdf object instance "<< std::endl; 
    return;
  }

  UpdateJointNames();

  //print some debug info on the new affordance
  m_log << "Joint names and numbers: " << std::endl;
  PrintJointNames();
  
  m_log << "created KDL object with " << m_tree.getNrOfJoints() << " joints and " 
	<< m_tree.getNrOfSegments() << " segments" << std::endl;
  PrintKdlTree(m_tree, m_tree.getRootSegment()->second, 0);
}

Affordance::Affordance(std::ostream& log, const drc::affordance_t& msg) : m_log(log)
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

  //print some debug info on the new affordance
  m_log << "xml string: " << std::endl 
	<< urdf_xml_string << std::endl;

  m_log << "Joint names and numbers: " << std::endl;
  PrintJointNames();
  
  m_log << "created KDL object with " << m_tree.getNrOfJoints() << " joints and " 
	<< m_tree.getNrOfSegments() << " segments" << std::endl;
  PrintKdlTree(m_tree, m_tree.getRootSegment()->second, 0);
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
    m_log << iter->name << " #" << iter->num << std::endl;
  }
}

void Affordance::UpdateJointNames(const KDL::TreeElement* element /*=NULL*/)
{
  if ( !element ) {
    m_joints.clear();
    UpdateJointNames(&m_tree.getRootSegment()->second);
    
    //perform sanity checks; loop through the joints and verify that the q_nr's are sequential and start at zero
    int q_nr = 0;
    for ( JointMultiIndex::nth_index<0>::type::iterator iter = m_joints.begin();
	iter != m_joints.end(); ++iter ) {
      if ( q_nr != iter->num ) {
	m_log << "ERROR: while parsing tree, expected joint #" << q_nr << ", found joint #" << iter->num << std::endl;
	m_joints.clear();
	m_tree = KDL::Tree();
	return;
      }
      q_nr++;
    }
    return;
  }

  if ( element->segment.getJoint().getType() != KDL::Joint::None ) {
    m_joints.insert(Joint(element->segment.getJoint().getName(), element->q_nr));
  }

  for ( unsigned int i = 0; i < element->children.size(); i++ ) {
    UpdateJointNames(&element->children[i]->second);
  }

}

/*
void Affordance::GetState(std::vector<double>& state) const
{
  state = ConstraintApp::FrameToVector(m_basepose);

  for ( JointMultiIndex::nth_index<0>::type::const_iterator iter = m_joints.begin();
	iter != m_joints.end(); ++iter ) {
    state.push_back(iter->value);
  }
}
*/

bool Affordance::GetStateFromMsg(const drc::affordance_t& msg, StateVector& state) 
{
  //extract the x,y,z,r,p,w from the msg params
  ConstraintApp::OptionalKDLFrame msgFrame(ConstraintApp::GetFrameFromParams(&msg));
  if ( !msgFrame ) {
    m_log << "ERROR: Affordance::UpdateStateFromMsg unable to recover frame from message" << std::endl;
    return false;
  }
  state = ConstraintApp::FrameToVector(*msgFrame);

  //convert the msg.state to a map
  typedef std::map<std::string, double> ValueMap;
  ValueMap values;
  for ( int i=0; i<msg.nstates; i++ ) {
    values.insert(std::pair<std::string, double>(msg.state_names[i], msg.states[i]));
  }

  //iterate through the joints according to their q_nr number
  for ( JointMultiIndex::nth_index<0>::type::iterator iter = m_joints.begin();
	iter != m_joints.end(); ++iter ) {
    ValueMap::iterator match = values.find(iter->name);
    if ( match == values.end() ) {
      m_log << "ERROR: unable to find joint " << iter->name << " in the message state" << std::endl;
      return false;
    }
    state.push_back(match->second);
  }

  //perform sanity checks
  if ( state.size() != 6 + m_joints.size() ) {
    m_log << "ERROR: state has improper size" << std::endl;
    return false;
  }

  return true;
}

void Affordance::PrintState(const StateVector& state)
{
  for ( int i = 0; i < state.size(); i++ ) {
    m_log << state[i] << ", ";
  }
  m_log << std::endl;
}

bool Affordance::DecodeState(const StateVector& state, KDL::Frame& root_expressedIn_world, KDL::JntArray& joints)
{
  const int basePoseSize = 6;
  if ( state.size() < basePoseSize ) {
    m_log << "ERROR: Affordance::DecodeState must be of at least size 6 (not " << state.size() << ")" << std::endl;
    m_log << "   state=";
    for ( int i = 0; i < state.size(); i++ ) m_log << state[i] << ", ";
    m_log << std::endl;
    return false;
  }

  root_expressedIn_world = GetFrameFromVector(state);

  int numJoints = state.size() - basePoseSize;
  if ( numJoints < 0 ) return false;
  joints.resize(numJoints);
  for ( int i = 0; i < numJoints; i++ ) {
    joints(i) = state[i+basePoseSize];
  }

  return true;
}

bool Affordance::GetSegmentExpressedInWorld(const std::string& segmentName, const StateVector& state, 
					    KDL::Frame& segment_expressedIn_world)
{
  KDL::Frame root_expressedIn_world;
  KDL::JntArray joints;
  if ( !DecodeState(state, root_expressedIn_world, joints) ) {
    return false;
  }

  KDL::Frame segment_expressedIn_root;
  KDL::TreeFkSolverPos_recursive fk(m_tree);
  int fkret = fk.JntToCart(joints, segment_expressedIn_root, segmentName);
  if ( fkret < 0 ) {
    m_log << "ERROR: Affordance::GetSegmentExpressedInWorld: fk returned " << fkret << std::endl
	  << "   segmentName=" << segmentName << std::endl
	  << "   joints=" << joints.data << std::endl;
    m_log << "   state=";
    for ( int i = 0; i < state.size(); i++ ) m_log << state[i] << ", ";
    m_log << std::endl;
    return false;
  }

  segment_expressedIn_world = root_expressedIn_world * segment_expressedIn_root;

  return true;
}

KDL::Frame Affordance::GetFrameFromVector(const StateVector& state) 
{
  KDL::Frame ret;
  ret.p[0] = state[0];
  ret.p[1] = state[1];
  ret.p[2] = state[2];
  ret.M = KDL::Rotation::RPY(state[3], state[4], state[5]);
  return ret;
}
