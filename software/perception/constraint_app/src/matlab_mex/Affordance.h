#ifndef AFFORDANCE_H
#define AFFORDANCE_H

#include <boost/shared_ptr.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <lcmtypes/drc_lcmtypes.h>
#include <lcm/lcm.h>
#include <otdf_parser/otdf_parser.h>
#include <lcmtypes/bot_core.h>
#include <path_util/path_util.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <forward_kinematics/treefksolverposfull_recursive.hpp>
#include <otdf_parser/otdf_parser.h>
#include <otdf_parser/otdf_urdf_converter.h>
#include <boost/bimap.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>

using namespace boost::multi_index;

class Affordance {
 public:
  class Joint {
  public:
  Joint(const std::string& _name, const int& _num) : name(_name), num(_num) {}
    std::string name;
    int num;
  };

  typedef boost::shared_ptr<Affordance> Ptr;
  typedef std::vector<double> StateVector;
  
  Affordance(const std::string& filename, std::ostream& log = std::cout);
  Affordance(std::ostream& log, const drc::affordance_t& msg);  
  bool GetStateFromMsg(const drc::affordance_t& msg, StateVector& state);
  void PrintState(const StateVector& state);
  KDL::Tree& GetTree() { return m_tree; }
  bool GetSegmentExpressedInWorld(const std::string& segmentName, const StateVector& state, 
				  KDL::Frame& segment_expressedIn_world);
  bool GetSegmentJacobianExpressedInWorld(const std::string& segmentName, 
					  const StateVector& state, 
					  KDL::Jacobian& jacobian);
  bool GetSegmentJacobianExpressedInWorld2(const std::string& segmentName, 
					   const StateVector& state, 
					   KDL::Jacobian& jacobian);
  bool DecodeState(const StateVector& state, KDL::Frame& root_expressedIn_world, KDL::JntArray& joints);
  void DecodeState(const StateVector& state, KDL::JntArray& joints);
  void PrintKdlTree() { PrintKdlTree(m_tree, m_tree.getRootSegment()->second, 0); }

 private:
  void PrintKdlTree(const KDL::Tree& tree, 
		    const KDL::TreeElement& segment,
		    unsigned int depth); 
  void UpdateJointNames(const KDL::TreeElement* element = NULL);
  void PrintJointNames();

 public:
  static KDL::Frame GetFrameFromVector(const StateVector& state);

 public:
  std::ostream& m_log;
  KDL::Tree m_tree;

  typedef boost::multi_index_container<
    Joint,
    indexed_by<
      ordered_unique<member<Joint, int, &Joint::num> >,
      ordered_unique<member<Joint, std::string, &Joint::name> >
    > 
    > JointMultiIndex;
  JointMultiIndex m_joints;    
};

#endif
