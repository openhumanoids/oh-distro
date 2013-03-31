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
  Joint(const std::string& _name, const int& _num) : name(_name), num(_num), value(0.0) {}
  Joint(const std::string& _name, const int& _num, const double& _value) : name(_name), num(_num), value(_value) {}
    std::string name;
    int num;
    double value;
  };

  typedef boost::shared_ptr<Affordance> Ptr;
  
  Affordance(std::ofstream& log, const drc::affordance_t& msg);
  void GetState(std::vector<double>& state) const;
  void UpdateStateFromMsg(const drc::affordance_t& msg);
  void PrintState();

 private:
  void PrintKdlTree(const KDL::Tree& tree, 
		    const KDL::TreeElement& segment,
		    unsigned int depth); 
  void UpdateJointNames(const KDL::TreeElement* element = NULL);
  void PrintJointNames();

 private:
  std::ofstream& m_log;
  KDL::Tree m_tree;

  KDL::Frame m_basepose; //pose of root link

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
