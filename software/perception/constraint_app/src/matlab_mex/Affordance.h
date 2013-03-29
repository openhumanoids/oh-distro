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

class Affordance {
 public:
  typedef boost::shared_ptr<Affordance> Ptr;
  
  Affordance(std::ofstream& log, const drc_affordance_t& msg);

  void PrintKdlTree(const KDL::Tree& tree, 
		    const KDL::TreeElement& segment,
		    unsigned int depth); 
 private:
  std::ofstream& m_log;
};

#endif
