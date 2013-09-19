#ifndef MYTREEJNTTOJACSOLVER_HPP_
#define MYTREEJNTTOJACSOLVER_HPP_

#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

class MyTreeJntToJacSolver {
public:
  explicit MyTreeJntToJacSolver(const KDL::Tree& tree);

  virtual ~MyTreeJntToJacSolver();

  int JntToJac(const KDL::JntArray& q_in, KDL::Jacobian& jac,
	       const std::string& segmentname);

private:
  KDL::Tree tree;
  
};

#endif /* TREEJNTTOJACSOLVER_H_ */
