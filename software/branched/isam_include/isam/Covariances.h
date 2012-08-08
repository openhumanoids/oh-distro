/**
* @file Covariances.h
* @brief Providing access to entries of the covariance.
* @author Michael Kaess
* @version $Id: Covariances.h 6244 2012-02-25 15:28:32Z kaess $
*
* [insert iSAM license]
*
*/

#pragma once

#include <list>
#include <map>
#include <Eigen/Dense>

#include "SparseSystem.h"
#include "Node.h"

namespace isam {

class Slam;

class Covariances {
private:
  // either directly coupled to a Slam object...
  Slam* _slam;

  // ...or we operate on a copy of the relevant data from a Slam object
  SparseSystem _R;
  std::map<Node*, std::pair<int, int> > _index;

  // utility function for _index
  int get_start(Node* node) const;
  int get_dim(Node* node) const;

  // only used for cloning below
  Covariances(Slam& slam);

public:

  /**
   * Create an instance based on a Slam object, that always refers to
   * the latest state of slam.
   */
  Covariances(Slam* slam) : _slam(slam), _R(1,1) {}

  virtual ~Covariances() {};

  /**
   * Create a stand-alone copy, useful for calculating covariances in
   * a separate thread. Copies all necessary data structures to work
   * independently of the Slam object.
   * @return Covariances object that is independent of Slam object.
   */
  virtual Covariances clone() const {
    return Covariances(*_slam);
  }

  typedef std::list<std::list<Node*> > node_lists_t;
  typedef std::list<std::pair<Node*, Node*> > node_pair_list_t;

  /**
  * Calculates marginal covariance over a list of
  * lists. Significantly more efficient than calling
  * marginal_covariance multiple times with separate lists, as
  * intermediate results are being reused.
  * @param node_lists List of list of nodes.
  * @return List of marginal covariance matrices.
  */
  virtual std::list<Eigen::MatrixXd> marginal(const node_lists_t& node_lists) const;

  /**
  * Calculates marginal covariance over a list of nodes.
  * @param nodes List of nodes.
  * return Marginal covariance matrix.
  */
  virtual Eigen::MatrixXd marginal(const std::list<Node*>& nodes) const;

  /**
  * Calculates individual entries of the covariance matrix (as
  * opposed to marginal_covariance, which calculates blocks
  * containing select variables). Note that a single call with a long
  * list of entries is significantly more efficient than repeatedly
  * calling this function, as intermediate results are being reused.
  * @param entry_list List of pairs of nodes, indexing entries in
  * the covariance matrix in (column, row) format.
  * @return List of matrices.
  */
  virtual std::list<Eigen::MatrixXd> access(const node_pair_list_t& node_pair_list) const;

};

}
