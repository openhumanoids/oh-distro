/**
 * @file covariances.cpp
 * @brief How to recover covariances.
 * @author Michael Kaess
 * @version $Id: covariances.cpp 2839 2010-08-20 14:11:11Z kaess $
 *
 * Copyright (C) 2009-2010 Massachusetts Institute of Technology.
 * Michael Kaess, Hordur Johannsson and John J. Leonard
 *
 * This file is part of iSAM.
 *
 * iSAM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * iSAM is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with iSAM.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <iostream>
#include <stdio.h>

#include <isam/isam.h>

using namespace std;
using namespace isam;

Slam slam;

int main(int argc, const char* argv[]) {
  Pose2d origin;
  Matrix sqrtinf = 10. * Matrix::eye(3);
  Pose2d_Node* pose_node_1 = new Pose2d_Node(); // create node
  slam.add_node(pose_node_1); // add it to the Slam graph
  Pose2d_Factor* prior = new Pose2d_Factor(pose_node_1, origin, sqrtinf); // create prior measurement, an factor
  slam.add_factor(prior); // add it to the Slam graph

  Pose2d_Node* pose_node_2 = new Pose2d_Node(); // create node
  slam.add_node(pose_node_2); // add it to the Slam graph

  Pose2d delta(1., 0., 0.);
  Pose2d_Pose2d_Factor* odo = new Pose2d_Pose2d_Factor(pose_node_1, pose_node_2, delta, sqrtinf);
  slam.add_factor(odo);

  slam.batch_optimization();

  // recovering the full covariance matrix
  cout << "Full covariance matrix:" << endl;
  Matrix cov_full = slam.marginal_covariance(slam.get_nodes());
  cov_full.print();

  // recovering the block-diagonals only of the full covariance matrix
  cout << "Block-diagonals only:" << endl;
  Slam::node_lists_t node_lists;
  list<Node*> nodes;
  nodes.push_back(pose_node_1);
  node_lists.push_back(nodes);
  nodes.clear();
  nodes.push_back(pose_node_2);
  node_lists.push_back(nodes);
  list<Matrix> cov_blocks = slam.marginal_covariance(node_lists);
  int i = 1;
  for (list<Matrix>::iterator it = cov_blocks.begin(); it!=cov_blocks.end(); it++, i++) {
    cout << "block " << i << endl;
    it->print();
  }

  delete pose_node_1;
}
