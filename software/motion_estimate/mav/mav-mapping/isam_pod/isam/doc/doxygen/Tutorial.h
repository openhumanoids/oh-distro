/**
 * @file Tutorial.h
 * @brief iSAM tutorial in doxygen format
 * @author Michael Kaess
 * @version $Id: Tutorial.h 2898 2010-08-24 01:06:18Z kaess $
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

/** @page Tutorial iSAM in Depth


@section tutorial_history History

iSAM is my Ph.D. dissertation work that was advised by Frank Dellaert
at the Georgia Institute of Technology. Back in 2005 I worked on data
association and needed marginal covariances for the SLAM
problem. Calculating those in the naive way yielded hour-long waiting
times even for relatively small problems. At that time I got
interested in ongoing work by Frank called Square Root SAM and finally
helped with the experiments of the IJRR 2006 paper. Together with
Ananth Ranganathan we figured out a way of recovering arbitrary parts
of the full covariance matrix from the square root information factor
that Square Root SAM calculates. However, calculating that square root
information matrix itself from scratch eventually becomes expensive
for larger problems. Instead, I started to incrementally update the
square root information matrix, which is advantageous because most of
the time new measurements only affect a small part of the variables of
the SLAM problem. We eventually selected Givens rotations as a simple
mechanism to update a QR matrix factorization, which gave rise to the
iSAM algorithm, first published in IJCAI in January 2007.


@section tutorial_fg SLAM as a Factor Graph

In the iSAM library, We represent the SLAM problem as a factor
graph. A factor graph is a bipartite graph that contains two types of
nodes: variable nodes and factor nodes. Edges are only allowed between
different types of nodes, ie. they always connect a node with a
factor. An example of a factor graph representing a SLAM problem is
given below:

\image html factor_graph.png

Colored circles represent variable nodes, and small solid black
circles represent factor nodes. Robot poses x_i are marked blue and
are connected by a chain of odometry measurements u_i. Other
constraints, for example derived from laser scan-matching are not
necessarily connecting subsequent poses, but can also encode loop
closing constraints. So far the SLAM problem represents a so-called
pose graph, consisting only of poses and constraints.

SLAM can also contain landmarks, here the landmark locations l_j shown
in green. Landmark observations u_k are also represented by
factors. Landmarks can for example be observed by laser range finders
or cameras. The resulting optimization problem is referred to as
bundle adjustment in photogrammetry and as structure from motion in
computer vision.

While this example can readily be represented as a Bayes net, as
typically found in the SLAM literature, more complicated scenarios are
more easily represented as factor graphs. For example, the calibration
of a sensor can be represented by another node that connects to all
relevant factors. A more advanced example is provided by the concept
of anchor nodes, see \ref tutorial_anchor "anchor nodes".

For a code example, please see the \ref Example.


@section tutorial_important Most Important Classes

The isam::Slam class in Slam.h represents the factor graph, providing
methods for creating the graph as well as optimizing. Of particular
interest are the methods isam::Slam::add_node() and
isam::Slam::add_factor() that are needed to incrementally build the
SLAM graph. Suitable nodes and factors for 2D SLAM problems can be
found in slam2d.h. The method isam::Slam::update() performs the
necessary calculations to obtain a new estimate, and is typically
called after all factors of a time step are added. The detailed
behavior of the algorithm can be controlled through
isam::Slam::update_properties().


@section tutorial_init Automatic and Manual Initialization of Nodes

In many cases the iSAM library can automatically generate initial
estimates for new nodes based on the first measurement that gets
connected to the node. For odometry, for example, the current estimate
of the previous pose together with the odometry measurement provides
the initialization of the new pose. In other situations, such as when
partial constraints are being added (eg. stereo measurements),
multiple measurements might be needed to constrain the new variable
(eg. camera pose). In that case the user can provide the
initialization that was for example obtained by a RANSAC algorithm
with 5-point algorithm. In that case the user can call
isam::Node::init() with the appropriate estimate.


@section tutorial_factors Defining New Factors and Nodes

New types of factors can easily be defined by the user. Examples of
factors are provided in slam2d.h. The user simply creates a new class
derived from Factor and defines a cost function. To allow fast
prototyping, iSAM by default uses numerical differentiation. For
time-critical implementations, the user can optionally provide a
symbolic derivative by overwriting isam::Factor::jacobian.

For the case that a new Node type is required, simply define a new
class that provides the interface required by isam::Node::NodeT. The
template NodeT is then used to create the actual new node. This design
is different from a Factor and has been chosen to allow use of the
data type itself independently of nodes (also, a Node internally
creates two instances of the data type, one for the current estimate
and one for the linearization point).


@section tutorial_covs Marginal Covariances

A key advantage of iSAM is the efficient access to the exact marginal
covariances. See examples/covariances.cpp for how to access selected
parts of the full marginal covariance matrix. Note that recovering the
full marginal covariance is infeasible for large problems, independent
of the algorithm used, as the matrix itself contains O(n^2) entries
that would have to be calculated.


@section tutorial_anchor Anchor Nodes and Multi-Robot/Session Mapping

We have recently presented a novel approach to multi-robot or
multi-session mapping. An implementation of anchor nodes is included
in iSAM, for usage see examples/anchorNodes.cpp.

\image html anchor_nodes.png

The figure shows three robot trajectories that are not synchronized,
and do not require initial relative pose estimates. Encounters later
define the alignment of the trajectories in a common frame. For more
details please refer to:

@li "Multiple Relative Pose Graphs for Robust Cooperative Mapping" by
B. Kim, M. Kaess, L. Fletcher, J. Leonard, A. Bachrach, N. Roy,
S. Teller, IEEE Intl. Conf. on Robotics and Automation (ICRA), May
2010, <a href="http://www.cc.gatech.edu/~kaess/pub/Kim10icra.pdf"
target="_top">PDF</a>


@section tutorial_conventions General Comments

- A file A.cpp contains "class A" (Slam.h), while a.cpp contains
  non-class functions (util.cpp) or collections of classes (slam2d.h).

- The CHOLMOD library is used for efficiency in sparse batch
  factorization.

- The custom class SparseSystem is used for incremental updates, as
  updates are expensive for the usual compressed storage.

- Only on some computers, simply linking with GLUT/GL slows down the
  isam executable by 50%, even when no GLUT functions are called;
  therefore, set USE_GUI to OFF for timing.

*/
