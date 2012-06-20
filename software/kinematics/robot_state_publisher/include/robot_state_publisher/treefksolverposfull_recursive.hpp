// Copyright  (C)  2009 Willow Garage Inc

// Version: 1.0
// Author: Wim Meeussen <meeussen at willowgarage dot com>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


#ifndef KDLTREEFKSOLVERPOSFULL_RECURSIVE_HPP
#define KDLTREEFKSOLVERPOSFULL_RECURSIVE_HPP

#include <kdl/tree.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

namespace KDL {

class TreeFkSolverPosFull_recursive

{
public:
  TreeFkSolverPosFull_recursive(const Tree& _tree);
  ~TreeFkSolverPosFull_recursive();

  int JntToCart(const std::map<std::string, double>& q_in, std::map<std::string, drc::transform_t>& p_out, bool flatten_tree=true);

private:
  void addFrameToMap(const std::map<std::string, double>& q_in, 
		     std::map<std::string, drc::transform_t  >& p_out,
		     const KDL::Frame& previous_frame,
		     const SegmentMap::const_iterator this_segment,
		     bool flatten_tree);
  
  void TransformKDLToLCMFrame(const KDL::Frame &k, drc::transform_t &t)
  {
    t.translation.x = k.p[0];
    t.translation.y = k.p[1];
    t.translation.z = k.p[2];

    double x,y,z,w;

    k.M.GetQuaternion(x,y,z,w);
    t.rotation.x =x;
    t.rotation.y =y;
    t.rotation.z =z;
    t.rotation.w =w;
  };

  Tree tree;

};
}

#endif
