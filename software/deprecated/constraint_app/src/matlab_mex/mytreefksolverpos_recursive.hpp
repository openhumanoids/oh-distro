// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Copyright  (C)  2008 Julia Jesse

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
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

#ifndef MYKDLTREEFKSOLVERPOS_RECURSIVE_HPP
#define MYKDLTREEFKSOLVERPOS_RECURSIVE_HPP

#include <kdl/treefksolver.hpp>
#include <map>
#include <string>

/**
 * Implementation of a recursive forward position kinematics
 * algorithm to calculate the position transformation from joint
 * space to Cartesian space of a general kinematic tree (KDL::Tree).
 *
 * @ingroup KinematicFamily
 */
class MyTreeFkSolverPos_recursive
{
public:
  typedef std::map<std::string, KDL::Frame> SegmentToPoseMap;
  MyTreeFkSolverPos_recursive(const KDL::Tree& tree);
  ~MyTreeFkSolverPos_recursive();
  
  virtual int JntToCart(const KDL::JntArray& q_in, SegmentToPoseMap& result);
  
private:
  const KDL::Tree tree;
  
  void addFrameToMap(const KDL::JntArray& q_in,
		     const KDL::SegmentMap::const_iterator thisSegment,
		     const KDL::Frame& previousFrame,
		     SegmentToPoseMap& result );
  //KDL::Frame recursiveFk(const KDL::JntArray& q_in, const KDL::SegmentMap::const_iterator& it);
};

#endif
