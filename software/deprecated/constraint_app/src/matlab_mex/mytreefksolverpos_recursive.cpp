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

#include "mytreefksolverpos_recursive.hpp"
#include <iostream>
#include <kdl/frames_io.hpp>

using namespace KDL;

MyTreeFkSolverPos_recursive::MyTreeFkSolverPos_recursive(const Tree& _tree):
  tree(_tree)
{
}

int MyTreeFkSolverPos_recursive::JntToCart(const KDL::JntArray& q_in, SegmentToPoseMap& result)
{
  if(q_in.rows() != tree.getNrOfJoints())
    return -1;

  result.clear();
  addFrameToMap(q_in, tree.getRootSegment(), KDL::Frame::Identity(), result);
  
  return 0;
}

void MyTreeFkSolverPos_recursive::addFrameToMap(const JntArray& q_in,
						const SegmentMap::const_iterator thisSegment,
						const KDL::Frame& previousFrame,
						SegmentToPoseMap& result )
{
  //gets the frame for the current element (segment)
  const TreeElement& currentElement = thisSegment->second;
  Frame currentFrame = currentElement.segment.pose(q_in(currentElement.q_nr));

  KDL::Frame thisFrame = previousFrame * currentFrame;
  
  if (thisSegment->first != tree.getRootSegment()->first)
    result.insert(std::make_pair(thisSegment->first, thisFrame));

  //std::cout << "addFrameToMap: frame=" << thisSegment->first << ", frame=" << std::endl
  //	    << thisFrame << std::endl;

  // get poses of child segments
  for (std::vector<SegmentMap::const_iterator>::const_iterator child = thisSegment->second.children.begin(); 
       child !=thisSegment->second.children.end(); 
       ++child ) {
    addFrameToMap(q_in, *child, thisFrame, result);
  }      
}
/*
Frame MyTreeFkSolverPos_recursive::recursiveFk(const JntArray& q_in, const SegmentMap::const_iterator& it)
{
  //gets the frame for the current element (segment)
  const TreeElement& currentElement = it->second;
  Frame currentFrame = currentElement.segment.pose(q_in(currentElement.q_nr));
  
  SegmentMap::const_iterator rootIterator = tree.getRootSegment();
  if(it == rootIterator){
    return currentFrame;	
  }
  else{
    SegmentMap::const_iterator parentIt = currentElement.parent;
    return recursiveFk(q_in, parentIt) * currentFrame;
  }
}
*/
MyTreeFkSolverPos_recursive::~MyTreeFkSolverPos_recursive()
{
}

