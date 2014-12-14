/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#ifndef TF_PUBLISHER_HPP
#define TF_PUBLISHER_HPP

//#include <ros/ros.h>
#include <stdint.h> 
#include <lcm/lcm-cpp.hpp>
#include <boost/scoped_ptr.hpp>
//#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include "lcmtypes/drc/transform_stamped_t.hpp"
#include "lcmtypes/drc/tf_t.hpp"


namespace tf_publisher{

class SegmentPair
{
public:
  SegmentPair(const KDL::Segment& p_segment, const std::string& p_root, const std::string& p_tip):
    segment(p_segment), root(p_root), tip(p_tip){}

  KDL::Segment segment;
  std::string root, tip;
};


class TfPublisher
{
public:
  /** Constructor
   * \param tree The kinematic model of a robot, represented by a KDL Tree 
   */
  TfPublisher(const KDL::Tree& tree);

  /// Destructor
  ~TfPublisher(){};

  /** Publish transforms
   * \param joint_positions A map of joint names and joint positions. 
   * \param time The time at which the joint positions were recorded
   */
  void publishTransforms(const std::map<std::string, double>& joint_positions, const int64_t& time, std::string robot_name);
  void publishFixedTransforms(const int64_t& time, std::string robot_name);

private:
  void addChildren(const KDL::SegmentMap::const_iterator segment);
  void TransformKDLToLCMFrame(const KDL::Frame &k, drc::transform_stamped_t &t)
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

  std::map<std::string, SegmentPair> segments_, segments_fixed_;
  //tf::TransformBroadcaster tf_broadcaster_;
 
  lcm::LCM lcm;
};



}

#endif
