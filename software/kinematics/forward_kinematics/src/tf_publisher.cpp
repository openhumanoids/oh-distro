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


// Based robot_state_publisher package in ROS, adapted to work within lcm+pods.
// The tf_publisher sents out a tf tree with frame id and child frame id information.


#include "forward_kinematics/tf_publisher.hpp"
#include <kdl/frames_io.hpp>


//#include <tf_conversions/tf_kdl.h>

using namespace std;
//using namespace ros;



namespace tf_publisher{

  TfPublisher::TfPublisher(const KDL::Tree& tree)
  {
    // walk the tree and add segments to segments_
    addChildren(tree.getRootSegment());
  }


  // add children to correct maps
  void TfPublisher::addChildren(const KDL::SegmentMap::const_iterator segment)
  {
    const std::string& root = segment->second.segment.getName();

    const std::vector<KDL::SegmentMap::const_iterator>& children = segment->second.children;
    for (unsigned int i=0; i<children.size(); i++){
      const KDL::Segment& child = children[i]->second.segment;
      SegmentPair s(children[i]->second.segment, root, child.getName());
      if (child.getJoint().getType() == KDL::Joint::None){
        segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
        //ROS_DEBUG("Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
      }
      else{
        segments_.insert(make_pair(child.getJoint().getName(), s));
        //ROS_DEBUG("Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
      }
      addChildren(children[i]);
    }
  }


  // publish moving transforms
  void TfPublisher::publishTransforms(const map<string, double>& joint_positions, const int64_t& time, std::string robot_name)
  {
    //ROS_DEBUG("Publishing transforms for moving joints");

   /* std::vector<tf::StampedTransform> tf_transforms;
    tf::StampedTransform tf_transform;
    tf_transform.stamp_ = time; */
    std::vector< drc::transform_stamped_t > tf_transforms;
    drc::transform_stamped_t tf_transform;
  

    // loop over all joints
    for (map<string, double>::const_iterator jnt=joint_positions.begin(); jnt != joint_positions.end(); jnt++){
      std::map<std::string, SegmentPair>::const_iterator seg = segments_.find(jnt->first);
      if (seg != segments_.end()){
        TransformKDLToLCMFrame(seg->second.segment.pose(jnt->second), tf_transform);    
        tf_transform.frame_id_ = seg->second.root;
        tf_transform.child_frame_id_ = seg->second.tip;
        tf_transforms.push_back(tf_transform);

      }
    }


    drc::tf_t message;  
    message.utime = time;
    message.robot_name = robot_name;
    message.num_joints = tf_transforms.size();
    message.tf = tf_transforms;
	

    if(!lcm.good())
        std::cerr <<"ERROR: lcm.good()  returned false in TfPublisher::publishTransforms" <<std::endl;
  
     lcm.publish("JOINT_TRANSFORMS", &message);
     // tf_broadcaster_.sendTransform(tf_transforms);
  }


  // publish fixed transforms
  void TfPublisher::publishFixedTransforms(const int64_t& time, std::string robot_name)
  {
    //ROS_DEBUG("Publishing transforms for moving joints");
    /*std::vector<tf::StampedTransform> tf_transforms;
    tf::StampedTransform tf_transform;
    tf_transform.stamp_ = ros::Time::now()+ros::Duration(0.5);  // future publish by 0.5 seconds */

    std::vector<drc::transform_stamped_t> tf_transforms;
    drc::transform_stamped_t tf_transform;   
   
    // loop over all fixed segments
    for (map<string, SegmentPair>::const_iterator seg=segments_fixed_.begin(); seg != segments_fixed_.end(); seg++){
      TransformKDLToLCMFrame(seg->second.segment.pose(0), tf_transform);    
      tf_transform.frame_id_ = seg->second.root;
      tf_transform.child_frame_id_ = seg->second.tip;
      tf_transforms.push_back(tf_transform);
    }
    
   drc::tf_t message;  
   message.utime = time;
   message.robot_name = robot_name;
   message.num_joints = tf_transforms.size();
   message.tf = tf_transforms;

    if(!lcm.good())
      std::cerr <<"ERROR: lcm.good() returned false in TfPublisher::publishFixedTransforms" <<std::endl;


    lcm.publish("FIXEDJOINT_TRANSFORMS", &message);
    //tf_broadcaster_.sendTransform(tf_transforms);
  }

}



