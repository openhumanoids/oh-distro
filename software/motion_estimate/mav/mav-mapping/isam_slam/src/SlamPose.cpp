/*
 * SlamPose.cpp
 *
 *  Created on: Oct 22, 2009
 *      Author: abachrac
 */

#include "SlamPose.hpp"

using namespace scanmatch;
SlamPose::SlamPose(int64_t _utime, int _node_id, Scan * _scan, Scan * _maxRangeScan, double _height, double _rp[2]) :
  utime(_utime), node_id(_node_id), scan(_scan), maxRangeScan(_maxRangeScan), height(_height),
      loopClosureChecked(false), rendered(false)
{
  pose2d_node = new Pose2d_Node();
  constraint_ids.push_back(node_id - 1); //assume constraint between current and prev

  if (scan != NULL)
    allScans.push_back(scan);
  if (maxRangeScan != NULL)
    allScans.push_back(maxRangeScan);
  if (_rp != NULL)
    memcpy(rp, _rp, 2 * sizeof(double));
}

SlamPose::~SlamPose()
{
  if (scan != NULL)
    delete scan;
  if (maxRangeScan != NULL)
    delete maxRangeScan;
  if (pose2d_node != NULL)
    delete pose2d_node;

  constraint_ids.clear();
  for (int i = 0; i < intermediate_nodes.size(); i++)
    delete intermediate_nodes[i];
  intermediate_nodes.clear();
}

int SlamPose::updateScan()
{
  int updated = 0;
  Pose2d value = pose2d_node->value();
  for (unsigned i = 0; i < allScans.size(); i++) {
    Scan * s = allScans[i];
    //check whether the ScanTransform needs to be updated
    if (sqrt(sm_sq(s->T.x - value.x()) + sm_sq(s->T.y - value.y())) > .001 || fabs(sm_angle_subtract(s->T.theta,
        value.t())) > .001) {
      //need to update ths ScanTranform
      ScanTransform newT;
      newT.x = value.x();
      newT.y = value.y();
      newT.theta = value.t();
      s->applyTransform(newT);
      updated = 1;
    }
  }
  return updated;
}
