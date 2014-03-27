/*
 * SlamPose.h
 *
 *  Created on: Oct 22, 2009
 *      Author: abachrac
 */

#ifndef SLAMPOSE_H_
#define SLAMPOSE_H_
#include <scanmatch/Scan.hpp>
#include <vector>
#include <isam/isam.h>

using namespace std;
using namespace scanmatch;
using namespace isam;
class SlamPose {
public:
  SlamPose(int64_t _utime, int _node_id, Scan * _scan, Scan *_maxRangeScan,double _height,double _rp[2]);
  virtual ~SlamPose();

  int updateScan();

  int64_t utime;
  int node_id;
  Scan * scan;
  Scan * maxRangeScan;

  //for drawing in 3D
  double height;
  double rp[2];

  vector<Scan *> allScans;

  bool loopClosureChecked;
  bool rendered;
  Pose2d_Node * pose2d_node;
  vector<int> constraint_ids;
  vector<Pose2d_Pose2d_Factor *> constraints;

  vector<SlamPose*> intermediate_nodes;

};

#endif /* SLAMPOSE_H_ */
