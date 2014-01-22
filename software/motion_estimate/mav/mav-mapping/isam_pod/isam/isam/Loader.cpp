/**
 * @file Loader.cpp
 * @brief Loading files with constraints/factors.
 * @author Michael Kaess
 * @version $Id: Loader.cpp 2955 2010-09-07 22:56:02Z kaess $
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

#include <vector>
#include <map>
#include <list>

#include <isam/slam2d.h>

#include "Loader.h"

// set to true for external data files with other convention
const bool Y_FORWARD = false;

using namespace std;
using namespace isam;

/**
 * Create first node at origin: we add a prior to keep the
 * first pose at the origin, which is an arbitrary choice.
 */
void Loader::add_prior() {
  _nodes.resize(1);
  _factors.resize(1);
  _pose_mapper.add(0);
  // add 2D or 3D prior
  if (_is_3d) {
    // create first node
    Pose3d pose0;
    Matrix sqrtinf = 100. * Matrix::eye(6);
    Pose3d_Node* new_pose_node = new Pose3d_Node();
    _nodes[_pose_mapper[0]].push_back(new_pose_node);
    if (_verbose) cout << *new_pose_node << endl;
    _pose_nodes.push_back(new_pose_node);
    // create prior measurement
    Pose3d_Factor* prior = new Pose3d_Factor(new_pose_node, pose0, sqrtinf);
    _factors[0].push_back(prior);
    if (_verbose) cout << *prior << endl;
  } else {
    Pose2d pose0;
    Matrix sqrtinf = 100. * Matrix::eye(3);
    Pose2d_Node* new_pose_node = new Pose2d_Node();
    _nodes[_pose_mapper[0]].push_back(new_pose_node);
    if (_verbose) cout << *new_pose_node << endl;
    _pose_nodes.push_back(new_pose_node);
    Pose2d_Factor* prior = new Pose2d_Factor(new_pose_node, pose0, sqrtinf);
    _factors[0].push_back(prior);
    if (_verbose) cout << *prior << endl;
  }
}

bool Loader::advance(unsigned int idx_x1, unsigned int next_point_id) {
  // advance to next time step if needed
  bool added = _pose_mapper.add(idx_x1);
  if (added ) {
    _step++;
    _nodes.resize(_step+1);
    _factors.resize(_step+1);
    _num_points.resize(_step+1);
    _num_points[_step] = next_point_id;
    _num_constraints.resize(_step+1);
    _num_constraints[_step] = _constraints.size();
    _num_measurements.resize(_step+1);
    _num_measurements[_step] = _measurements.size();
  }
  return added;
}

void Loader::add_odometry(unsigned int idx_x0, unsigned int idx_x1, const Pose2d& measurement, const Matrix& sqrtinf) {
  if (advance(idx_x1, _point_nodes.size())) {
    Pose2d_Node* new_pose_node = new Pose2d_Node();
    _nodes[_step].push_back(new_pose_node);
    _pose_nodes.push_back(new_pose_node);
    if (_verbose) cout << idx_x1 << " " << *new_pose_node << endl;
  }
  unsigned int i_x0 = _pose_mapper[idx_x0];
  unsigned int i_x1 = _pose_mapper[idx_x1];
  Pose2d_Pose2d_Factor* factor = new Pose2d_Pose2d_Factor(
      dynamic_cast<Pose2d_Node*>(_pose_nodes[i_x0]),
      dynamic_cast<Pose2d_Node*>(_pose_nodes[i_x1]),
      measurement, sqrtinf);
  _factors[i_x1].push_back(factor);
  _constraints.push_back(make_pair(i_x0, i_x1));
  _num_constraints[i_x1] = _constraints.size();
  if (_verbose) cout << i_x1 << " " << *factor << endl;
}

void Loader::add_odometry3(unsigned int idx_x0, unsigned int idx_x1, const Pose3d& measurement, const Matrix& sqrtinf) {
  if (advance(idx_x1, _point_nodes.size())) {
    Pose3d_Node* new_pose_node = new Pose3d_Node();
    _nodes[_step].push_back(new_pose_node);
    _pose_nodes.push_back(new_pose_node);
    if (_verbose) cout << idx_x1 << " " << *new_pose_node << endl;
  }
  unsigned int i_x0 = _pose_mapper[idx_x0];
  unsigned int i_x1 = _pose_mapper[idx_x1];
  Pose3d_Pose3d_Factor* factor = new Pose3d_Pose3d_Factor(
      dynamic_cast<Pose3d_Node*>(_pose_nodes[i_x0]),
      dynamic_cast<Pose3d_Node*>(_pose_nodes[i_x1]),
      measurement, sqrtinf);
  _factors[i_x1].push_back(factor);
  _constraints.push_back(make_pair(i_x0, i_x1));
  _num_constraints[i_x1] = _constraints.size();
  if (_verbose) cout << i_x1 << " " << *factor << endl;
}

void Loader::add_measurement(unsigned int idx_x, unsigned int idx_l, const Point2d& measurement, const Matrix& sqrtinf) {
  if (_point_mapper.add(idx_l)) {
    // new point has to be added
    Point2d_Node* new_point_node = new Point2d_Node();
    _nodes[_step].push_back(new_point_node);
    _point_nodes.push_back(new_point_node);
    _num_points[_step] = _point_nodes.size();
    if (_verbose) cout << idx_x << " " << *new_point_node << endl;
  }
  unsigned int i_x = _pose_mapper[idx_x];
  unsigned int i_l = _point_mapper[idx_l];
  Pose2d_Point2d_Factor* factor = new Pose2d_Point2d_Factor(
      dynamic_cast<Pose2d_Node*>(_pose_nodes[i_x]),
      dynamic_cast<Point2d_Node*>(_point_nodes[i_l]),
      measurement, sqrtinf);
  _factors[i_x].push_back(factor);
  _measurements.push_back(make_pair(i_x, i_l));
  _num_measurements[i_x] = _measurements.size();
  if (_verbose) cout << i_x << " " << *factor << endl;
}

Loader::Loader(const char* fname, int num_lines, bool verbose) {
  _verbose = verbose;
  _step = 0;
  _is_3d = false;

  // parse and process data file
  FILE* in = fopen(fname, "r");
  if (!in) {
    printf("ERROR: Failed to open log file %s.\n", fname);
    exit(1);
  }
  int i = 0;
  while (!feof(in) && (num_lines==0 || i<num_lines)) {
    char str[2000];
    if (fgets(str, 2000, in)) {
      char keyword_c[2000];
      int key_length;
      sscanf(str, "%s%n", keyword_c, &key_length);
      const char* arguments = &str[key_length];
      string keyword(keyword_c);
      if (keyword == "ODOMETRY" || keyword == "EDGE2") {
        unsigned int idx_x0, idx_x1;
        double x, y, t, sxx, sxy, sxt, syy, syt, stt;
        int res = sscanf(arguments, "%i %i %lg %lg %lg %lg %lg %lg %lg %lg %lg", &idx_x0, &idx_x1, &x, &y, &t, &sxx, &sxy, &sxt, &syy, &syt, &stt);
        if (res!=11) {
          cout << "Error while parsing ODOMETRY entry" << endl;
          exit(1);
        }
        Pose2d measurement(x, y, t);
        Matrix sqrtinf = make_Matrix(3, 3,
            sxx, sxy, sxt,
             0., syy, syt,
             0.,  0., stt);
        if (_step==0) {
          add_prior();
        }
        add_odometry(idx_x0, idx_x1, measurement, sqrtinf);
      } else if (keyword == "LANDMARK") {
        unsigned int idx_x, idx_l;
        double x, y, sxx, sxy, syy;
        int res = sscanf(arguments, "%i %i %lg %lg %lg %lg %lg", &idx_x, &idx_l, &x, &y, &sxx, &sxy, &syy);
        if (res!=7) {
          cout << "Error while parsing LANDMARK entry" << endl;
          exit(1);
        }
        Point2d measurement(x, y);
        Matrix sqrtinf = make_Matrix(2, 2,
            sxx, sxy,
             0., syy);
        add_measurement(idx_x, idx_l, measurement, sqrtinf);
      } else if (keyword == "EDGE3") {
        unsigned int idx_x0, idx_x1;
        double x, y, z, yaw, pitch, roll, i11, i12, i13, i14, i15, i16;
        double i22, i23, i24, i25, i26, i33, i34, i35, i36, i44, i45, i46, i55, i56, i66;
        int res = sscanf(arguments, "%i %i %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg",
            &idx_x0, &idx_x1, &x, &y, &z, &roll, &pitch, &yaw, // note reverse order of angles, also see covariance below
            &i11, &i12, &i13, &i14, &i15, &i16, &i22, &i23, &i24, &i25, &i26,
            &i33, &i34, &i35, &i36, &i44, &i45, &i46, &i55, &i56, &i66);
        if (res!=29 && res!=8) {
          cout << "Error while parsing EDGE3 entry" << endl;
          exit(1);
        }
        Pose3d delta;
        if (Y_FORWARD) {
          delta = Pose3d(y, -x, z, yaw, pitch, roll); // converting from external format with Y pointing forward
        } else {
          delta = Pose3d(x, y, z, yaw, pitch, roll); // standard convention (X points forward)
        }
        // square root information matrix
        Matrix sqrtinf;
        if (res==8) {
          sqrtinf = Matrix::eye(6); // no information matrix: use identity
        } else {
          sqrtinf = make_Matrix(6, 6,
              i11, i12, i13, i14, i15, i16,
              0., i22, i23, i24, i25, i26,
              0.,  0., i33, i34, i35, i36,
              //                0.,  0.,  0., i44, i45, i46,
              //                0.,  0.,  0.,  0., i55, i56,
              //                0.,  0.,  0.,  0.,  0., i66);
              0.,  0.,  0., i66, i56, i46, // note: reversed yaw, pitch, roll, also see sscanf above
              0.,  0.,  0.,  0., i55, i45,
              0.,  0.,  0.,  0.,  0., i44);
        }
        // reverse constraint if needed
        unsigned int i, j;
        if (idx_x0<idx_x1) {
          i = idx_x1;
          j = idx_x0;
        } else {
          delta = Pose3d(delta.oTw());
          i = idx_x0;
          j = idx_x1;
        }
        if (_step==0) {
          _is_3d = true;
          add_prior();
        }
        add_odometry3(j, i, delta, sqrtinf);
      }
    }
    i++;
  }
  fclose(in);
}

void Loader::print_stats() const {
  int n = num_steps();
  cout << "Number of poses: " << n << endl;
  if (_point_nodes.size()>0) {
    cout << "Number of landmarks: " << _num_points[n-1] << endl;
    cout << "Number of measurements: " << _num_measurements[n-1] << endl;
  }
  cout << "Number of constraints: " << _num_constraints[n-1] << endl;
}

const vector<Pose3d> Loader::poses(unsigned int step) const {
  vector<Pose3d> poses;
  poses.resize(step+1);
  for (unsigned int i=0; i<step+1; i++) {
    if (is_3d()) {
      poses[i] = dynamic_cast<Pose3d_Node*>(_pose_nodes[i])->value();
    } else {
      poses[i].of_pose2d(dynamic_cast<Pose2d_Node*>(_pose_nodes[i])->value());
    }
  }
  return poses;
}

const vector<Pose3d> Loader::points(unsigned int step) const {
  vector<Pose3d> points;
  points.resize(_num_points[step]);
  for (unsigned int i=0; i<points.size(); i++) {
    if (is_3d()) {
      points[i].of_point3d(dynamic_cast<Point3d_Node*>(_point_nodes[i])->value());
    } else {
      points[i].of_point2d(dynamic_cast<Point2d_Node*>(_point_nodes[i])->value());
    }
  }
  return points;
}

const vector<pair<int,int> > Loader::constraints(unsigned int step) const {
  return vector<pair<int,int> >(_constraints.begin(), _constraints.begin()+_num_constraints[step]);
}

const vector<pair<int,int> > Loader::measurements(unsigned int step) const {
  return vector<pair<int,int> >(_measurements.begin(), _measurements.begin()+_num_measurements[step]);
}
