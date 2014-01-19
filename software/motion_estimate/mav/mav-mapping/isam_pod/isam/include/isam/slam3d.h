/**
 * @file slam3d.h
 * @brief Provides specialized nodes and factors for 3D SLAM
 * @author Michael Kaess
 * @version $Id: slam3d.h 2922 2010-08-27 05:42:42Z kaess $
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

#pragma once

#include "Node.h"
#include "Factor.h"
#include "Pose3d.h"
#include "Point3d.h"

namespace isam {

typedef NodeT<Pose3d> Pose3d_Node;
typedef NodeT<Point3d> Point3d_Node;

class Pose3d_Factor : public Factor {
public:
  const Pose3d prior;
  /**
   * Constructor.
   * @param pose The pose node the prior acts on.
   * @param prior The actual prior measurement.
   * @param sqrtinf The 6x6 square root information matrix (upper triangular).
   */
  Pose3d_Factor(Pose3d_Node* pose, const Pose3d& prior, const Matrix& sqrtinf)
  : Factor("Pose3d_Factor", 6, sqrtinf), prior(prior) {
    _nodes.resize(1);
    _nodes[0] = pose;
  }
  void initialize() {
    Pose3d_Node* pose = dynamic_cast<Pose3d_Node*>(_nodes[0]);
    if (!pose->initialized()) {
      Pose3d predict = prior;
      pose->init(predict);
    }
  }
  Vector basic_error(const std::vector<Vector>& vec) const {
    Vector err = vec[0] - prior.vector();
    err.set(3, standardRad(err(3)));
    err.set(4, standardRad(err(4)));
    err.set(5, standardRad(err(5)));
    return err;
  }
};

class Pose3d_Pose3d_Factor : public Factor {
public:
  const Pose3d measure;
  /**
   * Constructor.
   * @param pose1 The pose from which the measurement starts.
   * @param pose2 The pose to which the measurement extends.
   * @param measure The relative measurement from pose1 to pose2 (pose2 in pose1's frame).
   * @param sqrtinf The 6x6 square root information matrix (upper triangular).
   * @param anchor1 Optional anchor node for trajectory to which pose1 belongs to.
   * @param anchor2 Optional anchor node for trajectory to which pose2 belongs to.
   */
  Pose3d_Pose3d_Factor(Pose3d_Node* pose1, Pose3d_Node* pose2,
      const Pose3d& measure, const Matrix& sqrtinf,
      Pose3d_Node* anchor1 = NULL, Pose3d_Node* anchor2 = NULL)
  : Factor("Pose3d_Pose3d_Factor", 6, sqrtinf), measure(measure) {
    require((anchor1==NULL && anchor2==NULL) || (anchor1!=NULL && anchor2!=NULL),
        "slam3d: Pose3d_Pose3d_Factor requires either 0 or 2 anchor nodes");
    if (anchor1) { // offset between two relative pose graphs
      _nodes.resize(4);
      _nodes[2] = anchor1;
      _nodes[3] = anchor2;
    } else {
      _nodes.resize(2);
    }
    _nodes[0] = pose1;
    _nodes[1] = pose2;
  }
  void initialize() {
    Pose3d_Node* pose1 = dynamic_cast<Pose3d_Node*>(_nodes[0]);
    Pose3d_Node* pose2 = dynamic_cast<Pose3d_Node*>(_nodes[1]);
    require(pose1->initialized() || pose2->initialized(),
        "slam3d: Pose3d_Pose3d_Factor requires pose1 or pose2 to be initialized");
    if (!pose1->initialized() && pose2->initialized()) {
      // Reverse constraint 
      Pose3d a = pose2->value();
      Pose3d z;
      Pose3d predict = a.oplus(z.ominus(measure));
      pose1->init(predict);
    } else if (pose1->initialized() && !pose2->initialized()) {
      Pose3d a = pose1->value();
      Pose3d predict = a.oplus(measure);
      pose2->init(predict);
    }
    if (_nodes.size()==4) {
      Pose3d_Node* anchor1 = dynamic_cast<Pose3d_Node*>(_nodes[2]);
      Pose3d_Node* anchor2 = dynamic_cast<Pose3d_Node*>(_nodes[3]);
      require(anchor1->initialized(), "slam3d: Pose3d_Pose3d_Factor requires anchor1 to be initialized");
      if (!anchor2->initialized()) {
        Pose3d a = pose1->value();
        Pose3d b = pose2->value();
        Pose3d b1 = anchor1->value();
        Pose3d d = measure.ominus(b.ominus(b1.oplus(a)));
        anchor2->init(d);
      }
    }
  }
  Vector basic_error(const std::vector<Vector>& vec) const {
    Pose3d p1(vec[0]);
    Pose3d p2(vec[1]);
    Pose3d predicted;
    if (vec.size()==4) {
      Pose3d anchor1(vec[2]);
      Pose3d anchor2(vec[3]);
      predicted = (anchor2.oplus(p2)).ominus(anchor1.oplus(p1));
    } else {
      Pose3d p = p2.ominus(p1);
      predicted = p.vector();
    }
    Vector err = predicted.vector() - measure.vector();
    err.set(3, standardRad(err(3)));
    err.set(4, standardRad(err(4)));
    err.set(5, standardRad(err(5)));
    return err;
  }
};

class Pose3d_Point3d_Factor : public Factor {
public:
  const Point3d measure;
  /**
   * Constructor.
   * @param pose The pose from which the landmark is observed.
   * @param point The point or landmark that is observed
   * @param measure The relative observation of the landmark in the pose's frame.
   * @param sqrtinf The 3x3 square root information matrix (upper triangular).
   */
  Pose3d_Point3d_Factor(Pose3d_Node* pose, Point3d_Node* point,
      const Point3d& measure, const Matrix& sqrtinf)
  : Factor("Pose3d_Point3d_Factor", 3, sqrtinf), measure(measure) {
    _nodes.resize(2);
    _nodes[0] = pose;
    _nodes[1] = point;
  }
  void initialize() {
    Pose3d_Node* pose = dynamic_cast<Pose3d_Node*>(_nodes[0]);
    Point3d_Node* point = dynamic_cast<Point3d_Node*>(_nodes[1]);
    require(pose->initialized(), "slam3d: Pose3d_Point3d_Factor requires pose to be initialized");
    if (!point->initialized()) {
      Pose3d p = pose->value();
      Point3d predict = p.transform_from(measure);
      point->init(predict);
    }
  }
  Vector basic_error(const std::vector<Vector>& vec) const {
    Pose3d po(vec[0]);
    Point3d pt(vec[1]);
    Point3d p = po.transform_to(pt);
    Vector predicted = p.vector();
    return (predicted - measure.vector());
  }
};

}
