/**
 * @file slam_auv.h
 * @brief Provides specialized nodes and factors for AUV applications.
 * @author Maurice Fallon
 * @author Ayoung Kim
 * @author Michael Kaess
 * @version $Id: slam_auv.h 5691 2011-11-10 21:25:23Z kaess $
 *
 * [insert iSAM license]
 *
 */

#pragma once

#include <string>
#include <sstream>
#include <Eigen/Dense>

#include "Node.h"
#include "Factor.h"
#include "Pose2d.h"
#include "Pose3db.h"
#include "Point2d.h"
#include "slam2d.h"

namespace isam {

/** 
 * This class is for the 5 dof camera measurement
 * @author Ayoung Kim (ayoungk@umich.edu)
 */
class Pose3db_Pose3db_Factor : public FactorT<Pose3db> {
  Pose3d_Node* _pose1;
  Pose3d_Node* _pose2;

public:

  Pose3db_Pose3db_Factor(Pose3d_Node* pose1, Pose3d_Node* pose2, 
                         const Pose3db& measure, const Noise& noise,
                         Pose3d *senxform1 = NULL, Pose3d *senxform2 = NULL)
    : FactorT<Pose3db>("Pose3db_Pose3db_Factor", 5, noise, measure), _pose1(pose1), _pose2(pose2) {
    require((senxform1==NULL && senxform2==NULL) || (senxform1!=NULL && senxform2!=NULL),
            "slam3d: Pose3db_Pose3db_Factor requires either 0 or 2 sensor xform");

    _nodes.resize(2);
    _nodes[0] = pose1;
    _nodes[1] = pose2;
    if (senxform1 != NULL) {
      need_senxform = 1;
      _senxform1 = Pose3d(senxform1->vector());
      _senxform2 = Pose3d(senxform2->vector());
    }
    else
      need_senxform = 0;
  }

  void initialize() {
    require(_nodes[0]->initialized() && _nodes[1]->initialized(), "Pose3db_Pose3db_Factor: both nodes have to be initialized");
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    Pose3d p1(_pose1->value(s));
    Pose3d p2(_pose2->value(s));
    Pose3d predicted;

    if (need_senxform) {
      predicted = (p1.oplus(_senxform1)).ominus(p2.oplus(_senxform2));
    } else {
      predicted = p1.ominus(p2);   // p21: pose1 seen by pose2
    }

    Pose3db predicted_bearing;
    predicted_bearing.of_pose3d(predicted);

    Eigen::VectorXd err = predicted_bearing.vector() - _measure.vector();

    err(0) =  standardRad(err(0));
    err(1) = standardRad(err(1));
    err(2) = standardRad(err(2));
    err(3) = standardRad(err(3));
    err(4) = standardRad(err(4));

    return err;
  }

  void write(std::ostream &out) const {
    Factor::write(out);
    if (need_senxform)
      out << " " << _measure << " " << noise_to_string(_noise) << " " << _senxform1  << " " << _senxform2;
    else
      out << " " << _measure << " " << noise_to_string(_noise);
  }

private:
  Pose3d _senxform1;
  Pose3d _senxform2;
  bool need_senxform;
};

/**
 * This class is for the relative sensor transformation
 * @author ayoung kim jan 2011
 */
class SenPose3d_SenPose3d_Factor : public FactorT<Pose3d> {
  Pose3d_Node* _pose1;
  Pose3d_Node* _pose2;

public:

  SenPose3d_SenPose3d_Factor(Pose3d_Node* pose1, Pose3d_Node* pose2, 
                             const Pose3d& measure, const Noise& noise,
                             Pose3d *senxform1, Pose3d *senxform2)
    : FactorT<Pose3d>("SenPose3d_SenPose3d_Factor", 6, noise, measure), _pose1(pose1), _pose2(pose2) {
    require((senxform1==NULL && senxform2==NULL) || (senxform1!=NULL && senxform2!=NULL),
            "slam3d: SenPose3d_SenPose3d_Factor requires either 0 or 2 sensor xform");

    _nodes.resize(2);
    _nodes[0] = pose1;
    _nodes[1] = pose2;
    if (senxform1 != NULL) {
      need_senxform = 1;
      _senxform1 = Pose3d(senxform1->vector());
      _senxform2 = Pose3d(senxform2->vector());
    }
    else
      need_senxform = 0;
  }

  void initialize() {
    require(_nodes[0]->initialized() && _nodes[1]->initialized(), "Pose3db_Pose3db_Factor: both nodes have to be initialized");
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    Pose3d p1(_pose1->value(s));
    Pose3d p2(_pose2->value(s));
    Pose3d predicted;

    if (need_senxform) {
      predicted = (p2.oplus(_senxform2)).ominus(p1.oplus(_senxform1));
    } else {
      predicted = p2.ominus(p1);   // = p12: pose2 seen by pose1
    }

    Eigen::VectorXd err = predicted.vector() - _measure.vector();
    err(3) = standardRad(err(3));
    err(4) = standardRad(err(4));
    err(5) = standardRad(err(5));

    return err;
  }

private:
  Pose3d _senxform1;
  Pose3d _senxform2;
  bool need_senxform;
};

/**
 * This class is for the odometry between to AUV positions
 * See Point2d_Point2d_RangeFactor for a similar class for range measurements
 * @author mfallon april 2010
 */
class Point2d_Point2d_Factor : public FactorT<Point2d> {
  Point2d_Node* _pose1;
  Point2d_Node* _pose2;

public:

  Point2d_Point2d_Factor(Point2d_Node* pose1, Point2d_Node* pose2,
                         const Point2d& measure, const Noise& noise,
                         Point2d_Node* base1 = NULL, Point2d_Node* base2 = NULL)
    : FactorT<Point2d>("Point2d_Point2d_Factor", 2, noise, measure), _pose1(pose1), _pose2(pose2) {
    require((base1==NULL && base2==NULL) || (base1!=NULL && base2!=NULL),
        "slam_auv: Point2d_Point2d_Factor requires either 0 or 2 base nodes");
    if (base1) { // offset between two relative pose graphs
      _nodes.resize(4);
      _nodes[2] = base1;
      _nodes[3] = base2;
    } else {
      _nodes.resize(2);
    }
    _nodes[0] = pose1;
    _nodes[1] = pose2;

  }

  void initialize() {
    require(_pose1->initialized(), "slam_auv: Point2d_Point2d_Factor requires pose1 to be initialized");
    if (!_pose2->initialized()) {
      Point2d a = _pose1->value(); // take the first pose
      Point2d predict = a.plus(_measure); // compose it with the measurement ... still need to be modified for coopnav
      _pose2->init(predict); // gives you the 1st est of pose2
    }
    if (_nodes.size()==4) {
      Point2d_Node* base1 = dynamic_cast<Point2d_Node*>(_nodes[2]);
      Point2d_Node* base2 = dynamic_cast<Point2d_Node*>(_nodes[3]);
      require(base1->initialized(), "slam_auv: Point2d_Point2d_Factor requires base1 to be initialized");
      if (!base2->initialized()) {
        Point2d a = _pose1->value(); // todo
        Point2d b = _pose2->value();
        Point2d b1 = base1->value();
        Point2d d = _measure.minus(b.minus(b1.plus(a)));
        base2->init(d);
      }
    }
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    Point2d p1(_pose1->value(s));
    Point2d p2(_pose2->value(s));
    Point2d predicted;
    if (_nodes.size()==4) {
      Point2d base1(_nodes[2]->vector(s));
      Point2d base2(_nodes[3]->vector(s));
      predicted = (base2.plus(p2)).minus(base1.plus(p1));
    } else {
      Point2d p = p2.minus(p1);
      predicted = p.vector();
    }
    Eigen::VectorXd err = predicted.vector() - _measure.vector();
    return err;
  }

  void write(std::ostream &out) const {
    Factor::write(out);
    out << " " << _measure << " " << noise_to_string(_noise);
    if (_nodes.size()==4) {
      out << " " << _nodes[2]->unique_id() << " " << _nodes[3]->unique_id();
    }
  }
};

/**
 * The following class was added by mfallon to support Point2d_Factor
 * in the same manner as Pose2d_Factor - from used with MLBL
 * This class is for AUV to CNA ranges. It can also be used for AUV-to-AUV ranges later
 * See Point2d_Point2d_RangeFactor for a simular class for range measurements
 * @author mfallon april 2010
 */
class Point2d_Point2d_RangeFactor : public FactorT<double> {
  Point2d_Node* _pose1;
  Point2d_Node* _pose2;

public:

  Point2d_Point2d_RangeFactor(Point2d_Node* pose1, Point2d_Node* pose2,
                              const double& measure, const Noise& noise)
    : FactorT<double>("Point2d_Point2d_RangeFactor", 1, noise, measure), _pose1(pose1), _pose2(pose2) {
    _nodes.resize(2);
    _nodes[0] = pose1;
    _nodes[1] = pose2;
  }

  void initialize() {
    require(_nodes[0]->initialized(), "slam_auv: Point2d_Point2d_RangeFactor requires pose1 to be initialized");
    require(_nodes[1]->initialized(), "slam_auv: Point2d_Point2d_RangeFactor requires pose2 to be initialized");
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    Point2d p1(_pose1->value(s));
    Point2d p2(_pose2->value(s));
    double predicted;
    predicted = (p1.vector() - p2.vector()).norm(); // sqrt( (x1-x2)^2 + (y1-y2)^2 )
    double temp = predicted - _measure;
    Eigen::VectorXd err(1);
    err << temp;
    return err;
  }

  void write(std::ostream &out) const {
    Factor::write(out);
    out << " " << _measure << " " << noise_to_string(_noise);
    if (_nodes.size()==4) {
      out << " " << _nodes[2]->unique_id() << " " << _nodes[3]->unique_id();
    }
  }
};

}
