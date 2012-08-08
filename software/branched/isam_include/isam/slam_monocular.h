/**
 * @file slam_monocular.h
 * @brief Provides nodes and factors for monocular vision applications.
 * @author Michael Kaess
 * @version $Id: slam_monocular.h 6369 2012-03-28 23:26:19Z kaess $
 *
 * [insert iSAM license]
 *
 */

// WARNING: The monocular factor has not been tested extensively...

#pragma once

#include <string>
#include <sstream>
#include <math.h>
#include <Eigen/Dense>

#include "Node.h"
#include "Factor.h"
#include "Pose3d.h"
#include "Point3dh.h"

namespace isam {

class MonocularMeasurement {
  friend std::ostream& operator<<(std::ostream& out, const MonocularMeasurement& t) {
    t.write(out);
    return out;
  }

public:
  double u;
  double v;
  bool valid;

  MonocularMeasurement(double u, double v) : u(u), v(v), valid(true) {}
  MonocularMeasurement(double u, double v, bool valid) : u(u), v(v), valid(valid) {}

  Eigen::Vector3d vector() const {
    Eigen::Vector3d tmp(u, v);
    return tmp;
  }

  void write(std::ostream &out) const {
    out << "(" << u << ", " << v << ")";
  }
};

class MonocularCamera { // for now, camera and robot are identical
  double _f;
  Eigen::Vector2d _pp;
  double _b;

public:

  MonocularCamera() : _f(1), _pp(Eigen::Vector2d(0.5,0.5)) {}
  MonocularCamera(double f, const Eigen::Vector2d& pp) : _f(f), _pp(pp) {}

  inline double focalLength() const {return _f;}

  inline Eigen::Vector2d principalPoint() const {return _pp;}

  MonocularMeasurement project(const Pose3d& pose, const Point3dh& Xw) const {
    Point3dh X = pose.transform_to(Xw);
    // camera system has z pointing forward, instead of x
    double x = X.y();
    double y = X.z();
    double z = X.x();
    double w = X.w();
    if ((z/w) > 0.) { // check if point infront of camera
      double fz = _f / z;
      double u = x * fz + _pp(0);
      double v = y * fz + _pp(1);
      return MonocularMeasurement(u, v);
    } else {
      return MonocularMeasurement(0., 0., false);
    }
  }

  Point3dh backproject(const Pose3d& pose, const MonocularMeasurement& measure,
      double distance = 5.) const {
    double lx = (measure.u-_pp(0));
    double ly = (measure.v-_pp(1));
    double lz = _f;
    if (distance<0.) {
      std::cout << "Warning: MonocularCamera.backproject called with negative distance\n";
    }
// BEGIN REMOVE FROM RELEASE
    // todo: correct?
// END REMOVE FROM RELEASE
    double lw = 1./distance;
    Point3dh X(lz, lx, ly, lw);

    return pose.transform_from(X);
  }

};

typedef NodeT<Point3dh> Point3dh_Node;

/**
 * Monocular observation of a 3D homogeneous point;
 * projective or Euclidean geometry depending on constructor used.
 */
class Monocular_Factor : public FactorT<MonocularMeasurement> {
  Pose3d_Node* _pose;
  Point3d_Node* _point;
  Point3dh_Node* _point_h;
  MonocularCamera* _camera;

public:

  // constructor for projective geometry
  Monocular_Factor(Pose3d_Node* pose, Point3dh_Node* point, MonocularCamera* camera,
      const MonocularMeasurement& measure, const Eigen::MatrixXd& sqrtinf)
    : FactorT<MonocularMeasurement>("Monocular_Factor", 2, sqrtinf, measure),
      _pose(pose), _point(NULL), _point_h(point), _camera(camera) {
    // MonocularCamera could also be a node later (either with 0 variables,
    // or with calibration as variables)
    _nodes.resize(2);
    _nodes[0] = pose;
    _nodes[1] = point;
  }

  // constructor for Euclidean geometry - WARNING: only use for points at short range
  Monocular_Factor(Pose3d_Node* pose, Point3d_Node* point, MonocularCamera* camera,
      const MonocularMeasurement& measure, const Eigen::MatrixXd& sqrtinf)
    : FactorT<MonocularMeasurement>("Monocular_Factor", 2, sqrtinf, measure),
      _pose(pose), _point(point), _point_h(NULL), _camera(camera) {
    _nodes.resize(2);
    _nodes[0] = pose;
    _nodes[1] = point;
  }

  void initialize() {
    require(_pose->initialized(), "Monocular_Factor requires pose to be initialized");
    bool initialized = (_point_h!=NULL) ? _point_h->initialized() : _point->initialized();
    if (!initialized) {
      Point3dh predict = _camera->backproject(_pose->value(), _measure);
      // normalize homogeneous vector
      predict = Point3dh(predict.vector()).normalize();
      if (_point_h!=NULL) {
        _point_h->init(predict);
      } else {
        _point->init(predict.to_point3d());
      }
    }
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    Point3dh point = (_point_h!=NULL) ? _point_h->value(s) : _point->value(s);
    MonocularMeasurement predicted = _camera->project(_pose->value(s), point);
    if (predicted.valid == true) {
      return (predicted.vector() - _measure.vector());
    } else {
      // effectively disables points behind the camera
      return Eigen::Vector2d::Zero();
    }
  }

};

}
