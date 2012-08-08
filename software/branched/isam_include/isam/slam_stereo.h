/**
 * @file slam_stereo.h
 * @brief Provides specialized nodes and factors for stereo vision applications.
 * @author Michael Kaess
 * @version $Id: slam_stereo.h 6536 2012-04-22 16:30:15Z kaess $
 *
 * [insert iSAM license]
 *
 */

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

class StereoMeasurement {
  friend std::ostream& operator<<(std::ostream& out, const StereoMeasurement& t) {
    t.write(out);
    return out;
  }

public:
  double u;
  double v;
  double u2;
  bool valid;

  StereoMeasurement(double u, double v, double u2)
    : u(u), v(v), u2(u2), valid(true) {
  }
  StereoMeasurement(double u, double v, double u2, bool valid)
    : u(u), v(v), u2(u2), valid(valid) {
  }

  Eigen::Vector2d left_pixel() const {Eigen::Vector2d V(2); V << u, v; return V;}

  Eigen::Vector2d right_pixel() const {Eigen::Vector2d V(2); V << u2, v; return V;}

  double disparity() const {return u-u2;}

  Eigen::Vector3d vector() const {
    Eigen::Vector3d tmp(u, v, u2);
    return tmp;
  }

  void write(std::ostream &out) const {
    out << "(" << u << ", " << v << ", " << u2 << ")";
  }
};

class StereoCamera { // for now, camera and robot are identical
  double _f;
  Eigen::Vector2d _pp;
  double _b;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StereoCamera() : _f(1), _pp(Eigen::Vector2d(0.5,0.5)), _b(0.1) {}
  StereoCamera(double f, const Eigen::Vector2d& pp, double b) : _f(f), _pp(pp), _b(b) {}

  inline double focalLength() const {return _f;}

  inline Eigen::Vector2d principalPoint() const {return _pp;}

  inline double baseline() const {return _b;}

  StereoMeasurement project(const Pose3d& pose, const Point3dh& Xw) const {
    Point3dh X = pose.transform_to(Xw);
    // camera system has z pointing forward, instead of x
    double x = X.y();
    double y = X.z();
    double z = X.x();
    double w = X.w();
    // left camera
    double fz = _f / z;
    double u = x * fz + _pp(0);
    double v = y * fz + _pp(1);
    // right camera
    double u2 = (x - w*_b) * fz + _pp(0);
    bool valid = ((w==0&&z>0) || (z/w) > 0.); // infront of camera?
    return StereoMeasurement(u, v, u2, valid);
  }

// BEGIN REMOVE FROM RELEASE
#ifdef USE_QUATERNIONS
  Eigen::MatrixXd project_dpose_dXw(const Pose3d& pose, const Point3dh& Xw) const {
    Point3dh X = pose.transform_to(Xw);
    Eigen::MatrixXd trans = pose.transform_to_dpose_dpoint(Xw); // 4x9
    // projection with respect to X
    double x = X.x();
    double y = X.y();
    double z = X.z();
    double w = X.w();
    double x2 = x*x;
    Eigen::MatrixXd proj(3,4); // 3x4
    proj <<
      -_f*y/x2,         _f/x,     0,  0,
      -_f*z/x2,            0,  _f/x,  0,
      -_f*(y-_b*w)/x2,  _f/x,     0,  -_b*_f/x;
    // chain rule: 3x9
    return proj*trans;
  }
#endif
// END REMOVE FROM RELEASE

  Point3dh backproject(const Pose3d& pose, const StereoMeasurement& measure) const {
    double lx = (measure.u-_pp(0))*_b;
    double ly = (measure.v-_pp(1))*_b;
    double lz = _f*_b;
    double lw = measure.u - measure.u2;
    if (lw<0.) {
      std::cout << "Warning: StereoCamera.backproject called with negative disparity\n";
    }
    Point3dh X(lz, lx, ly, lw);

    return pose.transform_from(X);
  }

};

typedef NodeT<Point3dh> Point3dh_Node;

/**
 * Stereo observation of a 3D homogeneous point;
 * projective or Euclidean geometry depending on constructor used.
 */
class Stereo_Factor : public FactorT<StereoMeasurement> {
  Pose3d_Node* _pose;
  Point3d_Node* _point;
  Point3dh_Node* _point_h;
  StereoCamera* _camera;
  bool _relative;
  Pose3d_Node* _base;

public:

  // constructor for projective geometry
  Stereo_Factor(Pose3d_Node* pose, Point3dh_Node* point, StereoCamera* camera,
                         const StereoMeasurement& measure, const Noise& noise, bool relative = false)
    : FactorT<StereoMeasurement>("Stereo_Factor", 3, noise, measure),
      _pose(pose), _point(NULL), _point_h(point), _camera(camera), _relative(relative), _base(NULL) {
    // StereoCamera could also be a node later (either with 0 variables,
    // or with calibration as variables)
    _nodes.resize(2);
    _nodes[0] = pose;
    _nodes[1] = point;
    // for relative parameterization recover base pose
    if (_relative && !point->factors().empty()) {
      _nodes.resize(3);
      _nodes[2] = point->factors().front()->nodes()[0];
      // todo: first factor might refer to a prior or other type of node...
      _base = dynamic_cast<Pose3d_Node*>(_nodes[2]);
    } else {
      _base = _pose;
    }
  }

  // constructor for Euclidean geometry
  // WARNING: only use for points at short range
  Stereo_Factor(Pose3d_Node* pose, Point3d_Node* point, StereoCamera* camera,
                         const StereoMeasurement& measure, const Noise& noise)
    : FactorT<StereoMeasurement>("Stereo_Factor", 3, noise, measure),
      _pose(pose), _point(point), _point_h(NULL), _camera(camera) {
    _nodes.resize(2);
    _nodes[0] = pose;
    _nodes[1] = point;
  }

  void initialize() {
    require(_pose->initialized(), "Stereo_Factor requires pose to be initialized");
    bool initialized = (_point_h!=NULL) ? _point_h->initialized() : _point->initialized();
    if (!initialized) {
      Point3dh predict;
      if (_relative) {
        predict = _camera->backproject(Pose3d(), _measure);
      } else {
        predict = _camera->backproject(_pose->value(), _measure);
      }
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
    Pose3d pose = _pose->value(s);
    if (_base) {
      // pose of camera relative to base camera (which might be this one!)
      pose = pose.ominus(_base->value(s));
    }
    StereoMeasurement predicted = _camera->project(pose, point);
    if (_point_h!=NULL || predicted.valid == true) {
      return (predicted.vector() - _measure.vector());
    } else {
      std::cout << "Warning - StereoFactor.basic_error: point behind camera, dropping term.\n";
      return Eigen::Vector3d::Zero();
    }
  }
// BEGIN REMOVE FROM RELEASE
#if 0
#ifdef USE_QUATERNIONS
  // todo: non-homogeneous case
  Jacobian jacobian() {
    // 3x9 matrix
    Eigen::MatrixXd d = _camera->project_dpose_dXw(_pose->value(LINPOINT), _point_h->value(LINPOINT));

    Eigen::MatrixXd M1 = sqrtinf() * d.leftCols(6); // 3x6
    Eigen::MatrixXd M2 = sqrtinf() * d.rightCols(3); // 3x3
    Eigen::VectorXd r = sqrtinf() * basic_error();
    Jacobian jac(r);
    jac.add_term(_nodes[0], M1);
    jac.add_term(_nodes[1], M2);
    return jac;
  }
#endif
#endif
// END REMOVE FROM RELEASE

};

}
