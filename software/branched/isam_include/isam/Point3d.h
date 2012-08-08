/**
 * @file Point3d.h
 * @brief Simple 3D point class.
 * @author Michael Kaess
 * @version $Id: Point3d.h 6538 2012-04-23 04:05:21Z kaess $
 *
 * [insert iSAM license]
 *
 */

#pragma once

#include <Eigen/Dense>

#include "Point2d.h"

namespace isam {

class Point3d {
  friend std::ostream& operator<<(std::ostream& out, const Point3d& p) {
    p.write(out);
    return out;
  }

  double _x;
  double _y;
  double _z;
public:
  static const int dim = 3;
  static const int size = 3;
  static const char* name() {
    return "Point3d";
  }
  Point3d() : _x(0.), _y(0.), _z(0.) {}
  Point3d(double x, double y, double z) : _x(x), _y(y), _z(z) {}
  Point3d(const Eigen::Vector3d& vec) : _x(vec(0)), _y(vec(1)), _z(vec(2)) {}

  double x() const {return _x;}
  double y() const {return _y;}
  double z() const {return _z;}

  void set_x(double x) {_x = x;}
  void set_y(double y) {_y = y;}
  void set_z(double z) {_z = z;}

  Point3d exmap(const Eigen::Vector3d& delta) const {
    Point3d res = *this;
    res._x += delta(0);
    res._y += delta(1);
    res._z += delta(2);
    return res;
  }

  Eigen::Vector3d vector() const {
    Eigen::Vector3d tmp(_x, _y, _z);
    return tmp;
  }
  void set(double x, double y, double z) {
    _x = x;
    _y = y;
    _z = z;
  }
  void set(const Eigen::Vector3d& v) {
    _x = v(0);
    _y = v(1);
    _z = v(2);
  }

  Point3d to_point3d() {
    return *this;
  }

  void of_point2d(const Point2d& p) {
    _x = p.x();
    _y = p.y();
    _z = 0.;
  }
  void write(std::ostream &out) const {
    out << "(" << _x << ", " << _y << ", " << _z << ")";
  }
};

}
