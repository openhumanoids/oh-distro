/**
 * @file Point2d.h
 * @brief Simple 2D point class.
 * @author Michael Kaess
 * @version $Id: Point2d.h 4133 2011-03-22 20:40:38Z kaess $
 *
 * [insert iSAM license]
 *
 */

#pragma once

#include <Eigen/Dense>

namespace isam {

class Point2d {
  friend std::ostream& operator<<(std::ostream& out, const Point2d& p) {
    p.write(out);
    return out;
  }

  double _x;
  double _y;

public:
  static const int dim = 2;
  static const int size = 2;
  static const char* name() {
    return "Point2d";
  }
  Point2d() : _x(0), _y(0) {}
  Point2d(double x, double y) : _x(x), _y(y) {}
  Point2d(const Eigen::Vector2d& vec) : _x(vec(0)), _y(vec(1)) {}

  double x() const {return _x;}
  double y() const {return _y;}

  void set_x(double x) {_x = x;}
  void set_y(double y) {_y = y;}

  Point2d exmap(const Eigen::Vector2d& delta) const {
    Point2d res = *this;
    res._x += delta(0);
    res._y += delta(1);
    return res;
  }

  Eigen::Vector2d vector() const {
    Eigen::Vector2d v(_x, _y);
    return v;
  }
  void set(double x, double y) {
    _x = x;
    _y = y;
  }
  void set(const Eigen::Vector2d& v) {
    _x = v(0);
    _y = v(1);
  }
  void write(std::ostream &out) const {
    out << "(" << _x << ", " << _y << ")";
  }

  // BEGIN REMOVE FROM RELEASE
  /**
   * Adding points.
   * @param d Pose difference to add.
   * @return d transformed from being local in this frame to the global frame.
   */
  Point2d plus(const Point2d& d) const {
    double px = x() + d.x();
    double py = y() + d.y();
    return Point2d(px,py);
  }

  /**
   * Subtracting points.
   * @param b Base frame.
   * @return This from expressed in local frame of b.
   */
  Point2d minus(const Point2d& b) const {
    double dx = x() - b.x();
    double dy = y() - b.y();
    double ox =  dx;
    double oy =  dy;
    return Point2d(ox,oy);
  }    
  // END REMOVE FROM RELEASE
};

}
