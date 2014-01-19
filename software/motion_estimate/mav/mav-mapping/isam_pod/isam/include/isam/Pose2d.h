/**
 * @file Pose2d.h
 * @brief Simple 2D pose class.
 * @author Michael Kaess
 * @version $Id: Pose2d.h 3349 2010-11-06 12:45:27Z kaess $
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

#include <cmath>
#include <ostream>

#include "util.h"
#include "Vector.h"
#include "Point2d.h"

namespace isam {

class Pose2d {
  friend std::ostream& operator<<(std::ostream& out, const Pose2d& p) {
    p.write(out);
    return out;
 }

  double _x;
  double _y;
  double _t;

public:
  // assignment operator and copy constructor implicitly created, which is ok
  static const int dim = 3;
  static const char* name() {
    return "Pose2d";
  }
  Pose2d() : _x(0.), _y(0.), _t(0.) {}
  Pose2d(double x, double y, double t) : _x(x), _y(y), _t(t) {}
  Pose2d(const Vector& vec) : _x(vec(0)), _y(vec(1)), _t(vec(2)) {}

  double x() const {return _x;}
  double y() const {return _y;}
  double t() const {return _t;}

  void set_x(double x) {_x = x;}
  void set_y(double y) {_y = y;}
  void set_t(double t) {_t = t;}

  Vector vector() const {
    // disabled - faster but not portable, even using a "packed" struct
    //    return Vector(3, &_x);
    Vector v = make_Vector(3, _x, _y, _t);
    return v;
  }
  void set(double x, double y, double t) {
    _x = x;
    _y = y;
    _t = t;
  }
  void set(const Vector& v) {
    _x = v(0);
    _y = v(1);
    _t = standardRad(v(2));
  }
  void write(std::ostream &out) const {
    out << "(" << _x << ", " << _y << ", " << _t << ")";
  }

  /**
   * Calculate new pose b composed from this pose (a) and the odometry d.
   * Follows notation of Lu&Milios 1997.
   * \f$ b = a \oplus d \f$
   * @param d Pose difference to add.
   * @return d transformed from being local in this frame (a) to the global frame.
   */
  Pose2d oplus(const Pose2d& d) const {
    double c = cos(t());
    double s = sin(t());
    double px = x() + c*d.x() - s*d.y();
    double py = y() + s*d.x() + c*d.y();
    double pt = t() + d.t();
    return Pose2d(px,py,pt);
  }

  /**
   * Odometry d from b to this pose (a). Follows notation of
   * Lu&Milios 1997.
   * \f$ d = a \ominus b \f$
   * @param b Base frame.
   * @return Global this (a) expressed in base frame b.
   */
  Pose2d ominus(const Pose2d& b) const {
    double c = cos(b.t());
    double s = sin(b.t());
    double dx = x() - b.x();
    double dy = y() - b.y();
    double ox =  c*dx + s*dy;
    double oy = -s*dx + c*dy;
    double ot = t() - b.t();
    return Pose2d(ox,oy,ot);
  }

  /**
   * Project point into this coordinate frame.
   * @param p Point to project
   * @return Point p locally expressed in this frame.
   */
  Point2d transform_to(const Point2d& p) const {
    double c = cos(t());
    double s = sin(t());
    double dx = p.x() - x();
    double dy = p.y() - y();
    double x =  c*dx + s*dy;
    double y = -s*dx + c*dy;
    return Point2d(x,y);
  }

  Point2d transform_from(const Point2d& p) const {
    double c = cos(t());
    double s = sin(t());
    double px = x() + c*p.x() - s*p.y();
    double py = y() + s*p.x() + c*p.y();
    return Point2d(px,py);
  }

};

}
