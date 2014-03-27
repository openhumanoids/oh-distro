/**
 * @file Point3dh.h
 * @brief 3D homogeneous point class.
 * @author Michael Kaess
 * @version $Id: Point3dh.h 2839 2010-08-20 14:11:11Z kaess $
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

#include "Vector.h"
#include "Point3d.h"

namespace isam {

class Point3dh {
  friend std::ostream& operator<<(std::ostream& out, const Point3dh& p) {
    p.write(out);
    return out;
  }

  double _x;
  double _y;
  double _z;
  double _w;
public:
  static const int dim = 4;
  static const char* name() {
    return "Point3dh";
  }
  Point3dh() : _x(0.), _y(0.), _z(0.), _w(0.) {}
  Point3dh(double x, double y, double z, double w) : _x(x), _y(y), _z(z), _w(w) {}
  Point3dh(const Vector& vec) : _x(vec(0)), _y(vec(1)), _z(vec(2)), _w(vec(3)) {}
  Point3dh(const Point3d& p) : _x(p.x()), _y(p.y()), _z(p.z()), _w(1.) {}

  double x() const {return _x;}
  double y() const {return _y;}
  double z() const {return _z;}
  double w() const {return _w;}

  void x(double x) {_x = x;}
  void y(double y) {_y = y;}
  void z(double z) {_z = z;}
  void w(double w) {_w = w;}

  Vector vector() const {
    return make_Vector(4, _x, _y, _z, _w);
  }
  void set(double x, double y, double z, double w) {
    _x = x;
    _y = y;
    _z = z;
    _w = w;
  }
  void set(const Vector& v) {
    _x = v(0);
    _y = v(1);
    _z = v(2);
    _w = v(3);
  }
  Point3d to_point3d() {
    double w_inv = 1. / _w;
    return Point3d(_x*w_inv, _y*w_inv, _z*w_inv);
  }
  void write(std::ostream &out) const {
    out << "(" << _x << ", " << _y << ", " << _z << ", " << _w << ")";
  }
};

}
