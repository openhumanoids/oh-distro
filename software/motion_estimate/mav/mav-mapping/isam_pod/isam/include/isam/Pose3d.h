/**
 * @file Pose3d.h
 * @brief 3D pose class.
 * @author Michael Kaess
 * @version $Id: Pose3d.h 2896 2010-08-23 20:37:44Z kaess $
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

/** @class isam::Pose3d
 *
 * Conventions:
 *
 * Right-handed coordinate system (NED: north-east-down)
 * X forward (along default motion of robot)
 * Y right
 * Z down
 *
 * Rotations are represented using standard Euler angles
 * yaw
 * pitch
 * roll
 *
 * Note that Euler angles transform objects from the global into the local frame
 * of the vehicle: First yaw rotates around Z (changing X and Y axes to X' and Y'),
 * then pitch around the new Y' axis, and finally roll around the new X' axis.
 *
 * In contrast, the returned rotation and transformation matrices are defined
 * in the opposite direction:
 * wTo transforms a point from the local (second) system to the global (first) system
 *
 */

#pragma once

#include <cmath>

#include "util.h"
#include "Vector.h"
#include "Rot3d.h"
#include "Pose2d.h"
#include "Point3d.h"
#include "Point3dh.h"
#include "Point2d.h"

namespace isam {

class Pose3d {
  friend std::ostream& operator<<(std::ostream& out, const Pose3d& p) {
    p.write(out);
    return out;
  }

  Point3d _t;
  Rot3d _rot;
public:
  static const int dim = 6;
  static const char* name() {
    return "Pose3d";
  }

  Pose3d() {} // everything 0.
  Pose3d(double x, double y, double z, double yaw, double pitch, double roll) : _t(x, y, z), _rot(yaw, pitch, roll) {}
  Pose3d(const Vector& vec) : _t(vec(0), vec(1), vec(2)), _rot(vec(3), vec(4), vec(5)) {}

  double x() const {return _t.x();}
  double y() const {return _t.y();}
  double z() const {return _t.z();}
  double yaw()   const {return _rot.yaw();}
  double pitch() const {return _rot.pitch();}
  double roll()  const {return _rot.roll();}

  Point3d trans() const {return _t;}
  Rot3d rot() const {return _rot;}

  void set_x(double x) {_t.set_x(x);}
  void set_y(double y) {_t.set_y(y);}
  void set_z(double z) {_t.set_z(z);}
  void set_yaw  (double yaw)   {_rot.set_yaw(yaw);}
  void set_pitch(double pitch) {_rot.set_pitch(pitch);}
  void set_roll (double roll)  {_rot.set_roll(roll);}

  Vector vector() const {
    return make_Vector(6, x(), y(), z(), yaw(), pitch(), roll());
  }

  void set(double x, double y, double z, double yaw, double pitch, double roll) {
    _t = Point3d(x, y, z);
    _rot = Rot3d(yaw, pitch, roll);
  }
  void set(const Vector& v) {
    _t = Point3d(v(0), v(1), v(2));
    _rot = Rot3d(standardRad(v(3)), standardRad(v(4)), standardRad(v(5)));
  }

  void of_pose2d(const Pose2d& p) {
    set(p.x(), p.y(), 0., p.t(), 0., 0.);
  }

  void of_point2d(const Point2d& p) {
    set(p.x(), p.y(), 0., 0., 0., 0.);
  }

  void of_point3d(const Point3d& p) {
    set(p.x(), p.y(), p.z(), 0., 0., 0.);
  }

  void write(std::ostream &out) const {
    out << "(" << x() << ", " << y() << ", " << z() << "; "
        << yaw() << ", " << pitch() << ", " << roll() << ")";
  }

  /**
   * Convert a homogeneous 4x4 transformation matrix to a Pose3.
   * @param wTo 4x4 transformation matrix.
   */
  Pose3d(const Matrix& wTo) {
    Matrix T = wTo / wTo(3,3); // enforce T(3,3)=1
    Matrix t = Matrix(0,3,3,1,T);
    Matrix wRo = Matrix(0,0,3,3,T);
    _t = Point3d(t(0,0), t(1,0), t(2,0));
    _rot = Rot3d(wRo);
  }

  /**
   * Convert Pose3 to homogeneous 4x4 transformation matrix.
   * @return wTo
   */
  Matrix wTo() const {
    Matrix R = _rot.wRo();
    Matrix t = make_Matrix(3,1, x(), y(), z());
    Matrix unit = Matrix::unit(4,3).transpose();
    return ((R|t) ^ unit);
  }

  /**
   * Convert Pose3 to homogeneous 4x4 transformation matrix. Avoids inverting wTo.
   * @return oTw
   */
  Matrix oTw() const {
    Matrix oRw = _rot.wRo().transpose();
    Matrix t = make_Matrix(3,1, x(), y(), z());
    Matrix C = - oRw * t;
    Matrix unit = Matrix::unit(4,3).transpose();
    return ((oRw|C) ^ unit);
  }

  /**
   * Calculate new pose b composed from this pose (a) and the odometry d.
   * Follows notation of Lu&Milios 1997.
   * \f$ b = a \oplus d \f$
   * @param d Pose difference to add.
   * @return d transformed from being local in this frame (a) to the global frame.
   */
  Pose3d oplus(const Pose3d& d) const {
    return Pose3d(wTo() * d.wTo());
  }

  /**
   * Odometry d from b to this pose (a). Follows notation of
   * Lu&Milios 1997.
   * \f$ d = a \ominus b \f$
   * @param b Base frame.
   * @return Global this (a) expressed in base frame b.
   */
  Pose3d ominus(const Pose3d& b) const {
    return Pose3d(b.oTw() * wTo());
  }

  /**
   * Project point into this coordinate frame.
   * @param p Point to project
   * @return Point p locally expressed in this frame.
   */
  Point3dh transform_to(const Point3dh& p) const {
    return Point3dh(oTw() * p.vector());
  }

  Point3d transform_to(const Point3d& p) const {
    return transform_to(Point3dh(p)).to_point3d();
  }

  Point3dh transform_from(const Point3dh& p) const {
    return Point3dh(wTo() * p.vector());
  }

  Point3d transform_from(const Point3d& p) const {
    return transform_from(Point3dh(p)).to_point3d();
  }

};

}
