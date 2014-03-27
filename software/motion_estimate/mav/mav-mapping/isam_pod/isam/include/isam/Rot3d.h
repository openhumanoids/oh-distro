/**
 * @file Rot3d.h
 * @brief 3D rotation class.
 * @author Michael Kaess
 * @version $Id: Rot3d.h 2925 2010-08-27 17:48:05Z kaess $
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

/** @class isam::Rot3d
 *
 * For conventions, see isam::Pose3d.h
 */

#pragma once

#include <cmath>

namespace isam {

class Rot3d {
  double _yaw;
  double _pitch;
  double _roll;

public:
  Rot3d() : _yaw(0.), _pitch(0.), _roll(0.) {}
  Rot3d(double yaw, double pitch, double roll) : _yaw(yaw), _pitch(pitch), _roll(roll) {}

  /**
   * Initialize Euler angles from 3x3 wRo rotation matrix.
   */
  Rot3d(const Matrix& wRo) {
    // note that getting the sign right requires recovering both sin and cos
    // for each angle; some equations exploit the fact that sin^2+cos^2=1
    _yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(_yaw);
    double s = sin(_yaw);
    _pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    _roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
  }

  double yaw()   const {return _yaw;}
  double pitch() const {return _pitch;}
  double roll()  const {return _roll;}

  void set_yaw  (double yaw)   {_yaw = yaw;}
  void set_pitch(double pitch) {_pitch = pitch;}
  void set_roll (double roll)  {_roll = roll;}

  /**
   * Generate 3x3 rotation matrix from Rot3d.
   * @return wRo
   */
  Matrix wRo() const {
    double c__ = cos(_yaw);
    double _c_ = cos(_pitch);
    double __c = cos(_roll);
    double s__ = sin(_yaw);
    double _s_ = sin(_pitch);
    double __s = sin(_roll);
    double cc_ = c__ * _c_;
    double cs_ = c__ * _s_;
    double sc_ = s__ * _c_;
    double ss_ = s__ * _s_;
    double c_c = c__ * __c;
    double c_s = c__ * __s;
    double s_c = s__ * __c;
    double s_s = s__ * __s;
    double _cc = _c_ * __c;
    double _cs = _c_ * __s;
    double csc = cs_ * __c;
    double css = cs_ * __s;
    double ssc = ss_ * __c;
    double sss = ss_ * __s;
    Matrix wRo = make_Matrix(3,3,
      cc_  , css-s_c,  csc+s_s,
      sc_  , sss+c_c,  ssc-c_s,
     -_s_  ,     _cs,      _cc);
    return wRo;
  }
};

}

