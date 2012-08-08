/**
 * @file Pose3d.h
 * @brief 3D pose class.
 * @author Michael Kaess
 * @version $Id: Pose3d.h 6538 2012-04-23 04:05:21Z kaess $
 *
 * [insert iSAM license]
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
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "util.h"
#include "Rot3d.h"
#include "Pose2d.h"
#include "Point3d.h"
#include "Point3dh.h"
#include "Point2d.h"

namespace isam {

typedef Eigen::Matrix< double, 6, 1> Vector6d;

class Pose3d {
  friend std::ostream& operator<<(std::ostream& out, const Pose3d& p) {
    p.write(out);
    return out;
  }

  Point3d _t;
  Rot3d _rot;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const int dim = 6;
  static const char* name() {
    return "Pose3d";
  }

  Pose3d() : _t(0,0,0), _rot(0,0,0) {}

  Pose3d(double x, double y, double z, double yaw, double pitch, double roll) : _t(x, y, z), _rot(yaw, pitch, roll) {}

  Pose3d(const Eigen::MatrixXd& m) {
    if (m.rows()==6 && m.cols()==1) {
      _t = Point3d(m(0), m(1), m(2));
      _rot = Rot3d(m(3), m(4), m(5));
    } else if (m.rows()==4 && m.cols()==4) {
      // Convert a homogeneous 4x4 transformation matrix to a Pose3.
      Eigen::Matrix4d wTo = m / m(3,3); // enforce T(3,3)=1
      Eigen::Vector3d t = wTo.col(3).head(3);
      Eigen::Matrix3d wRo = wTo.topLeftCorner(3,3);
      _t = Point3d(t(0), t(1), t(2));
      _rot = Rot3d(wRo);
    } else {
      require(false, "Pose3d constructor called with matrix of wrong dimension");
    }
  }

  explicit Pose3d(const Eigen::Isometry3d & T) {
    Eigen::Vector3d t(T.translation());
    _t = Point3d(t(0), t(1), t(2));
    _rot = Rot3d(T.rotation());
  }

  Pose3d(const Point3d& t, const Rot3d& rot) : _t(t), _rot(rot) {}

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

  Pose3d exmap(const Vector6d& delta) const {
    Pose3d res = *this;
    res._t   = res._t.exmap(delta.head(3));
    res._rot = res._rot.exmap(delta.tail(3));
    return res;
  }

  Vector6d vector() const {
    double Y, P, R;
    // cheaper to recover ypr at once
    _rot.ypr(Y, P, R);
    Vector6d tmp;
    tmp << x(), y(), z(), Y, P, R;
    return tmp;
  }

  void set(double x, double y, double z, double yaw, double pitch, double roll) {
    _t = Point3d(x, y, z);
    _rot = Rot3d(yaw, pitch, roll);
  }

  void set(const Vector6d& v) {
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
   * Convert Pose3 to homogeneous 4x4 transformation matrix.
   * The returned matrix is the object coordinate frame in the world
   * coordinate frame. In other words it transforms a point in the object
   * frame to the world frame.
   *
   * @return wTo
   */
  Eigen::Matrix4d wTo() const {
    Eigen::Matrix4d T;
    T.topLeftCorner(3,3) = _rot.wRo();
    T.col(3).head(3) << x(), y(), z();
    T.row(3) << 0., 0., 0., 1.;
    return T;
  }

  /**
   * Convert Pose3 to homogeneous 4x4 transformation matrix. Avoids inverting wTo.
   * The returned matrix is the world coordinate frame in the object
   * coordinate frame. In other words it transforms a point in the world
   * frame to the object frame.
   *
   * @return oTw
   */
  Eigen::Matrix4d oTw() const {
    Eigen::Matrix3d oRw = _rot.wRo().transpose();
    Eigen::Vector3d t(x(), y(), z());
    Eigen::Vector3d C = - oRw * t;
    Eigen::Matrix4d T;
    T.topLeftCorner(3,3) = oRw;
    T.col(3).head(3) = C;
    T.row(3) << 0., 0., 0., 1.;
    return T;
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

// BEGIN REMOVE FROM RELEASE
#ifdef USE_QUATERNIONS
  /* Matlab generated
    syms q0 q1 q2 q3 xw yw zw ww x0 y0 z0 real
    wRo=[q0*q0+q1*q1-q2*q2-q3*q3 2*(q1*q2-q0*q3) 2*(q0*q2+q1*q3); 2*(q1*q2+q0*q3) q0*q0-q1*q1+q2*q2-q3*q3 2*(q2*q3-q0*q1); 2*(q1*q3-q0*q2) 2*(q0*q1+q2*q3) q0*q0-q1*q1-q2*q2+q3*q3]
    oTw = [wRo' -wRo'*[x0;y0;z0]; 0 0 0 1]
    Xc=oTw*[xw;yw;zw;ww]
    J=jacobian(Xc, [x0 y0 z0 q0 q1 q2 q3 xw yw zw ww])
    ccode(J, 'file', 'jacobian.c')
  */
  Eigen::MatrixXd transform_to_dpose_dpoint(const Point3dh& p) const {
    double xw = p.x();
    double yw = p.y();
    double zw = p.z();
    double ww = p.w();
    Eigen::Quaterniond q = _rot.quaternion();
    double q0 = q.w();
    double q1 = q.x();
    double q2 = q.y();
    double q3 = q.z();
    double x0 = _t.x();
    double y0 = _t.y();
    double z0 = _t.z();
    Eigen::MatrixXd M(4,11);
    double t2 = q0*q0;
    double t3 = q1*q1;
    double t4 = q2*q2;
    double t5 = q3*q3;
    double t6 = t2+t3-t4-t5;
    double t7 = q0*q3*2.0;
    double t8 = q1*q2*2.0;
    double t9 = t7+t8;
    double t10 = q0*q2*2.0;
    double t11 = q1*q3*2.0;
    double t12 = q0*y0*2.0;
    double t13 = q1*z0*2.0;
    double t45 = q3*x0*2.0;
    double t14 = t12+t13-t45;
    double t15 = q0*yw*2.0;
    double t16 = q1*zw*2.0;
    double t46 = t14*ww;
    double t47 = q3*xw*2.0;
    double t17 = t15+t16-t46-t47;
    double t18 = q2*x0*2.0;
    double t19 = q0*z0*2.0;
    double t41 = q1*y0*2.0;
    double t20 = t18+t19-t41;
    double t21 = t20*ww;
    double t22 = q1*yw*2.0;
    double t23 = q1*x0*2.0;
    double t24 = q2*y0*2.0;
    double t25 = q3*z0*2.0;
    double t26 = t23+t24+t25;
    double t27 = q1*xw*2.0;
    double t28 = q2*yw*2.0;
    double t29 = q3*zw*2.0;
    double t51 = t26*ww;
    double t30 = t27+t28+t29-t51;
    double t31 = q0*x0*2.0;
    double t32 = q3*y0*2.0;
    double t48 = q2*z0*2.0;
    double t33 = t31+t32-t48;
    double t34 = q0*xw*2.0;
    double t35 = q3*yw*2.0;
    double t36 = t2-t3+t4-t5;
    double t37 = q0*q1*2.0;
    double t38 = q2*q3*2.0;
    double t39 = t37+t38;
    double t40 = t7-t8;
    double t42 = q2*xw*2.0;
    double t43 = q0*zw*2.0;
    double t44 = -t21-t22+t42+t43;
    double t49 = t33*ww;
    double t50 = q2*zw*2.0;
    double t52 = t10+t11;
    double t53 = t2-t3-t4+t5;
    double t54 = t37-t38;
    M(0,0) = -t6*ww;
    M(0,1) = -t9*ww;
    M(0,2) = ww*(t10-q1*q3*2.0);
    M(0,3) = t34+t35-q2*zw*2.0-t33*ww;
    M(0,4) = t30;
    M(0,5) = t21+t22-q2*xw*2.0-q0*zw*2.0;
    M(0,6) = t17;
    M(0,7) = t6;
    M(0,8) = t9;
    M(0,9) = -t10+t11;
    M(0,10) = -t6*x0-t9*y0+z0*(t10-t11);
    M(1,0) = t40*ww;
    M(1,1) = -t36*ww;
    M(1,2) = -t39*ww;
    M(1,3) = t17;
    M(1,4) = t44;
    M(1,5) = t30;
    M(1,6) = -t34-t35+t49+t50;
    M(1,7) = -t7+t8;
    M(1,8) = t36;
    M(1,9) = t39;
    M(1,10) = t40*x0-t36*y0-t39*z0;
    M(2,0) = -t52*ww;
    M(2,1) = t54*ww;
    M(2,2) = -t53*ww;
    M(2,3) = t44;
    M(2,4) = -t15-t16+t46+t47;
    M(2,5) = t34+t35-t49-t50;
    M(2,6) = t30;
    M(2,7) = t52;
    M(2,8) = -t37+t38;
    M(2,9) = t53;
    M(2,10) = -t52*x0+t54*y0-t53*z0;
    M(3,10) = 1.0;

    Eigen::MatrixXd MM(4,9);
    MM.leftCols(3) = M.leftCols(3);
    MM.block(0,3, 4,3) = M.block(0,3, 4,4) * Rot3d::exmap_dquat(/*q*/);
    MM.rightCols(3) = M.rightCols(4) * Rot3d::exmap_dquat(/*p*/);
    return MM;
  }
#endif
// END REMOVE FROM RELEASE

  /**
   * Project point into this coordinate frame.
   * @param p Point to project
   * @return Point p locally expressed in this frame.
   */
  Point3d transform_to(const Point3d& p) const {
    return transform_to(Point3dh(p)).to_point3d();
  }

  /**
   * Project point from this coordinate frame.
   * @param p Point to project
   * @return Point p is expressed in the global frame.
   */
  Point3dh transform_from(const Point3dh& p) const {
    return Point3dh(wTo() * p.vector());
  }

  /**
   * Project point from this coordinate frame.
   * @param p Point to project
   * @return Point p is expressed in the global frame.
   */
  Point3d transform_from(const Point3d& p) const {
    return transform_from(Point3dh(p)).to_point3d();
  }

};

}
