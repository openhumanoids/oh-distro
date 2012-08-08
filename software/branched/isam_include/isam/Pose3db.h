/**
 * @file Pose3db.h
 * @brief Bearing-only 3D pose class.
 * @author Ayoung Kim (ayoungk@umich.edu)
 * @version $Id: Pose3db.h 4133 2011-03-22 20:40:38Z kaess $
 *
 */

/* Conventions:
 *
 * Azimuth (bearing in xy plane) and elevation
 *
 * For convention of rotations see Pose3d.h
 *
 */

#pragma once

#include <cmath>
#include <Eigen/Dense>

#include "util.h"
#include "Rot3d.h"

#define RTOD 180/PI
namespace isam {

class Bearing3d {
  friend std::ostream& operator<<(std::ostream& out, const Bearing3d& p) {
    p.write(out);
    return out;
  }

  double _a;
  double _e;

public:
  static const int dim = 2;
  static const char* name() {
    return "Bearing3d";
  }
  Bearing3d() : _a(0.), _e(0.) {}
  Bearing3d(double a, double e) : _a(a), _e(e) {}
  Bearing3d(const Eigen::VectorXd& vec) : _a(vec(0)), _e(vec(1)) {}
  double a() const {return _a;}
  double e() const {return _e;}

  Eigen::VectorXd vector() const {
    Eigen::VectorXd tmp(2);
    tmp << _a, _e;
    return tmp;
  }
  void set(double a, double e) {
    _a = a;
    _e = e;
  }
  void set(const Eigen::VectorXd& v) {
    _a = v(0);
    _e = v(1);
  }
  void set_trans2dm (const Eigen::VectorXd& v) {
    // compute b
    const double x = v(0); // x = mag*cos(elev)*cos(azim)
    const double y = v(1); // y = mag*cos(elev)*sin(azim)
    const double z = v(2); // z = mag*sin(elev)

    _a = atan2 (y, x);
    _e = atan2 (z, sqrt (x*x+y*y));

    /* analytical expression for jacobian
     * J = [-y/alpha^2,          x/alpha^2,         0; ...
     *      -z*x/(alpha*mag^2), -z*y/(alpha*mag^2), alpha/mag^2; ...
     *       x/mag,              y/mag,             z/mag];
     */
    //J[0]=-y/(alpha*alpha);     J[1]=x/(alpha*alpha);      J[2]=0;
    //J[3]=-z*x/(alpha*mag*mag); J[4]=-z*y/(alpha*mag*mag); J[5]=alpha/(mag*mag);
    //J[6]=x/mag;                J[7]=y/mag;                J[8]=z/mag;
    

  }

  void write(std::ostream &out) const {
    out << "(" << _a << ", " << _e << ")";
  }

  Eigen::VectorXd bearing2trans(double mag) {
    double se, ce, sa, ca;
    se = sin(_e); ce = cos(_e);
    sa = sin(_a); ca = cos(_a);

    double tz = mag*se;
    double tx = mag*ce*ca;
    double ty = mag*ce*sa;

    /* analytical expression for jacobian matrix
     * J = [-mag*ce*sa, -mag*se*ca, ce*ca; ...
     *       mag*ce*ca, -mag*se*sa, ce*sa; ...
     *               0,     mag*ce,    se];
     */
    //J[0]=-mag*ce*sa; J[1]=-mag*se*ca; J[2]=ce*ca;
    //J[3]= mag*ce*ca; J[4]=-mag*se*sa; J[5]=ce*sa;
    //J[6]= 0;         J[7]= mag*ce;    J[8]=se;

    Eigen::VectorXd tmp(3);
    tmp << tx, ty, tz;
    return tmp;
  }
};

class Pose3db {
  friend std::ostream& operator<<(std::ostream& out, const Pose3db& p) {
    p.write(out);
    return out;
  }

  Bearing3d _b;
  Rot3d _rot;
public:
  static const int dim = 5;
  static const char* name() {
    return "Pose3db";
  }

  Pose3db() {} // everything 0.
  Pose3db(double a, double e, double yaw, double pitch, double roll) : _b(a, e), _rot(yaw, pitch, roll) {}
  Pose3db(const Eigen::VectorXd& vec) : _b(vec(0), vec(1)), _rot(vec(2), vec(3), vec(4)) {}

  Rot3d rot() const {return _rot;}

  double a() const {return _b.a();}
  double e() const {return _b.e();}
  double yaw()   const {return _rot.yaw();}
  double pitch() const {return _rot.pitch();}
  double roll()  const {return _rot.roll();}

  Bearing3d bearing() const {return _b;}

  Eigen::VectorXd vector() const {
    Eigen::VectorXd tmp(5);
    tmp << a(), e(), yaw(), pitch(), roll();
    return tmp;
  }

  void set(double a, double e, double yaw, double pitch, double roll) {
    _b = Bearing3d(a, e);
    _rot = Rot3d(yaw, pitch, roll);
  }
  void set(const Eigen::VectorXd& v) {
    _b = Bearing3d(v(0), v(1));
    _rot = Rot3d(standardRad(v(2)), standardRad(v(3)), standardRad(v(4)));
  }

  void of_pose3d(const Pose3d& p) {
    double x = p.x();
    double y = p.y();
    double z = p.z();

    double a = atan2 (y, x);
    double e = atan2 (z, sqrt (x*x+y*y));

    set(a, e, p.yaw(), p.pitch(), p.roll());
  }

  void write(std::ostream &out) const {
    out << "(" << a() << ", " << e() << "; "
        << yaw() << ", " << pitch() << ", " << roll() << ")";
  }

  void write_indeg(std::ostream &out) const {
    out << "(" << a()*RTOD << ", " << e()*RTOD << "; "
        << yaw()*RTOD << ", " << pitch()*RTOD << ", " << roll()*RTOD << ")";
  }

};

}
