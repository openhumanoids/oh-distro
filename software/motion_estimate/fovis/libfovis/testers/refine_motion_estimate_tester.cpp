#include <stdio.h>
#include <iostream>

#include <Eigen/Geometry>

#include "../libfovis/refine_motion_estimate.hpp"

#define dump(v) std::cerr << #v << " : " << (v) << "\n"
#define dumpT(v) std::cerr << #v << " : " << (v).transpose() << "\n"

using namespace fovis;

static inline Eigen::Isometry3d
isometryFromXYZRollPitchYaw(const Eigen::Matrix<double, 6, 1>& params)
{
  Eigen::Isometry3d result;

  double roll = params(3), pitch = params(4), yaw = params(5);
  double halfroll = roll / 2;
  double halfpitch = pitch / 2;
  double halfyaw = yaw / 2;
  double sin_r2 = sin(halfroll);
  double sin_p2 = sin(halfpitch);
  double sin_y2 = sin(halfyaw);
  double cos_r2 = cos(halfroll);
  double cos_p2 = cos(halfpitch);
  double cos_y2 = cos(halfyaw);

  Eigen::Quaterniond quat(
    cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2,
    sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2,
    cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2,
    cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2);

  result.setIdentity();
  result.translate(params.head<3>());
  result.rotate(quat);

  return result;
}

static inline Eigen::Vector3d
isometryGetRollPitchYaw(const Eigen::Isometry3d& M)
{
  Eigen::Quaterniond q(M.rotation());
  double roll_a = 2 * (q.w()*q.x() + q.y()*q.z());
  double roll_b = 1 - 2 * (q.x()*q.x() + q.y()*q.y());
  double pitch_sin = 2 * (q.w()*q.y() - q.z()*q.x());
  double yaw_a = 2 * (q.w()*q.z() + q.x()*q.y());
  double yaw_b = 1 - 2 * (q.y()*q.y() + q.z()*q.z());

  return Eigen::Vector3d(atan2(roll_a, roll_b),
      asin(pitch_sin),
      atan2(yaw_a, yaw_b));
}

int main(int argc, char** argv)
{

  int num_points = 9;

  Eigen::Matrix<double, 4, Eigen::Dynamic> points_xyz(4, num_points);
  Eigen::Matrix<double, 4, Eigen::Dynamic> transformed_xyz(4, num_points);
  Eigen::Matrix<double, 2, Eigen::Dynamic> ref_projections(2, num_points);

  double tx    = 1.0;
  double ty    = 1.5;
  double tz    = 0.5;
  double roll  = 10 * (M_PI / 180);
  double pitch =  1 * (M_PI / 180);
  double yaw   =  5 * (M_PI / 180);
  //	tx = 0;
  //	ty = 0;
  //	tz = 0;
  //	roll = 0;
  //	pitch = 0;
  //	yaw = 1 * (M_PI / 180);

  Eigen::Matrix<double, 6, 1> params;
  params << tx, ty, tz, roll, pitch, yaw;
  Eigen::Isometry3d true_motion = isometryFromXYZRollPitchYaw(params);
  double fx = 528;
  double cx = 320;
  double cy = 240;

  Eigen::Matrix<double, Eigen::Dynamic, 2> tmp(num_points, 2);
  tmp << 
    0, 0,
    320, 0,
    640, 0,
    0, 240,
    320, 240,
    640, 240,
    0, 480,
    320, 480,
    640, 480;
  ref_projections = tmp.transpose();
  double depths[] = {
    1, 0.5, 0.75,
    100, 1, 0.75,
    0.75, 0.5, 1,
  };

  Eigen::Matrix<double, 3, 4> K;
  K << fx, 0, cx, 0,
    0, fx, cy, 0,
    0, 0, 1, 0;

  for(int i=0; i<num_points; i++) {
    points_xyz(0, i) = depths[i] * (ref_projections(0, i) - cx) / fx;
    points_xyz(1, i) = depths[i] * (ref_projections(1, i) - cy) / fx;
    points_xyz(2, i) = depths[i];
    points_xyz(3, i) = 1;

    Eigen::Vector3d uvw = K * points_xyz.col(i);
    uvw(0) /= uvw(2);
    uvw(1) /= uvw(2);
    uvw(2) = 1;

    transformed_xyz.col(i) = true_motion.inverse().matrix() * points_xyz.col(i);

    Eigen::Vector4d t = transformed_xyz.col(i);
    Eigen::Vector4d p = points_xyz.col(i);

    printf("%3d : %6.2f %6.2f  ->  %6.2f %6.2f %6.2f  -> %7.3f %7.3f %7.3f\n",
        i,
        uvw(0), uvw(1),
        p(0), p(1), p(2),
        t(0), t(1), t(2));
  }
  printf("=======\n");


  Eigen::Isometry3d initial_estimate;
  initial_estimate.setIdentity();
  Eigen::Vector3d initial_rpy = isometryGetRollPitchYaw(initial_estimate);
  Eigen::Vector3d initial_trans = initial_estimate.translation();

  Eigen::Isometry3d estimate = refineMotionEstimate(transformed_xyz,
      ref_projections,
      fx, cx, cy,
      initial_estimate, 6);

  Eigen::Matrix<double, 3, 4> P = K * estimate.matrix();

  Eigen::Vector3d estimated_rpy = isometryGetRollPitchYaw(estimate);
  Eigen::Vector3d estimated_trans = estimate.translation();
  printf("       Estimate   True    Initial\n"
      "tx:    %6.2f %6.2f %6.2f\n"
      "ty:    %6.2f %6.2f %6.2f\n"
      "tz:    %6.2f %6.2f %6.2f\n"
      "roll:  %6.2f %6.2f %6.2f\n"
      "pitch: %6.2f %6.2f %6.2f\n"
      "yaw:   %6.2f %6.2f %6.2f\n",
      estimated_trans(0), tx, initial_trans(0), 
      estimated_trans(1), ty, initial_trans(1), 
      estimated_trans(2), tz, initial_trans(2), 
      estimated_rpy(0) * 180 / M_PI, roll * 180 / M_PI,  initial_rpy(0) * 180 / M_PI,
      estimated_rpy(1) * 180 / M_PI, pitch * 180 / M_PI, initial_rpy(1) * 180 / M_PI,
      estimated_rpy(2) * 180 / M_PI, yaw * 180 / M_PI,   initial_rpy(2) * 180 / M_PI);

  // compute reprojection error
  for(int i=0; i<num_points; i++) {
    Eigen::Vector3d uvw = P * transformed_xyz.col(i);
    double u = uvw(0) / uvw(2);
    double v = uvw(1) / uvw(2);
    double ref_u = ref_projections(0, i);
    double ref_v = ref_projections(1, i);
    double err_u = u - ref_u;
    double err_v = v - ref_v;

    printf("%3d  %6.1f %6.1f -> %6.1f %6.1f (%6.2f %6.2f)\n", i, ref_u, ref_v, u, v, err_u, err_v);
  }

  return 0;
}
