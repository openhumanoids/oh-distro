#include "pointcloud_math.hpp"


void scale_quaternion(double r,Eigen::Quaterniond q,Eigen::Quaterniond &q_out){
  if (q.w() ==1){
    // BUG: 15dec2011
    // TODO: check if this is a rigerous solution
    // q.w() ==1 mean no translation. so just return the input
    // if q.w()==1 and r=0.5 ... what is created below will have w=0 ... invalid
    //std::cout << "unit quaternion input:\n";
    q_out = q;
    return;
  }
  double theta = acos(q.w());
  //double sin_theta_inv = 1/ sin(theta);
  q_out.w() = sin((1-r)*theta)  + sin(r*theta) * q.w();
  q_out.x() = sin(r*theta) * q.x();
  q_out.y() = sin(r*theta) * q.y();
  q_out.z() = sin(r*theta) * q.z();
  q_out.normalize();
}


Eigen::Quaterniond euler_to_quat(double yaw, double pitch, double roll) {
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}


Eigen::Quaternionf euler_to_quat_f(double yaw, double pitch, double roll) {
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaternionf(w,x,y,z);
}

void quat_to_euler(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}


void print_Isometry3d(Eigen::Isometry3d pose, std::stringstream &ss){
  Eigen::Vector3d t(pose.translation());
  Eigen::Quaterniond r(pose.rotation());
  ss <<t[0]<<", "<<t[1]<<", "<<t[2]<<" | " 
       <<r.w()<<", "<<r.x()<<", "<<r.y()<<", "<<r.z() ;
  //  std::cout << ss.str() << "q\n";
}

void print_Quaterniond(Eigen::Quaterniond r, std::stringstream &ss){
  ss <<r.w()<<", "<<r.x()<<", "<<r.y()<<", "<<r.z() ;
  //  std::cout << r.str() << "q\n";
}
