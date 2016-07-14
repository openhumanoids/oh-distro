#include "zmp_planner.h"
#include "generate_spline.h"
#include <fstream>
#include <iostream>

int main() {
  ZMPPlanner zmp;

  std::vector<double> Ts;
  std::vector<Eigen::Vector2d> zmp_d;
  std::ofstream out;

  for (int i = 0; i < 5; i++) {
    Ts.push_back(i);
  }
  zmp_d.push_back(Eigen::Vector2d(0., 1));
  zmp_d.push_back(Eigen::Vector2d(0.1, 0.8));
  zmp_d.push_back(Eigen::Vector2d(0.4, 0.5));
  zmp_d.push_back(Eigen::Vector2d(0.7, 0.2));
  zmp_d.push_back(Eigen::Vector2d(0.8, 0.1));

  Eigen::Vector4d x0(0, 1, 0, 0);
  PiecewisePolynomial<double> zmp_traj_ = GeneratePCHIPSpline(Ts, zmp_d);
  zmp.Plan(zmp_traj_, x0, 1);

  std::cout << "S: " << zmp.S_ << std::endl;
  
  out.open("/home/sfeng/zmp_d");
  for (double t = Ts[0]; t <= Ts[Ts.size()-1]; t+= 0.01) {
    out << t << " " << zmp.zmp_traj_.value(t).transpose() << std::endl;
  }
  out.close();

  out.open("/home/sfeng/com");
  for (double t = Ts[0]; t <= Ts[Ts.size()-1]; t+= 0.01) {
    out << t << " " << zmp.com_traj_.value(t).transpose() << std::endl;
  }
  out.close(); 

  out.open("/home/sfeng/s1");
  for (double t = Ts[0]; t <= Ts[Ts.size()-1]; t+= 0.01) {
    out << t << " " << zmp.s1_traj_.value(t).transpose() << std::endl;
  }
  out.close();
  
}                             
