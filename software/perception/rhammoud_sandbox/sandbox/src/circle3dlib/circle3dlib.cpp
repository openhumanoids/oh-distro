#include "circle3dlib.hpp"


Circle3D::Circle3D(){
  std::cout << "finished constructor\n";
}


void Circle3D::setWidth(int width){
  width_ = width;
}

bool Circle3D::doCircle3D(pcl::PointCloud<pcl::PointXYZ> &cloud){

  std::cout << "doAlgorithm here\n";
  return true;
}
