#include "Utils.hpp"

#include <random>
#include <chrono>

using namespace maps;

uint64_t Utils::
rand64() {
  std::chrono::high_resolution_clock::duration duration =
    std::chrono::high_resolution_clock::now().time_since_epoch();
  std::default_random_engine generator;
  generator.seed(duration.count());
  std::uniform_int_distribution<uint64_t> distribution;
  return distribution(generator);
}


bool Utils::
isOrthographic(const Eigen::Matrix4f& iMatrix) {
  const Eigen::Vector4f unitW(0,0,0,1);
  float dot = fabs(iMatrix.row(3).normalized()[3]);
  return (fabs(dot-1) < 1e-6);
}

bool Utils::
composeViewMatrix(Eigen::Projective3f& oMatrix, const Eigen::Matrix3f& iCalib,
                  const Eigen::Isometry3f& iPose, const bool iIsOrthographic) {
  Eigen::Projective3f calib = Eigen::Projective3f::Identity();
  calib.matrix().col(2).swap(calib.matrix().col(3));
  calib.matrix().topLeftCorner<2,3>() = iCalib.matrix().topRows<2>();
  calib.matrix().bottomLeftCorner<1,3>() = iCalib.matrix().bottomRows<1>();
  if (iIsOrthographic) calib.matrix().col(2).swap(calib.matrix().col(3));
  oMatrix = calib*iPose.inverse();
  return true;
}

bool Utils::
factorViewMatrix(const Eigen::Projective3f& iMatrix,
                 Eigen::Matrix3f& oCalib, Eigen::Isometry3f& oPose,
                 bool& oIsOrthographic) {
  oIsOrthographic = isOrthographic(iMatrix.matrix());

  // get appropriate rows
  std::vector<int> rows = {0,1,2};
  if (!oIsOrthographic) rows[2] = 3;

  // get A matrix (upper left 3x3) and t vector
  Eigen::Matrix3f A;
  Eigen::Vector3f t;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      A(i,j) = iMatrix(rows[i],j);
    }
    t[i] = iMatrix(rows[i],3);
  }

  // determine translation vector
  oPose.setIdentity();
  oPose.translation() = -(A.inverse()*t);

  // determine calibration matrix
  Eigen::Matrix3f AAtrans = A*A.transpose();
  AAtrans.col(0).swap(AAtrans.col(2));
  AAtrans.row(0).swap(AAtrans.row(2));
  Eigen::LLT<Eigen::Matrix3f, Eigen::Upper> llt(AAtrans);
  oCalib = llt.matrixU();
  oCalib.col(0).swap(oCalib.col(2));
  oCalib.row(0).swap(oCalib.row(2));
  oCalib.transposeInPlace();

  // compute rotation matrix
  oPose.linear() = (oCalib.inverse()*A).transpose();

  return true;
}
