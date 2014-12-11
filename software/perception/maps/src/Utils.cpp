#include "Utils.hpp"

#include <random>
#include <chrono>

using namespace maps;

// helper functions
template<int Idx, int Sgn>
static inline bool
clip(const Eigen::Vector3f& iOrigin, const Eigen::Vector3f& iDirection,
     const Eigen::Vector3f& iPlanePoint,
     Eigen::Vector3f& oOrigin, Eigen::Vector3f& oEndPoint,
     float& oMinT, float& oMaxT) {
  float planeDist = -Sgn*iPlanePoint[Idx];
  float dist1 = Sgn*oOrigin[Idx] + planeDist;
  float dist2 = Sgn*oEndPoint[Idx] + planeDist;
  if ((dist1 < 0) && (dist2 < 0)) return false;
  if ((dist1 >= 0) && (dist2 >= 0)) return true;
  float t = -dist1/(Sgn*iDirection[Idx]);
  if (dist1 >= 0) {
    oMaxT = t;
    oEndPoint = iOrigin + iDirection*t;
  }
  if (dist2 >= 0) {
    oMinT = t;
    oOrigin = iOrigin + iDirection*t;
  }
  return true;
}



bool Utils::
clipRay(const Eigen::Vector3f& iOrigin, const Eigen::Vector3f& iEndPoint,
        const Eigen::Vector3f& iBoxMin, const Eigen::Vector3f& iBoxMax,
        Eigen::Vector3f& oOrigin, Eigen::Vector3f& oEndPoint,
        float& oMinT, float& oMaxT) {

  Eigen::Vector3f dir = iEndPoint - iOrigin;
  oMinT = 0;
  oMaxT = 1;
  oOrigin = iOrigin;
  oEndPoint = iEndPoint;

  // n'x = -d
  // x = o + r t
  // -d = n'o + n'r t
  // t = -(d+n'o)/n'r

  // xmin plane:  1,0,0,-xmin
  if (!clip<0,1>(iOrigin, dir, iBoxMin, oOrigin, oEndPoint, oMinT, oMaxT)) {
    return false;
  }

  // xmax plane: -1,0,0,xmax
  if (!clip<0,-1>(iOrigin, dir, iBoxMax, oOrigin, oEndPoint, oMinT, oMaxT)) {
    return false;
  }

  // ymin plane:  0,1,0,-ymin
  if (!clip<1,1>(iOrigin, dir, iBoxMin, oOrigin, oEndPoint, oMinT, oMaxT)) {
    return false;
  }

  // ymax plane:  0,-1,0,ymax
  if (!clip<1,-1>(iOrigin, dir, iBoxMax, oOrigin, oEndPoint, oMinT, oMaxT)) {
    return false;
  }

  // zmin plane:  0,0,1,-zmin
  if (!clip<2,1>(iOrigin, dir, iBoxMin, oOrigin, oEndPoint, oMinT, oMaxT)) {
    return false;
  }

  // zmax plane:  0,0,-1,zmax
  if (!clip<2,-1>(iOrigin, dir, iBoxMax, oOrigin, oEndPoint, oMinT, oMaxT)) {
    return false;
  }

  return true;
}

bool Utils::
clipRay(const Eigen::Vector3f& iOrigin, const Eigen::Vector3f& iEndPoint,
        const std::vector<Eigen::Vector4f>& iPlanes,
        Eigen::Vector3f& oOrigin, Eigen::Vector3f& oEndPoint,
        float& oMinT, float& oMaxT) {
  Eigen::Vector3f dir = iEndPoint - iOrigin;
  oMinT = 0;
  oMaxT = 1;
  oOrigin = iOrigin;
  oEndPoint = iEndPoint;
  for (int i = 0; i < iPlanes.size(); ++i) {
    Eigen::Vector4f plane = iPlanes[i];
    Eigen::Vector3f normal(plane[0], plane[1], plane[2]);
    float dist1 = normal.dot(oOrigin) + plane[3];
    float dist2 = normal.dot(oEndPoint) + plane[3];
    if ((dist1 < 0) && (dist2 < 0)) return false;
    if ((dist1 >= 0) && (dist2 >= 0)) continue;
    float t = (normal.dot(iOrigin) + plane[3]) / normal.dot(dir);
    if (dist1 >= 0) {
      oMaxT = t;
      oEndPoint = iOrigin + dir*t;
    }
    if (dist2 >= 0) {
      oMinT = t;
      oOrigin = iOrigin + dir*t;
    }
  }
  return true;
}

std::vector<Eigen::Vector4f> Utils::
planesFromBox(const Eigen::Vector3f& iBoundMin,
              const Eigen::Vector3f& iBoundMax) {
  std::vector<Eigen::Vector4f> planes(6);
  planes[0] = Eigen::Vector4f( 1, 0, 0, -iBoundMin[0]);
  planes[1] = Eigen::Vector4f(-1, 0, 0,  iBoundMax[0]);
  planes[2] = Eigen::Vector4f( 0, 1, 0, -iBoundMin[1]);
  planes[3] = Eigen::Vector4f( 0,-1, 0,  iBoundMax[1]);
  planes[4] = Eigen::Vector4f( 0, 0, 1, -iBoundMin[2]);
  planes[5] = Eigen::Vector4f( 0, 0,-1,  iBoundMax[2]);
  return planes;
}


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
polyhedronFromPlanes(const std::vector<Eigen::Vector4f>& iPlanes,
                     std::vector<Eigen::Vector3f>& oVertices,
                     std::vector<std::vector<int> >& oFaces,
                     const double iTol) {
  const double kTol = iTol;

  // first find all 3-plane intersections
  int numPlanes = iPlanes.size();
  Eigen::Matrix<double,3,4> planeMatrix;
  for (int i = 0; i < numPlanes; ++i) {
    planeMatrix.row(0) = iPlanes[i].cast<double>();
    for (int j = i+1; j < numPlanes; ++j) {
      planeMatrix.row(1) = iPlanes[j].cast<double>();
      for (int k = j+1; k < numPlanes; ++k) {
        planeMatrix.row(2) = iPlanes[k].cast<double>();

        // compute intersection
        Eigen::FullPivLU<Eigen::Matrix<double,3,4> > lu(planeMatrix);
        Eigen::MatrixXd nullSpace = lu.kernel();
        if (nullSpace.cols() != 1) {
          continue;
        }
        double denom = nullSpace(3,0);
        if (fabs(denom) < kTol) {
          continue;
        }
        Eigen::Vector3f pt = (nullSpace.topRows<3>()/denom).cast<float>();

        // evaluate against all planes
        bool success = true;
        for (int p = 0; p < numPlanes; ++p) {
          const Eigen::Vector4f& plane = iPlanes[p];
          double dist =
            pt[0]*plane[0] + pt[1]*plane[1] + pt[2]*plane[2] + plane[3];
          if (dist < -kTol) {
            success = false;
            break;
          }
        }
        if (!success) {
          continue;
        }

        // keep this intersection if it is on or inside the polyhedron
        oVertices.push_back(pt);
      }
    }
  }

  // form point-plane incidence matrix
  int numPoints = oVertices.size();
  Eigen::MatrixXf incidence(numPlanes, numPoints);
  for (int i = 0; i < numPoints; ++i) {
    const Eigen::Vector3f& pt = oVertices[i];
    for (int j = 0; j < numPlanes; ++j) {
      const Eigen::Vector4f& plane = iPlanes[j];
      incidence(j,i) = 0;
      double dist =
        pt[0]*plane[0] + pt[1]*plane[1] + pt[2]*plane[2] + plane[3];
      if (fabs(dist) < kTol) {
        incidence(j,i) = 1;
      }
    }
  }

  // form point-point adjacency matrix
  Eigen::MatrixXf adjacency = incidence.transpose()*incidence -
    Eigen::MatrixXf::Identity(numPoints, numPoints)*2;
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> adj =
    (adjacency.array() >= 2);

  // form faces as cycles
  // TODO: make this more efficient
  for (int i = 0; i < numPlanes; ++i) {

    // find indices of points incident on this face
    std::vector<int> indices;
    indices.reserve(numPoints);
    for (int j = 0; j < numPoints; ++j) {
      if (incidence(i,j) > 0) {
        indices.push_back(j);
      }
    }
    int numIndices = indices.size();
    if (numIndices < 3) {
      continue;
    }

    // extract adjacency sub-matrix
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>
      adjSub(numIndices, numIndices);
    for (int j = 0; j < numIndices; ++j) {
      for (int k = 0; k < numIndices; ++k) {
        adjSub(j,k) = adj(indices[j], indices[k]);
      }
    }

    // find cycle using adjacency sub-matrix
    std::vector<bool> hits(numIndices);
    std::fill(hits.begin(), hits.end(), false);
    std::vector<int> face(numIndices);
    face[0] = 0;
    hits[0] = true;
    for (int j = 1; j < numIndices; ++j) {
      for (int k = 0; k < numIndices; ++k) {
        if (!adjSub(face[j-1],k) || hits[k]) {
          continue;
        }
        face[j] = k;
        hits[k] = true;
        break;
      }
    }

    // remap to real vertex indices
    for (int j = 0; j < face.size(); ++j) {
      face[j] = indices[face[j]];
    }

    // ensure ordering is consistent with face normal
    Eigen::Vector3f normal = iPlanes[i].head<3>();
    Eigen::Vector3f v1 = oVertices[face[1]] - oVertices[face[0]];
    Eigen::Vector3f v2 = oVertices[face[2]] - oVertices[face[1]];
    Eigen::Vector3f crossProd = v1.cross(v2);
    if (normal.dot(crossProd) < 0) {
      std::reverse(face.begin(), face.end());
    }

    // add face
    oFaces.push_back(face);
  }
  return true;
}

std::vector<Eigen::Vector4f> Utils::
planesFromPolyhedron(const std::vector<Eigen::Vector3f>& iVertices,
                     const std::vector<std::vector<int> >& iFaces) {
  std::vector<Eigen::Vector4f> planes;
  planes.reserve(iFaces.size());
  for (int i = 0; i < planes.size(); ++i) {
    if (iFaces[i].size() < 3) {
      continue;
    }
    Eigen::Vector3f v1 = iVertices[iFaces[i][1]] - iVertices[iFaces[i][0]];
    Eigen::Vector3f v2 = iVertices[iFaces[i][2]] - iVertices[iFaces[i][1]];
    Eigen::Vector3f normal = v1.cross(v2);
    normal.normalize();
    Eigen::Vector4f plane;
    plane.head<3>() = normal;
    plane[3] = -normal.dot(iVertices[iFaces[i][0]]);
    planes.push_back(plane);
  }
  return planes;
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
