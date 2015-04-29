#include "FillMethods.hpp"

#include <mini-maps/DepthImageView.hpp>
#include <mini-maps/DepthImage.hpp>

using namespace terrainmap;

bool FillMethods::
fillEntireView(View& ioView, const Eigen::Vector4d& iPlane) {

  // get depth image array
  auto type = maps::DepthImage::TypeDepth;
  auto img = ioView->getDepthImage();
  auto& depths = const_cast<std::vector<float>&>(img->getData(type));

  // transform plane to depth image coords
  const auto planeTransform =
    ioView->getTransform().cast<double>().inverse().matrix().transpose();
  auto plane = planeTransform*iPlane;
  Eigen::Vector3d abd(plane[0], plane[1], plane[3]);
  abd /= -plane[2];

  // fill in according to plane
  const int w = img->getWidth();
  const int h = img->getHeight();
  for (int i = 0; i < h; ++i) {
    for (int j = 0; j < w; ++j) {
      int idx = i*w + j;
      depths[idx] = abd[0]*j + abd[1]*i + abd[2];
    }
  }

  // put depth values back into view
  img->setData(depths, type);

  return true;
}

bool FillMethods::
fillBox(View& ioView, const Eigen::Vector3d& iPosition,
        const double iRadius, const Eigen::Vector4d& iPlane) {

  // get depth image array
  auto type = maps::DepthImage::TypeDepth;
  auto img = ioView->getDepthImage();
  auto& depths = const_cast<std::vector<float>&>(img->getData(type));

  // project position and radius into depth image
  auto pos = iPosition.cast<float>();
  auto mapPos = img->project(pos, type);
  auto radiusPosition = pos + iRadius*Eigen::Vector3f(1,0,0);
  auto mapRadiusPos = img->project(radiusPosition, type);
  float radiusPixels = (mapRadiusPos - mapPos).norm();
  
  // transform plane to depth image coords
  const auto planeTransform =
    ioView->getTransform().cast<double>().inverse().matrix().transpose();
  auto plane = planeTransform*iPlane;
  Eigen::Vector3d abd(plane[0], plane[1], plane[3]);
  abd /= -plane[2];

  // fill in invalid values
  const int cx(mapPos[0] + 0.5f), cy(mapPos[1] + 0.5f), r(ceil(radiusPixels));
  const int w(img->getWidth()), h(img->getHeight());
  const float invalidValue = img->getInvalidValue(type);
  for (int i = cy-r; i <= cy+r; ++i) {
    if ((i < 0) || (i >= h)) continue;
    for (int j = cx-r; j <= cx+r; ++j) {
      if ((j < 0) || (j >= w)) continue;
      int idx = i*w + j;
      float z = depths[idx];
      if (z != invalidValue) continue;
      depths[idx] = abd[0]*j + abd[1]*i + abd[2];
    }
  }

  // put depth values back into view
  img->setData(depths, type);

  return true;
}

bool FillMethods::
fillHolesIterative(View& ioView) {

  // get depth image array
  auto type = maps::DepthImage::TypeDepth;
  auto img = ioView->getDepthImage();
  auto& depths = const_cast<std::vector<float>&>(img->getData(type));

  // iterate
  const float invalidValue = img->getInvalidValue(type);
  const int w(img->getWidth()), h(img->getHeight());
  int curPass = 0;
  while (true) {
    int updateCount = 0;

    for (int i = 1; i < h-1; ++i) {
      for (int j = 1; j < w-1; ++j) {
        int idx = i*w+j;
        float z = depths[idx];
        if (z != invalidValue) continue;

        // look at neighbors
        int goodNeighbors = 0;
        float total = 0;
        float val;
        val = depths[idx+1];
        if (val != invalidValue) { total += val; ++goodNeighbors; }
        val = depths[idx-1];
        if (val != invalidValue) { total += val; ++goodNeighbors; }
        val = depths[idx+w];
        if (val != invalidValue) { total += val; ++goodNeighbors; }
        val = depths[idx-w];
        if (val != invalidValue) { total += val; ++goodNeighbors; }

        // set to average if there are enough neighbors
        if (goodNeighbors < 3) continue;
        depths[idx] = total/goodNeighbors;
        ++updateCount;
      }
    }

    if (updateCount == 0) break;
    ++curPass;
  }

  // put depth values back into view
  img->setData(depths, type);

  return true;
}

bool FillMethods::
fillMissing(View& ioView, const Eigen::Vector4d& iPlane) {
  // get depth image array
  auto type = maps::DepthImage::TypeDepth;
  auto img = ioView->getDepthImage();
  auto& depths = const_cast<std::vector<float>&>(img->getData(type));

  // transform plane to depth image coords
  const auto planeTransform =
    ioView->getTransform().cast<double>().inverse().matrix().transpose();
  auto plane = planeTransform*iPlane;
  Eigen::Vector3d abd(plane[0], plane[1], plane[3]);
  abd /= -plane[2];

  // fill in according to plane
  const float invalidValue = img->getInvalidValue(type);
  const int w = img->getWidth();
  const int h = img->getHeight();
  for (int i = 0; i < h; ++i) {
    for (int j = 0; j < w; ++j) {
      int idx = i*w + j;
      if (depths[idx] != invalidValue) continue;
      depths[idx] = abd[0]*j + abd[1]*i + abd[2];
    }
  }

  // put depth values back into view
  img->setData(depths, type);

  return true;
}
