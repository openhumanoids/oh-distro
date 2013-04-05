#ifndef _tracking_PyramidLevel_hpp_
#define _tracking_PyramidLevel_hpp_

#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>

struct PyramidLevel {
  typedef std::shared_ptr<PyramidLevel> Ptr;

  cv::Mat mImage;
  std::vector<cv::KeyPoint> mKeyPoints;
  
};

#endif
