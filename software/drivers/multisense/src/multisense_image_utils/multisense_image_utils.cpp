#include "multisense_image_utils.hpp"

bool multisense_image_utils::
removeSpeckles(cv::Mat& ioImage, const double iMaxDisparityDiff,
               const int iMaxBlobSize) {
  cv::Mat img(ioImage.rows, ioImage.cols, CV_16SC1, ioImage.data);
  cv::filterSpeckles(img, 0, iMaxBlobSize, iMaxDisparityDiff);
  return true;
}

bool multisense_image_utils::
removeSmall(cv::Mat& ioImage, const uint16_t iValueThresh,
            const int iSizeThresh) {
  const int w = ioImage.cols;
  const int h = ioImage.rows;

  // allocate output labels image
  std::vector<int> labelImage(w*h);
  std::fill(labelImage.begin(), labelImage.end(), 0);

  // allocate equivalences table
  std::vector<int> equivalences(w*h);
  std::fill(equivalences.begin(), equivalences.end(), 0);

  // utility function
  auto collapse = [&](const int iIndex) {
    int out = iIndex;
    while (equivalences[out] != out) out = equivalences[out];
    return out;
  };

  int curLabel = 0;

  // loop over image pixels
  for (int i = 0; i < h; ++i) {
    int* labels = labelImage.data() + i*w;
    const uint16_t* im = ioImage.ptr<uint16_t>(i);
    for (int j = 0; j < w; ++j, ++im, ++labels) {

      // ignore pixels below thresh
      if (*im <= iValueThresh) continue;

      // look at neighbors (for 4-connectedness)
      int labelAbove = (i>0) ? collapse(labels[-w]) : 0;
      int labelLeft = (j>0) ? collapse(labels[-1]) : 0;

      if (labelAbove>0) {
        *labels = labelAbove;
        if ((labelLeft>0) && (labelAbove!=labelLeft)) {
          equivalences[labelLeft] = labelAbove;
        }
      }
      else {
        if (labelLeft>0) {
          *labels = labelLeft;
        }
        else {
          ++curLabel;
          *labels = curLabel;
          equivalences[curLabel] = curLabel;
        }
      }
    }
  }

  // collapse label equivalences
  ++curLabel;
  for (int i = 0; i < curLabel; ++i) {
    equivalences[i] = collapse(i);
  }

  // remap labels image and count connected components
  std::vector<int> counts(curLabel);
  std::fill(counts.begin(), counts.end(), 0);
  for (int i = 0; i < w*h; ++i) {
    int& label = labelImage[i];
    label = equivalences[label];
    ++counts[label];
  }

  // remap once more
  std::vector<uint8_t> valid(counts.size());
  for (int i = 0; i < curLabel; ++i) {
    valid[i] = (counts[i] < iSizeThresh) ? 0 : 1;
  }
  const int* labels = labelImage.data();
  for (int i = 0; i < h; ++i) {
    uint16_t* im = ioImage.ptr<uint16_t>(i);
    for (int j = 0; j < w; ++j, ++im, ++labels) {
      if (valid[*labels] == 0) *im = 0;
    }
  }

  return true;
}
