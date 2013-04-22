#include "PointMatcher.hpp"

#include "PatchUtils.hpp"

using namespace tracking;

PointMatcher::
PointMatcher() {
}

PointMatch PointMatcher::
refine(const Eigen::Vector2f& iCurPos,
       const std::vector<cv::Mat>& iCurPyramid,
       const Eigen::Affine2f& iRefTransform,
       const std::vector<cv::Mat>& iRefPyramid,
       const int iPatchRadiusX, const int iPatchRadiusY,
       const Eigen::Vector2f& iConstraintDir,
       const int iMaxIterations) const {

  PointMatch match;
  match.mScore = -1;
  match.mRefPos = iRefTransform.translation();
  match.mCurPos = iCurPos;
  bool shouldProject = iConstraintDir.norm() > 1e-4f;
  std::cout << "CONSTRAINT " << iConstraintDir.transpose() << std::endl;
  if (shouldProject) {
    std::cout << "WHY" << std::endl;
  } else {
    std::cout << "NOT" << std::endl;
  }

  // create patches
  cv::Mat refPatch(2*iPatchRadiusX+1, 2*iPatchRadiusY+1, CV_32FC1);
  cv::Mat curPatch(refPatch.rows, refPatch.cols, refPatch.type());
  cv::Mat gx(refPatch.rows, refPatch.cols, refPatch.type());
  cv::Mat gy(refPatch.rows, refPatch.cols, refPatch.type());
  cv::Mat gt(refPatch.rows, refPatch.cols, refPatch.type());

  // iterate over pyramid levels
  for (int level = iCurPyramid.size()-1; level >= 0; --level) {
    float scaleFactor = 1.0f / (1<<level);

    // form ref patch at this level
    Eigen::Affine2f xform = iRefTransform;
    xform.translation() *= scaleFactor;
    PatchUtils::interpolate(iRefPyramid[level], xform, refPatch);

    // compute gradients and hessian
    PatchUtils::computeGradients(refPatch, gx, gy);
    float hxx(0), hxy(0), hyy(0), detInv(0);
    if (shouldProject) {
      gx = gx*iConstraintDir[0] + gy*iConstraintDir[1];
      detInv = 1.0f / gx.dot(gx);
    }
    else {
      hxx = gx.dot(gx);
      hxy = gx.dot(gy);
      hyy = gy.dot(gy);
      float det = hxx*hyy - hxy*hxy;
      detInv = 1.0f/det;
    }

    // transform current position to current level coords
    match.mCurPos *= scaleFactor;

    // iterate
    int iter;
    for (iter = 0; iter < iMaxIterations; ++iter) {
      // form current patch
      PatchUtils::interpolate(iCurPyramid[level], match.mCurPos, curPatch);

      // error patch
      gt = refPatch - curPatch;

      /* TODO: debugging
      PatchUtils::save(refPatch, "/home/antone/refpatch.pfm");
      PatchUtils::save(curPatch, "/home/antone/curpatch.pfm");
      PatchUtils::save(gx, "/home/antone/gx.pfm");
      PatchUtils::save(gy, "/home/antone/gy.pfm");
      PatchUtils::save(gt, "/home/antone/gt.pfm");
      exit(-1);
      */

      // solve for position delta
      Eigen::Vector2f delta;
      float gxt = gx.dot(gt);
      if (shouldProject) {
        delta = iConstraintDir*gxt*detInv;
      }
      else {
        float gyt = gy.dot(gt);
        delta = Eigen::Vector2f(hyy*gxt - hxy*gyt, -hxy*gxt + hxx*gyt)*detInv;
      }
      std::cout << "DELTA " << delta.transpose() << std::endl;

      match.mCurPos += delta;
      if (delta.norm() < 0.01f) break;
    }
    std::cout << "ITERS " << iter << std::endl;

    // transform current position back to base level coords
    match.mCurPos /= scaleFactor;
    exit(-1);
  }

  // compute final score
  match.mScore = PatchUtils::normalizedCrossCorrelation(refPatch, curPatch);

  return match;
}
