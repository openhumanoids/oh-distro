#include "visualization.hpp"

#include <iostream>

#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>

#include <fovis/fovis.hpp>

namespace fovis
{

static float ORANGE_TO_BLUE_RGB[12][3] = {
   {1.000,   0.167,   0.000},
   {1.000,   0.400,   0.100},
   {1.000,   0.600,   0.200},
   {1.000,   0.800,   0.400},
   {1.000,   0.933,   0.600},
   {1.000,   1.000,   0.800},
   {0.800,   1.000,   1.000},
   {0.600,   0.933,   1.000},
   {0.400,   0.800,   1.000},
   {0.200,   0.600,   1.000},
   {0.100,   0.400,   1.000},
   {0.000,   0.167,   1.000},
};

void
Visualization::colormap(float z, float rgb[3])
{
  // TODO a nonlinear mapping
  float t = std::max(std::min(z, _max_z), _min_z)/(_max_z-_min_z);

  int max_range = sizeof(ORANGE_TO_BLUE_RGB)/sizeof(ORANGE_TO_BLUE_RGB[0])-1;
  int row = static_cast<int>(floor(max_range*t));
  if (row >= max_range) {
    rgb[0] = ORANGE_TO_BLUE_RGB[row][0];
    rgb[1] = ORANGE_TO_BLUE_RGB[row][1];
    rgb[2] = ORANGE_TO_BLUE_RGB[row][2];
    return;
  }
  float w = max_range*t - row;
  rgb[0] = ORANGE_TO_BLUE_RGB[row][0]*w + ORANGE_TO_BLUE_RGB[row+1][0]*(1.-w);
  rgb[1] = ORANGE_TO_BLUE_RGB[row][1]*w + ORANGE_TO_BLUE_RGB[row+1][1]*(1.-w);
  rgb[2] = ORANGE_TO_BLUE_RGB[row][2]*w + ORANGE_TO_BLUE_RGB[row+1][2]*(1.-w);
}

Visualization::Visualization(bot_lcmgl_t* lcmgl, const StereoCalibration* calib)
  : _lcmgl(lcmgl),
    _calibration(calib)
{
  // take the  Z corresponding to disparity 5 px as 'max Z'
  Eigen::Matrix4d uvdtoxyz = calib->getUvdToXyz();
  Eigen::Vector4d xyzw = uvdtoxyz * Eigen::Vector4d(1, 1, 5, 1);
  xyzw /= xyzw.w();
  _max_z = xyzw.z();

  // take the  Z corresponding to 3/4 disparity img width px as 'min Z'
  xyzw = uvdtoxyz * Eigen::Vector4d(1, 1, (3*calib->getWidth())/4, 1);
  xyzw /= xyzw.w();
  _min_z = xyzw.z();
}

Visualization::~Visualization() {
  bot_lcmgl_destroy(_lcmgl);
}

void
Visualization::draw(const VisualOdometry* odom)
{
  const OdometryFrame* target_frame = odom->getTargetFrame();
  int num_levels = target_frame->getNumLevels();
  int base_width = target_frame->getLevel(0)->getWidth();
  int base_height = target_frame->getLevel(0)->getHeight();

  bot_lcmgl_push_matrix(_lcmgl);
  bot_lcmgl_rotated(_lcmgl, -90, 0, 0, 1);
  bot_lcmgl_scalef(_lcmgl, 10.0 / base_width, -10.0 / base_width, 1);

  int x_offset = 0;
  for (int i=0; i<num_levels; i++) {
    x_offset += (2*base_width >> i) + 10;
    bot_lcmgl_push_matrix(_lcmgl);
    bot_lcmgl_translated(_lcmgl, x_offset, 0, 0);
    draw_pyramid_level_matches(odom, i);
    bot_lcmgl_pop_matrix(_lcmgl);
  }

  x_offset = 0;
  for (int i=0; i<num_levels; i++) {
    x_offset += (2*base_width >> i) + 10;
    bot_lcmgl_push_matrix(_lcmgl);
    bot_lcmgl_translated(_lcmgl, x_offset, 2*base_height+10, 0);
    draw_pyramid_level_flow(odom, i);
    bot_lcmgl_pop_matrix(_lcmgl);
  }

  bot_lcmgl_pop_matrix(_lcmgl);

  bot_lcmgl_switch_buffer(_lcmgl);
}

void
Visualization::draw_pyramid_level_flow(const VisualOdometry* odom, int level_num)
{
  const OdometryFrame* ref_frame = odom->getReferenceFrame();
  const OdometryFrame* target_frame = odom->getTargetFrame();
  const PyramidLevel* ref_level = ref_frame->getLevel(level_num);
  const PyramidLevel* target_level = target_frame->getLevel(level_num);

  int width = ref_level->getWidth();
  int height = ref_level->getHeight();

  const MotionEstimator* estimator = odom->getMotionEstimator();
  const FeatureMatch* matches = estimator->getMatches();
  int num_matches = estimator->getNumMatches();

  // current image
  bot_lcmgl_color3f(_lcmgl, 1, 1, 1);
  const uint8_t* target_gray = target_level->getGrayscaleImage();
  int target_gray_stride = target_level->getGrayscaleImageStride();
  int gray_texid = bot_lcmgl_texture2d(_lcmgl, target_gray, width, height,
                                       target_gray_stride, BOT_LCMGL_LUMINANCE,
                                       BOT_LCMGL_UNSIGNED_BYTE,
                                       BOT_LCMGL_COMPRESS_NONE);
  bot_lcmgl_texture_draw_quad(_lcmgl, gray_texid,
      0     , 0      , 0   ,
      0     , height , 0   ,
      width , height , 0   ,
      width , 0      , 0);

  float rgb[3];

#if 0
  // draw target features
  bot_lcmgl_color3f(_lcmgl, 0, 1, 0);
  bot_lcmgl_point_size(_lcmgl, 3.0f);
  bot_lcmgl_begin(_lcmgl, GL_POINTS);
  for(int i=0, nfeatures=target_level->getNumKeypoints(); i<nfeatures; ++i) {
    const KeypointData& kpdata(*target_level->getKeypointData(i));
    colormap(kpdata.xyz.z(), rgb);
    bot_lcmgl_color3f(_lcmgl, rgb[0], rgb[1], rgb[2]);
    bot_lcmgl_vertex2f(_lcmgl, kpdata.kp.u, kpdata.kp.v);
  }
  bot_lcmgl_end(_lcmgl);
#endif

#if 0
  // draw 9x9 boxes around keypoints
  bot_lcmgl_line_width(_lcmgl, 1.0);
  bot_lcmgl_color3f(_lcmgl, .5, .5, 1);
  bot_lcmgl_begin(_lcmgl, GL_LINES);
  for(int i=0, num_kp=target_level->getNumKeypoints();
      i < num_kp;
      ++i) {
    const KeypointData& kpdata(*target_level->getKeypointData(i));
    colormap(kpdata.xyz.z(), rgb);
    bot_lcmgl_color3f(_lcmgl, rgb[0], rgb[1], rgb[2]);

    bot_lcmgl_vertex2f(_lcmgl, kpdata.kp.u-4, kp.v-4);
    bot_lcmgl_vertex2f(_lcmgl, kpdata.kp.u-4, kp.v+4);

    bot_lcmgl_vertex2f(_lcmgl, kpdata.kp.u-4, kp.v+4);
    bot_lcmgl_vertex2f(_lcmgl, kpdata.kp.u+4, kp.v+4);

    bot_lcmgl_vertex2f(_lcmgl, kpdata.kp.u+4, kp.v+4);
    bot_lcmgl_vertex2f(_lcmgl, kpdata.kp.u+4, kp.v-4);

    bot_lcmgl_vertex2f(_lcmgl, kpdata.kp.u+4, kp.v-4);
    bot_lcmgl_vertex2f(_lcmgl, kpdata.kp.u-4, kp.v-4);
  }
  bot_lcmgl_end(_lcmgl);
#endif

#if 1
  // draw inliers
  bot_lcmgl_point_size(_lcmgl, 4.0f);
  bot_lcmgl_begin(_lcmgl, GL_POINTS);
  for (int i=0; i<num_matches; i++) {
    const FeatureMatch& match = matches[i];
    if (!match.inlier ||
        match.target_keypoint->pyramid_level != level_num)
      continue;
    int cur_x = match.target_keypoint->kp.u;
    int cur_y = match.target_keypoint->kp.v;
    colormap(match.target_keypoint->xyz(2), rgb);
    bot_lcmgl_color3f(_lcmgl, rgb[0], rgb[1], rgb[2]);
    bot_lcmgl_vertex2f(_lcmgl, cur_x, cur_y);
  }
  bot_lcmgl_end(_lcmgl);
#endif

#if 1
  // draw ref-to-target 'flow'
  //bot_lcmgl_color3f(_lcmgl, 0, 1, 0);
  bot_lcmgl_line_width(_lcmgl, 2.0f);
  bot_lcmgl_begin(_lcmgl, GL_LINES);
  for (int i=0; i<num_matches; i++) {
    const FeatureMatch& match = matches[i];
    if (!match.inlier ||
        match.target_keypoint->pyramid_level != level_num)
      continue;
    int cur_x = match.target_keypoint->kp.u;
    int cur_y = match.target_keypoint->kp.v;
    int prev_x = match.ref_keypoint->kp.u;
    int prev_y = match.ref_keypoint->kp.v;
    colormap(match.target_keypoint->xyz(2), rgb);
    bot_lcmgl_color3f(_lcmgl, rgb[0], rgb[1], rgb[2]);
    bot_lcmgl_vertex2f(_lcmgl, cur_x, cur_y);
    bot_lcmgl_vertex2f(_lcmgl, prev_x, prev_y);
  }
  bot_lcmgl_end(_lcmgl);
#endif

  if (level_num == 0) {
    //draw the ESM homography estimate
    bot_lcmgl_line_width(_lcmgl, 2.0);
    bot_lcmgl_color3f(_lcmgl, 1, 1, 0);
    bot_lcmgl_begin(_lcmgl, GL_LINE_STRIP);
    const Eigen::Matrix3d & H = odom->getInitialHomography();
    Eigen::MatrixXd vertices(5, 3);
    vertices <<
        0     , 0      , 1,
        width , 0      , 1,
        width , height , 1,
        0     , height , 1,
        0     , 0      , 1;
    Eigen::MatrixXd warpedPoints = H*vertices.transpose();
    warpedPoints.row(0) = warpedPoints.row(0).array()/warpedPoints.row(2).array();
    warpedPoints.row(1) = warpedPoints.row(1).array()/warpedPoints.row(2).array();
    for (int i=0;i<warpedPoints.cols();i++) {
      bot_lcmgl_vertex2f(_lcmgl, warpedPoints(0, i) ,warpedPoints(1, i));
    }
    bot_lcmgl_end(_lcmgl);
  }

}

void
Visualization::draw_pyramid_level_matches(const VisualOdometry* odom, int level_num)
{
  const OdometryFrame* ref_frame = odom->getReferenceFrame();
  const OdometryFrame* target_frame = odom->getTargetFrame();
  const PyramidLevel* ref_level = ref_frame->getLevel(level_num);
  const PyramidLevel* target_level = target_frame->getLevel(level_num);

  int width = ref_level->getWidth();
  int height = ref_level->getHeight();

  const MotionEstimator* estimator = odom->getMotionEstimator();
  const FeatureMatch* matches = estimator->getMatches();
  int num_matches = estimator->getNumMatches();

  // previous image
  bot_lcmgl_color3f(_lcmgl, 1,1,1);
  const uint8_t* ref_gray = ref_level->getGrayscaleImage();
  int ref_gray_stride = ref_level->getGrayscaleImageStride();
  int prev_gray_texid = bot_lcmgl_texture2d(_lcmgl, ref_gray,
      width, height, ref_gray_stride,
      BOT_LCMGL_LUMINANCE, BOT_LCMGL_UNSIGNED_BYTE, BOT_LCMGL_COMPRESS_NONE);

  bot_lcmgl_push_matrix(_lcmgl);
  bot_lcmgl_translated(_lcmgl, 0, height + 10, 0);
  bot_lcmgl_texture_draw_quad(_lcmgl, prev_gray_texid,
      0, 0, 0,
      0, height, 0,
      width, height, 0,
      width, 0, 0);

  // draw features in reference frame
  bot_lcmgl_color3f(_lcmgl, 1, 0, 1);
  bot_lcmgl_point_size(_lcmgl, 1.5f);
  bot_lcmgl_begin(_lcmgl, GL_POINTS);
  for(int i=0, nfeatures=ref_level->getNumKeypoints(); i<nfeatures; i++) {
    const KeyPoint& kp = ref_level->getKeypoint(i);
    bot_lcmgl_vertex2f(_lcmgl, kp.u, kp.v);
  }
  bot_lcmgl_end(_lcmgl);
  bot_lcmgl_pop_matrix(_lcmgl);

  // current image
  bot_lcmgl_color3f(_lcmgl, 1,1,1);
  const uint8_t* target_gray = target_level->getGrayscaleImage();
  int target_gray_stride = target_level->getGrayscaleImageStride();
  int gray_texid = bot_lcmgl_texture2d(_lcmgl, target_gray,
      width, height, target_gray_stride,
      BOT_LCMGL_LUMINANCE, BOT_LCMGL_UNSIGNED_BYTE, BOT_LCMGL_COMPRESS_NONE);
  bot_lcmgl_texture_draw_quad(_lcmgl, gray_texid,
      0, 0, 0,
      0, height, 0,
      width, height, 0,
      width, 0, 0);

  // draw features
  bot_lcmgl_color3f(_lcmgl, 0, 1, 0);
  bot_lcmgl_point_size(_lcmgl, 3.0f);
  bot_lcmgl_begin(_lcmgl, GL_POINTS);
  for(int i=0, nfeatures=target_level->getNumKeypoints(); i<nfeatures; i++) {
    const KeyPoint& kp = target_level->getKeypoint(i);
    bot_lcmgl_vertex2f(_lcmgl, kp.u, kp.v);
  }
  bot_lcmgl_end(_lcmgl);

  // draw matches that are not in the maximal clique
  bot_lcmgl_color3f(_lcmgl, 0.3, 0, 0);
  bot_lcmgl_begin(_lcmgl, GL_LINES);
  for(int i=0; i<num_matches; i++) {
    const FeatureMatch& match = matches[i];
    if(match.inlier || match.in_maximal_clique || match.target_keypoint->pyramid_level != level_num)
        continue;
    int cur_x = match.target_keypoint->kp.u;
    int cur_y = match.target_keypoint->kp.v;
    int prev_x = match.ref_keypoint->kp.u;
    int prev_y = match.ref_keypoint->kp.v;
    bot_lcmgl_vertex2f(_lcmgl, cur_x, cur_y);
    bot_lcmgl_vertex2f(_lcmgl, prev_x, prev_y + height + 10);
  }
  bot_lcmgl_end(_lcmgl);

  // draw inliers
  bot_lcmgl_color3f(_lcmgl, 0, 0, 1);
  bot_lcmgl_line_width(_lcmgl, 2.0);
  bot_lcmgl_begin(_lcmgl, GL_LINES);
  for(int i=0; i<num_matches; i++) {
    const FeatureMatch& match = matches[i];
    if(!match.inlier || match.target_keypoint->pyramid_level != level_num)
        continue;
    int cur_x = match.target_keypoint->kp.u;
    int cur_y = match.target_keypoint->kp.v;
    int prev_x = match.ref_keypoint->kp.u;
    int prev_y = match.ref_keypoint->kp.v;
    bot_lcmgl_vertex2f(_lcmgl, cur_x, cur_y);
    bot_lcmgl_vertex2f(_lcmgl, prev_x, prev_y + height + 10);
  }
  bot_lcmgl_end(_lcmgl);

  // draw matches that are in the maximal clique but failed the projection test
  bot_lcmgl_line_width(_lcmgl, 1.0);
  for(int i=0; i<num_matches; i++) {
    const FeatureMatch& match = matches[i];
    if(match.in_maximal_clique && !match.inlier && match.target_keypoint->pyramid_level == level_num) {
      int cur_x = match.target_keypoint->kp.u;
      int cur_y = match.target_keypoint->kp.v;
      int prev_x = match.ref_keypoint->kp.u;
      int prev_y = match.ref_keypoint->kp.v;
      bot_lcmgl_color3f(_lcmgl, 1, 0, 0);
      bot_lcmgl_begin(_lcmgl, GL_LINES);
      bot_lcmgl_vertex2f(_lcmgl, cur_x, cur_y);
      bot_lcmgl_vertex2f(_lcmgl, prev_x, prev_y + height + 10);
      bot_lcmgl_end(_lcmgl);

      bot_lcmgl_color3f(_lcmgl, 1, 1, 1);
      double cur_xyz[] = { cur_x, cur_y + 10, 0 };
      char txt[500];
      snprintf(txt, 80, "%.3f", match.reprojection_error);
      bot_lcmgl_text(_lcmgl, cur_xyz, txt);
    }
  }

  if (level_num ==0){
    //draw the ESM homography estimate
    bot_lcmgl_line_width(_lcmgl, 2.0);
    bot_lcmgl_color3f(_lcmgl, 1, 1, 0);
    bot_lcmgl_begin(_lcmgl,GL_LINE_STRIP);
    const Eigen::Matrix3d & H = odom->getInitialHomography();
    Eigen::MatrixXd vertices(5, 3);
    vertices <<
        0     , 0      , 1  ,
        width , 0      , 1  ,
        width , height , 1  ,
        0     , height , 1  ,
        0     , 0      , 1;
    Eigen::MatrixXd warpedPoints = H*vertices.transpose();
    warpedPoints.row(0) = warpedPoints.row(0).array()/warpedPoints.row(2).array();
    warpedPoints.row(1) = warpedPoints.row(1).array()/warpedPoints.row(2).array();
    for (int i=0;i<warpedPoints.cols();i++){
      bot_lcmgl_vertex2f(_lcmgl, warpedPoints(0, i), warpedPoints(1, i));
    }
    bot_lcmgl_end(_lcmgl);
  }

}

}
