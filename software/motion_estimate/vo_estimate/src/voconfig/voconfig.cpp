/*
 * configuration.cpp
 *
 *  Created on: Dec 5, 2011
 *      Author: Maurice Fallon
 *      Author: Hordur Johannsson
 */

#include "voconfig.hpp"
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <Eigen/Dense>

#include <path_util/path_util.h>

namespace voconfig
{

BotParamConfiguration::BotParamConfiguration(lcm_t* bot_param_lcm,
  BotParam* bot_param,
  const std::string & key_prefix) : bot_param_lcm_(bot_param_lcm),
                                    bot_param_(bot_param),
                                    key_prefix_(key_prefix)
{
}

BotParamConfiguration::~BotParamConfiguration() {}

bool BotParamConfiguration::has_key(const std::string & key)
{
  return bot_param_has_key(bot_param_, (key_prefix_+"."+key).c_str());
}

bool BotParamConfiguration::get(const std::string & key, bool default_value)
{
  int bval;
  if (bot_param_get_boolean(bot_param_, (key_prefix_+"."+key).c_str(), &bval) != 0) bval = default_value;
  return bval;
}

int BotParamConfiguration::get(const std::string & key, int default_value)
{
  int ival;
  if (bot_param_get_int(bot_param_, (key_prefix_+"."+key).c_str(), &ival) != 0) ival = default_value;
  return ival;
}

double BotParamConfiguration::get(const std::string & key, double default_value)
{
  double dval;
  if (bot_param_get_double(bot_param_, (key_prefix_+"."+key).c_str(), &dval) != 0) dval = default_value;
  return dval;
}

std::string BotParamConfiguration::get(const std::string & key, const std::string & default_value)
{
  std::string val;
  char * sval = NULL;
  if (bot_param_get_str(bot_param_, (key_prefix_+"."+key).c_str(), &sval) == 0) val = std::string(sval);
  else val = default_value;
  if (sval) free(sval);
  return val;
}

KmclConfiguration::KmclConfiguration(BotParam* botparam,
                            const std::string & depth_source_name)
    : depth_source_type_(UNKNOWN)
{
  init(botparam, depth_source_name);
}

Configuration::Ptr KmclConfiguration::get_section(const std::string & key_prefix) const
{
  return BotParamConfiguration::Ptr(new BotParamConfiguration(bot_param_lcm_, bot_param_, key_prefix));
}

void KmclConfiguration::init(BotParam* bot_param,
                            const std::string & depth_source_name) {
  bot_param_lcm_ = lcm_create(NULL);

//  bot_param_ = bot_param_new_from_file(config_name.c_str());
  bot_param_ = bot_param;
  if (bot_param_ == NULL) {
    std::cerr << "Couldn't get bot param" << std::endl;
    exit(-1);
  }

  /*
  // proposal
  char * loop_proposal_str = bot_param_get_str_or_fail(bot_param_, "vision_slam.loop_proposal.loop_proposal");
  char * vocabulary_str = bot_param_get_str_or_fail(bot_param_, "vision_slam.loop_proposal.vocabulary");
  char * vocabulary_weights_str = bot_param_get_str_or_fail(bot_param_, "vision_slam.loop_proposal.vocabulary_weights");
  char * vocabulary_descriptor_str = bot_param_get_str_or_fail(bot_param_, "vision_slam.loop_proposal.vocabulary_descriptor");
  char * builder_descriptor_str = bot_param_get_str_or_fail(bot_param_, "vision_slam.vocabulary_builder.descriptor");
  loop_proposal_ = loop_proposal_str;
  //vocabulary_ = std::string(getConfigPath()) + "/" + std::string(vocabulary_str);
  //vocabulary_weights_ = std::string(getConfigPath()) + "/" + std::string(vocabulary_weights_str);
  vocabulary_descriptor_ = std::string(vocabulary_descriptor_str);
  builder_descriptor_ = std::string(builder_descriptor_str);
  free(loop_proposal_str);
  free(vocabulary_str);
  free(vocabulary_weights_str);
  free(vocabulary_descriptor_str);
  free(builder_descriptor_str);
  min_time_ = bot_param_get_int_or_fail(bot_param_, "vision_slam.loop_proposal.min_time");
  min_score_ = bot_param_get_double_or_fail(bot_param_, "vision_slam.loop_proposal.min_score");
  max_num_proposals_ = bot_param_get_int_or_fail(bot_param_, "vision_slam.loop_proposal.max_num_proposals");

  // closure
  char * alignment_descriptor_str = bot_param_get_str_or_fail(bot_param_, "vision_slam.loop_closure.alignment_descriptor");
  alignment_descriptor_ = std::string(alignment_descriptor_str);
  free(alignment_descriptor_str);
  min_inliers_ = bot_param_get_int_or_fail(bot_param_, "vision_slam.loop_closure.min_inliers");
  */
  
  
  // cameras
  key_prefix_ = "cameras." + depth_source_name;

  char * depth_source_type_str = bot_param_get_str_or_fail(bot_param_, (key_prefix_ + ".type").c_str());
  if (depth_source_type_str == std::string("stereo")) {
    depth_source_type_ = STEREO;
  } else if (depth_source_type_str == std::string("openni")) {
    depth_source_type_ = OPENNI;
  } else if (depth_source_type_str == std::string("primesense")) {
    depth_source_type_ = PRIMESENSE;
  } else {
    std::cerr << "Unknown depth source type: '" << depth_source_type_str
              << "'. Should be 'stereo' or 'primesense'\n";
    exit(-1);
  }
  free (depth_source_type_str);

  char * lcm_channel_str = bot_param_get_str_or_fail(bot_param_, (key_prefix_ + ".lcm_channel").c_str());
  lcm_channel_ = lcm_channel_str;
  free(lcm_channel_str);
}

KmclConfiguration::~KmclConfiguration()
{
  bot_param_destroy(bot_param_);
}

boost::shared_ptr<fovis::PrimeSenseCalibration>
KmclConfiguration::load_primesense_calibration() const {
  assert (depth_source_type_==PRIMESENSE || depth_source_type_==OPENNI);
  if (depth_source_type_ != PRIMESENSE && depth_source_type_ != OPENNI) {
    return boost::shared_ptr<fovis::PrimeSenseCalibration>();
  }
  std::string key_prefix_str;
  fovis::PrimeSenseCalibrationParameters kparams;
  for (int i=0; i < 2; ++i) {
    fovis::CameraIntrinsicsParameters* params;

    if (i == 0) {
      key_prefix_str = std::string(key_prefix_) + ".rgb";
      params = &(kparams.rgb_params);
    } else {
      key_prefix_str = std::string(key_prefix_) + ".depth";
      params = &(kparams.depth_params);
    }
    params->width = bot_param_get_int_or_fail(bot_param_, (key_prefix_str+".width").c_str());
    params->height = bot_param_get_int_or_fail(bot_param_,(key_prefix_str+".height").c_str());
    params->fx = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".fx").c_str());
    params->fy = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".fy").c_str());
    params->cx = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".cx").c_str());
    params->cy = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".cy").c_str());
  }

  kparams.width = kparams.rgb_params.width;
  kparams.height = kparams.rgb_params.width;
  kparams.shift_offset = bot_param_get_double_or_fail(
      bot_param_, (key_prefix_+".shift_offset").c_str());
  kparams.projector_depth_baseline = bot_param_get_double_or_fail(
      bot_param_, (key_prefix_+".projector_depth_baseline").c_str());

  Eigen::Matrix3d R;
  R << 0.999999, -0.000796, 0.001256, 0.000739, 0.998970, 0.045368, -0.001291, -0.045367, 0.998970;
  kparams.depth_to_rgb_translation[0] = -0.015756;
  kparams.depth_to_rgb_translation[1] = -0.000923;
  kparams.depth_to_rgb_translation[2] =  0.002316;
  Eigen::Quaterniond Q(R);
  kparams.depth_to_rgb_quaternion[0] = Q.w();
  kparams.depth_to_rgb_quaternion[1] = Q.x();
  kparams.depth_to_rgb_quaternion[2] = Q.y();
  kparams.depth_to_rgb_quaternion[3] = Q.z();

  // We assume rotation is a rotation matrix
  double rotation[9], translation[3];
  bot_param_get_double_array_or_fail(bot_param_,
                                     (key_prefix_str+".rotation").c_str(),
                                     &rotation[0],
                                     9);
  bot_param_get_double_array_or_fail(bot_param_,
                                     (key_prefix_str+".translation").c_str(),
                                     &translation[0],
                                     3);

  bot_matrix_to_quat(rotation, kparams.depth_to_rgb_quaternion);
  std::copy(translation, translation+3, kparams.depth_to_rgb_translation);

  return boost::shared_ptr<fovis::PrimeSenseCalibration>(new fovis::PrimeSenseCalibration(kparams));
}

boost::shared_ptr<fovis::StereoCalibration>
KmclConfiguration::load_stereo_calibration() const {
  assert (depth_source_type_==STEREO);

  if (depth_source_type_ != STEREO) {
    return boost::shared_ptr<fovis::StereoCalibration>();
  }
  fovis::StereoCalibrationParameters stereo_params;
  std::string key_prefix_str;
  fovis::CameraIntrinsicsParameters* params;

  for (int i=0; i < 2; ++i) {
    if (i == 0) {
      key_prefix_str = std::string(key_prefix_) + ".left";
      params = &(stereo_params.left_parameters);
    } else {
      key_prefix_str = std::string(key_prefix_) + ".right";
      params = &(stereo_params.right_parameters);
    }
    params->width = bot_param_get_int_or_fail(bot_param_, (key_prefix_str+".width").c_str());
    params->height = bot_param_get_int_or_fail(bot_param_,(key_prefix_str+".height").c_str());
    params->fx = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".fx").c_str());
    params->fy = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".fy").c_str());
    params->cx = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".cx").c_str());
    params->cy = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".cy").c_str());
    params->k1 = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".k1").c_str());
    params->k2 = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".k2").c_str());
    params->k3 = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".k3").c_str());
    params->p1 = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".p1").c_str());
    params->p2 = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".p2").c_str());
  }

  // We assume rotation is a rotation matrix
  double rotation[9], translation[3];
  bot_param_get_double_array_or_fail(bot_param_,
                                     (key_prefix_str+".rotation").c_str(),
                                     &rotation[0],
                                     9);
  bot_param_get_double_array_or_fail(bot_param_,
                                     (key_prefix_str+".translation").c_str(),
                                     &translation[0],
                                     3);

  bot_matrix_to_quat(rotation, stereo_params.right_to_left_rotation);
  std::copy(translation, translation+3, stereo_params.right_to_left_translation);

  return boost::shared_ptr<fovis::StereoCalibration>(new fovis::StereoCalibration(stereo_params));
}

void KmclConfiguration::set_vo_option_int(fovis::VisualOdometryOptions & vo_opts,
    const std::string & option) const
{
  int ival;
  std::string param = std::string("fovis.") + option;
  if (bot_param_get_int(bot_param_, param.c_str(), &ival) == 0)
    vo_opts[option] = boost::lexical_cast<std::string>(ival);
}

void KmclConfiguration::set_vo_option_double(fovis::VisualOdometryOptions & vo_opts,
    const std::string & option) const
{
  double dval;
  std::string param = std::string("fovis.") + option;
  if (bot_param_get_double(bot_param_, param.c_str(), &dval) == 0)
    vo_opts[option] = boost::lexical_cast<std::string>(dval);
}

void KmclConfiguration::set_vo_option_boolean(fovis::VisualOdometryOptions & vo_opts,
    const std::string & option) const
{
  int bval;
  std::string param = std::string("fovis.") + option;
  if (bot_param_get_boolean(bot_param_, param.c_str(), &bval) == 0)
    vo_opts[option] = boost::lexical_cast<std::string>((bval?"true":"false"));
}

fovis::VisualOdometryOptions KmclConfiguration::visual_odometry_options() const {
  fovis::VisualOdometryOptions vo_opts = fovis::VisualOdometry::getDefaultOptions();

  set_vo_option_int(vo_opts, "feature-window-size");
  set_vo_option_int(vo_opts, "max-pyramid-level");
  set_vo_option_int(vo_opts, "min-pyramid-level");
  set_vo_option_int(vo_opts, "target-pixels-per-feature");
  set_vo_option_int(vo_opts, "fast-threshold");
  set_vo_option_double(vo_opts, "fast-threshold-adaptive-gain");
  set_vo_option_boolean(vo_opts, "use-adaptive-threshold");
  set_vo_option_boolean(vo_opts, "use-homography-initialization");
  set_vo_option_int(vo_opts, "ref-frame-change-threshold");

  // OdometryFrame
  set_vo_option_boolean(vo_opts, "use-bucketing");
  set_vo_option_int(vo_opts, "bucket-width");
  set_vo_option_int(vo_opts, "bucket-height");
  set_vo_option_int(vo_opts, "max-keypoints-per-bucket");
  set_vo_option_boolean(vo_opts, "use-image-normalization");

  // MotionEstimator
  set_vo_option_double(vo_opts, "inlier-max-reprojection-error");
  set_vo_option_double(vo_opts, "clique-inlier-threshold");
  set_vo_option_int(vo_opts, "min-features-for-estimate");
  set_vo_option_double(vo_opts, "max-mean-reprojection-error");
  set_vo_option_boolean(vo_opts, "use-subpixel-refinement");
  set_vo_option_int(vo_opts, "feature-search-window");
  set_vo_option_boolean(vo_opts, "update-target-features-with-refined");

  // StereoDepth
  set_vo_option_boolean(vo_opts, "stereo-require-mutual-match");
  set_vo_option_double(vo_opts, "stereo-max-dist-epipolar-line");
  set_vo_option_double(vo_opts, "stereo-max-refinement-displacement");
  set_vo_option_int(vo_opts, "stereo-max-disparity");

  return vo_opts;
}

} // namespace vs
