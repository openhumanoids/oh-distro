/*
 * configuration.hpp
 *
 *  Created on: Jun 5, 2011
 *      Author: Maurice Fallon
 *      Author: Hordur Johannsson
 */

#ifndef KMCL_CONFIGURATION_HPP_
#define KMCL_CONFIGURATION_HPP_

#include <vector>
#include <string>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <lcm/lcm.h>
#include <boost/shared_ptr.hpp>
#include <fovis/fovis.hpp>





namespace voconfig
{
/**
 * An interface for configuration objects.
 */
class Configuration {
public:
  virtual ~Configuration() {}

  virtual bool has_key(const std::string & key) = 0;
  virtual bool get(const std::string & key, bool default_value) = 0;
  virtual int get(const std::string & key, int default_value) = 0;
  virtual double get(const std::string & key, double default_value) = 0;
  virtual std::string get(const std::string & key, const std::string & default_value) = 0;

  typedef boost::shared_ptr<Configuration> Ptr;
  typedef boost::shared_ptr<const Configuration> ConstPtr;
};

/**
 * Implements the configuration interface for a BotParam provider.
 */
class BotParamConfiguration : public Configuration
{
public:
  BotParamConfiguration(lcm_t* bot_param_lcm, BotParam* bot_param, const std::string & key_prefix);
  virtual ~BotParamConfiguration();

  virtual bool has_key(const std::string & key);
  virtual bool get(const std::string & key, bool default_value);
  virtual int get(const std::string & key, int default_value);
  virtual double get(const std::string & key, double default_value);
  virtual std::string get(const std::string & key, const std::string & default_value);

private:
  lcm_t* bot_param_lcm_;
  BotParam* bot_param_;
  std::string key_prefix_;
};

/**
 * A configuration class for KMCL
 */
class KmclConfiguration {
public:
  enum DepthSourceTypeCode {
    UNKNOWN,
    STEREO,
    PRIMESENSE,
    OPENNI
  };

  KmclConfiguration(BotParam* botparam,
      const std::string & depth_source_name);

  ~KmclConfiguration();

  /**
   * Return a configuration object pointing to a specific section.
   */
  Configuration::Ptr get_section(const std::string & key_prefix) const;

  /**
   * Creates a StereoCalibration block using the provided configuration source and camera.
   *
   * @param config A configuration source to load the configuration from.
   * @param key_prefix names the configuration block that contains
   *                   the calibration information.
   *
   * @return If successful return a shared pointer to the StereoCalibration object
   *         and nullptr otherwise.
   */
  boost::shared_ptr<fovis::StereoCalibration> load_stereo_calibration() const;

  /**
   * Creates a PrimeSenseCalibration block using the provided configuration source and camera.
   *
   * @param config A configuration source to load the configuration from.
   * @param key_prefix names the configuration block that contains
   *                   the calibration information.
   *
   * @return If successful return a shared pointer to the PrimeSenseCalibration object
   *         and nullptr otherwise.
   */
  boost::shared_ptr<fovis::PrimeSenseCalibration> load_primesense_calibration() const;

  /**
   * @return Parameters for fovis::VisualOdometry
   */
  fovis::VisualOdometryOptions visual_odometry_options() const;

  /**
   * @return Name of LCM channel where the depth source is published.
   */
  std::string depth_source_channel() const { return lcm_channel_; }

  DepthSourceTypeCode depth_source_type() const { return depth_source_type_; }

  Configuration::Ptr get_camera_section() const
    { return get_section(key_prefix_); }

  std::string loop_proposal() const { return loop_proposal_; }
  std::string vocabulary() const { return vocabulary_; }
  std::string vocabulary_weights() const { return vocabulary_weights_; }
  std::string vocabulary_descriptor() const { return vocabulary_descriptor_; }
  std::string alignment_descriptor() const { return alignment_descriptor_; }
  std::string builder_descriptor() const { return builder_descriptor_; }

  int min_time() { return min_time_; }
  double min_score() { return min_score_; }
  int min_inliers() { return min_inliers_; }
  int max_num_proposals() { return max_num_proposals_; }

private:
  void init(BotParam* bot_param,
            const std::string & depth_source_name);

  void set_vo_option_int(fovis::VisualOdometryOptions & vo_opts,
                         const std::string & option) const;
  void set_vo_option_double(fovis::VisualOdometryOptions & vo_opts,
                            const std::string & option) const;
  void set_vo_option_boolean(fovis::VisualOdometryOptions & vo_opts,
                             const std::string & option) const;
			     

  lcm_t* bot_param_lcm_;
  BotParam* bot_param_;
  std::string key_prefix_;
  std::string lcm_channel_;

  std::string loop_proposal_;
  std::string vocabulary_;
  std::string vocabulary_weights_;

  std::string vocabulary_descriptor_;
  std::string alignment_descriptor_;
  std::string builder_descriptor_;

  int min_time_;
  double min_score_;
  int min_inliers_;
  int max_num_proposals_;

  DepthSourceTypeCode depth_source_type_;
};

}

#endif /* CONFIGURATION_HPP_ */
