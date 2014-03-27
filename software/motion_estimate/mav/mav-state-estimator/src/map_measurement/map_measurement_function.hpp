#ifndef MAP_MEASUREMENT_FUNCTION_H_
#define MAP_MEASUREMENT_FUNCTION_H_
#include <mav_state_est/rbis.hpp>
#include <mav_state_est/gpf/laser_gpf_lib.hpp>
#include <occ_map/PixelMap.hpp>
#include <laser_utils/LaserSim3D.hpp>
#include <lcmtypes/mav_map_measurement_function_t.h>
#include <lcmtypes/occ_map_pixel_map_t.h>

namespace MavStateEst {

#define NUM_HOKUYO_RANGES 1080
#define HOKUYO_RAD0 bot_to_radians(-135.0)
#define HOKUYO_RADSTEP bot_to_radians(0.25)


class MapMeasurementFunction {
public:
  typedef occ_map::PixelMap<Eigen::Matrix3d> CovPixelMap;
  typedef occ_map::PixelMap<CovPixelMap *> CovPixelPixelMap;
  typedef occ_map::PixelMap<occ_map::FloatPixelMap*> FloatPixelPixelMap;

  CovPixelPixelMap * phi_psi_xy_cov_map; //pixel map in phi, psi of pixelmaps in x, y of covariances
  FloatPixelPixelMap * phi_psi_xy_information_map; //pixel map in phi, psi of pixelmaps in xy of information gain
  occ_map::FloatPixelMap * xy_max_information; //pixel map in xy of max information gain for all phi,psi
  occ_map::FloatPixelMap * xy_min_information; //pixel map in xy of min information gain for all phi,psi

  //spatial parameters not swept in pixelmap representations
  double z_height; //height at which measurements are simulated in map
  double theta; //pitch angle at which measurements are simulated

  mav_map_measurement_function_t * msg;

  MapMeasurementFunction();

  /**
   * visualize will publish pose and laser messages along with GPF lcmgl visualization as measurments are computed, but it will be much slower.
   */
  void generateFromOctomap(const char * base_map_name, double mPP, double phi_max, double radPP,
      lcm_t * lcm, BotParam * param, BotFrames * frames, double sigma_prior, double z_height, double vis_pause,
      double max_laser_range);

  void publishInformationMaps(lcm_t * lcm);

  void publishMapMeasurementFunction(lcm_t * lcm);

  void saveToFile(const char * name);

  static mav_map_measurement_function_t * load_map_measurement_function_t_from_file(const char * name);

  void loadFromFile(const char * name);

  const mav_map_measurement_function_t * get_map_measurement_function_t(int64_t utime_);

  void set_from_map_measurement_function_t(const mav_map_measurement_function_t * msg);

  /**
   * get the measurement covariance at state
   */
  void getMeasurementCov(const RBIS & state, Eigen::Matrix3d & R_effective);
};

}

#endif
