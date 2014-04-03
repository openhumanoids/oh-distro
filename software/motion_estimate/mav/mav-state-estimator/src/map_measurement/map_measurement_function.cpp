#include "map_measurement_function.hpp"

namespace MavStateEst {

MapMeasurementFunction::MapMeasurementFunction()
{
  msg = NULL;

  phi_psi_xy_cov_map = NULL;
  phi_psi_xy_information_map = NULL;
  xy_max_information = NULL;
  xy_min_information = NULL;
  z_height = 0;
  theta = 0;
}

/**
 * visualize will publish pose and laser messages along with GPF lcmgl visualization as measurments are computed, but it will be much slower.
 */
void MapMeasurementFunction::generateFromOctomap(const char * base_map_name, double mPP, double phi_max, double radPP,
    lcm_t * lcm, BotParam * param, BotFrames * frames, double sigma_prior, double z_height, double vis_pause,
    double max_laser_range)
{
  bool visualize = vis_pause > 0;

  double laser_sim_minNegLogLike;
//    octomap::OcTree * sim_map = octomap::loadOctomap(base_map_name, &laser_sim_minNegLogLike);
  octomap::OcTree * sim_map = new octomap::OcTree(base_map_name);
  double minX, minY, minZ, maxX, maxY, maxZ;
  sim_map->getMetricMin(minX, minY, minZ);
  sim_map->getMetricMax(maxX, maxY, maxZ);
  printf("\nmap bounds: [%.2f, %.2f, %.2f] - [%.2f, %.2f, %.2f]  res: %f\n", minX, minY, minZ, maxX, maxY, maxZ,
      sim_map->getResolution());

  this->z_height = z_height;
  LaserGPF laser_gpf = LaserGPF(lcm, param, frames);
//  laser_gpf.motion_project = false;
  laser_gpf.motion_mode = LaserGPF::motion_none;

  //set the number of ranges for the simulator to the number after decimation then reset decimation
  //FIXME also resest laser_gpf.spatial_decimation ?
//    int nranges = NUM_HOKUYO_RANGES / ((double) laser_gpf.beam_skip); //put up with integer roundoff since it's big
//    laser_gpf.beam_skip = 1;

  //FIXME this right abe?
//    int numHeightBeamsStart = laser_gpf.laser_projector->heightDownRegion[1];
//    int numHeightBeamsEnd = laser_gpf.laser_projector->heightUpRegion[1] - laser_gpf.laser_projector->heightUpRegion[0];

  int numHeightBeamsStart = 20;
  int numHeightBeamsEnd = 0;

  laser_util::LaserSim3D laser_sim = laser_util::LaserSim3D(sim_map, param, frames, "sim_laser");

  double xy0[2] = { minX, minY };
  double xy1[2] = { maxX, maxY };

  double phi_psi_0[2];
  double phi_psi_1[2];

  phi_psi_0[0] = -phi_max;
  phi_psi_1[0] = phi_max + radPP;

  phi_psi_0[1] = -M_PI + radPP / 2.0;
  phi_psi_1[1] = M_PI + radPP / 2.0;

  xy_max_information = new occ_map::FloatPixelMap(xy0, xy1, mPP, 0);
  xy_min_information = new occ_map::FloatPixelMap(xy0, xy1, mPP, INFINITY);

  int x_dim = xy_max_information->dimensions[0];
  int y_dim = xy_max_information->dimensions[1];

  phi_psi_xy_cov_map = new CovPixelPixelMap(phi_psi_0, phi_psi_1, radPP, NULL);
  phi_psi_xy_information_map = new FloatPixelPixelMap(phi_psi_0, phi_psi_1, radPP, 0);

  int phi_dim = phi_psi_xy_cov_map->dimensions[0];
  int psi_dim = phi_psi_xy_cov_map->dimensions[1];

  int num_cells = x_dim * y_dim * phi_dim * psi_dim;

  printf("Creating measurment map, x_dim = %d, y_dim = %d, phi_dim = %d, psi_dim = %d, %d total cells to evaluate\n",
      x_dim, y_dim, phi_dim, psi_dim, num_cells);

  printf("phi: min=%f, max=%f\npsi: min=%f, max=%f\n",
      bot_to_degrees(phi_psi_xy_cov_map->xy0[0]), bot_to_degrees(phi_psi_xy_cov_map->xy1[0]),
      bot_to_degrees(phi_psi_xy_cov_map->xy0[1]), bot_to_degrees(phi_psi_xy_cov_map->xy1[1]));

  double x, y, phi, psi;

  int phipsi_spatial_inds[2], xy_spatial_inds[2];
  int phipsi_ind, xy_ind;

  double phipsi_loc[2];
  double xy_loc[2];

  RBIS state = RBIS();
  BotTrans bot_trans;

  Eigen::VectorXd z_effective;
  Eigen::MatrixXd R_effective;

  RBIM prior_cov = RBIM::Zero();
  prior_cov.block<3, 3>(RBIS::position_ind, RBIS::position_ind) = sigma_prior * Eigen::Matrix3d::Identity();

  int num_completed = 0;
  Eigen::Vector3d eulers;

  for (int i = 0; i < phi_dim; i++) {
    for (int j = 0; j < psi_dim; j++) {
      phipsi_spatial_inds[0] = i;
      phipsi_spatial_inds[1] = j;
      phipsi_ind = phi_psi_xy_cov_map->getInd(phipsi_spatial_inds);
      phi_psi_xy_cov_map->indToLoc(phipsi_ind, phipsi_loc);

      CovPixelMap * xy_cov_map = new CovPixelMap(xy0, xy1, mPP);
      occ_map::FloatPixelMap * xy_information_map = new occ_map::FloatPixelMap(xy0, xy1, mPP, 0);
      phi_psi_xy_cov_map->writeValue(phipsi_spatial_inds, xy_cov_map);
      phi_psi_xy_information_map->writeValue(phipsi_spatial_inds, xy_information_map);

      for (int k = 0; k < x_dim; k++) {
        for (int l = 0; l < y_dim; l++) {
          xy_spatial_inds[0] = k;
          xy_spatial_inds[1] = l;
          xy_ind = xy_cov_map->getInd(xy_spatial_inds);
          xy_cov_map->indToLoc(xy_ind, xy_loc);

          //add .5 pixel res to all of them except roll
          state.position()(0) = xy_loc[0];
          state.position()(1) = xy_loc[1];
          state.position()(2) = z_height;
          eulers << phipsi_loc[0], 0, phipsi_loc[1];
          state.setQuatEulerAngles(eulers);

          state.getBotTrans(&bot_trans);
          const bot_core_planar_lidar_t * laser_msg = laser_sim.simulate(&bot_trans);

          bot_core::planar_lidar_t * laser_msg_cpp = new bot_core::planar_lidar_t;
          laser_msg_cpp->intensities = std::vector<float>(laser_msg->nintensities);
          laser_msg_cpp->nintensities = laser_msg->nintensities;
          memcpy(&laser_msg_cpp->intensities[0], &laser_msg->intensities[0], laser_msg_cpp->nintensities * sizeof(float));

          laser_msg_cpp->ranges = std::vector<float>(laser_msg->nranges);
          laser_msg_cpp->nranges = laser_msg->nranges;
          memcpy(&laser_msg_cpp->ranges[0], &laser_msg->ranges[0], laser_msg_cpp->nranges * sizeof(float));

          laser_msg_cpp->rad0 = laser_msg->rad0;
          laser_msg_cpp->radstep = laser_msg->radstep;
          laser_msg_cpp->utime = laser_msg->utime;

          laser_gpf.getMeasurement(state, prior_cov, laser_msg_cpp, z_effective, R_effective);

          delete laser_msg_cpp;

          double information_gain = R_effective.inverse().trace();

          xy_cov_map->writeValue(xy_spatial_inds, R_effective);
          xy_information_map->writeValue(xy_spatial_inds, information_gain);

          if (information_gain > xy_max_information->readValue(xy_spatial_inds)) {
            xy_max_information->writeValue(xy_spatial_inds, information_gain);
          }

          if (information_gain < xy_min_information->readValue(xy_spatial_inds)) {
            xy_min_information->writeValue(xy_spatial_inds, information_gain);
          }

          if (visualize) {
            rigid_body_pose_t pose;
            state.getPose(&pose);
            rigid_body_pose_t_publish(lcm, "STATE_ESTIMATOR_POSE", &pose);
            bot_core_planar_lidar_t_publish(lcm, "LASER", laser_msg);

            mav_indexed_measurement_t * gpf_msg = gpfCreateLCMmsg(laser_gpf.laser_gpf_measurement_indices,
                z_effective, R_effective);
            gpf_msg->utime = 0;
            gpf_msg->state_utime = 0;
            mav_indexed_measurement_t_publish(lcm, "GPF_MEASUREMENT", gpf_msg);
            mav_indexed_measurement_t_destroy(gpf_msg);

            usleep(vis_pause * 1e6);
          }

          num_completed++;
          fprintf(stdout, "\r evaluated %d/%d", num_completed, num_cells);
        }

      }
    }
  }

  delete sim_map;
}

void MapMeasurementFunction::publishInformationMaps(lcm_t * lcm)
{
  occ_map_pixel_map_t_publish(lcm, "GPF_MIN_INFO_CHANNEL", xy_min_information->get_pixel_map_t(0));
  occ_map_pixel_map_t_publish(lcm, "GPF_MAX_INFO_CHANNEL", xy_max_information->get_pixel_map_t(0));
  double zz[2] = { 0, 0 };
  occ_map_pixel_map_t_publish(lcm, "GPF_00_INFO_CHANNEL",
      phi_psi_xy_information_map->readValue(zz)->get_pixel_map_t(0));
}

void MapMeasurementFunction::publishMapMeasurementFunction(lcm_t * lcm)
{
  mav_map_measurement_function_t_publish(lcm, "FIXIE_MAP_MEAS", get_map_measurement_function_t(0));
}

void MapMeasurementFunction::saveToFile(const char * name)
{
  const mav_map_measurement_function_t * msg = get_map_measurement_function_t(0);
  int sz = mav_map_measurement_function_t_encoded_size(msg);
  char * buf = (char *) malloc(sz * sizeof(char));
  mav_map_measurement_function_t_encode(buf, 0, sz, msg);
  std::ofstream ofs(name, std::ios::binary);
  ofs << sz;
  ofs.write(buf, sz);
  ofs.close();
  free(buf);
}

mav_map_measurement_function_t * MapMeasurementFunction::load_map_measurement_function_t_from_file(
    const char * name)
{
  std::ifstream ifs(name, std::ios::binary);
  int sz;
  ifs >> sz;
  char * tmpdata = (char *) malloc(sz * sizeof(char));
  ifs.read(tmpdata, sz * sizeof(char));
  ifs.close();
  mav_map_measurement_function_t * ret_msg = (mav_map_measurement_function_t *) calloc(1,
      sizeof(mav_map_measurement_function_t));
  mav_map_measurement_function_t_decode(tmpdata, 0, sz, ret_msg);
  free(tmpdata);
  return ret_msg;
}

void MapMeasurementFunction::loadFromFile(const char * name)
{
  mav_map_measurement_function_t * tmpmsg = load_map_measurement_function_t_from_file(name);
  set_from_map_measurement_function_t(tmpmsg);
  mav_map_measurement_function_t_destroy(tmpmsg);
}

const mav_map_measurement_function_t * MapMeasurementFunction::get_map_measurement_function_t(int64_t utime_)
{
  if (this->msg == NULL
  )
    msg = (mav_map_measurement_function_t *) calloc(1, sizeof(mav_map_measurement_function_t));

  msg->utime = utime_;
  msg->theta = this->theta;
  msg->z_height = this->z_height;

  msg->num_xy_maps = this->phi_psi_xy_information_map->num_cells;

  msg->xy_max_information = *((occ_map_pixel_map_t *) this->xy_max_information->get_pixel_map_t(utime_));
  msg->xy_min_information = *((occ_map_pixel_map_t *) this->xy_min_information->get_pixel_map_t(utime_));
  msg->phi_psi_xy_cov_map = *((occ_map_pixel_map_t *) this->phi_psi_xy_cov_map->get_pixel_map_t(utime_));
  msg->phi_psi_xy_information_map = *((occ_map_pixel_map_t *) this->phi_psi_xy_information_map->get_pixel_map_t(
      utime_));

  msg->xy_cov_maps = (occ_map_pixel_map_t *) calloc(msg->num_xy_maps, sizeof(occ_map_pixel_map_t));
  msg->xy_information_maps = (occ_map_pixel_map_t *) calloc(msg->num_xy_maps, sizeof(occ_map_pixel_map_t));

  for (int ii = 0; ii < msg->num_xy_maps; ii++) {
    //FIXME this encoding is maybe a bit sketcy with Eigen's alignment storage stuff
    msg->xy_cov_maps[ii] = *((occ_map_pixel_map_t *) this->phi_psi_xy_cov_map->data[ii]->get_pixel_map_t(utime_));
    msg->xy_information_maps[ii] =
        *((occ_map_pixel_map_t *) this->phi_psi_xy_information_map->data[ii]->get_pixel_map_t(utime_));
    fprintf(stdout, "\r encoded xy pixel maps %d/%d", ii, msg->num_xy_maps);
  }
  return msg;
}

void MapMeasurementFunction::set_from_map_measurement_function_t(const mav_map_measurement_function_t * msg)
{
  if (this->phi_psi_xy_cov_map != NULL
  )
    delete phi_psi_xy_cov_map, phi_psi_xy_information_map, xy_max_information, xy_min_information;

  this->theta = msg->theta;
  this->z_height = msg->z_height;

  this->xy_max_information = new occ_map::FloatPixelMap((occ_map_pixel_map_t *) &msg->xy_max_information);
  this->xy_min_information = new occ_map::FloatPixelMap((occ_map_pixel_map_t *) &msg->xy_min_information);

  this->phi_psi_xy_cov_map = new CovPixelPixelMap((occ_map_pixel_map_t *) &msg->phi_psi_xy_cov_map);
  this->phi_psi_xy_information_map = new FloatPixelPixelMap((occ_map_pixel_map_t *) &msg->phi_psi_xy_information_map);

  for (int ii = 0; ii < msg->num_xy_maps; ii++) {
    this->phi_psi_xy_cov_map->data[ii] = new CovPixelMap((occ_map_pixel_map_t *) &msg->xy_cov_maps[ii]);
    this->phi_psi_xy_information_map->data[ii] = new occ_map::FloatPixelMap(
        (occ_map_pixel_map_t *) &msg->xy_information_maps[ii]);
    fprintf(stdout, "\r encoded xy pixel maps %d/%d", ii, msg->num_xy_maps);
  }
}

/**
 * get the measurement covariance at state
 */
void MapMeasurementFunction::getMeasurementCov(const RBIS & state, Eigen::Matrix3d & R_effective)
{
  double xy[2] = { state.position()(0), state.position()(1) };

  Eigen::Vector3d eulers = state.getEulerAngles();
  double phipsi[2] = { eulers(0), eulers(2) };

  R_effective = phi_psi_xy_cov_map->readValue(phipsi)->readValue(xy);

  double theta_state = eulers(1);
  double z_state = state.position()(2);

  if (fabs(z_state - z_height) > 2.0) {
    fprintf(stderr, "warning, querying measurement at different height than measurements were simulated, %f, %f\n",
        z_state, z_height);
  }

  if (fabs(theta_state - theta) > bot_to_radians(20)) {
    fprintf(stderr,
        "warning, querying measurement at different theta (pitch) than measurements were simulated, %f, %f\n",
        bot_to_degrees(theta_state), bot_to_degrees(theta));
  }
}

}
