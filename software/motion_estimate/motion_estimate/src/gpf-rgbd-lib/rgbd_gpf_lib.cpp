#include "rgbd_gpf_lib.hpp"
#include <path_util/path_util.h>

namespace MavStateEst {

const std::string RgbdGPF::rgbd_gpf_substate_stings[num_substates] = {
    "pos_only",
    "pos_yaw",
    "pos_chi",
    "all_states",
    "z_only",
    "xy_only" };

#include <lcmtypes/octomap_utils.h>

#define MIN_VALID_BEAMS 5

using namespace std;
using namespace Eigen;
using namespace eigen_utils;

uint8_t* rgbd_data_buffer= new uint8_t [3* 640*480]; // 2 color scale images stacked


void RgbdGPF::RgbdGPFBaseConstructor(int num_samples, bool gpf_vis, RgbdLikelihoodInterface * likelihood_interface,
    rgbd_gpf_substate gpf_orientation_mode, lcm_t * lcm, BotParam * param, BotFrames * frames)
{

  this->lcm = lcm;
  this->frames = frames;
  this->param = param;

  this->verbose = false;
  this->motion_project = true;

  this->num_samples = num_samples;
  this->laser_like_iface = likelihood_interface;
  this->gpf_substate_mode = gpf_orientation_mode;

  this->beam_skip = bot_param_get_int_or_fail(param, "state_estimator.rgbd_gpf.beam_skip"); //FIXME important hard coded parameter
  this->spatial_decimation_min = bot_param_get_double_or_fail(param, "state_estimator.rgbd_gpf.spatial_decimation_min");
  this->spatial_decimation_max = bot_param_get_double_or_fail(param, "state_estimator.rgbd_gpf.spatial_decimation_max");

  //  //--- publish the map (blurred, not ideal)
  //  octomap_raw_t msg;
  //  msg.utime = bot_timestamp_now();
  //
  //  std::stringstream datastream;
  //  ocTree->writeBinaryConst(datastream);
  //  std::string datastring = datastream.str();
  //  msg.data = (uint8_t *) datastring.c_str();
  //  msg.length = datastring.size();
  //
  //  octomap_raw_t_publish(lcm, "OCTOMAP", &msg);
  //  //----------------------------------------------------

  this->laser_projector = laser_projector_new(this->param, this->frames, "rgbd", 1); //TODO: laser name should be a param

  BotTrans body_to_rgbd;
  bot_frames_get_trans(this->frames, "body", this->laser_projector->coord_frame, &body_to_rgbd);
  Quaterniond body_to_rgbd_quat;
  botDoubleToQuaternion(body_to_rgbd_quat, body_to_rgbd.rot_quat);
  this->R_body_to_rgbd = body_to_rgbd_quat.toRotationMatrix();

  int delta_start_ind = RBIS::position_ind;
  int chi_start_ind = RBIS::chi_ind;

  fprintf(stderr, "LaserGpf is using the " "%s" " substate mode\n",
      rgbd_gpf_substate_stings[this->gpf_substate_mode].c_str());

  switch (this->gpf_substate_mode) {
  case pos_only:
    this->rgbd_gpf_measurement_indices = VectorXi::Zero(3);
    this->rgbd_gpf_measurement_indices = RBIS::positionInds();
    break;
  case pos_yaw:
    this->rgbd_gpf_measurement_indices = VectorXi(4);
    this->rgbd_gpf_measurement_indices.bottomRows(3) = RBIS::positionInds();
    this->rgbd_gpf_measurement_indices(0) = RBIS::chiInds()[2];
    break;
  case pos_chi:
    this->rgbd_gpf_measurement_indices = VectorXi(6);
    this->rgbd_gpf_measurement_indices.topRows(3) = RBIS::chiInds();
    this->rgbd_gpf_measurement_indices.bottomRows(3) = RBIS::positionInds();
    break;
  case all_states:
    this->rgbd_gpf_measurement_indices = VectorXi(9);
    this->rgbd_gpf_measurement_indices.segment < 3 > (0) = RBIS::velocityInds();
    this->rgbd_gpf_measurement_indices.segment < 3 > (3) = RBIS::chiInds();
    this->rgbd_gpf_measurement_indices.segment < 3 > (6) = RBIS::positionInds();
    break;
  case z_only:
    this->rgbd_gpf_measurement_indices = VectorXi(1);
    this->rgbd_gpf_measurement_indices(0) = RBIS::position_ind + 2;
    break;
  default:
    assert(false);
    break;
  }

  eigen_dump(rgbd_gpf_measurement_indices);

  if (gpf_vis) {
    this->lcmgl_rgbd = bot_lcmgl_init(this->lcm, "GPF_rgbd");
    this->lcmgl_particles_rgbd = bot_lcmgl_init(this->lcm, "GPF_particles_rgbd");
  }
  else {
    this->lcmgl_rgbd = NULL;
    this->lcmgl_particles_rgbd = NULL;
  }

}

RgbdGPF::RgbdGPF(lcm_t * lcm, BotParam * param, BotFrames * frames)
{

  //param loading
  int num_samples_ = bot_param_get_int_or_fail(param, "state_estimator.rgbd_gpf.gpf_num_samples");
  bool gpf_vis = bot_param_get_boolean_or_fail(param, "state_estimator.rgbd_gpf.gpf_vis");

  char * tmpstr = bot_param_get_str_or_fail(param, "state_estimator.rgbd_gpf.map_name");
  std::string data_dir = getDataPath();
  string map_name;
  if (g_path_is_absolute (tmpstr))
    map_name = tmpstr;
  else if (g_str_has_prefix(tmpstr, "~"))
    map_name = g_build_filename(g_get_home_dir(), tmpstr + 1, NULL);
  else
    map_name = data_dir + "/" + tmpstr;

  free(tmpstr);

  max_weight_proportion = bot_param_get_double_or_fail(param, "state_estimator.rgbd_gpf.max_weight_proportion"); // added mfallon
  double unknown_loglike = bot_param_get_double_or_fail(param, "state_estimator.rgbd_gpf.unknown_loglike");
  double cov_scaling = bot_sq(bot_param_get_double_or_fail(param, "state_estimator.rgbd_gpf.sigma_scaling"));
  RgbdLikelihoodInterface * laser_like_iface_ = new RgbdOctomapLikelihoodInterface(map_name.c_str(), unknown_loglike, cov_scaling);


  rgbd_gpf_substate gpf_substate_mode_ = RgbdGPF::num_substates;
  char * substate_str = bot_param_get_str_or_fail(param, "state_estimator.rgbd_gpf.gpf_substate");
  for (int i = 0; i < RgbdGPF::num_substates; i++) {
    if (substate_str == RgbdGPF::rgbd_gpf_substate_stings[i]) {
      gpf_substate_mode_ = (RgbdGPF::rgbd_gpf_substate) i;
      break;
    }
  }
  if (gpf_substate_mode_ == RgbdGPF::num_substates) {
    fprintf(stderr, "ERROR: %s is not a valid gpf substate!\n", substate_str);
    exit(1);
  }
  free(substate_str); // the string returned by bot_param needs to be freed

  RgbdGPFBaseConstructor(num_samples_, gpf_vis, laser_like_iface_, gpf_substate_mode_, lcm, param, frames);

}

RgbdGPF::~RgbdGPF()
{
  laser_projector_destroy(laser_projector);

  delete laser_like_iface;

  if (this->lcmgl_rgbd != NULL) {
    bot_lcmgl_destroy(lcmgl_rgbd);
  }
}

double RgbdGPF::likelihoodFunction(const RBIS & state)
{

  BotTrans state_to_map;
  state.getBotTrans(&state_to_map);
  double likelihood = this->laser_like_iface->evaluateScanLogLikelihood(this->projected_laser_scan, state);

  if (this->lcmgl_rgbd != NULL) {

    bot_lcmgl_point_size(this->lcmgl_rgbd, 2);
    bot_lcmgl_begin(this->lcmgl_rgbd, GL_POINTS);

    double point_likelihood = 0;
    for (int i = 0; i < this->projected_laser_scan->npoints; i++) {
      if (this->projected_laser_scan->point_status[i] > laser_valid_projection)
        continue;
      double proj_xyz[3];
      bot_trans_apply_vec(&state_to_map, point3d_as_array(&this->projected_laser_scan->points[i]), proj_xyz);
      point_likelihood = this->laser_like_iface->evaluatePointLogLikelihood(proj_xyz);
      float * color = bot_color_util_jet(exp(point_likelihood + this->laser_like_iface->minNegLogLike));
      bot_lcmgl_color3f(this->lcmgl_rgbd, color[0], color[1], color[2]);
      bot_lcmgl_vertex3f(this->lcmgl_rgbd, proj_xyz[0], proj_xyz[1], proj_xyz[2]);
    }
    bot_lcmgl_end(this->lcmgl_rgbd);

  }

  return likelihood;
}







int rgbd_projected_scan(Laser_projector * projector, laser_projected_scan * proj_scan,
    const char * dest_frame, const kinect::frame_msg_t* msg)
{
  if (!bot_frames_get_trans_with_utime(projector->bot_frames, projector->coord_frame, dest_frame, proj_scan->utime,
      &proj_scan->origin)) {
    return proj_scan->projection_status;
  }

  int64_t latest_trans_timestamp;
  bot_frames_get_trans_latest_timestamp(projector->bot_frames, projector->coord_frame, dest_frame,
      &latest_trans_timestamp);

  bot_frames_get_trans_with_utime(projector->bot_frames, "body", bot_frames_get_root_name(projector->bot_frames),
      proj_scan->utime, &proj_scan->body);

  if (proj_scan->projection_status == 0 && (latest_trans_timestamp - 100000) > proj_scan->utime) {
    proj_scan->projection_status = -1;
  }
  else if (latest_trans_timestamp > proj_scan->utime) {
    proj_scan->projection_status = 1;
  }
  else
    proj_scan->projection_status = 0;

  proj_scan->numValidPoints = 0;

  double aveRange = 0;
  double aveRangeSq = 0;
  double surroundCount = 0;

  //values for motion correcting the scan
  //double timestep = (proj_scan->rawScan->radstep / (2 * M_PI)) / proj_scan->projector->laser_frequency;
  //double scan_time = proj_scan->rawScan->nranges * timestep;
  double dt;
  BotTrans laser_hit_time_to_laser_current_time;
  BotTrans laser_hit_time_to_dest;

  /////////////////////////////////////////////////////////////////////
  /// 1.2. DEPTH:
  const uint16_t* val;
  // TODO: put uncompress_buffer in object
  uint8_t* uncompress_buffer;
  int uncompress_buffer_size;
  // Is this copy required??? Ask Abe?
  copy(msg->depth.depth_data.begin(), msg->depth.depth_data.end(), rgbd_data_buffer);
  // 1.2.1 De-compress if necessary:
  if(msg->depth.compression != kinect::depth_msg_t::COMPRESSION_NONE) {
    //std:: cout << "compression \n ";
    if(msg->depth.uncompressed_size > uncompress_buffer_size) {
      uncompress_buffer_size = msg->depth.uncompressed_size;
      uncompress_buffer = (uint8_t*) realloc(uncompress_buffer, uncompress_buffer_size);
    }
    unsigned long dlen = msg->depth.uncompressed_size;
    int status = uncompress(uncompress_buffer, &dlen, 
        rgbd_data_buffer, msg->depth.depth_data_nbytes);
    if(status != Z_OK) {
      return -1;
    }
    val = reinterpret_cast<const uint16_t*>( rgbd_data_buffer );
  }else{
    val = reinterpret_cast<const uint16_t*>( rgbd_data_buffer );
  }

  int npixels = msg->depth.width * msg->depth.height;

  int kinect_decimate=32;
  double fx = 576.09757860; // hard coded focal length

  /////////////////////////////////////////////////////////////////////
  // Openni Data:
  double xyzw[4];
  int j=0;
  int j2=0;
  for(int v=0; v<msg->depth.height; v=v+ kinect_decimate) { // t2b self->height 480
    for(int u=0; u<msg->depth.width; u=u+kinect_decimate ) {  //l2r self->width 640
      double sensor_xyz[3]; //point in sensor coordinates
      double range=0;
      double disparity_d = val[v*msg->depth.width+u]  / 1000.0; // convert to m
      if (disparity_d==0){ // place null points at 0,0,0
        sensor_xyz[0] = 0;
        sensor_xyz[1] = 0;
        sensor_xyz[2] = 0;
        proj_scan->point_status[j2] = 3;
        proj_scan->point_status[j2] = bot_max(proj_scan->point_status[j2],laser_invalid_projection);
      }else{
        double constant = 1.0f / fx;//kcal->intrinsics_rgb.fx ;
        sensor_xyz[0] = disparity_d;  //x forward+
        sensor_xyz[1] =  - (((double) u)- 319.50)*disparity_d*constant; //y right+ (check)
        sensor_xyz[2] = - (((double) v)- 239.50)*disparity_d*constant;  //z up+

        range= sqrt( pow(sensor_xyz[0],2) + pow(sensor_xyz[1],2) + pow(sensor_xyz[2],2) );

        aveRange += range;
        aveRangeSq += range * range;
        surroundCount++;
        proj_scan->point_status[j2] = bot_max(proj_scan->point_status[j2],laser_surround);
      }

      if (proj_scan->point_status[j2] < laser_valid_projection) {
        proj_scan->numValidPoints++;
      }

      /* convert to local frame */
      //dt will be negative since we start farthest back in time (points[0]) and step forward to the last point
      //todo: check for 0 velocities and don't do set_from_velocities step to save computation
      // mfallon: I've crippled the interpolation:
      dt =0;// -scan_time + timestep * i;
      double laser_angular_rate[] ={0.0,0.0,0.0};
      double laser_velocity[] ={0.0,0.0,0.0};

      bot_trans_set_from_velocities(&laser_hit_time_to_laser_current_time, laser_angular_rate, laser_velocity, dt);
      bot_trans_apply_trans_to(&proj_scan->origin, &laser_hit_time_to_laser_current_time, &laser_hit_time_to_dest);
      bot_trans_apply_vec(&laser_hit_time_to_dest, sensor_xyz, point3d_as_array(&proj_scan->points[j2]));

      j2++;
    }
  }

  aveRange /= surroundCount;
  aveRangeSq /= surroundCount;
  proj_scan->aveSurroundRange = aveRange;
  proj_scan->stddevSurroundRange = sqrt(aveRangeSq - aveRange * aveRange);
  return proj_scan->projection_status;
}


laser_projected_scan *laser_create_projected_scan_from_rgbd_with_motion(Laser_projector * projector,
    const kinect::frame_msg_t *msg, const char * dest_frame, const double laser_angular_rate[3],
    const double laser_velocity[3])
{
  laser_projected_scan * proj_scan = (laser_projected_scan *) calloc(1, sizeof(laser_projected_scan));
  proj_scan->npoints =20*15;//msg->nranges;
  proj_scan->numValidPoints = 0;
  proj_scan->utime = msg->timestamp;//msg->utime;
  proj_scan->projector = projector;
  proj_scan->rawScan =NULL;// bot_core_planar_lidar_t_copy(msg);

  proj_scan->points = (point3d_t*) calloc(proj_scan->npoints, sizeof(point3d_t));
  g_assert(proj_scan->points);
  proj_scan->point_status = (uint8_t *) calloc(proj_scan->npoints, sizeof(uint8_t));
  g_assert(proj_scan->point_status);


  if (!bot_frames_have_trans(projector->bot_frames, projector->coord_frame, dest_frame)) {
    laser_destroy_projected_scan(proj_scan);
    return NULL;
  } else {
    if (rgbd_projected_scan(projector, proj_scan, dest_frame, msg) == -1) { //scan is arriving way late
      laser_destroy_projected_scan(proj_scan);
      return NULL;
    }
  }

  return proj_scan;
}


laser_projected_scan *laser_create_projected_scan_from_rgbd(Laser_projector * projector,
    const kinect::frame_msg_t *msg, const char * dest_frame)
{
  double zeros[3] = { 0 };
  return laser_create_projected_scan_from_rgbd_with_motion(projector, msg, dest_frame, zeros, zeros);
}

bool RgbdGPF::getMeasurement(const RBIS & state, const RBIM & cov, const kinect::frame_msg_t * laser_msg,
    Eigen::VectorXd & z_effective, Eigen::MatrixXd & R_effective)
{

  if (this->motion_project){
    Vector3d laser_omega = this->R_body_to_rgbd * state.angularVelocity();
    Vector3d laser_vel = this->R_body_to_rgbd * state.velocity();
    this->projected_laser_scan = laser_create_projected_scan_from_rgbd_with_motion(this->laser_projector,
        laser_msg, "body", laser_omega.data(), laser_vel.data());
  }
  else {
    this->projected_laser_scan = laser_create_projected_scan_from_rgbd(this->laser_projector, laser_msg,
        "body");
  }

  printf("mfallon: RgbdGPF::getMeasurement\n");

  // mfallon :printf("valid pts: %d\n",this->projected_laser_scan->numValidPoints);
  if (this->projected_laser_scan->numValidPoints < MIN_VALID_BEAMS) {
    if (this->verbose) {
      fprintf(stderr, "Not enough valid beams (%d), discarding scan!\n", this->projected_laser_scan->numValidPoints);
      laser_destroy_projected_scan(this->projected_laser_scan);
      return false;
    }
  }
  // This is done entirely within rgbd project
  //  laser_decimate_projected_scan(this->projected_laser_scan, this->beam_skip, this->spatial_decimation_min, this->spatial_decimation_max);

  gpfMeasurement(this, state, cov, this->rgbd_gpf_measurement_indices, z_effective, R_effective, this->num_samples,
      this->max_weight_proportion, this->lcmgl_particles_rgbd);

  laser_destroy_projected_scan(this->projected_laser_scan);

  if (this->lcmgl_rgbd != NULL) {
    bot_lcmgl_switch_buffer(this->lcmgl_rgbd);
  }
  if (this->lcmgl_particles_rgbd != NULL) {
    bot_lcmgl_switch_buffer(this->lcmgl_particles_rgbd);
  }


  bool ret = true;
  if (hasNan(z_effective)) {
    fprintf(stderr, "ERROR: z_effective has a Nan!\n");
    eigen_dump(z_effective.transpose());
    ret = false;
  }
  if (hasNan(R_effective)) {
    fprintf(stderr, "ERROR: R_effective has a Nan!\n");
    eigen_dump(R_effective);
    ret = false;
  }

  return ret;
}


}
