// Local Map using Velodyne data:
// - listens to Velodyne data chunks (continously)
// - If it heres a trigger message:
// - takes a portion of them - currently the forward facing 180degrees
// - Publishes as a single point cloud message
// - Continues
//
// mfallon sept 2012

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <getopt.h>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include <velodyne/velodyne.h>
#include <path_util/path_util.h>
#include <lcmtypes/bot_param_update_t.h>

#include "lcmtypes/senlcm_velodyne_t.h"
#include "lcmtypes/senlcm_velodyne_list_t.h"

#include "local_map_velodyne.hpp"

#define GAMMA 0.1
#define PUBLISH_HZ 50
#define VELODYNE_DATA_CIRC_SIZE 1
#define SHORT_RANGE_FILTER 0.3

using namespace std;

typedef struct _xyzi_t {
  double xyz[3];
  double intensity;
} xyzi_t;

typedef struct _pose_data_t pose_data_t;
struct _pose_data_t {
  double pose[6];
  double motion[6];
  int64_t utime;
};


local_map::local_map(lcm_t* publish_lcm,lcm_t* subscribe_lcm):
          publish_lcm_(publish_lcm),
          subscribe_lcm_(subscribe_lcm),
          current_poseT(0, Eigen::Isometry3d::Identity()),
          null_poseT(0, Eigen::Isometry3d::Identity()),
          local_poseT(0, Eigen::Isometry3d::Identity()) {

  cloud_counter =0;

  // Vis Config:
  pc_vis_ = new pointcloud_vis(publish_lcm_);
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  vector <float> colors_v;
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Null",5,0) );
//  pc_vis_->obj_cfg_list.push_back( obj_cfg(800,"Pose - Laser Map Init",5,0) );
  float colors_b[] ={0.0,0.0,1.0};
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(801,"Cloud - Laser Map"     ,1,1, 1000,1,colors_v));
}

void circ_free_velodyne_data(void *user, void *p) {
  velodyne_laser_return_collection_t *lrc = (velodyne_laser_return_collection_t*) p;
  velodyne_free_laser_return_collection (lrc);
}


void write_pcd(vector <xyzi_t> &points,std::ofstream &fstream ){

  fstream << "# .PCD v.5 - Point Cloud Data file format" << endl;
  fstream << "FIELDS x y z intensity" << endl;
  fstream << "SIZE 4 4 4 4" << endl;
  fstream << "TYPE F F F F" << endl;
  fstream << "WIDTH " << points.size() << endl;
  fstream << "HEIGHT 1" << endl;
  fstream << "POINTS " << points.size() << endl;
  fstream << "DATA ascii" << endl;

  for (size_t i=0; i < points.size(); i++){
    xyzi_t lr = points[i];
    ostringstream temp0;
    temp0 << lr.xyz[0] <<" " << lr.xyz[1]<< " " << lr.xyz[2] << " "<< lr.intensity << endl;
    fstream << temp0.str();
  }
  fstream.close();
}


void local_map::unpack_velodyne(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){

  int size = bot_ptr_circular_size(velodyne_data_circ);
  vector < xyzi_t > points;
  int hist_len = VELODYNE_DATA_CIRC_SIZE;

  for (unsigned int cidx = 0;
      cidx < bot_ptr_circular_size(velodyne_data_circ) && cidx < hist_len;
      cidx++) {

    velodyne_laser_return_collection_t *lrc =(velodyne_laser_return_collection_t *) bot_ptr_circular_index(velodyne_data_circ, cidx);

    double sensor_to_local[12];
    // Working here on bot frames

    if (!bot_frames_get_trans_mat_3x4 (frames, "VELODYNE",
        "local",
        sensor_to_local)) {
      fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
      return;
    }

    //printf("LASER returns : %d\n", lrc->num_lr);

    int chunk_size = 32;// * 12;

    // Resize here:
    cloud->width   = lrc->num_lr;
    cloud->height   = 1;
    cloud->points.resize (cloud->width  *cloud->height);

    for (int s = 0; s < lrc->num_lr; s++) {
      velodyne_laser_return_t *lr = &(lrc->laser_returns[s]);

      if(s % chunk_size == 0){
        //updated the sensor_to_local transform
        if (!bot_frames_get_trans_mat_3x4_with_utime (frames, "VELODYNE",
            "local", lr->utime,
            sensor_to_local)) {
          fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
          return;
        }
      }

      //fprintf(stderr, "\t %d - %d : %f\n", lr->physical, lr->logical, lr->phi);
      //double local_xyz[3];

      if (lr->range > SHORT_RANGE_FILTER){
        double local_xyz[3];
        bot_vector_affine_transform_3x4_3d (sensor_to_local, lr->xyz, local_xyz);
        cloud->points[s].x = local_xyz[0];
        cloud->points[s].y = local_xyz[1];
        cloud->points[s].z = local_xyz[2];
        cloud->points[s].r = (int) lr->intensity; // should use XYZI instead
        cloud->points[s].g = (int) lr->intensity; // should use XYZI instead
        cloud->points[s].b = (int) lr->intensity; // should use XYZI instead
      }
    }
  }
}

void local_map::newmap_handler(const drc_localize_reinitialize_cmd_t *msg){
  cout << "got trigger\n";

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr vcloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  unpack_velodyne(vcloud);

  //b Send Null Pose
  Eigen::Isometry3d null_pose;
  null_pose.setIdentity();
  null_poseT = Isometry3dTime(msg->utime, null_pose);
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT);

  //c Send Point Cloud:
  vector <float> colors_v;
  float colors_a[3];
  colors_a[0] = pc_vis_->colors[3*cloud_counter];
  colors_a[1] = pc_vis_->colors[3*cloud_counter+1];
  colors_a[2] = pc_vis_->colors[3*cloud_counter+2];
  colors_v.assign(colors_a,colors_a+4*sizeof(float));
  stringstream ss;
  ss << "Cloud - Local Map " << cloud_counter;
  int map_coll_id = cloud_counter + 1000 + 1; // so that first is 1001 and null is 1000
  ptcld_cfg pcfg = ptcld_cfg(map_coll_id,    ss.str()     ,1,1, 1000,1,colors_v);
  pc_vis_->ptcld_to_lcm(pcfg, *vcloud, null_poseT.utime, null_poseT.utime );

  cout << cloud_counter << " sent | " << vcloud->points.size() << " points\n";



  pc_vis_->pointcloud2_to_lcm(*vcloud,"LOCAL_MAP_POINTS", null_poseT.utime);


  if(1==0){
  pcl::PCDWriter writer;
  stringstream ss2;
  ss2 << "cloud" << cloud_counter << ".pcd";
  writer.write (ss2.str(), *vcloud, false);
  }

  cloud_counter++;
  if(cloud_counter*3 >= pc_vis_->colors.size() ){
    cloud_counter=0; 
  }  
}




void local_map::pose_handler(const bot_core_pose_t *msg){
  bot_pose_last = bot_core_pose_t_copy (msg);
}

void local_map::velodyne_handler(const senlcm_velodyne_list_t *msg){

  static int64_t last_redraw_utime = 0;
  int64_t now = bot_timestamp_now();

  //cout << msg->num_packets << " num packets\n";
  for (int i=0; i < msg->num_packets; i++)
    process_velodyne (&(msg->packets[i]));

  return;
}


// Processes
int local_map::process_velodyne (const senlcm_velodyne_t *v)
{

  int do_push_motion = 0; // only push motion data if we are starting a new collection or there is a new pose

  // MFallon: I chose this parameter - not sure what its effect is:
  double hist_spc = 1000.0;//bot_gtk_param_widget_get_double (self->pw, PARAM_HISTORY_FREQUENCY);


  // Algorithm:
  // Decodes and pushes velodyne_t packets into the collector
  // When the collector is full it pushes the collector in its entirety into the circular buffer
  // The collector is a portion of a scan

  // Is this a scan packet?
  if (v->packet_type == SENLCM_VELODYNE_T_TYPE_DATA_PACKET) {

    velodyne_laser_return_collection_t *lrc =
        velodyne_decode_data_packet(calib, v->data, v->datalen, v->utime);

    int ret = velodyne_collector_push_laser_returns (collector, lrc);

    velodyne_free_laser_return_collection (lrc);

    if (VELODYNE_COLLECTION_READY == ret) {

      velodyne_laser_return_collection_t *lrc =
          velodyne_collector_pull_collection (collector);

      // if enough time has elapsed since the last scan push it onto the circular buffer
      if (abs (lrc->utime - last_velodyne_data_utime) > (int64_t)(1E6/hist_spc)) {

        bot_ptr_circular_add (velodyne_data_circ, lrc);
        last_velodyne_data_utime = lrc->utime;
      } else {
        // memory leak city if this isnt here as soon as you increase the history spacing
        velodyne_free_laser_return_collection (lrc);
      }

      //starting a new collection
      do_push_motion = 1;
    }
    else if(VELODYNE_COLLECTION_READY_LOW == ret) {
      fprintf(stderr,"Low packet - ignoring");

      velodyne_laser_return_collection_t *lrc =
          velodyne_collector_pull_collection (collector);

      velodyne_free_laser_return_collection (lrc);
    }
  }

  // Update the Velodyne's state information (pos, rpy, linear/angular velocity)
  if (do_push_motion) {
    //cout << "do_push_motion 1\n";

    if (!bot_pose_last)
      return 0;

    //cout << "do_push_motion 2\n";


    // push new motion onto collector
    velodyne_state_t state;

    state.utime = v->utime;

    // find sensor pose in local/world frame
    //
    // double x_lr[6] = {self->pose->x, self->pose->y, self->pose->z,
    //                   self->pose->r, self->pose->p, self->pose->h};
    // double x_ls[6] = {0};
    // ssc_head2tail (x_ls, NULL, x_lr, self->x_vs);

    BotTrans velodyne_to_local;
    bot_frames_get_trans_with_utime (frames, "VELODYNE", "local", v->utime, &velodyne_to_local);

    memcpy (state.xyz, velodyne_to_local.trans_vec, 3*sizeof(double));
    bot_quat_to_roll_pitch_yaw (velodyne_to_local.rot_quat, state.rph);

    // Compute translational velocity
    //
    // v_velodyne = v_bot + r x w
    BotTrans velodyne_to_body;
    bot_frames_get_trans (frames, "VELODYNE", "body", &velodyne_to_body);

    double v_velodyne[3];
    double r_body_to_velodyne_local[3];
    bot_quat_rotate_to (bot_pose_last->orientation, velodyne_to_body.trans_vec, r_body_to_velodyne_local);

    // vel_rot = r x w
    double vel_rot[3];
    bot_vector_cross_3d (r_body_to_velodyne_local, bot_pose_last->rotation_rate, vel_rot);

    bot_vector_add_3d (bot_pose_last->vel, vel_rot, state.xyz_dot);

    // Compute angular rotation rate
    memcpy (state.rph_dot, bot_pose_last->rotation_rate, 3*sizeof(double));

    velodyne_collector_push_state (collector, state);
    do_push_motion = 0;
  }
  return 1;
}


int local_map::initialize(int argc, char **argv){

  // TODO: read the cfg param from the command line using Concise args:
  string config_file("drc_rig.cfg");

  if (0==1){// (config_file){
    fprintf(stderr,"Reading velodyne config from file\n");
    char config_path[2048];
    sprintf(config_path, "%s/%s", getConfigPath(),config_file.c_str());
    printf("%s is config_path\n",config_path);
    param = bot_param_new_from_file(config_path);

    if(!param){
      fprintf(stderr, "Couldn't get bot param from file %s\n",
          config_path);
      return 0;
    }
  }else {
    param = bot_param_new_from_server(subscribe_lcm_, 0);
    if (param == NULL) {
      fprintf(stderr, "Couldn't get bot param from server.\n");
      return 0;
    }
  }

  frames = bot_frames_get_global (subscribe_lcm_, param);

  char *velodyne_model = bot_param_get_str_or_fail (param, "calibration.velodyne.model");
  char *calib_file = bot_param_get_str_or_fail (param, "calibration.velodyne.intrinsic_calib_file");
  char calib_file_path[2048];
  sprintf(calib_file_path, "%s/%s", getConfigPath(), calib_file);

  if (0 == strcmp (velodyne_model, VELODYNE_HDL_32E_MODEL_STR))
    calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_32E, calib_file_path);
  else if (0 == strcmp (velodyne_model, VELODYNE_HDL_64E_S1_MODEL_STR))
    calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_64E_S1, calib_file_path);
  else if (0 == strcmp (velodyne_model, VELODYNE_HDL_64E_S2_MODEL_STR))
    calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_64E_S2, calib_file_path);
  else
    fprintf (stderr, "ERROR: Unknown Velodyne model \'%s\'", velodyne_model);

  free (velodyne_model);
  free (calib_file);


  // Which polar segments of the velodyne returns should be logged: NBNBNBNBNB NBNBNBNBNB NBNBNBNBNB
  // self->collector = velodyne_laser_return_collector_create (1, 0, 2* M_PI); // full scan
  collector = velodyne_laser_return_collector_create (0, M_PI, 2* M_PI); // front facing pixels only

  // Create a cyclic buffer for messages:
  velodyne_data_circ = bot_ptr_circular_new (VELODYNE_DATA_CIRC_SIZE,
      circ_free_velodyne_data, this);


  drc_localize_reinitialize_cmd_t_subscribe(subscribe_lcm_, "LOCALIZE_NEW_MAP",
      newmap_handler_aux, this);
  bot_core_pose_t_subscribe(subscribe_lcm_, "POSE",
      local_map::pose_handler_aux, this);
  senlcm_velodyne_list_t_subscribe(subscribe_lcm_, "VELODYNE_LIST",
      local_map::velodyne_handler_aux, this);

  return 1;
}


static void usage(const char *progname){
  fprintf (stderr, "usage: %s [options]\n"
      "\n"
      "  -c, --config PATH      Location of config file\n"
      , g_path_get_basename(progname));
}

int main(int argc, char ** argv) {
  char *optstring = "hc:";
  int c;
  struct option long_opts[] = {
      {"help", no_argument, 0, 'h'},
      {"config", required_argument, 0, 'c'},
  };

  int exit_code = 0;
  char *config_file =NULL;// g_strdup("velodyne.cfg");

  while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0)
  {
    switch (c)
    {
    case 'c':
      free(config_file);
      config_file = g_strdup(optarg);
      break;
    case 'h':
    default:
      usage(argv[0]);
      return 1;
    }
  }

  lcm_t * lcm;
  lcm = lcm_create(NULL);//"udpm://239.255.76.67:7667?recv_buf_size=100000");

  local_map app(lcm,lcm);
  int status =  app.initialize(argc, argv);
  if (!status) { return status; }


  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}
