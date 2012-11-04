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

#include <path_util/path_util.h>
#include <lcmtypes/bot_param_update_t.h>

#include "rgbd_vis.hpp"

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


rgbd_vis::rgbd_vis(lcm_t* publish_lcm,lcm_t* subscribe_lcm):
          publish_lcm_(publish_lcm),
          subscribe_lcm_(subscribe_lcm),
          current_poseT(0, Eigen::Isometry3d::Identity()),
          null_poseT(0, Eigen::Isometry3d::Identity()),
          local_poseT(0, Eigen::Isometry3d::Identity()) {

  cloud_counter =0;

  pc_lcm_ = new pointcloud_lcm(publish_lcm_);
  pc_lcm_->set_kinect_decimate(4.0); // same as used by kixie

  // Vis Config:
  pc_vis_ = new pointcloud_vis(publish_lcm_);
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  vector <float> colors_v;
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Null",5,0) );
//  pc_vis_->obj_cfg_list.push_back( obj_cfg(800,"Pose - Laser Map Init",5,0) );
  float colors_b[] ={0.0,0.0,1.0};
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Cloud - Laser Map"     ,1,0, 1000,0,colors_v));

  rgb_buf_size_ = 640 * 480;
  rgb_buf_ = (uint8_t*) malloc(rgb_buf_size_* 3);
}


void rgbd_vis::kframe_handler(const kinect_frame_msg_t *msg){
  cout << "got kinect\n";

  std::cout << msg->timestamp << "\n";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pc_lcm_->unpack_kinect_frame(msg,rgb_buf_,cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;


/*  Eigen::Isometry3d null_pose;
  null_pose.setIdentity();
  null_poseT = Isometry3dTime(msg->timestamp, null_pose);
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT);
*/


    BotTrans cam_to_local;
    bot_frames_get_trans_with_utime (frames, "body", "local", msg->timestamp, &cam_to_local);
//    bot_frames_get_trans_with_utime (frames, "rgbd", "local", msg->timestamp, &cam_to_local);

  current_poseT.pose.setIdentity();
  current_poseT.pose.translation() << cam_to_local.trans_vec[0], cam_to_local.trans_vec[1], cam_to_local.trans_vec[2];
  Eigen::Quaterniond m;
  m  = Eigen::Quaterniond(cam_to_local.rot_quat[0],cam_to_local.rot_quat[1],cam_to_local.rot_quat[2],cam_to_local.rot_quat[3]);
  current_poseT.pose.rotate(m);
  current_poseT.utime = msg->timestamp;
  pc_vis_->pose_to_lcm_from_list(1000, current_poseT);

  pc_vis_->ptcld_to_lcm_from_list(1001, *cloud, current_poseT.utime, current_poseT.utime);

   cout << cam_to_local.trans_vec[0] << ", "
	 << cam_to_local.trans_vec[1] << ", "
	 << cam_to_local.trans_vec[2] << " | "
	 << cam_to_local.rot_quat[0] << ", "
	 << cam_to_local.rot_quat[1] << ", "
	 << cam_to_local.rot_quat[2] << ", "
	 << cam_to_local.rot_quat[3] << "\n";


}


void rgbd_vis::pose_handler(const bot_core_pose_t *msg){
  bot_pose_last = bot_core_pose_t_copy (msg);
}


int rgbd_vis::initialize(int argc, char **argv){

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


  kinect_frame_msg_t_subscribe(subscribe_lcm_, "KINECT_FRAME",
      rgbd_vis::kframe_handler_aux, this);
  bot_core_pose_t_subscribe(subscribe_lcm_, "POSE",
      rgbd_vis::pose_handler_aux, this);

  return 1;
}


static void usage(const char *progname){
  fprintf (stderr, "usage: %s [options]\n"
      "\n"
      "  -c, --config PATH      Location of config file\n"
      , g_path_get_basename(progname));
}

int main(int argc, char ** argv) {

  lcm_t * lcm;
  lcm = lcm_create(NULL);//"udpm://239.255.76.67:7667?recv_buf_size=100000");

  rgbd_vis app(lcm,lcm);
  int status =  app.initialize(argc, argv);
  if (!status) { return status; }


  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}
