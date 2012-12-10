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

#include "drc-octomap-trigger.hpp"
#include <lcmtypes/drc_map_params_t.h>

#include <ConciseArgs>

using namespace std;


octomap_trigger::octomap_trigger(lcm_t* publish_lcm,lcm_t* subscribe_lcm):
          publish_lcm_(publish_lcm),
          subscribe_lcm_(subscribe_lcm),
          current_poseT(0, Eigen::Isometry3d::Identity()),
          null_poseT(0, Eigen::Isometry3d::Identity()),
          local_poseT(0, Eigen::Isometry3d::Identity()) {


}


/*
void octomap_trigger::kframe_handler(const kinect_frame_msg_t *msg){
  cout << "got kinect\n";

  std::cout << msg->timestamp << "\n";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pc_lcm_->unpack_kinect_frame(msg,rgb_buf_,cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;




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
*/


void octomap_trigger::pose_handler(const bot_core_pose_t *msg){
  
  drc_map_params_t msgout;
  msgout.utime = bot_timestamp_now();
  msgout.message_id = 0;
  msgout.map_id = id_;
  msgout.resolution = res_;
  msgout.dimensions[0] = xdim_;
  msgout.dimensions[1] = ydim_;
  msgout.dimensions[2] = zdim_;
  
  
  msgout.transform_to_local.translation.x = msg->pos[0];
  msgout.transform_to_local.translation.y = msg->pos[1];
  msgout.transform_to_local.translation.z = msg->pos[2];
  msgout.transform_to_local.rotation.x = 0;
  msgout.transform_to_local.rotation.y = 0;
  msgout.transform_to_local.rotation.z = 0;
  msgout.transform_to_local.rotation.w = 1; // keep world aligned
  
  drc_map_params_t_publish(publish_lcm_,"MAP_CREATE",&msgout);
  cout << "done sending new map\n";
  exit(-1);
}


int octomap_trigger::initialize(int argc, char **argv){

  string channel = "MAP_CREATE";
  int id(0);
  double res(0.02);
  double xdim(10), ydim(10), zdim(10);
  ConciseArgs opt(argc, (char**)argv);
  opt.add(channel, "c", "channel", "channel to publish create message");
  opt.add(id, "i", "id", "id of new map");
  opt.add(res, "r", "res", "resolution of new map");
  opt.add(xdim, "x", "xdim", "x size of new map");
  opt.add(ydim, "y", "ydim", "y size of new map");
  opt.add(zdim, "z", "zdim", "z size of new map");
  opt.parse();
  
  id_ = id;
  res_ = res;
  xdim_ = xdim;
  ydim_ = ydim;
  zdim_ = zdim;
  
/*  
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
*/

  bot_core_pose_t_subscribe(subscribe_lcm_, "POSE",
      octomap_trigger::pose_handler_aux, this);

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

  octomap_trigger app(lcm,lcm);
  int status =  app.initialize(argc, argv);
  if (!status) { return status; }


  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}
