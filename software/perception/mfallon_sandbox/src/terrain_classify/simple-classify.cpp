#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/assign/std/vector.hpp>
#include <lcmtypes/visualization.h>

#include "simple-classify.hpp"

using namespace std;
using namespace pcl;
using namespace boost::assign; // bring 'operator+()' into scope


simple_classify::simple_classify(lcm_t* publish_lcm):
          publish_lcm_(publish_lcm),
          current_pose_headT(0, Eigen::Isometry3d::Identity()),
          current_pose_bodyT(0, Eigen::Isometry3d::Identity()),
          null_poseT(0, Eigen::Isometry3d::Identity()),
          local_poseT(0, Eigen::Isometry3d::Identity()) {

  botparam_ = bot_param_new_from_server(publish_lcm_, 0);
  botframes_= bot_frames_get_global(publish_lcm_, botparam_);

  counter_ =0;

  // Unpack Config:
  pc_lcm_ = new pointcloud_lcm(publish_lcm_);
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis(publish_lcm_);
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(600,"Pose - Laser",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(601,"Cloud - Laser"         ,1,1, 600,1, {0.0, 0.0, 1.0} ));

  pc_vis_->obj_cfg_list.push_back( obj_cfg(1200,"Pose - Body2Lidar",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1201,"Cloud - Body2Lidar"     ,1,1, 1200,1, {0.0, 0.0, 1.0} ));
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(800,"Pose - Body",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(801,"Cloud - Body"     ,1,1, 800,1, {0.0, 0.0, 1.0} ));

  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Null",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Cloud - Null"             ,1,0, 1000,0, { -1, -1, -1} ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1004,"Cloud - Body Yaw relative",1,0, 1000,0, {1.0,0.0,1.0}));
    
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1010,"POSE_HEAD",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1011,"POSE_BODY",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1012,"POSE_BODY (Yaw only)",5,1) );

  // LCM:
  lcm_t* subscribe_lcm_ = publish_lcm_;
  bot_core_planar_lidar_t_subscribe(subscribe_lcm_, "ROTATING_SCAN",
      simple_classify::lidar_handler_aux, this);
  bot_core_pose_t_subscribe(subscribe_lcm_, "POSE_HEAD",
      simple_classify::pose_handler_aux, this);
  bot_core_pose_t_subscribe(subscribe_lcm_, "POSE_BODY",
      simple_classify::pose_body_handler_aux, this);
}

void simple_classify::pose_body_handler(const bot_core_pose_t *msg){
  current_pose_bodyT.pose.setIdentity();
  current_pose_bodyT.pose.translation() << msg->pos[0], msg->pos[1], msg->pos[2];
  Eigen::Quaterniond m;
  m  = Eigen::Quaterniond(msg->orientation[0],msg->orientation[1],msg->orientation[2],msg->orientation[3]);
  current_pose_bodyT.pose.rotate(m);
  current_pose_bodyT.utime = msg->utime;
}

void simple_classify::pose_handler(const bot_core_pose_t *msg){
  current_pose_headT.pose.setIdentity();
  current_pose_headT.pose.translation() << msg->pos[0], msg->pos[1], msg->pos[2];
  Eigen::Quaterniond m;
  m  = Eigen::Quaterniond(msg->orientation[0],msg->orientation[1],msg->orientation[2],msg->orientation[3]);
  current_pose_headT.pose.rotate(m);
  current_pose_headT.utime = msg->utime;
}

void simple_classify::lidar_handler(const bot_core_planar_lidar_t *msg){

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  double maxRange = 29.7;//29.7;
  double validBeamAngles[] ={-10,10}; // consider everything
  convertLidar(msg->ranges, msg->nranges, msg->rad0,
      msg->radstep, lidar_cloud, maxRange,
      validBeamAngles[0], validBeamAngles[1]);

  //  int64_t pose_id=msg->utime;
  counter_++;
  if (counter_ >=20){ // set this to 1 to only see the last return
    counter_=0;
  }
  int64_t pose_id=counter_;//msg->utime;
  
  Eigen::Isometry3d null_pose;
  null_pose.setIdentity();
  null_poseT = Isometry3dTime(pose_id, null_pose);
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT);
  
  pc_vis_->pose_to_lcm_from_list(1010, current_pose_headT);
  pc_vis_->pose_to_lcm_from_list(1011, current_pose_bodyT);
  
  Eigen::Quaterniond r(current_pose_bodyT.pose.rotation());
  double ypr[3];
  quat_to_euler(r, ypr[0], ypr[1], ypr[2]);
  Eigen::Quaterniond r_yaw = euler_to_quat(ypr[0] ,0, 0);
  Eigen::Isometry3d current_pose_body_yaw;
  current_pose_body_yaw.setIdentity();  
  current_pose_body_yaw.translation()  << current_pose_bodyT.pose.translation().x(), current_pose_bodyT.pose.translation().y(), current_pose_bodyT.pose.translation().z();
  current_pose_body_yaw.rotate(r_yaw);
  Isometry3dTime current_pose_body_yawT = Isometry3dTime(pose_id, current_pose_body_yaw) ;
  pc_vis_->pose_to_lcm_from_list(1012, current_pose_body_yawT);


  Eigen::Isometry3d lidar_to_local;
  frames_cpp->get_trans_with_utime( botframes_ ,  "ROTATING_SCAN", "local", msg->utime, lidar_to_local);
  
  Eigen::Isometry3d lidar_to_body;
  frames_cpp->get_trans_with_utime( botframes_ ,  "ROTATING_SCAN", "body", msg->utime, lidar_to_body);
  Eigen::Isometry3d body_to_lidar;
  frames_cpp->get_trans_with_utime( botframes_ , "body",  "ROTATING_SCAN", msg->utime, body_to_lidar);
  
  Eigen::Isometry3d body_to_local;
  frames_cpp->get_trans_with_utime( botframes_ ,  "body", "local", msg->utime, body_to_local);
  
  Isometry3dTime lidar_poseT = Isometry3dTime(pose_id, lidar_to_local);
  pc_vis_->pose_to_lcm_from_list(600, lidar_poseT);
  pc_vis_->ptcld_to_lcm_from_list(601, *lidar_cloud, pose_id, pose_id);


  Isometry3dTime lidar_to_bodyT = Isometry3dTime(pose_id, lidar_to_body);
  Eigen::Isometry3f l2b_f = Isometry_d2f(lidar_to_body );
  Eigen::Quaternionf l2b_quat(l2b_f.rotation());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_cloud_l2b (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::transformPointCloud (*lidar_cloud, *lidar_cloud_l2b,
      l2b_f.translation(), l2b_quat); // !! modifies lidar_cloud
  
  pc_vis_->pose_to_lcm_from_list(1200, null_poseT);
  pc_vis_->ptcld_to_lcm_from_list(1201, *lidar_cloud_l2b, pose_id, pose_id);
  
  
  Eigen::Isometry3f pose_f = Isometry_d2f(lidar_poseT.pose);
  Eigen::Quaternionf pose_quat(pose_f.rotation());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_cloud_l2local (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::transformPointCloud (*lidar_cloud, *lidar_cloud_l2local,
      pose_f.translation(), pose_quat);

//  pc_vis_->pose_to_lcm_from_list(800, lidar_poseT);
  Isometry3dTime body_poseT = Isometry3dTime(pose_id, body_to_local);
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT);  
  
  Eigen::Isometry3f posey_f = Isometry_d2f(current_pose_body_yawT.pose.inverse());
  Eigen::Quaternionf posey_quat(posey_f.rotation());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_rel_bodyyaw (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::transformPointCloud (*lidar_cloud_l2local, *lidar_rel_bodyyaw,
      posey_f.translation(), posey_quat);
  

  int n_obstacle_returns =0;
  for (size_t i=0; i < lidar_rel_bodyyaw->points.size() ; i ++){
    pcl::PointXYZRGB pt = lidar_rel_bodyyaw->points[i];
    if (( fabs(pt.x) < 2.0 ) &&  ( fabs(pt.y) < 2.0 )) {  // if near to robot - returns from the car
      pt.r =120.0;
      pt.g =120.0;
      pt.b =120.0;
    }else{
      if ( pt.z > -1.0 ){ // If above the ground (TODO: make this automatic
        pt.r =255.0;
        pt.g =0.0;
        pt.b =0.0;
        if ( pt.x < 6.0 ){ // within 6m in front
          if ( fabs(pt.y) < 2.5 ){ // within 2m left and right
            pt.r =0.0;
            pt.g =0.0;
            pt.b =255.0;
            n_obstacle_returns++;
          }
        }
      }else{ // not near car, not in front of the car
        pt.r =0.0;
        pt.g =255.0;
        pt.b =0.0;
      }
    }
    lidar_rel_bodyyaw->points[i]=pt;
    lidar_cloud_l2local->points[i].r=pt.r;
    lidar_cloud_l2local->points[i].g=pt.g;
    lidar_cloud_l2local->points[i].b=pt.b;
  }
  
  pc_vis_->ptcld_to_lcm_from_list(1004, *lidar_rel_bodyyaw, pose_id, pose_id);
  pc_vis_->ptcld_to_lcm_from_list(1001, *lidar_cloud_l2local, pose_id, pose_id);
  
  if (n_obstacle_returns > 10){
    cout << "obstacle in front: " << n_obstacle_returns << "\n"; 
    double goal_timeout=3.0;
  
    drc_nav_goal_timed_t msgout;
    msgout.utime = msg->utime; //bot_timestamp_now();
    msgout.timeout = (int64_t) 1E6*goal_timeout;
    msgout.robot_name = "atlas"; // this should be set from robot state message

    msgout.goal_pos.translation.x = 0;
    msgout.goal_pos.translation.y = 0;
    msgout.goal_pos.translation.z = 0;
    msgout.goal_pos.rotation.w = 0;
    msgout.goal_pos.rotation.x = 0;
    msgout.goal_pos.rotation.y = 0;
    msgout.goal_pos.rotation.z = 0;
    printf("Sending NAV_GOAL_ESTOP\n");
    drc_nav_goal_timed_t_publish(publish_lcm_, "NAV_GOAL_ESTOP", &msgout);
  
    
    // publish estop message
    // on arbirtator end:
    // listen to estop message, keep timestamp, 
    // if timestamp is recent stop robot
    // else drive as usual
  }
  
}


int main(int argc, char ** argv){
  lcm_t * lcm;
  lcm = lcm_create(NULL);
  simple_classify app(lcm);

  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}
