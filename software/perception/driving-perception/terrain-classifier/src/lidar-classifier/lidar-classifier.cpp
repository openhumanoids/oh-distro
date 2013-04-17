// publish estop message
// on arbirtator end:
// listen to estop message, keep timestamp, 
// if timestamp is recent stop robot
// else drive as usual

#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/assign/std/vector.hpp>
#include <lcmtypes/visualization.h>

#include "lidar-classifier.hpp"
#include <ConciseArgs>


using namespace std;
using namespace pcl;
using namespace boost::assign; // bring 'operator+()' into scope
using namespace Eigen;


lidar_classifier::lidar_classifier(lcm_t* lcm_, bool verbose_, int vis_history_, int estop_threshold_):
      lcm_(lcm_), verbose_(verbose_), vis_history_(vis_history_), estop_threshold_(estop_threshold_){
  botparam_ = bot_param_new_from_server(lcm_, 0);
  botframes_= bot_frames_get_global(lcm_, botparam_);

  vis_counter_ =0;

  // Unpack Config:
  pc_lcm_ = new pointcloud_lcm(lcm_);
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis(lcm_);
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4000,"Lidar Class | Pose - Null",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4001,"Lidar Class | Cloud - Null"             ,1,0, 4000,0, { -1, -1, -1} ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4002,"Lidar Class | Cloud - Body Yaw relative",1,0, 4000,0, {1.0,0.0,1.0}));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4003,"Lidar Class | Cloud - Body2Lidar"     ,1,1, 4000,1, {0.0, 0.0, 1.0} ));

  pc_vis_->obj_cfg_list.push_back( obj_cfg(4010,"Lidar Class | Pose - Laser",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4011,"Lidar Class | Cloud - Laser"         ,1,1, 4010,1, {0.0, 0.0, 1.0} ));
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4004,"Lidar Class | POSE_BODY",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4005,"Lidar Class | POSE_BODY (Yaw only)",5,1) );

  bot_core_planar_lidar_t_subscribe(lcm_, "SCAN_FREE",
      lidar_classifier::lidar_handler_aux, this);
}

void lidar_classifier::lidar_handler(const bot_core_planar_lidar_t *msg){
  int64_t pose_id=vis_counter_;//msg->utime;

  // 1. Convert the LIDAR into a point cloud:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  double maxRange(29.7), minRange(0.0);
  double validBeamAngles[] ={-10,10}; // consider everything
  convertLidar(msg->ranges, msg->nranges, msg->rad0, msg->radstep, cloud, minRange, maxRange,
      validBeamAngles[0], validBeamAngles[1]);
  
  Eigen::Isometry3d body_to_local = frames_cpp_->get_trans_with_utime( botframes_ ,  "body", "local", msg->utime);
  Eigen::Isometry3d lidar_to_local =  frames_cpp_->get_trans_with_utime( botframes_ ,  "SCAN_FREE", "local", msg->utime);

  // 2. Determine the lookahead transform - the forward direction from the pelvis ignoring pitch and roll:
  double ypr[3];
  quat_to_euler( Eigen::Quaterniond(body_to_local.rotation()) , ypr[0], ypr[1], ypr[2]);
  Matrix3d m;
  m = AngleAxisd ( ypr[0], Vector3d::UnitZ ())
                  * AngleAxisd (0 , Vector3d::UnitY ())
                  * AngleAxisd ( 0 , Vector3d::UnitX ());  
  Eigen::Isometry3d body_to_local_yaw =  Eigen::Isometry3d::Identity();
  body_to_local_yaw *= m;  
  body_to_local_yaw.translation()  << body_to_local.translation().x(), body_to_local.translation().y(), body_to_local.translation().z();    


  // 3. Convert the lidar point cloud first to the local frames_cpp_
  //    Then convert it to a frame relative to the lookahead transform
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_local (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::transformPointCloud (*cloud, *cloud_local,
      lidar_to_local.cast<float>().translation(), Eigen::Quaternionf(lidar_to_local.cast<float>().rotation()));

  Eigen::Isometry3f local_to_body_yaw = body_to_local_yaw.cast<float>().inverse();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_body_yaw (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::transformPointCloud (*cloud_local, *cloud_body_yaw,
      local_to_body_yaw.translation(), Eigen::Quaternionf(local_to_body_yaw.rotation()));
  

  // 4. Classify the returns relative to the robot's pelvis
  // Do not consider returns near the robot:
  double self_collision_front = 1.5;
  double self_collision_side  = 1.5; 
  // Do not consider returns too high or too low (including ground)
  double height_to_ground = 1.0;
  double height_to_head = 1.5; // overestimated
  // Do not consider returns too far in front or to the side
  double max_collision_front = 4.0;
  double max_collision_side = 2.0;
  
  int n_obstacle_returns =0;
  for (size_t i=0; i < cloud_body_yaw->points.size() ; i ++){
    pcl::PointXYZRGB pt = cloud_body_yaw->points[i];
    if (( fabs(pt.x) < self_collision_front ) &&  ( fabs(pt.y) < self_collision_side )) {  // if near to robot - returns from the car
      pt.r =120.0;      pt.g =120.0;      pt.b =120.0;
    }else{
      if ( (pt.z > - height_to_ground) && (pt.z < height_to_head) ){ // If less then XXm from robot in z, could be an obstacle below or above
        pt.r =0.0;        pt.g =0.0;        pt.b =255.0;
        if ( pt.x < max_collision_front ){ // within Xm in front
          if ( fabs(pt.y) < max_collision_side ){ // within 2m left and right
            pt.r =255.0;            pt.g =0.0;            pt.b =0.0;
            n_obstacle_returns++;
          }
        }
      }else{ // not near car, not in front of the car
        pt.r =0.0;        pt.g =255.0;        pt.b =0.0;
      }
    }
    cloud_body_yaw->points[i]=pt;
    cloud_local->points[i].r=pt.r;
    cloud_local->points[i].g=pt.g;
    cloud_local->points[i].b=pt.b;
  }
  
  // 5. If the number of obstacles is above a threshold then publish an estop message:
  if (n_obstacle_returns > estop_threshold_){
    cout << "Sending NAV_GOAL_ESTOP: obstacle in front: " << n_obstacle_returns << "\n"; 
    double goal_timeout=3.0;
    drc_nav_goal_timed_t msgout;
    msgout.utime = msg->utime;
    msgout.timeout = (int64_t) 1E6*goal_timeout;
    msgout.robot_name = "atlas";
    msgout.goal_pos.translation.x = 0;
    msgout.goal_pos.translation.y = 0;
    msgout.goal_pos.translation.z = 0;
    msgout.goal_pos.rotation.w = 0;
    msgout.goal_pos.rotation.x = 0;
    msgout.goal_pos.rotation.y = 0;
    msgout.goal_pos.rotation.z = 0;
    drc_nav_goal_timed_t_publish(lcm_, "NAV_GOAL_ESTOP", &msgout);
  }
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  if (vis_history_ >0){
    Eigen::Isometry3d null_pose = Eigen::Isometry3d::Identity();
    Isometry3dTime null_poseT = Isometry3dTime(pose_id, null_pose);
    pc_vis_->pose_to_lcm_from_list(4000, null_poseT);
    pc_vis_->ptcld_to_lcm_from_list(4001, *cloud_local, pose_id, pose_id);  

    bool verbose_=false;
    if(verbose_){
      Isometry3dTime body_to_localT = Isometry3dTime(pose_id, body_to_local);
      pc_vis_->pose_to_lcm_from_list(4004, body_to_localT);
      Isometry3dTime body_to_local_yawT = Isometry3dTime(pose_id, body_to_local_yaw) ;
      pc_vis_->pose_to_lcm_from_list(4005, body_to_local_yawT);

      // This output is the most useful for debugging: ////////////////////////////////////////////////
      // Lidar in the robot body frame as 0,0,0. This is what the classification is actually done on:
      pc_vis_->ptcld_to_lcm_from_list(4002, *cloud_body_yaw, pose_id, pose_id);
    
      Isometry3dTime lidar_poseT = Isometry3dTime(pose_id, lidar_to_local);
      pc_vis_->pose_to_lcm_from_list(4010, lidar_poseT);
      pc_vis_->ptcld_to_lcm_from_list(4011, *cloud, pose_id, pose_id);
    
      Eigen::Isometry3d lidar_to_body = frames_cpp_->get_trans_with_utime( botframes_ ,  "SCAN_FREE", "body", msg->utime);

      Isometry3dTime lidar_to_bodyT = Isometry3dTime(pose_id, lidar_to_body);
      Eigen::Isometry3f l2b_f = lidar_to_body.cast<float>() ;
      Eigen::Quaternionf l2b_quat(l2b_f.rotation());
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_l2b (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pcl::transformPointCloud (*cloud, *cloud_l2b,
          l2b_f.translation(), l2b_quat); // !! modifies output
      pc_vis_->ptcld_to_lcm_from_list(4003, *cloud_l2b, pose_id, pose_id);
    }  
  }
  
  vis_counter_++;
  if (vis_counter_ >=vis_history_){ // set this to 1 to only see the last return
    vis_counter_=0;
  }  
}


int main(int argc, char ** argv){
  int history=5;
  bool verbose=false;
  int estop_threshold=10;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(history, "y", "history","History of Returns to show");
  opt.add(verbose, "v", "verbose","verbose");
  opt.add(estop_threshold, "e", "estop_threshold","Number of returns to trigger an estop");
  opt.parse();
  std::cout << "history: " << history << "\n";    
  std::cout << "verbose: " << verbose << "\n";     
  std::cout << "estop_threshold: " << estop_threshold << "\n";     
  
  lcm_t * lcm;
  lcm = lcm_create(NULL);
  lidar_classifier app(lcm, verbose, history, estop_threshold);

  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}
