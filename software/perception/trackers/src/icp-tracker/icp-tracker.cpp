#include "icp-tracker.hpp"
#include <iostream>


#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h> //for computePointNormal


#include <rgbd_simulation/rgbd_primitives.hpp> // to create basic meshes


using namespace std;

ICPTracker::ICPTracker(boost::shared_ptr<lcm::LCM> &lcm_, int verbose_lcm_): 
                               lcm_(lcm_), verbose_lcm_(verbose_lcm_),
                               null_poseT_(0, Eigen::Isometry3d::Identity()){

  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );

  pc_vis_->obj_cfg_list.push_back( obj_cfg(771000,"[ICP] Pose - Null",5,0) );

  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(771001,"[ICP] New Cloud within B Box"     ,1,1, 771000,1, {0,1,1}));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(771002,"[ICP] Prev Cloud (at Null)"     ,1,1, 771000,1, {1,0,0}));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(771003,"[ICP] New Cloud (at Null)"     ,1,1, 771000,1, {0,0.6,0}));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(771004,"[ICP] Prev Cloud Aligned (at Null)"     ,1,1, 771000,1, {0,0,1}));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(771005,"[ICP] Bounding Box"     ,4,1, 771000,1, {0,1,1}));

  pc_vis_->obj_cfg_list.push_back( obj_cfg(771010,"[ICP] Pose - New",5,0) );
  
}


void ICPTracker::setBoundingBox( Eigen::Vector3f  boundbox_lower_left_in, Eigen::Vector3f  boundbox_upper_right_in,
                  Eigen::Isometry3f  boundingbox_pose_in ){
  boundbox_lower_left_=boundbox_lower_left_in;
  boundbox_upper_right_=boundbox_upper_right_in;
  boundingbox_pose_ = boundingbox_pose_in;
}


//std::vector<float> ICPTracker::ICPTracker(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts, uint8_t* img_data,
bool ICPTracker::doICPTracker(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &previous_cloud, 
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_cloud, Eigen::Isometry3d previous_pose){

  // Apply a bounding box relative to the previous pose:
  boundingBoxFilter(new_cloud, previous_pose.cast<float>() ) ;
  cout << "New to be aligned cloud contains: " << new_cloud->points.size() << "\n";
  if (verbose_lcm_>0){
    pc_vis_->pose_to_lcm_from_list(771000, null_poseT_);
    pc_vis_->ptcld_to_lcm_from_list(771001, *new_cloud, null_poseT_.utime, null_poseT_.utime);
  }

  // 3. Calculate ICP
  // First remove the offset of the affordance pose so that the TF will be relative to that pose:
  removePoseOffset( previous_cloud, new_cloud, previous_pose.inverse().cast<float>() );
  Eigen::Matrix4f tf_previous_to_new;
  double score = 100;
  bool converged = doICP(previous_cloud, new_cloud, tf_previous_to_new, &score);
  new_pose_=  previous_pose * tf_previous_to_new.cast<double>() ;

  if(verbose_lcm_>=1){
    pc_vis_->pose_to_lcm_from_list(771000, null_poseT_);
    pc_vis_->ptcld_to_lcm_from_list(771004, *previous_cloud, null_poseT_.utime, null_poseT_.utime); // Previous cloud now aligned with curren scene
    Isometry3dTime new_poseT = Isometry3dTime(0, new_pose_); 
    pc_vis_->pose_to_lcm_from_list(771010, new_poseT);
  }
  return converged;
}



void ICPTracker::removePoseOffset(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &previous_cloud, 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_cloud, Eigen::Isometry3f previous_pose ){

  Eigen::Quaternionf pose_quat(previous_pose.rotation());
  
  pcl::transformPointCloud (*new_cloud, *new_cloud,
        previous_pose.translation(), pose_quat); // !! modifies lidar_cloud

  if(verbose_lcm_ >= 1){
    pc_vis_->pose_to_lcm_from_list(771000, null_poseT_);
    pc_vis_->ptcld_to_lcm_from_list(771002, *previous_cloud, null_poseT_.utime, null_poseT_.utime);
    pc_vis_->ptcld_to_lcm_from_list(771003, *new_cloud, null_poseT_.utime, null_poseT_.utime);
  }
}


bool ICPTracker::doICP( pcl::PointCloud<pcl::PointXYZRGB>::Ptr &previous_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_cloud, 
                        Eigen::Matrix4f & tf_previous_to_new, double *score){
  IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
  icp.setInputTarget( previous_cloud );
  icp.setInputCloud( new_cloud );
  //params:
  // default is 10
  // for car 50 worked well
  icp.setMaximumIterations (50);
  //try with a crappy max distance and then try with a smaller one to get a better alignment 
  icp.setMaxCorrespondenceDistance(0.05);
  //icp.setTransformationEpsilon (1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  //icp.setEuclideanFitnessEpsilon (1);

  PointCloud<PointXYZRGB>::Ptr downsampled_output (new PointCloud<PointXYZRGB>);
  icp.align(*downsampled_output);

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;

  *score = icp.getFitnessScore();

  if(icp.getFitnessScore() > 0.01){
    fprintf(stderr, "No convergence\n");
  }
  //smaller the score the better 

  // Apply inverted transform to transform original cloud onto the new pose:
  tf_previous_to_new = icp.getFinalTransformation().inverse();
  pcl::transformPointCloud(*previous_cloud, *previous_cloud, icp.getFinalTransformation());

  return icp.hasConverged();
}



void ICPTracker::boundingBoxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, Eigen::Isometry3f pose){
  if (boundbox_lower_left_(0) == boundbox_upper_right_(0) ){
    std::cout <<  "boundingBoxFilter() box is degenerate [0]\n";
    exit(-1);
  }
  if (boundbox_lower_left_(1) == boundbox_upper_right_(1) ){
    std::cout <<  "boundingBoxFilter() box is degenerate [1]\n";
    exit(-1);
  }  
  if (boundbox_lower_left_(2) == boundbox_upper_right_(2) ){
    std::cout <<  "boundingBoxFilter() box is degenerate [2]\n";
    exit(-1);
  }
  
  // Determine the bounding box in the world frame
  Eigen::Isometry3f bb_pose_world =  pose* boundingbox_pose_;
  
  Eigen::Isometry3f bb_pose_world_i = bb_pose_world.inverse();
  Eigen::Quaternionf quat_i(bb_pose_world_i.rotation());
  pcl::transformPointCloud (*cloud, *cloud,
        bb_pose_world_i.translation(), quat_i); // !! modifies lidar_cloud

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (boundbox_lower_left_(0), boundbox_upper_right_(0) ); //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (boundbox_lower_left_(1), boundbox_upper_right_(1) ); //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);
  
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits ( boundbox_lower_left_(2), boundbox_upper_right_(2)); //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);


  Eigen::Quaternionf quat(bb_pose_world.rotation());
  pcl::transformPointCloud (*cloud, *cloud,
        bb_pose_world.translation(), quat); // !! modifies lidar_cloud
}


void ICPTracker::drawBoundingBox(Eigen::Isometry3f pose){
  cout << "use getBoundingBoxCloud() in affordance utils instead ===========\n";
  return;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr bb_pts (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointXYZRGB pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8;
  pt1.x = boundbox_lower_left_(0);       pt1.y = boundbox_lower_left_(1);    pt1.z = boundbox_lower_left_(2);  
  pt2.x = boundbox_lower_left_(0);       pt2.y = boundbox_lower_left_(1);    pt2.z = boundbox_upper_right_(2);  

  pt3.x = boundbox_lower_left_(0);       pt3.y = boundbox_upper_right_(1);    pt3.z = boundbox_lower_left_(2);  
  pt4.x = boundbox_lower_left_(0);       pt4.y = boundbox_upper_right_(1);    pt4.z = boundbox_upper_right_(2);  

  pt5.x = boundbox_upper_right_(0);       pt5.y = boundbox_lower_left_(1);    pt5.z = boundbox_lower_left_(2);  
  pt6.x = boundbox_upper_right_(0);       pt6.y = boundbox_lower_left_(1);    pt6.z = boundbox_upper_right_(2);  

  pt7.x = boundbox_upper_right_(0);       pt7.y = boundbox_upper_right_(1);    pt7.z = boundbox_lower_left_(2);  
  pt8.x = boundbox_upper_right_(0);       pt8.y = boundbox_upper_right_(1);    pt8.z = boundbox_upper_right_(2); 

  // z-dir
  bb_pts->points.push_back(pt1);      bb_pts->points.push_back(pt2);    
  bb_pts->points.push_back(pt3);      bb_pts->points.push_back(pt4);    
  bb_pts->points.push_back(pt5);  bb_pts->points.push_back(pt6);
  bb_pts->points.push_back(pt7);  bb_pts->points.push_back(pt8);
  // x-dir
  bb_pts->points.push_back(pt1);  bb_pts->points.push_back(pt5);   
  bb_pts->points.push_back(pt2);  bb_pts->points.push_back(pt6);    
  bb_pts->points.push_back(pt3);  bb_pts->points.push_back(pt7);    
  bb_pts->points.push_back(pt4);  bb_pts->points.push_back(pt8);    
  // y-dir
  bb_pts->points.push_back(pt1);  bb_pts->points.push_back(pt3);   
  bb_pts->points.push_back(pt2);  bb_pts->points.push_back(pt4);    
  bb_pts->points.push_back(pt5);  bb_pts->points.push_back(pt7);    
  bb_pts->points.push_back(pt6);  bb_pts->points.push_back(pt8);    

  Eigen::Quaternionf quat(pose.rotation());
  pcl::transformPointCloud (*bb_pts, *bb_pts,
        pose.translation(), quat); // !! modifies lidar_cloud

  pc_vis_->pose_to_lcm_from_list(771000, null_poseT_);
  pc_vis_->ptcld_to_lcm_from_list(771005, *bb_pts, null_poseT_.utime, null_poseT_.utime);
}
