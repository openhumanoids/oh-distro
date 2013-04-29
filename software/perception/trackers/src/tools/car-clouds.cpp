
// pcl_voxel_grid cloud1234.pcd cloud1234_down.pcd -leaf 0.01,0.01,0.01


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/bot_core.hpp"

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ConciseArgs>


#include <pcl/filters/passthrough.h>


#include <rgbd_simulation/rgbd_primitives.hpp> // to create basic meshes
#include <pointcloud_tools/pointcloud_vis.hpp>

#include <pointcloud_tools/filter_planes.hpp>

using namespace Eigen;
using namespace std;
using namespace boost;



class StatePub
{
public:
  StatePub(boost::shared_ptr<lcm::LCM> &lcm_, string pcd_filename_a, string pcd_filename_b);
  ~StatePub() {}
  boost::shared_ptr<lcm::LCM> lcm_;
  
private:
    pcl::PolygonMesh::Ptr prim_mesh_ ;
    rgbd_primitives*  prim_;
      pointcloud_vis* pc_vis_;

  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPCD(std::string filename);
  
  void boxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  void moveCloud();
    
  Isometry3dTime null_poseT_;  
  
};

StatePub::StatePub(boost::shared_ptr<lcm::LCM> &lcm_, std::string pcd_filename_a, string pcd_filename_b):
    lcm_(lcm_), null_poseT_(0, Eigen::Isometry3d::Identity()){
  
  pc_vis_ = new pointcloud_vis(lcm_->getUnderlyingLCM());

  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  float colors_b[] ={1.0,0.0,1.0};
  std::vector<float> colors_v;
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(349995,"Vehicle Tracker - Null",5,1) );
  // 7 = full | 2 wireframe

  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Null",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Cloud - A"     ,1,1, 1000,1, {0,0,1}));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1002,"Cloud - B"     ,1,1, 1000,1, {1,0,0}));

  
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1010,"Cloud - Filtered"     ,1,1, 1000,1, {1,0,1}));
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_a (new pcl::PointCloud<pcl::PointXYZRGB> ());
  cloud_a = readPCD(pcd_filename_a);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_b (new pcl::PointCloud<pcl::PointXYZRGB> ());
  cloud_b = readPCD(pcd_filename_b);

  pc_vis_->pose_to_lcm_from_list(1000, null_poseT_);
  pc_vis_->ptcld_to_lcm_from_list(1001, *cloud_a, null_poseT_.utime, null_poseT_.utime);
  pc_vis_->ptcld_to_lcm_from_list(1002, *cloud_b, null_poseT_.utime, null_poseT_.utime);

  *cloud_a += *cloud_b;

  boxFilter(cloud_a);
  //moveCloud();

  pc_vis_->ptcld_to_lcm_from_list(1010, *cloud_a, null_poseT_.utime, null_poseT_.utime);

  pcl::io::savePCDFileASCII ("filtered_pcd.pcd", *cloud_a);


}


void StatePub::boxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-1.5, 1.5); //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-1.5, 1.5); //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);
  
//  pass.setInputCloud (cloud);
//  pass.setFilterFieldName ("z");
//  pass.setFilterLimits (1.035, 3.5); //pass.setFilterLimitsNegative (true);
//  pass.filter (*cloud);

}


void StatePub::moveCloud(){
  /*
    Eigen::Isometry3f local_to_lidar;
    Eigen::Quaternionf quat = euler_to_quat_f( yaw*M_PI/180.0 ,0,0);
    //Eigen::Quaternionf quat = Eigen::Quaternionf(1,0,0,0);
    local_to_lidar.setIdentity();
//    local_to_lidar.translation()  << 1.2575, 1.3, 1.16;
    local_to_lidar.translation()  << x,y,z;//0,0,-0.63;
    local_to_lidar.rotate(quat);

    
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1005,"[BoxFilter] Pose to removed",5,0) );  
  Isometry3dTime local_to_lidarT = Isometry3dTime(0, local_to_lidar.cast<double>()  ); 
  pc_vis_->pose_to_lcm_from_list(1005, local_to_lidarT);
    
    Eigen::Isometry3f local_to_lidar_i = local_to_lidar.inverse();  
    Eigen::Quaternionf quat_i  =  Eigen::Quaternionf ( local_to_lidar_i.rotation()  );
  
    pcl::transformPointCloud (*_cloud, *_cloud,
        local_to_lidar_i.translation(), quat_i); // !! modifies lidar_cloud

  */
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr StatePub::readPCD(std::string filename){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ( filename, *cloud) == -1){ // load the file
    cout << "couldnt read " << filename << "\n";
    exit(-1);
  } 
  cout << "read a cloud with " << cloud->points.size() << " points\n";
  return cloud;
}


int main (int argc, char ** argv){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  string pcd_filename_a = "/home/mfallon/drc/software/perception/trackers/data/car_simulator/vehicle_200000000.pcd";
  string pcd_filename_b = "/home/mfallon/drc/software/perception/trackers/data/car_simulator/chassis.txt";
  parser.add(pcd_filename_a, "a", "pcd_filename_a", "pcd_filename_a");
  parser.add(pcd_filename_b, "b", "pcd_filename_b", "pcd_filename_b");
  parser.parse();
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  StatePub app(lcm, pcd_filename_a, pcd_filename_b);
  cout << "StatePub ready"<< endl;
//  while(0 == lcm->handle());
  return 0;
}
