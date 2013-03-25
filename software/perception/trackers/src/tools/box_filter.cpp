//measured:
//- clip cloud 2m^3 in front
//mesh:
//x,y,z,nx,ny,nz,[convex hull], with 50points


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

double yaw=0;
double x=0;
double y=5;
double z=0;

struct Primitive
{
  std::string link;
  std::string type;
  std::vector<float> params;
};


class StatePub
{
public:
  StatePub(boost::shared_ptr<lcm::LCM> &lcm_, string pcd_filename, string urdf_file, int pts_per_m_squared_);
  ~StatePub() {}
  boost::shared_ptr<lcm::LCM> lcm_;
  
private:
    pcl::PolygonMesh::Ptr prim_mesh_ ;
    rgbd_primitives*  prim_;
      pointcloud_vis* pc_vis_;

  
  bool readPCD();
  
  void boxFilter();
  void moveCloud();
  
  string pcd_filename_;
  
  pcl::PointCloud<PointXYZRGB>::Ptr _cloud;    
  Isometry3dTime null_poseT_;  
  
  int pts_per_m_squared_;
  
};

StatePub::StatePub(boost::shared_ptr<lcm::LCM> &lcm_, std::string pcd_filename_, string urdf_file, int pts_per_m_squared_):
    lcm_(lcm_), pcd_filename_(pcd_filename_), pts_per_m_squared_(pts_per_m_squared_),null_poseT_(0, Eigen::Isometry3d::Identity()){
  prim_ = new rgbd_primitives();
  
  pc_vis_ = new pointcloud_vis(lcm_->getUnderlyingLCM());
  prim_ = new rgbd_primitives();

  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  float colors_b[] ={1.0,0.0,1.0};
  std::vector<float> colors_v;
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(349995,"Vehicle Tracker - Null",5,1) );
  // 7 = full | 2 wireframe

  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Null",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1002,"Cloud - Orig"     ,1,1, 1000,1, {0,0,1}));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Cloud - Filtered"     ,1,1, 1000,1, {1,0,1}));
  
      
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  _cloud = cloud_ptr ;  
  readPCD();

  pc_vis_->pose_to_lcm_from_list(1000, null_poseT_);
  pc_vis_->ptcld_to_lcm_from_list(1002, *_cloud, null_poseT_.utime, null_poseT_.utime);


//  boxFilter();
  moveCloud();

  pc_vis_->ptcld_to_lcm_from_list(1001, *_cloud, null_poseT_.utime, null_poseT_.utime);

  pcl::io::savePCDFileASCII ("filtered_pcd.pcd", *_cloud);


}


void StatePub::boxFilter(){

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (_cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (1.15, 1.6); //pass.setFilterLimitsNegative (true);
  pass.filter (*_cloud);

  pass.setInputCloud (_cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (1, 1.6); //pass.setFilterLimitsNegative (true);
  pass.filter (*_cloud);
  
  pass.setInputCloud (_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (1.035, 3.5); //pass.setFilterLimitsNegative (true);
  pass.filter (*_cloud);
  
}


void StatePub::moveCloud(){
    Eigen::Isometry3f local_to_lidar;
    Eigen::Quaternionf quat = Eigen::Quaternionf(1,0,0,0);
    local_to_lidar.setIdentity();
    local_to_lidar.translation()  << 1.275, 1.3, 1.16277;
    local_to_lidar.rotate(quat);

    
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1005,"[BoxFilter] Pose to removed",5,0) );  
  Isometry3dTime local_to_lidarT = Isometry3dTime(0, local_to_lidar.cast<double>()  ); 
  pc_vis_->pose_to_lcm_from_list(1005, local_to_lidarT);
    
    Eigen::Isometry3f local_to_lidar_i = local_to_lidar.inverse();  
    Eigen::Quaternionf quat_i  =  Eigen::Quaternionf ( local_to_lidar_i.rotation()  );
  
    pcl::transformPointCloud (*_cloud, *_cloud,
        local_to_lidar_i.translation(), quat_i); // !! modifies lidar_cloud

  
}


bool StatePub::readPCD(){
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ( pcd_filename_, *_cloud) == -1){ // load the file
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (false);
  } 
  cout << "read a cloud with " << _cloud->points.size() << " points\n";
  return true;
}


int main (int argc, char ** argv){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  string filename = "/home/mfallon/drc/software/perception/trackers/data/car_simulator/chassis.txt";
  string pcd_filename = "/home/mfallon/drc/software/perception/trackers/data/car_simulator/vehicle_200000000.pcd";
  int pts_per_m_squared = 2000; // was 1000
  parser.add(filename, "f", "filename", "filename");
  parser.add(pcd_filename, "p", "pcd_filename", "pcd_filename");
  parser.add(pts_per_m_squared, "d", "density_of_points", "Density of points (per m^2)");
  parser.add(x, "x", "x", "x [m]");
  parser.add(y, "y", "y", "y [m]");
  parser.add(z, "z", "z", "z [m]");
  parser.add(yaw, "r", "rotation", "Yaw [rads]");
  parser.parse();
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  StatePub app(lcm, pcd_filename, filename, pts_per_m_squared);
  cout << "StatePub ready"<< endl;
//  while(0 == lcm->handle());
  return 0;
}
