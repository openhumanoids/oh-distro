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
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/vtk_lib_io.h>

#include <rgbd_simulation/rgbd_primitives.hpp> // to create basic meshes
#include <pointcloud_tools/pointcloud_vis.hpp>

#include <pointcloud_tools/filter_planes.hpp>

using namespace Eigen;
using namespace std;
using namespace boost;

double rpy[] = {0,0,0};
double xyz[] = {0,0,0};

struct Primitive
{
  std::string link;
  std::string type;
  std::vector<float> params;
};


class StatePub
{
public:
  StatePub(boost::shared_ptr<lcm::LCM> &lcm_, string filename_, int pts_per_m_squared_);
  ~StatePub() {}
  boost::shared_ptr<lcm::LCM> lcm_;
  
  void doPointCloudWork();
  void doMeshWork();
  
  
  
private:
    pcl::PolygonMesh::Ptr prim_mesh_ ;
    rgbd_primitives*  prim_;
      pointcloud_vis* pc_vis_;

  
  
  void boxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
  void moveModel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
  void mirrorModel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
  
  string filename_;
  
  Isometry3dTime null_poseT_;  
  
  int pts_per_m_squared_;
  
};

StatePub::StatePub(boost::shared_ptr<lcm::LCM> &lcm_, std::string filename_, int pts_per_m_squared_):
    lcm_(lcm_), filename_(filename_), pts_per_m_squared_(pts_per_m_squared_),null_poseT_(0, Eigen::Isometry3d::Identity()){
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

  
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(2002,"Cloud - Orig"     ,7,1, 1000,0, {0,0,1}));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(2001,"Cloud - Filtered"     ,7,1, 1000,0, {1,0,1}));
  
}

void StatePub::doPointCloudWork(){
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT_);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ( filename_, *cloud_ptr) == -1){ // load the file
    PCL_ERROR ("Couldn't read pcd file\n");
    return;
  }   
  
  pc_vis_->ptcld_to_lcm_from_list(1002, *cloud_ptr, null_poseT_.utime, null_poseT_.utime);

  
  
  //boxFilter(cloud_ptr);
  moveModel(cloud_ptr);
  
  
  pc_vis_->ptcld_to_lcm_from_list(1001, *cloud_ptr, null_poseT_.utime, null_poseT_.utime);
  pcl::io::savePCDFileASCII ("filtered_output.pcd", *cloud_ptr);
}

void StatePub::doMeshWork(){
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT_);

  pcl::PolygonMesh mesh;     // (new pcl::PolygonMesh);
  pcl::io::loadPolygonFile (filename_, mesh);
  pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh (mesh));  
  
  pc_vis_->mesh_to_lcm_from_list(2002, mesh_ptr, null_poseT_.utime, null_poseT_.utime);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::fromROSMsg(mesh_ptr->cloud, *cloud_ptr);  

  // DO WORK HERE:
  //boxFilter(&newcloud);
  //moveModel(cloud_ptr);
  mirrorModel(cloud_ptr);
  
  
  pcl::toROSMsg(*cloud_ptr, mesh_ptr->cloud);
  
  pc_vis_->mesh_to_lcm_from_list(2001, mesh_ptr, null_poseT_.utime, null_poseT_.utime);

  std::cout << "mesh3\n";
  pcl::io::savePolygonFile("filtered_output.ply", *mesh_ptr);
  std::cout << "mesh4\n";

  
}


void StatePub::boxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud){

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (1.15, 1.6); //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (1, 1.6); //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);
  
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (1.035, 3.5); //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);
  
}


// TODO: provide axis about which to mirror, currently just y
void StatePub::mirrorModel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud){

  
  for(int j=0; j< cloud->points.size(); j++) {
    cloud->points[j].x = cloud->points[j].x;
    cloud->points[j].y = -cloud->points[j].y;
    cloud->points[j].z = cloud->points[j].z;
  }  
  
}


void StatePub::moveModel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud){
  Eigen::Isometry3d local_to_lidar;
  Eigen::Quaterniond quat = euler_to_quat( rpy[0]*M_PI/180.0,rpy[1]*M_PI/180.0, rpy[2]*M_PI/180.0 );
  local_to_lidar.setIdentity();
  local_to_lidar.translation()  << xyz[0], xyz[1], xyz[2];
  local_to_lidar.rotate(quat);

  pc_vis_->obj_cfg_list.push_back( obj_cfg(1005,"Pose to removed",5,0) );  
  Isometry3dTime local_to_lidarT = Isometry3dTime(0, local_to_lidar  ); 
  pc_vis_->pose_to_lcm_from_list(1005, local_to_lidarT);
    
  Eigen::Isometry3f local_to_lidar_i = local_to_lidar.cast<float>().inverse();  
  Eigen::Quaternionf quat_i  =  Eigen::Quaternionf ( local_to_lidar_i.rotation()  );
  
  pcl::transformPointCloud (*cloud, *cloud,
        local_to_lidar_i.translation(), quat_i); // !! modifies lidarcloud
}

/*
bool StatePub::readMesh(){
  
  pcl::PolygonMesh combined_mesh;     // (new pcl::PolygonMesh);
  pcl::io::loadPolygonFile ("/home/mfallon/drc/software/models/mit_gazebo_objects/mit_firehose_short/meshes/coupling_hexagon.ply", combined_mesh);
  
  // cout << "read a cloud with " << cloud->points.size() << " points\n";
  return true;
} */

std::string getTheFileExtension(const std::string& FileName)
{
    if(FileName.find_last_of(".") != std::string::npos)
        return FileName.substr(FileName.find_last_of(".")+1);
    return "";
}

int main (int argc, char ** argv){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  string filename = "/home/mfallon/drc/software/perception/trackers/data/car_simulator/vehicle_200000000.pcd";
  int pts_per_m_squared = 2000; // was 1000
  parser.add(filename, "f", "filename", "filename");
  parser.add(pts_per_m_squared, "d", "density_of_points", "Density of points (per m^2)");
  parser.add(xyz[0], "x", "x", "x [m]");
  parser.add(xyz[1], "y", "y", "y [m]");
  parser.add(xyz[2], "z", "z", "z [m]");
  parser.add(rpy[0], "roll", "roll", "Roll [rads]");
  parser.add(rpy[1], "pitch", "pitch", "Pitch [rads]");
  parser.add(rpy[2], "rot", "yaw", "Yaw [rads]");
  parser.parse();
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  StatePub app(lcm, filename, pts_per_m_squared);
  cout << "StatePub ready"<< endl;

  
  std::string file_ex= getTheFileExtension(filename);
  if ( file_ex == "pcd"){
    app.doPointCloudWork();
  }else if(file_ex == "ply") {
    app.doMeshWork();
  }
  
  return 0;
}
