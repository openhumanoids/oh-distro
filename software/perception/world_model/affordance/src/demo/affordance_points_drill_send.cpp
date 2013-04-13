#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>


// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include "pcl/PolygonMesh.h"
#include <pcl/common/transforms.h>


#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds and write mesh


using namespace pcl;
using namespace pcl::io;

using namespace std;
class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~Pass(){
    }    
    

    void doDemo();
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    drc::affordance_plus_t getAffordancePlus(std::string filename, std::vector<double> &xyzrpy, int uid);
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_): lcm_(lcm_){
  cout << "Finished setting up\n";
}


Eigen::Quaterniond euler_to_quat(double yaw, double pitch, double roll) {
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}



void scaleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr){
  for (size_t i=0; i < cloud_ptr->points.size(); i++){ 
    pcl::PointXYZRGB pt = cloud_ptr->points[i];
    pt.x/=1000;
    pt.y/=1000;
    pt.z/=1000;
    
    cloud_ptr->points[i] = pt;
  }  
}


void moveCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr){
    Eigen::Isometry3d local_to_lidar;
    local_to_lidar.setIdentity();
    local_to_lidar.translation()  << 34,0,-230;
    Eigen::Quaterniond quat = euler_to_quat(-M_PI/2,0,0);
    local_to_lidar.rotate(quat);

    pcl::transformPointCloud (*cloud_ptr, *cloud_ptr,
        local_to_lidar.translation().cast<float>(), quat.cast<float>()); // !! modifies lidar_cloud

  
}



bool getMeshAsLists(std::string filename,
                  std::vector< std::vector< float > > &points, 
                  std::vector< std::vector< int > > &triangles){ 

  pcl::PolygonMesh combined_mesh;     // (new pcl::PolygonMesh);
  pcl::io::loadPolygonFile (filename, combined_mesh);
  pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh (combined_mesh));

  pcl::PointCloud<pcl::PointXYZRGB> newcloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::fromROSMsg(mesh->cloud, *cloud_ptr );  

  /*
  moveCloud(cloud_ptr);
  scaleCloud(cloud_ptr);
  
  
  pcl::toROSMsg (*cloud_ptr, mesh->cloud);
  pcl::io::savePolygonFile("drill_pclio.ply", *mesh);
  savePLYFile(mesh, "drill_mfallonio.ply");
  */
  
  for (size_t i=0; i < cloud_ptr->points.size(); i++){ 
    Eigen::Vector4f tmp = cloud_ptr->points[i].getVector4fMap();
    std::vector<float> point;
    point.push_back ( (float) tmp(0)  );
    point.push_back ( (float) tmp(1)  );
    point.push_back ( (float) tmp(2)  );
    points.push_back(point);
  }
  
  for(size_t i=0; i<  mesh->polygons.size (); i++){ // each triangle/polygon
    pcl::Vertices poly = mesh->polygons[i];//[i];
    if (poly.vertices.size() > 3){
      cout << "poly " << i << " is of size " << poly.vertices.size() << " lcm cannot support this\n";
      exit(-1); 
    }
    vector<int> triangle(poly.vertices.begin(), poly.vertices.end());
    triangles.push_back(triangle );
  }
  cout << "Read a mesh with " << points.size() << " points and " << triangles.size() << " triangles\n";
  return true;
}    
    
    
bool getCloudAsLists(std::string filename,
                  std::vector< std::vector< float > > &points, 
                  std::vector< std::vector< int > > &triangles){    
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ( filename, *cloud_ptr) == -1){ // load the file
      cout << "Couldn't read " << filename << ", quitting\n";
      exit(-1);
    } 
    
  for (size_t i=0; i < cloud_ptr->points.size(); i++){ 
    Eigen::Vector4f tmp = cloud_ptr->points[i].getVector4fMap();
    std::vector<float> point;
    point.push_back ( (float) tmp(0)  );
    point.push_back ( (float) tmp(1)  );
    point.push_back ( (float) tmp(2)  );
    points.push_back(point);
  }
      
  cout << "Read a point cloud with " << points.size() << " points\n";
    
}
    
    
drc::affordance_plus_t Pass::getAffordancePlus(std::string filename, std::vector<double> &xyzrpy, int uid){ 
  drc::affordance_plus_t p;
  
  drc::affordance_t a;
  a.utime =0;
  a.map_id =0;
  a.uid =uid;
  a.otdf_type ="mesh";
  a.aff_store_control = drc::affordance_t::NEW;

/*  a.param_names.push_back("radius");
  a.params.push_back(0.020000);
  a.param_names.push_back("length");
  a.params.push_back(0.13); */
  a.param_names.push_back("mass");
  a.params.push_back(1.0); // unknown
  a.param_names.push_back("filename");
  a.params.push_back( -1 ); //add string here
  
  a.nparams =a.params.size();
  a.nstates =0;

  a.origin_xyz[0]=xyzrpy[0]; a.origin_xyz[1]=xyzrpy[1]; a.origin_xyz[2]=xyzrpy[2]; 
  a.origin_rpy[0]=xyzrpy[3]; a.origin_rpy[1]=xyzrpy[4]; a.origin_rpy[2]=xyzrpy[5]; 
  
  a.bounding_xyz[0]=xyzrpy[0]; a.bounding_xyz[1]=xyzrpy[1]; a.bounding_xyz[2]=xyzrpy[2]; 
  a.bounding_rpy[0]=xyzrpy[3]; a.bounding_rpy[1]=xyzrpy[4]; a.bounding_rpy[2]=xyzrpy[5]; 
  //a.bounding_rpy = { xyzrpy[3], xyzrpy[4], xyzrpy[5]};
  // a.bounding_lwh = { 0.3, 0.36, 0.4};
  
  p.aff = a;
  
  std::vector< std::vector< float > > points;
  std::vector< std::vector< int > > triangles;
  

  
  int length = filename.length() ;
  std::string extension = filename.substr (length-3,3);
  cout << extension<<" q\n";
  
  if (extension=="ply"){
    getMeshAsLists(filename , points, triangles);
  }else if(extension=="pcd"){
    getCloudAsLists(filename , points, triangles);
  }
    
  p.points =points;
  p.npoints=points.size(); 
  p.triangles = triangles;
  p.ntriangles =p.triangles.size();
  
  return p;
}




void Pass::doDemo(){
  char* pHome;
  pHome = getenv("HOME");  
  string home = string(pHome);
  cout << home << "\n";  
  
  
  int uid = 12;
  std::vector<double> xyzrpy = {1.27 , 1.30 , 1.16, 0. , 0 , 0};
  //string filename = string(home+ "/drc/software/models/mit_gazebo_models/mesh_otdf/meshes/drill_mfallonio.ply");
  //string filename = string(home+ "/drc/software/models/mit_gazebo_models/mesh_otdf/meshes/drill.pcd");
  //string filename = string(home+ "/drc/software/models/mit_gazebo_models/mesh_otdf/meshes/drill_sensed_smoothed.pcd");
  string filename = string(home+ "/drc/software/models/mit_gazebo_models/mesh_otdf/meshes/drill_sensed_smoothed.ply");
  drc::affordance_plus_t a0 = getAffordancePlus(filename, xyzrpy, uid);
  a0.aff.bounding_lwh[0]=0.36;       a0.aff.bounding_lwh[1]=0.33;      a0.aff.bounding_lwh[2]=0.3; 
  lcm_->publish("AFFORDANCE_FIT",&a0);


  uid = 13;
  xyzrpy = {-2.85 , -2.55 , 0.64 , 0. , 0 , M_PI};
  filename = string(home+ "/drc/software/models/mit_gazebo_models/vehicle_otdf/meshes/vehicle.pcd");
  drc::affordance_plus_t a1 = getAffordancePlus(filename, xyzrpy, uid);
  a1.aff.bounding_lwh[0]=2.8;       a1.aff.bounding_lwh[1]=1.6;      a1.aff.bounding_lwh[2]=1.7; 
  a1.aff.otdf_type = "car";
  lcm_->publish("AFFORDANCE_FIT",&a1);
  
  

/*  
  drc::affordance_plus_collection_t aplus_coll;
  aplus_coll.affs_plus.push_back( a0 );
  aplus_coll.affs_plus.push_back( a1 );
  aplus_coll.naffs = aplus_coll.affs_plus.size();
  while(1==1){
    lcm_->publish("AFFORDANCE_PLUS_COLLECTION",&aplus_coll);
    sleep(1);
  }
*/  
}

int main( int argc, char** argv ){
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "Demo Ready" << endl << "============================" << endl;
  app.doDemo();
  return 0;
}
