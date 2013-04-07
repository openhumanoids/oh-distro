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
    drc::affordance_plus_t getAffordance(std::vector<double> &xyzrpy, int uid);
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
    
    
drc::affordance_plus_t Pass::getAffordance(std::vector<double> &xyzrpy, int uid){ 
  drc::affordance_plus_t p;
  
  drc::affordance_t a;
  a.utime =0;
  a.map_id =0;
  a.uid =uid;
  a.otdf_type ="mesh";
  a.aff_store_control = drc::affordance_t::NEW;

  a.param_names.push_back("x");
  a.params.push_back(xyzrpy[0]);
  a.param_names.push_back("y");
  a.params.push_back(xyzrpy[1]);
  a.param_names.push_back("z");
  a.params.push_back(xyzrpy[2]);

  a.param_names.push_back("roll");
  a.params.push_back( xyzrpy[3]);
  a.param_names.push_back("pitch");
  a.params.push_back( xyzrpy[4]);
  a.param_names.push_back("yaw");
  a.params.push_back( xyzrpy[5] );

/*  a.param_names.push_back("radius");
  a.params.push_back(0.020000);
  a.param_names.push_back("length");
  a.params.push_back(0.13); */
  a.param_names.push_back("mass");
  a.params.push_back(1.0); // unknown
  a.nparams =a.params.size();
  a.nstates =0;
  
  p.aff =a;
  
  std::vector< std::vector< float > > points;
  std::vector< std::vector< int > > triangles;
  if (uid==0){
    getMeshAsLists("/home/mfallon/drc/software/perception/trackers/data_non_in_svn/drill_clouds/affordance_version/drill_mfallonio.ply", points, triangles);
  }else if(uid==1){
    getCloudAsLists("/home/mfallon/drc/software/perception/trackers/data_non_in_svn/drill_clouds/affordance_version/drill.pcd", points, triangles );
  }else if(uid==2){
    getCloudAsLists("/home/mfallon/drc/software/perception/trackers/data_non_in_svn/drill_clouds/affordance_version/drill_sensed_smoothed.pcd", points, triangles );
  }else{
    getMeshAsLists("/home/mfallon/drc/software/perception/trackers/data_non_in_svn/drill_clouds/affordance_version/drill_sensed_smoothed.ply", points, triangles);
  }    
  p.points =points;
  p.npoints=points.size();
  p.triangles = triangles;
  p.ntriangles =p.triangles.size();
  
  
  return p;
}

void Pass::doDemo(){
  std::vector<double> xyzrpy = {0 , 0 , 0, 0. , 0 , 1.571};
  drc::affordance_plus_t a0 = getAffordance(xyzrpy, 0);
  std::vector<double> xyzrpy1 = {0.5 , 0. , 0., 0., 0., 1.571};
  drc::affordance_plus_t a1 = getAffordance(xyzrpy1, 1);
  std::vector<double> xyzrpy2 = {1. , 0. , 0., 0., 0. , 1.571};
  drc::affordance_plus_t a2 = getAffordance(xyzrpy2, 2);
  std::vector<double> xyzrpy3 = {1.5 , 0 , 0., 0, 0 , 1.571};
  drc::affordance_plus_t a3 = getAffordance(xyzrpy3, 3);

  drc::affordance_plus_collection_t aff_plus_coll;
  aff_plus_coll.name  = "Map Name";
  aff_plus_coll.utime = 0;
  aff_plus_coll.map_id =0;
  
  aff_plus_coll.affs_plus.push_back( a0);
  aff_plus_coll.affs_plus.push_back( a1);
  aff_plus_coll.affs_plus.push_back( a2);
  aff_plus_coll.affs_plus.push_back( a3);
  aff_plus_coll.naffs =aff_plus_coll.affs_plus.size();
  lcm_->publish("AFFORDANCE_PLUS_COLLECTION",&aff_plus_coll);
  
  /*
  drc::affordance_collection_t aff_coll;
  aff_coll.name  = "Map Name";
  aff_coll.utime = 0;
  aff_coll.map_id =0;
  
  aff_coll.affs.push_back( a0.aff);
  aff_coll.affs.push_back( a1.aff);
  aff_coll.affs.push_back( a2.aff);
  aff_coll.affs.push_back( a3.aff);
  aff_coll.naffs =aff_coll.affs.size();
  lcm_->publish("AFFORDANCE_COLLECTION",&aff_coll);  */
  
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
