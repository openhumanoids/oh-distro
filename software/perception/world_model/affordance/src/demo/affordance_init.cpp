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
#include <ConciseArgs>

#include <affordance/AffordanceUtils.hpp>


using namespace pcl;
using namespace pcl::io;

using namespace std;
class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~Pass(){
    }    
    

    void doDemo(int which_publish);
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    drc::affordance_plus_t getCarAffordancePlus(std::string filename, std::vector<double> &xyzrpy, int uid);
    drc::affordance_plus_t getDynamicMeshAffordancePlus(std::string filename, std::vector<double> &xyzrpy, int uid);
    drc::affordance_plus_t getDynamicMeshCylinderAffordancePlus(std::string filename, std::vector<double> &xyzrpy, int uid);
    
    AffordanceUtils affutils;
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


drc::affordance_plus_t Pass::getDynamicMeshAffordancePlus(std::string filename, std::vector<double> &xyzrpy, int uid){ 
  drc::affordance_plus_t p;
  
  drc::affordance_t a;
  a.utime =0;
  a.map_id =0;
  a.uid =uid;
  a.otdf_type ="dynamic_mesh";
  a.aff_store_control = drc::affordance_t::NEW;

  a.nparams =a.params.size();
  a.nstates =0;

  a.origin_xyz[0]=xyzrpy[0]; a.origin_xyz[1]=xyzrpy[1]; a.origin_xyz[2]=xyzrpy[2]; 
  a.origin_rpy[0]=xyzrpy[3]; a.origin_rpy[1]=xyzrpy[4]; a.origin_rpy[2]=xyzrpy[5]; 
  
  a.bounding_xyz[0]=0.0; a.bounding_xyz[1]=0; a.bounding_xyz[2]=0; 
  a.bounding_rpy[0]=0.0; a.bounding_rpy[1]=0.0; a.bounding_rpy[2]=0.0;   

  p.aff = a;
  
  std::vector< std::vector< float > > points;
  std::vector< std::vector< int > > triangles;
  affutils.getModelAsLists(filename, points, triangles);
  p.points =points;
  p.npoints=points.size(); 
  p.triangles = triangles;
  p.ntriangles =p.triangles.size();
  
  return p;
}

drc::affordance_plus_t Pass::getDynamicMeshCylinderAffordancePlus(std::string filename, std::vector<double> &xyzrpy, int uid){ 
  drc::affordance_plus_t p;
  
  drc::affordance_t a;
  a.utime =0;
  a.map_id =0;
  a.uid =uid;
  a.otdf_type ="dynamic_mesh_w_1_cylinder";
  a.aff_store_control = drc::affordance_t::NEW;
  
  a.params.push_back(0.1  ); a.param_names.push_back("length");
  a.params.push_back(0.03  ); a.param_names.push_back("radius");
  a.params.push_back(2.0  ); a.param_names.push_back("mass");
  a.params.push_back(0.0  ); a.param_names.push_back("pitch_offset");
  a.params.push_back(0.0  ); a.param_names.push_back("roll_offset");
  a.params.push_back( 1.571); a.param_names.push_back("yaw_offset");
  a.params.push_back( 0.00); a.param_names.push_back("x_offset");
  a.params.push_back(-0.005); a.param_names.push_back("y_offset");
  a.params.push_back( -0.02); a.param_names.push_back("z_offset");
  a.nparams =a.params.size();
  a.nstates =0;

  a.origin_xyz[0]=xyzrpy[0]; a.origin_xyz[1]=xyzrpy[1]; a.origin_xyz[2]=xyzrpy[2]; 
  a.origin_rpy[0]=xyzrpy[3]; a.origin_rpy[1]=xyzrpy[4]; a.origin_rpy[2]=xyzrpy[5]; 
  
  a.bounding_xyz[0]=0.0; a.bounding_xyz[1]=0; a.bounding_xyz[2]=0; 
  a.bounding_rpy[0]=0.0; a.bounding_rpy[1]=0.0; a.bounding_rpy[2]=0.0;   
 
  p.aff = a;
  
  std::vector< std::vector< float > > points;
  std::vector< std::vector< int > > triangles;
  affutils.getModelAsLists(filename, points, triangles);
  p.points =points;
  p.npoints=points.size(); 
  p.triangles = triangles;
  p.ntriangles =p.triangles.size();
  
  return p;
}


    
drc::affordance_plus_t Pass::getCarAffordancePlus(std::string filename, std::vector<double> &xyzrpy, int uid){ 
  drc::affordance_plus_t p;
  
  drc::affordance_t a;
  a.utime =0;
  a.map_id =0;
  a.uid =uid;
  a.otdf_type ="car";
  a.aff_store_control = drc::affordance_t::NEW;

  a.nparams =0;
  a.nstates =0;

  a.origin_xyz[0]=xyzrpy[0]; a.origin_xyz[1]=xyzrpy[1]; a.origin_xyz[2]=xyzrpy[2]; 
  a.origin_rpy[0]=xyzrpy[3]; a.origin_rpy[1]=xyzrpy[4]; a.origin_rpy[2]=xyzrpy[5]; 
  
  a.bounding_xyz[0]=0.0; a.bounding_xyz[1]=0; a.bounding_xyz[2]=1.0; 
  a.bounding_rpy[0]=0.0; a.bounding_rpy[1]=0.0; a.bounding_rpy[2]=0.0; 
  //a.bounding_rpy = { xyzrpy[3], xyzrpy[4], xyzrpy[5]};
  // a.bounding_lwh = { 0.3, 0.36, 0.4};
  
  p.aff = a;
  
  std::vector< std::vector< float > > points;
  std::vector< std::vector< int > > triangles;
  affutils.getModelAsLists(filename, points, triangles);
  p.points =points;
  p.npoints=points.size(); 
  p.triangles = triangles;
  p.ntriangles =p.triangles.size();
  
  return p;
}




void Pass::doDemo(int which_publish){
  char* pHome;
  pHome = getenv("HOME");  
  string home = string(pHome);
  cout << home << "\n";  
  
  if ((which_publish==1) || (which_publish==0)){
    int uid0 = 12;
    std::vector<double> xyzrpy0 = {1.27 , 1.30 , 1.16, 0. , 0 , 0};
    //string filename = string(home+ "/drc/software/models/mit_gazebo_models/mesh_otdf/meshes/drill_mfallonio.ply");
    //string filename = string(home+ "/drc/software/models/mit_gazebo_models/mesh_otdf/meshes/drill.pcd");
    //string filename = string(home+ "/drc/software/models/mit_gazebo_models/mesh_otdf/meshes/drill_sensed_smoothed.pcd");
    string filename0 = string(home+ "/drc/software/models/otdf/drill.ply");
    drc::affordance_plus_t a0 = getDynamicMeshCylinderAffordancePlus(filename0, xyzrpy0, uid0);
    a0.aff.bounding_lwh[0]=0.36;       a0.aff.bounding_lwh[1]=0.33;      a0.aff.bounding_lwh[2]=0.3; 
    lcm_->publish("AFFORDANCE_FIT",&a0);
  }

  if ((which_publish==2) || (which_publish==0)){
    int uid0 = 13;
    std::vector<double> xyzrpy0 = {0.51 , 0.23 , 1.16, 0. , 0 , -1.571};
    //string filename = string(home+ "/drc/software/models/mit_gazebo_models/mesh_otdf/meshes/drill_mfallonio.ply");
    //string filename = string(home+ "/drc/software/models/mit_gazebo_models/mesh_otdf/meshes/drill.pcd");
    //string filename = string(home+ "/drc/software/models/mit_gazebo_models/mesh_otdf/meshes/drill_sensed_smoothed.pcd");
    string filename0 = string(home+ "/drc/software/models/otdf/drill.ply");
    drc::affordance_plus_t a0 = getDynamicMeshCylinderAffordancePlus(filename0, xyzrpy0, uid0);
    a0.aff.bounding_lwh[0]=0.36;       a0.aff.bounding_lwh[1]=0.33;      a0.aff.bounding_lwh[2]=0.3; 
    lcm_->publish("AFFORDANCE_FIT",&a0);
  }
  
  if ((which_publish==3) || (which_publish==0)){
    int uid1 = 14;
    std::vector<double> xyzrpy1 = {4.0 , -0.9 , 0.0 , 0. , 0 , -M_PI/2};  
    string filename1 = string(home+ "/drc/software/models/otdf/car.pcd");
    drc::affordance_plus_t a1 = getCarAffordancePlus(filename1, xyzrpy1, uid1 );
    a1.aff.bounding_lwh[0]=3.0;       a1.aff.bounding_lwh[1]=1.7;      a1.aff.bounding_lwh[2]=2.2;//1.7;
    a1.aff.otdf_type = "car";
    lcm_->publish("AFFORDANCE_FIT",&a1);
  }
  
  if ((which_publish==4) || (which_publish==0)){
    int uid1 = 15;
    std::vector<double> xyzrpy1 = {4.18 , 3.65 , 1.2 , -M_PI/2 , 0 , 0};  
    string filename1 = string(home+ "/drc/software/models/otdf/standpipe.ply");
    drc::affordance_plus_t a1 = getDynamicMeshAffordancePlus(filename1, xyzrpy1, uid1 );
    a1.aff.bounding_lwh[0]=0.5;       a1.aff.bounding_lwh[1]=1.7;      a1.aff.bounding_lwh[2]=1.0;//1.7;
    lcm_->publish("AFFORDANCE_FIT",&a1);
  }
/*  if ((which_publish==5) || (which_publish==0)){
    int uid1 = 16;
    std::vector<double> xyzrpy1 = {-1.20 , 2.32 , 1.09 , 0 , 0 , M_PI/2};  
    string filename1 = string(home+ "/drc/software/models/otdf/coupling.ply");
    drc::affordance_plus_t a1 = getDynamicMeshAffordancePlus(filename1, xyzrpy1, uid1 );
    a1.aff.bounding_lwh[0]=3.0;       a1.aff.bounding_lwh[1]=1.7;      a1.aff.bounding_lwh[2]=2.2;//1.7;
    lcm_->publish("AFFORDANCE_FIT",&a1);
  }*/
  if ((which_publish==5) || (which_publish==0)){
    int uid1 = 16;
    std::vector<double> xyzrpy1 = {4.8 , 2.3 , 1.05 , M_PI/2 , 0 , 0};  
    string filename1 = string(home+ "/drc/software/models/otdf/firehose.ply");
    drc::affordance_plus_t a1 = getDynamicMeshAffordancePlus(filename1, xyzrpy1, uid1 );
    a1.aff.bounding_lwh[0]=3.0;       a1.aff.bounding_lwh[1]=1.7;      a1.aff.bounding_lwh[2]=2.2;//1.7;
    lcm_->publish("AFFORDANCE_FIT",&a1);
  }
  
  if ((which_publish==6) || (which_publish==0)){
    int uid1 = 17;
    std::vector<double> xyzrpy1 = {3.18, 3.65, 1.2, 0, -1.5707, 0};  
    string filename1 = string(home+ "/drc/software/models/otdf/valve.ply");
    drc::affordance_plus_t a1 = getDynamicMeshAffordancePlus(filename1, xyzrpy1, uid1 );
    a1.aff.bounding_lwh[0]=3.0;       a1.aff.bounding_lwh[1]=1.7;      a1.aff.bounding_lwh[2]=2.2;//1.7;
    lcm_->publish("AFFORDANCE_FIT",&a1);
    // 3.17
    //valve
    
    // 1.0 -6.3 1.2 1.571 0.1 1.212<
  }

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
  std::cout << "manip drill is 1\n";    
  std::cout << "qual2 drill is 2\n";    
  std::cout << "manip car is 3\n";    
  std::cout << "manip standpipe is 4\n";      
  std::cout << "manip firehose is 5\n";      
  std::cout << "manip valve is 6\n";      

  int which_publish=0;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(which_publish, "e", "which_publish","which_publish [0 is all]");
  opt.parse();
  std::cout << "which_publish: " << which_publish << "\n";    
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "Demo Ready" << endl << "============================" << endl;
  app.doDemo(which_publish);
  return 0;
}
