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

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <affordance/AffordanceUtils.hpp>

using namespace Eigen;
using namespace pcl;
using namespace pcl::io;

using namespace std;
class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~Pass(){
    }    
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void affordanceHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::affordance_plus_collection_t* msg);
    pointcloud_vis* pc_vis_;
    
    AffordanceUtils affutils;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_): lcm_(lcm_){
  lcm_->subscribe("AFFORDANCE_PLUS_COLLECTION",&Pass::affordanceHandler,this);  
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  cout << "Finished setting up\n";
}

void Pass::affordanceHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::affordance_plus_collection_t* msg){
  cout << "got "<< msg->affs_plus.size() <<" affs\n";
  for (size_t i=0; i < msg->affs_plus.size() ; i++){

    drc::affordance_plus_t a = msg->affs_plus[i];
    int aff_id =a.aff.uid;
    int cfg_root = aff_id*10;
    
    Matrix3d m;
    m = AngleAxisd ( a.aff.origin_rpy[2], Vector3d::UnitZ ())
                  * AngleAxisd( a.aff.origin_rpy[1], Vector3d::UnitY ())
                  * AngleAxisd( a.aff.origin_rpy[0], Vector3d::UnitX ());  
    Eigen::Isometry3d pose =  Eigen::Isometry3d::Identity();
    pose *= m;  
    pose.translation()  << a.aff.origin_xyz[0] , a.aff.origin_xyz[1], a.aff.origin_xyz[2];
    //Eigen::Isometry3d pose = affutils.getPose( a.aff.param_names, a.aff.params );
    // obj: id name type reset
    // pts: id name type reset objcoll usergb rgb
    
    if (a.points.size() !=0 ){
      obj_cfg oconfig = obj_cfg(cfg_root,   string( "Affordance Pose " + std::to_string(aff_id))   ,5,1);
      Isometry3dTime poseT = Isometry3dTime ( 0, pose  );
      pc_vis_->pose_to_lcm(oconfig,poseT);

      if (a.triangles.size() ==0){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = affutils.getCloudFromAffordance(a.points);
        ptcld_cfg pconfig = ptcld_cfg(cfg_root+1,  string( "Affordance Cloud " + std::to_string(aff_id))     ,1,1, cfg_root,1, {0.2,0,0.2} );
        pc_vis_->ptcld_to_lcm(pconfig, *cloud, 0, 0);  
      }else{
        pcl::PolygonMesh::Ptr mesh = affutils.getMeshFromAffordance(a.points, a.triangles);
        ptcld_cfg pconfig = ptcld_cfg(cfg_root+2,    string( "Affordance Mesh " + std::to_string(aff_id))     ,7,1, cfg_root,1, {0.2,0,0.2} );
        pc_vis_->mesh_to_lcm(pconfig, mesh, 0, 0);  
      }
      
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr bb_cloud = affutils.getBoundingBoxCloud(a.aff.bounding_xyz, a.aff.bounding_rpy, a.aff.bounding_lwh);
      ptcld_cfg pconfig = ptcld_cfg(cfg_root+3,    string( "Affordance Bounding Box " + std::to_string(aff_id))     ,4,1, cfg_root,1, {0.2,0,0.2} );
      pc_vis_->ptcld_to_lcm(pconfig, *bb_cloud, 0, 0);  
    }
  }
}


int main( int argc, char** argv ){
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "Demo Receive Ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
