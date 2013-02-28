#include <iostream>
#include <Eigen/Dense>


#include <rgbd_simulation/rgbd_primitives.hpp>

#define VTK_EXCLUDE_STRSTREAM_HEADERS


#include <lcm/lcm-cpp.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

#include <ConciseArgs>

using namespace std;
using namespace Eigen;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &publish_lcm);
    
    ~Pass(){
    }
    
    void doTest();
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    std::string camera_channel_;

    
    pointcloud_vis* pc_vis_;
    bool verbose_;
    
    boost::shared_ptr<rgbd_primitives>  prim_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_):          
    lcm_(lcm_){

  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  float colors_b[] ={1.0,0.0,1.0};
  std::vector<float> colors_v;
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(9995,"RGBD Primatives - Null",5,1) );
  
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(9996,"RGBD Primitive - Cylinder"     ,7,1, 9995,0, colors_v ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(9997,"RGBD Primitive - Cube"     ,7,1, 9995,0, colors_v ));
  // 2 gives a wireframe
  
  verbose_ =false;  
}

void Pass::doTest(){

  double length = 0.5;
  double radius = 0.05;
  
  Eigen::Isometry3d transform;
  Eigen::Quaterniond quat = Eigen::Quaterniond(1,0,0,0);
  transform.setIdentity();
  transform.translation()  << 0.18,0,0;
  transform.rotate(quat);

  // cylinder:
  pcl::PolygonMesh::Ptr mesh_cylinder = prim_->getCylinderWithTransform(transform, radius, radius, length);
  
  // cube:
  transform.translation()  << -0.18,-0.3,-0.4;
  pcl::PolygonMesh::Ptr mesh_cube = prim_->getCubeWithTransform(transform, 0.1, 0.2,0.5);
    
  int64_t pose_id =0;
  Eigen::Isometry3d null_pose;
  null_pose.setIdentity();
  Isometry3dTime null_poseT = Isometry3dTime(pose_id, null_pose);
  
  pc_vis_->pose_to_lcm_from_list(9995, null_poseT);
  pc_vis_->mesh_to_lcm_from_list(9996, mesh_cylinder, pose_id , pose_id);
  pc_vis_->mesh_to_lcm_from_list(9997, mesh_cube, pose_id , pose_id);
}


int 
main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  string camera_channel="CAMERALEFT";
  parser.add(camera_channel, "c", "camera_channel", "Camera channel");
  parser.parse();
  cout << camera_channel << " is camera_channel\n"; 
  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "image-passthrough ready" << endl << endl;
  
  //app.prim_(4.0,4.0, 10.0, 32,1);
  
  app.doTest();
  
  

  
  return 0;
}
