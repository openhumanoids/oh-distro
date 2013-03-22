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
    
    void doTest(std::string fname);
    
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
  
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(9986,"RGBD Primitive - Cylinder"     ,7,1, 9995,0, colors_v ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(9987,"RGBD Primitive - Cube"     ,7,1, 9995,0, colors_v ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(9988,"RGBD Primitive - Disc"     ,7,1, 9995,0, colors_v ));
  // 3 gives a wireframe
  
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(9996,"RGBD Primitive - Points on Cylinder"     ,1,1, 9995,0, colors_v ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(9997,"RGBD Primitive - Points on Cube"     ,1,1, 9995,0, colors_v ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(9998,"RGBD Primitive - Points on Disc"     ,1,1, 9995,0, colors_v ));

  
  verbose_ =false;  
}

void Pass::doTest(std::string fname){

  // Visualise:
  int64_t pose_id =0;
  Eigen::Isometry3d null_pose;
  null_pose.setIdentity();
  Isometry3dTime null_poseT = Isometry3dTime(pose_id, null_pose);
  pc_vis_->pose_to_lcm_from_list(9995, null_poseT);
  
  double length = 0.5;
  double radius = 0.25;
  
  // transform to be applied to object:
  Eigen::Isometry3d transform;
  Eigen::Quaterniond quat = Eigen::Quaterniond(1,0,0,0);
  transform.setIdentity();
  transform.translation()  << 0.18,0,0;
  transform.rotate(quat);

  // cylinder:
  pcl::PolygonMesh::Ptr mesh_cylinder = prim_->getCylinderWithTransform(transform, radius, radius, length);
  pc_vis_->mesh_to_lcm_from_list(9986, mesh_cylinder, pose_id , pose_id);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts =prim_->sampleMesh(mesh_cylinder, 400);
  pc_vis_->ptcld_to_lcm_from_list(9996, *pts, pose_id , pose_id);  
  
  // cube:
  transform.translation()  << 0.18,0.8,0.0;
  pcl::PolygonMesh::Ptr mesh_cube = prim_->getCubeWithTransform(transform, 0.21, 0.21,0.5);
  pc_vis_->mesh_to_lcm_from_list(9987, mesh_cube, pose_id , pose_id);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts2 =prim_->sampleMesh(mesh_cube, 400);
  pc_vis_->ptcld_to_lcm_from_list(9997, *pts2, pose_id , pose_id);  

  transform.translation()  << 0.18,-0.8,0.0;
  pcl::PolygonMesh::Ptr mesh_disc = prim_->getCylinderWithTransform(transform, radius, 0.0, 0);
  pc_vis_->mesh_to_lcm_from_list(9988, mesh_disc, pose_id , pose_id);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts3 =prim_->sampleMesh(mesh_disc, 400); // NB: disc sampled for both top and bottom lid - givine twice the density
  pc_vis_->ptcld_to_lcm_from_list(9998, *pts3, pose_id , pose_id);  
 
  
  
  
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
  
  app.doTest(camera_channel);
  
  

  
  return 0;
}
