#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZRGB PointT;


#include "gen-mesh-app.hpp"



gen_mesh_app::gen_mesh_app(lcm_t* publish_lcm):
      publish_lcm_(publish_lcm),
      null_poseT(0, Eigen::Isometry3d::Identity()) {

  // Vis Config:
  pc_vis_ = new pointcloud_vis(publish_lcm_);
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Null",5,0) );
  vector <float> colors_v;
  float colors_b[] ={0.0,0.0,1.0};
  colors_v.assign(colors_b,colors_b+3*sizeof(float));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Cloud - Raw"          ,1,1, 1000,1,colors_v));
  float colors_r[] ={1.0,0.0,0.0};
  colors_v.assign(colors_r,colors_r+3*sizeof(float));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1002,"Mesh - Cylinder"      ,6,1, 1000,1,colors_v));

}

void gen_mesh_app::do_app(){
  double radius =10;
  double length =40;

  pcl::PolygonMesh  mesh;
  pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh(mesh));

  gen_mesh gmesh;
  gmesh.gen_cylinder(mesh_ptr, radius, length);
   
  // transform mesh:
  Eigen::Vector3f world_trans(0 , 0 , 10); 
  Eigen::Quaternionf world_rot = euler_to_quat_f(0, 0, -1.4);
  pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
  pcl::fromROSMsg(mesh_ptr->cloud, temp_cloud);
  pcl::transformPointCloud (temp_cloud, temp_cloud,world_trans, world_rot);  
  pcl::toROSMsg (temp_cloud, mesh_ptr->cloud);

  send_cylinder(mesh_ptr);

  /*
   *   values[0]:   1.10136
  values[1]:   -0.82165
  values[2]:   10.235
  values[3]:   -0.00528161
  values[4]:   -0.023136
  values[5]:   0.999718
  values[6]:   0.143551
   *
   The seven coefficients of the cylinder are given by:
   * a point on its axis,
   * the axis direction, and a radius, as:
   * [point_on_axis.x point_on_axis.y point_on_axis.z
   * axis_direction.x axis_direction.y axis_direction.z
   * radius]*
   */


}

void gen_mesh_app::send_cylinder(pcl::PolygonMesh::Ptr &mesh){
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT);

  pcl::PointCloud<pcl::PointXYZRGB> cloudA;
  pcl::fromROSMsg(mesh->cloud, cloudA);
  pc_vis_->ptcld_to_lcm_from_list(1001, cloudA, null_poseT.utime, null_poseT.utime);

  pc_vis_->mesh_to_lcm_from_list(1002, mesh, null_poseT.utime, null_poseT.utime );
}


int main (int argc, char** argv){
  std::cout << "gen-mesh-app\n";
  lcm_t * lcm = lcm_create(NULL);
  gen_mesh_app app(lcm);
  app.do_app();
  return 0;
}
