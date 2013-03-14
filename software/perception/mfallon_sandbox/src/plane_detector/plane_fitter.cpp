#include <iostream>
#include <boost/assign/std/vector.hpp>



#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


#include <lcm/lcm.h>

#include <ConciseArgs>

#include <pointcloud_tools/pointcloud_vis.hpp>
#include <pointcloud_tools/filter_planes.hpp>


using namespace std;
using namespace boost::assign;





int
main (int argc, char** argv)
{
  string filename= "file_name.pcd";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(filename, "m", "filename",
          "filename of input pcd");
  opt.parse();  
  cout << "filename: " << filename <<"\n";
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  // 1. Read input:
  pcl::PCDReader reader;
  reader.read (filename, *cloud_filtered);
  std::cerr << "[IN] : " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  int64_t fake_timestamp = 1;
  int coll_id =1;
  lcm_t * publish_lcm_;
  publish_lcm_ = lcm_create(NULL);
  
  pointcloud_vis* pc_vis_;
  pc_vis_ = new pointcloud_vis(publish_lcm_);
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Null",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Cloud - Laser Map"     ,1,1, 1000,1, {0,0,1}));
  
  Isometry3dTime null_poseT = Isometry3dTime(fake_timestamp, Eigen::Isometry3d::Identity());
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT);
  pc_vis_->ptcld_to_lcm_from_list(1001, *cloud_filtered, fake_timestamp, fake_timestamp);
  
  // 2. Extract the major planes and send them to lcm:
  FilterPlanes filtp;
  filtp.setInputCloud(cloud_filtered);
  filtp.setPoseIDs(coll_id,fake_timestamp);
  filtp.setLCM(publish_lcm_);
  vector<BasicPlane> plane_stack; 
  filtp.filterPlanes(plane_stack);
  std::cout << "[OUT] number of planes extracted: " << plane_stack.size() << "\n";
  
  // 3. Visualise normals:
  for (size_t i = 0; i< plane_stack.size() ; i++){
    BasicPlane plane = plane_stack[i];
    char minor_char[10],major_char[10];
    sprintf(major_char,"%d",plane.major);
    sprintf(minor_char,"%d",plane.minor);
    stringstream name_out;
    name_out << "local_polygons_major_" << major_char << "_minor_" << minor_char;
    int plane_id = 290+ i ;
    ptcld_cfg pcfg = ptcld_cfg(plane_id,    name_out.str()     ,3,1, 1000,1,{-1,-1,-1} );
    pc_vis_->ptcld_to_lcm(pcfg, (plane.cloud), null_poseT.utime, null_poseT.utime );
  } 
  
  for (size_t i=0;i<plane_stack.size();i++){
    BasicPlane plane = plane_stack[i];
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr normals_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    normals_cloud->points.push_back( plane.cloud.points[0]);
    pcl::PointXYZRGB pt;
    pt.x= plane.cloud.points[0].x + plane.coeffs.values[0];
    pt.y= plane.cloud.points[0].y + plane.coeffs.values[1];
    pt.z= plane.cloud.points[0].z + plane.coeffs.values[2];
    pt.r =0;      pt.g =255;      pt.b =0;
    normals_cloud->points.push_back( pt );

    stringstream name_out2;
    name_out2 << "global_normal_local_polygons_major_" << plane.major << "_minor_" << plane.minor;
    int plane_id2 = 590+ i  ;
    ptcld_cfg pcfg2 = ptcld_cfg(plane_id2,    name_out2.str()     ,3,1, 1000,1,{0.3,.8,0.1} );
    pc_vis_->ptcld_to_lcm(pcfg2, *normals_cloud, null_poseT.utime, null_poseT.utime );        
  }   
  
  return 0;
}