#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointT;

#include "simple_segment.hpp"


simple_segment::simple_segment(lcm_t* publish_lcm):
      publish_lcm_(publish_lcm),
      null_poseT(0, Eigen::Isometry3d::Identity()) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clf (new pcl::PointCloud<pcl::PointXYZRGB> ());
  incloud = cloud_clf;

  // Unpack Config:
  pc_lcm_ = new pointcloud_lcm(publish_lcm_);

  // Vis Config:
  pc_vis_ = new pointcloud_vis(publish_lcm_);
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Null",5,0) );
  float colors_b[] ={0.0,0.0,1.0};
  vector <float> colors_v;
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Cloud - Raw"          ,1,1, 1000,1,colors_v));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1002,"Cloud - Floor"        ,1,1, 1000,1,colors_v));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1003,"Cloud - Remainder"    ,1,1, 1000,1,colors_v));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1004,"Cloud - Floor Righted",1,1, 1000,1,colors_v));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1005,"Cloud - Remainder Righted"   ,1,1, 1000,1,colors_v));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1006,"Cloud - Box Filtered" ,1,1, 1000,1,colors_v));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1007,"Cloud - Cylinder" ,1,1, 1000,1,colors_v));

  // LCM:
  //lcm_t* subscribe_lcm_ = publish_lcm_;
}




void simple_segment::do_segment(){
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT);
  pc_vis_->ptcld_to_lcm_from_list(1001, *incloud, null_poseT.utime, null_poseT.utime);

  pcl::PointCloud<PointT>::Ptr cloud_filtered0 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
  find_and_remove_plane(incloud, cloud_filtered0, cloud_plane, 0.25, coeff);



  double pitch = atan(coeff->values[0]/coeff->values[2]);
  double roll =- atan(coeff->values[1]/coeff->values[2]);
  double coeff_norm = sqrt(pow(coeff->values[0],2) +
	pow(coeff->values[1],2) + pow(coeff->values[2],2));
  double height = (coeff->values[2]*coeff->values[3]) / coeff_norm;
  if (1==1){ //(verbose_text>0){
    cout  <<  "\nRANSAC Floor Coefficients: " << coeff->values[0]
      << " " << coeff->values[1] << " "  << coeff->values[2] << " " << coeff->values[3] << endl;
    cout << "Pitch: " << pitch << " (" << (pitch*180/M_PI) << "d). positive nose down\n";
    cout << "Roll : " << roll << " (" << (roll*180/M_PI) << "d). positive right side down\n";
    cout << "Height : " << height << " of device off ground [m]\n";
  }

  // drop clouds to the ground:
  Eigen::Vector3f world_trans(0 , 0 , height ); 
  Eigen::Quaternionf world_rot = euler_to_quat_f(0, -pitch, -roll);

  pcl::transformPointCloud (*cloud_plane, *cloud_plane,world_trans, world_rot);  
  pc_vis_->ptcld_to_lcm_from_list(1004, *cloud_plane, null_poseT.utime, null_poseT.utime);
  pcl::transformPointCloud (*cloud_filtered0, *cloud_filtered0,world_trans, world_rot);  
  pc_vis_->ptcld_to_lcm_from_list(1005, *cloud_filtered0, null_poseT.utime, null_poseT.utime);


  box_filter(cloud_filtered0);


  pcl::PointCloud<PointT>::Ptr cloud_filtered1 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_plane1(new pcl::PointCloud<PointT> ());
  pcl::ModelCoefficients::Ptr coeff1 (new pcl::ModelCoefficients);
  find_and_remove_plane(cloud_filtered0, cloud_filtered1, cloud_plane1, 0.1, coeff1);


  //////////////////////////////////////////////////
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  find_cylinder(cloud_filtered1, cloud_cylinder, 0.05);




}


void simple_segment::find_and_remove_plane(pcl::PointCloud<PointT>::Ptr cloud_filtered,
			pcl::PointCloud<PointT>::Ptr cloud_filtered2,
			pcl::PointCloud<PointT>::Ptr cloud_plane,
			double threshold, pcl::ModelCoefficients::Ptr coeff){


  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());


  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);



  std::cout << cloud_filtered->points.size() << " points in total\n";
  std::cout << cloud_normals->points.size() << " normals in total\n";

  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (threshold);//(0.25);
  // seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coeff);
  //std::cerr << "Plane coefficients: " << *coeff << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  extract.filter (*cloud_plane);
  
  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);


  /* pcl::ExtractIndices<pcl::Normal> extract_normals;
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);*/

  if (1==0){
    pc_vis_->ptcld_to_lcm_from_list(1002, *cloud_plane, null_poseT.utime, null_poseT.utime);
    pc_vis_->ptcld_to_lcm_from_list(1003, *cloud_filtered2, null_poseT.utime, null_poseT.utime);
  }
}


void simple_segment::find_cylinder(pcl::PointCloud<PointT>::Ptr cloud_filtered,
			   pcl::PointCloud<PointT>::Ptr cloud_filtered2,double threshold){

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::ExtractIndices<PointT> extract;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
//  seg.setRadiusLimits (0, 0.1);
  seg.setRadiusLimits (0, 0.5);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);


  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
//	  writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
  }

  pc_vis_->ptcld_to_lcm_from_list(1007, *cloud_cylinder, null_poseT.utime, null_poseT.utime);


  pcl::PointCloud<PointT>::Ptr cloudx (new pcl::PointCloud<PointT>);
  pcl::PointXYZRGB pt;
  pt.x = coefficients_cylinder->values[0];
  pt.y = coefficients_cylinder->values[1];
  pt.z = coefficients_cylinder->values[2];
  cloudx->points.push_back(pt);
  pt.x = coefficients_cylinder->values[3];
  pt.y = coefficients_cylinder->values[4];
  pt.z = coefficients_cylinder->values[5];
  cloudx->points.push_back(pt);
  cloudx->width    = 1;
  cloudx->height   = cloudx->points.size();
  cloudx->is_dense = false;


  vector <float> colors_v;
  float colors_b[] ={0.0,0.0,1.0};
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  stringstream ss;
  ss << "Cloud - base point";
  int map_coll_id = 4000;
  ptcld_cfg pcfg = ptcld_cfg(map_coll_id,    ss.str()     ,1,1, 1000,1,colors_v);
  pc_vis_->ptcld_to_lcm(pcfg, *cloudx, null_poseT.utime, null_poseT.utime );




}


void simple_segment::box_filter(pcl::PointCloud<PointT>::Ptr cloud){

  pcl::PassThrough<PointT> pass;

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (0, 2.0);
  pass.filter (*cloud);

  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.45, 20.0);
  pass.filter (*cloud);

  pc_vis_->ptcld_to_lcm_from_list(1006, *cloud, null_poseT.utime, null_poseT.utime);
}


int main (int argc, char** argv){
  std::cout << "simple_segment\n";
  std::cout << "Arguments: program_name cloud.pcd\n";
  std::cout << "  program: " << argv[ 0 ] << "\n";
  std::cout << " pcd file: " << argv[ 1 ] << " [Input]\n";
  
  string fname = argv[ 1 ];
  lcm_t * lcm = lcm_create(NULL);
  simple_segment app(lcm);

  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  reader.read (fname, *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  app.set_input_cloud(cloud);
  app.do_segment();


  return (0);

}
