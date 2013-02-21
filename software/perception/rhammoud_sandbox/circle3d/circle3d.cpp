#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/sample_consensus/sac_model_circle3d.h> 
#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/rransac.h>
#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/rmsac.h>
#include <pcl/sample_consensus/mlesac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <gtest/gtest.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/sample_consensus/sac_model_normal_sphere.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>
#include <pcl/features/normal_3d.h>
using namespace std;

typedef pcl::PointXYZ PointT;

using namespace pcl;
using namespace pcl::io;

typedef SampleConsensusModelCircle3D<PointXYZ>::Ptr SampleConsensusModelCircle3DPtr;

void saveLineCoef(char *fname, Eigen::VectorXf &fLineCoefs){

  FILE *fid = fopen(fname, "w");
  float _x = fLineCoefs[0];
  float _y = fLineCoefs[1];
  float _z = fLineCoefs[2];
  float _r = fLineCoefs[3];
  float _nx = fLineCoefs[4];
  float _ny = fLineCoefs[5];
  float _nz = fLineCoefs[6];

  fprintf(fid, "%f %f %f %f %f %f %f\n", _x, _y, _z, _r, _nx, _ny, _nz);

  fclose(fid); 

}



int
main (int argc, char** argv)
{

  if (argc < 7) {
    std::cout << "\n\nUsage: progName \n\n"
	      << "bounding box limits: [xLow xHigh yLow yHigh zLow zHigh] \n"
	      << "Fitting parameters: [dist_thresh] (radius limits: [low, high])  \n"   
	      << "input PCD file.pcd\n" 
	      << "output_file_name.pcd\n" 
	      << "\n\n";
    exit(0);
  }

  // All the objects needed
  pcl::PCDReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered_y (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered_z (new pcl::PointCloud<PointT>);


  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Read in the cloud data
  // reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
  //    reader.read ("waterBottle.pcd", *cloud);
  reader.read(argv[10], *cloud); //tube.pcd //table.pcd", *cloud); 
std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

// Create a shared 3d circle model pointer directly
  SampleConsensusModelCircle3DPtr model (new SampleConsensusModelCircle3D<PointXYZ> (cloud));

 // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac(model, 0.01);
 // Algorithm tests
  bool result = sac.computeModel ();

  cout << "convergence: " << result << endl;

  std::vector<int> sample;
  sac.getModel (sample);
 
  std::vector<int> inliers;
  sac.getInliers (inliers);

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  
  saveLineCoef("fitted_3dcircle_params.txt", coeff); 

  cout << "\n------------------------\n 3D circle fitting resutls:\n " << coeff << endl;

  return 1; 

/* 
//----------------------------------
// compute centroid 

Eigen::Vector4f centroid4f; 
double zm = 0; 
centroid4f[0] = 0; 
centroid4f[1] = 0; 
centroid4f[2] = 0; 

for (size_t i = 0; i < cloud->points.size(); i++)
{
centroid4f[0] += cloud->points[i].x;
centroid4f[1] += cloud->points[i].y;
zm = zm + cloud->points[i].z; 
}

centroid4f[0] =   centroid4f[0]/cloud->points.size();
centroid4f[1] =   centroid4f[1]/cloud->points.size();
zm = zm/cloud->points.size();

cout << "Cloud centroid:: " << centroid4f[0] << ",  " << centroid4f[1] <<  ":: Z= " << zm << endl;

//------------------------------------

*/

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  //pass.setFilterLimits (-10, 10);
  float xL = atof(argv[1]); 
  float xH = atof(argv[2]);
  pass.setFilterLimits (xL, xH); //-1.5, 0);
  pass.filter (*cloud_filtered);

  cout << xL << " " << xH << endl;
  // y
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  //pass.setFilterLimits (-10, 10);
  float yL = atof(argv[3]); 
  float yH = atof(argv[4]); 
  pass.setFilterLimits (yL, yH); //0, 2);
  pass.filter (*cloud_filtered_y);

  cout << yL << " " << yH << endl;

  // z
  pass.setInputCloud (cloud_filtered_y);
  pass.setFilterFieldName ("z");
  //pass.setFilterLimits (-10, 10);
  float zL = atof(argv[5]); 
  float zH = atof(argv[6]); 
  
  pass.setFilterLimits (zL, zH); //0, 0.05);
  pass.filter (*cloud_filtered_z);

  cout << zL << " " << zH << endl;

  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  writer.write ("table_filtered.pcd", *cloud_filtered_z, false);


  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered_z);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered_z);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered_z);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  writer.write ("table_plane.pcd", *cloud_plane, false);

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_MLESAC);//SAC_RANSAC);
  //seg.setNormalDistanceWeight (0.5);
  //seg.setMaxIterations (10000);
  float distThre = atof(argv[7]); 
  seg.setDistanceThreshold (distThre); //0.09);
  float rL = atof(argv[8]);
  float rH = atoi(argv[9]);
  seg.setRadiusLimits (rL, rH); //0.05, 2);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  writer.write ("table_objects.pcd", *cloud_filtered2, false);

  /*// Create a shared 3d circle model pointer directly
  SampleConsensusModelCircle3DPtr model (new SampleConsensusModelCircle3D<PointXYZ> (cloud_filtered2));

 // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac(model, 0.03);
 // Algorithm tests
  bool result = sac.computeModel ();

  std::vector<int> sample;
  sac.getModel (sample);
 
  std::vector<int> inliers;
  sac.getInliers (inliers);

  Eigen::VectorXf coeff;
  sac.getModelCoefficients (coeff);
  
  
  cout << "\n------------------------\n 3D circle fitting resutls:\n " << coeff << endl;
  */

  // Riad

  FILE *fid = fopen("cylinder_coef_table_out.txt", "w");

  float _x = coefficients_cylinder->values[0];
  float _y = coefficients_cylinder->values[1];
  float _z = coefficients_cylinder->values[2];   

  float _nx = coefficients_cylinder->values[3];
  float _ny = coefficients_cylinder->values[4];
  float _nz = coefficients_cylinder->values[5]; 

  float _radius = coefficients_cylinder->values[6];

  fprintf(fid, "%f %f %f \n %f %f %f\n %f \n", _x, _y, _z, _nx, _ny, _nz, _radius);

  fclose(fid);
  

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);

  // "cylinder_table_pcd_out.pcd"

  if (cloud_cylinder->points.empty ()) 
    std::cerr << "\n\n ------------------------------------- \n\n Can't find the cylindrical component. \n\n --------------------------------------" << std::endl;
  else
  {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
	  // writer.write ("waterBottle_textured_cylinder.pcd", *cloud_cylinder, false);
	  writer.write (argv[11], *cloud_cylinder, false);
  }
  return (0);
}
 

