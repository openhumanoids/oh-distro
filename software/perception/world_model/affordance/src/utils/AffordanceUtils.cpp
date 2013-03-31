#include <map>
#include "AffordanceUtils.hpp"

using namespace Eigen;
using namespace std;


AffordanceUtils::AffordanceUtils() {
  
}

Eigen::Isometry3d AffordanceUtils::getPose(std::vector<std::string> param_names, std::vector<double> params ){
  std::map<std::string,double> am;
  for (size_t j=0; j< param_names.size(); j++){
    am[ param_names[j] ] = params[j];
  }
  
  // Convert Euler to Isometry3d:
  Matrix3d m;
  m = AngleAxisd (am.find("yaw")->second, Vector3d::UnitZ ())
                  * AngleAxisd (am.find("pitch")->second, Vector3d::UnitY ())
                  * AngleAxisd (am.find("roll")->second, Vector3d::UnitX ());  
  Eigen::Isometry3d pose =  Eigen::Isometry3d::Identity();
  pose *= m;  
  pose.translation()  << am.find("x")->second , am.find("y")->second, am.find("z")->second;
  return pose;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr AffordanceUtils::getCloudFromAffordance(std::vector< std::vector< float > > &points){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts (new pcl::PointCloud<pcl::PointXYZRGB> ());
  for (size_t i=0; i < points.size(); i++){ 
    pcl::PointXYZRGB pt;
    pt.x = points[i][0];
    pt.y = points[i][1];
    pt.z = points[i][2];
    pts->points.push_back(pt);
  }
  cout << pts->points.size() << " points extracted\n";
  return pts;
}


pcl::PolygonMesh::Ptr AffordanceUtils::getMeshFromAffordance(std::vector< std::vector< float > > &points, 
                  std::vector< std::vector< int > > &triangles, Eigen::Isometry3d & transform){
  
  pcl::PolygonMesh::Ptr mesh = getMeshFromAffordance(points, triangles);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::fromROSMsg(mesh->cloud, *cloud);  
  
  // Adjust object to be centered on z-axis (standard used by URDF)
//  pcl::transformPointCloud (*cloud, *cloud,
//        Eigen::Vector3f(0,0, -height/2.0), Eigen::Quaternionf(1.0, 0.0,0.0,0.0)); // !! modifies cloud
    
  Eigen::Isometry3f pose_f = transform.cast<float>();
  Eigen::Quaternionf quat_f(pose_f.rotation());
  pcl::transformPointCloud (*cloud, *cloud,
      pose_f.translation(), quat_f); // !! modifies cloud
  
  pcl::toROSMsg(*cloud, mesh->cloud);  
  return mesh;
}

pcl::PolygonMesh::Ptr AffordanceUtils::getMeshFromAffordance(std::vector< std::vector< float > > &points, 
                  std::vector< std::vector< int > > &triangles){
  pcl::PolygonMesh mesh;   
  pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh (mesh));  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts (new pcl::PointCloud<pcl::PointXYZRGB> ());
  
  for (size_t i=0; i < points.size(); i++){ 
    pcl::PointXYZRGB pt;
    pt.x = points[i][0];
    pt.y = points[i][1];
    pt.z = points[i][2];
    pts->points.push_back(pt);
  }
  pcl::toROSMsg (*pts, mesh_ptr->cloud);
  
  vector <pcl::Vertices> verts;
  for(size_t i=0; i<  triangles.size (); i++){ // each triangle/polygon
    pcl::Vertices poly;
    std::vector <unsigned int> v ( triangles[i].begin(), triangles[i].end()) ;
    poly.vertices = v;
    verts.push_back(poly);
  }
  mesh_ptr->polygons = verts;
  
  return mesh_ptr;
}

void AffordanceUtils::setXYZRPYFromPlane(std::vector<std::string> &param_names, std::vector<double> &params, 
                std::vector<float> plane_coeffs, Eigen::Vector3d plane_centroid){
  
  double run = sqrt(plane_coeffs[0]*plane_coeffs[0] + plane_coeffs[1]*plane_coeffs[1] 
                        +  plane_coeffs[2]*plane_coeffs[2]);
  double yaw = atan2 ( plane_coeffs[1] , plane_coeffs[0]);
  double pitch = acos( plane_coeffs[2]/ run);
  double roll =0; // not constrained - properly set to zero

  for (size_t j=0; j< param_names.size(); j++){
    if (param_names[j] == "x"){
      params[j] = plane_centroid(0);
    }else if(param_names[j] == "y"){
      params[j] = plane_centroid(1);
    }else if(param_names[j] == "z"){
      params[j] = plane_centroid(2);
    }else if(param_names[j] == "yaw"){
      params[j] = yaw;
    }else if(param_names[j] == "pitch"){
      params[j] = pitch;
    }else if(param_names[j] == "roll"){
      params[j] = roll;
    }
  }
  
}


/// This function replicates one in pointcloud_math. But does a function exist in Eigen?
void quat_to_euler(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}


void AffordanceUtils::setXYZRPYFromIsometry3d(std::vector<std::string> &param_names, std::vector<double> &params, 
                   Eigen::Isometry3d pose){
  Eigen::Quaterniond r(pose.rotation());
  double yaw, pitch, roll;
  quat_to_euler(r, yaw, pitch, roll);  

  for (size_t j=0; j< param_names.size(); j++){
    if (param_names[j] == "x"){
      params[j] = pose.translation().x();
    }else if(param_names[j] == "y"){
      params[j] = pose.translation().y();
    }else if(param_names[j] == "z"){
      params[j] = pose.translation().z();
    }else if(param_names[j] == "yaw"){
      params[j] = yaw;
    }else if(param_names[j] == "pitch"){
      params[j] = pitch;
    }else if(param_names[j] == "roll"){
      params[j] = roll;
    }
  }
}  



pcl::PointCloud<pcl::PointXYZRGB>::Ptr AffordanceUtils::getBoundingBoxCloud(float bounding_pos[], float bounding_rpy[], float bounding_lwh[]){
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr bb_pts (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointXYZRGB pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8;
  pt1.x = -bounding_lwh[0]/2;       pt1.y = -bounding_lwh[1]/2;    pt1.z = -bounding_lwh[2]/2;  
  pt2.x = -bounding_lwh[0]/2;       pt2.y = -bounding_lwh[1]/2;    pt2.z = bounding_lwh[2]/2;  

  pt3.x = -bounding_lwh[0]/2;       pt3.y = bounding_lwh[1]/2;    pt3.z = -bounding_lwh[2]/2;  
  pt4.x = -bounding_lwh[0]/2;       pt4.y = bounding_lwh[1]/2;    pt4.z = bounding_lwh[2]/2;  

  pt5.x = bounding_lwh[0]/2;       pt5.y = -bounding_lwh[1]/2;    pt5.z = -bounding_lwh[2]/2;  
  pt6.x = bounding_lwh[0]/2;       pt6.y = -bounding_lwh[1]/2;    pt6.z = bounding_lwh[2]/2;  

  pt7.x = bounding_lwh[0]/2;       pt7.y = bounding_lwh[1]/2;    pt7.z = -bounding_lwh[2]/2;  
  pt8.x = bounding_lwh[0]/2;       pt8.y = bounding_lwh[1]/2;    pt8.z = bounding_lwh[2]/2; 

  // z-dir
  bb_pts->points.push_back(pt1);      bb_pts->points.push_back(pt2);    
  bb_pts->points.push_back(pt3);      bb_pts->points.push_back(pt4);    
  bb_pts->points.push_back(pt5);  bb_pts->points.push_back(pt6);
  bb_pts->points.push_back(pt7);  bb_pts->points.push_back(pt8);
  // x-dir
  bb_pts->points.push_back(pt1);  bb_pts->points.push_back(pt5);   
  bb_pts->points.push_back(pt2);  bb_pts->points.push_back(pt6);    
  bb_pts->points.push_back(pt3);  bb_pts->points.push_back(pt7);    
  bb_pts->points.push_back(pt4);  bb_pts->points.push_back(pt8);    
  // y-dir
  bb_pts->points.push_back(pt1);  bb_pts->points.push_back(pt3);   
  bb_pts->points.push_back(pt2);  bb_pts->points.push_back(pt4);    
  bb_pts->points.push_back(pt5);  bb_pts->points.push_back(pt7);    
  bb_pts->points.push_back(pt6);  bb_pts->points.push_back(pt8);    

  return bb_pts;
  //Eigen::Quaternionf quat(pose.rotation());
  //pcl::transformPointCloud (*bb_pts, *bb_pts,
  //      pose.translation(), quat); // !! modifies lidar_cloud

  //pc_vis_->ptcld_to_lcm_from_list(771005, *bb_pts, null_poseT_.utime, null_poseT_.utime);
}

