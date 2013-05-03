#include <map>
#include <boost/assign/std/vector.hpp>

#include "AffordanceUtils.hpp"

using namespace Eigen;
using namespace std;
using namespace boost::assign; // bring 'operator+()' into scope


AffordanceUtils::AffordanceUtils() {
  
}

Eigen::Isometry3d AffordanceUtils::getPose(double xyz[3], double rpy[3]){
//  std::vector<std::string> param_names, std::vector<double> params ){
  /*
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
  */
  Matrix3d m;
  m = AngleAxisd ( rpy[2], Vector3d::UnitZ ())
                  * AngleAxisd (rpy[1] , Vector3d::UnitY ())
                  * AngleAxisd ( rpy[0] , Vector3d::UnitX ());  
  Eigen::Isometry3d pose =  Eigen::Isometry3d::Identity();
  pose *= m;  
  pose.translation()  << xyz[0], xyz[1], xyz[2];  
  
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
  cout << pts->points.size() << " points extracted [converted]\n";
  return pts;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr AffordanceUtils::getCloudFromAffordance(std::vector< std::vector< float > > &points,
                      std::vector< std::vector< int > > &triangles, double pts_per_msquared){
  pcl::PolygonMesh::Ptr mesh = getMeshFromAffordance(points,triangles);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts =prim_->sampleMesh(mesh, pts_per_msquared);
  cout << pts->points.size() << " points extracted [sampled] at " <<  pts_per_msquared << " psm\n";
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

void AffordanceUtils::setPlaneFromXYZYPR(double xyz[3], double rpy[3], 
                std::vector<float> &plane_coeffs, Eigen::Vector3d &plane_centroid){
  // Ridiculously hacky way of converting from plane affordance to plane coeffs.
  // the x-direction of the plane pose is along the axis - hence this
  Matrix3d m;
  m = AngleAxisd ( rpy[2], Vector3d::UnitZ ())
                  * AngleAxisd (rpy[1] , Vector3d::UnitY ())
                  * AngleAxisd ( rpy[0] , Vector3d::UnitX ());  
  Eigen::Isometry3d transform =  Eigen::Isometry3d::Identity();
  transform *= m;  
  transform.translation()  << xyz[0], xyz[1], xyz[2];    
  

  Eigen::Isometry3d ztransform;
  ztransform.setIdentity();
  ztransform.translation()  << 0 ,0, 1; // determine a point 1m in the z direction... use this as the normal
  ztransform = transform*ztransform;
  float a =(float) ztransform.translation().x() -  transform.translation().x();
  float b =(float) ztransform.translation().y() -  transform.translation().y();
  float c =(float) ztransform.translation().z() -  transform.translation().z();
  float d = - (a*xyz[0] + b*xyz[1] + c*xyz[2]);
  plane_coeffs.clear();
  plane_coeffs += a, b, c, d;
  /*
  cout << "pitch : " << 180.*am.find("pitch")->second/M_PI << "\n";
  cout << "yaw   : " << 180.*am.find("yaw")->second/M_PI << "\n";
  cout << "roll   : " << 180.*am.find("roll")->second/M_PI << "\n";   
  obj_cfg oconfig = obj_cfg(1251000,"Tracker | Affordance Pose Z",5,1);
  Isometry3dTime reinit_poseT = Isometry3dTime ( 0, ztransform );
  pc_vis_->pose_to_lcm(oconfig,reinit_poseT);
  */

  plane_centroid=  Eigen::Vector3d( xyz[0], xyz[1], xyz[2]); // last element held at zero
}

void AffordanceUtils::setXYZRPYFromPlane(double xyz[3], double rpy[3], 
                std::vector<float> plane_coeffs, Eigen::Vector3d plane_centroid){
  
  double run = sqrt(plane_coeffs[0]*plane_coeffs[0] + plane_coeffs[1]*plane_coeffs[1] 
                        +  plane_coeffs[2]*plane_coeffs[2]);
  double yaw = atan2 ( plane_coeffs[1] , plane_coeffs[0]);
  double pitch = acos( plane_coeffs[2]/ run);
  // Conversion from Centroid+Plane to XYZRPY is not constrained
  // - properly set to zero
  double roll =0; 

  xyz[0] = plane_centroid(0);
  xyz[1] = plane_centroid(1);
  xyz[2] = plane_centroid(2);
  rpy[0] = roll;
  rpy[1] = pitch;
  rpy[2] = yaw;
  
/*  for (size_t j=0; j< param_names.size(); j++){
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
*/  
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


void AffordanceUtils::setXYZRPYFromIsometry3d(double xyz[3], double rpy[3], 
                   Eigen::Isometry3d &pose){
  Eigen::Quaterniond r(pose.rotation());
  double yaw, pitch, roll;
  quat_to_euler(r, yaw, pitch, roll);  

  xyz[0] = pose.translation().x();
  xyz[1] = pose.translation().y();
  xyz[2] = pose.translation().z();
  
  
  rpy[0] = roll;
  rpy[1] = pitch;
  rpy[2] = yaw;
}  



pcl::PointCloud<pcl::PointXYZRGB>::Ptr AffordanceUtils::getBoundingBoxCloud(double bounding_xyz[], double bounding_rpy[], double bounding_lwh[]){
  
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

