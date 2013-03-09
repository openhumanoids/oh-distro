#include "rgbd_primitives.hpp"
#define PCL_VERBOSITY_LEVEL L_ERROR

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

// random numbers on range 0->1
boost::variate_generator<boost::mt19937, boost::uniform_01<> >
    rand01(boost::mt19937(time(0)),
              boost::uniform_01<>());




rgbd_primitives::rgbd_primitives(){

}

void set_color(pcl::PointXYZRGB &pt){
 pt.r =55.0;
 pt.g = 125.0;
 pt.b = 50.0;
}

// Duplicates function in pointcloud_math - and surely this can be done in a functioncall
Eigen::Isometry3f Isometry_d2f(Eigen::Isometry3d pose_in){
  
  Eigen::Quaterniond r(pose_in.rotation());
  Eigen::Quaternionf rf(r.w() , r.x() , r.y() , r.z() );

  Eigen::Isometry3f pose_out;
  pose_out.setIdentity();
  pose_out.translation()  << pose_in.translation().x() , pose_in.translation().y() , pose_in.translation().z();
  pose_out.rotate(rf);
  return pose_out;
}


pcl::PolygonMesh::Ptr rgbd_primitives::getCylinderWithTransform(Eigen::Isometry3d transform, double base, double top, double height){

  pcl::PolygonMesh::Ptr mesh_cylinder = getCylinder(base ,top , height, 36,1);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::fromROSMsg(mesh_cylinder->cloud, *cloud);  
  
  // Adjust object to be centered on z-axis (standard used by URDF)
  pcl::transformPointCloud (*cloud, *cloud,
        Eigen::Vector3f(0,0, -height/2.0), Eigen::Quaternionf(1.0, 0.0,0.0,0.0)); // !! modifies cloud
    
  Eigen::Isometry3f pose_f = Isometry_d2f(transform);
  Eigen::Quaternionf quat_f(pose_f.rotation());
  pcl::transformPointCloud (*cloud, *cloud,
      pose_f.translation(), quat_f); // !! modifies cloud
  
  pcl::toROSMsg(*cloud, mesh_cylinder->cloud);
    
  return mesh_cylinder;
}




pcl::PolygonMesh::Ptr rgbd_primitives::getCubeWithTransform(Eigen::Isometry3d transform, double xdim, double ydim, double zdim){

  pcl::PolygonMesh::Ptr mesh_cylinder = getCube(xdim, ydim, zdim);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::fromROSMsg(mesh_cylinder->cloud, *cloud);  
  
  // Adjust object to be centered on z-axis (standard used by URDF)
  pcl::transformPointCloud (*cloud, *cloud,
        Eigen::Vector3f( -xdim/2.0 , -ydim/2.0, -zdim/2.0), Eigen::Quaternionf(1.0, 0.0,0.0,0.0)); // !! modifies cloud
    
  Eigen::Isometry3f pose_f = Isometry_d2f(transform);
  Eigen::Quaternionf quat_f(pose_f.rotation());
  pcl::transformPointCloud (*cloud, *cloud,
      pose_f.translation(), quat_f); // !! modifies cloud
  
  pcl::toROSMsg(*cloud, mesh_cylinder->cloud);
    
  return mesh_cylinder;
}



// create a polygon mesh of a cylinder (clones gluCylinder)
// TODO: add top and bottom discs
// base Specifies the radius of the cylinder at z = 0.
// top Specifies the radius of the cylinder at z = height.
// height Specifies the height of the cylinder.
// slices Specifies the number of subdivisions around the z axis.
// stacks Specifies the number of subdivisions along the z axis.
// This method will return a cylinder if a radius is set to zero
// This method will return a pyramid if the slices is set to 4
// stacks not used
pcl::PolygonMesh::Ptr rgbd_primitives::getCylinder(double base, double top, double height, int slices, int stacks){ 
  pcl::PolygonMesh mesh;   
  pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh (mesh));  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts (new pcl::PointCloud<pcl::PointXYZRGB> ());
  std::vector <pcl::Vertices> verts;
  
  double delta = (2*M_PI/slices);
  int i_int=0;
  for (double i=0,  i_int=0;i< 2*M_PI; i=i+ delta, i_int++){
    pcl::PointXYZRGB pt1, pt2, pt3, pt4, pt5, pt6;
    pt1.x = base*cos(i);       pt1.y = base*sin(i);  pt1.z = 0;  
    pt2.x =  top*cos(i);       pt2.y =  top*sin(i); pt2.z = height; 
    pt3.x = base*cos(i+delta); pt3.y = base*sin(i+delta);  pt3.z = 0;
    set_color(pt1); set_color(pt2); set_color(pt3);
    pts->points.push_back(pt1);
    pts->points.push_back(pt2);
    pts->points.push_back(pt3);
    pcl::Vertices vert;
    vert.vertices.push_back(i_int*6 ); vert.vertices.push_back(i_int*6+1); vert.vertices.push_back(i_int*6+2);
    verts.push_back(vert);
    
    pt4.x = base*cos(i+delta)  ; pt4.y = base*sin(i+delta); pt4.z = 0;
    pt5.x =  top*cos(i+delta)  ; pt5.y =  top*sin(i+delta); pt5.z = height;
    pt6.x =  top*cos(i)        ; pt6.y =  top*sin(i); pt6.z = height;
    set_color(pt4); set_color(pt5); set_color(pt6);
    pts->points.push_back(pt4);
    pts->points.push_back(pt5);
    pts->points.push_back(pt6);
    pcl::Vertices vertB;
    vertB.vertices.push_back(i_int*6 +3); vertB.vertices.push_back(i_int*6+4); vertB.vertices.push_back(i_int*6+5);
    verts.push_back(vertB);
  }
  
  /* 
  std::cout << "blah\n";
  std::cout << verts[0] << " 0verts\n";
  std::cout << verts[1] << " 1verts\n";
  std::cout << pts->points[0] << " pts\n";
  */  
  mesh_ptr->polygons = verts;
  pcl::toROSMsg (*pts, mesh_ptr->cloud);  
  //std::cout << *mesh_ptr << "\n";
  return mesh_ptr;
}

pcl::PolygonMesh::Ptr rgbd_primitives::getCube(double xdim, double ydim, double zdim){ 
  pcl::PolygonMesh mesh;   
  pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh (mesh));  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts (new pcl::PointCloud<pcl::PointXYZRGB> ());
  std::vector <pcl::Vertices> verts;
  
  int i_int=0;
  
  // Top and Bottom:
  for (double z=0 ; z <= zdim; z=z+zdim, i_int++){
    pcl::PointXYZRGB pt1, pt2, pt3, pt4, pt5, pt6;
    pt1.x = 0;       pt1.y = 0;    pt1.z = z;  
    pt2.x = 0;       pt2.y = ydim; pt2.z = z; 
    pt3.x = xdim;    pt3.y = ydim; pt3.z = z;
    pt4.x = xdim ; pt4.y =ydim; pt4.z = z;
    pt5.x = xdim ; pt5.y =0   ; pt5.z = z;
    pt6.x =  0   ; pt6.y =0   ; pt6.z = z;
    set_color(pt1); set_color(pt2); set_color(pt3); set_color(pt4); set_color(pt5); set_color(pt6);
    pts->points.push_back(pt1);    pts->points.push_back(pt2);
    pts->points.push_back(pt3);    pts->points.push_back(pt4);
    pts->points.push_back(pt5);    pts->points.push_back(pt6);
    pcl::Vertices vert;
    vert.vertices.push_back(i_int*6 ); vert.vertices.push_back(i_int*6+1); vert.vertices.push_back(i_int*6+2);
    verts.push_back(vert);
    pcl::Vertices vertB;
    vertB.vertices.push_back(i_int*6 +3); vertB.vertices.push_back(i_int*6+4); vertB.vertices.push_back(i_int*6+5);
    verts.push_back(vertB);    
  }
  
  // Front and Back
  for (double x=0 ; x <= xdim; x=x+xdim, i_int++){
    pcl::PointXYZRGB pt1, pt2, pt3, pt4, pt5, pt6;
    pt1.z = 0;       pt1.y = 0   ; pt1.x = x;  
    pt2.z = 0;       pt2.y = ydim; pt2.x = x; 
    pt3.z = zdim;    pt3.y = ydim; pt3.x = x;
    pt4.z = zdim;    pt4.y =ydim ; pt4.x = x;
    pt5.z = zdim;    pt5.y =0    ; pt5.x = x;
    pt6.z =  0;      pt6.y =0    ; pt6.x = x;
    set_color(pt1); set_color(pt2); set_color(pt3); set_color(pt4); set_color(pt5); set_color(pt6);
    pts->points.push_back(pt1);    pts->points.push_back(pt2);
    pts->points.push_back(pt3);    pts->points.push_back(pt4);
    pts->points.push_back(pt5);    pts->points.push_back(pt6);
    pcl::Vertices vert;
    vert.vertices.push_back(i_int*6 ); vert.vertices.push_back(i_int*6+1); vert.vertices.push_back(i_int*6+2);
    verts.push_back(vert);
    pcl::Vertices vertB;
    vertB.vertices.push_back(i_int*6 +3); vertB.vertices.push_back(i_int*6+4); vertB.vertices.push_back(i_int*6+5);
    verts.push_back(vertB);    
  }  
  
  // Left and Right
  for (double y=0 ; y <= ydim; y=y+ydim, i_int++){
    pcl::PointXYZRGB pt1, pt2, pt3, pt4, pt5, pt6;
    pt1.x = 0;       pt1.z = 0  ;  pt1.y = y;  
    pt2.x = 0;       pt2.z =zdim;  pt2.y = y; 
    pt3.x = xdim;    pt3.z =zdim;  pt3.y = y;
    pt4.x = xdim ;   pt4.z =zdim;  pt4.y = y;
    pt5.x = xdim ;   pt5.z =0   ;  pt5.y = y;
    pt6.x =  0   ;   pt6.z =0   ;  pt6.y = y;
    set_color(pt1); set_color(pt2); set_color(pt3); set_color(pt4); set_color(pt5); set_color(pt6);
    pts->points.push_back(pt1);    pts->points.push_back(pt2);
    pts->points.push_back(pt3);    pts->points.push_back(pt4);
    pts->points.push_back(pt5);    pts->points.push_back(pt6);
    pcl::Vertices vert;
    vert.vertices.push_back(i_int*6 ); vert.vertices.push_back(i_int*6+1); vert.vertices.push_back(i_int*6+2);
    verts.push_back(vert);
    pcl::Vertices vertB;
    vertB.vertices.push_back(i_int*6 +3); vertB.vertices.push_back(i_int*6+4); vertB.vertices.push_back(i_int*6+5);
    verts.push_back(vertB);    
  }    
  
  /* 
  std::cout << "blah\n";
  std::cout << verts[0] << " 0verts\n";
  std::cout << verts[1] << " 1verts\n";
  std::cout << pts->points[0] << " pts\n";
  */  
  mesh_ptr->polygons = verts;
  pcl::toROSMsg (*pts, mesh_ptr->cloud);  
  //std::cout << *mesh_ptr << "\n";
  return mesh_ptr;
}




double areaOfTriangle(pcl::PointXYZRGB p0, pcl::PointXYZRGB p1, pcl::PointXYZRGB p2){
  // heron's formula
  double d01 = sqrt( pow(p0.x-p1.x,2) + pow(p0.y-p1.y,2) + pow(p0.z-p1.z,2) );
  double d02 = sqrt( pow(p0.x-p2.x,2) + pow(p0.y-p2.y,2) + pow(p0.z-p2.z,2) );
  double d12 = sqrt( pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2) + pow(p1.z-p2.z,2) );
  double p = ( d01 + d02 + d12 ) /2.0;
  return sqrt(p*(p- d01)*(p- d02)*(p- d12));
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbd_primitives::sampleMesh(pcl::PolygonMesh::Ptr &mesh, double pts_per_msquared){
  
  int N_polygonsB = mesh->polygons.size ();
  pcl::PointCloud<pcl::PointXYZRGB> cloudB;  
  pcl::fromROSMsg(mesh->cloud, cloudB);
  Eigen::Vector4f tmp;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts (new pcl::PointCloud<pcl::PointXYZRGB> ());
  for(size_t i=0; i< N_polygonsB; i++){ // each triangle/polygon N_polygonsB
    pcl::Vertices apoly_in = mesh->polygons[i];
    double area = areaOfTriangle(cloudB.points[ apoly_in.vertices[0] ] , 
                                          cloudB.points[ apoly_in.vertices[1] ] ,
                                          cloudB.points[ apoly_in.vertices[2] ]);
    //std::cout << area << "\n";
    // determine the number of points to sample on the surface:
    int n_pts = floor(area*pts_per_msquared + 0.5);// rounding at .5
    //std::cout << n_pts<< "\n";
    for (size_t i=0;i <n_pts; i++){
      pcl::PointXYZRGB     pt;
      pt = samplePointInTriangle(  cloudB.points[ apoly_in.vertices[0] ] , 
                                          cloudB.points[ apoly_in.vertices[1] ] ,
                                          cloudB.points[ apoly_in.vertices[2] ] );
      pts->points.push_back(pt);
    }
  } 
  pts->width = pts->points.size();
  pts->height = 1;
  
  return pts;
}

//% from wykobi library
// sample a random point inside a triangle
pcl::PointXYZRGB  rgbd_primitives::samplePointInTriangle(pcl::PointXYZRGB p0, pcl::PointXYZRGB p1,
                                             pcl::PointXYZRGB p2){
  double a = rand01(); // 0-1
  double b = rand01(); // 0-1
  if ((a + b) > 1){
    a=1-a;
    b=1-b;
  }
  double c = (1 - a - b);
  
  pcl::PointXYZRGB pt;
  pt.x = p0.x * a + p1.x * b + p2.x * c ;
  pt.y = p0.y * a + p1.y * b + p2.y * c ;
  pt.z = p0.z * a + p1.z * b + p2.z * c ;
  return pt;
}