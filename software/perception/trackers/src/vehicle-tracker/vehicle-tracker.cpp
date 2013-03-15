//measured:
//- clip cloud 2m^3 in front
//mesh:
//x,y,z,nx,ny,nz,[convex hull], with 50points


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/bot_core.hpp"

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ConciseArgs>


#include <pcl/filters/passthrough.h>


#include <rgbd_simulation/rgbd_primitives.hpp> // to create basic meshes
#include <pointcloud_tools/pointcloud_vis.hpp>

#include <pointcloud_tools/filter_planes.hpp>

using namespace Eigen;
using namespace std;
using namespace boost;

double yaw=0;
double x=0;
double y=5;
double z=0;

struct Primitive
{
  std::string link;
  std::string type;
  std::vector<float> params;
};


class StatePub
{
public:
  StatePub(boost::shared_ptr<lcm::LCM> &lcm_, string pcd_filename, string urdf_file, int pts_per_m_squared_);
  ~StatePub() {}
  boost::shared_ptr<lcm::LCM> lcm_;
  
private:
    pcl::PolygonMesh::Ptr prim_mesh_ ;
    rgbd_primitives*  prim_;
      pointcloud_vis* pc_vis_;

  
  bool readURDFString(std::string urdf_file, std::string &urdf_string);
  bool readModel(std::string model_filename, std::vector<Primitive> &primitives);
  void primitivesToMesh(std::vector<Primitive> &primitives);
  bool readPCD();
  
  void transformMesh();
  void sendMesh();
  
  void boxFilter();
  
  void findPlanes (pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
  
  void visualizePlanes(vector<BasicPlane> &plane_stack );
  

  string pcd_filename_;
  
  pcl::PointCloud<PointXYZRGB>::Ptr _cloud;    
  Isometry3dTime null_poseT_;  
  
  int pts_per_m_squared_;
  
};

StatePub::StatePub(boost::shared_ptr<lcm::LCM> &lcm_, std::string pcd_filename_, string urdf_file, int pts_per_m_squared_):
    lcm_(lcm_), pcd_filename_(pcd_filename_), pts_per_m_squared_(pts_per_m_squared_),null_poseT_(0, Eigen::Isometry3d::Identity()){
  
  pc_vis_ = new pointcloud_vis(lcm_->getUnderlyingLCM());
  prim_ = new rgbd_primitives();

  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  float colors_b[] ={1.0,0.0,1.0};
  std::vector<float> colors_v;
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(349995,"Vehicle Tracker - Null",5,1) );
  // 7 = full | 2 wireframe
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(349996,"Vehicle Tracker - Prior Mesh"     ,3,1, 349995,0, colors_v ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(349997,"Vehicle Tracker - Prior Points"  ,1,1, 349995,0, colors_v ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(349998,"Vehicle Tracker - Lidar Points"  ,1,1, 349995,1, colors_v ));
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Null",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Cloud - Laser Map"     ,1,1, 1000,1, {0,0,1}));
  
      
  std::vector<Primitive> primitives;
  readModel( urdf_file, primitives );
  primitivesToMesh( primitives );
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  _cloud = cloud_ptr ;  
  readPCD();

  transformMesh();
  sendMesh();
}

void StatePub::primitivesToMesh(std::vector<Primitive> &primitives){
  pcl::PolygonMesh::Ptr aff_mesh_temp(new pcl::PolygonMesh());
  prim_mesh_ = aff_mesh_temp;
  for (int i=0; i < primitives.size(); i++){
    Eigen::Isometry3d transform;
    transform.setIdentity();
    transform.translation()  << primitives[i].params[0] , primitives[i].params[1] , primitives[i].params[2];
    Eigen::Quaterniond quat = euler_to_quat( primitives[i].params[5],  primitives[i].params[4],  primitives[i].params[3]);
    transform.rotate(quat);  
    
    pcl::PolygonMesh::Ptr mesh_ptr_temp(new pcl::PolygonMesh());
    if (primitives[i].type.compare( "box" ) == 0){
      mesh_ptr_temp = prim_->getCubeWithTransform(transform, primitives[i].params[6], primitives[i].params[7], primitives[i].params[8]);
    }else if (primitives[i].type.compare( "cylinder" ) == 0){
      mesh_ptr_temp = prim_->getCylinderWithTransform(transform, primitives[i].params[6], primitives[i].params[6], primitives[i].params[7]);
    }
    pc_vis_->mergePolygonMesh(aff_mesh_temp , mesh_ptr_temp);
  }
}


void StatePub::transformMesh(){
    Eigen::Isometry3d transform;
    transform.setIdentity();
    transform.translation()  << -0.2 +x ,-2.5 + y,0 +z;
    Eigen::Quaterniond quat = euler_to_quat( M_PI + yaw,0,0);
    transform.rotate(quat);  

  
  
    pcl::PointCloud<pcl::PointXYZRGB> mesh_cloud_1st;  
    pcl::fromROSMsg(prim_mesh_->cloud, mesh_cloud_1st);

    // transform
    Eigen::Isometry3f pose_f_1st =isometryDoubleToFloat(transform);
    Eigen::Quaternionf pose_quat_1st(pose_f_1st.rotation());
    pcl::transformPointCloud (mesh_cloud_1st, mesh_cloud_1st, pose_f_1st.translation(), pose_quat_1st);  
    pcl::toROSMsg (mesh_cloud_1st, prim_mesh_->cloud);  
}


void StatePub::sendMesh(){
  // Visualise:
  int64_t pose_id =0;
  Eigen::Isometry3d null_pose;
  null_pose.setIdentity();
  Isometry3dTime null_poseT = Isometry3dTime(pose_id, null_pose);
  pc_vis_->pose_to_lcm_from_list(349995, null_poseT);
  pc_vis_->mesh_to_lcm_from_list(349996, prim_mesh_, pose_id , pose_id);

  
  // Sample the mesh to get a point cloud:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampled_pts =prim_->sampleMesh(prim_mesh_, pts_per_m_squared_); // no. of points per m^2
    if(1==1){
      if (sampled_pts->points.size() > 0) {
      cout << sampled_pts->points.size() << " sampled points output\n";
        pcl::PCDWriter writer;
        stringstream ss2;
        ss2 << "vehicle_sampled.pcd";
        writer.write (ss2.str(), *sampled_pts, false);
      }
    }
  
  pc_vis_->ptcld_to_lcm_from_list(349997, *sampled_pts, pose_id , pose_id);  
  pc_vis_->ptcld_to_lcm_from_list(349998, *_cloud, pose_id , pose_id);  

  boxFilter();
  cout << _cloud->points.size() << " cloud size\n";
  findPlanes (_cloud);

}


void StatePub::boxFilter(){

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (_cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-1.5, 1.5); //pass.setFilterLimitsNegative (true);
  pass.filter (*_cloud);

  pass.setInputCloud (_cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-3.5, -1.5); //pass.setFilterLimitsNegative (true);
  pass.filter (*_cloud);
  
  
}



void StatePub::visualizePlanes(vector<BasicPlane> &plane_stack ){
  // 3. Visualise normals:
  for (size_t i = 0; i< plane_stack.size() ; i++){
    BasicPlane plane = plane_stack[i];
    stringstream name_out;
    name_out << "local_polygons_major_" << plane.major << "_minor_" << plane.minor;
    int plane_id = 290+ i ;
    ptcld_cfg pcfg = ptcld_cfg(plane_id,    name_out.str()     ,3,1, 1000,1,{-1,-1,-1} );
    pc_vis_->ptcld_to_lcm(pcfg, (plane.cloud), null_poseT_.utime, null_poseT_.utime );
  } 
  
  for (size_t i=0;i<plane_stack.size();i++){
    BasicPlane plane = plane_stack[i];
    if (plane.cloud.points.size() ==0 ){
      cout <<"ERROR: CONVEX HULL HAS NO POINTS! - NEED TO RESOLVE THIS\n"; 
      continue;
    }
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
    pc_vis_->ptcld_to_lcm(pcfg2, *normals_cloud, null_poseT_.utime, null_poseT_.utime );        
  } 
}


void StatePub::findPlanes (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered){
  // 1. Read input:
  std::cerr << "[IN] : " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
  //null_poseT_.utime = 0;//cloud_timestamp_;
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT_);
  pc_vis_->ptcld_to_lcm_from_list(1001, *cloud_filtered, null_poseT_.utime, null_poseT_.utime);

  int plane_fitter_id_ =1;
  
  // 2. Extract the major planes and send them to lcm:
  FilterPlanes filtp;
  filtp.setInputCloud(cloud_filtered);
  filtp.setPoseIDs(plane_fitter_id_,null_poseT_.utime);
  filtp.setLCM(lcm_->getUnderlyingLCM());
  filtp.setDistanceThreshold(0.02); // simulated lidar
  filtp.setStopProportion(0.10); 
  vector<BasicPlane> plane_stack; 
  filtp.filterPlanes(plane_stack);
  std::cout << "[OUT] number of planes extracted: " << plane_stack.size() << "\n";
  visualizePlanes(plane_stack);
  
  GrowCloud grow;
  std::stringstream ss;
  grow.printPlaneStackCoeffs(plane_stack, ss);
  cout << ss.str();

  std::stringstream ss2;
  grow.printPlaneStackHull(plane_stack, ss2);
  cout << ss2.str();
}



bool StatePub::readModel(std::string model_filename, std::vector <Primitive> &primitives){
  
  string line;
  ifstream myfile (model_filename.c_str());
  if (myfile.is_open()){
    while ( myfile.good() ){
      getline (myfile,line);
      if (line.size() > 4){
        char_separator<char> sep(",");
        tokenizer< char_separator<char> > tokens(line, sep);
        int count=0;
        vector <float> params;
        string collision_type;
        string link;
        BOOST_FOREACH (const string& t, tokens) {
          if (count==0){
            link = t;
          }else if(count==1){
            collision_type = t;
          }else{
            params.push_back( atof ( t.c_str() ) );
          }
          count++;
        }
        cout << "link ["<< link <<"] type [" << collision_type << "] params [";
        for (size_t i=0; i <params.size() ; i++){
          cout << params[i] << ", " ;
        }
        cout << "]\n";
        Primitive primitive;
        primitive.link = link;
        primitive.type =collision_type;
        primitive.params = params;
        primitives.push_back(primitive);
      }
    }
    myfile.close();
  } else{
    printf( "Unable to open scans file\n%s",model_filename.c_str());
    return false;
  }
  
  return true;
}

bool StatePub::readPCD(){
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ( pcd_filename_, *_cloud) == -1){ // load the file
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (false);
  } 
  return true;
}


int main (int argc, char ** argv){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  string filename = "/home/mfallon/drc/software/perception/trackers/data/car_simulator/chassis.txt";
  string pcd_filename = "/home/mfallon/drc/software/perception/trackers/data/car_simulator/vehicle_200000000.pcd";
  int pts_per_m_squared = 2000; // was 1000
  parser.add(filename, "f", "filename", "filename");
  parser.add(pcd_filename, "p", "pcd_filename", "pcd_filename");
  parser.add(pts_per_m_squared, "d", "density_of_points", "Density of points (per m^2)");
  parser.add(x, "x", "x", "x [m]");
  parser.add(y, "y", "y", "y [m]");
  parser.add(z, "z", "z", "z [m]");
  parser.add(yaw, "r", "rotation", "Yaw [rads]");
  parser.parse();
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  StatePub app(lcm, pcd_filename, filename, pts_per_m_squared);
  cout << "StatePub ready"<< endl;
//  while(0 == lcm->handle());
  return 0;
}
