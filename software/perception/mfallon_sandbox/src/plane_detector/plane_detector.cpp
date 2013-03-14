#include <iostream>

#include <boost/thread.hpp>
#include <boost/assign/std/vector.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


#include <maps/LocalMap.hpp>
#include <maps/MapWrapper.hpp>


#include <pointcloud_tools/pointcloud_vis.hpp>
#include <pointcloud_tools/filter_planes.hpp>

using namespace std;


#define verbose_txt 0
// 0 means publish nothing
// 1 means publish very important only [runs without issue]
// 2 means publish important
// 3 means publish all
#define verbose_lcm 3 // was 0

class Detector {
public:
  Detector(boost::shared_ptr<MapWrapper> iWrapper):
          null_poseT_(0, Eigen::Isometry3d::Identity()) {
    mWrapper = iWrapper;
    
  cloud_timestamp_ = 0;
  plane_fitter_id_ =1;
  publish_lcm_ = lcm_create(NULL);
  
  
  pc_vis_ = new pointcloud_vis(publish_lcm_);
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Null",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Cloud - Laser Map"     ,1,1, 1000,1, {0,0,1}));
  
    

  // initial values of floor coeff which correspond to holding out in front:
  last_floor_coeff_.values = {-0.089914, -0.00691448, 0.995926, 0.769824} ;
  
  /*
  last_floor_coeff_.values.push_back(-0.089914);
  last_floor_coeff_.values.push_back(-0.00691448);
  last_floor_coeff_.values.push_back(0.995926);
  last_floor_coeff_.values.push_back(0.769824);  
  */
  
  }
  
  void write_pcd(maptypes::PointCloud::Ptr &cloud,std::ofstream &fstream );
  
  void find_floor_with_aligned_cloud(pcl::PointCloud<PointXYZRGB>::Ptr &cloud_global);
  
  void find_planes (pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
  
  void visualize_planes(vector<BasicPlane> &plane_stack );
  
  void operator ()() {
    while (true) {
      maptypes::PointCloud::Ptr cloud;
      mWrapper->lock();
      if (mWrapper->getMap() != NULL) {
        cloud = mWrapper->getMap()->getAsPointCloud();
      }
      mWrapper->unlock();

      cout << "Grabbed a point cloud of " << cloud->points.size() <<
        " points from octree" << endl;
        
      if (cloud->points.size() >1){
        ofstream outfile ("test.pcd");
        write_pcd( cloud,outfile);

        find_floor_with_aligned_cloud(cloud);
    
        //find_planes (cloud);
        
        cout << "done\n";
      }

      sleep(2);
    }
  }
  
private:
  int64_t cloud_timestamp_;
  int plane_fitter_id_; // id for visualizing plane fitting collections
  lcm_t * publish_lcm_;
  pointcloud_vis* pc_vis_;
  Isometry3dTime null_poseT_;

  
  pcl::ModelCoefficients last_floor_coeff_;
    // The precise adjustment of the pose required to aligne the floor in the cloud with horizontal:
    double precise_height_, precise_pitch_ , precise_roll_;
  
  

protected:
  boost::shared_ptr<MapWrapper> mWrapper;
};



void Detector::write_pcd(maptypes::PointCloud::Ptr &cloud,std::ofstream &fstream ){
  fstream << "# .PCD v.5 - Point Cloud Data file format" << endl;
  fstream << "FIELDS x y z" << endl;
  fstream << "SIZE 4 4 4" << endl;
  fstream << "TYPE F F F" << endl;
  fstream << "WIDTH " << cloud->points.size() << endl;
  fstream << "HEIGHT 1" << endl;
  fstream << "POINTS " << cloud->points.size() << endl;
  fstream << "DATA ascii" << endl;

  for (size_t i=0; i < cloud->points.size(); i++){
    pcl::PointXYZRGB lr = cloud->points[i];
    ostringstream temp0;
    temp0 << lr.x <<" " << lr.y<< " " << lr.z << endl;
    fstream << temp0.str();
  }
  fstream.close();
    cout << "finished writing points: "<< cloud->points.size() << endl;
}

void Detector::visualize_planes(vector<BasicPlane> &plane_stack ){
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





void Detector::find_floor_with_aligned_cloud(pcl::PointCloud<PointXYZRGB>::Ptr &cloud_filtered){
  // MODUS OPERANDUS:
  // given previous estimate of ground and accurate RPY

  std::cerr << "[IN] : " << cloud_filtered->points.size()<< " data points." << std::endl;
  null_poseT_.utime = cloud_timestamp_;
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT_);
  // visualize raw input cloud:
  //pc_vis_->ptcld_to_lcm_from_list(1001, *cloud_filtered, cloud_timestamp_, cloud_timestamp_);
  
  // 1. Find Planes in cloud:
  FilterPlanes filtp;
  filtp.setInputCloud(cloud_filtered);
  filtp.setPoseIDs(plane_fitter_id_,cloud_timestamp_);
  filtp.setLCM(publish_lcm_,0); // 0 means no publish to lcm, higher does
  filtp.setDistanceThreshold(0.04);
  filtp.setStopProportion(0.60); 
  vector<BasicPlane> plane_stack;
  filtp.filterPlanes(plane_stack);
  std::cout << "[OUT] number of planes extracted: " << plane_stack.size() << "\n";
  // visualize plane stack:
  //visualize_planes(plane_stack);

  // 2. Find the ground plane in the plane set:
  int ground_id = -1;
  for (size_t  i=0 ; i <plane_stack.size() ; i ++ ){
    pcl::ModelCoefficients coeff = plane_stack[i].coeffs;
    double pitch = atan(coeff.values[0]/coeff.values[2]);
    double roll =- atan(coeff.values[1]/coeff.values[2]);
    double coeff_norm = sqrt(pow(coeff.values[0],2) +
    pow(coeff.values[1],2) + pow(coeff.values[2],2));
    double height = (coeff.values[2]*coeff.values[3]) / coeff_norm;

    //  cout  <<  "\nRANSAC Floor Coefficients: " << coeff.values[0]
    //    << " " << coeff.values[1] << " "  << coeff.values[2] << " " << coeff.values[3] << endl;
    //  cout << "Pitch: " << pitch << " (" << (pitch*180/M_PI) << "d). positive nose down\n";
    //  cout << "Roll : " << roll << " (" << (roll*180/M_PI) << "d). positive right side down\n";
    //  cout << "Height : " << height << " of device off ground [m]\n";
    
    // quit if we find a plane fitting the model

    // 0.1 rads or 5.7 degrees is pretty tight for a walking platform but usually works on fixed
    //if ((fabs(pitch) < 0.1) && (fabs(roll) < 0.1)){
    if ((fabs(pitch) < 0.2) && (fabs(roll) < 0.2)){
      cout << "GROUND PLANE FOUND\n";
      ground_id = (int) i;
      break;
    }
  }

  pcl::ModelCoefficients floor_coeff;
  if (ground_id<0){
    cout << "NO GROUND PLANE FOUND - using last Height|P|R\n";
    floor_coeff = last_floor_coeff_;
  }else{
    // 1. Using precise height pitch and roll, align the cloud
    floor_coeff = plane_stack[ground_id].coeffs;
  }

  precise_pitch_ = atan(floor_coeff.values[0]/floor_coeff.values[2]);
  precise_roll_ =- atan(floor_coeff.values[1]/floor_coeff.values[2]);
  double coeff_norm = sqrt(pow(floor_coeff.values[0],2) + pow(floor_coeff.values[1],2) + pow(floor_coeff.values[2],2));
  precise_height_ = (floor_coeff.values[2]*floor_coeff.values[3]) / coeff_norm;
  cout << "Pitch: " << precise_pitch_ << " (" << (precise_pitch_*180/M_PI) << "d). positive nose down\n";
  cout << "Roll : " << precise_roll_ << " (" << (precise_roll_*180/M_PI) << "d). positive right side down\n";
  cout << "Height : " << precise_height_ << " of device off ground [m]\n";
  last_floor_coeff_ = floor_coeff;

  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_global_x (new pcl::PointCloud<pcl::PointXYZRGB> ());
  //  Eigen::Vector3f world_trans(0 , 0 ,  precise_height_ );
  //  Eigen::Quaternionf world_rot = euler_to_quat_f(0, -precise_pitch_, -precise_roll_); // heading
  //  pcl::transformPointCloud (*cloud_filtered, *cloud_global_x,world_trans, world_rot);
  cloud_global_x = cloud_filtered ;

  // 2. Extract only the obstacles by excluding the floor:
  // TODO use pcl filter for this instead
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  for (size_t i=0;i < cloud_global_x->points.size() ; i ++){

    // any thing  withing this height is consider to be floor, else obstacle
    double height_thres =0.125; // was 0.025
    
    double dist_to_ground = pcl::pointToPlaneDistance    (  cloud_global_x->points[i],
                     floor_coeff.values[0], floor_coeff.values[1], floor_coeff.values[2], floor_coeff.values[3]) ; 
    
    if ( dist_to_ground  >  height_thres){
      // alternative: pcl::pointToPlaneDistance(pt,a,b,c,d)
      inliers->indices.push_back(i);
    }
  }
  
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr occupied_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  
  pcl::ExtractIndices<PointXYZRGB> extract;
  extract.setInputCloud (cloud_global_x);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*occupied_cloud);
  extract.setNegative (true);
  extract.filter (*floor_cloud);

  if (verbose_lcm > 1){ // quite important

    ptcld_cfg pcfg1 = ptcld_cfg(50112,    "Occupied w/o floor [Global Frame]" ,1,1, 1000,1,{1.0,0.0,0.0} );
    pc_vis_->ptcld_to_lcm(pcfg1, *occupied_cloud, null_poseT_.utime, null_poseT_.utime );        

    ptcld_cfg pcfg2 = ptcld_cfg(50113,    "Floor w/o occupied [Global Frame]" ,1,1, 1000,1,{0.0,1.0,0.0} );
    pc_vis_->ptcld_to_lcm(pcfg2, *floor_cloud, null_poseT_.utime, null_poseT_.utime );        
    
    /*
    floorcoll_cfg.id =50112;
    floorcoll_cfg.name.assign("Occupied w/o floor [Global Frame]");
    floorcoll_cfg.element_id = cloud_utime;
    floorcoll_cfg.collection = 3001;
    floorcoll_cfg.npoints = occupied_cloud->points.size();
    pcdXYZRGB_to_lcm(publish_lcm_,floorcoll_cfg, *occupied_cloud);

    floorcoll_cfg.id =51110113;
    floorcoll_cfg.name.assign("Floor w/o occupied [Global Frame]");
    floorcoll_cfg.element_id = cloud_utime;
    floorcoll_cfg.collection = 3001;
    floorcoll_cfg.npoints = floor_cloud->points.size();
    pcdXYZRGB_to_lcm(publish_lcm_,floorcoll_cfg, *floor_cloud);*/
  }
  
  

}




void Detector::find_planes (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered){
  // 1. Read input:
  std::cerr << "[IN] : " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
  null_poseT_.utime = cloud_timestamp_;
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT_);
  pc_vis_->ptcld_to_lcm_from_list(1001, *cloud_filtered, cloud_timestamp_, cloud_timestamp_);

  
  
  // 2. Extract the major planes and send them to lcm:
  FilterPlanes filtp;
  filtp.setInputCloud(cloud_filtered);
  filtp.setPoseIDs(plane_fitter_id_,cloud_timestamp_);
  filtp.setLCM(publish_lcm_);
  filtp.setDistanceThreshold(0.04);
  filtp.setStopProportion(0.10); 
  vector<BasicPlane> plane_stack; 
  filtp.filterPlanes(plane_stack);
  std::cout << "[OUT] number of planes extracted: " << plane_stack.size() << "\n";
  visualize_planes(plane_stack);
  
  
}



int main(const int iArgc, const char** iArgv) {
  boost::shared_ptr<lcm::LCM> theLcm(new lcm::LCM());
  if (!theLcm->good()) {
    cerr << "Cannot create lcm instance." << endl;
    return -1;
  }

  boost::shared_ptr<MapWrapper> wrapper(new MapWrapper());
  wrapper->setLcm(theLcm);
  wrapper->setMapChannel("LOCAL_MAP");
  wrapper->start();

  Detector action(wrapper);
  boost::thread thread(boost::ref(action));

  while (0 == theLcm->handle());

  wrapper->stop();

  return 0;
}
