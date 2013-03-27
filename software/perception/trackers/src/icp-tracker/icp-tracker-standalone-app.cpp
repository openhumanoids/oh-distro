// With an actual clipping:
// drc-icp-tracker-standalone-app -o hd_drill_at_zero.pcd  -n ../origi/sweep_cloud_305798000.pcd 

// With an actual alignment:
//drc-icp-tracker-standalone-app -o hd_drill_at_zero.pcd  -n ../a_moved.pcd 

// work in progress - v2


/*
capture a sweep
- with BB around previous position, clip the incoming point cloud
- (and possibly with position of a plane, remove the plane)
- run icp
*/

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/shared_ptr.hpp>

#include <lcm/lcm-cpp.hpp>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ConciseArgs>

#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h> //for computePointNormal

#include <pcl/filters/passthrough.h>
#include <pointcloud_tools/pointcloud_vis.hpp>

#include <trackers/icp-tracker.hpp>


#include <lcmtypes/drc_lcmtypes.hpp>

using namespace Eigen;
using namespace std;
using namespace boost;

class StatePub
{
public:
  StatePub(boost::shared_ptr<lcm::LCM> &lcm_, string new_cloud_filename, string previous_cloud_filename);
  ~StatePub() {}
  boost::shared_ptr<lcm::LCM> lcm_;
  
private:
  pointcloud_vis* pc_vis_;
  
  bool readPCD(string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);


  ICPTracker* icp_tracker_;

  void boxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, Eigen::Isometry3f pose,
     Eigen::Vector3f bb_ll, Eigen::Vector3f bb_ur);

  void removePoseOffset(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &previous_cloud, 
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_cloud, 
			Eigen::Isometry3f previous_pose );

  bool doICP( pcl::PointCloud<pcl::PointXYZRGB>::Ptr &previous_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_cloud,
			Eigen::Matrix4f & tf_previous_to_new);


  drc::affordance_t getAffordance(Eigen::Isometry3d &pose, int uid);

  Isometry3dTime null_poseT_;    
};

StatePub::StatePub(boost::shared_ptr<lcm::LCM> &lcm_, std::string new_cloud_filename, std::string previous_cloud_filename):
    lcm_(lcm_),null_poseT_(0, Eigen::Isometry3d::Identity()){
  
  pc_vis_ = new pointcloud_vis(lcm_->getUnderlyingLCM());

  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  float colors_b[] ={1.0,0.0,1.0};
  std::vector<float> colors_v;
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(349995,"Vehicle Tracker - Null",5,1) );
  // 7 = full | 2 wireframe

  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"[ICPApp] Pose - Null",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"[ICPApp] Prev Cloud (at Prev Pose)"     ,1,1, 1005,1, {1,0,0}));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1002,"[ICPApp] New Cloud"     ,1,1, 1000,1, {0,0,1}));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1005,"[ICPApp] Pose - Previous",5,0) );
      

  icp_tracker_ = new ICPTracker(lcm_, 2);
  
  
  
  // 1. Read Previous Point Cloud and Define Pose and Bounding Box:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr previous_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  readPCD(previous_cloud_filename,previous_cloud);

  Eigen::Isometry3d previous_pose= Eigen::Isometry3d::Identity();
  //    <include><uri>model://mit_cordless_drill</uri><pose>1.3 1.3 1.02 0 0 -1.571</pose></include>
  Eigen::Quaterniond quat = Eigen::Quaterniond(1,0,0,0);
  previous_pose.translation()  << 1.275, 1.3, 1.16277;
  previous_pose.rotate(quat);
  Eigen::Vector3f boundbox_lower_left = Eigen::Vector3f(-0.15, -0.07, -0.135); //-0.135
  Eigen::Vector3f boundbox_upper_right = Eigen::Vector3f( 0.15, 0.18,  0.2);
    
  if(1==1){
    Eigen::Isometry3d previous_pose= Eigen::Isometry3d::Identity();
    //    <include><uri>model://mit_cordless_drill</uri><pose>1.3 1.3 1.02 0 0 -1.571</pose></include>
    Eigen::Quaterniond quat = Eigen::Quaterniond(1,0,0,0);
    previous_pose.translation()  <<0.5, -1.3, 1.16277;
    previous_pose.rotate(quat);
    Eigen::Vector3f boundbox_lower_left = Eigen::Vector3f(-0.15, -0.07, -0.135); //-0.135
    Eigen::Vector3f boundbox_upper_right = Eigen::Vector3f( 0.15, 0.18,  0.2);
  }  
  
  icp_tracker_->setBoundingBox (boundbox_lower_left, boundbox_upper_right);  

  Isometry3dTime previous_poseT = Isometry3dTime(0, previous_pose); 
  pc_vis_->pose_to_lcm_from_list(1005, previous_poseT);
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT_);
  pc_vis_->ptcld_to_lcm_from_list(1001, *previous_cloud, null_poseT_.utime, null_poseT_.utime);

  // 2. Read new point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  readPCD(new_cloud_filename,new_cloud);
  pc_vis_->ptcld_to_lcm_from_list(1002, *new_cloud, null_poseT_.utime, null_poseT_.utime);



  icp_tracker_->doICPTracker( previous_cloud, new_cloud, previous_pose );
  Eigen::Isometry3d new_pose = icp_tracker_->getUpdatedPose();


  drc::affordance_t previous_aff = StatePub::getAffordance(previous_pose, 0);
  drc::affordance_t new_aff = StatePub::getAffordance(new_pose, 1);
  drc::affordance_collection_t aff_coll;
  aff_coll.name  = "Map Name";
  aff_coll.utime = 0;
  aff_coll.map_id =0;
  aff_coll.affs.push_back( previous_aff);
  aff_coll.affs.push_back( new_aff);
  aff_coll.naffs =2;

  lcm_->publish("AFFORDANCE_COLLECTION",&aff_coll);

}



drc::affordance_t StatePub::getAffordance(Eigen::Isometry3d &pose, int uid){ 

    Eigen::Quaterniond r(pose.rotation());
    double yaw, pitch, roll;
    quat_to_euler(r, yaw, pitch, roll);    
    double x,y,z;
    x = pose.translation().x() ;
    y = pose.translation().y() ;
    z = pose.translation().z() ;

    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =uid;
    a.otdf_type ="cylinder";
    a.aff_store_control = drc::affordance_t::NEW;
    a.nparams =9;

    a.param_names.push_back("x");
    a.params.push_back(x);
    a.param_names.push_back("y");
    a.params.push_back(y);
    a.param_names.push_back("z");
    a.params.push_back(z);

    a.param_names.push_back("roll");
    a.params.push_back( roll );
    a.param_names.push_back("pitch");
    a.params.push_back( pitch);
    a.param_names.push_back("yaw");
    a.params.push_back( yaw );

    a.param_names.push_back("radius");
    a.params.push_back(0.020000);
    a.param_names.push_back("length");
    a.params.push_back(0.13);
    a.param_names.push_back("mass");
    a.params.push_back(1.0); // unknown
    a.nstates =0;
    a.npoints =0;
    a.nvertices=0;

  return a;
}    


bool StatePub::readPCD(string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ( filename, *cloud) == -1){ // load the file
    cout << "Couldn't read " << filename << "\n";
    return (false);
  } 
  cout << "read a cloud with " << cloud->points.size() << " points\n";
  return true;
}


void StatePub::removePoseOffset(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &previous_cloud, 
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_cloud, 
				Eigen::Isometry3f previous_pose ){
  Eigen::Quaternionf pose_quat(previous_pose.rotation());
  pcl::transformPointCloud (*previous_cloud, *previous_cloud,
        previous_pose.translation(), pose_quat); // !! modifies lidar_cloud
  pcl::transformPointCloud (*new_cloud, *new_cloud,
        previous_pose.translation(), pose_quat); // !! modifies lidar_cloud

  pc_vis_->ptcld_to_lcm_from_list(1011, *previous_cloud, null_poseT_.utime, null_poseT_.utime);
  pc_vis_->ptcld_to_lcm_from_list(1012, *new_cloud, null_poseT_.utime, null_poseT_.utime);

}


bool StatePub::doICP( pcl::PointCloud<pcl::PointXYZRGB>::Ptr &previous_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_cloud, Eigen::Matrix4f & tf_previous_to_new){
    //iterative closest point : setup inputs
    IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
    icp.setInputTarget( previous_cloud );
    icp.setInputCloud( new_cloud );

    //outputs
    //icp.setMaxCorrespondenceDistance(0.05);

    PointCloud<PointXYZRGB>::Ptr downsampled_output (new PointCloud<PointXYZRGB>);
    icp.align(*downsampled_output);

    //icp.align(*_object_to_track); //*newest_cloud_pcl); //*_object_to_track;

    //---transform: apply transform from the downsampled output to the full-res _object_to_track
    tf_previous_to_new = icp.getFinalTransformation().inverse();
    //cout << tf_previous_to_new << " final transform\n";

    pcl::transformPointCloud(*previous_cloud, *previous_cloud, tf_previous_to_new);
}



void StatePub::boxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, Eigen::Isometry3f pose,
     Eigen::Vector3f bb_ll, Eigen::Vector3f bb_ur){
  
  Eigen::Isometry3f pose_i = pose.inverse();
  Eigen::Quaternionf quat_i(pose_i.rotation());
  pcl::transformPointCloud (*cloud, *cloud,
        pose_i.translation(), quat_i); // !! modifies lidar_cloud


  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (bb_ll(0), bb_ur(0) ); //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (bb_ll(1), bb_ur(1) ); //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);
  
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits ( bb_ll(2), bb_ur(2)); //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);


  Eigen::Quaternionf quat(pose.rotation());
  pcl::transformPointCloud (*cloud, *cloud,
        pose.translation(), quat); // !! modifies lidar_cloud

/*
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (1.15, 1.6); //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (1, 1.6); //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);
  
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (1.05, 3.5); //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);
  */
}


int main (int argc, char ** argv){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  string previous_cloud_filename = "old_cloud.pcd";
  string new_cloud_filename = "new_cloud.pcd";
  parser.add(previous_cloud_filename, "o", "previous_cloud_filename", "previous_cloud_filename");
  parser.add(new_cloud_filename, "n", "new_cloud_filename", "new_cloud_filename");
  parser.parse();
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  StatePub app(lcm, new_cloud_filename, previous_cloud_filename);
  cout << "StatePub ready"<< endl;
  return 0;
}
