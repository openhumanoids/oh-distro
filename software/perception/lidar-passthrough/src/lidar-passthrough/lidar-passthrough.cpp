// Lidar Pass trough filter
// TODO/Bugs:
// - Timing of bullet collision detection seems quasi random.
//   difficult to see how to optimize this as a result
// - Collision is done by intersecting spheres with the model. The spheres size is set in collision_object_point_cloud.cc
//   as of march 2013 it was increased from 0.01m to 0.04m

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>

#include <lcm/lcm-cpp.hpp>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>

#include <model-client/model-client.hpp>

#include <collision/collision.h>
#include <collision/collision_detector.h>
#include <collision/collision_object_gfe.h>
#include <collision/collision_object_point_cloud.h>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <pointcloud_tools/pointcloud_lcm.hpp> // unpack lidar to xyz
#include "lcmtypes/bot_core.hpp"
#include <ConciseArgs>

using namespace std;
using namespace drc;
using namespace Eigen;
using namespace collision;
using namespace boost::assign; // bring 'operator+()' into scope

// all ranges shorter than this are assumed to be with the head
#define ASSUMED_HEAD 0.3//0.3
// all ranges longer than this are assumed to be free
#define ASSUMED_FAR 2.0// 2.0
// set all collisions to this range
#define COLLISION_RANGE 0.0

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string lidar_channel_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    boost::shared_ptr<ModelClient> model_;
    bool verbose_;
    std::string lidar_channel_;
    
    void urdfHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_urdf_t* msg);
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg);   
    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);   
    
    Collision_Object_GFE* collision_object_gfe_;
    Collision_Object_Point_Cloud* collision_object_point_cloud_;
    Collision_Detector* collision_detector_;
    int n_collision_points_;
    
    void DoCollisionCheck(int64_t current_utime);
    
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* frames_cpp_;
    bot_lcmgl_t* lcmgl_;
    
    pointcloud_vis* pc_vis_;
    pointcloud_lcm* pc_lcm_;
    int vis_counter_; // used for visualization
    int printf_counter_; // used for terminal feedback
    
    bool urdf_parsed_;
    bool urdf_subscription_on_;
    lcm::Subscription *urdf_subscription_; //valid as long as urdf_parsed_ == false
    // Last robot state: this is used as the collision gfe/robot
    drc::robot_state_t last_rstate_;
    bool init_rstate_;
    // Scan as pointcloud in local frame: this is used as the collision points
    pcl::PointCloud<PointXYZRGB>::Ptr scan_cloud_s2l_;    
    
    // Output filter lidar:
    bot_core::planar_lidar_t lidar_msgout_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string lidar_channel_):
    lcm_(lcm_), verbose_(verbose_), 
    lidar_channel_(lidar_channel_),urdf_parsed_(false){
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  
  // TODO: get the urdf model from LCM:
  collision_object_gfe_ = new Collision_Object_GFE( "collision-object-gfe", model_->getURDFString() );
  n_collision_points_ = 1081; // was 1000, real lidar from sensor head has about 1081 returns (varies)
  collision_object_point_cloud_ = new Collision_Object_Point_Cloud( "collision-object-point-cloud", n_collision_points_ );
  // create the collision detector
  collision_detector_ = new Collision_Detector();
  // add the two collision objects to the collision detector with different groups and filters (to prevent checking of self collisions)
  collision_detector_->add_collision_object( collision_object_gfe_, COLLISION_DETECTOR_GROUP_1, COLLISION_DETECTOR_GROUP_2 );
  collision_detector_->add_collision_object( collision_object_point_cloud_, COLLISION_DETECTOR_GROUP_2, COLLISION_DETECTOR_GROUP_1 );   
  
  
  lcmgl_= bot_lcmgl_init(lcm_->getUnderlyingLCM(), "lidar-pt");
  lcm_->subscribe( lidar_channel_ ,&Pass::lidarHandler,this);
  lcm_->subscribe("EST_ROBOT_STATE",&Pass::robotStateHandler,this);
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60000,"Pose - Laser",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60001,"Cloud - Laser"         ,1,0, 60000,1, {0.0, 0.0, 1.0} ));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Null",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Cloud - Null"           ,1,0, 1000,1, { 0.0, 1.0, 0.0} ));
  vis_counter_ =0;  
  printf_counter_ =0;
  
  init_rstate_ =false;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_cloud_s2l_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  scan_cloud_s2l_ = scan_cloud_s2l_ptr;  
  cout << "Finished setting up\n";
}


// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


void Pass::DoCollisionCheck(int64_t current_utime ){
  // 1. create the list of points to be considered:
  vector< Vector3f > points;
  std::vector<unsigned int> possible_indices; // the indices of points that could possibly be in intersection: not to near and not too far
  int which=0; // 1 the actual lidar returns, 1 random points in a box [for testing], 2 a line [for testing], 3 a 2d grid for testing
  if (which==0){
    for( unsigned int i = 0; i < scan_cloud_s2l_->points.size() ; i++ ){
      if ( (lidar_msgout_.ranges[i] > ASSUMED_HEAD  ) &&(lidar_msgout_.ranges[i] < ASSUMED_FAR )){
        Vector3f point(scan_cloud_s2l_->points[i].x, scan_cloud_s2l_->points[i].y, scan_cloud_s2l_->points[i].z );
        points.push_back( point );
        possible_indices.push_back(i);
      }
    }
  }else if (which ==1){
    for( unsigned int i = 0; i < 500; i++ ){
      Vector3f point(last_rstate_.origin_position.translation.x +  -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0,
                    last_rstate_.origin_position.translation.y + -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0,
                    last_rstate_.origin_position.translation.z +  -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0 );
      points.push_back( point );
      possible_indices.push_back( points.size() - 1 );
    }
  }else if(which==2){
    for( unsigned int i = 0; i < 500; i++ ){
      Vector3f point(-0.0, -1 + 0.005*i,1.4);
      points.push_back( point );
      possible_indices.push_back( points.size() - 1 );
    }
  }else if(which==3){
    Vector3f bot_root(last_rstate_.origin_position.translation.x,last_rstate_.origin_position.translation.y,last_rstate_.origin_position.translation.z);
    Vector3f offset( 0.,0.,0.45);
    for( float i = -0.3; i < 0.3; i=i+0.02 ){
      for( float j = -0.3; j < 0.3; j=j+0.02 ){
        Vector3f point =  Vector3f(i,j,0.) + bot_root + offset;
        points.push_back( point );
        possible_indices.push_back( points.size() - 1 );
        if (points.size() > n_collision_points_){
          break; 
        }
      }
      if (points.size() > n_collision_points_){
        break; 
      }      
    }
  }
  
  
  // 2. Do the filtering:
  // set the state of the collision objects
  collision_object_gfe_->set( last_rstate_ );
  //cout << "gfe obj size: " << collision_object_gfe_->bt_collision_objects().size() << "\n";
  collision_object_point_cloud_->set( points );
  // get the vector of collisions by running collision detection
  //int64_t tic = _timestamp_now();
  vector< Collision > collisions = collision_detector_->get_collisions();
  //cout << lidar_msgout_.ranges.size() << " and " << points.size() << " and " << scan_cloud_s2l_->points.size() << " " <<  (_timestamp_now() - tic)*1e-6 << " dt\n";;
  
  
  // 3. Extract the incidices and modify the outgoing ranges as a result:
  vector< Vector3f > free_points;
  vector< Vector3f > colliding_points;      
  vector< unsigned int > filtered_point_indices;
  for( unsigned int j = 0; j < collisions.size(); j++ ){
    filtered_point_indices.push_back( atoi( collisions[ j ].second_id().c_str() ) );
  }
  for( unsigned int j = 0; j < points.size(); j++ ){
    for( unsigned int k = 0; k < filtered_point_indices.size(); k++ ){
      if( j == filtered_point_indices[ k ] ){
        lidar_msgout_.ranges[  possible_indices[j] ] = COLLISION_RANGE;// 15.0; // Set filtered returns to max range
      }
    }
  }
  for( unsigned int j = 0; j < lidar_msgout_.ranges.size(); j++ ){
    if (lidar_msgout_.ranges[j] < ASSUMED_HEAD ){
        lidar_msgout_.ranges[j] = COLLISION_RANGE;//20.0; // Set filtered returns to max range
    }  
  }
  
  // 4. Output channel is incoming channel appended with FREE
  lcm_->publish( (lidar_channel_ + "_FREE") , &lidar_msgout_);        

  ///////////////////////////////////////////////////////////////////////////
  if (verbose_){
    for( unsigned int j = 0; j < points.size(); j++ ){
      bool point_filtered = false;
      for( unsigned int k = 0; k < filtered_point_indices.size(); k++ ){
        if( j == filtered_point_indices[ k ] ){
          point_filtered = true;
        }
      }
      if( point_filtered ){
        colliding_points.push_back( points[ j ] );
      } else {
        free_points.push_back( points[ j ] );
      }
    }    
    
    cout << current_utime << " | total returns "<< scan_cloud_s2l_->points.size()  
        << " | colliding " << colliding_points.size() << " | free " << free_points.size() << endl;
    bot_lcmgl_point_size(lcmgl_, 4.5f);
    bot_lcmgl_color3f(lcmgl_, 0, 1, 0);
    for (int i = 0; i < free_points.size(); ++i) {
      bot_lcmgl_begin(lcmgl_, LCMGL_POINTS);
      bot_lcmgl_vertex3f(lcmgl_, free_points[i][0], free_points[i][1], free_points[i][2]);
      bot_lcmgl_end(lcmgl_);
    }

    bot_lcmgl_point_size(lcmgl_, 10.5f);
    bot_lcmgl_color3f(lcmgl_, 1, 0, 0);
    for (int i = 0; i < colliding_points.size(); ++i) {
      bot_lcmgl_begin(lcmgl_, LCMGL_POINTS);
      bot_lcmgl_vertex3f(lcmgl_, colliding_points[i][0], colliding_points[i][1], colliding_points[i][2]);
      bot_lcmgl_end(lcmgl_);
    }
    bot_lcmgl_switch_buffer(lcmgl_);  
  }
}


void Pass::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
  if (!init_rstate_){
    cout << "have laser but no robot state, refusing to publish\n";
    return;
  }
  // TODO: check if the rstate is stale

  // A counter for visualization:
  vis_counter_++;
  if (vis_counter_ >=1){ // set this to 1 to only see the last return
    vis_counter_=0;
  }
  int64_t pose_id=vis_counter_;
  // int64_t pose_id=msg->utime;

  // 0. Make Local copy to later output
  lidar_msgout_ = *msg;
  
  // 1. Convert scan into simple XY point cloud:  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  // consider everything - don't remove any points
  double minRange =0.0;
  double maxRange = 100.0;
  double validBeamAngles[] ={-10,10}; 
  convertLidar(msg->ranges, msg->nranges, msg->rad0,
      msg->radstep, scan_cloud, minRange, maxRange,
      validBeamAngles[0], validBeamAngles[1]);  
  
  // 2. Project the scan into local frame:
  Eigen::Isometry3d scan_to_local;
  frames_cpp_->get_trans_with_utime( botframes_ ,  lidar_channel_.c_str() , "local", msg->utime, scan_to_local);
  Eigen::Isometry3f pose_f = isometryDoubleToFloat(scan_to_local);
  Eigen::Quaternionf pose_quat(pose_f.rotation());
  pcl::transformPointCloud (*scan_cloud, *scan_cloud_s2l_,
      pose_f.translation(), pose_quat);  

  if (verbose_){  
    // Plot original scan in sensor frame:
    Isometry3dTime scan_to_localT = Isometry3dTime(pose_id, scan_to_local);
    pc_vis_->pose_to_lcm_from_list(60000, scan_to_localT);
    pc_vis_->ptcld_to_lcm_from_list(60001, *scan_cloud, pose_id, pose_id);  
  
    // Plot scan in local frame:
    Eigen::Isometry3d null_pose;
    null_pose.setIdentity();
    Isometry3dTime null_poseT = Isometry3dTime(pose_id, null_pose);
    pc_vis_->pose_to_lcm_from_list(1000, null_poseT);  
    pc_vis_->ptcld_to_lcm_from_list(1001, *scan_cloud_s2l_, pose_id, pose_id);
  }
  
  if (scan_cloud_s2l_->points.size() > n_collision_points_){
    std::cout << "more points in scan ("<<scan_cloud_s2l_->points.size() <<") than reserved in collision detector ("<< n_collision_points_ << ")"
              << "\nincrease detector size to match\n";
    exit(-1);
  }

  DoCollisionCheck(msg->utime);
  
  if (printf_counter_%80 ==0){
    cout << "Filtering: " << lidar_channel_ << " "  << msg->utime << "\n";
  }
  printf_counter_++;
}


void Pass::robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  last_rstate_= *msg;  
  init_rstate_=true;
}

int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool verbose=FALSE;
  string lidar_channel="SCAN";
  parser.add(verbose, "v", "verbose", "Verbosity");
  parser.add(lidar_channel, "l", "lidar_channel", "Incoming LIDAR channel");
  parser.parse();
  cout << verbose << " is verbose\n";
  cout << lidar_channel << " is lidar_channel\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,verbose,lidar_channel);
  cout << "Ready to filter lidar points" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
