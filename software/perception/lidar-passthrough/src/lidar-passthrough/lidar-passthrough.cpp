// Lidar Pass trough filter
// TODO/Bugs:
// - Timing of bullet collision detection seems quasi random.
//   difficult to see how to optimize this as a result
// - Collision is done by intersecting spheres with the model. The spheres size is set in collision_object_point_cloud.cc
//   as of march 2013 it was increased from 0.01m to 0.04m

// UPDATE:
// Computation/Timing is dependent on the number of points tested
// When the lidar is horizontal, few points are tested. When intersecting the arms, lots are
// ASSUMED_HEAD = 0.3 achieves ~30Hz worse case (previous default)
// ASSUMED_HEAD = 0.85 achieves ~100Hz worse case

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>

#include <lcm/lcm-cpp.hpp>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>

#include <model-client/model-client.hpp>

#include <pronto_utils/pronto_vis.hpp> // visualize pt clds
#include <pronto_utils/pronto_lcm.hpp> // unpack lidar to xyz
#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/bot_core/robot_urdf_t.hpp"
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include <ConciseArgs>
#include <drake/systems/plants/RigidBodyTree.h>
#include <drake/util/drakeGeometryUtil.h>

using namespace std;

using namespace Eigen;
//using namespace collision;
using namespace boost::assign; // bring 'operator+()' into scope

#define DO_TIMING_PROFILE FALSE

// all ranges shorter than this are assumed to be with the head
#define ASSUMED_HEAD 0.3//0.3
// all ranges longer than this are assumed to be free
#define ASSUMED_FAR 2.0// 2.0
// set all collisions to this range
#define COLLISION_RANGE 0.0
// set all unlikely returns to this range (same range is produced by real sensor)
#define MAX_RANGE 60.0

#define INTENSITY_FILTER_MIN_VALUE 2000
#define INTENSITY_FILTER_MIN_RANGE 2 // meters
#define EDGE_FILTER_MIN_RANGE 2 // meters

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string lidar_channel_, double collision_threshold_,
         bool simulated_data_, double delta_threshold_);
    
    ~Pass(){
    }    

    // thread function for doing actual work
    void operator()();
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    boost::shared_ptr<ModelClient> model_;
    bool verbose_;
    bool simulated_data_;
    std::string lidar_channel_;
    
    double collision_threshold_;
    double delta_threshold_;

    bool running_;
    std::thread worker_thread_;
    std::condition_variable worker_condition_;
    std::mutex worker_mutex_;
    std::list<std::shared_ptr<bot_core::planar_lidar_t> > data_queue_;
    std::mutex data_mutex_;
    std::mutex robot_state_mutex_;
    
    void urdfHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::robot_urdf_t* msg);
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg);   
    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::robot_state_t* msg);   

    int n_collision_points_;
    
  void DoCollisionCheck(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& scan_cloud_s2l,
                        std::shared_ptr<bot_core::planar_lidar_t>& msg);
    
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* frames_cpp_;
    bot_lcmgl_t* lcmgl_;
    
    pronto_vis* pc_vis_;
    pronto_lcm* pc_lcm_;
    int vis_counter_; // used for visualization
    int printf_counter_; // used for terminal feedback
    
    bool urdf_parsed_;
    bool urdf_subscription_on_;
    lcm::Subscription *urdf_subscription_; //valid as long as urdf_parsed_ == false
    // Last robot state: this is used as the collision robot
    bot_core::robot_state_t last_rstate_;
    bool init_rstate_;

    RigidBodyTree drake_model_;
    map<string, int> dofMap_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string lidar_channel_, double collision_threshold_,
         bool simulated_data_, double delta_threshold_):
    lcm_(lcm_), verbose_(verbose_),
    lidar_channel_(lidar_channel_),urdf_parsed_(false),
    simulated_data_(simulated_data_),
    collision_threshold_(collision_threshold_),
    delta_threshold_(delta_threshold_){
  do {
    botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  } while (botparam_ == NULL);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  
  // TODO: get the urdf model from LCM:
  drake_model_.addRobotFromURDFString(model_->getURDFString());
  drake_model_.compile();
  dofMap_ = drake_model_.computePositionNameToIndexMap();

  worker_thread_ = std::thread(std::ref(*this));
  
  lcmgl_= bot_lcmgl_init(lcm_->getUnderlyingLCM(), "lidar-pt");
  lcm_->subscribe( lidar_channel_ ,&Pass::lidarHandler,this);
  lcm_->subscribe("EST_ROBOT_STATE",&Pass::robotStateHandler,this);
  
  // Vis Config:
  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60000,"Pose - Laser",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60001,"Cloud - Laser"         ,1,0, 60000,1, {0.0, 0.0, 1.0} ));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Null",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Cloud - Null"           ,1,0, 1000,1, { 0.0, 1.0, 0.0} ));
  vis_counter_ =0;  
  printf_counter_ =0;
  
  init_rstate_ =false;
  
  cout << "Finished setting up\n";
}


// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

VectorXd robotStateToDrakePosition(const bot_core::robot_state_t& rstate,
                                   const map<string, int>& dofMap, 
                                   int num_positions)
{
  VectorXd q = VectorXd::Zero(num_positions, 1);
  for (int i=0; i < rstate.num_joints; ++i) {
    auto iter = dofMap.find(rstate.joint_name.at(i));
    if (iter != dofMap.end()) {
      int index = iter->second;
      q(index) = rstate.joint_position[i];
    }
  }

  map<string,int>::const_iterator iter;
  iter = dofMap.find("base_x");
  if (iter!=dofMap.end()) {
    int index = iter->second;
    q[index] = rstate.pose.translation.x;
  }
  iter = dofMap.find("base_y");
  if (iter!=dofMap.end()) {
    int index = iter->second;
    q[index] = rstate.pose.translation.y;
  }
  iter = dofMap.find("base_z");
  if (iter!=dofMap.end()) {
    int index = iter->second;
    q[index] = rstate.pose.translation.z;
  }

  Vector4d quat;
  quat[0] = rstate.pose.rotation.w;
  quat[1] = rstate.pose.rotation.x;
  quat[2] = rstate.pose.rotation.y;
  quat[3] = rstate.pose.rotation.z;
  Vector3d rpy = quat2rpy(quat);

  iter = dofMap.find("base_roll");
  if (iter!=dofMap.end()) {
    int index = iter->second;
    q[index] = rpy[0];
  }

  iter = dofMap.find("base_pitch");
  if (iter!=dofMap.end()) {
    int index = iter->second; 
    q[index] = rpy[1];
  }

  iter = dofMap.find("base_yaw");
  if (iter!=dofMap.end()) {
    int index = iter->second; 
    q[index] = rpy[2];
  }

  return q;
}

void Pass::DoCollisionCheck(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& scan_cloud_s2l,
                            std::shared_ptr<bot_core::planar_lidar_t>& msg){
  
  #if DO_TIMING_PROFILE
    std::vector<int64_t> tic_toc;
    tic_toc.push_back(_timestamp_now());
  #endif  

  // 0. copy robot state
  bot_core::robot_state_t rstate;
  {
    std::unique_lock<std::mutex> lock(robot_state_mutex_);
    rstate = last_rstate_;
  }
  
  // 1. create the list of points to be considered:
  vector< Vector3d > points;
  std::vector<unsigned int> possible_indices; // the indices of points that could possibly be in intersection: not to near and not too far
  int which=0; // 0 actual lidar returns | Test modes: 1 random points in a box, 2 a line [for testing], 3 a 2d grid
  if (which==0){
    for( unsigned int i = 0; i < scan_cloud_s2l->points.size() ; i++ ){
      if ( (msg->ranges[i] > ASSUMED_HEAD  ) &&(msg->ranges[i] < ASSUMED_FAR )){
        Vector3d point(static_cast<double>(scan_cloud_s2l->points[i].x), 
                       static_cast<double>(scan_cloud_s2l->points[i].y), 
                       static_cast<double>(scan_cloud_s2l->points[i].z));
        points.push_back( point );
        possible_indices.push_back(i);
      }
    }
  }else if (which ==1){
    for( unsigned int i = 0; i < 1000; i++ ){
      Vector3d point(rstate.pose.translation.x +  -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0,
                    rstate.pose.translation.y + -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0,
                    rstate.pose.translation.z +  -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0 );
      points.push_back( point );
      possible_indices.push_back( points.size() - 1 );
    }
  }else if(which==2){
    for( unsigned int i = 0; i < 500; i++ ){
      Vector3d point(-0.0, -1 + 0.005*i,1.4);
      points.push_back( point );
      possible_indices.push_back( points.size() - 1 );
    }
  }else if(which==3){
    Vector3d bot_root(rstate.pose.translation.x,rstate.pose.translation.y,rstate.pose.translation.z);
    Vector3d offset( 0.,0.,0.45);
    for( float i = -0.3; i < 0.3; i=i+0.02 ){
      for( float j = -0.3; j < 0.3; j=j+0.02 ){
        Vector3d point =  Vector3d(i,j,0.) + bot_root + offset;
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
  
  
  // 2. Extract the indices of the points in collision:
  VectorXd q(robotStateToDrakePosition(rstate, dofMap_, drake_model_.num_positions));
  KinematicsCache<double> cache = drake_model_.doKinematics(q);
  vector<size_t> filtered_point_indices = drake_model_.collidingPoints(cache, points, collision_threshold_);
  
  // 3. Modify the outgoing ranges of the colliding points:
  std::vector<float> original_ranges =  msg->ranges;
  
  
  vector< Vector3d > free_points;
  vector< Vector3d > colliding_points;      
  for( unsigned int j = 0; j < points.size(); j++ ){
    for( unsigned int k = 0; k < filtered_point_indices.size(); k++ ){
      if( j == filtered_point_indices[ k ] ){
        msg->ranges[  possible_indices[j] ] = COLLISION_RANGE;// 15.0; // Set filtered returns to max range
      }
    }
  }
  for( unsigned int j = 0; j < msg->ranges.size(); j++ ){
    if (original_ranges[j] < ASSUMED_HEAD ){
        msg->ranges[j] = COLLISION_RANGE;//20.0; // Set filtered returns to max range
    }  
    
    // For Real Data: apply an addition set of filters
    // NB: These filters are not compatiable with Gazebo Simulation output NBNBNB
    if (!simulated_data_){
      // heuristic filtering of the weak intensity lidar returns
      if (( msg->intensities[j] < INTENSITY_FILTER_MIN_VALUE ) && ( original_ranges[j] < INTENSITY_FILTER_MIN_RANGE) ){
        msg->ranges[j] = MAX_RANGE;
      }
      
      // Edge effect filter
      if ( (j>0) && (j<msg->ranges.size()) ){
        float right_diff = fabs(original_ranges[j] - original_ranges[j-1]);
        float left_diff = fabs(original_ranges[j] - original_ranges[j+1]);
        if (( right_diff > delta_threshold_) || (left_diff > delta_threshold_ )){
          // cout << i<< ": " << right_diff << " and " << left_diff <<"\n";
          if (original_ranges[j] < EDGE_FILTER_MIN_RANGE){
            msg->ranges[j] = MAX_RANGE;
          }
        }
      }      
      
    }
    
  }
  
  // 4. Output channel is incoming channel appended with FREE
  lcm_->publish( (lidar_channel_ + "_FREE") , msg.get());        

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
    
    cout << msg->utime << " | total returns "<< scan_cloud_s2l->points.size()  
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
  
  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
    double dt =  ((tic_toc[1] - tic_toc[0])*1E-6);
    
    std::cout << dt << " | " << 1/dt  << " | "
              << msg->utime << " | total returns "<< scan_cloud_s2l->points.size()  
              << " | collisions " << collisions.size() << " | indices " << filtered_point_indices.size() << endl;    
  #endif    
  
}


void Pass::operator()() {
  running_ = true;
  while (running_) {
    std::unique_lock<std::mutex> lock(worker_mutex_);
    worker_condition_.wait_for(lock, std::chrono::milliseconds(1000));

    // copy current workload from data queue to work queue
    std::vector<std::shared_ptr<bot_core::planar_lidar_t> > work_queue;
    {
      std::unique_lock<std::mutex> lock(data_mutex_);
      while (!data_queue_.empty()) {
        work_queue.push_back(data_queue_.front());
        data_queue_.pop_front();
      }
    }

    // process workload
    for (auto msg : work_queue) {
  
      // 1. Convert scan into simple XY point cloud:  
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      // consider everything - don't remove any points
      double minRange =-100.0;
      double maxRange = 100.0;
      double validBeamAngles[] ={-10,10}; 
      convertLidar(msg->ranges, msg->nranges, msg->rad0,
                   msg->radstep, scan_cloud, minRange, maxRange,
                   validBeamAngles[0], validBeamAngles[1]);  
  
      if (scan_cloud->points.size() !=  msg->nranges ){
        std::cout << "npoints and nranges are not equal\n";
        std::cout << scan_cloud->points.size() << "\n";
        std::cout << msg->nranges << "\n";
        exit(-1); 
      }  
  
      // 2. Project the scan into local frame:
      Eigen::Isometry3d scan_to_local;
      frames_cpp_->get_trans_with_utime( botframes_ ,  lidar_channel_.c_str() , "local", msg->utime, scan_to_local);
      Eigen::Isometry3f pose_f = scan_to_local.cast<float>();
      Eigen::Quaternionf pose_quat(pose_f.rotation());
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_cloud_s2l(new pcl::PointCloud<pcl::PointXYZRGB> ());
      pcl::transformPointCloud (*scan_cloud, *scan_cloud_s2l,
                                pose_f.translation(), pose_quat);  

      // A counter for visualization:
      vis_counter_++;
      if (vis_counter_ >=1){ // set this to 1 to only see the last return
        vis_counter_=0;
      }
      int64_t pose_id=vis_counter_;
      // int64_t pose_id=msg->utime;

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
        pc_vis_->ptcld_to_lcm_from_list(1001, *scan_cloud_s2l, pose_id, pose_id);
      }
  
      if (printf_counter_%80 ==0){
        cout << "Filtering: " << lidar_channel_ << " "  << msg->utime << "\n";
      }
      printf_counter_++;

      if (scan_cloud_s2l->points.size() > n_collision_points_){
        std::cout << "more points in scan ("<<scan_cloud_s2l->points.size() <<") than reserved in collision detector ("<< n_collision_points_ << ")"
                  << "\nincrease detector size to match\n";
        exit(-1);
      }

      DoCollisionCheck(scan_cloud_s2l,msg);
    }
  }
}

void Pass::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
  if (!init_rstate_){
    cout << "have laser but no robot state, refusing to publish\n";
    return;
  }
  // TODO: check if the rstate is stale

  // push this scan onto the work queue
  // TODO: can increase max size if necessary
  const int max_queue_size = 100;
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    std::shared_ptr<bot_core::planar_lidar_t> data
      (new bot_core::planar_lidar_t(*msg));
    data_queue_.push_back(data);
    if (data_queue_.size() > max_queue_size) {
      std::cout << "WARNING: dropping " << 
        (data_queue_.size()-max_queue_size) << " scans" << std::endl;
    }
    while (data_queue_.size() > max_queue_size) {
      data_queue_.pop_front();
    }
  }
  worker_condition_.notify_one();
}


void Pass::robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::robot_state_t* msg){
  std::unique_lock<std::mutex> lock(robot_state_mutex_);
  last_rstate_= *msg;  
  init_rstate_=true;
}

int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool verbose=FALSE;
  string lidar_channel="SCAN";
  // was 0.04 for a long time
  // using 0.06 for the simulator
  // using 0.1 for the real robot - until the new urdf arrives ... aug 2013
  double collision_threshold = 0.06; 
  double delta_threshold = 0.03; 
  bool simulated_data = FALSE;
  parser.add(verbose, "v", "verbose", "Verbosity");
  parser.add(simulated_data, "s", "simulated_data", "Simulated Data expected (don't filter with intensities)");  
  parser.add(lidar_channel, "l", "lidar_channel", "Incoming LIDAR channel");
  parser.add(collision_threshold, "c", "collision_threshold", "Lidar sphere radius [higher removes more points close to the robot]");
  parser.add(delta_threshold, "d", "delta_threshold", "Maximum Delta in Lidar Range allowed in workspace");
  parser.parse();
  cout << verbose << " is verbose\n";
  cout << lidar_channel << " is lidar_channel\n";
  cout << simulated_data << " is simulated_data\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  Pass app(lcm,verbose,lidar_channel, collision_threshold, 
           simulated_data, delta_threshold);
  cout << "Ready to filter lidar points" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
