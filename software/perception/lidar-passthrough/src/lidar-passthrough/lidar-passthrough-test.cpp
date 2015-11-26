// Lidar Passthrough filter, demo program

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

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc/robot_urdf_t.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"
#include <ConciseArgs>
#include <drake/RigidBodyManipulator.h>
#include <drake/drakeGeometryUtil.h>

using namespace std;
using namespace drc;
using namespace Eigen;
//using namespace collision;
using namespace boost::assign; // bring 'operator+()' into scope

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_, double collision_threshold_);
    
    ~Pass(){
    }    

    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    boost::shared_ptr<ModelClient> model_;
    bool verbose_;
    double collision_threshold_;

    void urdfHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_urdf_t* msg); 
    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);   

    int n_collision_points_;
    
  void DoCollisionCheck();
    
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* frames_cpp_;
    bot_lcmgl_t* lcmgl_;
    
    int printf_counter_; // used for terminal feedback
    
    bool urdf_parsed_;
    bool urdf_subscription_on_;
    lcm::Subscription *urdf_subscription_; //valid as long as urdf_parsed_ == false
    // Last robot state: this is used as the collision robot
    drc::robot_state_t last_rstate_;
    bool init_rstate_;

    RigidBodyManipulator drake_model_;
    map<string, int> dofMap_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_, double collision_threshold_):
    lcm_(lcm_), verbose_(verbose_),
    urdf_parsed_(false),
    collision_threshold_(collision_threshold_){
  do {
    botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  } while (botparam_ == NULL);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  
  // TODO: get the urdf model from LCM:
  drake_model_.addRobotFromURDFString(model_->getURDFString());
  drake_model_.compile();
  dofMap_ = drake_model_.computePositionNameToIndexMap();

 
  lcmgl_= bot_lcmgl_init(lcm_->getUnderlyingLCM(), "lidar-pt");
  lcm_->subscribe("EST_ROBOT_STATE",&Pass::robotStateHandler,this);
  
  printf_counter_ =0;
  
  n_collision_points_ = 1000;
  init_rstate_ =false;
  
  cout << "Finished setting up\n";
}


// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

VectorXd robotStateToDrakePosition(const drc::robot_state_t& rstate,
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

void Pass::DoCollisionCheck(){
  
  // 0. copy robot state
  drc::robot_state_t rstate;
  rstate = last_rstate_;
  
  // 1. create the list of points to be considered:
  vector< Vector3d > points;
  std::vector<unsigned int> possible_indices; // the indices of points that could possibly be in intersection: not to near and not too far
  int which=3; // 0 actual lidar returns | Test modes: 1 random points in a box, 2 a line [for testing], 3 a 2d grid
  if (which ==1){
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
    for( float i = -0.4; i < 0.4; i=i+0.02 ){
      for( float j = -0.2; j < 0.2; j=j+0.02 ){
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
  std::cout << q << "\n";
  std::cout << drake_model_.num_positions << " np\n";

  KinematicsCache<double> cache = drake_model_.doKinematics(q, 0);
  vector<size_t> filtered_point_indices = drake_model_.collidingPoints(cache, points, collision_threshold_);
  
  
  vector< Vector3d > free_points;
  vector< Vector3d > colliding_points;     

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
    
    cout << 0 << " | total returns "<< 0
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



void Pass::robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  last_rstate_= *msg;  
  init_rstate_=true;

  DoCollisionCheck();
}

int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool verbose=FALSE;
  // was 0.04 for a long time
  // using 0.06 for the simulator
  // using 0.1 for the real robot - until the new urdf arrives ... aug 2013
  double collision_threshold = 0.06; 
  parser.add(verbose, "v", "verbose", "Verbosity");
  parser.add(collision_threshold, "c", "collision_threshold", "Lidar sphere radius [higher removes more points close to the robot]");
  parser.parse();
  cout << verbose << " is verbose\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  Pass app(lcm,verbose,collision_threshold);
  cout << "Ready to filter lidar points" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
