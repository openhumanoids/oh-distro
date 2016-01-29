// Demonstrates Bullet and Drake collision checking
// places robot at q = 0, creates some points
// checks the points for collisions with bullet
// publishes result to lcm
//
// Example invocations:
// bash -c 'ROS_PACKAGE_PATH=${DRC_BASE}/software/models:${DRC_BASE}/software/models/common_components:${ROS_PACKAGE_PATH} 
// drc-testDrakeBulletCollisions -c 0.0 -v -u /home/mfallon/main-distro/software/models/val_description/urdf/valkyrie_sim_drake.urdf -w 4
//
// bash -c 'ROS_PACKAGE_PATH=${DRC_BASE}/software/models:${DRC_BASE}/software/models/common_components:${ROS_PACKAGE_PATH} 
// drc-testDrakeBulletCollisions -c 0.0 -v -u /home/mfallon/main-distro/software/models/val_description/urdf/valkyrie_sim.urdf -w 1'

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include <lcm/lcm-cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>

#include "lcmtypes/bot_core.hpp"
#include <ConciseArgs>
#include <drake/systems/plants/RigidBodyTree.h>
#include <drake/util/drakeGeometryUtil.h>

using namespace std;
using namespace Eigen;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_, double collision_threshold_, std::string urdf_file_, int which_);
    
    ~Pass(){
    }    

    void DoCollisionCheck();

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool verbose_;
    double collision_threshold_;
    int which_;
    std::string urdf_file_;
    bot_lcmgl_t* lcmgl_;
    RigidBodyTree drake_model_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_, double collision_threshold_, std::string urdf_file_, int which_):
    lcm_(lcm_), verbose_(verbose_),
    collision_threshold_(collision_threshold_),
    urdf_file_(urdf_file_), which_(which_){

  drake_model_.addRobotFromURDF(urdf_file_);
  drake_model_.compile();
  lcmgl_= bot_lcmgl_init(lcm_->getUnderlyingLCM(), "lidar-pt");
  cout << "Finished setting up\n";
}


void Pass::DoCollisionCheck(){
  
  Eigen::Vector3d robot_pos(0,0,0);

  // 1. create the list of points to be considered:
  vector< Vector3d > points;
  std::vector<unsigned int> possible_indices; // the indices of points that could possibly be in intersection: not to near and not too far
  if (which_ ==1){
    for( unsigned int i = 0; i < 10000; i++ ){
      Vector3d point(robot_pos(0) +  -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0,
                    robot_pos(1) + -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0,
                    robot_pos(2) +  -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0 );
      points.push_back( point );
      possible_indices.push_back( points.size() - 1 );
    }
  }else if(which_ ==2){
    for( unsigned int i = 0; i < 500; i++ ){
      Vector3d point(-0.0, -1 + 0.005*i,1.4);
      points.push_back( point );
      possible_indices.push_back( points.size() - 1 );
    }
  }else if(which_ ==3){ // useful for valkyrie (v2)
    Vector3d offset( 0.,0.,0.3);
    for( float i = -0.5; i < 0.5; i=i+0.03 ){
      for( float j = -0.5; j < 0.5; j=j+0.03 ){
        Vector3d point =  Vector3d(i,j,0.) + robot_pos + offset;
        points.push_back( point );
        possible_indices.push_back( points.size() - 1 );
      }
    }
  }else if(which_ ==4){ // useful for atlas (v5)
    Vector3d offset( 0.,0.,0.45);
    for( float i = -0.5; i < 0.5; i=i+0.03 ){
      for( float j = -0.5; j < 0.5; j=j+0.03 ){
        Vector3d point =  Vector3d(i,j,0.) + robot_pos + offset;
        points.push_back( point );
        possible_indices.push_back( points.size() - 1 );
      }
    }
  }
  
  
  // 2. Extract the indices of the points in collision:
  VectorXd q = VectorXd::Zero(drake_model_.num_positions, 1);
  KinematicsCache<double> cache = drake_model_.doKinematics(q);
  vector<size_t> filtered_point_indices = drake_model_.collidingPoints(cache, points, collision_threshold_);
  ///////////////////////////////////////////////////////////////////////////
  
  vector< Vector3d > free_points;
  vector< Vector3d > colliding_points;     
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




int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool verbose=false;
  std::string urdf_file = "filename.urdf";
  // Test modes: 1 random points in a box, 2 a line [for testing], 3 a 2d grid
  int which = 3; 
  // was 0.04 for a long time
  // using 0.06 for the simulator
  // using 0.1 for the real robot - until the new urdf arrives ... aug 2013
  double collision_threshold = 0.06; 
  parser.add(verbose, "v", "verbose", "Verbosity");
  parser.add(collision_threshold, "c", "collision_threshold", "Lidar sphere radius [higher removes more points close to the robot]");
  parser.add(urdf_file, "u", "urdf_file", "Path to the robot URDF file");
  parser.add(which, "w", "which", "Which type of test? 1 random, 2 a line, 3 a box (for val), 4 a box (for atlas)");
  parser.parse();
  cout << verbose << " is verbose\n";
  cout << urdf_file << " is urdf_file\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  Pass app(lcm,verbose,collision_threshold, urdf_file, which);
  cout << "Ready to filter lidar points" << endl << "============================" << endl;

  app.DoCollisionCheck();

  return 0;
}
