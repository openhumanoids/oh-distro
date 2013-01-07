#include <iostream>
#include <Eigen/Dense>
#include <collision_detection/collision.h>
#include <collision_detection/collision_detector.h>
#include <collision_detection/collision_object_gfe.h>
#include <collision_detection/collision_object_point_cloud.h>


#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>

using namespace std;
using namespace drc;
using namespace Eigen;
using namespace collision_detection;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &publish_lcm);
    
    ~Pass(){
    }
    
    Collision_Object_GFE* collision_object_gfe_;
    Collision_Object_Point_Cloud* collision_object_point_cloud_;
    Collision_Detector* collision_detector_;
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);   
    
    bot_lcmgl_t* lcmgl_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_):          lcm_(lcm_){


  lcmgl_= bot_lcmgl_init(lcm_->getUnderlyingLCM(), "lidar-pt");
  //30
  lcm_->subscribe("EST_ROBOT_STATE",&Pass::robot_state_handler,this);  
  
  

//  collision_object_gfe_ = new Collision_Object_GFE( "collision-object-gfe","/home/mfallon/drc/software/build/models/mit_gazebo_models/mit_robot/model.urdf" );
  collision_object_gfe_ = new Collision_Object_GFE( "collision-object-gfe","/home/mfallon/drc/software/perception/mfallon_sandbox/src/lidar-passthrough/model.sdf" );
  
  collision_object_point_cloud_ = new Collision_Object_Point_Cloud( "collision-object-point-cloud", 1000 );

  // create the collision detector
  collision_detector_ = new Collision_Detector();

  // add the two collision objects to the collision detector with different groups and filters (to prevent checking of self collisions)
  collision_detector_->add_collision_object( collision_object_gfe_, COLLISION_DETECTOR_GROUP_1, COLLISION_DETECTOR_GROUP_2 );
  collision_detector_->add_collision_object( collision_object_point_cloud_, COLLISION_DETECTOR_GROUP_2, COLLISION_DETECTOR_GROUP_1 ); 
  
}

void Pass::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
cout << "got: " << msg->utime << "\n";
  

  cout << "gfe obj size: " << collision_object_gfe_->bt_collision_objects().size() << "\n";
  
  robot_state_t robot_state= *msg;
  
  
  // create a std::vector< Eigen::Vector3f > of points
  vector< Vector3f > points;
  int which=1;
  if (which==0){
  for( unsigned int i = 0; i < 500; i++ ){
    Vector3f point(robot_state.origin_position.translation.x +  -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0,
                   robot_state.origin_position.translation.y + -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0,
                   robot_state.origin_position.translation.z +  -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0 );
    points.push_back( point );
  }

  }else{
  for( unsigned int i = 0; i < 500; i++ ){
    Vector3f point(0,
                   -1 + 0.005*i,
                   -0.2);
    points.push_back( point );
  }
  }
  
  
  // set the state of the collision objects
  collision_object_gfe_->set( robot_state );
  collision_object_point_cloud_->set( points );

  // get the vector of collisions by running collision detection
  vector< Collision > collisions = collision_detector_->get_collisions();

  // display the collisions (debugging purposes only)
  for( unsigned int j = 0; j < collisions.size(); j++ ){
    cout << "collisions[" << j << "]: " << collisions[ j ] << endl;
  }

  vector< Vector3f > unfiltered_points;// = points;
  vector< Vector3f > filtered_points;
      
  vector< unsigned int > filtered_point_indices;
  for( unsigned int j = 0; j < collisions.size(); j++ ){
    filtered_point_indices.push_back( atoi( collisions[ j ].second_id().c_str() ) );
  }

  for( unsigned int j = 0; j < points.size(); j++ ){
    bool point_filtered = false;
    for( unsigned int k = 0; k < filtered_point_indices.size(); k++ ){
      if( j == filtered_point_indices[ k ] ){
        point_filtered = true;
      }
    }
    if( point_filtered ){
      filtered_points.push_back( points[ j ] );
    } else {
      unfiltered_points.push_back( points[ j ] );
    }
  }

  cout << "filtered_points.size(): " << filtered_points.size() << endl;
  cout << "unfiltered_points.size(): " << unfiltered_points.size() << endl;

  cout << endl << "end of collision-detector-point-cloud-filtering-test" << endl << endl;    


  bot_lcmgl_point_size(lcmgl_, 4.5f);
//  bot_lcmgl_color3f(lcmgl_, 0, 0, 1);
//  for (int i = 0; i < points.size(); ++i) {
//    bot_lcmgl_begin(lcmgl_, LCMGL_POINTS);
//    bot_lcmgl_vertex3f(lcmgl_, points[i][0], points[i][1], points[i][2]);
//    bot_lcmgl_end(lcmgl_);
//  }

  bot_lcmgl_color3f(lcmgl_, 0, 1, 0);
  for (int i = 0; i < unfiltered_points.size(); ++i) {
    bot_lcmgl_begin(lcmgl_, LCMGL_POINTS);
    bot_lcmgl_vertex3f(lcmgl_, unfiltered_points[i][0], unfiltered_points[i][1], unfiltered_points[i][2]);
    bot_lcmgl_end(lcmgl_);
  }

  bot_lcmgl_color3f(lcmgl_, 1, 0, 0);
  for (int i = 0; i < filtered_points.size(); ++i) {
    bot_lcmgl_begin(lcmgl_, LCMGL_POINTS);
    bot_lcmgl_vertex3f(lcmgl_, filtered_points[i][0], filtered_points[i][1], filtered_points[i][2]);
    bot_lcmgl_end(lcmgl_);
  }
  bot_lcmgl_switch_buffer(lcmgl_);  
  
  
  //
  
  
  
}

int 
main( int argc, 
      char** argv )
{
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  cout << "start of collision-detector-point-cloud-filtering-test" << endl << endl;

  Pass app(lcm);
  while(0 == lcm->handle());
  return 0;
}
