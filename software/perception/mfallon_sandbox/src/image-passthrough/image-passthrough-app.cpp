#include <iostream>
#include <Eigen/Dense>
#include <collision_detection/collision.h>
#include <collision_detection/collision_detector.h>
#include <collision_detection/collision_object_gfe.h>
#include <collision_detection/collision_object_point_cloud.h>


#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>

#include "image-passthrough.hpp"

using namespace std;
using namespace drc;
using namespace Eigen;
using namespace collision_detection;

class Pass{
  public:
    Pass(int argc, char** argv, boost::shared_ptr<lcm::LCM> &publish_lcm);
    
    ~Pass(){
    }
    
    Collision_Object_GFE* collision_object_gfe_;
    Collision_Object_Point_Cloud* collision_object_point_cloud_;
    Collision_Detector* collision_detector_;

    SimExample::Ptr simexample;

    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);   

    void send_output(string fname_root);
    
    bot_lcmgl_t* lcmgl_;
};

Pass::Pass(int argc, char** argv, boost::shared_ptr<lcm::LCM> &lcm_):          
    lcm_(lcm_){


  lcmgl_= bot_lcmgl_init(lcm_->getUnderlyingLCM(), "lidar-pt");
  //30
  lcm_->subscribe("EST_ROBOT_STATE",&Pass::robot_state_handler,this);  
  
  
  collision_object_gfe_ = new Collision_Object_GFE( "collision-object-gfe","/home/mfallon/drc/software/build/models/mit_gazebo_models/mit_robot/model.sdf" );
  
  collision_object_point_cloud_ = new Collision_Object_Point_Cloud( "collision-object-point-cloud", 1000 );

  // create the collision detector
  collision_detector_ = new Collision_Detector();

  // add the two collision objects to the collision detector with different groups and filters (to prevent checking of self collisions)
  collision_detector_->add_collision_object( collision_object_gfe_, COLLISION_DETECTOR_GROUP_1, COLLISION_DETECTOR_GROUP_2 );
  collision_detector_->add_collision_object( collision_object_point_cloud_, COLLISION_DETECTOR_GROUP_2, COLLISION_DETECTOR_GROUP_1 ); 
  

  // Construct the simulation method - with camera params as of Jan 2013:
  int width = 1024;
  int height = 544;
  double fx = 610.1778; 
  double fy = 610.1778;
  double cx = 512.5;
  double cy = 272.5;
  simexample = SimExample::Ptr (new SimExample (argc, argv, height,width , lcm_));
  simexample->setCameraIntrinsicsParameters (width, height, fx, fy, cx, cy);

}


// Output the simulated output to file:
void Pass::send_output(string fname_root){ 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_out (new pcl::PointCloud<pcl::PointXYZRGB>);
  
    // Read Color Buffer from the GPU before creating PointCloud:
    // By default the buffers are not read back from the GPU
    //simexample->rl_->getColorBuffer ();
    //simexample->rl_->getDepthBuffer ();  
    // Add noise directly to the CPU depth buffer 
    //simexample->rl_->addNoise ();

    // Optional argument to save point cloud in global frame:
    // Save camera relative:
    //simexample->rl_->getPointCloud(pc_out);
    // Save in global frame - applying the camera frame:
    //simexample->rl_->getPointCloud(pc_out,true,simexample->camera_->getPose());
    // Save in local frame
    //simexample->rl_->getPointCloud (pc_out,false,simexample->camera_->getPose ());
    // TODO: what to do when there are more than one simulated view?
    
    /*if (pc_out->points.size()>0){
    std::cout << pc_out->points.size() << " points written to file\n";
    
    pcl::PCDWriter writer;
    //writer.write ( string (fname_root + ".pcd"), *pc_out,	false);  /// ASCII
    writer.writeBinary (  string (fname_root + ".pcd")  , *pc_out);
    //cout << "finished writing file\n";
    }else{
    std::cout << pc_out->points.size() << " points in cloud, not written\n";
    }*/

    //simexample->write_score_image (simexample->rl_->getScoreBuffer (), 
    //   		   string (fname_root + "_score.png") );  
    simexample->write_rgb_image (simexample->rl_->getColorBuffer (), 
				 string (fname_root + "_rgb.png") );  
    simexample->write_depth_image (simexample->rl_->getDepthBuffer (),
				    string (fname_root + "_depth.png") );  

    //simexample->write_depth_image_uint (simexample->rl_->getDepthBuffer (),
    //                                string (fname_root + "_depth_uint.png") );

}


void Pass::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  cout << "got: " << msg->utime << "\n";

  cout << "gfe obj size: " << collision_object_gfe_->bt_collision_objects().size() << "\n";
  
  robot_state_t robot_state= *msg;
  
  /*
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

  vector< Vector3f > unfiltered_points = points;
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


  //bot_lcmgl_color3f(lcmgl_, pc_vis_->colors[counter_*3], pc_vis_->colors[counter_*3+1], pc_vis_->colors[counter_*3+2]);
  bot_lcmgl_point_size(lcmgl_, 4.5f);
  bot_lcmgl_color3f(lcmgl_, 0, 0, 1);
  for (int i = 0; i < points.size(); ++i) {
    bot_lcmgl_begin(lcmgl_, LCMGL_POINTS);
    bot_lcmgl_vertex3f(lcmgl_, points[i][0], points[i][1], points[i][2]);
    bot_lcmgl_end(lcmgl_);
  }

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
*/
  

  // Construct model view within opengl simulator:
// cd ~/projects/kmcl/models/table_models
// rgbd-sim-terminal-demo 1 ~/Desktop/drc_mesh_rgbd/utorso.obj 

//- for each object:
//-- determine the joint position with forward kinematics
//-- add to model

//-- render in rgbd_simulation ... done
//-- publish a mask with depth and binary ... done


    Eigen::Vector3d focus_center(-3.0,0.0,-1.5);
    double halo_r = 4;
    double halo_dz = 2;
    int n_poses=16;
    double t=0;

    double x = halo_r*cos(t);
    double y = halo_r*sin(t);
    double z = halo_dz;
    double pitch =atan2( halo_dz,halo_r); 
    double yaw = atan2(-y,-x);
   
    Eigen::Isometry3d pose;
    pose.setIdentity();
    Eigen::Matrix3d m;
    m = AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
	* AngleAxisd(pitch, Eigen::Vector3d::UnitY())
	* AngleAxisd(0, Eigen::Vector3d::UnitZ());    

    pose *=m;
    Vector3d v(x,y,z);
    v += focus_center;
    pose.translation() = v;


    simexample->doSim(pose);
    
    send_output("passthrough");
    cout << "output send\n";



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

  Pass app(argc,argv, lcm);


  while(0 == lcm->handle());
  return 0;
  
  
  // create the gfe collision object (args: id)
  Collision_Object_GFE collision_object_gfe( "collision-object-gfe","/home/mfallon/drc/software/build/models/mit_gazebo_models/mit_robot/model.sdf" );
//  Collision_Object_GFE collision_object_gfe( "collision-object-gfe","/home/mfallon/drc/software/build/models/mit_gazebo_models/mit_robot_PnC/model.sdf" );

  // create the point cloud collision object (args: id, max # points)
  Collision_Object_Point_Cloud collision_object_point_cloud( "collision-object-point-cloud", 1000 );


  
  // create the collision detector
  Collision_Detector collision_detector;

  // add the two collision objects to the collision detector with different groups and filters (to prevent checking of self collisions)
  collision_detector.add_collision_object( &collision_object_gfe, COLLISION_DETECTOR_GROUP_1, COLLISION_DETECTOR_GROUP_2 );
  collision_detector.add_collision_object( &collision_object_point_cloud, COLLISION_DETECTOR_GROUP_2, COLLISION_DETECTOR_GROUP_1 );   
  
  
  
  // create a drc::robot_state_t class
  robot_state_t robot_state;
  robot_state.utime =1;
  robot_state.robot_name ="d";
  robot_state.origin_position.translation.x = 0.0;
  robot_state.origin_position.translation.y = 0.0;
  robot_state.origin_position.translation.z = 1.0;
  robot_state.origin_position.rotation.x = 0.0;
  robot_state.origin_position.rotation.y = 0.0;
  robot_state.origin_position.rotation.z = 0.0;
  robot_state.origin_position.rotation.w = 1.0;
  robot_state.origin_twist.linear_velocity.x =0.0;
  robot_state.origin_twist.linear_velocity.y =0.0;
  robot_state.origin_twist.linear_velocity.z =0.0;
  robot_state.origin_twist.angular_velocity.x =0.0;
  robot_state.origin_twist.angular_velocity.y =0.0;
  robot_state.origin_twist.angular_velocity.z =0.0;
  for(int i = 0; i < 6; i++)  {
    for(int j = 0; j < 6; j++) {
      robot_state.origin_cov.position_cov[i][j] = 0;
      robot_state.origin_cov.twist_cov[i][j] = 0;
    }
  }
  
  robot_state.num_joints = collision_object_gfe.kinematics_model().tree().getNrOfJoints();
  
  cout << "gfe obj size: " << collision_object_gfe.bt_collision_objects().size() << "\n";
  std::vector< btCollisionObject* > objs = collision_object_gfe.bt_collision_objects();
  cout << objs[0]->getBroadphaseHandle()->getUid() << " mat\n";
  
  robot_state.joint_position.resize( collision_object_gfe.kinematics_model().tree().getNrOfJoints() );
  for( unsigned int i = 0; i < collision_object_gfe.kinematics_model().tree().getNrOfJoints(); i++ ){
    robot_state.joint_position[ i ] = -0.5 + 1.0 * ( rand() % 1000 ) / 1000.0;
    robot_state.joint_velocity.push_back(0);
    robot_state.measured_effort.push_back(0);
    
    robot_state.joint_name.push_back("sdfsd");
    drc::joint_covariance_t j_cov;
    j_cov.variance = 0;
    robot_state.joint_cov.push_back(j_cov);    
  }
  
  
  // dummy ground contact states
  robot_state.contacts.num_contacts =8;
  robot_state.contacts.id.push_back("LFootToeIn");
  robot_state.contacts.id.push_back("LFootToeOut");
  robot_state.contacts.id.push_back("LFootHeelIn");
  robot_state.contacts.id.push_back("LFootHeelOut");
  robot_state.contacts.id.push_back("RFootToeIn");
  robot_state.contacts.id.push_back("RFootToeOut");
  robot_state.contacts.id.push_back("RFootHeelIn");
  robot_state.contacts.id.push_back("RFootHeelOut");
  for (int i=0; i< robot_state.contacts.num_contacts; i++){
    robot_state.contacts.inContact.push_back(0);
    drc::vector_3d_t f_zero;
    f_zero.x = 0;f_zero.y = 0;f_zero.z = 0;
    robot_state.contacts.contactForce.push_back(f_zero);
  }  
  
  
  
  cout << "dog1\n";
  
   lcm->publish("EST_ROBOT_STATE", &robot_state);
  cout << "dog\n";
  

  // create a std::vector< Eigen::Vector3f > of points
  vector< Vector3f > points;
  for( unsigned int i = 0; i < 500; i++ ){
    Vector3f point( -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0,
                    -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0,
                    0.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0 );
    points.push_back( point );
  }

  // set the state of the collision objects
  collision_object_gfe.set( robot_state );
  collision_object_point_cloud.set( points );

  // get the vector of collisions by running collision detection
  vector< Collision > collisions = collision_detector.get_collisions();

  // display the collisions (debugging purposes only)
  for( unsigned int j = 0; j < collisions.size(); j++ ){
    cout << "collisions[" << j << "]: " << collisions[ j ] << endl;
  }

  vector< Vector3f > unfiltered_points = points;
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
  return 0;
}
