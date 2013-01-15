// Module to use raw Vicon data to produce end effector goals for the DRC robot 
// cpda, mfallon, jan 2013
//
// TODO: correct incoming data to be in metres
// TODO: make sure data has utimes
#include <stdio.h>
#include <GL/gl.h>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>

#include <lcm/lcm-cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc_lcmtypes.hpp"
#include <lcmtypes/vicon_drc.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <ConciseArgs>


using namespace std;
using namespace boost::assign; // bring 'operator+()' into scope

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string vicon_channel_, std::string output_type_, 
         std::string vicon_model_, double human_to_robot_scale_factor);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool verbose_;
    std::string output_type_, vicon_channel_, vicon_model_;
    
    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);   
    void publishEndEffectorGoal(Isometry3dTime waist_to_segment, std::string channel, std::string ee_name);
    void visualizeIncomingData(const viconstructs::vicon_t *msg);
    void viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  viconstructs::vicon_t* msg);   
    
    bot_lcmgl_t* lcmgl_;
    
    pointcloud_vis* pc_vis_;
    int vis_counter_; // used for visualization
    int printf_counter_; // used for terminal feedback
    
    bool first_rstate_received_;
    
    int64_t dummy_utime_;

    Eigen::Isometry3d world_to_robot_waist_;    
    
    Eigen::Vector3d human_to_robot_scale_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string vicon_channel_, std::string output_type_, std::string vicon_model_,
         double human_to_robot_scale_factor):
    lcm_(lcm_), verbose_(verbose_), vicon_channel_(vicon_channel_), 
    output_type_(output_type_), vicon_model_(vicon_model_){
  
  lcmgl_= bot_lcmgl_init(lcm_->getUnderlyingLCM(), "teleop-4markers");
  lcm_->subscribe("EST_ROBOT_STATE",&Pass::robotStateHandler,this);  
  lcm_->subscribe( vicon_channel_ ,&Pass::viconHandler,this);

  float colors_a[] ={1.0,0.0,0.0};
  vector <float> colors_v;
  colors_v.assign(colors_a,colors_a+4*sizeof(float));
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(70000,"World to Segment",5,0) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(70002,"Waist to Segment",5,0) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(70004,"Waist to Segment Scaled",5,0) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(70006,"Robot World to Segment Scaled",5,0) );
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Null",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Markers (World)"           ,1,0, 1000,1, colors_v ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1002,"Markers (Waist)"           ,1,0, 1000,1, colors_v ));

  pc_vis_->obj_cfg_list.push_back( obj_cfg(1003,"Left Goal  (Waist)",4,0) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1004,"Left Goal  (World)",4,0) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1005,"Right Goal (Waist)",4,0) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1006,"Right Goal (World)",4,0) );

  vis_counter_ =0;  
  printf_counter_ =0;

  // Currently the human_to_robot_scale is a single number in x,y,z:  
  human_to_robot_scale_= Eigen::Vector3d(human_to_robot_scale_factor,human_to_robot_scale_factor,human_to_robot_scale_factor);

  first_rstate_received_=false;
  
  dummy_utime_=0;
  cout << "Finished setting up\n";
}

void Pass::visualizeIncomingData(const viconstructs::vicon_t *msg){
  //plot points for all markers in vicon world
  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_point_size(lcmgl_, 3.5f);
  bot_lcmgl_begin(lcmgl_, GL_POINTS);  // render as points
  bot_lcmgl_color3f(lcmgl_, 0, 0, 1); // Blue

  int i;
  for (i=0; i < msg->models[0].nummarkers; ++i) {
    double xyz[3];
    xyz[0] = msg->models[0].markers[i].xyz.x/1000.0;
    xyz[1] = msg->models[0].markers[i].xyz.y/1000.0;
    xyz[2] = msg->models[0].markers[i].xyz.z/1000.0;
//    std::cout << xyz[0] <<"|"<< xyz[1] <<"|"<< xyz[2] <<"\n";
    bot_lcmgl_vertex3f(lcmgl_, xyz[0], xyz[1], xyz[2]);
  }
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);

  //plot points for segments in vicon world:
  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_point_size(lcmgl_, 3.5f);
  bot_lcmgl_begin(lcmgl_, GL_POINTS);  // render as points
  bot_lcmgl_color3f(lcmgl_, 1, 0, 0); //

  for (i=0; i < msg->models[0].numsegments; ++i) {
    double xyz_segments[3];
    xyz_segments[0] = msg->models[0].segments[i].T.x/1000.0;
    xyz_segments[1] = msg->models[0].segments[i].T.y/1000.0;
    xyz_segments[2] = msg->models[0].segments[i].T.z/1000.0;
    //cout << xyz[0] <<"|"<< xyz[1] <<"|"<< xyz[2] <<"\n";
    bot_lcmgl_vertex3f(lcmgl_, xyz_segments[0], xyz_segments[1], xyz_segments[2]);
  }
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);

  // Draw lines between connected components:
  /*
  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_point_size(lcmgl_, 3.0f);
  bot_lcmgl_begin(lcmgl_, GL_LINES);  // render as lines
  bot_lcmgl_color3f(lcmgl_, 1, 0, 0); // Red
  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_RightShoulder_c[0] , msgb->Segment_RightShoulder_c[1], msgb->Segment_RightShoulder_c[2] );
  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_RightHand_c[0] , msgb->Segment_RightHand_c[1], msgb->Segment_RightHand_c[2] );
  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_RightShoulder_c[0] , msgb->Segment_RightShoulder_c[1], msgb->Segment_RightShoulder_c[2] );
  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_LeftShoulder_c[0] , msgb->Segment_LeftShoulder_c[1], msgb->Segment_LeftShoulder_c[2] );
  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_LeftHand_c[0] , msgb->Segment_LeftHand_c[1], msgb->Segment_LeftHand_c[2] );
  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_LeftShoulder_c[0] ,   msgb->Segment_LeftShoulder_c[1], msgb->Segment_LeftShoulder_c[2] );
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);
  */

//  bot_lcmgl_push_matrix(lcmgl_);
//  bot_lcmgl_begin(lcmgl_, GL_POINTS);  // render as points
//  bot_lcmgl_color3f(lcmgl_, 0, 1, 0); // Green
//  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_RightShoulder_a[0] , msgb->Segment_RightShoulder_a[1]/100.0, msgb->Segment_RightShoulder_a[2] );
//  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_RightHand_a[0] , msgb->Segment_RightHand_a[1], msgb->Segment_RightHand_a[2] );
//  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_LeftHand_a[0] , msgb->Segment_LeftHand_a[1], msgb->Segment_LeftHand_a[2] );
//  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_LeftShoulder_a[0] ,         msgb->Segment_LeftShoulder_a[1], msgb->Segment_LeftShoulder_a[2] );
//  bot_lcmgl_end(lcmgl_);
//  bot_lcmgl_pop_matrix(lcmgl_);

  bot_lcmgl_switch_buffer(lcmgl_);
}



void Pass::publishEndEffectorGoal(Isometry3dTime waist_to_segment , std::string channel, std::string ee_name) //KDL::Frame &T_body_ee, 
{
  drc::ee_goal_t goalmsg;
  goalmsg.robot_name = "atlas";
  goalmsg.ee_name = ee_name;
  goalmsg.root_name = "utorso";
  double x,y,z,w;

  goalmsg.ee_goal_pos.translation.x = waist_to_segment.pose.translation().x();
  goalmsg.ee_goal_pos.translation.y = waist_to_segment.pose.translation().y();
  goalmsg.ee_goal_pos.translation.z = waist_to_segment.pose.translation().z();

  Eigen::Quaterniond r(waist_to_segment.pose.rotation());
  goalmsg.ee_goal_pos.rotation.x = r.x();
  goalmsg.ee_goal_pos.rotation.y = r.y();
  goalmsg.ee_goal_pos.rotation.z = r.z();
  goalmsg.ee_goal_pos.rotation.w = r.w();

  goalmsg.ee_goal_twist.linear_velocity.x = 0.0;
  goalmsg.ee_goal_twist.linear_velocity.y = 0.0;
  goalmsg.ee_goal_twist.linear_velocity.z = 0.0;
  goalmsg.ee_goal_twist.angular_velocity.x = 0.0;
  goalmsg.ee_goal_twist.angular_velocity.y = 0.0;
  goalmsg.ee_goal_twist.angular_velocity.z = 0.0;

  goalmsg.num_chain_joints  = 6;
  // No specified posture bias
  goalmsg.use_posture_bias  = false;
  goalmsg.joint_posture_bias.resize(goalmsg.num_chain_joints);
  goalmsg.chain_joint_names.resize(goalmsg.num_chain_joints);
  for(int i = 0; i < goalmsg.num_chain_joints; i++){
      goalmsg.joint_posture_bias[i]=0;
      goalmsg.chain_joint_names[i]= "dummy_joint_names";
  }

  // Publish the message
  goalmsg.halt_ee_controller = false;

  lcm_->publish(channel, &goalmsg);
};


void getSegmentPose(viconstructs::segment_t  &segment, Eigen::Isometry3d &pose){
  pose.setIdentity();
  Eigen::Matrix3d m;
  //std::cout << segment.A.x << " " << segment.A.y << " " << segment.A.z <<"\n";
  m = Eigen::AngleAxisd(  segment.A.z  , Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd( segment.A.y , Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd( segment.A.x , Eigen::Vector3d::UnitZ());    

  pose *=m;
  Eigen::Vector3d v( segment.T.x/1000.0 , segment.T.y/1000.0 , segment.T.z/1000.0 );
  pose.translation() = v;   
}

void Pass::viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  viconstructs::vicon_t* msg){
  dummy_utime_=dummy_utime_+100 ; // replace this with timestamp in message - when i add it
  dummy_utime_ = dummy_utime_%100;

  if (!first_rstate_received_){
    cout << "No Robot State yet. ignoring vicon data\n";
    return; 
  }
  // Occasionally say we are still alive:
  if (printf_counter_%100 ==0){
    cout << "Vicon: " << vicon_channel_ << " "  << msg->nummodels << " nummodels\n";
  }
  printf_counter_++;    
  visualizeIncomingData(msg);
  
  // 1. Check there is one model in the message and that it is the one we expect:
  if (msg->nummodels !=1){
    std::cout << "Number of models is " <<msg->nummodels << " ... but we only support one model\n";
    return;
  }
  viconstructs::model_t model = msg->models[0];
  if (model.name.compare( vicon_model_ ) != 0){
    cout <<"Incoming model ["<< model.name << "] does not match expected model ["<< vicon_model_ <<"... exiting\n";
    return;
  }

  // 2 Extract the segments - and specifically the waist
  std::vector< Isometry3dTime > world_to_segmentTs;
  std::vector< std::string > seg_names;
  std::vector< int64_t > seg_utimes;
  Eigen::Isometry3d world_to_waist;
  for(int j = 0; j < model.numsegments; j++){
    viconstructs::segment_t segment = model.segments[j];
    seg_utimes.push_back( dummy_utime_+j );
    Isometry3dTime segment_poseT = Isometry3dTime( dummy_utime_ + j , Eigen::Isometry3d::Identity() );
    getSegmentPose(segment, segment_poseT.pose );
    world_to_segmentTs.push_back( segment_poseT );
    
    seg_names.push_back( segment.name);
    if (segment.name.compare( "Segment_Waist" ) == 0){
      world_to_waist = segment_poseT.pose;
      // cout << "got waist" << j<< "\n";
    }
  }
  pc_vis_->pose_collection_to_lcm_from_list(70000, world_to_segmentTs); // all joints in world frame
  pc_vis_->text_collection_to_lcm(70001, 70000, "World Rel [Labels]", seg_names, seg_utimes );    
  
  
  // 3 Find segments relative to the person waist
  std::vector< Isometry3dTime > waist_to_segmentTs;
  for (size_t j=0; j < world_to_segmentTs.size() ; j++){
    Eigen::Isometry3d segment_pose = world_to_waist.inverse() * world_to_segmentTs[j].pose ;
    Isometry3dTime segment_poseT = Isometry3dTime( dummy_utime_ + j , segment_pose );
    waist_to_segmentTs.push_back(  segment_poseT   );
  }
  pc_vis_->pose_collection_to_lcm_from_list(70002, waist_to_segmentTs); // all joints in world frame
  pc_vis_->text_collection_to_lcm(70003, 70002, "Waist Rel [Labels]", seg_names, seg_utimes );    
  
  
  // 4. Scale the relative segment positions:
  std::vector< Isometry3dTime > waist_to_segmentTs_scaled = waist_to_segmentTs;
  for (size_t j=0; j < world_to_segmentTs.size() ; j++){
    waist_to_segmentTs_scaled[j].pose.translation().x() *= human_to_robot_scale_(0);
    waist_to_segmentTs_scaled[j].pose.translation().y() *= human_to_robot_scale_(1);
    waist_to_segmentTs_scaled[j].pose.translation().z() *= human_to_robot_scale_(2);
  }
  pc_vis_->pose_collection_to_lcm_from_list(70004, waist_to_segmentTs_scaled); // all joints in world frame
  pc_vis_->text_collection_to_lcm(70005, 70004, "Waist to Segment Scaled [Labels]", seg_names, seg_utimes );    
  

  // 5. Place relative segments onto robot (only visual):
  std::vector< Isometry3dTime > robotworld_to_segmentTs_scaled;
  for (size_t j=0; j < waist_to_segmentTs_scaled.size() ; j++){
    Isometry3dTime robotworld_to_segmentT = waist_to_segmentTs_scaled[j];
    robotworld_to_segmentT.pose = world_to_robot_waist_*robotworld_to_segmentT.pose;
    robotworld_to_segmentTs_scaled.push_back(robotworld_to_segmentT );
  }
  pc_vis_->pose_collection_to_lcm_from_list(70006, robotworld_to_segmentTs_scaled); // all joints in world frame
  pc_vis_->text_collection_to_lcm(70007, 70006, "Robot World to Segment Scaled [Labels]", seg_names, seg_utimes );    
  
  

  // 6a Extract and visualize the raw markers - both at the original positions ...
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr world_marker_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  for(int j = 0; j < model.nummarkers; j++){
    PointXYZRGB pt;
    pt.x = model.markers[j].xyz.x / 1000.0; 
    pt.y = model.markers[j].xyz.y / 1000.0; 
    pt.z = model.markers[j].xyz.z / 1000.0; 
    world_marker_cloud->points.push_back(pt);
  }
  Eigen::Isometry3d null_pose;
  null_pose.setIdentity();
  Isometry3dTime null_poseT = Isometry3dTime(dummy_utime_, null_pose);
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT);  
  pc_vis_->ptcld_to_lcm_from_list(1001, *world_marker_cloud, dummy_utime_, dummy_utime_);
  // 6b .... and now relative to the waist
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr waist_relative_marker_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  Eigen::Isometry3f world_to_waist_f = Isometry_d2f( world_to_waist.inverse()  );
  Eigen::Quaternionf pose_quat_f(world_to_waist_f.rotation());
  pcl::transformPointCloud (*world_marker_cloud, *waist_relative_marker_cloud,
    world_to_waist_f.translation(), pose_quat_f);  
  pc_vis_->ptcld_to_lcm_from_list(1002, *waist_relative_marker_cloud, dummy_utime_, dummy_utime_);
  
  
  // 7. Determine the end effector goals and publish them to the controllers:
  if ((output_type_.compare( "left" ) == 0) || (output_type_.compare( "both" ) == 0) ) {
    string goal_seg = "Segment_LeftHand";
    int j = std::distance( seg_names.begin(), std::find( seg_names.begin(), seg_names.end(), goal_seg ) );
    Isometry3dTime waist_to_goal = waist_to_segmentTs_scaled[j];
    pc_vis_->pose_to_lcm_from_list(1003, waist_to_goal); 
    publishEndEffectorGoal( waist_to_segmentTs_scaled[j],  "L_HAND_GOAL","l_hand");

    Isometry3dTime robotworld_to_goal = waist_to_goal;
    robotworld_to_goal.pose = world_to_robot_waist_*waist_to_goal.pose;
    pc_vis_->pose_to_lcm_from_list(1004, robotworld_to_goal);
  }
    
  if ((output_type_.compare( "right" ) == 0) || (output_type_.compare( "both" ) == 0) ) {
    string goal_seg = "Segment_RightHand";
    int j = std::distance( seg_names.begin(), std::find( seg_names.begin(), seg_names.end(), goal_seg ) );
    Isometry3dTime waist_to_goal = waist_to_segmentTs_scaled[j];
    pc_vis_->pose_to_lcm_from_list(1005, waist_to_goal); 
    publishEndEffectorGoal( waist_to_segmentTs_scaled[j],  "R_HAND_GOAL","r_hand");

    Isometry3dTime robotworld_to_goal = waist_to_goal;
    robotworld_to_goal.pose = world_to_robot_waist_*waist_to_goal.pose;
    pc_vis_->pose_to_lcm_from_list(1006, robotworld_to_goal);
  }
}

void Pass::robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  if (first_rstate_received_==false){
    cout << "got first Robot State @ " << msg->utime << "\n";
    first_rstate_received_=true;
  }

  // Extract World to Body TF:
  world_to_robot_waist_.setIdentity();
  world_to_robot_waist_.translation()  << msg->origin_position.translation.x, msg->origin_position.translation.y, msg->origin_position.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->origin_position.rotation.w, msg->origin_position.rotation.x, 
                                               msg->origin_position.rotation.y, msg->origin_position.rotation.z);
  world_to_robot_waist_.rotate(quat);
}


int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool verbose=FALSE;
  string vicon_channel="drc_vicon";
  string output_type="both";
  string vicon_model="DRC_PALADIN_MODEL_v1";
  double scale=1.3;
  parser.add(verbose, "v", "verbose", "Verbosity");
  parser.add(vicon_channel, "l", "vicon_channel", "Incoming LIDAR channel");
  parser.add(output_type, "o", "output_type", "Command: left right both");
  parser.add(vicon_model, "v", "vicon_model", "Name of Vicon Model");
  parser.add(scale, "s", "scale", "Human to Robot Scale Factor");
  parser.parse();
  cout << verbose << " is verbose\n";
  cout << vicon_channel << " is vicon_channel\n";
  cout << output_type << " is output_type\n";
  cout << vicon_model << " is vicon_model\n";
  cout << scale << " is scale\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm, verbose, vicon_channel, output_type, vicon_model, scale);
  cout << "Ready to process incoming LCM" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}