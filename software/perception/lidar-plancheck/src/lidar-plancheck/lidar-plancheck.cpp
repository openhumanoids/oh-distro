// Plan checker for collisions with lidar points and self-collisions
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
struct GFEFilterCallback : public btOverlapFilterCallback
{
  const boost::shared_ptr<const KDL::Tree> gfe_tree_;

  GFEFilterCallback(const Collision_Object_GFE* gfe) : gfe_tree_(&(gfe->kinematics_model().tree())) {};
  // return true when pairs need collision
  virtual bool  needBroadphaseCollision(btBroadphaseProxy* proxy0,
                                        btBroadphaseProxy* proxy1) const
  {
    bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
    collides = collides && (proxy1->m_collisionFilterGroup &
        proxy0->m_collisionFilterMask);

    //add some additional logic here that modified 'collides'
    if (collides) {
      vector< string > ignored_links{"hokuyo_link", "l_uglut", "l_lglut", "r_uglut", "r_lglut","ltorso","mtorso"};
      map< string, vector<string> > pseudo_children;
      pseudo_children["pelvis"] = vector<string>({"l_uleg","r_uleg"});
      pseudo_children["l_uleg"] = vector<string>({"r_uleg"});
      btCollisionObject* bt_collision_object0 = (btCollisionObject*) proxy0->m_clientObject;
      btCollisionObject* bt_collision_object1 = (btCollisionObject*) proxy1->m_clientObject;
      if ((bt_collision_object0->getUserPointer() == NULL) || 
          (bt_collision_object1->getUserPointer() == NULL)) {
          return false;
      }
      const KDL::TreeElement* element0 = 
        (KDL::TreeElement*) bt_collision_object0->getUserPointer();
      const KDL::TreeElement* element1 = 
        (KDL::TreeElement*) bt_collision_object1->getUserPointer();
      string name0 = element0->segment.getName();
      string name1 = element1->segment.getName();
      if (find(ignored_links.begin(),ignored_links.end(),name0) != ignored_links.end()) {
          return false;
      }
      if (find(ignored_links.begin(),ignored_links.end(),name1) != ignored_links.end()) {
          return false;
      }
      //cout << "Checking pair: (" << name0 << ", " << name1 << ")" << endl;
      KDL::SegmentMap::const_iterator it0 = gfe_tree_->getSegment(name0);
      KDL::SegmentMap::const_iterator it1 = gfe_tree_->getSegment(name1);
      collides = (it0 != it1);
      for (KDL::SegmentMap::const_iterator child : it0->second.children) {
        collides = collides && (child != it1);
        if (collides){
          for (KDL::SegmentMap::const_iterator grandchild : child->second.children) {
            collides = collides && (grandchild != it1);
            //for (KDL::SegmentMap::const_iterator greatgrandchild : grandchild->second.children) {
              //collides = collides && (greatgrandchild != it1);
            //}
            //if (!collides) {
              //cout << name0 << " is the great-grand-parent of " << name1 << ". Ignoring collisions for this pair ..." << endl;
            //}
          }
          if (!collides) {
            //cout << name0 << " is the grand-parent of " << name1 << ". Ignoring collisions for this pair ..." << endl;
          }
        } else {
          //cout << name0 << " is the parent of " << name1 << ". Ignoring collisions for this pair ..." << endl;
        }
      }
      auto pseudo_children0_it = pseudo_children.find(name0);
      if (pseudo_children0_it != pseudo_children.end()){
        for (string pchild : pseudo_children0_it->second) {
          collides = collides && (pchild != name1);
        }
      }
      for (KDL::SegmentMap::const_iterator child : it1->second.children) {
        collides = collides && (child != it0);
        if (collides){
          for (KDL::SegmentMap::const_iterator grandchild : child->second.children) {
            collides = collides && (grandchild != it0);
            //for (KDL::SegmentMap::const_iterator greatgrandchild : grandchild->second.children) {
              //collides = collides && (greatgrandchild != it0);
            //}
            //if (!collides) {
              //cout << name1 << " is the great-grand-parent of " << name0 << ". Ignoring collisions for this pair ..." << endl;
            //}
          }
          if (!collides) {
            //cout << name1 << " is the grand-parent of " << name0 << ". Ignoring collisions for this pair ..." << endl;
          }
        } else {
          //cout << name1 << " is the parent of " << name0 << ". Ignoring collisions for this pair ..." << endl;
        }
      }
      auto pseudo_children1_it = pseudo_children.find(name1);
      if (pseudo_children1_it != pseudo_children.end()){
        for (string pchild : pseudo_children1_it->second) {
          collides = collides && (pchild != name0);
        }
      }
    }
    return collides;
  }
};

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

class PlanCheck{
  public:
    PlanCheck(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
        std::string lidar_channel_, double collision_threshold_,
        bool simulated_data_, double delta_threshold_);

    ~PlanCheck(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    boost::shared_ptr<ModelClient> model_;
    bool verbose_;
    bool simulated_data_;
    std::string lidar_channel_;

    double collision_threshold_;
    double delta_threshold_;

    void urdfHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_urdf_t* msg);
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg);   
    void robotPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_plan_t* msg);   
    void robotPlanWKeyframesHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_plan_w_keyframes_t* msg);   

    Collision_Object_GFE* collision_object_gfe_;
    Collision_Object_Point_Cloud* collision_object_point_cloud_;
    Collision_Detector* collision_detector_;
    int n_collision_points_;

    void DoCollisionCheck(int64_t current_utime);
    void ProcessLastLidar(vector< Vector3f > points, vector<unsigned int> possible_indices);

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
    // Robot plan to be checked for collisions
    drc::robot_plan_t rplan_;
    // Scan as pointcloud in local frame: this is used as the collision points
    pcl::PointCloud<PointXYZRGB>::Ptr scan_cloud_s2l_;    

    // Output filter lidar:
    bot_core::planar_lidar_t last_lidar_msg_;
    bool init_lidar_;
};

PlanCheck::PlanCheck(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
    std::string lidar_channel_, double collision_threshold_,
    bool simulated_data_, double delta_threshold_):
  lcm_(lcm_), verbose_(verbose_), 
  lidar_channel_(lidar_channel_),urdf_parsed_(false),
  simulated_data_(simulated_data_), delta_threshold_(delta_threshold_){
    botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
    botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

    model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));

    // TODO: get the urdf model from LCM:
    collision_object_gfe_ = new Collision_Object_GFE( "collision-object-gfe", model_->getURDFString(), COLLISION_OBJECT_GFE_COLLISION_OBJECT_VISUAL );
    // Add pointer to KDL tree elements to the btCollisionObjects so that we
    // can filter out pairs of adjacent links during Bullet's broadphase.
    std::vector< Collision_Object* > gfe_collision_objects(collision_object_gfe_->collision_objects());
    const KDL::Tree& gfe_tree = collision_object_gfe_->kinematics_model().tree();
    for (Collision_Object* collision_object : gfe_collision_objects) {
      for (btCollisionObject* bt_collision_object : collision_object->bt_collision_objects()) {
        bt_collision_object->setUserPointer(const_cast<KDL::TreeElement *>(&(gfe_tree.getSegment(collision_object->id())->second)));
        // We're not trying to resolve collisions as they occur, so using a
        // margin only gets us false positives.
        bt_collision_object->getCollisionShape()->setMargin(0.0);
      }
    }
    n_collision_points_ = 1081; // was 1000, real lidar from sensor head has about 1081 returns (varies)

    collision_object_point_cloud_ = new Collision_Object_Point_Cloud( "collision-object-point-cloud", n_collision_points_ , collision_threshold_);
    // create the collision detector
    collision_detector_ = new Collision_Detector();
    btOverlapFilterCallback* filter_callback = new GFEFilterCallback(collision_object_gfe_);
    collision_detector_->bt_collision_world().getPairCache()->setOverlapFilterCallback(filter_callback);
    // add the two collision objects to the collision detector (note: no
    // special grouping is used here, as we want to check self-collisions. The
    // filter callback should take care adjacent links)
    collision_detector_->add_collision_object( collision_object_gfe_, COLLISION_DETECTOR_GROUP_1);
    collision_detector_->add_collision_object( collision_object_point_cloud_, COLLISION_DETECTOR_GROUP_2, COLLISION_DETECTOR_GROUP_1 );   


    lcmgl_= bot_lcmgl_init(lcm_->getUnderlyingLCM(), "lidar-pt");
    lcm_->subscribe( lidar_channel_ ,&PlanCheck::lidarHandler,this);
    lcm_->subscribe("CANDIDATE_ROBOT_PLAN",&PlanCheck::robotPlanHandler,this);
    lcm_->subscribe("CANDIDATE_MANIP_PLAN",&PlanCheck::robotPlanWKeyframesHandler,this);

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

    init_lidar_ =false;

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


void PlanCheck::DoCollisionCheck(int64_t current_utime ){

#if DO_TIMING_PROFILE
  std::vector<int64_t> tic_toc;
  tic_toc.push_back(_timestamp_now());
#endif  


  // 2. Check for collisions
  if (init_lidar_) {
    // set the state of the lidar collision objects
    vector< Vector3f > points;
    vector<unsigned int> possible_indices; // the indices of points that could possibly be in intersection: not to near and not too far
    ProcessLastLidar(points,possible_indices);
    if (points.size() > 0) {
      collision_object_point_cloud_->set( points );
    }
  }

  drc::robot_plan_t in_collision_plan(rplan_);
  drc::robot_collision_array_t collision_msg;
  collision_msg.utime = rplan_.utime;
  in_collision_plan.plan.clear();
  vector< vector<Collision> > collisions;
  int i = 0;
  for (drc::robot_state_t rstate : rplan_.plan) {
    cout << "PlanCheck::DoCollisionCheck: Checking state " << i << endl;
    collision_object_gfe_->set( rstate );
    collisions.push_back(collision_detector_->get_collisions());
    if (collisions.back().size() > 0) {
      drc::robot_collision_t collision_msg_i;
      collision_msg_i.utime = rstate.utime;
      collision_msg_i.plan_idx = i;
      cout << "PlanCheck::DoCollisionCheck: State " << i << " is in collision. Adding it to output plan ..." << endl;
      in_collision_plan.plan.push_back(rstate);
      cout << "Links in collision: " << endl;
      for (Collision collision : collisions.back()) {
        if (find(collision_msg_i.links_in_collision.begin(), 
                 collision_msg_i.links_in_collision.end(),
                 collision.first_id()) 
            == collision_msg_i.links_in_collision.end()){
          collision_msg_i.links_in_collision.push_back(collision.first_id());
        }
        if (find(collision_msg_i.links_in_collision.begin(), 
                 collision_msg_i.links_in_collision.end(),
                 collision.second_id()) 
            == collision_msg_i.links_in_collision.end()){
          collision_msg_i.links_in_collision.push_back(collision.second_id());
        }
        cout << "\t" << collision.first_id() << " with " << collision.second_id() << endl;
      }
      collision_msg_i.num_links_in_collision = collision_msg_i.links_in_collision.size();
      collision_msg.collision_states.push_back(collision_msg_i);
    }
    i++;
  }
  collision_msg.num_collision_states = collision_msg.collision_states.size();
  // 4. Output plan containing in-collision states
  if (in_collision_plan.plan.size() > 0) {
    in_collision_plan.num_states = in_collision_plan.plan.size();
    cout << "PlanCheck::DoCollisionCheck: Publishing in-collision plan" << endl;
    lcm_->publish( ("ROBOT_COLLISIONS") , &collision_msg);        
  } else {
    cout << "PlanCheck::DoCollisionCheck: Plan is clean" << endl;
  }

  //cout << "gfe obj size: " << collision_object_gfe_->bt_collision_objects().size() << "\n";
  
  // get the vector of collisions by running collision detection
  //int64_t tic = _timestamp_now();
  //cout << last_lidar_msg_.ranges.size() << " and " << points.size() << " and " << scan_cloud_s2l_->points.size() << " " <<  (_timestamp_now() - tic)*1e-6 << " dt\n";;


  // 3. Extract the indices of the points in collision and modify the outgoing ranges as a result:
  //std::vector<float> original_ranges =  last_lidar_msg_.ranges;


  //vector< Vector3f > free_points;
  //vector< Vector3f > colliding_points;      
  //vector< unsigned int > filtered_point_indices;
  //for( unsigned int j = 0; j < collisions.size(); j++ ){
    //filtered_point_indices.push_back( atoi( collisions[ j ].second_id().c_str() ) );
  //}
  //for( unsigned int j = 0; j < points.size(); j++ ){
    //for( unsigned int k = 0; k < filtered_point_indices.size(); k++ ){
      //if( j == filtered_point_indices[ k ] ){
        //last_lidar_msg_.ranges[  possible_indices[j] ] = COLLISION_RANGE;// 15.0; // Set filtered returns to max range
      //}
    //}
  //}
  //for( unsigned int j = 0; j < last_lidar_msg_.ranges.size(); j++ ){
    //if (original_ranges[j] < ASSUMED_HEAD ){
      //last_lidar_msg_.ranges[j] = COLLISION_RANGE;//20.0; // Set filtered returns to max range
    //}  

    // For Real Data: apply an addition set of filters
    // NB: These filters are not compatiable with Gazebo Simulation output NBNBNB
    //if (!simulated_data_){
      // heuristic filtering of the weak intensity lidar returns
      //if (( last_lidar_msg_.intensities[j] < INTENSITY_FILTER_MIN_VALUE ) && ( original_ranges[j] < INTENSITY_FILTER_MIN_RANGE) ){
        //last_lidar_msg_.ranges[j] = MAX_RANGE;
      //}

      // Edge effect filter
      //if ( (j>0) && (j<last_lidar_msg_.ranges.size()) ){
        //float right_diff = fabs(original_ranges[j] - original_ranges[j-1]);
        //float left_diff = fabs(original_ranges[j] - original_ranges[j+1]);
        //if (( right_diff > delta_threshold_) || (left_diff > delta_threshold_ )){
          // cout << i<< ": " << right_diff << " and " << left_diff <<"\n";
          //if (original_ranges[j] < EDGE_FILTER_MIN_RANGE){
            //last_lidar_msg_.ranges[j] = MAX_RANGE;
          //}
        //}
      //}      

    //}

  //}


  ///////////////////////////////////////////////////////////////////////////
  //if (verbose_){
    //for( unsigned int j = 0; j < points.size(); j++ ){
      //bool point_filtered = false;
      //for( unsigned int k = 0; k < filtered_point_indices.size(); k++ ){
        //if( j == filtered_point_indices[ k ] ){
          //point_filtered = true;
        //}
      //}
      //if( point_filtered ){
        //colliding_points.push_back( points[ j ] );
      //} else {
        //free_points.push_back( points[ j ] );
      //}
    //}    

    //cout << current_utime << " | total returns "<< scan_cloud_s2l_->points.size()  
      //<< " | colliding " << colliding_points.size() << " | free " << free_points.size() << endl;
    //bot_lcmgl_point_size(lcmgl_, 4.5f);
    //bot_lcmgl_color3f(lcmgl_, 0, 1, 0);
    //for (int i = 0; i < free_points.size(); ++i) {
      //bot_lcmgl_begin(lcmgl_, LCMGL_POINTS);
      //bot_lcmgl_vertex3f(lcmgl_, free_points[i][0], free_points[i][1], free_points[i][2]);
      //bot_lcmgl_end(lcmgl_);
    //}

    //bot_lcmgl_point_size(lcmgl_, 10.5f);
    //bot_lcmgl_color3f(lcmgl_, 1, 0, 0);
    //for (int i = 0; i < colliding_points.size(); ++i) {
      //bot_lcmgl_begin(lcmgl_, LCMGL_POINTS);
      //bot_lcmgl_vertex3f(lcmgl_, colliding_points[i][0], colliding_points[i][1], colliding_points[i][2]);
      //bot_lcmgl_end(lcmgl_);
    //}
    //bot_lcmgl_switch_buffer(lcmgl_);  
  //}

//#if DO_TIMING_PROFILE
  //tic_toc.push_back(_timestamp_now());
  //double dt =  ((tic_toc[1] - tic_toc[0])*1E-6);

  //std::cout << dt << " | " << 1/dt  << " | "
    //<< current_utime << " | total returns "<< scan_cloud_s2l_->points.size()  
    //<< " | collisions " << collisions.size() << " | indices " << filtered_point_indices.size() << endl;    
//#endif    

}

void PlanCheck::ProcessLastLidar(vector< Vector3f > points, vector<unsigned int> possible_indices){
   //A counter for visualization:
  vis_counter_++;
  if (vis_counter_ >=1){ // set this to 1 to only see the last return
    vis_counter_=0;
  }
  int64_t pose_id=vis_counter_;
   //int64_t pose_id=last_lidar_msg_.utime;

  // 0. Convert scan into simple XY point cloud:  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  // consider everything - don't remove any points
  double minRange =-100.0;
  double maxRange = 100.0;
  double validBeamAngles[] ={-10,10}; 
  convertLidar(last_lidar_msg_.ranges, last_lidar_msg_.nranges, last_lidar_msg_.rad0,
      last_lidar_msg_.radstep, scan_cloud, minRange, maxRange,
      validBeamAngles[0], validBeamAngles[1]);  

  if (scan_cloud->points.size() !=  last_lidar_msg_.nranges ){
    std::cout << "npoints and nranges are not equal\n";
    std::cout << scan_cloud->points.size() << "\n";
    std::cout << last_lidar_msg_.nranges << "\n";
    exit(-1); 
  }  

  // 3. Project the scan into local frame:
  Eigen::Isometry3d scan_to_local;
  frames_cpp_->get_trans_with_utime( botframes_ ,  lidar_channel_.c_str() , "local", last_lidar_msg_.utime, scan_to_local);
  Eigen::Isometry3f pose_f = scan_to_local.cast<float>();
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
  
  // 2. create the list of points to be considered:
  int which=2; // 0 actual lidar returns | Test modes: 1 random points in a box, 2 a line [for testing], 3 a 2d grid
  if (which==0){
    for( unsigned int i = 0; i < scan_cloud_s2l_->points.size() ; i++ ){
      if ( (last_lidar_msg_.ranges[i] > ASSUMED_HEAD  ) &&(last_lidar_msg_.ranges[i] < ASSUMED_FAR )){
        Vector3f point(scan_cloud_s2l_->points[i].x, scan_cloud_s2l_->points[i].y, scan_cloud_s2l_->points[i].z );
        points.push_back( point );
        possible_indices.push_back(i);
      }
    }
  //}else if (which ==1){
    //for( unsigned int i = 0; i < 500; i++ ){
      //Vector3f point(last_rstate_.pose.translation.x +  -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0,
          //last_rstate_.pose.translation.y + -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0,
          //last_rstate_.pose.translation.z +  -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0 );
      //points.push_back( point );
      //possible_indices.push_back( points.size() - 1 );
    //}
  }else if(which==2){
    for( unsigned int i = 0; i < 500; i++ ){
      Vector3f point(-0.0, -1 + 0.005*i,1.4);
      points.push_back( point );
      possible_indices.push_back( points.size() - 1 );
    }
  }//else if(which==3){
    //Vector3f bot_root(last_rstate_.pose.translation.x,last_rstate_.pose.translation.y,last_rstate_.pose.translation.z);
    //Vector3f offset( 0.,0.,0.45);
    //for( float i = -0.3; i < 0.3; i=i+0.02 ){
      //for( float j = -0.3; j < 0.3; j=j+0.02 ){
        //Vector3f point =  Vector3f(i,j,0.) + bot_root + offset;
        //points.push_back( point );
        //possible_indices.push_back( points.size() - 1 );
        //if (points.size() > n_collision_points_){
          //break; 
        //}
      //}
      //if (points.size() > n_collision_points_){
        //break; 
      //}      
    //}
  //}

}

void PlanCheck::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
  last_lidar_msg_ = *msg;
  init_lidar_ = true;
}


void PlanCheck::robotPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_plan_t* msg){
  rplan_ = *msg;  
  DoCollisionCheck(msg->utime);
}

void PlanCheck::robotPlanWKeyframesHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_plan_w_keyframes_t* msg){
  rplan_.utime = msg->utime;
  rplan_.robot_name = msg->robot_name;
  rplan_.num_states = msg->num_states;
  rplan_.plan = msg->plan;
  rplan_.plan_info = msg->plan_info;
  rplan_.num_bytes = msg->num_bytes;
  rplan_.matlab_data = msg->matlab_data;
  rplan_.num_grasp_transitions = msg->num_grasp_transitions;
  DoCollisionCheck(msg->utime);
}
int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool verbose=FALSE;
  string lidar_channel="SCAN";
  // was 0.04 for a long time
  // using 0.06 for the simulator
  // using 0.1 for the real robot - until the new urdf arrives ... aug 2013
  double collision_threshold = 0.02; 
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

  PlanCheck app(lcm,verbose,lidar_channel, collision_threshold, 
      simulated_data, delta_threshold);
  cout << "Ready to check plans for collisions" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
