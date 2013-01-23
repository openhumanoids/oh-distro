// TODO: Can significantly reduce the latency by transmitting the image timestamp directly 
// to this program - which will be quicker than waiting for the message to show up alone.
// TODO: - Improve mergePolygonMesh() code avoiding mutliple conversions and keeping the polygon list
//       - Improve scene.draw() by giving it directly what it need - to avoid conversion.
//       - One of these can vastly increase the rendering speed:
// BENCHMARK as of jan 2013:
// - with convex hull models: (about 20Hz)
//   component solve fk  , createScene  render  , sendOuput
//   fraction: 0.00564941, 0.0688681  , 0.185647, 0.739836, 
//   time sec: 0.00031   , 0.003779   , 0.010187, 0.040597,
// - with original models: (including complex head model) (about 5Hz)
//   component solve fk  , createScene  render  , sendOuput
//   fraction: 0.0086486 , 0.799919   , 0.072785, 0.118647, 
//   time sec: 0.001635  , 0.151223   , 0.01376 , 0.02243, 
// - These numbers are for RGB. Outputing Gray reduces sendOutput by half
//   so sending convex works at about 35Hz

#include <iostream>
#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>

#include "image-passthrough.hpp"
#include <visualization_utils/GlKinematicBody.hpp>
#include <visualization_utils/GlKinematicBody.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

#include <lcm/lcm-cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>

#define DO_TIMING_PROFILE TRUE
#include <ConciseArgs>


using namespace std;
using namespace drc;
using namespace Eigen;
using namespace boost;
using namespace boost::assign; // bring 'operator+()' into scope

class Pass{
  public:
    Pass(int argc, char** argv, boost::shared_ptr<lcm::LCM> &publish_lcm, std::string camera_channel_);
    
    ~Pass(){
    }
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void urdfHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_urdf_t* msg);
    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);   
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    std::string camera_channel_;

    bot_lcmgl_t* lcmgl_;
    pointcloud_vis* pc_vis_;
    bool verbose_;
    
    // OpenGL:
    void sendOutput(int64_t utime);
    SimExample::Ptr simexample;
    
    // Last robot state: this is used to extract link positions:
    drc::robot_state_t last_rstate_;
    bool init_rstate_;    

    // New Stuff:
    boost::shared_ptr<visualization_utils::GlKinematicBody> gl_robot_;
    std::map<std::string, boost::shared_ptr<urdf::Link> > links_map_;
    
    bool urdf_parsed_;
    bool urdf_subscription_on_;
    lcm::Subscription *urdf_subscription_; //valid as long as urdf_parsed_ == false
    std::string robot_name_;
    std::string urdf_xml_string_; 
    
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
};

Pass::Pass(int argc, char** argv, boost::shared_ptr<lcm::LCM> &lcm_, std::string camera_channel_):          
    lcm_(lcm_),urdf_parsed_(false), camera_channel_(camera_channel_){

  lcmgl_= bot_lcmgl_init(lcm_->getUnderlyingLCM(), "lidar-pt");
  lcm_->subscribe("EST_ROBOT_STATE",&Pass::robotStateHandler,this);  
  lcm_->subscribe(camera_channel_,&Pass::imageHandler,this);  
  urdf_subscription_ = lcm_->subscribe("ROBOT_MODEL", &Pass::urdfHandler,this);    
  urdf_subscription_on_ = true;
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(9999,"Pass - Pose - Left",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(9998,"Pass - Frames",5,1) );
  
  float colors_b[] ={0.0,0.0,0.0};
  std::vector<float> colors_v;
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(9995,"Pass - Pose - Left Original",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(9996,"Pass - Sim Cloud"     ,1,1, 9995,0, colors_v ));
  
  verbose_ =false;  

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

// same as bot_timestamp_now():
int64_t _timestamp_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


// Output the simulated output to file:
void Pass::sendOutput(int64_t utime){ 
  
  bool do_timing=true;
  std::vector<int64_t> tic_toc;
  if (do_timing){
    tic_toc.push_back(_timestamp_now());
  }
  simexample->write_rgb_image (simexample->rl_->getColorBuffer (), camera_channel_ );  
  if (do_timing==1){
    tic_toc.push_back(_timestamp_now());
  }
  
  //simexample->write_depth_image (simexample->rl_->getDepthBuffer (), camera_channel_ );  
  if (do_timing==1){
    tic_toc.push_back(_timestamp_now());
    display_tic_toc(tic_toc,"sendOutput");
  }
  
  /*
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_out (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Save camera relative:
  simexample->rl_->getPointCloud(pc_out,false,simexample->camera_->getPose () );
  cout << *pc_out << " pc_out info\n";
  pc_vis_->ptcld_to_lcm_from_list(9996, *pc_out, utime, utime);
  */
  
  //pcl::PCDWriter writer;
  //    writer.writeBinary (  string ("image_passthrough.pcd")  , *pc_out);
}


pcl::PolygonMesh::Ptr getPolygonMesh(std::string filename){
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile(  filename    ,mesh);
  pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh(mesh));
  cout << "read in :" << filename << "\n"; 
  //state->model = mesh_ptr;  
  return mesh_ptr;
}





void Pass::urdfHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_urdf_t* msg){
  if(urdf_parsed_ ==false){
    cout<< "URDF handler"<< endl;
    // Received robot urdf string. Store it internally and get all available joints.
    robot_name_      = msg->robot_name;
    urdf_xml_string_ = msg->urdf_xml_string;
    cout<< "Received urdf_xml_string of robot [" << msg->robot_name << "], storing it internally as a param" << endl;
    
    gl_robot_ = shared_ptr<visualization_utils::GlKinematicBody>(new visualization_utils::GlKinematicBody(urdf_xml_string_));
    cout<< "Number of Joints: " << gl_robot_->get_num_joints() <<endl;
    
    links_map_ = gl_robot_->get_links_map();
    cout<< "Size of Links Map: " << links_map_.size() <<endl;
    
    std::vector< std::string > mesh_link_names, mesh_file_paths ;
    
    typedef map<string, shared_ptr<urdf::Link> > links_mapType;
    for(links_mapType::const_iterator it =  links_map_.begin(); it!= links_map_.end(); it++){ 
      cout << it->first << endl;
      if(it->second->visual){
        std::cout << it->first<< " link"
          << it->second->visual->geometry->type << " type [visual only]\n";
        
        int type = it->second->visual->geometry->type;
        enum {SPHERE, BOX, CYLINDER, MESH}; 
        if  (type == MESH){
          shared_ptr<urdf::Mesh> mesh(shared_dynamic_cast<urdf::Mesh>(it->second->visual->geometry));
          
          mesh_link_names.push_back( it->first );
          // Verify the existance so the file:
          mesh_file_paths.push_back( gl_robot_->evalMeshFilePath(mesh->filename) );
        }
      }
    }
    
    simexample->setPolygonMeshs(mesh_link_names, mesh_file_paths );
    
    
    //////////////////////////////////////////////////////////////////
    // Get a urdf Model from the xml string and get all the joint names.
    urdf::Model robot_model; 
    if (!robot_model.initString( msg->urdf_xml_string)){
      cerr << "ERROR: Could not generate robot model" << endl;
    }

    // Parse KDL tree
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf_xml_string_,tree))
    {
      cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
      return;
    }

    fksolver_ = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));

    //unsubscribe from urdf messages
    lcm_->unsubscribe(urdf_subscription_); 
    urdf_parsed_ = true;
  }
}



  // Construct model view within opengl simulator:
// cd ~/projects/kmcl/models/table_models
// rgbd-sim-terminal-demo 1 ~/Desktop/drc_mesh_rgbd/utorso.obj 

//- for each object:
//-- determine the joint position with forward kinematics
//-- add to model

//-- render in rgbd_simulation ... done
//-- publish a mask with depth and binary ... done

// Sample Rendering Code - I think this worked for rendering colour!!!!!!!!!!!!!!!!!!!!!!!!!!!
/*
  // 1. Read the model files in:
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile(  "/home/mfallon/Desktop/drc_mesh_rgbd/l_hand.obj"    ,mesh);
  pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh(mesh));
  //state->model = mesh_ptr;  
  std::cout << "file has been read\n";

  // transform
  Eigen::Vector3f world_trans(1 , 0 , 0 ); 
  pcl::PointCloud<pcl::PointXYZRGB> mesh_cloud;  
  pcl::fromROSMsg(mesh_ptr->cloud, mesh_cloud);
  Eigen::Quaternionf world_rot (1 ,0,0,0);
  pcl::transformPointCloud (mesh_cloud, mesh_cloud,world_trans, world_rot);  
  pcl::toROSMsg (mesh_cloud, mesh_ptr->cloud);  


  TriangleMeshModel::Ptr model = TriangleMeshModel::Ptr(new TriangleMeshModel(mesh_ptr));
  scene_->add(model); 
  model_ptr = mesh_ptr;  

*/
void Pass::robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  if (!urdf_parsed_){      return;    }
  if(urdf_subscription_on_){
    cout << "robotStateHandler: unsubscribing from urdf" << endl;
    lcm_->unsubscribe(urdf_subscription_); //unsubscribe from urdf messages
    urdf_subscription_on_ =  false;   
  }  
  
  last_rstate_= *msg;  
  init_rstate_=true; // both urdf parsed and robot state handled... ready to handle data
}  
  

// Initially we will transmit a mask 1-for-1 with the image

void Pass::imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg){
  if (!init_rstate_){
    std::cout << "Either ROBOT_MODEL or EST_ROBOT_STATE has not been received, ignoring image\n";
    return;
  }
  
  #if DO_TIMING_PROFILE
    std::vector<int64_t> tic_toc;
    tic_toc.push_back(_timestamp_now());
  #endif
  
  // 0. Extract World to Body TF:  
  Eigen::Isometry3d world_to_body;
  world_to_body.setIdentity();
  world_to_body.translation()  << last_rstate_.origin_position.translation.x, last_rstate_.origin_position.translation.y, last_rstate_.origin_position.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(last_rstate_.origin_position.rotation.w, last_rstate_.origin_position.rotation.x, 
                                               last_rstate_.origin_position.rotation.y, last_rstate_.origin_position.rotation.z);
  world_to_body.rotate(quat);    

  
  // 1. Determine the Camera in World Frame:
  // 1a. Solve for Forward Kinematics
  // TODO: use gl_robot routine instead of replicating this here:
  map<string, double> jointpos_in;
  for (uint i=0; i< (uint) last_rstate_.num_joints; i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(last_rstate_.joint_name[i], last_rstate_.joint_position[i]));

  // Calculate forward position kinematics
  map<string, drc::transform_t > cartpos_out;
  bool flatten_tree=true;
  bool kinematics_status = fksolver_->JntToCart(jointpos_in,cartpos_out,flatten_tree);
  if(kinematics_status>=0){
    // cout << "Success!" <<endl;
  }else{
    cerr << "Error: could not calculate forward kinematics!" <<endl;
    return;
  }    
  
  // 1b. Determine World to Camera Pose:
  Eigen::Isometry3d world_to_camera;
  map<string, drc::transform_t>::const_iterator transform_it;
  transform_it=cartpos_out.find("left_camera_optical_frame");
  if(transform_it!=cartpos_out.end()){// fk cart pos exists
    Eigen::Isometry3d body_to_camera;
    body_to_camera.setIdentity();
    body_to_camera.translation()  << transform_it->second.translation.x, transform_it->second.translation.y, transform_it->second.translation.z;
    Eigen::Quaterniond quat = Eigen::Quaterniond( transform_it->second.rotation.w, transform_it->second.rotation.x,
                              transform_it->second.rotation.y, transform_it->second.rotation.z );
    body_to_camera.rotate(quat);    
    world_to_camera = world_to_body*body_to_camera;
  }
  
  // 2. Determine all Body-to-Link transforms for Visual elements:
  gl_robot_->set_state( last_rstate_);
  std::vector<drc::link_transform_t> link_tfs= gl_robot_->get_link_tfs();
  
  
  // Loop through joints and extract world positions:
  std::vector<Eigen::Isometry3d> link_tfs_e;
  int counter =msg->utime;  
  std::vector<Isometry3dTime> world_to_jointTs;
  std::vector< int64_t > world_to_joint_utimes;
  std::vector< std::string > link_names;
  for (size_t i=0; i < link_tfs.size() ; i++){
    Eigen::Isometry3d body_to_joint;
    body_to_joint.setIdentity();
    body_to_joint.translation() << link_tfs[i].tf.translation.x , link_tfs[i].tf.translation.y ,link_tfs[i].tf.translation.z; // 0.297173
    Eigen::Quaterniond m;
    m  = Eigen::Quaterniond(  link_tfs[i].tf.rotation.w , link_tfs[i].tf.rotation.x , link_tfs[i].tf.rotation.y , link_tfs[i].tf.rotation.z);
    body_to_joint.rotate(m);
    
    link_names.push_back( link_tfs[i].link_name  );
    link_tfs_e.push_back( body_to_joint );
    
    // For visualization only:
    world_to_joint_utimes.push_back( counter);
    Isometry3dTime body_to_jointT(counter, body_to_joint);
    world_to_jointTs.push_back(body_to_jointT);
    counter++;
  }
  pc_vis_->pose_collection_to_lcm_from_list(9998, world_to_jointTs); // all joints in world frame
  pc_vis_->text_collection_to_lcm(9997, 9998, "Pass - Frames [Labels]", link_names, world_to_joint_utimes );    

  if (verbose_){
    cout << "link_tfs size: " << link_tfs.size() <<"\n";
    for (size_t i=0; i < link_tfs.size() ; i ++){
      cout << i << ": " <<link_tfs[i].link_name << ": " 
        << link_tfs[i].tf.translation.x << " " << link_tfs[i].tf.translation.y << " " << link_tfs[i].tf.translation.z
        << " | " << link_tfs[i].tf.rotation.w << " " << link_tfs[i].tf.rotation.x << " " << link_tfs[i].tf.rotation.y << " " << link_tfs[i].tf.rotation.z << "\n";
    }
    for (size_t i=0; i < link_tfs_e.size() ; i ++){
      std::stringstream ss;
      print_Isometry3d(link_tfs_e[i], ss);
      cout << i << ": " << link_names[i] << ": " << ss.str() << "\n";  
    }    
  }
  
  
  Isometry3dTime world_to_cameraTorig = Isometry3dTime( msg->utime, world_to_camera);
  pc_vis_->pose_to_lcm_from_list(9995, world_to_cameraTorig);
  
  
  // Fix to rotate Camera into correct frame:
  // TODO: Fix this or determine the cause:
  Eigen::Isometry3d fixrotation_pose;
  fixrotation_pose.setIdentity();
  fixrotation_pose.translation() << 0,0,0;    
  Eigen::Quaterniond fix_r = euler_to_quat(0.0*M_PI/180.0, -90.0*M_PI/180.0 , 90.0*M_PI/180.0);
  fixrotation_pose.rotate(fix_r);    
  world_to_camera = world_to_camera*fixrotation_pose;
    
  std::stringstream ss;
  print_Isometry3d(world_to_camera, ss);
  cout << ss.str() << " head_pose\n";
    
  Isometry3dTime world_to_cameraT = Isometry3dTime( msg->utime, world_to_camera);
  pc_vis_->pose_to_lcm_from_list(9999, world_to_cameraT);
    
  // Pull the trigger and render scene:
  
  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
  #endif
  
  simexample->createScene("/home/mfallon/Desktop/drc_mesh_rgbd/", link_names, link_tfs_e);
  
  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
  #endif
  
  simexample->doSim(world_to_camera);
  
  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
  #endif
  
  sendOutput(msg->utime);
  
  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
    display_tic_toc(tic_toc,"imageHandler");
  #endif  
}

int 
main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  string camera_channel="CAMERALEFT";
  parser.add(camera_channel, "c", "camera_channel", "Camera channel");
  parser.parse();
  cout << camera_channel << " is camera_channel\n"; 
  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(argc,argv, lcm, camera_channel);
  cout << "image-passthrough ready" << endl << endl;
  while(0 == lcm->handle());
  return 0;
}
