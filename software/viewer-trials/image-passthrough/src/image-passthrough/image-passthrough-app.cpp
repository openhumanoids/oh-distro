// Todo:
// - support rendering primitives for the fingers
// - why does TriangleModelMesh not work????
// - use model_client to parse the urdf in advance of the lcm_handle
//
// Current Mask Values:
// 0 - free (black - by default)
// 64-??? affordances
// 128-??? joints

// Also, I've removed renderering of the head.

// TIMING INFORMATIONG
// TODO: - Improve mergePolygonMesh() code avoiding mutliple conversions and keeping the polygon list
//       - Improve scene.draw() by giving it directly what it need - to avoid conversion.
//       - One of these can vastly increase the rendering speed:
// BENCHMARK as of jan 2013:
// - with convex hull models: (about 20Hz, sending mesh as GL_POLYGONS)
//   component solve fk  , createScene  render  , sendOutput
//   fraction: 0.00564941, 0.0688681  , 0.185647, 0.739836,
//   time sec: 0.00031   , 0.003779   , 0.010187, 0.040597,
// - with original models: (including complex head model) (about 5Hz)
//   component solve fk  , createScene  render  , sendOutput
//   fraction: 0.0086486 , 0.799919   , 0.072785, 0.118647,
//   time sec: 0.001635  , 0.151223   , 0.01376 , 0.02243,
// - These numbers are for RGB. Outputing Gray reduces sendOutput by half
//   so sending convex works at about 35Hz


// BENCHMARK as of feb 2013: (1024x544pixels, Greyscale)
// - with convex hull models: (excluding complex head model) (about 33Hz, rendering mesh as GL_TRIANGLES, also adding 5 affordances)
//   component solve fk  , createScene  render  , sendOutput
//   fraction: 0.0154348, 0.116463, 0.324849, 0.543253,
//   time sec: 0.000473, 0.003569, 0.009955, 0.016648,
// - with original models: (excluding complex head model) (about 20Hz, rendering mesh as GL_TRIANGLES, also adding 5 affordances)
//   component solve fk, createScene, render, sendOutput
//   fraction: 0.0103196 0.331993 0.230543 0.427145,
//   time sec: 0.000526 0.016922 0.011751 0.021772,   ... creating Scene is much quicker
//
// BENCHMARK as of feb 2013: (1024x544pixels, Greyscale and ZLIB compression)
// - with convex hull models: (about 50Hz)
// - with original models: (excluding complex head model) (about 40Hz, rendering mesh as GL_TRIANGLES, also adding 5 affordances)
//   component solve fk, createScene, render, sendOutput
//   fraction: 0, 0.0146542, 0.459086, 0.384564, 0.141696,
//   time sec: 0, 0.000375, 0.011748, 0.009841, 0.003626, ... creating Scene is much quicker AND transmission as zip images is quick
/*
#include <iostream>
#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>
#include <lcm/lcm-cpp.hpp>

#include "image-passthrough.hpp"

#include <visualization_utils/GlKinematicBody.hpp>
#include <visualization_utils/GlKinematicBody.hpp>

#include <pronto_utils/pronto_vis.hpp> // visualize pt clds
#include <rgbd_simulation/rgbd_primitives.hpp> // to create basic meshes
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

#include <path_util/path_util.h>
#include <affordance/AffordanceUtils.hpp>




#define DO_TIMING_PROFILE FALSE
#define PCL_VERBOSITY_LEVEL L_ERROR
// offset of affordance in mask labelling:
#define AFFORDANCE_OFFSET 64
*/
#include "image-passthrough-app.hpp"

using namespace std;
using namespace drc;
using namespace Eigen;
using namespace boost;
using namespace boost::assign; // bring 'operator+()' into scope

#define LINK_OFFSET 128


Pass::Pass(int argc, char** argv, boost::shared_ptr<lcm::LCM> &lcm_,
           std::string camera_channel_, int output_color_mode_,
           bool use_convex_hulls_, string camera_frame_,
           CameraParams camera_params_, bool verbose_):
           lcm_(lcm_), output_color_mode_(output_color_mode_),
           use_convex_hulls_(use_convex_hulls_),
           init_rstate_(false), camera_channel_(camera_channel_),
           camera_frame_(camera_frame_), camera_params_(camera_params_),
           update_robot_state_(true), renderer_robot_(true),
           verbose_(verbose_){

  // Construct the simulation method:
  std::string path_to_shaders = string(getBasePath()) + "/bin/";
  simexample = SimExample::Ptr (new SimExample (argc, argv,
                                                camera_params_.height, camera_params_.width ,
                                                lcm_, output_color_mode_, path_to_shaders));
  simexample->setCameraIntrinsicsParameters (camera_params_.width, camera_params_.height,
                                             camera_params_.fx, camera_params_.fy,
                                             camera_params_.cx, camera_params_.cy);

  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  urdf_xml_string_ = model_->getURDFString();
  prepareModel();

  // LCM subscriptions:
  lcm_->subscribe("EST_ROBOT_STATE",&Pass::robotStateHandler,this);
  lcm_->subscribe("AFFORDANCE_PLUS_COLLECTION",&Pass::affordancePlusHandler,this);

  // Visual I-O:
  float colors_b[] ={0.0,0.0,0.0};
  std::vector<float> colors_v;
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(9999,"iPass - Pose - Left",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(9998,"iPass - Frames",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(9995,"iPass - Pose - Left Original",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(9996,"iPass - Sim Cloud"     ,1,1, 9995,0, colors_v ));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(9994,"iPass - Null",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(9993,"iPass - World "     ,7,1, 9994,0, colors_v ));
  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), 
                                  camera_params_.width, 
                                  camera_params_.height ); // Actually outputs the Mask PCLImage:


  // Keep a mesh for the affordances:
  pcl::PolygonMesh::Ptr combined_aff_mesh_ptr_temp(new pcl::PolygonMesh());
  combined_aff_mesh_ = combined_aff_mesh_ptr_temp;
  aff_mesh_filled_=false;
}

// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}



pcl::PolygonMesh::Ptr getPolygonMesh(std::string filename){
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile(  filename    ,mesh);
  pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh(mesh));
  //state->model = mesh_ptr;
  return mesh_ptr;
}

Eigen::Isometry3d URDFPoseToEigen(urdf::Pose& pose_in){
  Eigen::Isometry3d pose_out = Eigen::Isometry3d::Identity();

  pose_out.translation()  << pose_in.position.x,
                          pose_in.position.y,
                          pose_in.position.z;
  pose_out.rotate( Eigen::Quaterniond(pose_in.rotation.w,
                                    pose_in.rotation.x,
                                    pose_in.rotation.y,
                                    pose_in.rotation.z) );
  return pose_out;
}

KDL::Frame EigenToKDL(Eigen::Isometry3d tf){
  KDL::Frame tf_out;
  tf_out.p[0] = tf.translation().x();
  tf_out.p[1] = tf.translation().y();
  tf_out.p[2] = tf.translation().z();
  Eigen::Quaterniond  r( tf.rotation() );
  tf_out.M =  KDL::Rotation::Quaternion( r.x(), r.y(), r.z(), r.w() );
  return tf_out;
}

//////////////////////////////////////////////////////////////////////
void Pass::prepareModel(){
  cout<< "URDF handler"<< endl;
  // Received robot urdf string. Store it internally and get all available joints.
  gl_robot_ = boost::shared_ptr<visualization_utils::GlKinematicBody>(new visualization_utils::GlKinematicBody(urdf_xml_string_));
  cout<< "Number of Joints: " << gl_robot_->get_num_joints() <<endl;
  links_map_ = gl_robot_->get_links_map();
  cout<< "Size of Links Map: " << links_map_.size() <<endl;
  
  typedef map<string, boost::shared_ptr<urdf::Link> > links_mapType;
  
  int i =0;
  for(links_mapType::const_iterator it =  links_map_.begin(); it!= links_map_.end(); it++){
    cout << it->first << endl;
    if(it->second->visual){
      std::cout << it->first<< " link" << it->second->visual->geometry->type << " type\n"; // visual type

      Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
      pcl::PolygonMesh::Ptr mesh_ptr(new pcl::PolygonMesh());

      // For each visual element within the link geometry:
      typedef map<string, boost::shared_ptr<vector<boost::shared_ptr<urdf::Visual> > > >  visual_groups_mapType;
      visual_groups_mapType::iterator v_grp_it = it->second->visual_groups.find("default");
      for (size_t iv = 0;iv < v_grp_it->second->size();iv++){  // 
        vector<boost::shared_ptr<urdf::Visual> > visuals = (*v_grp_it->second);
        boost::shared_ptr<urdf::Geometry> geom =  visuals[iv]->geometry;
        Eigen::Isometry3d visual_origin = URDFPoseToEigen( visuals[iv]->origin );

        if  (geom->type == urdf::Geometry::MESH){
          boost::shared_ptr<urdf::Mesh> mesh(dynamic_pointer_cast<urdf::Mesh>( geom ));
          // TODO: Verify the existance of the file:
          std::string file_path = gl_robot_->evalMeshFilePath(mesh->filename, use_convex_hulls_);

          // Read the Mesh, transform by the visual component origin:
          pcl::PolygonMesh::Ptr this_mesh = getPolygonMesh(file_path);
          pcl::PointCloud<pcl::PointXYZRGB> mesh_cloud_1st;  
          pcl::fromPCLPointCloud2(this_mesh->cloud, mesh_cloud_1st);
          Eigen::Isometry3f visual_origin_f= visual_origin.cast<float>();
          Eigen::Quaternionf visual_origin_quat(visual_origin_f.rotation());
          pcl::transformPointCloud (mesh_cloud_1st, mesh_cloud_1st, visual_origin_f.translation(), visual_origin_quat);  
          pcl::toPCLPointCloud2 (mesh_cloud_1st, this_mesh->cloud);  
          simexample->mergePolygonMesh(mesh_ptr, this_mesh );

        }else if(geom->type == urdf::Geometry::BOX){
          boost::shared_ptr<urdf::Box> box(dynamic_pointer_cast<urdf::Box>( geom ));
          simexample->mergePolygonMesh(mesh_ptr, 
                                      prim_->getCubeWithTransform(visual_origin, box->dim.x, box->dim.y, box->dim.z) );
        }else if(geom->type == urdf::Geometry::CYLINDER){
          boost::shared_ptr<urdf::Cylinder> cyl(dynamic_pointer_cast<urdf::Cylinder>( geom ));
          simexample->mergePolygonMesh(mesh_ptr, 
                                      prim_->getCylinderWithTransform(visual_origin, cyl->radius, cyl->radius, cyl->length) );
        }else if(geom->type == urdf::Geometry::SPHERE){
          boost::shared_ptr<urdf::Sphere> sphere(dynamic_pointer_cast<urdf::Sphere>(geom)); 
          simexample->mergePolygonMesh(mesh_ptr, 
                                      prim_->getSphereWithTransform(visual_origin, sphere->radius) );
        }else{
          std::cout << "Pass::urdfHandler() Unrecognised urdf object\n";
          exit(-1);
        }
      }

      // Apply Some Coloring:
      if(output_color_mode_==0){ // Set the mesh to a false color:
        int j =i%(simexample->colors_.size()/3);
        simexample->setPolygonMeshColor(mesh_ptr, simexample->colors_[j*3], simexample->colors_[j*3+1], simexample->colors_[j*3+2] );
      }else if(output_color_mode_==1){ // set link mesh in link range
        simexample->setPolygonMeshColor(mesh_ptr, LINK_OFFSET + (int) i, 0, 0 );
      }else{ // black or white
        simexample->setPolygonMeshColor(mesh_ptr, 255,0,0 ); // last two are not used
      }


      PolygonMeshStruct mesh_struct;
      mesh_struct.origin = origin;
      mesh_struct.link_name = it->first ;
      mesh_struct.polygon_mesh = mesh_ptr;
      simexample->polymesh_map_.insert(make_pair( it->first , mesh_struct));

      i++;
    }
  }

  //////////////////////////////////////////////////////////////////
  // Get a urdf Model from the xml string and get all the joint names.
  urdf::Model robot_model;
  if (!robot_model.initString( urdf_xml_string_)){
    cerr << "ERROR: Could not generate robot model" << endl;
  }
  // Parse KDL tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_xml_string_,tree)){
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    return;
  }
  fksolver_ = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
}


void Pass::affordancePlusInterpret(drc::affordance_plus_t affplus, int aff_uid, pcl::PolygonMesh::Ptr &mesh_out){
    std::map<string,double> am;
    for (size_t j=0; j< affplus.aff.nparams; j++){
      am[ affplus.aff.param_names[j] ] = affplus.aff.params[j];
    }

    //Eigen::Isometry3d transform = affutils.getPose(affplus.aff.param_names, affplus.aff.params);
    Eigen::Isometry3d transform = affutils.getPose(affplus.aff.origin_xyz, affplus.aff.origin_rpy );

    string otdf_type = affplus.aff.otdf_type;

    if (otdf_type == "box"){
      //cout  << aff_uid << " is a box\n";
      mesh_out = prim_->getCubeWithTransform(transform,am.find("lX")->second, am.find("lY")->second, am.find("lZ")->second);
    }else if(otdf_type == "cylinder"){
      //cout  << aff_uid << " is a cylinder\n";
      mesh_out = prim_->getCylinderWithTransform(transform, am.find("radius")->second, am.find("radius")->second, am.find("length")->second );
    }else if(otdf_type == "steering_cyl"){
      //cout  << aff_uid << " is a steering_cyl\n";
      mesh_out = prim_->getCylinderWithTransform(transform, am.find("radius")->second, am.find("radius")->second, am.find("length")->second );
    }else if(otdf_type == "dynamic_mesh"){
      //cout  << aff_uid << " is a dynamic_mesh ["<< affplus.points.size() << " pts and " << affplus.triangles.size() << " tri]\n";
      mesh_out = affutils.getMeshFromAffordance(affplus.points, affplus.triangles,transform);
    }else if(otdf_type == "dynamic_mesh_w_1_cylinder"){ // Ignores the cylinder and just draws the mesh
      //cout  << aff_uid << " is a dynamic_mesh_w_1_cylinder ["<< affplus.points.size() << " pts and " << affplus.triangles.size() << " tri]\n";
      mesh_out = affutils.getMeshFromAffordance(affplus.points, affplus.triangles,transform);
    }else if(otdf_type == "plane"){
      //cout  << aff_uid << " is a plane ["<< affplus.points.size() << " pts and " << affplus.triangles.size() << " tri]\n";
      mesh_out = affutils.getMeshFromAffordance(affplus.points, affplus.triangles,transform);
    }else if(otdf_type == "firehose"){
      // the simple two cylinder model maurice used in dec 2013
      // NB: I don't support otdf - so this is hard coded here for now
      // cout  << aff_uid << " is a firehose\n";
      mesh_out = prim_->getCylinderWithTransform(transform, 0.0266, 0.0266, 0.042 );
      Eigen::Isometry3d trans_2nd = Eigen::Isometry3d::Identity();
      trans_2nd.translation()  << 0,0, 0.033;
      trans_2nd = transform * trans_2nd;
      simexample->mergePolygonMesh(mesh_out, prim_->getCylinderWithTransform(trans_2nd, 0.031, 0.031, 0.024 ) );
    }else if(otdf_type == "dewalt_button"){
      //cout  << aff_uid << " is a wye_mesh\n";
      std::string fname = string(getenv( "DRC_BASE" )) + string( "/software/models/mit_gazebo_models/otdf/dewalt_button.obj");
      mesh_out = getPolygonMesh(fname);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

      // If the mesh is only XYZ, then manually copy into XYZRGB
      if (mesh_out->cloud.fields.size() == 3){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::fromPCLPointCloud2(mesh_out->cloud, *cloud_xyz);
        cloud->points.resize(cloud_xyz->points.size());
        for (size_t i = 0; i < cloud_xyz->points.size(); i++) {
          cloud->points[i].x = cloud_xyz->points[i].x;
          cloud->points[i].y = cloud_xyz->points[i].y;
          cloud->points[i].z = cloud_xyz->points[i].z;
        }
      }else{
        pcl::fromPCLPointCloud2(mesh_out->cloud, *cloud);
      }

      // Apply transform to polymesh:
      Eigen::Isometry3f pose_f = transform.cast<float>();
      Eigen::Quaternionf quat_f(pose_f.rotation());
      pcl::transformPointCloud (*cloud, *cloud,
      pose_f.translation(), quat_f); // !! modifies cloud
      pcl::toPCLPointCloud2(*cloud, mesh_out->cloud);
    }else if(otdf_type == "wye_mesh"){
      //cout  << aff_uid << " is a wye_mesh\n";
      std::string fname = string(getenv( "DRC_BASE" )) + string( "/software/models/mit_gazebo_models/otdf/wye.obj");
      mesh_out = getPolygonMesh(fname);
      std::cout << "more stuff\n";

      // Apply transform to polymesh:
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pcl::fromPCLPointCloud2(mesh_out->cloud, *cloud);  
      Eigen::Isometry3f pose_f = transform.cast<float>();
      Eigen::Quaternionf quat_f(pose_f.rotation());
      pcl::transformPointCloud (*cloud, *cloud,
      pose_f.translation(), quat_f); // !! modifies cloud
      pcl::toPCLPointCloud2(*cloud, mesh_out->cloud);       
      
    }else{
      cout  << aff_uid << " is a not recognised ["<< otdf_type <<"] not supported yet\n";
    }

    // demo of bounding boxes: (using radius as x and y dimensions
    //combined_mesh_ptr_temp = prim_->getCubeWithTransform(transform,am.find("radius")->second, am.find("radius")->second, am.find("length")->second);

    if(output_color_mode_==0){
      // Set the mesh to a false color:
      int j =aff_uid%(simexample->colors_.size()/3);
      simexample->setPolygonMeshColor(mesh_out, simexample->colors_[j*3], simexample->colors_[j*3+1], simexample->colors_[j*3+2] );
    }else if(output_color_mode_==1){
      simexample->setPolygonMeshColor(mesh_out,
                                      AFFORDANCE_OFFSET + aff_uid,0,0 ); // using red field as gray, last digits ignored
    }else{
      simexample->setPolygonMeshColor(mesh_out, 255,0,0 ); // white (last digits ignored
    }
}




// Receive affordances and creates a combined mesh in world frame
// TODO: properly parse the affordances
void Pass::affordancePlusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::affordance_plus_collection_t* msg){

  if (msg->naffs ==0){
    cout << "got "<< msg->naffs <<" affs [won't add anything]\n";
    aff_mesh_filled_=false;
    return;
  }
  cout << "got "<< msg->naffs <<" affs @ "<< msg->utime <<"\n";

  pcl::PolygonMesh::Ptr combined_aff_mesh_temp(new pcl::PolygonMesh());
  combined_aff_mesh_ = combined_aff_mesh_temp;
  for (size_t i=0; i < msg->affs_plus.size(); i++){
    pcl::PolygonMesh::Ptr new_aff_mesh(new pcl::PolygonMesh());
    affordancePlusInterpret(msg->affs_plus[i], msg->affs_plus[i].aff.uid, new_aff_mesh );
    simexample->mergePolygonMesh(combined_aff_mesh_, new_aff_mesh);
  }
  aff_mesh_filled_=true;
}


void Pass::robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  if (!update_robot_state_){
    // TODO: unsubscribe and resubscribe is better than this...
    std::cout << "skipping robot state update\n";
    return;
  }

  // Extract World to Body TF:
  world_to_body_.setIdentity();
  world_to_body_.translation()  << msg->pose.translation.x, msg->pose.translation.y, msg->pose.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->pose.rotation.w, msg->pose.rotation.x,
                                               msg->pose.rotation.y, msg->pose.rotation.z);
  world_to_body_.rotate(quat);

  jointpos_.clear();
  for (int i=0; i< msg->num_joints; i++) //cast to uint to suppress compiler warning
    jointpos_.insert(make_pair(msg->joint_name[i], msg->joint_position[i]));

  init_rstate_=true; // first robot state handled... ready to handle data
}


Eigen::Isometry3d KDLToEigen(KDL::Frame tf){
  Eigen::Isometry3d tf_out;
  tf_out.setIdentity();
  tf_out.translation()  << tf.p[0], tf.p[1], tf.p[2];
  Eigen::Quaterniond q;
  tf.M.GetQuaternion( q.x() , q.y(), q.z(), q.w());
  tf_out.rotate(q);
  return tf_out;
}

bool Pass::createMask(int64_t msg_time){
  if (!init_rstate_){
    std::cout << "Either ROBOT_MODEL or EST_ROBOT_STATE has not been received, ignoring image\n";
    return false;
  }

  #if DO_TIMING_PROFILE
    std::vector<int64_t> tic_toc;
    tic_toc.push_back(_timestamp_now());
  #endif


  // 1. Determine the Camera in World Frame:
  // 1a. Solve for Forward Kinematics
  // TODO: use gl_robot routine instead of replicating this here:
  // map<string, drc::transform_t > cartpos_out;
  map<string, KDL::Frame > cartpos_out;
  bool flatten_tree=true;
  bool kinematics_status = fksolver_->JntToCart(jointpos_,cartpos_out,flatten_tree);
  if(kinematics_status>=0){
    // cout << "Success!" <<endl;
  }else{
    cerr << "Error: could not calculate forward kinematics!" <<endl;
    return false;
  }

  // 1b. Determine World to Camera Pose:
  Eigen::Isometry3d world_to_camera;
  map<string, KDL::Frame>::const_iterator transform_it;
  transform_it=cartpos_out.find(camera_frame_);// usually "left_camera_optical_frame"
  if(transform_it!=cartpos_out.end()){// fk cart pos exists
    Eigen::Isometry3d body_to_camera = KDLToEigen(transform_it->second);
    //body_to_camera.setIdentity();
    //body_to_camera.translation()  << transform_it->second.translation.x, transform_it->second.translation.y, transform_it->second.translation.z;
    //Eigen::Quaterniond quat = Eigen::Quaterniond( transform_it->second.rotation.w, transform_it->second.rotation.x,
    //                          transform_it->second.rotation.y, transform_it->second.rotation.z );
    //body_to_camera.rotate(quat);
    world_to_camera = world_to_body_*body_to_camera;
  }

  // 2. Determine all Body-to-Link transforms for Visual elements:
  gl_robot_->set_state( EigenToKDL(world_to_body_), jointpos_);
  std::vector<visualization_utils::LinkFrameStruct> link_tfs= gl_robot_->get_link_tfs();


  // Loop through joints and extract world positions:
  std::vector<Eigen::Isometry3d> link_tfs_e;
  int counter =msg_time;
  std::vector<Isometry3dTime> world_to_jointTs;
  std::vector< int64_t > world_to_joint_utimes;
  std::vector< std::string > link_names;
  for (size_t i=0; i < link_tfs.size() ; i++){
    Eigen::Isometry3d body_to_joint;
    body_to_joint.setIdentity();
    body_to_joint.translation() << link_tfs[i].frame.p[0] , link_tfs[i].frame.p[1] ,link_tfs[i].frame.p[2]; // 0.297173
    double x,y,z,w;
    link_tfs[i].frame.M.GetQuaternion(x,y,z,w);
    Eigen::Quaterniond m;
    m  = Eigen::Quaterniond(w,x,y,z);
    body_to_joint.rotate(m);

    link_names.push_back( link_tfs[i].name  );
    link_tfs_e.push_back( body_to_joint);

    // For visualization only:
    world_to_joint_utimes.push_back( counter);
    Isometry3dTime body_to_jointT(counter, body_to_joint);
    world_to_jointTs.push_back(body_to_jointT);
    counter++;
  }

  if(verbose_){
    pc_vis_->pose_collection_to_lcm_from_list(9998, world_to_jointTs); // all joints in world frame
    pc_vis_->text_collection_to_lcm(9997, 9998, "IPass - Frames [Labels]", link_names, world_to_joint_utimes );

    cout << "link_tfs size: " << link_tfs.size() <<"\n";
    for (size_t i=0; i < link_tfs.size() ; i ++){
      double x,y,z,w;
      link_tfs[i].frame.M.GetQuaternion(x,y,z,w);
      cout << i << ": " <<link_tfs[i].name << ": "
        << link_tfs[i].frame.p[0]<< " " << link_tfs[i].frame.p[1] << " " << link_tfs[i].frame.p[2]
        << " | " << w << " " << x << " " << y << " " << z << "\n";
    }
    for (size_t i=0; i < link_tfs_e.size() ; i ++){
      std::stringstream ss;
      print_Isometry3d(link_tfs_e[i], ss);
      cout << i << ": " << link_names[i] << ": " << ss.str() << "\n";
    }
  }

  if (verbose_){
    Isometry3dTime world_to_cameraTorig = Isometry3dTime( msg_time, world_to_camera);
    pc_vis_->pose_to_lcm_from_list(9995, world_to_cameraTorig);
  }


  // NBNB I supply camera poses with x forward.
  // This rotates typical camera (z forward) to the camera into correct frame:
  // TODO: Fix this or determine the cause:
  Eigen::Isometry3d fixrotation_pose;
  fixrotation_pose.setIdentity();
  fixrotation_pose.translation() << 0,0,0;
  Eigen::Quaterniond fix_r = euler_to_quat(90.0*M_PI/180.0, -90.0*M_PI/180.0, 0.0*M_PI/180.0 );
  fixrotation_pose.rotate(fix_r);
  world_to_camera = world_to_camera*fixrotation_pose;

  if (verbose_){
    std::stringstream ss;
    print_Isometry3d(world_to_camera, ss);
    cout << ss.str() << " head_pose\n";

    Isometry3dTime world_to_cameraT = Isometry3dTime(msg_time, world_to_camera);
    pc_vis_->pose_to_lcm_from_list(9999, world_to_cameraT);
  }

  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
  #endif

  simexample->resetScene();
  if(renderer_robot_){ // Pull the trigger and render scene:
    simexample->createScene(link_names, link_tfs_e);
  }
  if(aff_mesh_filled_){
    simexample->mergePolygonMeshToCombinedMesh( combined_aff_mesh_);
  }
  if (verbose_){ // Visualize the entire world thats about to be renderered: NBNB THIS IS REALLY USEFUL FOR DEBUGGING
    Eigen::Isometry3d null_pose;
    null_pose.setIdentity();
    Isometry3dTime null_poseT = Isometry3dTime(msg_time, null_pose);
    pc_vis_->pose_to_lcm_from_list(9994, null_poseT);
    pc_vis_->mesh_to_lcm_from_list(9993, simexample->getCombinedMesh() , msg_time , msg_time);
  }
  simexample->addScene();

  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
  #endif

  simexample->doSim(world_to_camera);

  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
    display_tic_toc(tic_toc,"createMask");
  #endif

  return true;
}


// Output the simulated output to file/lcm:
void Pass::sendOutput(int64_t utime){
  bool do_timing=false;
  std::vector<int64_t> tic_toc;
  if (do_timing){
    tic_toc.push_back(_timestamp_now());
  }

  //imgutils_->sendImage( simexample->getDepthBufferAsColor(), utime, camera_params_.width, camera_params_.height, 3, string( camera_channel_ +  "_DEPTH") );
  if (output_color_mode_==0){
    // Zipping assumes gray for now - so don't zup for color (which will not be primarily be used:
    imgutils_->sendImage( simexample->getColorBuffer(3), utime,
                          camera_params_.width, camera_params_.height, 3,
                          string( camera_channel_ +  "_MASK") );
  }else{
    //imgutils_->sendImage( simexample->getColorBuffer(1), utime, camera_params_.width, camera_params_.height, 1, string( camera_channel_ +  "_MASK")  );
    imgutils_->sendImageZipped( simexample->getColorBuffer(1), utime,
                                camera_params_.width, camera_params_.height, 1,
                                string( camera_channel_ +  "_MASKZIPPED")  );
  }

  if (do_timing==1){
    tic_toc.push_back(_timestamp_now());
    display_tic_toc(tic_toc,"sendOutput");
  }
}

// Blend the simulated output with the input image:
void Pass::sendOutputOverlay(int64_t utime, uint8_t* img_buf){

  float blend = 0.5;
  if (1==1){
    uint8_t* mask_buf = simexample->getColorBuffer(1);
    for (size_t i=0; i < camera_params_.width * camera_params_.height; i++){
      img_buf[i*3] = int( blend*img_buf[i*3] + (1.0-blend)*mask_buf[i] );
      img_buf[i*3+1] =int( blend*img_buf[i*3+1] + (1.0-blend)*mask_buf[i] );
      img_buf[i*3+2] =int( blend*img_buf[i*3+2] + (1.0-blend)*mask_buf[i] );
    }
  }else{



    uint8_t* mask_buf = simexample->getColorBuffer(3);
    for (size_t i=0; i < camera_params_.width * camera_params_.height; i++){
      img_buf[i*3]   = int( blend*img_buf[i*3]   + (1.0-blend)*mask_buf[i*3]   );
      img_buf[i*3+1] = int( blend*img_buf[i*3+1] + (1.0-blend)*mask_buf[i*3+1] );
      img_buf[i*3+2] = int( blend*img_buf[i*3+2] + (1.0-blend)*mask_buf[i*3+2] );
    }
  }


  // Zipping assumes gray for now - so don't zup for color (which will not be primarily be used:
  imgutils_->sendImage( img_buf, utime,
                          camera_params_.width, camera_params_.height, 3,
                          string( camera_channel_ +  "_OVERLAY") );



}
