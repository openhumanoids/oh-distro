// LCM example program for interacting  with PCL data
// In this case it pushes the data back out to LCM in
// a different message type for which the "collections"
// renderer can view it in the LCM viewer
// mfallon aug 2012
#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>

//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <lcmtypes/visualization.h>


#include "terrain-classify.hpp"

using namespace std;
using namespace boost;
using namespace boost::assign;





/////////////////////////////////////

terrain_classify::terrain_classify(boost::shared_ptr<lcm::LCM> &publish_lcm):
          lcm_(publish_lcm), _urdf_parsed(false),
          world_to_bodyT_(0, Eigen::Isometry3d::Identity()),
          null_poseT(0, Eigen::Isometry3d::Identity()) {

  l2f_list_ += "head","head_rightedhokuyo";
  // "head_hokuyo"
  //"left_camera_optical_frame","head_statichokuyo"

  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM());
  // obj: id name type reset
  //pc_vis_->obj_cfg_list.push_back( obj_cfg(6000,"Frames [Zero]",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Null",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(6001,"Pose",5,1) );
  // pts: id name type reset objcoll usergb rgb
  
  float colors_b[] ={0.0,0.0,1.0};
  vector <float> colors_v;
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Count Grid Map"     ,1,1, 1000,0, colors_v ));
  
  
  lcm_->subscribe("EST_ROBOT_STATE",&terrain_classify::robot_state_handler,this);  

  _urdf_subscription =   lcm_->subscribe("ROBOT_MODEL",&terrain_classify::urdf_handler,this); 
  
  lcmgl_= bot_lcmgl_init(lcm_->getUnderlyingLCM(), "terrain_classify");

}



Eigen::Quaterniond terrain_classify::euler_to_quat(double yaw, double pitch, double roll) {
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}


void terrain_classify::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  /*
  if (!_urdf_parsed){
    cout << "\n handleRobotStateMsg: Waiting for urdf to be parsed" << endl;
    return;
  }

  // This republishes the head_hokuyo_joint angle as a transform
  // it assumes the delta pose = [0,0,0,0,0,0] ... 
  // it is important this is true, if not change it here!
  for (size_t i=0; i< msg->num_joints; i++){
    if (   msg->joint_name[i].compare( "head_hokuyo_joint" ) == 0 ){
        bot_core::rigid_transform_t tf;
        tf.utime = msg->utime;
        tf.trans[0] =0;
        tf.trans[1] =0;
        tf.trans[2] =0;
        Eigen::Quaterniond head_quat = euler_to_quat(0,0,msg->joint_position[i]);
        tf.quat[0] =head_quat.w();
        tf.quat[1] =head_quat.x();
        tf.quat[2] =head_quat.y();
        tf.quat[3] =head_quat.z();
        lcm_->publish("HEAD_TO_HEAD_HOKUYO", &tf);      
      
    }
  }
  
  
  
  //clear stored data
  _link_tfs.clear();

  // call a routine that calculates the transforms the joint_state_t* msg.
  map<string, double> jointpos_in;
  for (uint i=0; i< (uint) msg->num_joints; i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(msg->joint_name[i], msg->joint_position[i]));

  map<string, drc::transform_t > cartpos_out;

  // Calculate forward position kinematics
  bool kinematics_status;
  bool flatten_tree=true; // determines the absolute transforms with respect to robot origin.
  //Otherwise returns relative transforms between joints.

  kinematics_status = _fksolver->JntToCart(jointpos_in,cartpos_out,flatten_tree);
  if(kinematics_status>=0){
    // cout << "Success!" <<endl;
  }else{
    cerr << "Error: could not calculate forward kinematics!" <<endl;
    return;
  }
  
  // 1. Extract World Pose:
  world_to_bodyT_.pose.setIdentity();
  world_to_bodyT_.pose.translation()  << msg->origin_position.translation.x, msg->origin_position.translation.y, msg->origin_position.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->origin_position.rotation.w, msg->origin_position.rotation.x, 
                                               msg->origin_position.rotation.y, msg->origin_position.rotation.z);
  world_to_bodyT_.pose.rotate(quat);    
  world_to_bodyT_.utime = msg->utime;
  
  // Loop through joints and extract world positions:
  int counter =msg->utime;  
  std::vector<Isometry3dTime> body_to_jointTs, world_to_jointsT;
  std::vector< int64_t > body_to_joint_utimes;
  std::vector< std::string > joint_names;
  for( map<string, drc::transform_t>::iterator ii=cartpos_out.begin(); ii!=cartpos_out.end(); ++ii){
    std::string joint = (*ii).first;
    //cout << joint  << ": \n";
    joint_names.push_back( joint  );
    body_to_joint_utimes.push_back( counter);
    
    Eigen::Isometry3d body_to_joint;
    body_to_joint.setIdentity();
    body_to_joint.translation()  << (*ii).second.translation.x, (*ii).second.translation.y, (*ii).second.translation.z;
    Eigen::Quaterniond quat = Eigen::Quaterniond((*ii).second.rotation.w, (*ii).second.rotation.x, (*ii).second.rotation.y, (*ii).second.rotation.z);
    body_to_joint.rotate(quat);    
    Isometry3dTime body_to_jointT(counter, body_to_joint);
    body_to_jointTs.push_back(body_to_jointT);
    // convert to world positions
    Isometry3dTime world_to_jointT(counter, world_to_bodyT_.pose*body_to_joint);
    world_to_jointsT.push_back(world_to_jointT);
    
    // Also see if these joints are required to be published as BOT_FRAMES poses:
    BOOST_FOREACH(string l2f, l2f_list_ ){
      if ( l2f.compare( joint ) == 0 ){
        //cout << "got : " << l2f << "\n";
        bot_core::rigid_transform_t tf;
        tf.utime = msg->utime;
        tf.trans[0] = body_to_joint.translation().x();
        tf.trans[1] = body_to_joint.translation().y();
        tf.trans[2] = body_to_joint.translation().z();
    
        tf.quat[0] =quat.w();
        tf.quat[1] =quat.x();
        tf.quat[2] =quat.y();
        tf.quat[3] =quat.z();
        std::string l2f_upper ="BODY_TO_" + boost::to_upper_copy(l2f);
        lcm_->publish(l2f_upper, &tf);      
      }
    }
    counter++;
  }
  
 
  
  
  
  
  //std::cout << body_to_jointTs.size() << " jts\n";
  //pc_vis_->pose_collection_to_lcm_from_list(6000, body_to_jointTs); // all joints releative to body - publish if necessary
  pc_vis_->pose_collection_to_lcm_from_list(6001, world_to_jointsT); // all joints in world frame
  pc_vis_->text_collection_to_lcm(6002, 6001, "Frames [Labels]", joint_names, body_to_joint_utimes );    
  */
}


void terrain_classify::urdf_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
    const  drc::robot_urdf_t* msg)
{
  /*
  // Received robot urdf string. Store it internally and get all available joints.
  _robot_name      = msg->robot_name;
  _urdf_xml_string = msg->urdf_xml_string;
  cout<< "\nReceived urdf_xml_string of robot [" 
      << msg->robot_name << "], storing it internally as a param" << endl;

  // Get a urdf Model from the xml string and get all the joint names.
  urdf::Model robot_model; 
  if (!robot_model.initString( msg->urdf_xml_string))
  {
    cerr << "ERROR: Could not generate robot model" << endl;
  }

  typedef map<string, shared_ptr<urdf::Joint> > joints_mapType;
  for( joints_mapType::const_iterator it = robot_model.joints_.begin(); it!=robot_model.joints_.end(); it++)
  {
    if(it->second->type!=6) // All joints that not of the type FIXED.
      _joint_names_.push_back(it->first);
  }

  _links_map =  robot_model.links_;

  //---------parse the tree and stop listening for urdf messages

  // Parse KDL tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
  {
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    return;
  }

  //unsubscribe from urdf messages
  lcm_->unsubscribe(_urdf_subscription); 

  //
  _fksolver = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
  //remember that we've parsed the urdf already
  _urdf_parsed = true;

  cout<< "Number of Joints: " << _joint_names_.size() <<endl;
  */
};// end handleRobotUrdfMsg

void terrain_classify::do_terrain_classify(){
  cout << "do clasf\n";
  int counter_=0;

  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  string fname = "cloud 05 0.246173 -3.56331 -0.57998 0.700534 0.00217818 -0.000454681 0.713616.pcd";
  pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *cloud);
  
    istringstream iss(fname);
    string sub0, sub1, sub2, sub3, sub4, sub5, sub6, sub7, sub8;
    iss >> sub0;    cout << "Substring0: " << sub0 << endl;
    iss >> sub1;    cout << "Substring1: " << sub1 << endl;
    iss >> sub2;    cout << "Substring2: " << sub2 << endl;
    iss >> sub3;    cout << "Substring3: " << sub3 << endl;
    iss >> sub4;    cout << "Substring4: " << sub4 << endl;
    iss >> sub5;    cout << "Substring5: " << sub5 << endl;
    iss >> sub6;    cout << "Substring6: " << sub6 << endl;
    iss >> sub7;    cout << "Substring7: " << sub7 << endl;
    iss >> sub8;    cout << "Substring8: " << sub8 << endl;
    

    Eigen::Isometry3d body_to_joint;
    body_to_joint.setIdentity();
    body_to_joint.translation()  << atof(sub2.c_str()) , atof(sub3.c_str()) , atof(sub4.c_str());
    Eigen::Quaterniond quat = Eigen::Quaterniond(    
    atof(sub5.c_str()) , atof(sub6.c_str()) , atof(sub7.c_str()) , atof(sub8.c_str())    );
    body_to_joint.rotate(quat);    
    Isometry3dTime body_to_jointT(counter_, body_to_joint);
  
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;  
            

  pc_vis_->pose_to_lcm_from_list(6001, body_to_jointT); // all joints in world frame
            
            
            
  
//  bot_lcmgl_color3f(lcmgl_, pc_vis_->colors[counter_*3], pc_vis_->colors[counter_*3+1], pc_vis_->colors[counter_*3+2]);
  bot_lcmgl_color3f(lcmgl_, 0,0,1);  
  bot_lcmgl_begin(lcmgl_, LCMGL_POINTS);
  for (int i = 0; i < cloud->points.size(); ++i) {
    pcl::PointXYZRGB point = cloud->points[i];
    bot_lcmgl_vertex3f(lcmgl_, point.x, point.y, point.z);
  }
  bot_lcmgl_end(lcmgl_);
  
  std::cout << "Published " << cloud->points.size() << " points " << std::endl;     
  
  
  
  int xmin= -20;
  int ymin = -5;
  double res = 2.0;

  // Assign a 10x10 array
  // loop through the points and 
  
  double r_sum[10*10];
  double r_sum_sq[10*10];
  vector <int > r_count;
  for (int i=0;i<100;i++){
   r_count.push_back(0); 
  }

  bot_lcmgl_color3f(lcmgl_, 1,0,0);  
  bot_lcmgl_begin(lcmgl_, LCMGL_POINTS);
  for (int i = 0; i < cloud->points.size(); ++i) {
    pcl::PointXYZRGB point = cloud->points[i];
    
    int x = (int) round (point.x * res);
    int y = (int) round (point.y * res);
    bot_lcmgl_vertex3f(lcmgl_, ((double) x)/res, ((double)y)/res, point.z);
    
    if (x >=0 && x<10){ //0..9 = 10
      if (y >=0 && y<10){ // 0..9 = 10 .... 100 elements
         r_sum[x*10 + y] += point.z;
         r_sum_sq[x*10 + y] += pow(point.z,2);
         r_count[x*10 + y]++;
      }
    }    
  }
  bot_lcmgl_end(lcmgl_);          
  bot_lcmgl_switch_buffer(lcmgl_);
  
  
  
  std::vector<int>::iterator result = std::max_element(r_count.begin(), r_count.end());
  int max_element = std::distance(r_count.begin(), result); 
  double max_likelihood = r_count[max_element];
  result = std::min_element(r_count.begin(), r_count.end());
  int min_element = std::distance(r_count.begin(), result); 
  double min_likelihood = r_count[min_element];
  
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr grid_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  for (int x=0; x<10;x++){
    for (int y=0; y<10;y++){
      pcl::PointXYZRGB point ;
      point.x = (double)x;
      point.y = (double)y;
      point.z = 0;//(double)r_count[x*10 + y];
      
      float rgb_vector_float[3];
      float jet_rgb_in  = (r_count[ x*10 +y] - min_likelihood)/(max_likelihood - min_likelihood);
      jet_rgb( jet_rgb_in,rgb_vector_float);
      point.r = rgb_vector_float[0]*255.0;
      point.g = rgb_vector_float[1]*255.0;
      point.b = rgb_vector_float[2]*255.0;
      
      grid_cloud->points.push_back(point);
    }
  }  
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT); // remove this?
  pc_vis_->ptcld_to_lcm_from_list(1001, *grid_cloud, null_poseT.utime, null_poseT.utime);
            
}

int
main(int argc, char ** argv)
{
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;  
  
  terrain_classify app(lcm);
  
  app.do_terrain_classify();
  
//  while(0 == lcm->handle());
  return 0;
}

