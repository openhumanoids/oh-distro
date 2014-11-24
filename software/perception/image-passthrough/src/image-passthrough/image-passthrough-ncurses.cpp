//
// drc-image-passthrough-ncurses -o 0 -m -v
#include <sys/types.h>
#include <dirent.h>


#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <cstdio> 
#include <iostream>
#include <vector>
#include <glib.h>
#include <glib-object.h>
#include <ncurses.h>
#include <wchar.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <map>


#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/drc/robot_state_t.hpp"
#include <bot_core/bot_core.h>

#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
// #include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

#include <image-passthrough/image-passthrough-app.hpp>

// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>

#include <ConciseArgs>

#define COLOR_PLAIN 1
#define COLOR_TITLE 2
#define COLOR_ERROR 3
#define COLOR_WARN  4

using namespace std;


struct CommandlineConfig{
  std::string  camera_channel;
  int output_color_mode; // 0 =rgb, 1=grayscale mask, 2=binary black/white grayscale mask
  bool use_convex_hulls;
  string camera_frame;
  bool verbose;
  bool use_mono;
  
  CommandlineConfig () {
    camera_channel = "CAMERA_LEFT";
    output_color_mode = 1;
    use_convex_hulls = false;
    camera_frame = "left_camera_optical_frame";
    verbose = false;
    use_mono = false;
  }
};

class App{
  public:
    App(int argc, char** argv, boost::shared_ptr<lcm::LCM> &lcm_, CommandlineConfig& cfg);
    
    ~App(){
    }
    
    void publishCorrection();
    void resetCorrection(){
      trans_.assign(3,0);
      rpy_.assign(3,0);
      publishCorrection();
    }

    bool on_input();
    bool on_timer();
    int repaint (int64_t now);
    
    boost::shared_ptr<lcm::LCM> lcm_;
    guint timer_id;
    WINDOW *w;
    GMainLoop * mainloop;
    
  private:
    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::robot_state_t* msg);    
    
    void solveFK(drc::robot_state_t state);
    
    vector<double> trans_;
    vector<double> rpy_;
    
    int last_input_;

    Eigen::Isometry3d world_to_body_;
    map<string, double> jointpos_;
    
    boost::shared_ptr<ModelClient> model_;
    KDL::TreeFkSolverPosFull_recursive* fksolver_;
    map<string, KDL::Frame > cartpos_;

    drc::robot_state_t rstate_;

    bool rstate_init_;
    bool cartpos_ready_;
    bool use_left_hand_;
    
    
    Pass::Ptr pass;
    image_io_utils*  imgutils_; 
    BotParam* botparam_; 
    CameraParams camera_params_;   
    int img_buf_size_;
    uint8_t* img_buf_;    
    
    
    ///
    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                           const  bot_core::images_t* msg);    
    void cameraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                             const  bot_core::image_t* msg);    
    
    pcl::PolygonMesh::Ptr palm_mesh_;
};   

App::App(int argc, char** argv, boost::shared_ptr<lcm::LCM> &lcm_, CommandlineConfig& cfg):
   lcm_(lcm_){
     
  std::string mesh_filename =  string(getenv ("DRC_BASE")) + "/software/models/mit_gazebo_models/sandia_hand/meshes/palm.ply";
  std::cout << "About to read: " << mesh_filename << std::endl;
  pcl::PolygonMesh mesh;       // (new pcl::PolygonMesh);
  pcl::io::loadPolygonFile (mesh_filename, mesh);
  pcl::PolygonMesh::Ptr palm_mesh_temp (new pcl::PolygonMesh (mesh));
  palm_mesh_ = palm_mesh_temp;     
     
  rstate_init_ = false;
  cartpos_ready_ = false;
  use_left_hand_ = true;
  
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  KDL::Tree tree;
  if (!kdl_parser::treeFromString( model_->getURDFString() ,tree)){
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    exit(-1);
  }
  fksolver_ = new KDL::TreeFkSolverPosFull_recursive(tree);     
  
  mainloop = g_main_loop_new (NULL, FALSE);
     
  lcm_->subscribe("EST_ROBOT_STATE",&App::robotStateHandler,this);
    
  trans_.assign(3,0);
  rpy_.assign(3,0);
  
  
  //////////////////////// PCLImage Passthrough Stuff:
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  camera_params_.setParams(botparam_, string("cameras." + cfg.camera_channel) );
      
  pass = Pass::Ptr (new Pass (argc, argv, lcm_, 
                              cfg.camera_channel, cfg.output_color_mode, 
                              cfg.use_convex_hulls, cfg.camera_frame, 
                              camera_params_, cfg.verbose));
  pass->setUpdateRobotState(false); // Don't update the robot state using EST_ROBOT_STATE
  pass->setRendererRobot(false); // Don't render the robot at all
  
  if (cfg.use_mono){
    lcm_->subscribe("CAMERA_LEFT",&App::cameraHandler,this);  
  }else{
    lcm_->subscribe("CAMERA",&App::multisenseHandler,this);
  }
  
  img_buf_size_ = 3* camera_params_.width  * camera_params_.height;
  img_buf_= (uint8_t*) malloc(img_buf_size_);
  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), 
                                  camera_params_.width, 
                                  3*camera_params_.height );  
  
  

}

void App::multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                             const  bot_core::images_t* msg){
  imgutils_->decodeImageToRGB(&(msg->images[0]),  img_buf_ );
  
  int64_t msg_time = msg->utime;
  if (pass->createMask(msg_time) ){
    pass->sendOutput(msg_time);  
    
    
    pass->sendOutputOverlay(msg_time, img_buf_);  
  }
}


void App::cameraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                             const  bot_core::image_t* msg){
  imgutils_->decodeImageToRGB( msg,  img_buf_ );
  
  /*
  int64_t msg_time = msg->utime;
  if (pass->createMask(msg_time) ){
    pass->sendOutput(msg_time);  
    
    
    pass->sendOutputOverlay(msg_time, img_buf_);  
  }
  */
}




void App::robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  rstate_= *msg;
  rstate_init_ = true;
}
  
 
void App::solveFK(drc::robot_state_t state){
  // 0. Extract World Pose of body:
  world_to_body_.setIdentity();
  world_to_body_.translation()  << state.pose.translation.x, state.pose.translation.y, state.pose.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(state.pose.rotation.w, state.pose.rotation.x, 
                                               state.pose.rotation.y, state.pose.rotation.z);
  world_to_body_.rotate(quat);    
    
  // 1. Solve for Forward Kinematics:
  // call a routine that calculates the transforms the joint_state_t* msg.
  cartpos_.clear();
  jointpos_.clear();
  for (uint i=0; i< (uint) state.num_joints; i++) //cast to uint to suppress compiler warning
    jointpos_.insert(make_pair(state.joint_name[i], state.joint_position[i]));
  
  // Calculate forward position kinematics
  bool kinematics_status;
  bool flatten_tree=true; // determines absolute transforms to robot origin, otherwise relative transforms between joints.
  kinematics_status = fksolver_->JntToCart(jointpos_,cartpos_,flatten_tree);
  if(kinematics_status>=0){
    // cout << "Success!" <<endl;
  }else{
    cerr << "Error: could not calculate forward kinematics!" <<endl;
    return;
  }
  
  cartpos_ready_=true;  
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


void App::publishCorrection(){
  if(!rstate_init_){
    return;
  }
  solveFK(rstate_);
    
  pass->setRobotState(world_to_body_, jointpos_);
  
  
  uint8_t* clean_buf_= (uint8_t*) malloc(img_buf_size_);
  memcpy(clean_buf_,img_buf_, img_buf_size_);
  
  
  
  std::string palm_link = "right_palm";
  //if (use_left_hand_){
  //  palm_link = "left_palm";
  //}
  Eigen::Isometry3d body_to_palm = KDLToEigen(cartpos_.find( palm_link )->second);
  Eigen::Isometry3d world_to_palm =  world_to_body_* body_to_palm;
  
  Eigen::Isometry3d palm_to_palm_corrected ( Eigen::Isometry3d::Identity() ); // the hoped for correction of the palm
  palm_to_palm_corrected.translation()  << trans_[0], trans_[1], trans_[2];
  palm_to_palm_corrected.rotate(  euler_to_quat( rpy_[0] , rpy_[1], rpy_[2] ) );   
  
  world_to_palm = world_to_palm*palm_to_palm_corrected;
  
  pcl::PolygonMesh::Ptr palm_mesh_copy_ (new pcl::PolygonMesh ( *palm_mesh_));
  
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::fromPCLPointCloud2(palm_mesh_copy_->cloud, *cloud);  
  Eigen::Isometry3f pose_f = world_to_palm.cast<float>();
  
  
  Eigen::Quaternionf quat_f(pose_f.rotation());
  pcl::transformPointCloud (*cloud, *cloud,
      pose_f.translation(), quat_f); // !! modifies cloud
  pcl::toPCLPointCloud2(*cloud, palm_mesh_copy_->cloud);    
  
  pass->setAffordanceMesh(palm_mesh_copy_);
  
  int64_t msg_time =bot_timestamp_now();
  if (pass->createMask(msg_time) ){
    pass->sendOutput(msg_time);  
    pass->sendOutputOverlay(msg_time, clean_buf_);  
  }  
  
  free (clean_buf_);
  
}




int App::repaint (int64_t now){
  clear();
  color_set(COLOR_PLAIN, NULL);
  
  //update:
  color_set(COLOR_PLAIN, NULL);
  wmove(w, 0, 0);
  wprintw(w, "<-    ->: move lr | rf  roll  [thumb-pinky]");
  wmove(w, 1, 0);
  wprintw(w, "/\\    \\/: move fr | yh  yaw [extend arm ]");
  wmove(w, 2, 0);
  wprintw(w, "q      a: move ud | p;  pitch [palm dn-up ]");
  wmove(w, 3, 0);
  wprintw(w, "spacebar: reset pose");
  wmove(w, 4, 0);
  wprintw(w, "enter   : republish");

  wmove(w, 6, 0);
  wprintw(w, "trans: %.4f %.4f %.4f",trans_[0] ,trans_[1] ,trans_[2]);
  wmove(w, 7, 0);
  wprintw(w, "  rpy: %.4f %.4f %.4f",180*rpy_[0]/M_PI ,180*rpy_[1]/M_PI ,180*rpy_[2]/M_PI );
  wmove(w, 8, 0);
  wprintw(w, "last %d", last_input_);
  
  wmove(w, 12, 0);
  wprintw(w, "[n]  Use left hand: %d (otherwise right hand)",  (int) use_left_hand_);

  color_set(COLOR_TITLE, NULL);
  
  wrefresh (w);
  return 0;
}


gboolean on_input (GIOChannel * source, GIOCondition cond, gpointer data){
  App* s = (App*) (data);  
  return s->on_input();
}
  
bool App::on_input(){
  int64_t now = bot_timestamp_now ();

  int c = getch();
    
  double delta_trans[]={0.0025,0.0025,0.0025};
  double delta_rpy[]={1,1,1}; // degrees

  wmove(w, 5, 0);
  wprintw(w,"%i  ",c);

  last_input_ = c;

  // 65 up, 66 down,
  switch (c)
  {
    case 32: // space bar:
      resetCorrection();
      break;
    case 10: // enter:
      publishCorrection();
      break;
    case 65: // up arrow:
      trans_[0] += delta_trans[0] ;
      publishCorrection();
      break;
    case 66: // down arrow:
      trans_[0] -= delta_trans[0] ;
      publishCorrection();
      break;
    case 68: // left arrow:
      trans_[1] += delta_trans[1] ;
      publishCorrection();
      break;
    case 67: // right arrow:
      trans_[1] -= delta_trans[1] ;
      publishCorrection();
      break;
    case 'a': // a
      trans_[2] -= delta_trans[2] ;
      publishCorrection();
      break;
    case 'q': // q
      trans_[2] += delta_trans[2] ;
      publishCorrection();
      break;
    ////////////////////////////
    case 'r': // roll
      rpy_[0] += delta_rpy[0]*M_PI/180 ;
      publishCorrection();
      break;
    case 'f': // roll
      rpy_[0] -= delta_rpy[0]*M_PI/180 ;
      publishCorrection();
      break;
    case 'p': // pitch
      rpy_[1] += delta_rpy[1]*M_PI/180 ;
      publishCorrection();
      break;
    case 59: // pitch
      rpy_[1] -= delta_rpy[1]*M_PI/180 ;
      publishCorrection();
      break;
    case 'y': // yaw
      rpy_[2] += delta_rpy[2]*M_PI/180 ;
      publishCorrection();
      break;
    case 'h': // yaw
      rpy_[2] -= delta_rpy[2]*M_PI/180 ;
      publishCorrection();
      break;
    case 'n': // switch modes: left or right hand
      use_left_hand_= !use_left_hand_;
      publishCorrection();
      break;
  }

  repaint ( now);       
  return TRUE;
}

gboolean on_timer (void * data)
{
  App* s = (App*) (data);
  return s->on_timer(); 
}
bool App::on_timer(){  
  int64_t now =0;// bot_timestamp_now ();
    
  repaint (now);
  return TRUE;
}


int main(int argc, char *argv[]){
  CommandlineConfig cfg;  
  
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  parser.add(cfg.camera_channel, "c", "camera_channel", "Camera channel");
  parser.add(cfg.camera_frame, "f", "camera_frame", "Camera frame");
  parser.add(cfg.output_color_mode, "o", "output_color_mode", "0rgb |1grayscale |2b/w");
  parser.add(cfg.use_convex_hulls, "u", "use_convex_hulls", "Use convex hull models");
  parser.add(cfg.verbose, "v", "verbose", "Verbose");
  parser.add(cfg.use_mono, "m", "use_mono", "Key off of the left monocularimage");  
  parser.parse();
  cout << cfg.camera_channel << " is camera_channel\n"; 
  cout << cfg.camera_frame << " is camera_frame\n"; 
  cout << cfg.output_color_mode << " is output_color_mode\n"; 
  cout << cfg.use_convex_hulls << " is use_convex_hulls\n"; 
  cout << cfg.verbose << " is verbose\n";
  cout << cfg.use_mono << " is use_mono\n";  

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  

  App app(argc,argv, lcm, cfg);  
  
  bot_glib_mainloop_attach_lcm (app .lcm_->getUnderlyingLCM() );
  bot_signal_pipe_glib_quit_on_kill (app.mainloop);

  // Watch stdin 
  GIOChannel * channel = g_io_channel_unix_new (0);
  g_io_add_watch (channel, G_IO_IN, on_input,&app);
  app.timer_id = g_timeout_add (25, on_timer, &app);

  app.w = initscr();
  start_color();
  cbreak();
  noecho();

  init_pair(COLOR_PLAIN, COLOR_WHITE, COLOR_BLACK);
  init_pair(COLOR_TITLE, COLOR_BLACK, COLOR_WHITE);
  init_pair(COLOR_WARN, COLOR_BLACK, COLOR_YELLOW);
  init_pair(COLOR_ERROR, COLOR_BLACK, COLOR_RED);

  g_main_loop_run (app.mainloop);

  endwin ();
  g_source_remove (app.timer_id);
  bot_glib_mainloop_detach_lcm (app.lcm_->getUnderlyingLCM());
  g_main_loop_unref (app.mainloop);
  app.mainloop = NULL;
}
