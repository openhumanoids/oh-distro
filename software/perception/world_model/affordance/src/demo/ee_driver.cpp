// simple ncurses tool
// to modify a bot_frame transform 
//
// was used to explore/improve the calibration of the head

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

#include "lcmtypes/drc_lcmtypes.hpp"
#include <bot_core/bot_core.h>

#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <model-client/model-client.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds


#include <ConciseArgs>

#define COLOR_PLAIN 1
#define COLOR_TITLE 2
#define COLOR_ERROR 3
#define COLOR_WARN  4

using namespace std;

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~App(){
    }
    
    void publish_palm_goal();
    void publish_reset();

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
    void manipPlanHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::robot_plan_w_keyframes_t* msg);    
    
    void solveFK(drc::robot_state_t state);
    
    // Values are in the body frame - but finally published in the world frame
    vector<double> trans_;
    vector<double> rpy_;
    
    int last_input_;

    Eigen::Isometry3d world_to_body_;
    
    boost::shared_ptr<ModelClient> model_;
    KDL::TreeFkSolverPosFull_recursive* fksolver_;
    map<string, KDL::Frame > cartpos_;

    drc::robot_state_t rstate_;
    drc::robot_state_t planstate_;

    bool planstate_init_;
    bool rstate_init_;
    bool cartpos_ready_;
    bool plan_from_robot_state_;
    bool use_left_hand_;
    bool use_sandia_;
};   

App::App(boost::shared_ptr<lcm::LCM> &lcm_):
   lcm_(lcm_){
     
  planstate_init_ =false;
  rstate_init_ = false;
  cartpos_ready_ = false;
  plan_from_robot_state_ = true;
  use_left_hand_ = true;
  use_sandia_ = true;
  
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  KDL::Tree tree;
  if (!kdl_parser::treeFromString( model_->getURDFString() ,tree)){
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    exit(-1);
  }
  fksolver_ = new KDL::TreeFkSolverPosFull_recursive(tree);     
  
  mainloop = g_main_loop_new (NULL, FALSE);
     
  lcm_->subscribe("EST_ROBOT_STATE",&App::robotStateHandler,this);
  lcm_->subscribe("CANDIDATE_MANIP_PLAN",&App::manipPlanHandler,this);
    
  trans_ = {0.0, 0.0, 0.0};    
  rpy_ =  {0,0,0};
}


void App::manipPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_plan_w_keyframes_t* msg){
  planstate_= msg->plan[ msg->num_states-1];
  planstate_init_ = true;
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
  map<string, double> jointpos_in;
  cartpos_.clear();
  for (uint i=0; i< (uint) state.num_joints; i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(state.joint_name[i], state.joint_position[i]));
  
  // Calculate forward position kinematics
  bool kinematics_status;
  bool flatten_tree=true; // determines absolute transforms to robot origin, otherwise relative transforms between joints.
  kinematics_status = fksolver_->JntToCart(jointpos_in,cartpos_,flatten_tree);
  if(kinematics_status>=0){
    // cout << "Success!" <<endl;
  }else{
    cerr << "Error: could not calculate forward kinematics!" <<endl;
    return;
  }
  
  cartpos_ready_=true;  
}




void App::publish_palm_goal(){
  // palm goal is published in world frame
  
  Eigen::Isometry3d body_to_palm;
  body_to_palm.setIdentity();
  body_to_palm.translation()<< trans_[0] , trans_[1] , trans_[2];
  body_to_palm.rotate( euler_to_quat ( rpy_[0], rpy_[1], rpy_[2] ) );
  Eigen::Isometry3d world_to_palm =  world_to_body_* body_to_palm;
  
  drc::ee_goal_t msg;
  msg.utime = bot_timestamp_now();

  msg.ee_goal_pos.translation.x = world_to_palm.translation().x();
  msg.ee_goal_pos.translation.y = world_to_palm.translation().y();
  msg.ee_goal_pos.translation.z = world_to_palm.translation().z();
  
  Eigen::Quaterniond q = Eigen::Quaterniond (world_to_palm.rotation());
  msg.ee_goal_pos.rotation.w = q.w();
  msg.ee_goal_pos.rotation.x = q.x();
  msg.ee_goal_pos.rotation.y = q.y();
  msg.ee_goal_pos.rotation.z = q.z();
  msg.num_chain_joints = 0;

  if (!use_left_hand_){
    lcm_->publish("LEFT_PALM_GOAL_CLEAR", &msg);
    lcm_->publish("RIGHT_PALM_GOAL", &msg);
  }else {
    lcm_->publish("LEFT_PALM_GOAL", &msg);
    lcm_->publish("RIGHT_PALM_GOAL_CLEAR", &msg);
  }
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


void App::publish_reset(){
  if (plan_from_robot_state_){
    if(rstate_init_){
      solveFK(rstate_);
    }
  }else{
    if (planstate_init_){
      solveFK(planstate_);
    }
  }
    
  if (!cartpos_ready_){
   return; 
  }
  
  std::string palm_link = "left_palm";
  if (use_left_hand_ && use_sandia_){
    palm_link = "left_palm";
  }else if (!use_left_hand_ && use_sandia_){
    palm_link = "right_palm";
  }else if (use_left_hand_ && !use_sandia_){
    palm_link = "left_base_link";
  }else if (!use_left_hand_ && !use_sandia_){
    palm_link = "right_base_link";
  }
  Eigen::Isometry3d body_to_palm = KDLToEigen(cartpos_.find( palm_link )->second);
  trans_[0] = body_to_palm.translation().x();
  trans_[1] = body_to_palm.translation().y();
  trans_[2] = body_to_palm.translation().z();
  quat_to_euler(  Eigen::Quaterniond(body_to_palm.rotation()) ,rpy_[0],rpy_[1],rpy_[2] );
  
  /*
  Eigen::Isometry3d world_to_palm =  world_to_body_* body_to_palm;
  trans_[0] = world_to_palm.translation().x();
  trans_[1] = world_to_palm.translation().y();
  trans_[2] = world_to_palm.translation().z();
  quat_to_euler(  Eigen::Quaterniond(world_to_palm.rotation()) ,rpy_[0],rpy_[1],rpy_[2] );
  */
  publish_palm_goal();
}




int App::repaint (int64_t now){
  clear();
  color_set(COLOR_PLAIN, NULL);
  
  //update:
  color_set(COLOR_PLAIN, NULL);
  wmove(w, 0, 0);
  wprintw(w, "<-    ->: move lr | rf  roll");
  wmove(w, 1, 0);
  wprintw(w, "/\\    \\/: move fr | yh  yaw");
  wmove(w, 2, 0);
  wprintw(w, "q      a: move ud | p;  pitch");
  wmove(w, 3, 0);
  wprintw(w, "spacebar: reset pose");

  wmove(w, 6, 0);
  wprintw(w, "trans: %.4f %.4f %.4f",trans_[0] ,trans_[1] ,trans_[2]);
  wmove(w, 7, 0);
  wprintw(w, "  rpy: %.4f %.4f %.4f",180*rpy_[0]/M_PI ,180*rpy_[1]/M_PI ,180*rpy_[2]/M_PI );
  wmove(w, 8, 0);
  wprintw(w, "last: %d", last_input_);
  
  std::string mode_string = "Current State";
  if (!plan_from_robot_state_)
    mode_string = "Last Plan";  
  
  std::string hand_side_string = "Left ";
  if (!use_left_hand_)
    hand_side_string = "Right";
  
  std::string hand_type_string = "Sandia";
  if (!use_sandia_)
    hand_type_string = "iRobot";
  
  wmove(w, 10, 0);
  wprintw(w, "conf: %s %s with %s", hand_side_string.c_str() , hand_type_string.c_str(), mode_string.c_str() );
  wmove(w, 11, 0);
  wprintw(w, "      [z]   [x]         [c] ",  (int) use_sandia_);
  //wmove(w, 12, 0);
  //wprintw(w, "      L/R   S/iR        Current/Plan",  (int) use_sandia_);

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
    
  double delta_trans[]={0.01,0.01,0.01};
  double delta_rpy[]={5,5,5}; // degrees

  wmove(w, 5, 0);
  wprintw(w,"%i  ",c);

  last_input_ = c;

  // 65 up, 66 down,
  switch (c)
  {
    case 32: // space bar:
      publish_reset();
      break;
    case 65: // up arrow:
      trans_[0] += delta_trans[0] ;
      publish_palm_goal();
      break;
    case 66: // down arrow:
      trans_[0] -= delta_trans[0] ;
      publish_palm_goal();
      break;
    case 68: // left arrow:
      trans_[1] += delta_trans[1] ;
      publish_palm_goal();
      break;
    case 67: // right arrow:
      trans_[1] -= delta_trans[1] ;
      publish_palm_goal();
      break;
    case 'a': // a
      trans_[2] -= delta_trans[2] ;
      publish_palm_goal();
      break;
    case 'q': // q
      trans_[2] += delta_trans[2] ;
      publish_palm_goal();
      break;
    ////////////////////////////
    case 'r': // roll
      rpy_[0] += delta_rpy[0]*M_PI/180 ;
      publish_palm_goal();
      break;
    case 'f': // roll
      rpy_[0] -= delta_rpy[0]*M_PI/180 ;
      publish_palm_goal();
      break;
    case 'p': // pitch
      rpy_[1] += delta_rpy[1]*M_PI/180 ;
      publish_palm_goal();
      break;
    case 59: // pitch
      rpy_[1] -= delta_rpy[1]*M_PI/180 ;
      publish_palm_goal();
      break;
    case 'y': // yaw
      rpy_[2] += delta_rpy[2]*M_PI/180 ;
      publish_palm_goal();
      break;
    case 'h': // roll
      rpy_[2] -= delta_rpy[2]*M_PI/180 ;
      publish_palm_goal();
      break;
    case 'c': // switch modes: hand and end of plan
      plan_from_robot_state_= !plan_from_robot_state_;
      break;
    case 'z': // switch modes: hand and end of plan
      use_left_hand_= !use_left_hand_;
      publish_reset();
      break;
    case 'x': // switch modes: hand and end of plan
      use_sandia_= !use_sandia_;
      publish_reset();
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

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  

  App app(lcm);  
  
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