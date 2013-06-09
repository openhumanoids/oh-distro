#include <iostream>
#include <boost/function.hpp>

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include <GL/gl.h>
#include <bot_vis/bot_vis.h>


#include <bot_core/rotations.h>
#include <gdk/gdkkeysyms.h>
#include <path_util/path_util.h>


#include "RobotStateListener.hpp"
#include "pose_approval_gui_utils.hpp"
#include "popup_gui_utils.hpp"
#include "renderer_robot_state.hpp"



#define RENDERER_NAME "Robot State Display"


using namespace std;
using namespace boost;
using namespace Eigen;
using namespace visualization_utils;
using namespace collision;
using namespace renderer_robot_state;
using namespace renderer_robot_state_gui_utils;



static void
_renderer_free (BotRenderer *super)
{
  RobotStateRendererStruc *self = (RobotStateRendererStruc*) super->user;
  free(self);
}

//========================= Event Handling ================

static double pick_query (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], const double ray_dir[3])
{
  RobotStateRendererStruc *self = (RobotStateRendererStruc*) ehandler->user;

  if((self->selection_enabled==0)||(self->robotStateListener->_urdf_subscription_on)){
    return -1.0;
  }
  
  //fprintf(stderr, "RobotStateRenderer Pick Query Active\n");
  Eigen::Vector3f from,to;
  from << ray_start[0], ray_start[1], ray_start[2];

  Eigen::Vector3f plane_normal,plane_pt;
  plane_normal << 0,0,1;
  if(ray_start[2]<0)
      plane_pt << 0,0,10;
  else
      plane_pt << 0,0,-10;
  double lambda1 = ray_dir[0] * plane_normal[0]+
                   ray_dir[1] * plane_normal[1] +
                   ray_dir[2] * plane_normal[2];
   // check for degenerate case where ray is (more or less) parallel to plane
    if (fabs (lambda1) < 1e-9) return -1.0;

   double lambda2 = (plane_pt[0] - ray_start[0]) * plane_normal[0] +
       (plane_pt[1] - ray_start[1]) * plane_normal[1] +
       (plane_pt[2] - ray_start[2]) * plane_normal[2];
   double t = lambda2 / lambda1;// =1;
  
  to << ray_start[0]+t*ray_dir[0], ray_start[1]+t*ray_dir[1], ray_start[2]+t*ray_dir[2];
 
  self->ray_start = from;
  self->ray_end = to;
  self->ray_hit_t = t;
  self->ray_hit_drag = to;
  self->ray_hit = to;   

  double shortest_distance = get_shortest_distance_between_robot_links_and_jointdof_markers(self,from,to);
  return shortest_distance;
}

static int mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], const double ray_dir[3], const GdkEventButton *event)
{
  RobotStateRendererStruc *self = (RobotStateRendererStruc*) ehandler->user;
  if((ehandler->picking==0)||(self->selection_enabled==0)){
    return 0;
  }

  self->clicked = 1;
  
  if((*self->marker_selection)!=" ") {
     self->dragging = 1;
     KDL::Frame T_world_body = self->robotStateListener->_gl_robot->_T_world_body;
     self->marker_offset_on_press << self->ray_hit[0]-T_world_body.p[0],self->ray_hit[1]-T_world_body.p[1],self->ray_hit[2]-T_world_body.p[2]; 
     bot_viewer_request_redraw(self->viewer);
     return 1;// consumed
  }

    bot_viewer_request_redraw(self->viewer);
    return 0;
}

static int 
mouse_release (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
    const double ray_dir[3], const GdkEventButton *event)
{
  RobotStateRendererStruc *self = (RobotStateRendererStruc*) ehandler->user;
  self->clicked = 0;
  if((ehandler->picking==0)||(self->selection_enabled==0)){
    return 0;
  }
  if (self->dragging) {
    self->dragging = 0;
    
    Eigen::Vector3f diff=self->ray_hit_drag-self->ray_hit;
    double movement = diff.norm();
    if(((*self->marker_selection)  != " ")&&(movement>=1e-3)){
      cout << "publishing posture goal \n";
      string channel = "POSTURE_GOAL";
      publish_posture_goal(self,channel);
    }
  }  
  
  if (ehandler->picking==1)
    ehandler->picking=0; //release picking(IMPORTANT)
  bot_viewer_request_redraw(self->viewer);
  return 0;
}

static int mouse_motion (BotViewer *viewer, BotEventHandler *ehandler,  const double ray_start[3], const double ray_dir[3],   const GdkEventMotion *event)
{
  RobotStateRendererStruc *self = (RobotStateRendererStruc*) ehandler->user;
  
  if((!self->dragging)||(ehandler->picking==0)){
    return 0;
  }
  
  if((*self->marker_selection)  != " "){
    Eigen::Vector3f start,dir;
    dir<< ray_dir[0], ray_dir[1], ray_dir[2];
    start<< ray_start[0], ray_start[1], ray_start[2];
    set_desired_robot_posture_on_marker_motion(self,start,dir);
    //cout << (*self->marker_selection) << ": mouse drag\n";
  }
  bot_viewer_request_redraw(self->viewer);
  return 1;
}


//=================================

static void draw_ee_force(void *user,std::string ee_name)
{
    RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
    std::map<std::string, Eigen::Vector3f >::iterator it;
    it=self->robotStateListener->ee_forces_map.find(ee_name);
    KDL::Frame ee_frame;
    self->robotStateListener->_gl_robot->get_link_frame(ee_name, ee_frame);
    if(it!=self->robotStateListener->ee_forces_map.end()) 
    {
      //draw arrow
      Eigen::Vector3f F = it->second;
      
      //cout << "F:" <<F.transpose() << " " << F.norm() << endl;
      double length = 0.05*(F.norm()/9.81);//(F_meas/g)x0.1
      
      std::stringstream oss;
      oss << F.norm();
      std::string text = oss.str() + " N";
     double pos[3];
      pos[0] = ee_frame.p[0]; 
      pos[1] = ee_frame.p[1]; 
      pos[2] = ee_frame.p[2]+0.0;
      double head_width = 0.03; double head_length = 0.03;double body_width = 0.01;
      glColor4f(1,0,0,1);
      bot_gl_draw_text(pos, GLUT_BITMAP_HELVETICA_18, text.c_str(),0);
      glPushMatrix();
      glTranslatef(ee_frame.p[0], ee_frame.p[1],ee_frame.p[2]);
      Eigen::Vector3f F_worldframe;
      rotate_eigen_vector_given_kdl_frame(F,ee_frame,F_worldframe); 
     // cout << "F_worldframe:" <<F_worldframe.transpose() << " " << F_worldframe.norm() << endl;
      //--get rotation in angle/axis form
      double theta;
      Eigen::Vector3f direction=F_worldframe;
      direction.normalize(); 
      //--get rotation in angle/axis form
      Eigen::Vector3f axis;
      Eigen::Vector3f uz,ux;   uz << 0 , 0 , 1;ux << 1 , 0 , 0;
      axis = ux.cross(direction);axis.normalize();
      theta = acos(ux.dot(direction));
      glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]); 
      glTranslatef(length/2, 0,0);
      bot_gl_draw_arrow_3d(length,head_width, head_length,body_width);
      glTranslatef(length/2, 0,0);

      glPopMatrix();
    }
  
}

static void 
_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
 //int64_t tic = bot_timestamp_now();
 
  RobotStateRendererStruc *self = (RobotStateRendererStruc*) super->user;

  glEnable(GL_DEPTH_TEST);

  //-draw 
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
  //glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); 
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
  glEnable (GL_RESCALE_NORMAL);

  if((self->ehandler.picking)&&(self->selection_enabled)&&(self->clicked)){
        glLineWidth (3.0);
        glPushMatrix();
        glBegin(GL_LINES);
        glVertex3f(self->ray_start[0], self->ray_start[1],self->ray_start[2]); // object coord
        glVertex3f(self->ray_end[0], self->ray_end[1],self->ray_end[2]);
        glEnd();
        glPopMatrix();
  }
 
  float c[3] = {0.15,0.15,0.15}; // was 0.3
  //glColor3f(c[0],c[1],c[2]);
  glColor4f(c[0],c[1],c[2],self->alpha);
  
  float alpha = self->alpha;
  if(self->robotStateListener->_gl_robot)
  {
    self->robotStateListener->_gl_robot->show_bbox(self->visualize_bbox);
    self->robotStateListener->_gl_robot->enable_link_selection(self->selection_enabled);
    self->robotStateListener->_gl_robot->highlight_link((*self->selection));
    self->robotStateListener->_gl_robot->highlight_marker((*self->marker_selection));
    //self->robotStateListener->_gl_robot->enable_whole_body_selection(self->selection_enabled);
    self->robotStateListener->_gl_robot->draw_body (c,alpha);
    
    if(self->visualize_forces){
      draw_ee_force(self,"l_hand");   draw_ee_force(self,"r_hand");
      draw_ee_force(self,"l_foot");   draw_ee_force(self,"r_foot");
    }
  }
  
  if(self->robotStateListener->_end_pose_received){
     self->robotStateListener->_end_pose_received = false;
     spawn_pose_approval_dock(self);
   }
   
  if((self->teleop_popup !=NULL))
  {
      // move teleop popup to account for viewer movement
      gint root_x, root_y;
      gtk_window_get_position (GTK_WINDOW(self->viewer->window), &root_x, &root_y);
      gint width, height;
      gtk_window_get_size(GTK_WINDOW(self->viewer->window),&width,&height);
     
     
     gint pos_x, pos_y;
     pos_x=root_x+0.5*width-250;    pos_y=root_y+0.9*height-100;
     
     gint current_pos_x, current_pos_y;
     if((fabs(current_pos_x-pos_x)+fabs(current_pos_y-pos_y))>1)
          gtk_window_move(GTK_WINDOW(self->teleop_popup ),pos_x,pos_y);
  } 

// int64_t toc = bot_timestamp_now();
// cout << bot_timestamp_useconds(toc-tic) << endl;
}



static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
  if (! strcmp(name, PARAM_SELECTION)) {
    if (bot_gtk_param_widget_get_bool(pw, PARAM_SELECTION)) {
      //bot_viewer_request_pick (self->viewer, &(self->ehandler));
      self->selection_enabled = 1;
    }
    else{
      self->selection_enabled = 0;
    }
  }
  else if(! strcmp(name, PARAM_WIRE)) {
    if (bot_gtk_param_widget_get_bool(pw, PARAM_WIRE)){
      self->visualize_bbox= true;  
    }
    else{
      self->visualize_bbox = false;
    }
  }
  else if(! strcmp(name, PARAM_SHOW_FORCES)) {
    if (bot_gtk_param_widget_get_bool(pw, PARAM_SHOW_FORCES)){
      self->visualize_forces= true;  
    }
    else{
      self->visualize_forces = false;
    }
  }
  else if(! strcmp(name, PARAM_COLOR_ALPHA)) {
    self->alpha = (float) bot_gtk_param_widget_get_double(pw, PARAM_COLOR_ALPHA);
    bot_viewer_request_redraw(self->viewer);
  }
  else if(! strcmp(name, PARAM_SEND_POSTURE_GOAL)) {
    string channel = "POSTURE_GOAL";
    publish_posture_goal(self,channel);
  }
  else if(! strcmp(name, PARAM_RESET_POSTURE)) {
   if(self->robotStateListener->_gl_robot->is_future_state_changing())
      self->robotStateListener->_gl_robot->set_future_state_changing(false);
   self->robotStateListener->_gl_robot->set_future_state( self->robotStateListener->_gl_robot->_T_world_body, self->robotStateListener->_gl_robot->_current_jointpos);   
   self->robotStateListener->_gl_robot->disable_future_display();      
   bot_viewer_request_redraw(self->viewer);
  }
  else if (! strcmp(name, PARAM_ENABLE_POSTURE_ADJUSTMENT)) 
  {
      bool val = bot_gtk_param_widget_get_bool(pw, PARAM_ENABLE_POSTURE_ADJUSTMENT);
      if(!(self->robotStateListener->_urdf_subscription_on))
      {
        bot_gtk_param_widget_set_bool(pw, PARAM_SELECTION,val);
        self->selection_enabled = val; // enable selection
        if(val)
           std::cout << "enabling jointdof adjustment for robot " << std::endl;
        else
           std::cout << "disabling jointdof adjustment for robot "<< std::endl;
        self->robotStateListener->_gl_robot->enable_jointdof_adjustment(val); 
      }    
      else
        std::cerr << "Robot urdf not received, can't adjust robot posture as no robot object is created. "<< std::endl;
  }
  else if (! strcmp(name,PARAM_ENABLE_EE_TELEOP))
  {
     spawn_teleop_ee_popup(self);

  } 
}

void 
setup_renderer_robot_state(BotViewer *viewer, int render_priority, lcm_t *lcm, int operation_mode)
{
    RobotStateRendererStruc *self = (RobotStateRendererStruc*) calloc (1, sizeof (RobotStateRendererStruc));
    self->lcm = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcm));

    self->robotStateListener = boost::shared_ptr<RobotStateListener>(new RobotStateListener(self->lcm, 
              viewer, operation_mode));
    
    BotRenderer *renderer = &self->renderer;

    renderer->draw = _renderer_draw;
    renderer->destroy = _renderer_free;

    renderer->widget = bot_gtk_param_widget_new();
    renderer->name = (char *) RENDERER_NAME;
    if (operation_mode==1){
      renderer->name =(char *) "Robot State LB Compressed";
    }
    
    renderer->user = self;
    renderer->enabled = 1;

    self->viewer = viewer;

    self->pw = BOT_GTK_PARAM_WIDGET(renderer->widget);

    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SELECTION, 0, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_WIRE, 0, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_FORCES, 0, NULL);
    
    bool val=false;
    if(!(self->robotStateListener->_urdf_subscription_on))
      val = self->robotStateListener->_gl_robot->is_jointdof_adjustment_enabled();
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_ENABLE_POSTURE_ADJUSTMENT, val, NULL);
    bot_gtk_param_widget_add_buttons(self->pw,PARAM_RESET_POSTURE, NULL);
    //bot_gtk_param_widget_add_buttons(self->pw,PARAM_SEND_POSTURE_GOAL, NULL);
    

    bot_gtk_param_widget_add_buttons(self->pw,PARAM_ENABLE_EE_TELEOP,NULL);
    
    
    bot_gtk_param_widget_add_double (self->pw, PARAM_COLOR_ALPHA, BOT_GTK_PARAM_WIDGET_SLIDER, 0, 1, 0.001, 1);
      
    g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    self->alpha = 1.0;
    self->selection_enabled = 0;
    bot_gtk_param_widget_set_bool(self->pw, PARAM_SELECTION,self->selection_enabled);
    self->clicked = 0;	
    self->selection = new std::string(" ");
    self->dragging = 0;  
  	self->marker_selection = new std::string(" ");
  	self->pose_approval_dock= NULL; 
  	self->teleop_popup = NULL;
  	self->teleop_error_entry =NULL;
    self->visualize_bbox = false;
    self->visualize_forces = false;
    bot_viewer_add_renderer(viewer, &self->renderer, render_priority);


    //----------
    // create and register mode handler
    /*self->key_handler = (BotEventHandler*) calloc(1, sizeof(BotEventHandler));
    self->key_handler->name = strdup(std::string("Mode Control").c_str());
    self->key_handler->enabled = 0;
    self->key_handler->key_press = cb_key_press;
    //self->key_handler->key_release = cb_key_release;
    self->key_handler->user = self;
    bot_viewer_add_event_handler(viewer, self->key_handler, 1);*/
    
    
    BotEventHandler *ehandler = &self->ehandler;
    ehandler->name = (char*) RENDERER_NAME;
    if (operation_mode==1){
      ehandler->name =(char *) "Robot State LB Compressed";
    }
    ehandler->enabled = 1;
    ehandler->pick_query = pick_query;
    ehandler->hover_query = NULL;
    ehandler->mouse_press = mouse_press;
    ehandler->mouse_release = mouse_release;
    ehandler->mouse_motion = mouse_motion;
    ehandler->user = self;

    bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);
    
}

