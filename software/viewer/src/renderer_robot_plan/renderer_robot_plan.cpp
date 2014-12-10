
#include "renderer_robot_plan.hpp"
#include "RobotPlanListener.hpp"
#include "lcmtypes/drc/ee_goal_t.hpp"
#include "lcmtypes/drc/ee_manip_gain_t.hpp"
#include "lcmtypes/drc/manip_plan_control_t.hpp"
#include "lcmtypes/drc/plan_execution_speed_t.hpp"
#include "lcmtypes/drc/plan_adjust_mode_t.hpp"
#include "lcmtypes/drc_atlas_status_t.h"
#include "lcmtypes/drc_robot_plan_t.h"
#include "lcmtypes/drc_utime_t.h"

#define PARAM_KP_DEFAULT 5
#define PARAM_KD_DEFAULT 1
#define PARAM_KP_MIN 0
#define PARAM_KD_MIN 0
#define PARAM_KP_MAX 100
#define PARAM_KD_MAX 100
#define PARAM_KP_INC 1
#define PARAM_KD_INC 1

#define RENDERER_NAME "Planning"
#define PARAM_PLAN_PART "Part of Plan"  
#define PARAM_SHOW_FULLPLAN "Show Full Plan"	

using namespace std;
using namespace boost;
using namespace renderer_robot_plan;

static void
_renderer_free (BotRenderer *super)
{
  RendererRobotPlan *self = (RendererRobotPlan*) super->user;
  free(self);
}


// Convert number to jet colour coordinates
// In: number between 0-->1
// Out: rgb jet colours 0->1
// http://metastine.com/2011/01/implementing-a-continuous-jet-colormap-function-in-glsl/
static inline void jet_rgb(float value,float rgb[]){
  float fourValue = (float) 4 * value;
  rgb[0]   = std::min(fourValue - 1.5, -fourValue + 4.5);
  rgb[1] = std::min(fourValue - 0.5, -fourValue + 3.5);
  rgb[2]  = std::min(fourValue + 0.5, -fourValue + 2.5);
  for (int i=0;i<3;i++){
   if (rgb[i] <0) {
     rgb[i] =0;
   }else if (rgb[i] >1){
     rgb[i] =1;
   }
  }
}

static void 
draw_state(BotViewer *viewer, BotRenderer *super, uint i, float rgb[]){

  float c[3] = {0.3,0.3,0.6}; // light blue
  float alpha = 0.4;
  RendererRobotPlan *self = (RendererRobotPlan*) super->user;
 
  if((self->use_colormap)&&(self->displayed_plan_index==-1)&&(!self->robotPlanListener->_is_manip_plan)) {
    // Each model Jet: blue to red
    float j = (float)i/ (self->robotPlanListener->_gl_robot_list.size() -1);
    jet_rgb(j,c);
  }else{
    c[0] = rgb[0]; c[1] = rgb[1]; c[2] = rgb[2];
  }
  
  glColor4f(c[0],c[1],c[2], alpha);
  
  
  self->robotPlanListener->_gl_robot_list[i]->show_bbox(self->visualize_bbox);
  self->robotPlanListener->_gl_robot_list[i]->enable_link_selection(self->selection_enabled);
  //if((*self->selection)!=" ")
  self->robotPlanListener->_gl_robot_list[i]->highlight_link((*self->selection));
  self->robotPlanListener->_gl_robot_list[i]->draw_body (c,alpha);  
}



static void 
_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
  RendererRobotPlan *self = (RendererRobotPlan*) super->user;
  
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  //-draw 
  //glPointSize(5.0f);
  //glBegin(GL_POINTS);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

  if((self->selection_enabled)&&(self->clicked)){
    glLineWidth (3.0);
    glPushMatrix();
    glBegin(GL_LINES);
    glVertex3f(self->ray_start[0], self->ray_start[1],self->ray_start[2]); // object coord
    glVertex3f(self->ray_end[0], self->ray_end[1],self->ray_end[2]);
    glEnd();
    glPopMatrix();
  }

  int plan_size =   self->robotPlanListener->_gl_robot_list.size();
  if (plan_size ==0){ // nothing to renderer
  // on receipt of a apprved footstep plan, the current plan is purged in waiting for a new walking plan.
  // if any plan execution/approval dock's exist, they will be destroyed.
    if(self->plan_execution_dock!=NULL){
      gtk_widget_destroy(self->plan_execution_dock);
      self->plan_execution_dock= NULL;  
    } 
    if(self->multiapprove_plan_execution_dock!=NULL)
    {
      gtk_widget_destroy(self->multiapprove_plan_execution_dock);
      self->multiapprove_plan_execution_dock= NULL;  
    } 
    if(self->plan_approval_dock!=NULL){
      gtk_widget_destroy(self->plan_approval_dock);
      self->plan_approval_dock= NULL;  
    }   
    return;
  }

  self->selected_keyframe_index=-1; // clear selected keyframe index
  
  bool markeractive = false;  
    
  if (self->show_fullplan){
    int max_num_states = 20;
    int inc =1;
    int totol_states = self->robotPlanListener->_gl_robot_list.size();
    if ( totol_states > max_num_states) {
      inc = ceil( totol_states/max_num_states);
      inc = min(max(inc,1),max_num_states);	
    }    
    //std::cout << "totol_states is " << totol_states << "\n";    
    //std::cout << "inc is " << inc << "\n";    

    float c[3] = {0.3,0.3,0.6}; // light blue (holder)
    for(uint i = 0; i < totol_states; i=i+inc){//_gl_robot_list.size(); i++){ 
      if(!markeractive)
        draw_state(viewer,super,i,c);
    }
    self->displayed_plan_index = -1;    
  }else{
    double plan_part = bot_gtk_param_widget_get_double(self->pw, PARAM_PLAN_PART);
    uint w_plan = (uint) round(plan_part* (plan_size -1));
    //printf("                                  Show around %f of %d    %d\n", plan_part, plan_size, w_plan);
    self->displayed_plan_index = w_plan;
    
    float c[3] = {0.3,0.3,0.6}; // light blue
    if(!markeractive)
      draw_state(viewer,super,w_plan,c);
  }
  
}

  
//========================= Event Handling ================

static double pick_query (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], const double ray_dir[3])
{
  RendererRobotPlan *self = (RendererRobotPlan*) ehandler->user;
    return -1.0;
}

static int mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], const double ray_dir[3], const GdkEventButton *event)
{
  RendererRobotPlan *self = (RendererRobotPlan*) ehandler->user;

  bot_viewer_request_redraw(self->viewer);
  return 0;
}


static int 
mouse_release (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
    const double ray_dir[3], const GdkEventButton *event)
{
  RendererRobotPlan *self = (RendererRobotPlan*) ehandler->user;
  self->clicked = 0;
  if((ehandler->picking==0)||(self->selection_enabled==0)){
    //fprintf(stderr, "Ehandler Not active\n");
    return 0;
  }
  if (self->dragging) {
    self->dragging = 0;
    string channel = "MANIP_PLAN_CONSTRAINT";
    
    Eigen::Vector3f diff=self->ray_hit_drag-self->ray_hit;
    double movement = diff.norm();

  }
  if (ehandler->picking==1)
    ehandler->picking=0; //release picking(IMPORTANT)
  bot_viewer_request_redraw(self->viewer);
  return 0;
}


static int mouse_motion (BotViewer *viewer, BotEventHandler *ehandler,  const double ray_start[3], const double ray_dir[3],   const GdkEventMotion *event)
{
  RendererRobotPlan *self = (RendererRobotPlan*) ehandler->user;
  
  if((!self->dragging)||(ehandler->picking==0)){
    return 0;
  }
  
  if((*self->marker_selection)  != " "){
    double t = self->ray_hit_t;
    self->ray_hit_drag << ray_start[0]+t*ray_dir[0], ray_start[1]+t*ray_dir[1], ray_start[2]+t*ray_dir[2];
    //TODO: Add support for joint dof markers for keyframes [switch via check box in main pane instead of a popup]
    //TODO: JointDof constraint comms with planner (pain) 
      Eigen::Vector3f start,dir;
    dir<< ray_dir[0], ray_dir[1], ray_dir[2];
    start<< ray_start[0], ray_start[1], ray_start[2]; 
    adjust_keyframe_on_marker_motion(self,start,dir);
    // cout << (*self->marker_selection) << ": mouse drag\n";
  }
  bot_viewer_request_redraw(self->viewer);
  return 1;
}


static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererRobotPlan *self = (RendererRobotPlan*) user;
  if(! strcmp(name,PARAM_SHOW_FULLPLAN)) {
    self->show_fullplan = bot_gtk_param_widget_get_bool(pw, PARAM_SHOW_FULLPLAN);
  }
  bot_viewer_request_redraw(self->viewer);
  
}

void 
setup_renderer_robot_plan(BotViewer *viewer, int render_priority, lcm_t *lcm, int operation_mode, KeyboardSignalRef signalRef,AffTriggerSignalsRef affTriggerSignalsRef,RendererFoviationSignalRef rendererFoviationSignalRef)
{
    RendererRobotPlan *self = (RendererRobotPlan*) calloc (1, sizeof (RendererRobotPlan));
    self->lcm = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcm));
    
    self->robotPlanListener = boost::shared_ptr<RobotPlanListener>(new RobotPlanListener(self->lcm, viewer, operation_mode));
    //self->keyboardSignalHndlr = boost::shared_ptr<KeyboardSignalHandler>(new KeyboardSignalHandler(signalRef,keyboardSignalCallback));
    self->keyboardSignalHndlr = boost::shared_ptr<KeyboardSignalHandler>(new KeyboardSignalHandler(signalRef,boost::bind(&RendererRobotPlan::keyboardSignalCallback,self,_1,_2)));
    self->affTriggerSignalsHndlr = boost::shared_ptr<AffTriggerSignalsHandler>(new AffTriggerSignalsHandler(affTriggerSignalsRef,boost::bind(&RendererRobotPlan::affTriggerSignalsCallback,self,_1,_2,_3,_4)));
    self->_rendererFoviationSignalRef = rendererFoviationSignalRef;
    
    BotRenderer *renderer = &self->renderer;

    renderer->draw = _renderer_draw;
    renderer->destroy = _renderer_free;

    renderer->widget = bot_gtk_param_widget_new();
    renderer->name = (char *) RENDERER_NAME;
    if (operation_mode ==1){
      renderer->name =(char *) "Robot Plan Loopback";
    }else if(operation_mode ==2){
      renderer->name =(char *) "Robot Plan LB Compressed";      
    }
    renderer->user = self;
    renderer->enabled = 1;

    self->viewer = viewer;
    
    self->pw = BOT_GTK_PARAM_WIDGET(renderer->widget);
    
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_FULLPLAN, 1, NULL);
     
    bot_gtk_param_widget_add_double (self->pw, PARAM_PLAN_PART,
                                   BOT_GTK_PARAM_WIDGET_SLIDER, 0, 1, 0.005, 1);    
    
    g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    self->selection_enabled = 1;


    self->use_colormap = 1; // default - never changed now
    //bot_gtk_param_widget_set_bool(self->pw, PARAM_USE_COLORMAP,self->use_colormap);
    self->clicked = 0;	
    self->dragging = 0;    
  	self->selection = new std::string(" ");
    self->marker_selection = new std::string(" ");  
    self->trigger_source_otdf_id = new std::string(" "); 
    self->marker_choice_state = HANDS;
    self->is_left_in_motion =  true;
    self->visualize_bbox = false;
    self->multiapprove_plan_execution_dock= NULL; 
    self->plan_execution_dock= NULL; 
    self->ignore_plan_execution_warning = FALSE;
    self->plan_approval_dock= NULL; 
    self->plan_execute_button= NULL; 
    self->afftriggered_popup = NULL;
    self->selected_plan_index= -1;
    self->selected_keyframe_index = -1;
    self->atlas_state = drc::atlas_status_t::BEHAVIOR_STAND;
    self->atlas_status_utime = 0; 
    self->_renderer_foviate = false;
    int plan_size =   self->robotPlanListener->_gl_robot_list.size();
    self->show_fullplan = bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_FULLPLAN);
    double plan_part = bot_gtk_param_widget_get_double(self->pw, PARAM_PLAN_PART);
    if ((self->show_fullplan)||(plan_size==0)){
      self->displayed_plan_index = -1;
    }else{
      uint w_plan = (uint) round(plan_part* (plan_size -1));
      self->displayed_plan_index = w_plan;
    }
  
    //bot_viewer_add_renderer(viewer, &self->renderer, render_priority);
    bot_viewer_add_renderer_on_side(viewer,&self->renderer, render_priority, 0);
        
    BotEventHandler *ehandler = &self->ehandler;
    ehandler->name = (char*) RENDERER_NAME;
    if (operation_mode==1){
      ehandler->name =(char *) "Robot Plan Loopback";
    }else if(operation_mode==2){
      ehandler->name =(char *) "Robot Plan LB Compressed";
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
