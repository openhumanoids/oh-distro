#include "renderer_sticky_feet.hpp"
#include "FootStepPlanListener.hpp" // need parent renderer struct which contains a FootStepPlanListener... circular dependency.
#include "plan_approval_gui_utils.hpp"

#define RENDERER_NAME "FootStep Plans & Sticky Feet"
#define PARAM_AUTO_ADJUST_HT "Auto Adjust Height"
#define PARAM_CLEAR_FOOTSTEP_PLAN "Clear FootSteps"
using namespace std;
using namespace boost;
using namespace renderer_sticky_feet;
using namespace renderer_sticky_feet_gui_utils;


static void
_renderer_free (BotRenderer *super)
{
  RendererStickyFeet *self = (RendererStickyFeet*) super->user;
  delete self->perceptionData;
  free(self);
}
//================================= Drawing


static void 
draw_state(BotViewer *viewer, BotRenderer *super, uint i){

  float c_blue[3] = {0.3,0.3,0.6}; // light blue
  float c_grey[3] = {0.3,0.3,0.3}; // grey
  float c_green[3] = {0.3,0.5,0.3};  // green for right sticky feet
  float c_yellow[3] = {0.5,0.5,0.3}; //yellow for left sticky feet
  float alpha = 0.4;
  RendererStickyFeet *self = (RendererStickyFeet*) super->user;
 
 
 // update stickyfeet z offsets to the support surfaceace if points exist below it. 
 double x,y,z;
 x = self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_T_world_body.p[0];
 y = self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_T_world_body.p[1];
 z = self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_T_world_body.p[2];
 Eigen::Vector3f queryPt(x,y,z);
  //std::cout << "query" << queryPt.transpose()<<" " << z << std::endl;
  double z_surface;
  bool insupport= get_support_surface_height_from_perception(self, queryPt, z_surface);
  
  /*if(insupport && (!isnan(z_surface)))
  { 
     double offset=0;
    if(self->footStepPlanListener->_planned_stickyfeet_info_list[i].foot_type==0)
      offset = self->footStepPlanListener->_T_bodyframe_groundframe_left.p[2];
    else
      offset = self->footStepPlanListener->_T_bodyframe_groundframe_right.p[2];

    // std::cout <<  "footstep height: " << i <<" "<<  z_surface << " " << offset<< std::endl;
 
     KDL::Frame T_worldframe_footframe =  self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_T_world_body;
     T_worldframe_footframe.p[2] = offset+z_surface; // stick to support surface. TODO:: account for offset
     std::map<std::string, double> jointpos_in; 
     jointpos_in =  self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_current_jointpos;
     self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->set_state(T_worldframe_footframe,jointpos_in);          
  }  */
       
    
  if(self->footStepPlanListener->is_motion_copy(i))
  {
    x = self->footStepPlanListener->_gl_in_motion_copy->_T_world_body.p[0];
    y = self->footStepPlanListener->_gl_in_motion_copy->_T_world_body.p[1];
    z = self->footStepPlanListener->_gl_in_motion_copy->_T_world_body.p[2];
    Eigen::Vector3f queryPt(x,y,z);
    bool insupport= get_support_surface_height_from_perception(self, queryPt, z_surface);

    if(insupport && (!isnan(z_surface)))
    {
     double offset = 0;
     if(self->footStepPlanListener->_planned_stickyfeet_info_list[i].foot_type==0)
       offset = self->footStepPlanListener->_T_bodyframe_groundframe_left.p[2];
     else
       offset = self->footStepPlanListener->_T_bodyframe_groundframe_right.p[2];
      KDL::Frame T_worldframe_footframe =  self->footStepPlanListener->_gl_in_motion_copy->_T_world_body;
      //std::cout <<  "motion copy height: " << i <<" "<<T_worldframe_footframe.p[2] <<" "<<  z_surface << " " << offset<< std::endl;

      T_worldframe_footframe.p[2] = z_surface+offset;
      std::map<std::string, double> jointpos_in; 
      jointpos_in =  self->footStepPlanListener->_gl_in_motion_copy->_current_jointpos;  
      self->footStepPlanListener->_gl_in_motion_copy->set_state(T_worldframe_footframe,jointpos_in); 
    }
  }     

  
//  self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->show_bbox(self->visualize_bbox);
//  self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->enable_link_selection(self->ht_auto_adjust_enabled);
string no_selection =  " ";
 if((*self->selection)== self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_unique_name){   
   if((*self->marker_selection)==" ")
    self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->highlight_link((*self->selection));
   else
    self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->highlight_marker((*self->marker_selection));
 }
 else{ 
   self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->highlight_link(no_selection);
   self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->highlight_marker(no_selection);
 }
   
 if((*self->marker_selection)!=" "){ 
    if(self->footStepPlanListener->is_motion_copy(i))
       self->footStepPlanListener->_gl_in_motion_copy->highlight_marker((*self->marker_selection));
 }
 else{ 
   self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->highlight_marker(no_selection);
 }
 
 if(!self->footStepPlanListener->_planned_stickyfeet_info_list[i].is_fixed){
 if(self->footStepPlanListener->_planned_stickyfeet_info_list[i].foot_type == FootStepPlanListener::RIGHT)
    self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->draw_body (c_green,alpha); 
 else
    self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->draw_body (c_yellow,alpha);  
 }
 else {
    self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->draw_body (c_grey,alpha); 
 }   
    
    

 if(self->footStepPlanListener->is_motion_copy(i))
   self->footStepPlanListener->_gl_in_motion_copy->draw_body (c_blue,alpha);

  //self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->draw_whole_body_bbox();
}

static void 
_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
  RendererStickyFeet *self = (RendererStickyFeet*) super->user;

  
  glEnable(GL_DEPTH_TEST);

  //-draw 
  //glPointSize(5.0f);
  //glBegin(GL_POINTS);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

  if((self->ht_auto_adjust_enabled)&&(self->clicked)){
    glLineWidth (3.0);
    glPushMatrix();
    glBegin(GL_LINES);
    glVertex3f(self->ray_start[0], self->ray_start[1],self->ray_start[2]); // object coord
    glVertex3f(self->ray_end[0], self->ray_end[1],self->ray_end[2]);
    glEnd();
    glPopMatrix();
  }

  for(uint i = 0; i < self->footStepPlanListener->_gl_planned_stickyfeet_list.size(); i++) 
  { 
    //cout << "i:"<<i<< endl;
    double pos[3];
    pos[0] = self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_T_world_body.p[0]; 
    pos[1] = self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_T_world_body.p[1]; 
    pos[2] = self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_T_world_body.p[2]+0.0;  
    std::stringstream oss;
    oss << i;
    glColor4f(0,0,0,1);
    bot_gl_draw_text(pos, GLUT_BITMAP_HELVETICA_12, (oss.str()).c_str(),0);
    
    draw_state(viewer,super,i);
  }
    
  if(!self->footStepPlanListener->_last_plan_approved)
  {
    if((self->footStepPlanListener->_gl_planned_stickyfeet_list.size()>0)&&(self->plan_approval_dock==NULL))
        spawn_plan_approval_dock(self);
  }    

}

//========================= Event Handling ================

static double pick_query (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], const double ray_dir[3])
{
  RendererStickyFeet *self = (RendererStickyFeet*) ehandler->user;

  //fprintf(stderr, "RobotStateRenderer Pick Query Active\n");
  Eigen::Vector3f from,to;
  from << ray_start[0], ray_start[1], ray_start[2];

  Eigen::Vector3f plane_normal,plane_pt;
  plane_normal << 0,0,1;
  plane_pt << 0,0,-0.1;
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
  
  double shortest_distance = get_shortest_distance_between_stickyfeet_and_markers(self,from,to);
  
  return shortest_distance;
}

static int mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], const double ray_dir[3], const GdkEventButton *event)
{
  RendererStickyFeet *self = (RendererStickyFeet*) ehandler->user;
  if((ehandler->picking==0)){
    //fprintf(stderr, "Ehandler Not active\n");
   (*self->selection)  = " ";
    return 0;
  }

 // fprintf(stderr, "RobotPlanRenderer Ehandler Activated\n");
  self->clicked = 1;
  //fprintf(stderr, "Mouse Press : %f,%f\n",ray_start[0], ray_start[1]);

  collision::Collision_Object * intersected_object = NULL;
  self->footStepPlanListener->_gl_planned_stickyfeet_list[self->selected_planned_footstep_index]->_collision_detector->ray_test( self->ray_start, self->ray_end, intersected_object );
  if( intersected_object != NULL ){
      //cout << self->selected_planned_footstep_index << endl;  
      //std::cout << "prev selection :" << (*self->selection)  <<  std::endl;
    (*self->selection)  = self->footStepPlanListener->_gl_planned_stickyfeet_list[self->selected_planned_footstep_index]->_unique_name;
     std::cout << "intersected sticky foot:" << (*self->selection) <<  std::endl;
     // self->footStepPlanListener->_gl_planned_stickyfeet_list[self->selected_planned_footstep_index]->highlight_link((*self->selection));
   }
 
 
  if((((*self->selection)  != " ") || ((*self->marker_selection)  != " ")) &&(event->button==1) &&(event->type==GDK_2BUTTON_PRESS))
  {
  
    if((*self->marker_selection)  == " ")// dbl clk on link then toogle
    {     
       bool toggle=true;
       if (self->footStepPlanListener->is_motion_copy(self->selected_planned_footstep_index))
           toggle = !self->footStepPlanListener->_gl_in_motion_copy->is_bodypose_adjustment_enabled();
        self->footStepPlanListener->create_sticky_foot_local_copy(self->selected_planned_footstep_index);
        self->footStepPlanListener->_gl_in_motion_copy->enable_bodypose_adjustment(toggle);   
    }
  
    bot_viewer_request_redraw(self->viewer);
    std::cout << "RendererStickyFeet: Event is consumed" <<  std::endl;
    return 1;// consumed if pop up comes up.
  }
  else if(((*self->marker_selection)  != " "))
  {
    self->dragging = 1;

    KDL::Frame T_world_object;
    T_world_object = self->footStepPlanListener->_gl_in_motion_copy->_T_world_body; 
    self->marker_offset_on_press << self->ray_hit[0]-T_world_object.p[0],self->ray_hit[1]-T_world_object.p[1],self->ray_hit[2]-T_world_object.p[2]; 
    std::cout << "RendererStickyFeet: Event is consumed" <<  std::endl;
    return 1;// consumed
  }
  
  
  bot_viewer_request_redraw(self->viewer);

  return 0;
}


static int 
mouse_release (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
    const double ray_dir[3], const GdkEventButton *event)
{
  RendererStickyFeet *self = (RendererStickyFeet*) ehandler->user;
  self->clicked = 0;
  if((ehandler->picking==0)){
    //fprintf(stderr, "Ehandler Not active\n");
    return 0;
  }
  if (self->dragging) {
    self->dragging = 0;
  }
  if (ehandler->picking==1)
    ehandler->picking=0; //release picking(IMPORTANT)
  bot_viewer_request_redraw(self->viewer);
  return 0;
}


// ----------------------------------------------------------------------------
static int mouse_motion (BotViewer *viewer, BotEventHandler *ehandler,  const double ray_start[3], const double ray_dir[3],   const GdkEventMotion *event)
{
  RendererStickyFeet *self = (RendererStickyFeet*) ehandler->user;
  
  if((!self->dragging)||(ehandler->picking==0)){
    return 0;
  }
  
  if((*self->marker_selection)  != " "){
    double t = self->ray_hit_t;
    self->ray_hit_drag << ray_start[0]+t*ray_dir[0], ray_start[1]+t*ray_dir[1], ray_start[2]+t*ray_dir[2];
    set_object_desired_state_on_marker_motion(self);
  }
  bot_viewer_request_redraw(self->viewer);
  return 1;
}


static void onRobotUtime (const lcm_recv_buf_t * buf, const char *channel, 
                               const drc_utime_t *msg, void *user){
  RendererStickyFeet *self = (RendererStickyFeet*) user;
  self->robot_utime = msg->utime;
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererStickyFeet *self = (RendererStickyFeet*) user;
  if (! strcmp(name, PARAM_AUTO_ADJUST_HT)) {
    if (bot_gtk_param_widget_get_bool(pw, PARAM_AUTO_ADJUST_HT)) {
      self->ht_auto_adjust_enabled = 1;
      // cout << "TO BE IMPLEMENTED" << endl;
    }
    else{
      self->ht_auto_adjust_enabled = 0;
    }
  }
  else if(!strcmp(name, PARAM_CLEAR_FOOTSTEP_PLAN))
  {
    self->footStepPlanListener->_gl_planned_stickyfeet_list.clear();
  }

  bot_viewer_request_redraw(self->viewer);
  
}

void 
setup_renderer_sticky_feet(BotViewer *viewer, int render_priority, lcm_t *lcm, BotParam * param,
    BotFrames * frames)
{
    RendererStickyFeet *self = (RendererStickyFeet*) calloc (1, sizeof (RendererStickyFeet));
    self->lcm = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcm));
    
    self->perceptionData = new PerceptionData();
    self->perceptionData->mBotWrapper.reset(new maps::BotWrapper(lcm,param,frames));
    self->perceptionData->mViewClient.setBotWrapper(self->perceptionData->mBotWrapper);
    self->perceptionData->mViewClient.start();
      
    
   // self->footStepPlanListener = boost::shared_ptr<FootStepPlanListener>(new FootStepPlanListener(self));
    self->footStepPlanListener = boost::shared_ptr<FootStepPlanListener>(new FootStepPlanListener(self->lcm,viewer));
    BotRenderer *renderer = &self->renderer;

    renderer->draw = _renderer_draw;
    renderer->destroy = _renderer_free;

    renderer->widget = bot_gtk_param_widget_new();
    renderer->name = (char *) RENDERER_NAME;
    renderer->user = self;
    renderer->enabled = 1;

    self->viewer = viewer;

    self->pw = BOT_GTK_PARAM_WIDGET(renderer->widget);
    
    // C-style subscribe:
    drc_utime_t_subscribe(self->lcm->getUnderlyingLCM(),"ROBOT_UTIME",onRobotUtime,self); 

    bot_gtk_param_widget_add_buttons(self->pw, PARAM_CLEAR_FOOTSTEP_PLAN, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_AUTO_ADJUST_HT, 0, NULL);
//    bot_gtk_param_widget_add_buttons(self->pw, PARAM_START_PLAN, NULL);
//    bot_gtk_param_widget_add_buttons(self->pw, PARAM_SEND_COMMITTED_PLAN, NULL);

    self->ht_auto_adjust_enabled = 1;
  	bot_gtk_param_widget_set_bool(self->pw, PARAM_AUTO_ADJUST_HT,self->ht_auto_adjust_enabled);
  	
  	g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    self->clicked = 0;	
    self->dragging = 0;	
  	self->selection = new std::string(" ");
    self->marker_selection = new std::string(" ");
    //bot_viewer_add_renderer(viewer, &self->renderer, render_priority);
    bot_viewer_add_renderer_on_side(viewer,&self->renderer, render_priority, 0);
        
    BotEventHandler *ehandler = &self->ehandler;
    ehandler->name = (char*) RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->pick_query = pick_query;
    ehandler->hover_query = NULL;
    ehandler->mouse_press = mouse_press;
    ehandler->mouse_release = mouse_release;
    ehandler->mouse_motion = mouse_motion;
    ehandler->user = self;

    bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);
    
}

