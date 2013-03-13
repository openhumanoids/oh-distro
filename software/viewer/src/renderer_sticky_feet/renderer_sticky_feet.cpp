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
  free(self);
}
//================================= Drawing


static void 
draw_state(BotViewer *viewer, BotRenderer *super, uint i){

  float c_blue[3] = {0.3,0.3,0.6}; // light blue
  float c_green[3] = {0.3,0.5,0.3};  // green for right sticky feet
  float c_yellow[3] = {0.5,0.5,0.3}; //yellow for left sticky feet
  float alpha = 0.4;
  RendererStickyFeet *self = (RendererStickyFeet*) super->user;
 
  
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
 
   if(!self->adjust_via_localcopy)
     self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->highlight_marker((*self->marker_selection));
   else{ 
    if(self->footStepPlanListener->_gl_on_motion_copy)
     self->footStepPlanListener->_gl_on_motion_copy->highlight_marker((*self->marker_selection));
    }
 }
 else{ 
   self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->highlight_marker(no_selection);
 }
 
 if(self->footStepPlanListener->_planned_stickyfeet_info_list[i] == FootStepPlanListener::RIGHT)
    self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->draw_body (c_green,alpha); 
 else
    self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->draw_body (c_yellow,alpha);  
    
 if(self->adjust_via_localcopy) 
 {    
    if(self->footStepPlanListener->_gl_on_motion_copy)
      self->footStepPlanListener->_gl_on_motion_copy->draw_body (c_blue,alpha);
 }
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

    //printf("Show it all\n");
    for(uint i = 0; i < self->footStepPlanListener->_gl_planned_stickyfeet_list.size(); i++) 
    { 
      //cout << "i:"<<i<< endl;
      draw_state(viewer,super,i);
    }
    
   if(!self->footStepPlanListener->_last_plan_approved)
   {
      if((self->footStepPlanListener->_gl_planned_stickyfeet_list.size()>0)&&(self->plan_approval_dock==NULL))
          spawn_plan_approval_dock(self);
//      else if((self->footStepPlanListener->_gl_planned_stickyfeet_list.size()>0)&&(self->plan_approval_dock!=NULL))
//      {
//          // move dock to account for viewer movement
//          gint root_x, root_y;
//          gtk_window_get_position (GTK_WINDOW(self->viewer->window), &root_x, &root_y);
//          gint width, height;
//          gtk_window_get_size(GTK_WINDOW(self->viewer->window),&width,&height);
//          
//          
//           gint pos_x, pos_y;
//          pos_x=root_x+0.5*width;    pos_y=root_y+0.25*height;
//          
//         gint current_pos_x, current_pos_y;
//         if((fabs(current_pos_x-pos_x)+fabs(current_pos_y-pos_y))>1)
//              gtk_window_move(GTK_WINDOW(self->plan_approval_dock),pos_x,pos_y);
//         
//      }
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
      std::cout << "prev selection :" << (*self->selection)  <<  std::endl;
    (*self->selection)  = self->footStepPlanListener->_gl_planned_stickyfeet_list[self->selected_planned_footstep_index]->_unique_name;
     std::cout << "intersected sticky foot:" << (*self->selection) <<  std::endl;
     // self->footStepPlanListener->_gl_planned_stickyfeet_list[self->selected_planned_footstep_index]->highlight_link((*self->selection));
   }
 
 
  if((((*self->selection)  != " ") || ((*self->marker_selection)  != " ")) &&(event->button==1) &&(event->type==GDK_2BUTTON_PRESS))
  {
    //spawn_object_geometry_dblclk_popup(self);
    
    if(!self->adjust_via_localcopy) {
      bool toggle = !self->footStepPlanListener->_gl_planned_stickyfeet_list[self->selected_planned_footstep_index]->is_bodypose_adjustment_enabled();
      self->footStepPlanListener->_gl_planned_stickyfeet_list[self->selected_planned_footstep_index]->enable_bodypose_adjustment(toggle);
    }
    else{
    if((*self->marker_selection)  == " ")
    {
       bool toggle=true;
       int old_ind=self->footStepPlanListener->on_motion_footstep_index;
        if((self->footStepPlanListener->_gl_on_motion_copy)&&(self->selected_planned_footstep_index==old_ind))
           toggle = !self->footStepPlanListener->_gl_on_motion_copy->is_bodypose_adjustment_enabled();
        self->footStepPlanListener->create_sticky_foot_local_copy(self->selected_planned_footstep_index);
        self->footStepPlanListener->_gl_on_motion_copy->enable_bodypose_adjustment(toggle);   
      }
    }
    bot_viewer_request_redraw(self->viewer);
    std::cout << "RendererStickyFeet: Event is consumed" <<  std::endl;
    return 1;// consumed if pop up comes up.
  }
  else if(((*self->marker_selection)  != " "))
  {
    self->dragging = 1;

    KDL::Frame T_world_object;
    
    if(!self->adjust_via_localcopy) 
      T_world_object = self->footStepPlanListener->_gl_planned_stickyfeet_list[self->selected_planned_footstep_index]->_T_world_body;
    else
      T_world_object = self->footStepPlanListener->_gl_on_motion_copy->_T_world_body; 
     
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

    if(!self->adjust_via_localcopy) {
      string channel = "FOOTSTEP_PLAN_CONSTRAINT";
      uint i =  self->selected_planned_footstep_index;
      publish_traj_constraint(self,i,channel);
    }
    //clear on release
    self->footStepPlanListener->on_motion_footstep_index=-1;
    self->footStepPlanListener->on_motion_footstep_utime= 0;
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
		if(!self->adjust_via_localcopy) 
		{
			set_object_desired_state_on_marker_motion(self);  
			// make markers persistent across multiple plans for moving footsteps seamlessly while plans update
			self->footStepPlanListener->on_motion_footstep_index=self->selected_planned_footstep_index; 
			self->footStepPlanListener->on_motion_footstep_utime= self->footStepPlanListener->_gl_planned_stickyfeet_timestamps[self->selected_planned_footstep_index];
		}
		else {
			set_object_desired_state_on_marker_motion_via_local_copy(self);
		}
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
setup_renderer_sticky_feet(BotViewer *viewer, int render_priority, lcm_t *lcm)
{
    RendererStickyFeet *self = (RendererStickyFeet*) calloc (1, sizeof (RendererStickyFeet));
    self->lcm = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcm));
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
    self->adjust_via_localcopy = 1;
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

