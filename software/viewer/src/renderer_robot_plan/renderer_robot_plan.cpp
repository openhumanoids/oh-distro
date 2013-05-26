
#include "renderer_robot_plan.hpp"
#include "RobotPlanListener.hpp"
#include "plan_execution_gui_utils.hpp" 
#include "plan_approval_gui_utils.hpp"


#define RENDERER_NAME "Robot Plan Display"
#define PARAM_SELECTION "Enable Selection"
#define PARAM_WIRE "Show BBoxs For Meshes"  
#define PARAM_HIDE "Hide Plan"  
#define PARAM_USE_COLORMAP "Use Colormap"
#define PARAM_PLAN_PART "Part of Plan"  
#define DRAW_PERSIST_SEC 4
#define PARAM_START_PLAN "Start Planning"
#define PARAM_SEND_COMMITTED_PLAN "Send Plan"
#define PARAM_NEW_VICON_PLAN "Get Vicon Plan"
#define PARAM_ADJUST_ENDSTATE "Adjust end keyframe"
#define PARAM_SHOW_FULLPLAN "Show Full Plan"	
#define PARAM_SHOW_KEYFRAMES "Show Keyframes"

using namespace std;
using namespace boost;
using namespace renderer_robot_plan;
using namespace renderer_robot_plan_gui_utils;

static void
_renderer_free (BotRenderer *super)
{
  RendererRobotPlan *self = (RendererRobotPlan*) super->user;
  free(self);
}


//=================================


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
draw_keyframe(BotViewer *viewer, BotRenderer *super, uint i){

  float c_blue[3] = {0.3,0.3,0.6}; // light blue
  float alpha = 0.7;
  RendererRobotPlan *self = (RendererRobotPlan*) super->user;
  float c[3];
  if((self->use_colormap)) {
    // Each model Jet: blue to red
    float j = (float)i/ (self->robotPlanListener->_gl_robot_keyframe_list.size() -1);
    jet_rgb(j,c);
  }
  else{
   c[0] = c_blue[0];c[1] = c_blue[1];c[2] = c_blue[2];
  }
  
  if(self->robotPlanListener->_gl_left_hand->is_bodypose_adjustment_enabled())
    alpha = 0.2;
  
  glColor4f(c[0],c[1],c[2], alpha);
  
  //self->robotPlanListener->_gl_robot_keyframe_list[i]->enable_link_selection(self->selection_enabled);
  
	std::string selected_keyframe_name = " ";
	//if(self->selected_keyframe_index!=-1)
	if((*self->selection)  != " ")
	 selected_keyframe_name = self->robotPlanListener->_gl_robot_keyframe_list[self->selected_keyframe_index]->_unique_name; 
  self->robotPlanListener->_gl_robot_keyframe_list[i]->highlight_body(selected_keyframe_name);
  self->robotPlanListener->_gl_robot_keyframe_list[i]->draw_body(c,alpha);  
  
     
 if ((self->robotPlanListener->is_in_motion(i))&&(self->robotPlanListener->_gl_left_hand->is_bodypose_adjustment_enabled())) 
 {
  alpha = 0.9;
  self->robotPlanListener->_gl_left_hand->draw_body(c_blue,alpha);
  self->robotPlanListener->_gl_right_hand->draw_body(c_blue,alpha);
  self->robotPlanListener->_gl_left_foot->draw_body(c_blue,alpha);
  self->robotPlanListener->_gl_right_foot->draw_body(c_blue,alpha);
 } 
  
}

static void 
_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
  RendererRobotPlan *self = (RendererRobotPlan*) super->user;
  // if hide is enabled - then dont draw the plan:
  if (bot_gtk_param_widget_get_bool(self->pw, PARAM_HIDE)) {
     return;
  }
  
  glEnable(GL_DEPTH_TEST);

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
  if (plan_size ==0) // nothing to renderer
    return;
    

  
  
   // Show keyframes
  if((self->show_keyframes)&&(self->robotPlanListener->_is_manip_plan)&&(self->robotPlanListener->_gl_robot_keyframe_list.size()>0))
  {
    for(uint i = 1; i < self->robotPlanListener->_gl_robot_keyframe_list.size(); i++) 
    { 
        draw_keyframe(viewer,super,i);
    }
  }      
  
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
      draw_state(viewer,super,i,c);
    }
    self->displayed_plan_index = -1;    
  }else{
    double plan_part = bot_gtk_param_widget_get_double(self->pw, PARAM_PLAN_PART);
    uint w_plan = (uint) round(plan_part* (plan_size -1));
    //printf("                                  Show around %f of %d    %d\n", plan_part, plan_size, w_plan);
    self->displayed_plan_index = w_plan;
    
    float c[3] = {0.3,0.3,0.6}; // light blue
    draw_state(viewer,super,w_plan,c);
  }
  
  
  if(self->robotPlanListener->_controller_status == drc::controller_status_t::WALKING){ // walking 
    int rx_plan_size = self->robotPlanListener->_received_plan.num_states;
    int64_t last_plan_utime = self->robotPlanListener->_received_plan.plan[rx_plan_size-1].utime;
    double current_plan_part = ((double) self->robotPlanListener->_controller_utime / last_plan_utime);
    
    //printf ("controller time: %lld \n", self->robotPlanListener->_controller_utime); 
    //std::cout << self->robotPlanListener->_received_plan.num_states << " is rxd plan size\n";
    //std::cout << plan_size << " is plan size\n";
    //std::cout << last_plan_utime << " is last_plan_utime\n";
    //std::cout << current_plan_part << " is current_plan_part\n";    
    if((current_plan_part >0.0 )&&(current_plan_part <1.0)){
      double plan_part = bot_gtk_param_widget_get_double(self->pw, PARAM_PLAN_PART);
      uint w_plan = (uint) round(current_plan_part* (plan_size -1));
      //printf("                                  Show around %f of %d    %d\n", plan_part, plan_size, w_plan);
      self->displayed_plan_index = w_plan;
      
      float c[3] = {0.6,0.3,0.3}; // light red
      draw_state(viewer,super,w_plan,c);
    }
  }else{
   std::cout << "not walking\n"; 
  }

 if((self->plan_execution_dock==NULL)&&(!self->robotPlanListener->_is_manip_map))
      spawn_plan_execution_dock(self);

 if((self->plan_approval_dock==NULL)&&(self->robotPlanListener->_is_manip_map))
    spawn_plan_approval_dock (self);
}


// temporary method: will be replaced later
  static void publish_eegoal_to_start_planning(boost::shared_ptr<lcm::LCM> &_lcm, std::string channel)
  {
    drc::ee_goal_t goalmsg;
    goalmsg.robot_name = "atlas";
    goalmsg.root_name = "pelvis";
    goalmsg.ee_name = "ee_plan_start";
    
    double x,y,z,w;
    // desired ee position in world frame
    KDL::Frame T_body_ee;
    T_body_ee = KDL::Frame::Identity();; // send them in world frame for now.

    goalmsg.ee_goal_pos.translation.x = T_body_ee.p[0];
    goalmsg.ee_goal_pos.translation.y = T_body_ee.p[1];
    goalmsg.ee_goal_pos.translation.z = T_body_ee.p[2];

    goalmsg.ee_goal_pos.rotation.x = 0;
    goalmsg.ee_goal_pos.rotation.y = 0;
    goalmsg.ee_goal_pos.rotation.z = 0;
    goalmsg.ee_goal_pos.rotation.w = 1;

    goalmsg.ee_goal_twist.linear_velocity.x = 0.0;
    goalmsg.ee_goal_twist.linear_velocity.y = 0.0;
    goalmsg.ee_goal_twist.linear_velocity.z = 0.0;
    goalmsg.ee_goal_twist.angular_velocity.x = 0.0;
    goalmsg.ee_goal_twist.angular_velocity.y = 0.0;
    goalmsg.ee_goal_twist.angular_velocity.z = 0.0;

    goalmsg.num_chain_joints  =0;
    // No specified posture bias
    goalmsg.use_posture_bias  = false;
    goalmsg.joint_posture_bias.resize(goalmsg.num_chain_joints);
    goalmsg.chain_joint_names.resize(goalmsg.num_chain_joints);
    for(int i = 0; i < goalmsg.num_chain_joints; i++){
    goalmsg.joint_posture_bias[i]=0;
    goalmsg.chain_joint_names[i]= " ";
    }

    // Publish the message
    goalmsg.halt_ee_controller = false;

    _lcm->publish(channel, &goalmsg);
  }

//========================= Event Handling ================

static double pick_query (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], const double ray_dir[3])
{
  RendererRobotPlan *self = (RendererRobotPlan*) ehandler->user;
  if((self->selection_enabled==0)||(bot_gtk_param_widget_get_bool(self->pw, PARAM_HIDE))){
    return -1.0;
  }
  //fprintf(stderr, "RobotStateRenderer Pick Query Active\n");
  Eigen::Vector3f from,to;
  from << ray_start[0], ray_start[1], ray_start[2];

  Eigen::Vector3f plane_normal,plane_pt;
  plane_normal << 0,0,1;
  plane_pt << 0,0,0;
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
  
  //

  Eigen::Vector3f hit_pt;
  collision::Collision_Object * intersected_object = NULL;
  double shortest_distance = -1;
  self->selected_plan_index= 0;
  
  
  if(self->robotPlanListener->_is_manip_plan)  {
    shortest_distance = get_shortest_distance_between_keyframes_and_markers(self,from,to);
  }
  else   {
    shortest_distance = get_shortest_distance_from_a_plan_frame(self,from,to);
  }

  //std::cout  << "RobotStateRenderer distance " << -1.0 << std::endl;
  return shortest_distance;
}

static int mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], const double ray_dir[3], const GdkEventButton *event)
{
  RendererRobotPlan *self = (RendererRobotPlan*) ehandler->user;
  if((ehandler->picking==0)||(self->selection_enabled==0)){
    //fprintf(stderr, "Ehandler Not active\n");
   (*self->selection)  = " ";
    return 0;
  }
 // fprintf(stderr, "RobotPlanRenderer Ehandler Activated\n");
  self->clicked = 1;
  //fprintf(stderr, "Mouse Press : %f,%f\n",ray_start[0], ray_start[1]);
  collision::Collision_Object * intersected_object = NULL;
   if((self->robotPlanListener->_is_manip_plan)&&(self->selected_keyframe_index!=-1)&&(self->robotPlanListener->_gl_robot_keyframe_list.size()>0))
   {
   // cout << "keyframe: " << self->selected_keyframe_index << " " << self->robotPlanListener->_gl_robot_keyframe_list.size()<< endl;
    self->robotPlanListener->_gl_robot_keyframe_list[self->selected_keyframe_index]->_collision_detector->ray_test( self->ray_start, self->ray_end, intersected_object ); 
    if( intersected_object != NULL ){
        std::cout << "prev selection :" << (*self->selection)  <<  std::endl;
        std::cout << "intersected :" << intersected_object->id().c_str() <<  std::endl;
        (*self->selection)  = std::string(intersected_object->id().c_str());
        
	      std::string body_name = self->robotPlanListener->_gl_robot_keyframe_list[self->selected_keyframe_index]->_unique_name; 
        self->robotPlanListener->_gl_robot_keyframe_list[self->selected_keyframe_index]->highlight_body(body_name);
        //self->robotPlanListener->_gl_robot_keyframe_list[self->selected_keyframe_index]->highlight_link((*self->selection));
     }

   }
   else{
    self->robotPlanListener->_gl_robot_list[self->selected_plan_index]->_collision_detector->ray_test( self->ray_start, self->ray_end, intersected_object );
    if( intersected_object != NULL ){
        std::cout << "prev selection :" << (*self->selection)  <<  std::endl;
        std::cout << "intersected :" << intersected_object->id().c_str() <<  std::endl;
        (*self->selection)  = std::string(intersected_object->id().c_str());
        self->robotPlanListener->_gl_robot_list[self->selected_plan_index]->highlight_link((*self->selection));
     }
   }

  if((self->robotPlanListener->_is_manip_plan)&&(((*self->selection)  != " ") || ((*self->marker_selection)  != " "))&&(event->button==1) &&(event->type==GDK_2BUTTON_PRESS))
  {

   if((*self->marker_selection)  == " ")
    cout << "DblClk: " << (*self->selection) << endl;
   else
    cout << "DblClk on Marker: " << (*self->marker_selection) << endl;
   /* if((*self->marker_selection)  == " ")// dbl clk on keyframe then toogle
    { */
      bool toggle=true;
      if (self->robotPlanListener->is_in_motion(self->selected_keyframe_index)){
         toggle = !self->robotPlanListener->_gl_left_hand->is_bodypose_adjustment_enabled();
      }
      self->robotPlanListener->set_in_motion_hands_state(self->selected_keyframe_index);
      self->robotPlanListener->_gl_left_hand->enable_bodypose_adjustment(toggle); 
      self->robotPlanListener->_gl_right_hand->enable_bodypose_adjustment(toggle);
      self->robotPlanListener->set_in_motion_feet_state(self->selected_keyframe_index);
      self->robotPlanListener->_gl_left_foot->enable_bodypose_adjustment(toggle); 
      self->robotPlanListener->_gl_right_foot->enable_bodypose_adjustment(toggle);     
      
      if(!toggle){
        (*self->marker_selection)  = " ";
      }
   //}
        // On double click create/toggle  local copies of right and left sticky hand duplicates and spawn them with markers
   return 1;// consumed if pop up comes up.    
  }
  else if(((*self->marker_selection)  != " "))
  {
    self->dragging = 1;
    
    KDL::Frame T_world_ee;
    if(self->is_hand_in_motion){
      if(self->is_left_in_motion) {
        T_world_ee = self->robotPlanListener->_gl_left_hand->_T_world_body; 
      }
      else {
        T_world_ee = self->robotPlanListener->_gl_right_hand->_T_world_body; 
      }
    }
    else{
       if(self->is_left_in_motion) {
        T_world_ee = self->robotPlanListener->_gl_left_foot->_T_world_body; 
      }
      else {
        T_world_ee = self->robotPlanListener->_gl_right_foot->_T_world_body; 
      }
    }
    self->marker_offset_on_press << self->ray_hit[0]-T_world_ee.p[0],self->ray_hit[1]-T_world_ee.p[1],self->ray_hit[2]-T_world_ee.p[2]; 
    std::cout << "RendererRobotPlan: Event is consumed" <<  std::endl;
    return 1;// consumed
  }

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
    if(((*self->marker_selection)  != " ")&&(movement>=1e-3)){
    //cout << "publishing manip_plan_constraint \n";
    publish_traj_opt_constraint(self,channel,self->selected_keyframe_index);
    }
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
    adjust_keyframe_on_marker_motion(self);
    // cout << (*self->marker_selection) << ": mouse drag\n";
  }
  bot_viewer_request_redraw(self->viewer);
  return 1;
}

// ------------------------END Event Handling-------------------------------------------

static void onRobotUtime (const lcm_recv_buf_t * buf, const char *channel, 
                               const drc_utime_t *msg, void *user){
  RendererRobotPlan *self = (RendererRobotPlan*) user;
  self->robot_utime = msg->utime;
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererRobotPlan *self = (RendererRobotPlan*) user;
  if (! strcmp(name, PARAM_SELECTION)) {
    self->selection_enabled = bot_gtk_param_widget_get_bool(pw, PARAM_SELECTION);
  }  else if(! strcmp(name, PARAM_WIRE)) {
    self->visualize_bbox = bot_gtk_param_widget_get_bool(pw, PARAM_WIRE);
  }  else if(! strcmp(name,PARAM_USE_COLORMAP)) {
    self->use_colormap	= bot_gtk_param_widget_get_bool(pw, PARAM_USE_COLORMAP);
  }  else if(! strcmp(name,PARAM_USE_COLORMAP)) {
    self->use_colormap= bot_gtk_param_widget_get_bool(pw, PARAM_USE_COLORMAP);
  }  else if(! strcmp(name,PARAM_ADJUST_ENDSTATE)) {
    self->adjust_endstate = bot_gtk_param_widget_get_bool(pw, PARAM_ADJUST_ENDSTATE);
  }  else if(! strcmp(name,PARAM_SHOW_FULLPLAN)) {
    self->show_fullplan = bot_gtk_param_widget_get_bool(pw, PARAM_SHOW_FULLPLAN);
  }  else if(! strcmp(name,PARAM_SHOW_KEYFRAMES)) {
    self->show_keyframes = bot_gtk_param_widget_get_bool(pw, PARAM_SHOW_KEYFRAMES);
  }else if(!strcmp(name,PARAM_START_PLAN)){
    publish_eegoal_to_start_planning(self->lcm,"EE_PLAN_START");
  }else if(!strcmp(name,PARAM_SEND_COMMITTED_PLAN)){
    self->lcm->publish("COMMITTED_ROBOT_PLAN", &(self->robotPlanListener->_received_plan) );
  }else if(! strcmp(name, PARAM_NEW_VICON_PLAN)) {
    drc::plan_collect_t msg;
    msg.utime = self->robot_utime;//bot_timestamp_now();
    msg.type = self->vicon_type;
    msg.n_plan_samples = self->vicon_n_plan_samples;
    msg.sample_period = self->vicon_sample_period;
    self->lcm->publish("VICON_GET_PLAN", &msg);
    bot_viewer_set_status_bar_message(self->viewer, "Sent VICON_GET_PLAN [nsamples: %d, period %fsec] @ %lld",msg.n_plan_samples, msg.sample_period, msg.utime);    
  }

  bot_viewer_request_redraw(self->viewer);
  
}

void 
setup_renderer_robot_plan(BotViewer *viewer, int render_priority, lcm_t *lcm)
{
    RendererRobotPlan *self = (RendererRobotPlan*) calloc (1, sizeof (RendererRobotPlan));
    self->lcm = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcm));
    self->robotPlanListener = boost::shared_ptr<RobotPlanListener>(new RobotPlanListener(self->lcm, 
												    viewer));

    BotRenderer *renderer = &self->renderer;

    renderer->draw = _renderer_draw;
    renderer->destroy = _renderer_free;

    renderer->widget = bot_gtk_param_widget_new();
    renderer->name = (char *) RENDERER_NAME;
    renderer->user = self;
    renderer->enabled = 1;

    self->viewer = viewer;
    
    // default Vicon plan sample values:
    self->vicon_n_plan_samples = 20;
    self->vicon_sample_period = 0.5;

    self->pw = BOT_GTK_PARAM_WIDGET(renderer->widget);
    
    // C-style subscribe:
    drc_utime_t_subscribe(self->lcm->getUnderlyingLCM(),"ROBOT_UTIME",onRobotUtime,self); 

    
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SELECTION, 0, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_WIRE, 0, NULL);

    bot_gtk_param_widget_add_buttons(self->pw, PARAM_NEW_VICON_PLAN, NULL);
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_START_PLAN, NULL);
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_SEND_COMMITTED_PLAN, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_HIDE, 0, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_USE_COLORMAP, 0, NULL);
    self->adjust_endstate = false;
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_ADJUST_ENDSTATE, 0, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_FULLPLAN, 0, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_KEYFRAMES, 1, NULL);
    
    bot_gtk_param_widget_add_double (self->pw, PARAM_PLAN_PART,
                                   BOT_GTK_PARAM_WIDGET_SLIDER, 0, 1, 0.005, 1);    
   
  	g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  	self->selection_enabled = 1;
  	bot_gtk_param_widget_set_bool(self->pw, PARAM_SELECTION,self->selection_enabled);
  	self->use_colormap = 1;
  	bot_gtk_param_widget_set_bool(self->pw, PARAM_USE_COLORMAP,self->use_colormap);
    self->clicked = 0;	
    self->dragging = 0;    
  	self->selection = new std::string(" ");
    self->marker_selection = new std::string(" ");  	
    self->is_left_in_motion =  true;
    self->is_hand_in_motion =  true;
    self->visualize_bbox = false;
    
    int plan_size =   self->robotPlanListener->_gl_robot_list.size();
    self->show_fullplan = bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_FULLPLAN);
    self->show_keyframes = bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_KEYFRAMES);
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
    ehandler->enabled = 1;
    ehandler->pick_query = pick_query;
    ehandler->hover_query = NULL;
    ehandler->mouse_press = mouse_press;
    ehandler->mouse_release = mouse_release;
    ehandler->mouse_motion = mouse_motion;
    ehandler->user = self;

    bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);
   
}

