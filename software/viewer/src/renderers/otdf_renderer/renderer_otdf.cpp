#include "renderer_otdf.hpp"
#include "AffordanceCollectionListener.hpp"
#include "RobotStateListener.hpp"

using namespace otdf_renderer;

////////////////// THE FOLLOWING CODE WAS COPIED IN HERE TO AVOID
////////////////// DEPENDENCY WITH THE COMMON_UTILS/GEOM_UTILS POD [MFALLON]
#define GEOM_EPSILON 1e-9

using namespace std;



namespace otdf_renderer{

int geom_ray_plane_intersect_3d (const point3d_t *ray_point, const vec3d_t *ray_dir,
    const point3d_t *plane_point, const vec3d_t *plane_normal,
    point3d_t *result, double *u)
{
  double lambda1 = ray_dir->x * plane_normal->x +
      ray_dir->y * plane_normal->y +
      ray_dir->z * plane_normal->z;

  // check for degenerate case where ray is (more or less) parallel to plane
  if (fabs (lambda1) < GEOM_EPSILON) return 0;

  double lambda2 = (plane_point->x - ray_point->x) * plane_normal->x +
      (plane_point->y - ray_point->y) * plane_normal->y +
      (plane_point->z - ray_point->z) * plane_normal->z;
  double v = lambda2 / lambda1;
  result->x = ray_point->x + v * ray_dir->x;
  result->y = ray_point->y + v * ray_dir->y;
  result->z = ray_point->z + v * ray_dir->z;
  if (u) *u = v;
  return 1;
}

int geom_ray_z_plane_intersect_3d(const point3d_t *ray_point, 
    const point3d_t *ray_dir, double plane_z, point2d_t *result_xy)
{
  point3d_t plane_pt = { 0, 0, plane_z};
  point3d_t plane_normal = { 0, 0, 1};
  point3d_t plane_isect_point;
  double plane_point_dist;
  if (!geom_ray_plane_intersect_3d (ray_point, ray_dir, &plane_pt,
      &plane_normal, &plane_isect_point, &plane_point_dist) ||
      plane_point_dist <= 0) {
    return -1;
  }
  result_xy->x = plane_isect_point.x;
  result_xy->y = plane_isect_point.y;
  return 0;
}

// this function should go into otdf_utils library
int get_OTDF_filenames_from_dir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
      std::string fn =string(dirp->d_name);
      if(fn.substr(fn.find_last_of(".") + 1) == "otdf") 
        files.push_back(fn.substr(0,fn.find_last_of(".")));
    }
    closedir(dp);
    return 0;
}

}//end naemspace

////////////////////////////// END OF CODE COPIED IN FROM COMMON_UTILS


//-----------------------------------
static void link_draw(boost::shared_ptr<otdf::Geometry> link, const drc::link_transform_t &nextTf)
{

  //--get rotation in angle/axis form
  double theta;
  double axis[3];
  double quat[4] = {nextTf.tf.rotation.w,
		    nextTf.tf.rotation.x,
		    nextTf.tf.rotation.y,
		    nextTf.tf.rotation.z};
  bot_quat_to_angle_axis(quat, &theta, axis);

  
 GLUquadricObj* quadric = gluNewQuadric();
 gluQuadricDrawStyle(quadric, GLU_FILL);
 gluQuadricNormals(quadric, GLU_SMOOTH);
 gluQuadricOrientation(quadric, GLU_OUTSIDE);
  

  int type = link->type ;
  enum {SPHERE, BOX, CYLINDER, MESH, TORUS}; 
  
  if (type == SPHERE)
    {
  glPushMatrix();
      boost::shared_ptr<otdf::Sphere> sphere(boost::shared_dynamic_cast<otdf::Sphere>(link));	
      double radius = sphere->radius;
      glPointSize(radius);
      //glColor3ub(0,1,0);
      glBegin(GL_POINTS);
      glVertex3f(radius, radius, radius);
      glEnd();
  glPopMatrix();
    }
  else if  (type == BOX)
    {
    boost::shared_ptr<otdf::Box> box(boost::shared_dynamic_cast<otdf::Box>(link));
    double xDim = box->dim.x;
    double yDim = box->dim.y;
    double zDim = box->dim.z;
  //todo
    glPushMatrix();
        //size cuboid
    
      // move base up so that bottom face is at origin
     glTranslatef(nextTf.tf.translation.x,
 	 	nextTf.tf.translation.y,
  		nextTf.tf.translation.z);

     glRotatef(theta * 180/3.141592654, 
       	 axis[0], axis[1], axis[2]); 
     glScalef(xDim,yDim,zDim);
     bot_gl_draw_cube();
        //cube();
    glPopMatrix();
  

  }
  else if  (type == CYLINDER)
    {
    boost::shared_ptr<otdf::Cylinder> cyl(boost::shared_dynamic_cast<otdf::Cylinder>(link));

     glPushMatrix();
     double v[] = {0,0, -cyl->length/2.0};
     double result[3];
     bot_quat_rotate_to(quat,v,result);

   // Translate tf origin to cylinder centre
     glTranslatef(result[0],result[1],result[2]); 

     glTranslatef(nextTf.tf.translation.x,
		   nextTf.tf.translation.y,
		   nextTf.tf.translation.z);
 
     glRotatef(theta * 180/3.141592654, 
       		axis[0], axis[1], axis[2]); 

       gluCylinder(quadric,
		  cyl->radius,
		  cyl->radius,
		  (double) cyl->length,
		  36,
		  1);

  //gluDeleteQuadric(quadric);
  glPopMatrix();

// drawing two disks to make a SOLID cylinder
  glPushMatrix();  

  v[2] = -(cyl->length/2.0);
  bot_quat_rotate_to(quat,v,result);

   // Translate tf origin to cylinder centre
     glTranslatef(result[0],result[1],result[2]); 
     glTranslatef(nextTf.tf.translation.x,
		   nextTf.tf.translation.y,
		   nextTf.tf.translation.z);
     glRotatef(theta * 180/3.141592654, 
       		axis[0], axis[1], axis[2]); 
     gluDisk(quadric,
		  0,
		  cyl->radius,
		  36,
		  1);
  glPopMatrix();
  glPushMatrix(); 

 v[2] = (cyl->length/2.0);
  bot_quat_rotate_to(quat,v,result);

   // Translate tf origin to cylinder centre
     glTranslatef(result[0],result[1],result[2]); 
     glTranslatef(nextTf.tf.translation.x,
		   nextTf.tf.translation.y,
		   nextTf.tf.translation.z);
     glRotatef(theta * 180/3.141592654, 
       		axis[0], axis[1], axis[2]); 
     gluDisk(quadric,
		  0,
		  cyl->radius,
		  36,
		  1);
  glPopMatrix();

  }
  else if  (type == MESH)
    {
    //cout << "MESH"<< endl;
    //boost::shared_ptr<otdf::Mesh> mesh(boost::shared_dynamic_cast<otdf::Mesh>(it->second->visual->geometry));
    //renderMesh(mesh->filename)
  }
  else if  (type == TORUS)
  {
    boost::shared_ptr<otdf::Torus> torus(boost::shared_dynamic_cast<otdf::Torus>(link));
    double innerRadius = torus->tube_radius;
    double outerRadius = torus->radius;
   
  //todo
    glPushMatrix();
        //size cuboid
    
      // move base up so that bottom face is at origin
     glTranslatef(nextTf.tf.translation.x,
 	 	nextTf.tf.translation.y,
  		nextTf.tf.translation.z);
     glRotatef(theta * 180/3.141592654, 
       	 axis[0], axis[1], axis[2]); 
      glutSolidTorus(innerRadius,outerRadius,36,36); 
    // glutWireTorus(innerRadius,outerRadius,8,8); 
 
    glPopMatrix();
  

  }
  else {
    //cout << "UNKNOWN"<< endl;
  }

  gluDeleteQuadric(quadric);
}


static void
_draw (BotViewer *viewer, BotRenderer *renderer)
{
  RendererOtdf *self = (RendererOtdf*) renderer;
  
  glEnable(GL_DEPTH_TEST);

  //-draw 
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
  // glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); 
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
 
 double c[3] = {0.3,0.3,0.6};
 double alpha = 0.8;
  //glColor3f(c[0],c[1],c[2]);
  glColor4f(c[0],c[1],c[2],alpha);
  
  typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
  for(object_instance_map_type_::const_iterator it = self->instantiated_objects.begin(); it!=self->instantiated_objects.end(); it++)
   { 
     for(uint i = 0; i < it->second._link_tfs.size(); i++)
    {
      drc::link_transform_t nextTf = it->second._link_tfs[i];
      boost::shared_ptr<otdf::Geometry> nextLink = it->second._link_shapes[i];
      link_draw(nextLink, nextTf);
    }
   }
  
  int64_t now = bot_timestamp_now();

  if(!self->dragging && now > self->max_draw_utime)
    return;

  //glColor3f(0, 1, 0);
  glColor3f(self->circle_color[0], self->circle_color[1], self->circle_color[2]);
  glPushMatrix();
  glTranslatef(self->click_pos.x, self->click_pos.y, 0);

  bot_gl_draw_circle(self->goal_std);

  glBegin(GL_LINE_STRIP);
  glVertex2f(0.0,0.0);

  glVertex2f(self->goal_std*cos(self->theta),self->goal_std*sin(self->theta));
  glEnd();

  glPopMatrix();

}

static void
recompute_2d_goal_pose(RendererOtdf *self)
{

  self->click_pos = self->drag_start_local;
  double dx = self->drag_finish_local.x - self->drag_start_local.x;
  double dy = self->drag_finish_local.y - self->drag_start_local.y;

  double theta = atan2(dy,dx);
  self->theta = theta;

  self->goal_std = sqrt(dx*dx + dy*dy);
  if(self->goal_std < MIN_STD)
    self->goal_std = MIN_STD;
  if(self->goal_std > MAX_STD)
    self->goal_std = MAX_STD;
  self->max_draw_utime = bot_timestamp_now() + DRAW_PERSIST_SEC * 1000000;

}

static int 
mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
    const double ray_dir[3], const GdkEventButton *event)
{
  RendererOtdf *self = (RendererOtdf*) ehandler->user;

  //fprintf(stderr, "Active: %d | Mouse Press : %f,%f\n",self->active, ray_start[0], ray_start[1]);

  self->dragging = 0;

  if(ehandler->picking==0){
    //fprintf(stderr, "Ehandler Not active\n");
    return 0;
  }
  if(self->active==0){
    fprintf(stderr, "Not Active\n");
    return 0;
  }

  if(event->button != 1){
   // fprintf(stderr,"Wrong Button\n");
    return 0;
  }

  point2d_t click_pt_local;
  if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start),
      POINT3D(ray_dir), 0, &click_pt_local)) {
    bot_viewer_request_redraw(self->viewer);
    self->active = 0;
    return 0;
  }

  self->dragging = 1;

  self->drag_start_local = click_pt_local;
  self->drag_finish_local = click_pt_local;

  recompute_2d_goal_pose(self);

  bot_viewer_request_redraw(self->viewer);
  return 1;
}



static int mouse_release(BotViewer *viewer, BotEventHandler *ehandler,
    const double ray_start[3], const double ray_dir[3],
    const GdkEventButton *event)
{
  RendererOtdf *self = (RendererOtdf*) ehandler->user;

  if (self->dragging) {
    self->dragging = 0;
  }
  if (self->active != 0) {
    // check drag points and publish

    printf("x,y,t: %f %f %f.    std: %f\n",self->click_pos.x
        ,self->click_pos.y,self->theta,self->goal_std);

    fprintf(stderr," Activate Value : %d\n", self->active);
    if(self->active == 1){
      bot_viewer_set_status_bar_message(self->viewer, "self->active is 1");

    }
    self->last_active = self->active;
    self->active = 0;

    ehandler->picking = 0;
    return 1;
  }
  else
    ehandler->picking = 0;

  return 0;
}

static int mouse_motion (BotViewer *viewer, BotEventHandler *ehandler,
    const double ray_start[3], const double ray_dir[3],
    const GdkEventMotion *event)
{
  RendererOtdf *self = (RendererOtdf*) ehandler->user;

  if(!self->dragging || self->active==0)
    return 0;

  point2d_t drag_pt_local;
  if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start),
      POINT3D(ray_dir), 0, &drag_pt_local)) {
    return 0;
  }
  self->drag_finish_local = drag_pt_local;
  recompute_2d_goal_pose(self);

  bot_viewer_request_redraw(self->viewer);
  return 1;
}

void activate(RendererOtdf *self, int type)
{
  self->active = type;

  self->goal_std=0;
  if (self->active ==1){
    self->circle_color[0] = 1;
    self->circle_color[1] = 0;
    self->circle_color[2] = 0;
  }else if (self->active ==2){
    self->circle_color[0] = 0;
    self->circle_color[1] = 1;
    self->circle_color[2] = 0;
  }else if (self->active ==3){
    self->circle_color[0] = 0;
    self->circle_color[1] = 0;
    self->circle_color[2] = 1;
  }
}

static int key_press (BotViewer *viewer, BotEventHandler *ehandler, 
    const GdkEventKey *event)
{
  RendererOtdf *self = (RendererOtdf*) ehandler->user;

  if ((event->keyval == 'r' || event->keyval == 'R') && self->active==0) {
    printf("\n[R]einit key registered\n");
    activate(self,1);
    bot_viewer_request_pick (viewer, ehandler);
  }else if ((event->keyval == 'g' || event->keyval == 'G') && self->active==0) {
    printf("\n[G]oal key registered\n");
    activate(self,2);
    bot_viewer_request_pick (viewer, ehandler);
  }else if ((event->keyval == 'l' || event->keyval == 'L') && self->active==0) {
    printf("\n[L]eft arm key registered\n");
    activate(self,3);
    bot_viewer_request_pick (viewer, ehandler);
  } else if(event->keyval == GDK_Escape) {
    //self->active = 0;
    //ehandler->picking = 0;
    //bot_viewer_set_status_bar_message(self->viewer, "");
  }

  return 0;
}

// =================================================================================
//  Fk and maintaining  OtdfInstanceStruc


static void run_fk_and_gen_link_shapes_and_tfs (OtdfInstanceStruc &instance_struc)
{

    //clear stored data
    instance_struc._link_tfs.clear();
    instance_struc._link_shapes.clear();
    
    std::map<std::string, double> jointpos_in;
    enum  {UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED};
 
  
   typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
   for (joints_mapType::iterator joint = instance_struc._otdf_instance->joints_.begin();joint != instance_struc._otdf_instance->joints_.end(); joint++)
   {
      if(joint->second->type!=(int) FIXED) { // All joints that not of the type FIXED.
          double dof_current_pos = 0; // TODO: need object's initial dof state from fitting
          jointpos_in.insert(make_pair(joint->first, dof_current_pos)); 
       }
   }
   
  //Have to handle joint_patterns separately   
  // DoF of all joints in joint patterns.
  typedef std::map<std::string,boost::shared_ptr<otdf::Joint_pattern> > jp_mapType;
  for (jp_mapType::iterator jp_it = instance_struc._otdf_instance->joint_patterns_.begin();jp_it != instance_struc._otdf_instance->joint_patterns_.end(); jp_it++)
  {
    // for all joints in joint pattern.
    for (unsigned int i=0; i < jp_it->second->joint_set.size(); i++)
    {

        if(jp_it->second->joint_set[i]->type!=(int) FIXED) { // All joints that not of the type FIXED.
	     double dof_current_pos = 0; //TODO: need object's initial dof state from fitting
            jointpos_in.insert(make_pair(jp_it->second->joint_set[i]->name, dof_current_pos)); 
       } // end if
       
    } // end for all joints in jp
  }// for all joint patterns
    
    std::map<string, drc::transform_t > cartpos_out;
    
		  // Calculate forward position kinematics
    bool kinematics_status;
    bool flatten_tree=true; // determines the absolute transforms with respect to robot origin. 
                            //Otherwise returns relative transforms between joints. 
    
    kinematics_status = instance_struc._fksolver->JntToCart(jointpos_in,cartpos_out,flatten_tree);
    if(kinematics_status>=0){
      // cout << "Success!" <<endl;
    }
    else{
       std::cerr << "Error: could not calculate forward kinematics!" <<std::endl;
       return;
    }
    
   std::map<std::string, boost::shared_ptr<otdf::Link> > _links_map =  instance_struc._otdf_instance->links_;  // static links
    
  //Have to handle link_patterns separately  
  typedef std::map<std::string,boost::shared_ptr<otdf::Link_pattern> > lp_mapType;
  for (lp_mapType::iterator lp_it = instance_struc._otdf_instance->link_patterns_.begin();lp_it != instance_struc._otdf_instance->link_patterns_.end(); lp_it++)
  {
    // for all joints in joint pattern.
    for (unsigned int i=0; i < lp_it->second->link_set.size(); i++)
    {
      _links_map.insert(std::make_pair(lp_it->second->link_set[i]->name,lp_it->second->link_set[i]));       
    } // end for all links in lp
   }// for all link patterns
  
    
   typedef std::map<std::string, boost::shared_ptr<otdf::Link> > links_mapType;
   for( links_mapType::const_iterator it =  _links_map.begin(); it!= _links_map.end(); it++)
   { 
    	if(it->second->visual)
	{

	      otdf::Pose visual_origin = it->second->visual->origin;
	    KDL::Frame T_parentjoint_visual, T_body_parentjoint, T_body_visual, T_world_body, T_world_visual;

	      T_world_body.p[0]= instance_struc._otdf_instance->getParam("x");
	      T_world_body.p[1]= instance_struc._otdf_instance->getParam("y");
	      T_world_body.p[2]= instance_struc._otdf_instance->getParam("z");
	      T_world_body.M =  KDL::Rotation::RPY(instance_struc._otdf_instance->getParam("roll"),
	                                           instance_struc._otdf_instance->getParam("pitch"),
	                                           instance_struc._otdf_instance->getParam("yaw"));

	    std::map<std::string, drc::transform_t>::const_iterator transform_it;
	    transform_it=cartpos_out.find(it->first);	  
	       
	       if(transform_it!=cartpos_out.end())// fk cart pos exists
	       {
	          T_body_parentjoint.p[0]= transform_it->second.translation.x;
	          T_body_parentjoint.p[1]= transform_it->second.translation.y;
	          T_body_parentjoint.p[2]= transform_it->second.translation.z;	
               
	          T_body_parentjoint.M =  KDL::Rotation::Quaternion(transform_it->second.rotation.x, transform_it->second.rotation.y, transform_it->second.rotation.z, transform_it->second.rotation.w);


	          T_parentjoint_visual.p[0]=visual_origin.position.x;
	          T_parentjoint_visual.p[1]=visual_origin.position.y;
	          T_parentjoint_visual.p[2]=visual_origin.position.z;
	          T_parentjoint_visual.M =  KDL::Rotation::Quaternion(visual_origin.rotation.x, visual_origin.rotation.y, visual_origin.rotation.z, visual_origin.rotation.w);

	          T_body_visual  = T_body_parentjoint*T_parentjoint_visual;
	         
	          T_world_visual = T_world_body*T_body_visual;
                   //T_world_visual  = T_world_camera*T_camera_body*T_body_visual;

	          drc::link_transform_t state;	    

	          state.link_name = transform_it->first;

	          state.tf.translation.x = T_world_visual.p[0];
	          state.tf.translation.y = T_world_visual.p[1];
	          state.tf.translation.z = T_world_visual.p[2];
	          T_world_visual.M.GetQuaternion(state.tf.rotation.x,state.tf.rotation.y,state.tf.rotation.z,state.tf.rotation.w);
	            
  
	          boost::shared_ptr<otdf::Geometry> geom =  it->second->visual->geometry;

	          //---store
	          instance_struc._link_shapes.push_back(geom);
	          instance_struc._link_tfs.push_back(state);
	       }  // end if(transform_it!=cartpos_out.end())
	    
	    } // end if(it->second->visual)
       
   } // end for links in  _links_map

} // end function run_fk_and_gen_link_shapes_and_tfs

static void create_otdf_object_instance (RendererOtdf *self)
{

 std::string filename = self->otdf_filenames[self->otdf_id];
 std::string xml_string;
 if(!otdf::get_xml_string_from_file(filename, xml_string)){
   return; // file extraction failed
  }

  OtdfInstanceStruc instance_struc;
  instance_struc._otdf_instance = otdf::parseOTDF(xml_string);
  if (!instance_struc._otdf_instance){
    std::cerr << "ERROR: Model Parsing of " << filename << " the xml failed" << std::endl;
  }
  // TODO: create a KDL tree parser from OTDF instance, without having to convert to urdf.
  // otdf can contain some elements that are not part of urdf. e.g. TORUS  
  std::string _urdf_xml_string = otdf::convertObjectInstanceToCompliantURDFstring(instance_struc._otdf_instance);
  // Parse KDL tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
  {
    std::cerr << "ERROR: Failed to extract kdl tree from "  << filename << " xml object description "<< std::endl; 
  }
  
  instance_struc._fksolver = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
  run_fk_and_gen_link_shapes_and_tfs(instance_struc);

  
  std::map<std::string, int >::iterator it;
  it= self->instance_cnt.find(self->otdf_filenames[self->otdf_id]);
  it->second = it->second + 1;
  std::stringstream oss;
  oss << self-> otdf_filenames[self->otdf_id] << "_"<< it->second;  
  self->instantiated_objects.insert(std::make_pair(oss.str(), instance_struc));
  bot_viewer_request_redraw(self->viewer);
} 

static void update_OtdfInstanceStruc (OtdfInstanceStruc &instance_struc){
     instance_struc._otdf_instance->update();
      std::string _urdf_xml_string = otdf::convertObjectInstanceToCompliantURDFstring(instance_struc._otdf_instance);
      // Parse KDL tree
      // TODO: create a KDL tree parser from OTDF instance, without having to convert to urdf.
      // otdf can contain some elements that are not part of urdf. e.g. TORUS
      KDL::Tree tree;
      if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
      {
        std::cerr << "ERROR: Failed to extract kdl tree from "  << instance_struc._otdf_instance->getName() << " xml object description "<< std::endl; 
      }
      
      instance_struc._fksolver = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
      run_fk_and_gen_link_shapes_and_tfs(instance_struc);
 }


// =================================================================================
//  OTDF popups and associated cbs (2 stage popups)

static gboolean on_popup_close (GtkButton* button, GtkWidget* pWindow)
{
  gtk_widget_destroy (pWindow);
  return TRUE;
}

static void on_adjust_params_popup_close (BotGtkParamWidget *pw, void *user)
{
  RendererOtdf *self = (RendererOtdf*) user;
  std::string instance_name=  (*self->instance_selection_ptr);
  //typedef std::map<std::string, boost::shared_ptr<otdf::ModelInterface> > object_instance_map_type_;
  typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
  object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
   typedef std::map<std::string, double > params_mapType;
  // for( params_mapType::const_iterator it2 = it->second->params_map_.begin(); it2!=it->second->params_map_.end(); it2++)
   for( params_mapType::const_iterator it2 = it->second._otdf_instance->params_map_.begin(); it2!=it->second._otdf_instance->params_map_.end(); it2++)
   { 
    
        double t = bot_gtk_param_widget_get_double (pw,it2->first.c_str());
	//std::cout << it->first << ": " << t << std::endl;
	//it->second->setParam(it->first, t);
	it->second._otdf_instance->setParam(it2->first, t);
   }
   // it->second->update();
   
   // regen kdl::tree and reset fksolver
   // regen link tfs and shapes for display
   update_OtdfInstanceStruc(it->second);
   bot_viewer_request_redraw(self->viewer);
}

static void on_otdf_adjust_param_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
{
  RendererOtdf *self = (RendererOtdf*) user;
 
  std::string instance_name=  (*self->instance_selection_ptr);
  typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
  object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
 
   typedef std::map<std::string, double > params_mapType;
  for( params_mapType::const_iterator it2 = it->second._otdf_instance->params_map_.begin(); it2!=it->second._otdf_instance->params_map_.end(); it2++)
  { 
     if(!strcmp(name, it2->first.c_str())) {
        double t = bot_gtk_param_widget_get_double (pw,it2->first.c_str());
	//std::cout << it2->first << ": " << t << std::endl;
	  it->second._otdf_instance->setParam(it2->first, t);
     }
   }
   // gives realtime feedback of the geometry changing.
   update_OtdfInstanceStruc(it->second);
   bot_viewer_request_redraw(self->viewer);
}

static void on_adjust_dofs_popup_close (BotGtkParamWidget *pw, void *user)
{
//     RendererOtdf *self = (RendererOtdf*) user;
//   std::string instance_name=  (*self->instance_selection_ptr);
//   typedef std::map<std::string, boost::shared_ptr<otdf::ModelInterface> > object_instance_map_type_;
//   object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
   // TODO: Send publish affordance command msg
}

static void on_otdf_adjust_dofs_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
{
//   RendererOtdf *self = (RendererOtdf*) user;
//   std::string instance_name=  (*self->instance_selection_ptr);
//   typedef std::map<std::string, boost::shared_ptr<otdf::ModelInterface> > object_instance_map_type_;
//   object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
//  
 //TODO: do something
}

static void spawn_adjust_params_popup (RendererOtdf *self){

  GtkWidget *window, *close_button, *vbox;
  BotGtkParamWidget *pw;

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_transient_for(GTK_WINDOW(window), GTK_WINDOW(self->viewer->window));
  gtk_window_set_modal(GTK_WINDOW(window), FALSE);
  gtk_window_set_decorated  (GTK_WINDOW(window),FALSE);
  gtk_window_stick(GTK_WINDOW(window));
  gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_MOUSE);
  gtk_window_set_default_size(GTK_WINDOW(window), 300, 250);
    //gtk_widget_set_size_request (window, 300, 250);
  //gtk_window_set_resizable(GTK_WINDOW(window), FALSE);
  gtk_window_set_title(GTK_WINDOW(window), "Adjust Params");
  gtk_container_set_border_width(GTK_CONTAINER(window), 5);
  pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

  std::string instance_name=  (*self->instance_selection_ptr);

  typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
  object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);

  for(std::vector<std::string>::const_iterator it2 = it->second._otdf_instance->params_order_.begin(); it2 != it->second._otdf_instance->params_order_.end(); ++it2) 
  {
  
    double inc = it->second._otdf_instance->param_properties_map_[*it2].inc;
    double min = it->second._otdf_instance->param_properties_map_[*it2].min_value;
    double max = it->second._otdf_instance->param_properties_map_[*it2].max_value;
    double value = it->second._otdf_instance->params_map_[*it2];
    bot_gtk_param_widget_add_double(pw, (*it2).c_str(), BOT_GTK_PARAM_WIDGET_SPINBOX,
     min, max, inc, value); 
    
  }

  g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_otdf_adjust_param_widget_changed), self);


  close_button = gtk_button_new_with_label ("Close");
  g_signal_connect (G_OBJECT (close_button),
                    "clicked",
                    G_CALLBACK (on_popup_close),
                    (gpointer) window);
   g_signal_connect(G_OBJECT(pw), "destroy",
      G_CALLBACK(on_adjust_params_popup_close), self); 

  
  vbox = gtk_vbox_new (FALSE, 3);
  gtk_box_pack_end (GTK_BOX (vbox), close_button, FALSE, FALSE, 5);
  gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(pw), FALSE, FALSE, 5);
  gtk_container_add (GTK_CONTAINER (window), vbox);
  gtk_widget_show_all(window); 
}


static void spawn_adjust_dofs_popup (RendererOtdf *self){

  GtkWidget *window, *close_button, *vbox;
  BotGtkParamWidget *pw;

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_transient_for(GTK_WINDOW(window), GTK_WINDOW(self->viewer->window));
  gtk_window_set_modal(GTK_WINDOW(window), FALSE);
  gtk_window_set_decorated  (GTK_WINDOW(window),FALSE);
  gtk_window_stick(GTK_WINDOW(window));
  gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_MOUSE);
  gtk_window_set_default_size(GTK_WINDOW(window), 300, 250);
    //gtk_widget_set_size_request (window, 300, 250);
  //gtk_window_set_resizable(GTK_WINDOW(window), FALSE);
  gtk_window_set_title(GTK_WINDOW(window), "Adjust Dofs");
  gtk_container_set_border_width(GTK_CONTAINER(window), 5);
  pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

  std::string instance_name=  (*self->instance_selection_ptr);

  typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
  object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);

   enum
  {
    UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
  };
  
  // Need tarcked joint positions of all objects.
  
   typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
   for (joints_mapType::iterator joint = it->second._otdf_instance->joints_.begin();joint != it->second._otdf_instance->joints_.end(); joint++)
   {
     
      double current_dof_position = 0;// TODO: dof pos tracking
       if(joint->second->type!=(int) FIXED) { // All joints that not of the type FIXED.
	  if(joint->second->type==(int) CONTINUOUS) {
	    bot_gtk_param_widget_add_double(pw, joint->first.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
	      0, M_PI, .01, current_dof_position); 
	  }
	  else{
	    bot_gtk_param_widget_add_double(pw, joint->first.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
	      joint->second->limits->lower, joint->second->limits->upper, .01, current_dof_position);
	  }   
       }
   }
   //Have to handle joint_patterns separately   
  // DoF of all joints in joint patterns.
  typedef std::map<std::string,boost::shared_ptr<otdf::Joint_pattern> > jp_mapType;
  for (jp_mapType::iterator jp_it = it->second._otdf_instance->joint_patterns_.begin();jp_it != it->second._otdf_instance->joint_patterns_.end(); jp_it++)
  {
    // for all joints in joint pattern.
    for (unsigned int i=0; i < jp_it->second->joint_set.size(); i++)
    {
      double current_dof_position = 0;// TODO: dof pos tracking
        if(jp_it->second->joint_set[i]->type!=(int) FIXED) { // All joints that not of the type FIXED.
	  if(jp_it->second->joint_set[i]->type==(int) CONTINUOUS) {
	    bot_gtk_param_widget_add_double(pw, jp_it->second->joint_set[i]->name.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
	      0, M_PI, .01, current_dof_position); 
	  }
	  else{
	    bot_gtk_param_widget_add_double(pw, jp_it->second->joint_set[i]->name.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
	      jp_it->second->joint_set[i]->limits->lower, jp_it->second->joint_set[i]->limits->upper, .01, current_dof_position);
	  }   
       } // end if
       
    } // end for all joints in jp
  }// for all joint patterns

  g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_otdf_adjust_dofs_widget_changed), self);


  close_button = gtk_button_new_with_label ("Close");
  g_signal_connect (G_OBJECT (close_button),
                    "clicked",
                    G_CALLBACK (on_popup_close),
                    (gpointer) window);
   g_signal_connect(G_OBJECT(pw), "destroy",
      G_CALLBACK(on_adjust_dofs_popup_close), self); 

  
  vbox = gtk_vbox_new (FALSE, 3);
  gtk_box_pack_end (GTK_BOX (vbox), close_button, FALSE, FALSE, 5);
  gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(pw), FALSE, FALSE, 5);
  gtk_container_add (GTK_CONTAINER (window), vbox);
  gtk_widget_show_all(window); 
}



static void publish_eegoal(boost::shared_ptr<lcm::LCM> &_lcm, OtdfInstanceStruc &instance_struc, std::string channel, KDL::Frame &T_body_world)
 {
   drc::ee_goal_t goalmsg;
   
  double x,y,z,w;
  
  // desired ee position in world frame
  KDL::Frame T_world_ee,T_body_ee;
  T_world_ee.p[0]= instance_struc._otdf_instance->getParam("x");
	T_world_ee.p[1]= instance_struc._otdf_instance->getParam("y");
	T_world_ee.p[2]= instance_struc._otdf_instance->getParam("z");
	T_world_ee.M =  KDL::Rotation::RPY(instance_struc._otdf_instance->getParam("roll"),
	                                     instance_struc._otdf_instance->getParam("pitch"),
	                                     instance_struc._otdf_instance->getParam("yaw"));
	        
  //T_body_world = self->robotStateListener->T_body_world; //KDL::Frame::Identity(); // must also have robot state listener.
  
  // desired ee position wrt to robot body.
  T_body_ee = T_body_world*T_world_ee;
  
	T_body_ee.M.GetQuaternion(x,y,z,w);

	goalmsg.ee_goal_pos.translation.x = T_body_ee.p[0];
	goalmsg.ee_goal_pos.translation.y = T_body_ee.p[1];
	goalmsg.ee_goal_pos.translation.z = T_body_ee.p[2];

	goalmsg.ee_goal_pos.rotation.x = x;
	goalmsg.ee_goal_pos.rotation.y = y;
	goalmsg.ee_goal_pos.rotation.z = z;
	goalmsg.ee_goal_pos.rotation.w = w;

	goalmsg.ee_goal_twist.linear_velocity.x = 0.0;
	goalmsg.ee_goal_twist.linear_velocity.y = 0.0;
	goalmsg.ee_goal_twist.linear_velocity.z = 0.0;
	goalmsg.ee_goal_twist.angular_velocity.x = 0.0;
	goalmsg.ee_goal_twist.angular_velocity.y = 0.0;
	goalmsg.ee_goal_twist.angular_velocity.z = 0.0;
	
	goalmsg.num_chain_joints  = 6;
	// No specified posture bias
	goalmsg.use_posture_bias  = false;
	goalmsg.joint_posture_bias.resize(goalmsg.num_chain_joints);
	goalmsg.chain_joint_names.resize(goalmsg.num_chain_joints);
	for(int i = 0; i < goalmsg.num_chain_joints; i++){
		goalmsg.joint_posture_bias[i]=0;
		goalmsg.chain_joint_names[i]= "dummy_joint_names";
	}

	// Publish the message
	goalmsg.halt_ee_controller = false;

  _lcm->publish(channel, &goalmsg);
 
 }

static void on_otdf_instance_management_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
{
  RendererOtdf *self = (RendererOtdf*) user;
  
// int selection = bot_gtk_param_widget_get_enum (pw, PARAM_OTDF_INSTANCE_SELECT);
  const char *instance_name;
  instance_name = bot_gtk_param_widget_get_enum_str( pw, PARAM_OTDF_INSTANCE_SELECT );
  (*self->instance_selection_ptr)  = std::string(instance_name);

  if(!strcmp(name,PARAM_OTDF_ADJUST_PARAM)) {
     spawn_adjust_params_popup(self);
      
  }
  else if(!strcmp(name,PARAM_OTDF_ADJUST_DOF)) {
      spawn_adjust_dofs_popup(self);
  }
  else if(!strcmp(name,PARAM_OTDF_INSTANCE_CLEAR)) {
     fprintf(stderr,"\nClearing Selected Instance\n");
   
   //typedef std::map<std::string, boost::shared_ptr<otdf::ModelInterface> > object_instance_map_type_;
  typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
   object_instance_map_type_::iterator it = self->instantiated_objects.find(std::string(instance_name));
   self->instantiated_objects.erase(it);
   bot_viewer_request_redraw(self->viewer);
  }
  else if(!strcmp(name,PARAM_OTDF_INSTANCE_CLEAR_ALL)) {
     fprintf(stderr,"\nClearing Instantiated Objects\n");
    
     self->instantiated_objects.clear();
     for( std::map<std::string,int >::iterator it = self->instance_cnt.begin(); it!=self->instance_cnt.end(); it++)
     { 
       it->second = 0;
     }
     bot_viewer_request_redraw(self->viewer);
  }
  else if(!strcmp(name,PARAM_OTDF_REACH_OBJECT_L)) {
  fprintf(stderr,"\nReaching centroid of selected Object\n");
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
   object_instance_map_type_::iterator it = self->instantiated_objects.find(std::string(instance_name));
   KDL::Frame T_body_world = self->robotStateListener->T_body_world;
    publish_eegoal( self->lcm, it->second, "LWRISTROLL_LINK_GOAL",T_body_world);
  }
  else if(!strcmp(name,PARAM_OTDF_REACH_OBJECT_R)) {
  fprintf(stderr,"\nReaching centroid of selected Object\n");
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
   object_instance_map_type_::iterator it = self->instantiated_objects.find(std::string(instance_name));
   KDL::Frame T_body_world = self->robotStateListener->T_body_world;
    publish_eegoal( self->lcm, it->second, "RWRISTROLL_LINK_GOAL",T_body_world);
    
  }
}

static void spawn_instance_management_popup (RendererOtdf *self)
{
    GtkWidget *window, *close_button, *vbox;
    BotGtkParamWidget *pw; 

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_transient_for(GTK_WINDOW(window), GTK_WINDOW(self->viewer->window));
    gtk_window_set_modal(GTK_WINDOW(window), FALSE);
    gtk_window_set_decorated  (GTK_WINDOW(window),FALSE);
    gtk_window_stick(GTK_WINDOW(window));
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_MOUSE);
    gtk_window_set_default_size(GTK_WINDOW(window), 300, 250);
    gtk_window_set_title(GTK_WINDOW(window), "Instance Management");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    int num_otdf_instances;
    char ** otdf_instance_names;
    int * otdf_instance_nums;
  
    if( self->instantiated_objects.size() > 0)
    {

    num_otdf_instances = self->instantiated_objects.size();
    otdf_instance_names =(char **) calloc(num_otdf_instances, sizeof(char *));
    otdf_instance_nums = (int *)calloc(num_otdf_instances, sizeof(int));

    unsigned int i = 0;  
   //typedef std::map<std::string, boost::shared_ptr<otdf::ModelInterface> > object_instance_map_type_;
  typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
  for( object_instance_map_type_::const_iterator it = self->instantiated_objects.begin(); it!=self->instantiated_objects.end(); it++)
      { 
	    std::string instance_name = it->first;
	    otdf_instance_names[i] = (char *) instance_name.c_str();
	    otdf_instance_nums[i] =i;
	   // std::cout << "Instance:  " << instance_name  << " i: "<< i << std::endl;
	  // self->instantiated_objects_id.insert(std::make_pair(i,instance_name));
	    ++i;
      }
    }
    else 
    {
	num_otdf_instances = 1;
	otdf_instance_names =(char **) calloc(num_otdf_instances, sizeof(char *));
	otdf_instance_nums = (int *)calloc(num_otdf_instances, sizeof(int));
    
	    std::string instance_name = "No objects Instantiated";
	    otdf_instance_names[0]= (char *) instance_name.c_str();
	    otdf_instance_nums[0] =0; 
    }

    bot_gtk_param_widget_add_enumv (pw, PARAM_OTDF_INSTANCE_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 
					0,
					num_otdf_instances,
				        (const char **)  otdf_instance_names,
				        otdf_instance_nums);
  
   if( self->instantiated_objects.size() > 0)
   {
    bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_ADJUST_PARAM, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_ADJUST_DOF, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_INSTANCE_CLEAR, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_INSTANCE_CLEAR_ALL, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_REACH_OBJECT_L, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_REACH_OBJECT_R, NULL);
   }
   
   g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_otdf_instance_management_widget_changed), self);

   close_button = gtk_button_new_with_label ("Close");
   g_signal_connect (G_OBJECT (close_button),
                    "clicked",
                    G_CALLBACK (on_popup_close),
                    (gpointer) window);

  vbox = gtk_vbox_new (FALSE, 3);
  gtk_box_pack_end (GTK_BOX (vbox), close_button, FALSE, FALSE, 5);

  gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(pw), FALSE, FALSE, 5);
  gtk_container_add (GTK_CONTAINER (window), vbox);
  gtk_widget_show_all(window);
}



static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererOtdf *self = (RendererOtdf*) user;
  if(!strcmp(name, PARAM_MANAGE_INSTANCES)) {
    fprintf(stderr,"\nClicked Manage Instances\n");
    spawn_instance_management_popup(self);
   // activate(self, 1);
  }
  else if (! strcmp (name, PARAM_OTDF_SELECT)) {
   self->otdf_id = bot_gtk_param_widget_get_enum (self->pw, PARAM_OTDF_SELECT);

  }
  else if(!strcmp(name, PARAM_INSTANTIATE)) {
    std::cout << "\nInstantiating Selected Otdf:  " << self->otdf_filenames[self->otdf_id] << std::endl;
    create_otdf_object_instance(self);
    bot_viewer_request_pick (self->viewer, &(self->ehandler));
    activate(self, 1);
  }
  else if(!strcmp(name, PARAM_CLEAR)) {
    fprintf(stderr,"\nClearing Instantiated Objects\n");
    
     self->instantiated_objects.clear();
     for( std::map<std::string,int >::iterator it = self->instance_cnt.begin(); it!=self->instance_cnt.end(); it++)
     { 
       it->second = 0;
     }
     bot_viewer_request_redraw(self->viewer);
//     bot_viewer_request_pick (self->viewer, &(self->ehandler));
//     activate(self, 3);
  }
}

static void
_free (BotRenderer *renderer)
{
  free (renderer);
}

BotRenderer *renderer_otdf_new (BotViewer *viewer, int render_priority, lcm_t *lcm)
{
  
  RendererOtdf *self = (RendererOtdf*) calloc (1, sizeof (RendererOtdf));
  self->lcm = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcm));
  
 // boost::shared_ptr<AffordanceCollectionListener> affordanceMsgHandler(new AffordanceCollectionListener(self));
 self->affordanceMsgHandler = boost::shared_ptr<AffordanceCollectionListener>(new AffordanceCollectionListener(self));
 self->robotStateListener = boost::shared_ptr<RobotStateListener>(new RobotStateListener(self->lcm,viewer));

  self->viewer = viewer;
  self->renderer.draw = _draw;
  self->renderer.destroy = _free;
  self->renderer.name = (char*) RENDERER_NAME;
  self->renderer.user = self;
  self->renderer.enabled = 1;

  BotEventHandler *ehandler = &self->ehandler;
  ehandler->name = (char*) RENDERER_NAME;
  ehandler->enabled = 1;
  ehandler->pick_query = NULL;
  ehandler->key_press = NULL;
  ehandler->hover_query = NULL;
  ehandler->mouse_press = mouse_press;
  ehandler->mouse_release = mouse_release;
  ehandler->mouse_motion = mouse_motion;
  ehandler->user = self;
 

/*
 #define OTDF_LIBRARY_FOLDER "/software/object_template_model/otdf_library/"
    std::string dir;
  char * pPath;
  pPath = getenv ("DRC_PATH");
 if (pPath!=NULL){
   dir = string(pPath) + OTDF_LIBRARY_FOLDER;
  }
 else{
    pPath = getenv ("HOME");
     if (pPath!=NULL)
       printf ("The current home path is: %s\n",pPath);

  dir = string(pPath) + "/drc" + OTDF_LIBRARY_FOLDER;

 }
  //  self->otdf_dir = dir; seg faults when  self->otdf_dir is a string, so using a string ptr
 */

  string otdf_models_path = std::string(getModelsPath()) + "/otdf/"; // getModelsPath gives /drc/software/build/models/
  self->otdf_dir_name_ptr = new std::string(otdf_models_path);
  std::cout << "searching for otdf files in: "<< (*self->otdf_dir_name_ptr) << std::endl;
  std::vector<std::string> otdf_files = std::vector<std::string>();
  get_OTDF_filenames_from_dir(otdf_models_path.c_str(),otdf_files);
  std::cout << "found " << otdf_files.size() << " files"<< std::endl;
  self->num_otdfs = otdf_files.size();
  self->otdf_names =(char **) calloc(self->num_otdfs, sizeof(char *));
  self->otdf_nums = (int *)calloc(self->num_otdfs, sizeof(int));
  self->instance_cnt.clear();
  self->instantiated_objects.clear();
  self->instance_selection_ptr = new std::string(" ");

  for(size_t i=0;i<otdf_files.size();i++){
   std::cout << otdf_files[i] << std::endl;
   self->otdf_filenames.push_back(otdf_files[i]);
   self->instance_cnt.insert(std::make_pair(otdf_files[i], (int)0));
   self->otdf_names[i]=(char *) otdf_files[i].c_str();
   self->otdf_nums[i] =i;
  }

  bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);

  //self->lc = lcm; //globals_get_lcm_full(NULL,1);

  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
 
  
  self->otdf_id= 0; // default file

 bot_gtk_param_widget_add_enumv (self->pw, PARAM_OTDF_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 
					self->otdf_id,
					self->num_otdfs,
				        (const char **)  self->otdf_names,
				        self->otdf_nums);

 bot_gtk_param_widget_add_buttons(self->pw,PARAM_INSTANTIATE, NULL);
 bot_gtk_param_widget_add_buttons(self->pw, PARAM_MANAGE_INSTANCES, NULL);
 bot_gtk_param_widget_add_buttons(self->pw,PARAM_CLEAR, NULL);
 g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
 self->renderer.widget = GTK_WIDGET(self->pw);

 self->active = 0;


 return &self->renderer;
}

void setup_renderer_otdf(BotViewer *viewer, int render_priority, lcm_t *lcm)
{
  bot_viewer_add_renderer_on_side(viewer, renderer_otdf_new(viewer, render_priority, lcm),
      render_priority, 0); // 0= add on left hand side
}
