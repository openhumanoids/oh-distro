// This renderer is a direct copy of the renderer in libbot2
// which makes up bot-rwx-viewer, but as it has no .h I copied it over here
// mfallon 25march2011

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
#include "renderer_humanoid.hpp"



#define RENDERER_NAME "Humanoid"
#define PARAM_PICKING "Enable Selection"
#define PARAM_WIRE "Show BBoxs"  
#define PARAM_COLOR_ALPHA "Alpha"


using namespace std;
using namespace boost;
using namespace Eigen;
using namespace collision_detection;
using namespace humanoid_renderer;

typedef struct _RendererHumanoid 
{
  BotRenderer renderer;
  BotViewer          *viewer;
  BotGtkParamWidget *pw;
  boost::shared_ptr<fk::RobotStateListener> robotStateListener;
  boost::shared_ptr<lcm::LCM> lcm;
  //BotEventHandler *key_handler;
  BotEventHandler ehandler;
  bool picking;
  bool clicked;
  bool visualize_bbox;
  Eigen::Vector3f ray_start;
  Eigen::Vector3f ray_end;
  std::string* selection;
  
  // transparency of the model:
  double alpha;
} RendererHumanoid;

static void
_renderer_free (BotRenderer *super)
{
  RendererHumanoid *self = (RendererHumanoid*) super->user;
  free(self);
}




//=========================key and mouse press================

/*int cb_key_press (BotViewer *viewer, BotEventHandler *ehandler, const GdkEventKey *event)
{
  switch (event->keyval)
    {

    case GDK_Right:
      {
	cout << "\n right key pressed" << endl;
      }
    }
  
  return 1;
}*/


static int 
mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
    const double ray_dir[3], const GdkEventButton *event)
{
  RendererHumanoid *self = (RendererHumanoid*) ehandler->user;
  if(self->picking==0){
    //fprintf(stderr, "Ehandler Not active\n");
    return 0;
  }
  self->clicked = 1;
  fprintf(stderr, "Mouse Press : %f,%f\n",ray_start[0], ray_start[1]);

  Eigen::Vector3f from,to;
  from << ray_start[0], ray_start[1], ray_start[2];

  Eigen::Vector3f plane_normal,plane_pt;
  plane_normal << 0,0,1;
  plane_pt << 0,0,0;
  double lambda1 = ray_dir[0] * plane_normal[0]+
                   ray_dir[1] * plane_normal[1] +
                   ray_dir[2] * plane_normal[2];
   // check for degenerate case where ray is (more or less) parallel to plane
    if (fabs (lambda1) < 1e-9) return 0;

   double lambda2 = (plane_pt[0] - ray_start[0]) * plane_normal[0] +
       (plane_pt[1] - ray_start[1]) * plane_normal[1] +
       (plane_pt[2] - ray_start[2]) * plane_normal[2];
   double t = lambda2 / lambda1;// =1;
  
  to << ray_start[0]+t*ray_dir[0], ray_start[1]+t*ray_dir[1], ray_start[2]+t*ray_dir[2];
 
    self->ray_start = from;
  self->ray_end = to;
  std::cout  << "from " << from.transpose() << std::endl;
  std::cout  << "to " << to.transpose() << std::endl;
  
   //to << ray_dir[0], ray_dir[1], ray_dir[2];
 //std::cout  << "num_coll_objects: " << self->robotStateListener->_collision_object_map.size() <<  std::endl;
 //std::cout  << "num_colls: " << self->robotStateListener->_collision_detector.num_collisions() <<  std::endl;// segfaults
  collision_detection::Collision_Object * intersected_object = NULL;
  self->robotStateListener->_collision_detector.num_collisions();
  self->robotStateListener->_collision_detector.ray_test( from, to, intersected_object );
  if( intersected_object != NULL ){
    std::cout << "prev selection :" << (*self->selection)  <<  std::endl;
    std::cout << "intersected :" << intersected_object->id().c_str() <<  std::endl;
    (*self->selection)  = std::string(intersected_object->id().c_str());
  }
  else
  (*self->selection)  = " ";


  bot_viewer_request_redraw(self->viewer);

  return 1;
}

static int 
mouse_release (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
    const double ray_dir[3], const GdkEventButton *event)
{
  RendererHumanoid *self = (RendererHumanoid*) ehandler->user;
  self->clicked = 0;
  if(self->picking==0){
    //fprintf(stderr, "Ehandler Not active\n");
    return 0;
  }
  
  bot_viewer_request_redraw(self->viewer);
  return 1;
}

//=================================

static void draw(shared_ptr<urdf::Geometry> link, const drc::link_transform_t &nextTf, string& nextLinkname, BotRenderer *super)
{
  
   RendererHumanoid *self = (RendererHumanoid*) super->user;

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
  enum {SPHERE, BOX, CYLINDER, MESH}; 
  
  if (type == SPHERE)
    {
      shared_ptr<urdf::Sphere> sphere(shared_dynamic_cast<urdf::Sphere>(link));	
      double radius = sphere->radius;
       glPushMatrix();
       glTranslatef(nextTf.tf.translation.x, nextTf.tf.translation.y, nextTf.tf.translation.z);
	     drawSphere(6,  radius);
       glPopMatrix();
    
    }
  else if  (type == BOX)
    {
    shared_ptr<urdf::Box> box(shared_dynamic_cast<urdf::Box>(link));
    double xDim = box->dim.x;
    double yDim = box->dim.y;
    double zDim = box->dim.z;
  //todo
    glPushMatrix();
        //size cuboid
    
        // move base up so that bottom face is at origin
     // glTranslatef(0,0.5,0.0); 
     glTranslatef(nextTf.tf.translation.x,
 	 	nextTf.tf.translation.y,
  		nextTf.tf.translation.z);

     glRotatef(theta * 180/3.141592654, 
       	 axis[0], axis[1], axis[2]); 
     glScalef(xDim,yDim,zDim);
         bot_gl_draw_cube();
        //cube();
    glPopMatrix();
  

  }else if  (type == CYLINDER){
    shared_ptr<urdf::Cylinder> cyl(shared_dynamic_cast<urdf::Cylinder>(link));
    /*glPointSize(10.0f);
    glColor3ub(0,1,0);
    glBegin(GL_POINTS);
    glVertex3f(nextTf.tf.translation.x,
    nextTf.tf.translation.y,
    nextTf.tf.translation.z);
    glEnd();*/



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

    //cout << "CYLINDER"<< endl;
    //cout << "radius : "<<  cyl->radius << endl;
    //cout << "length : "<<  cyl->length << endl;
    // drawBox(radius,length, it->second -> visual->origin);
  }
  else if  (type == MESH)
    {
    //cout << "MESH: " << nextLinkname << endl;
    //shared_ptr<urdf::Mesh> mesh(shared_dynamic_cast<urdf::Mesh>(link));
    
     /* size_t found1;
      found1=nextLinkname.find("r_");
    if (found1!=std::string::npos)
      {*/
        glPushMatrix();
        
        glTranslatef(nextTf.tf.translation.x,
        nextTf.tf.translation.y,
        nextTf.tf.translation.z);
        
        glRotatef(theta * 180/3.141592654, 
        axis[0], axis[1], axis[2]); 


        std::map<std::string, fk::MeshStruct>::const_iterator mesh_map_it;
        mesh_map_it=self->robotStateListener->_mesh_map.find(nextLinkname);
        if(mesh_map_it!=self->robotStateListener->_mesh_map.end()) // exists in cache
        { 
          if(!self->visualize_bbox)
          {
            glCallList (mesh_map_it->second.displaylist);
          }
          else 
          {
            // get the vertices for mesh_map_it->second
            double xDim = mesh_map_it->second.span_x;
            double yDim = mesh_map_it->second.span_y;
            double zDim = mesh_map_it->second.span_z;
            double xc = mesh_map_it->second.offset_x;
            double yc = mesh_map_it->second.offset_y;
            double zc = mesh_map_it->second.offset_z;
            
            glCallList (mesh_map_it->second.displaylist);
             
            // (27th Nov- Steven and Sisir)
            // THE DRC SDF defines the visual origin internally within the mesh vertices, which is weird.
            // The visual origin itself is set to 0,0,0
            // So if you are drawing a simple geometry you need to get the visual origin from the average of the extrema vertices
            //============================================
            glTranslatef(xc, yc, zc);
            glScalef(xDim,yDim,zDim);
            bot_gl_draw_cube_frame();
            
          }	
        }

    glPopMatrix();

   // }// end if (found1!=std::string::npos)
  }
  else {
  //cout << "UNKNOWN"<< endl;
  }


  gluDeleteQuadric(quadric);
}


static void 
_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
 //int64_t tic = bot_timestamp_now();
 
  RendererHumanoid *self = (RendererHumanoid*) super->user;

  glEnable(GL_DEPTH_TEST);
  //
  vector<shared_ptr<urdf::Geometry> > link_shapes;
  vector<drc::link_transform_t> link_tfs;
  vector<string> link_names;
  self->robotStateListener->getState(link_shapes, link_tfs, link_names);
  //fk::RobotStateListener::printTransforms(link_shapes, link_tfs);
  
  //-draw 
  //glPointSize(5.0f);
  //glBegin(GL_POINTS);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  
  glEnable(GL_BLEND);
  // glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); 
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
  glEnable (GL_RESCALE_NORMAL);

  if((self->picking)&&(self->clicked)){
        glLineWidth (3.0);
        glPushMatrix();
        glBegin(GL_LINES);
        glVertex3f(self->ray_start[0], self->ray_start[1],self->ray_start[2]); // object coord
        glVertex3f(self->ray_end[0], self->ray_end[1],self->ray_end[2]);
        glEnd();
        glPopMatrix();
  }
 
 double c[3] = {0.3,0.3,0.3};
  //glColor3f(c[0],c[1],c[2]);
  glColor4f(c[0],c[1],c[2],self->alpha);
  for(uint i = 0; i < link_tfs.size(); i++)
    {
      drc::link_transform_t nextTf = link_tfs[i];
      shared_ptr<urdf::Geometry> nextLink = link_shapes[i];
      string nextLinkname = link_names[i];
      if((self->picking)&&((*self->selection) == nextLinkname)) {
        glColor4f(0.7,0.1,0.1,self->alpha);     
      }
      else
      {
       glColor4f(c[0],c[1],c[2],self->alpha);
      }
      draw(nextLink, nextTf,nextLinkname,super);
    }
    
  //glPopMatrix();

// int64_t toc = bot_timestamp_now();
// cout << bot_timestamp_useconds(toc-tic) << endl;
}


static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererHumanoid *self = (RendererHumanoid*) user;
  if (! strcmp(name, PARAM_PICKING)) {
    if (bot_gtk_param_widget_get_bool(pw, PARAM_PICKING)) {
      bot_viewer_request_pick (self->viewer, &(self->ehandler));
      self->picking = 1;
    }
    else{
      self->picking = 0;
    }
  }
  else if(! strcmp(name, PARAM_WIRE)) {
    if (bot_gtk_param_widget_get_bool(pw, PARAM_WIRE)){
      self->visualize_bbox= true;  
    }
    else{
      self->visualize_bbox = false;
    }
  }else if(! strcmp(name, PARAM_COLOR_ALPHA)) {
    self->alpha = bot_gtk_param_widget_get_double(pw, PARAM_COLOR_ALPHA);
    bot_viewer_request_redraw(self->viewer);
  }
}

void 
setup_renderer_humanoid(BotViewer *viewer, int render_priority, lcm_t *lcm)
{
    RendererHumanoid *self = (RendererHumanoid*) calloc (1, sizeof (RendererHumanoid));
    self->lcm = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcm));
    self->robotStateListener = boost::shared_ptr<fk::RobotStateListener>(new fk::RobotStateListener(self->lcm, 
												    viewer));

    BotRenderer *renderer = &self->renderer;

    renderer->draw = _renderer_draw;
    renderer->destroy = _renderer_free;

    renderer->widget = bot_gtk_param_widget_new();
    renderer->name = (char *) RENDERER_NAME;
    renderer->user = self;
    renderer->enabled = 1;

    self->viewer = viewer;

    self->pw = BOT_GTK_PARAM_WIDGET(renderer->widget);

    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_PICKING, 0, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_WIRE, 0, NULL);
    
    bot_gtk_param_widget_add_double (self->pw, PARAM_COLOR_ALPHA, BOT_GTK_PARAM_WIDGET_SLIDER, 0, 1, 0.001, 1);
      
  	g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  	self->alpha = 1.0;
  	self->picking = 0;
    self->clicked=0;	
  	self->selection = new std::string(" ");
    self->visualize_bbox = false;
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
    ehandler->enabled = 1;
    ehandler->pick_query = NULL;
    ehandler->hover_query = NULL;
    ehandler->mouse_press = mouse_press;
    ehandler->mouse_release = mouse_release;
    ehandler->mouse_motion = NULL;
    ehandler->user = self;

    bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);
    
}

