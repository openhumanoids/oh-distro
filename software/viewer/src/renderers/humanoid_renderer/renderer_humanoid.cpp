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

#include "renderer_humanoid.hpp"
#include "RobotStateListener.hpp"

#define RENDERER_NAME "Humanoid"


using namespace std;
using namespace boost;

typedef struct _RendererHumanoid 
{
  BotRenderer renderer;
  BotViewer          *viewer;
  BotGtkParamWidget *pw;
  boost::shared_ptr<fk::RobotStateListener> robotStateListener;
  boost::shared_ptr<lcm::LCM> lcm;
  BotEventHandler *key_handler;
} RendererHumanoid;

static void
_renderer_free (BotRenderer *super)
{
  RendererHumanoid *self = (RendererHumanoid*) super->user;
  free(self);
}




//=========================key press================

int cb_key_press (BotViewer *viewer, BotEventHandler *ehandler, const GdkEventKey *event)
{
  switch (event->keyval)
    {

    case GDK_Right:
      {
	cout << "\n right key pressed" << endl;
      }
    }
  
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
  
bool debug_mesh_display =false;

  int type = link->type ;
  enum {SPHERE, BOX, CYLINDER, MESH}; 
  
  if (type == SPHERE)
    {
  glPushMatrix();
      shared_ptr<urdf::Sphere> sphere(shared_dynamic_cast<urdf::Sphere>(link));	
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
  

  }
  else if  (type == CYLINDER)
    {



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
    
/*size_t found1;
found1=nextLinkname.find("r_");
if (found1!=std::string::npos)
{*/
    glPushMatrix();
    glTranslatef(nextTf.tf.translation.x,
		 nextTf.tf.translation.y,
		 nextTf.tf.translation.z);
     glRotatef(theta * 180/3.141592654, 
       		axis[0], axis[1], axis[2]); 

	std::map<std::string, GLuint>::const_iterator mesh_map_it;
	mesh_map_it=self->robotStateListener->_mesh_map.find(nextLinkname);
  std::map<std::string, fk::MeshExtrema>::const_iterator mesh_ext_map_it;
	mesh_ext_map_it=self->robotStateListener->_mesh_extrema_map.find(nextLinkname);
	if(mesh_map_it!=self->robotStateListener->_mesh_map.end()) // exists in cache
	{ 
	   if(!debug_mesh_display)
	     glCallList (mesh_map_it->second);
	   else {
		double xDim = mesh_ext_map_it->second.span_x;
		double yDim = mesh_ext_map_it->second.span_y;
		double zDim = mesh_ext_map_it->second.span_z;

 	    // get the vertices for mesh_map_it->second
		glScalef(xDim,yDim,zDim);
		bot_gl_draw_cube();
		//std::cout << "Steven wants to see this thing on a terminal!" << std::endl;
 	   }	   

	}

    glPopMatrix();
//}
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
 
 double c[3] = {0.3,0.3,0.3};
 double alpha = 1;
  //glColor3f(c[0],c[1],c[2]);
  glColor4f(c[0],c[1],c[2],alpha);
  for(uint i = 0; i < link_tfs.size(); i++)
    {
      drc::link_transform_t nextTf = link_tfs[i];
      shared_ptr<urdf::Geometry> nextLink = link_shapes[i];
      string nextLinkname = link_names[i];
      draw(nextLink, nextTf,nextLinkname,super);
    }
    
  //glPopMatrix();

// int64_t toc = bot_timestamp_now();
// cout << bot_timestamp_useconds(toc-tic) << endl;
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

    bot_viewer_add_renderer(viewer, &self->renderer, render_priority);


    //----------
    // create and register mode handler
    self->key_handler = (BotEventHandler*) calloc(1, sizeof(BotEventHandler));
    self->key_handler->name = strdup(std::string("Mode Control").c_str());
    self->key_handler->enabled = 0;
    self->key_handler->key_press = cb_key_press;
    //self->key_handler->key_release = cb_key_release;
    self->key_handler->user = self;
    bot_viewer_add_event_handler(viewer, self->key_handler, 1);
    
}

void polygon(int a, int b, int c , int d)
{

 
    float vertices[][3] = 
    {
        {-0.5,-0.5,-0.5},{0.5,-0.5,-0.5},
        {0.5,0.5,-0.5}, {-0.5,0.5,-0.5}, {-0.5,-0.5,0.5}, 
        {0.5,-0.5,0.5}, {0.5,0.5,0.5}, {-0.5,0.5,0.5}
    };
 
    float colors[][3] = {{0.0,0.5,0.5},{1.0,0.0,0.0},
    {1.0,1.0,0.0}, {0.0,1.0,0.0}, {0.0,0.0,1.0}, 
    {1.0,0.0,1.0}, {1.0,1.0,1.0}, {0.0,1.0,1.0}};

    // draw a polygon using colour of first vertex
 
    glBegin(GL_POLYGON);
       // glColor3fv(colors[a]);
        //glColor3f(0.15,0.15,0.15);  
        glVertex3fv(vertices[a]);
        glVertex3fv(vertices[b]);
        glVertex3fv(vertices[c]);
        glVertex3fv(vertices[d]);
    glEnd();
}
 
void cube(void)
{
    //Draw unit cube centred on the origin
 
/* map vertices to faces */
 
    polygon(0,3,2,1);
    polygon(2,3,7,6);
    polygon(4,7,3,0);
    polygon(1,2,6,5);
    polygon(7,4,5,6);
    polygon(5,4,0,1);
}
