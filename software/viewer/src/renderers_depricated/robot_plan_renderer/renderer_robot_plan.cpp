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


#include "renderer_robot_plan.hpp"
#include "RobotPlanListener.hpp"


#define RENDERER_NAME "Robot Plan Display"
#define DRAW_PERSIST_SEC 4

using namespace std;
using namespace boost;
using namespace robot_plan_renderer;

typedef struct _RendererRobotPlan
{
  BotRenderer renderer;
  BotViewer          *viewer;
  BotGtkParamWidget *pw;
  boost::shared_ptr<fk::RobotPlanListener> robotPlanListener;
  boost::shared_ptr<lcm::LCM> lcm;
  int64_t max_draw_utime;
} RendererRobotPlan;

static void
_renderer_free (BotRenderer *super)
{
  RendererRobotPlan *self = (RendererRobotPlan*) super->user;
  free(self);
}


//=================================

static void draw(shared_ptr<urdf::Geometry> link, const drc::link_transform_t &nextTf)
{
  
 
  //--get rotation in angle/axis form
  double theta;
  double axis[3];
  double quat[4] = {nextTf.tf.rotation.w,
		    nextTf.tf.rotation.x,
		    nextTf.tf.rotation.y,
		    nextTf.tf.rotation.z};
  bot_quat_to_angle_axis(quat, &theta, axis);
  
  //---debugging
 /* cout << "\n(w,x,y,z) = (" 
       << nextTf.tf.rotation.w 
       << "," << nextTf.tf.rotation.x 
       << "," << nextTf.tf.rotation.y 
       << "," << nextTf.tf.rotation.z 
       << ")" << endl;
  cout << "\naxis = (" 
       << axis[0] << "," 
       << axis[1] << ","
       << axis[2] << ")" << endl;
  cout << "theta = " << theta << endl;*/

  //----
  
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
      
//        glPushMatrix();
//      glPointSize(radius);
//      //glColor3ub(0,1,0);
//      glBegin(GL_POINTS);
//      glVertex3f(radius, radius, radius);
//      glEnd();
//      glPopMatrix();

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
    //cout << "MESH"<< endl;
    //shared_ptr<urdf::Mesh> mesh(shared_dynamic_cast<urdf::Mesh>(it->second->visual->geometry));
    //renderMesh(mesh->filename)
  }
  else {
    //cout << "UNKNOWN"<< endl;
  }

  gluDeleteQuadric(quadric);
}


static void 
_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
  RendererRobotPlan *self = (RendererRobotPlan*) super->user;
  int64_t now = bot_timestamp_now();
  self->max_draw_utime = self->robotPlanListener->_last_plan_msg_timestamp  + DRAW_PERSIST_SEC * 1000000;	
  if(now > self->max_draw_utime)
    return; // clear robot plan display
  
  glEnable(GL_DEPTH_TEST);
  //
  vector<shared_ptr<urdf::Geometry> > link_shapes;
  vector<drc::link_transform_t> link_tfs;
  self->robotPlanListener->getState(link_shapes, link_tfs);
 //fk::RobotPlanListener::printTransforms(link_shapes, link_tfs);
  
  //-draw 
  //glPointSize(5.0f);
  //glBegin(GL_POINTS);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
 
 double c[3] = {0.3,0.3,0.6};
 double alpha = 0.2;
  //glColor3f(c[0],c[1],c[2]);
  glColor4f(c[0],c[1],c[2],alpha);
  for(uint i = 0; i < link_tfs.size(); i++)
    {
      drc::link_transform_t nextTf = link_tfs[i];
      shared_ptr<urdf::Geometry> nextLink = link_shapes[i];
      draw(nextLink, nextTf);
    }
    
  //glPopMatrix();
}

void 
setup_renderer_robot_plan(BotViewer *viewer, int render_priority, lcm_t *lcm)
{
    RendererRobotPlan *self = (RendererRobotPlan*) calloc (1, sizeof (RendererRobotPlan));
    self->lcm = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcm));
    self->robotPlanListener = boost::shared_ptr<fk::RobotPlanListener>(new fk::RobotPlanListener(self->lcm, 
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
    
}

