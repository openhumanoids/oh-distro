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
} RendererHumanoid;

static void
_renderer_free (BotRenderer *super)
{
  RendererHumanoid *self = (RendererHumanoid*) super->user;
  free(self);
}

static void draw(shared_ptr<urdf::Geometry> link)
{
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
      glPointSize(radius);
      glColor3ub(0,1,0);
      glBegin(GL_POINTS);
      glVertex3f(radius, radius, radius);
      glEnd();
    }
  else if  (type == BOX)
    {
    shared_ptr<urdf::Box> box(shared_dynamic_cast<urdf::Box>(link));
    double xDim = box->dim.x;
    double yDim = box->dim.y;
    double zDim = box->dim.z;

    //todo

  }
  else if  (type == CYLINDER)
    {
    shared_ptr<urdf::Cylinder> cyl(shared_dynamic_cast<urdf::Cylinder>(link));
    gluCylinder(quadric,
		cyl->radius,
		cyl->radius,
		(double) cyl->length,
		36,
		36);

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
  RendererHumanoid *self = (RendererHumanoid*) super->user;

  glEnable(GL_DEPTH_TEST);
  //
  vector<shared_ptr<urdf::Geometry> > link_shapes;
  vector<drc::link_transform_t> link_tfs;
  self->robotStateListener->getState(link_shapes, link_tfs);
  //fk::RobotStateListener::printTransforms(link_shapes, link_tfs);
  
  //-draw 
  //glPointSize(5.0f);
  //glBegin(GL_POINTS);
  glColor3ub(0,1,0);
  for(uint i = 0; i < link_tfs.size(); i++)
    {
      drc::link_transform_t nextTf = link_tfs[i];
      shared_ptr<urdf::Geometry> nextLink = link_shapes[i];

      //--get rotation in angle/axis form
      double theta;
      double axis[3];
      double quat[4] = {nextTf.tf.rotation.w,
			nextTf.tf.rotation.x,
			nextTf.tf.rotation.y,
			nextTf.tf.rotation.z};
      bot_quat_to_angle_axis(quat, &theta, axis);

      //draw
      glPushMatrix();
      glTranslatef(nextTf.tf.translation.x,
		   nextTf.tf.translation.y,
		   nextTf.tf.translation.z);
      glRotatef(theta, axis[0], axis[1], axis[2]);
      
      draw(nextLink);

      glPopMatrix();

    }
    
  //glPopMatrix();
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


    //----------lcm stuff
}
