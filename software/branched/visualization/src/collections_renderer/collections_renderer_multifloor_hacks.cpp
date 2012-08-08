/*
 * renders collections (landmarks, trajectory, measurements...)
 * @author Michael Kaess
 * @author Hordur Johannsson
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <map>
#include <vector>
#include <list>
#include <utility>
#include <string>
#include <algorithm>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <glib.h>
#include <glib-object.h>
#include <gtk/gtk.h>
#include <gtk/gtksignal.h>

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_core/bot_core.h>

#include "visualization/collections.hpp"

#define GROUND_LEVEL 0.0
#define RENDERER_NAME "Collections"

using namespace std;
using namespace Eigen;

const char* PARAM_USE_TIME_COLLECTION = "Elevation by collection";
const bool PARAM_USE_TIME_COLLECTION_DEFAULT = false;

const char* PARAM_FILL_SCANS = "Fill scans";
const bool PARAM_FILL_SCANS_DEFAULT = false;

const char* PARAM_ALPHA_POINTS = "Points alpha";
const double PARAM_ALPHA_POINTS_MIN = 0.;
const double PARAM_ALPHA_POINTS_MAX = 1.;
const double PARAM_ALPHA_POINTS_DELTA = 0.001;
const double PARAM_ALPHA_POINTS_DEFAULT = 1.;

const char* PARAM_POINT_WIDTH_POINTS = "Point width";
const double PARAM_POINT_WIDTH_POINTS_MIN = 1;
const double PARAM_POINT_WIDTH_POINTS_MAX = 30;
const double PARAM_POINT_WIDTH_POINTS_DELTA = 1;
const double PARAM_POINT_WIDTH_POINTS_DEFAULT = 1;

const char* PARAM_POSE_WIDTH_POSES = "Pose width";
const double PARAM_POSE_WIDTH_POSES_MIN = 1;
const double PARAM_POSE_WIDTH_POSES_MAX = 30;
const double PARAM_POSE_WIDTH_POSES_DELTA = 1;
const double PARAM_POSE_WIDTH_POSES_DEFAULT = 1;

const char* PARAM_COLOR_TIME = "Color by time";
const bool PARAM_COLOR_TIME_DEFAULT = false;

const char* PARAM_USE_TIME = "Elevation by time";
const bool PARAM_USE_TIME_DEFAULT = false;

const char* PARAM_TIME_SCALE = "Max Elevation";
const double PARAM_TIME_SCALE_MIN = -10.;
const double PARAM_TIME_SCALE_MAX = 100.;
const double PARAM_TIME_SCALE_DELTA = 0.5;
const double PARAM_TIME_SCALE_DEFAULT = 10.;

const char* PARAM_RANGE_START = "Start";
const double PARAM_RANGE_START_MIN = 0.;
const double PARAM_RANGE_START_MAX = 1.;
const double PARAM_RANGE_START_DELTA = 0.001;
const double PARAM_RANGE_START_DEFAULT = 0.;

const char* PARAM_RANGE_END = "End";
const double PARAM_RANGE_END_MIN = 0.;
const double PARAM_RANGE_END_MAX = 1.;
const double PARAM_RANGE_END_DELTA = 0.001;
const double PARAM_RANGE_END_DEFAULT = 1.;

const char* PARAM_Z_UP = "Z-axis up";
const bool PARAM_Z_UP_DEFAULT = true;
float colors[] = {
    51/255.0, 160/255.0, 44/255.0,
    166/255.0, 206/255.0, 227/255.0,
    178/255.0, 223/255.0, 138/255.0,
    31/255.0, 120/255.0, 180/255.0,
    251/255.0, 154/255.0, 153/255.0,
    227/255.0, 26/255.0, 28/255.0,
    253/255.0, 191/255.0, 111/255.0,
    106/255.0, 61/255.0, 154/255.0,
    255/255.0, 127/255.0, 0/255.0,
    202/255.0, 178/255.0, 214/255.0,
    1.0, 0.0, 0.0, // red
    0.0, 1.0, 0.0, // green
    0.0, 0.0, 1.0, // blue
    1.0, 1.0, 0.0,
    1.0, 0.0, 1.0,
    0.0, 1.0, 1.0,
    0.5, 1.0, 0.0,
    1.0, 0.5, 0.0,
    0.5, 0.0, 1.0,
    1.0, 0.0, 0.5,
    0.0, 0.5, 1.0,
    0.0, 1.0, 0.5,
    1.0, 0.5, 0.5,
    0.5, 1.0, 0.5,
    0.5, 0.5, 1.0,
    0.5, 0.5, 1.0,
    0.5, 1.0, 0.5,
    0.5, 0.5, 1.0
};

const int num_colors = sizeof(colors)/(3*sizeof(float));

class Collection {
public:
  int id;
  string name;
  int type;
  bool show;

  Collection(int id, string name, int type, bool show) : id(id), name(name), type(type), show(show) {}

  virtual ~Collection() {}
  virtual void draw(void *self, int64_t range_start, int64_t range_end) = 0;
  virtual void clear() = 0;
};

typedef map<int, Collection*> collections_t;

typedef struct _RendererCollections RendererCollections;

/**
 * A configuration block for a single collection
 */
class CollectionConfig
{
public:
  CollectionConfig();
  
  void set(const char* name, const char* value);
  void set(const string & name, const string & value);
  const string & get(const string & name);

  // TODO: helper function to read arrays, double, int, ...

  bool is_configured() {return m_is_configured;}
  bool has_value(const std::string & name);
private:
  bool m_is_configured;
  map<string, string> m_properties;
};

CollectionConfig::CollectionConfig() : m_is_configured(false) {}
void CollectionConfig::set(const char* name, const char* value)
{
  const std::string a(name);
  const std::string b(value);
  set(a,b);
}

void CollectionConfig::set(const std::string & name, const std::string & value)
{
  m_is_configured = true;
  m_properties[name] = value;
}

bool CollectionConfig::has_value(const std::string & name)
{
  map<string, string>::iterator it = m_properties.find(name);
  return it != m_properties.end();
}

const std::string & CollectionConfig::get(const std::string & name)
{
  return m_properties[name];
}

struct _RendererCollections {
  BotRenderer renderer;
  BotViewer *viewer;

  BotGtkParamWidget  *pw;

  lcm_t *lcm;

  collections_t collections;

  bool param_use_time;
  bool param_use_time_collection;
  double param_time_scale;
  double param_range_start;
  double param_range_end;
  bool param_fill_scans;
  double param_alpha_points;
  int param_point_width;
  int param_pose_width;
  bool param_color_time;
  bool param_z_up;
  
  bool toggle_onoff;

  GMutex* collectionsMutex;

  int64_t    obj_maxid;
  int64_t    obj_minid;

  // for custom checkboxes
  GtkWidget* vbox; 
  GtkWidget* collectionsbox;
  
  std::vector < GtkWidget* > checkbox_ptr;
};

typedef struct {
  RendererCollections* self;
  int id;
  
} checkbox_info_t;

// Config for the collections
std::map<int32_t, CollectionConfig> collectionConfig;

// helper function
/**
 * Combines time and collection elevation
 */
static double time_elevation(RendererCollections *self, int64_t id, double z, int collid) {
  double time_scale;
  time_scale =self->param_time_scale * self->param_pose_width;

  if (!self->param_use_time && !self->param_use_time_collection) return z;
	double newz = 0.0;
  if (self->param_use_time) {
    int64_t min_id = self->obj_minid;
    int64_t max_id = self->obj_maxid;

    newz += ((double)(id-min_id) / (double)(max_id-min_id) * time_scale);
  }
  if (self->param_use_time_collection) {
    newz += collid * time_scale;
  }
  return newz;
}

/**
 * Only look at collection elevation
 */
static double time_elevation_collection(RendererCollections *self, int64_t id, double z, int collid) {
  double time_scale;
  time_scale =self->param_time_scale * self->param_pose_width;

  if (!self->param_use_time_collection) return z;
	double newz = 0.0;
  if (self->param_use_time_collection) {
    newz += collid * time_scale;
  }
  return newz;
}

void draw_ellipsoid(RendererCollections *self, double x, double y, double z, int n_covs, double* covs, bool is3d) {
  // covs is xx,xy,xt,yy,yt,tt for 2D poses and xx,xy,yy for 2D points
  // for 2D poses we only use the translational part
  // note that z is set to 0.1, so that the ellipsoid actually shows up

  return  ;

  // TODO: add 2D case
  if (is3d) {
    // populate the matrix
    MatrixXd S(3,3);
    S << covs[0], covs[1], covs[2],
         covs[1], covs[3], covs[4],
         covs[2], covs[4], covs[5];

    // eigenvectors and eigenvalues
    EigenSolver<MatrixXd> es(S);
    double eval1 = es.eigenvalues()[0].real();
    double eval2 = es.eigenvalues()[1].real();
    double eval3 = es.eigenvalues()[2].real();
    VectorXd evec1 = es.eigenvectors().col(0).real();
    VectorXd evec2 = es.eigenvectors().col(1).real();
    VectorXd evec3 = es.eigenvectors().col(2).real();

    // draw ellipsoid
    double k = 1.;    // scale factor
    double radius1 = k * sqrt(eval1);
    double radius2 = k * sqrt(eval2);
    double radius3 = k * sqrt(eval3);
    const double max_radius = 6.;
    if (radius1<max_radius && radius2<max_radius && radius3<max_radius) {
      GLUquadricObj *quadric = gluNewQuadric();
      glPushMatrix();
      glTranslated(x,y,z);
      GLdouble rotation[16] = {
        evec1(0), evec1(1), evec1(2), 0.,
        evec2(0), evec2(1), evec2(2), 0.,
        evec3(0), evec3(1), evec3(2), 0.,
        0., 0., 0., 1.
      };
      glMultMatrixd(rotation);
      glScaled(radius1, radius2, radius3);
      gluQuadricDrawStyle(quadric, GLU_FILL);
      gluQuadricNormals(quadric, GLU_SMOOTH);
      gluSphere(quadric, 0.3, 10, 10);
      glPopMatrix();
    }
  }
}

static void draw_tree(RendererCollections *self, double x, double y, double z) {
  GLUquadricObj * quadric = gluNewQuadric (); // todo
  glPushMatrix();
  glTranslatef (x, y, z);
  glColor3f(139./255., 69./255., 19./255.);
  glPushMatrix();
  glTranslatef(0,0,0.01);
  gluCylinder(quadric, 0.2, 0.2, 2.6, 8, 8);
  glPopMatrix();
  glPushMatrix();
  glTranslatef(0,0,4.5);
  glColor3f(0.1,0.7,0.1);
  gluSphere(quadric, 2., 8, 8);
  glPopMatrix();
  glPopMatrix();
}

static void draw_axis(RendererCollections *self, double x, double y, double z, double yaw, double pitch, double roll, double size, bool mark) 
{
  glPushMatrix();
  glPushAttrib(GL_CURRENT_BIT);

  glTranslatef(x, y, z);

  glRotatef(bot_to_degrees(yaw),  0., 0., 1.);
  glRotatef(bot_to_degrees(pitch),0., 1., 0.);
  glRotatef(bot_to_degrees(roll), 1., 0., 0.);

  glBegin(GL_LINES);
    glColor3f(1.0,0.0,0.0); glVertex3f(0.0,0.0,0.0); glVertex3f(size*1.0,0.0,0.0);
    glColor3f(0.0,1.0,0.0); glVertex3f(0.0,0.0,0.0); glVertex3f(0.0,size*1.0,0.0);
    glColor3f(0.0,0.0,1.0); glVertex3f(0.0,0.0,0.0); glVertex3f(0.0,0.0,size*1.0);
  glEnd();

  if (mark) {
//    glutWireSphere(size*1.5, 5, 5);
  }

  glPopAttrib();
  // todo: reset color?
  glPopMatrix();
}

static void draw_tag(RendererCollections *self, double x, double y, double z, double yaw, double pitch, double roll) {
  glPushMatrix();
  glPushAttrib(GL_CURRENT_BIT);

  glTranslatef(x, y, z);

  glRotatef(bot_to_degrees(yaw),  0., 0., 1.);
  glRotatef(bot_to_degrees(pitch),0., 1., 0.);
  glRotatef(bot_to_degrees(roll), 1., 0., 0.);

  double size = 0.166;
  double h = size / 2.;

  glBegin(GL_QUADS);
#if 1
  double d = size / 8.;
  for (int i=0; i<8; i++) {
    double x = i*d - h;
    for (int j=0; j<8; j++) {
      if (i==0 || i==7 || j==0 || j==7 || (i%2)==(j%2)) {
        double y = j*d - h;
        glVertex3f(  x,   y, 0);
        glVertex3f(  x, y+d, 0);
        glVertex3f(x+d, y+d, 0);
        glVertex3f(x+d,   y, 0);
      }
    }
  }
#else
  glVertex3f(-h,-h,0);
  glVertex3f(-h, h,0);
  glVertex3f( h, h,0);
  glVertex3f( h,-h,0);
#endif
  glEnd();

  glPopAttrib();
  // todo: reset color?
  glPopMatrix();
}

static void draw_triangle(RendererCollections *self, double x, double y, double z, double theta, double size, bool mark) {
  if (!self->viewer) return;

  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(bot_to_degrees(theta),  0., 0., 1.);

  if (mark) {
//    glutWireSphere(size*1.5, 5, 5);
    glPushAttrib(GL_CURRENT_BIT);
    glPushMatrix();
    glScalef(2.0,2.0,1.0);
    glBegin(GL_LINE_LOOP);
    glColor3f(0.9,0.1,0.1); glVertex3f(size,0.0,0.1);
    glColor3f(0.9,0.1,0.1); glVertex3f(-size,size/2.0,0.1);
    glColor3f(0.9,0.1,0.1); glVertex3f(-size,-size/2.0,0.1);
    glEnd();
    glPopMatrix();
    glPopAttrib();
  }

  glBegin(GL_POLYGON);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();

  glPopMatrix();
}

static int min_z = 1000;
static int max_z = -1000;

static void draw_equilateral_triangle(RendererCollections *self, double x, double y, double z, double theta, double size, bool mark) {
  if (!self->viewer) return;

  glPushMatrix();
  glTranslatef(x, y, z);
  //  glRotatef(bot_to_degrees(theta),  0., 0., 1.);

  glPushAttrib(GL_ALL_ATTRIB_BITS);
  size = size * 10;
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  if (z < min_z) min_z = z;
  if (z > max_z) max_z = z;
  float a = (z - min_z)/(max_z - min_z);

  //glColor4f(0.7,0.7,0.9,0.7);
  Eigen::Vector3f max_color(0.3,0.3,0.9);
  Eigen::Vector3f min_color(0.9,0.1,0.1);
  Eigen::Vector3f c = min_color*a + max_color*(1.0-a);
  glColor4f(c(0), c(1), c(2), 0.7);

  glBegin(GL_POLYGON);
  glVertex3f(size, size, 0.0);
  glVertex3f(size, -size, 0.0);
  glVertex3f(-size, -size, 0.0);
  glVertex3f(-size, size, 0.0);
  glEnd();
  glPopAttrib();
  glPopMatrix();

  return ;


  double r = size;
  double b = -r / sqrt(3.0);
  double h = sqrt(3.0) * r;

  glBegin(GL_LINE_LOOP);
  glVertex3f(r, b, 0.0);
  glVertex3f(-r, b, 0.0);
  glVertex3f(0.0, h+b, 0.0);
  glEnd();

  glPopMatrix();
}

static void draw_hexagon(RendererCollections *self, double x, double y, double z, double theta, double size, bool mark) {
  if (!self->viewer) return;

  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(bot_to_degrees(theta),  0., 0., 1.);
  double r = size;

  glBegin(GL_POLYGON);

  for (int i = 0; i < 6; i++) {
    float a = i*M_PI/3.0;
    glVertex3f(r*cos(a), r*sin(a), 0.0);
  }

  glEnd();

  glPopMatrix();
}


static void draw_camera(RendererCollections *self, double x, double y, double z, double yaw, double pitch, double roll, double size, bool mark) {
  // @todo implement camera rendering

  if (!self->viewer) return;

  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(bot_to_degrees(yaw),  0., 0., 1.);
  glRotatef(bot_to_degrees(pitch),0., 1., 0.);
  glRotatef(bot_to_degrees(roll), 1., 0., 0.);

  if (mark) {
    // glutWireSphere(size*1.5, 5, 5);
  }

  // Depth, height and width of pyramid
  float d = 3;
  float h = 1;
  float w = 1.5;

  glScalef(size,size,size);
  glBegin(GL_LINES);
  // Draw sides
  glVertex3f(0,0,0); glVertex3f(h,  w, h);
  glVertex3f(0,0,0); glVertex3f(h, -w, h);
  glVertex3f(0,0,0); glVertex3f(h,  w, -h);
  glVertex3f(0,0,0); glVertex3f(h, -w, -h);

  // Draw base;
  glVertex3f(h,  w,  h); glVertex3f(h, -w,  h);
  glVertex3f(h, -w,  h); glVertex3f(h, -w, -h);
  glVertex3f(h, -w, -h); glVertex3f(h,  w, -h);
  glVertex3f(h,  w, -h); glVertex3f(h,  w,  h);
  glEnd();

  glPopMatrix();
}


static void draw_tetra(RendererCollections *self, double x, double y, double z, double yaw, double pitch, double roll, double size, bool mark) {
  if (!self->viewer) return;

  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(bot_to_degrees(yaw),  0., 0., 1.);
  glRotatef(bot_to_degrees(pitch),0., 1., 0.);
  glRotatef(bot_to_degrees(roll), 1., 0., 0.);

  if (mark) {
//    glutWireSphere(size*1.5, 5, 5);
  }
  glBegin(GL_POLYGON);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_POLYGON);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_POLYGON);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glEnd();
  glBegin(GL_POLYGON);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  // draw outline in black
  glPushAttrib(GL_CURRENT_BIT);
  glColor3f(0,0,0);
  glBegin(GL_LINE_LOOP);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glPopAttrib();
  // todo: reset color?
  glPopMatrix();
}

static void draw_square(RendererCollections *self, double x, double y, double z, double theta, double size) {
  if (!self->viewer) return;

  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(bot_to_degrees(theta),0.0,0.0,1.0);
  glBegin(GL_LINE_LOOP);
  glVertex3f(size,size,0.0);
  glVertex3f(-size,size,0.0);
  glVertex3f(-size,-size,0.0);
  glVertex3f(size,-size,0.0);
  glEnd();
  glPopMatrix();
}

class ObjCollection : public Collection {
public:
  typedef vs_obj_collection_t my_vs_collection_t;
  typedef vs_obj_t my_vs_t;

  // index can be time stamp (64 bit!)
  typedef map<int64_t, my_vs_t>  elements_t;

  int64_t maxid;

  elements_t elements;

  static int get_size(const my_vs_collection_t *msg) {
    return msg->nobjs;
  }

  static void copy(const my_vs_collection_t *msg, int i, elements_t& dst_map) {
    // KMeans estimate of /run20120510_1_stata2nd_placegraphs
    static double floor_position[] = {-6.2949,   -1.5858,   -0.3699,    4.6892,    9.2344,   13.0977,   16.5079,   20.5194,  23.9043,   26.5422};
//    static double floor_position[] = {-6.2949,   -1.5858,   0.3699,    4.6892,    9.2344,   13.0977,   16.5079,   20.5194,  23.9043,   26.5422};
//    static double floor_position[] = {-6.2949,   -1.5858,   -0.3699,    5.6892,    9.2344,   13.0977,   16.5079,   20.5194,  23.9043,   26.5422};
//    static double floor_position[] = {-6.2949,   -1.0,   0.6,    6.0,    9.2344,   13.0977,   16.5079,   20.5194,  23.9043,   26.5422};

    /*
    double z = floor_position[0];
    for (int j = 1; j < 10; ++j)
      if (abs(msg->objs[i].z - floor_position[j]) < abs(z - floor_position[j])) z = floor_position[j];
    */
    double z = msg->objs[i].z;
    if (z > 25.0) z = floor_position[9];
    else if (z > 22) z = floor_position[8];
    else if (z > 19) z = floor_position[7];
    else if (z > 15) z = floor_position[6];
    else if (z > 12) z = floor_position[5];
    else if (z > 7) z = floor_position[4];
    else if (z > 2) z = floor_position[3];
    else if (z > -1) z = floor_position[2];
    else if (z > -4) z= floor_position[1];
    else z = floor_position[0];

    dst_map[msg->objs[i].id] = msg->objs[i];
    dst_map[msg->objs[i].id].z = 8 * z;
    dst_map[msg->objs[i].id].roll = 0;
    dst_map[msg->objs[i].id].pitch = 0;
  }

  ObjCollection(int id, string name, int type, bool show) : Collection(id, name, type, show) {}
  virtual ~ObjCollection() {}

  virtual void draw(void *_self, int64_t range_start, int64_t range_end) {
    RendererCollections *self = (RendererCollections*) _self;
    // preparations

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glEnable(GL_DEPTH_TEST);

    switch(type) {
    case VS_OBJ_COLLECTION_T_TREE:
      glEnable(GL_RESCALE_NORMAL);
      glShadeModel(GL_SMOOTH);
      glEnable(GL_LIGHTING);
      glEnable(GL_COLOR_MATERIAL);
      glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
      break;
    }
    for (elements_t::iterator it = elements.begin(); it != elements.end(); it++) {
      vs_obj_t& obj = it->second;
      // only draw if within range
      if (obj.id>=range_start && obj.id<=range_end) {

        // Set color
        CollectionConfig & config = collectionConfig[id];
        bool isconf = config.is_configured();
        GLfloat color[4];
        if (isconf && config.has_value(std::string("Color"))) {
          std::string sColor = config.get("Color");
          sscanf(sColor.c_str(), "%f,%f,%f,%f", &(color[0]), &(color[1]), &(color[2]), &(color[3]));
        } else {
          color[0] = colors[3*(id%num_colors)];
          color[1] = colors[3*(id%num_colors)+1];
          color[2] = colors[3*(id%num_colors)+2];
          color[3] = colors[3*(id%num_colors)+3];
        }

        // glColor3fv(&colors[3*(id%num_colors)]);
        glColor4fv(color);

        double z = time_elevation(self, obj.id, obj.z, it->first);

        double size = 0.1; // 0.1m is the size of the plotted poses
	      size = size*self->param_pose_width;

        bool is_last = (maxid == obj.id);
        switch(type) {
        case VS_OBJ_COLLECTION_T_SQUARE:
          draw_square (self, obj.x, obj.y, z, obj.yaw, size);
          break;
        case VS_OBJ_COLLECTION_T_POSE:
          draw_triangle (self, obj.x, obj.y, z, obj.yaw, size, is_last);
          break;
        case VS_OBJ_COLLECTION_T_POSE3D:
          draw_tetra (self, obj.x, obj.y, z, obj.yaw, obj.pitch, obj.roll, size, is_last);
          break;
        case VS_OBJ_COLLECTION_T_AXIS3D:
          draw_axis (self, obj.x, obj.y, z, obj.yaw, obj.pitch, obj.roll, size, is_last);
          break;
        case VS_OBJ_COLLECTION_T_TREE:
          draw_tree (self, obj.x, obj.y, z);
          break;
        case VS_OBJ_COLLECTION_T_TAG:
          draw_tag (self, obj.x, obj.y, z, obj.yaw, obj.pitch, obj.roll);
          break;
        case VS_OBJ_COLLECTION_T_CAMERA:
          draw_camera (self, obj.x, obj.y, z, obj.yaw, obj.pitch, obj.roll, size, is_last);
          draw_axis(self, obj.x, obj.y, obj.z, obj.yaw, obj.pitch, obj.roll, size, is_last);
          break;
        case VS_OBJ_COLLECTION_T_TRIANGLE:
          draw_equilateral_triangle (self, obj.x, obj.y, z, obj.yaw, size, is_last );
          break;
        case VS_OBJ_COLLECTION_T_HEXAGON:
          draw_hexagon (self, obj.x, obj.y, z, obj.yaw, size, is_last);
          break;
        }
      }
    }
    glPopAttrib ();
  }
  virtual void clear() {
    elements.clear();
  }
};

class LinkCollection : public Collection {
public:
  typedef vs_link_collection_t my_vs_collection_t;
  typedef vs_link_t my_vs_t;

  // index can be time stamp (64 bit!)
  typedef map<int64_t, my_vs_t> elements_t;

  elements_t elements;

  static int get_size(const my_vs_collection_t *msg) {
    return msg->nlinks;
  }

  static void copy(const my_vs_collection_t *msg, int i, elements_t& dst_map) {
    dst_map[msg->links[i].id] = msg->links[i];
  }

  LinkCollection(int id, string name, int type, bool show) : Collection(id, name, type, show) {}
  virtual ~LinkCollection() {}

  virtual void draw(void *_self, int64_t range_start, int64_t range_end) {
    RendererCollections *self = (RendererCollections*) _self;
		
    CollectionConfig & config = collectionConfig[id];
    bool isconf = config.is_configured();

    glEnable(GL_DEPTH_TEST);

    GLfloat color[4];
    if (isconf && config.has_value(std::string("Color"))) {
      std::string sColor = config.get("Color");
      sscanf(sColor.c_str(), "%f,%f,%f,%f", &(color[0]), &(color[1]), &(color[2]), &(color[3]));
    } else {
      color[0] = colors[3*(id%num_colors)];
      color[1] = colors[3*(id%num_colors)+1];
      color[2] = colors[3*(id%num_colors)+2];
      color[3] = colors[3*(id%num_colors)+3];
    }
		
    for (elements_t::iterator it = elements.begin(); it != elements.end(); it++) {
      vs_link_t& link = it->second;
      collections_t::iterator collection_it1 = self->collections.find(link.collection1);
      collections_t::iterator collection_it2 = self->collections.find(link.collection2);
      if (collection_it1 != self->collections.end()
          && collection_it2 != self->collections.end()) {
        ObjCollection::elements_t& objs1 = ((ObjCollection*)collection_it1->second)->elements;
        ObjCollection::elements_t::iterator it1 = objs1.find(link.id1);
        ObjCollection::elements_t& objs2 = ((ObjCollection*)collection_it2->second)->elements;
        ObjCollection::elements_t::iterator it2 = objs2.find(link.id2);
        if (it1 != objs1.end() && it2 != objs2.end()) {
          vs_obj_t& obj1 = it1->second;
          vs_obj_t& obj2 = it2->second;
          // only draw if at least one end point is within the range
          if ((obj1.id>=range_start && obj1.id<=range_end)
              || (obj2.id>=range_start && obj2.id<=range_end)) {
            glColor3fv(color);

            glBegin(GL_LINES);
            double z1 = time_elevation(self, obj1.id, obj1.z, collection_it1->first);
            glVertex3f(obj1.x, obj1.y, z1);
            double z2 = time_elevation(self, obj2.id, obj2.z, collection_it2->first);
            glVertex3f(obj2.x, obj2.y, z2);
            glEnd();
          }
        }
      }
    }
  }
  virtual void clear() {
    elements.clear();
  }
};

class CovCollection : public Collection {
public:
  typedef vs_cov_collection_t my_vs_collection_t;
  typedef struct { // special case: flatten out data structure instead
                   // of separate block of memory for covariance
                   // matrix entries
    int32_t collection_id;
    int64_t element_id;
    int32_t n;
    double entries[6];
  } my_vs_t;

  // index can be time stamp (64 bit!)
  typedef map<int64_t, my_vs_t> elements_t;

  elements_t elements;

  static int get_size(const my_vs_collection_t *msg) {
    return msg->ncovs;
  }

  static void copy(const my_vs_collection_t *msg, int i, elements_t& dst_map) {
    my_vs_t& dst = dst_map[msg->covs[i].id];
    // @todo covariance can be at most 6 entries
    //       should we assert it isn't larger?
    dst.n = std::min(msg->covs[i].n, 6);
    for (int k=0; k<dst.n; k++) {
      dst.entries[k] = msg->covs[i].entries[k];
    }
    dst.collection_id = msg->covs[i].collection;
    dst.element_id = msg->covs[i].element_id;
  }

  CovCollection(int id, string name, int type, bool show) : Collection(id, name, type, show) {}
  virtual ~CovCollection()  { }

  virtual void draw(void *_self, int64_t range_start, int64_t range_end) {
    RendererCollections *self = (RendererCollections*) _self;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    for (elements_t::iterator it = elements.begin(); it != elements.end(); it++) {
      my_vs_t & cov = it->second;

      collections_t::iterator collection_it = self->collections.find(cov.collection_id);
      if (collection_it != self->collections.end()) {
        ObjCollection::elements_t& objs = ((ObjCollection*)collection_it->second)->elements;
        ObjCollection::elements_t::iterator it = objs.find(cov.element_id);
        if (it != objs.end()) {
          vs_obj_t& obj = it->second;
          // only draw if at least one end point is within the range
          if (obj.id>=range_start && obj.id<=range_end) {
            float alpha = 0.4;
            float* rgb = &colors[3*(id%num_colors)];
            glColor4f(rgb[0],rgb[1],rgb[2],alpha);

            bool is3d = false;
            if (((ObjCollection*)collection_it->second)->type & VS_OBJ_COLLECTION_T_AXIS3D
                || ((ObjCollection*)collection_it->second)->type & VS_OBJ_COLLECTION_T_POSE3D)
              is3d = true;
            draw_ellipsoid(self, obj.x, obj.y, obj.z, cov.n, cov.entries, is3d);
          }
        }
      }
    }

    glDisable(GL_BLEND);

  }
  virtual void clear() {
    elements.clear();
  }
};

class PointsCollection : public Collection {

  // Specifies how the point cloud should be rendered.
  // It can be one of the types in vs_points_collection_t
  GLenum mode;

public:
  typedef vs_point3d_list_collection_t my_vs_collection_t;
  
  typedef struct _my_vs_t{
    _my_vs_t() : entries(0), colors(0), normals(0), ncolors(0), nnormals(0), npoints(0) {}
    int32_t collection_id;
    int64_t element_id;
    int32_t ncolors;
    int32_t nnormals;
    int32_t npoints;
    float* entries;
    float* colors;
    float* normals;
  } my_vs_t;
  
  // index can be time stamp (64 bit!)
  typedef map<int64_t, my_vs_t> elements_t;

  elements_t elements;

  PointsCollection(int id, string name, int type, bool show) : Collection(id, name, type, show)
  {
    if (type == VS_POINT3D_LIST_COLLECTION_T_POINT) mode = GL_POINTS;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_POINT) mode = GL_LINE_STRIP;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_LINE_STRIP) mode = GL_LINE_STRIP;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_LINE_LOOP) mode = GL_LINE_LOOP;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_LINES) mode = GL_LINES;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_TRIANGLE_STRIP) mode = GL_TRIANGLE_STRIP;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_TRIANGLE_FAN) mode = GL_TRIANGLE_FAN;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_TRIANGLES) mode = GL_TRIANGLES;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_QUAD_STRIP) mode = GL_QUAD_STRIP;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_QUADS) mode = GL_QUADS;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_POLYGON) mode = GL_POLYGON;
  }
  
  virtual ~PointsCollection() {
    clear();
  }

  static int get_size(const my_vs_collection_t *msg) {
    return msg->nlists;
  }

  static void copy(const my_vs_collection_t *msg, int i, elements_t& dst_map) {
    my_vs_t& dst = dst_map[msg->point_lists[i].id];

    dst.collection_id = msg->point_lists[i].collection;
    dst.element_id = msg->point_lists[i].element_id;
    dst.ncolors = msg->point_lists[i].ncolors;
    dst.nnormals = msg->point_lists[i].nnormals;
    dst.npoints = msg->point_lists[i].npoints;
    delete [] dst.entries;
    delete [] dst.colors;
    delete [] dst.normals;

    vs_point3d_list_t* point_list = &(msg->point_lists[i]);

    // Store the origin allows us to do fan rendering (e.g. triangle fan)
    dst.colors = NULL;
    dst.normals = NULL;
    dst.entries = new float[point_list->npoints*3 + 3];

    dst.entries[0] = 0;
    dst.entries[1] = 0;
    dst.entries[2] = 0;

    if (point_list->ncolors == point_list->npoints) {
      dst.colors = new float[point_list->ncolors*4 + 4];
      dst.colors[0] = 0;
      dst.colors[1] = 0;
      dst.colors[2] = 0;
      dst.colors[3] = 0;
    }

    if (point_list->nnormals == point_list->npoints) {
      dst.normals = new float[point_list->nnormals*3 + 3];
      dst.normals[0] = 0;
      dst.normals[1] = 0;
      dst.normals[2] = 0;
    }

    // TODO: now we should be able to memcopy some of the data
    //       at least the float values
    for (int k=0; k<point_list->npoints; k++) {
      dst.entries[3*k + 0 + 3] = point_list->points[k].x;
      dst.entries[3*k + 1 + 3] = point_list->points[k].y;
      dst.entries[3*k + 2 + 3] = 1.0; // 0.5; // point_list->points[k].z;

      if (dst.colors)
      {
        dst.colors[4*k + 0 + 4] = 0.0; // point_list->colors[k].r;
        dst.colors[4*k + 1 + 4] = 0.0; // point_list->colors[k].g;
        dst.colors[4*k + 2 + 4] = 0.0; // point_list->colors[k].b;
        dst.colors[4*k + 3 + 4] = 0.5;
      }
      if (dst.normals)
      {
        dst.normals[3*k + 0 + 3] = point_list->normals[k].x;
        dst.normals[3*k + 1 + 3] = point_list->normals[k].y;
        dst.normals[3*k + 2 + 3] = point_list->normals[k].z;
      }
    }
  }

  virtual void draw(void *_self, int64_t range_start, int64_t range_end) {
    RendererCollections *self = (RendererCollections*) _self;
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnableClientState(GL_VERTEX_ARRAY);
    glPointSize((GLfloat)self->param_point_width);

    // Perform two passes.
    //  1. Fill in scans if enabled
    //  2. Draw points
    for (int pass=0; pass<=1; ++pass) {
    for (elements_t::iterator it = elements.begin(); it != elements.end(); it++) {
      my_vs_t & element = it->second;
      float* entries = it->second.entries;
      float* colors = it->second.colors;

      if (colors) {
        glEnableClientState(GL_COLOR_ARRAY);
        // @todo this could be done more efficiently
        //       if we update only when the alpha changes
        for (int k=0; k<element.ncolors;k++) {
          colors[4*k + 3 + 4] = self->param_alpha_points;
        }
      }

      collections_t::iterator collection_it = self->collections.find(element.collection_id);
      if (collection_it != self->collections.end()) {
        ObjCollection* objs_ptr = dynamic_cast<ObjCollection*>(collection_it->second);
        if (objs_ptr == 0) continue;
        ObjCollection::elements_t& objs = objs_ptr->elements;
        ObjCollection::elements_t::iterator obj_it = objs.find(element.element_id);
        if (obj_it != objs.end()) {
          vs_obj_t& obj = obj_it->second;
          if (obj.id>=range_start && obj.id<=range_end) {
            glPushMatrix();
            double z = time_elevation_collection(self, obj.id, obj.z, collection_it->first);
            float* rgb = &::colors[3*(id%num_colors)];
            float* rgb4 = &::colors[3*(id%num_colors+1)];
            float rgb2[] = {1,1,1};
            float rgb3[] = {1,0,0};
            float colmix = 1.0;
            if (self->param_color_time) {
              colmix = (double)(obj.id-self->obj_minid) / (double)(self->obj_maxid-self->obj_minid);
            }

            float rgbmix[3];
            rgbmix[0] =rgb2[0]*colmix+(1-colmix)*rgb3[0];
            rgbmix[1] =rgb2[1]*colmix+(1-colmix)*rgb3[1];
            rgbmix[2] =rgb2[2]*colmix+(1-colmix)*rgb3[2];

            float alpha = self->param_alpha_points; 

            glVertexPointer(3, GL_FLOAT, 0, entries);
            if (colors) glColorPointer(4, GL_FLOAT, 0, colors);
            if (pass==0 && self->param_fill_scans) {
//              if (!colors) glColor4f(rgbmix[0],rgbmix[1],rgbmix[2],1);
//              glDrawArrays(GL_TRIANGLE_FAN, 0, element.npoints+1);
                glPushMatrix();

                glTranslatef(obj.x, obj.y, z+0.1);
                glRotatef(bot_to_degrees(obj.yaw),0.0,0.0,1.0);
                glRotatef(bot_to_degrees(obj.pitch),0.0,1.0,0.0);
                glRotatef(bot_to_degrees(obj.roll),1.0,0.0,0.0);

		glColor4f(255,255,155,1.0);
		glLineWidth((GLfloat)self->param_point_width);

                float line_fan[2*element.npoints*3];
                for (int i=0; i<element.npoints; ++i) {
                   int pos = i*2*3;
                   if ((  entries[3 + i*3]*entries[3 + i*3] 
                        + entries[3 + i*3+1]*entries[3 + i*3+1] 
                        + entries[3 + i*3+2]*entries[3 + i*3+2]) < 30*30)
                   {
                       line_fan[pos    ] = 0; //entries[0];
                       line_fan[pos + 1] = 0; //entries[1];
                       line_fan[pos + 2] = 0; //entries[2];
                       line_fan[pos + 3] = entries[3 + i*3];
                       line_fan[pos + 4] = entries[3 + i*3 + 1];
                       line_fan[pos + 5] = entries[3 + i*3 + 2];
                   }
                }

                glVertexPointer(3, GL_FLOAT, 0, line_fan);
                glDrawArrays(GL_LINES, 0, 2*element.npoints);

                glVertexPointer(3, GL_FLOAT, 0, entries);
                glPopMatrix();
            }

            if (self->param_fill_scans) {
 	        rgb[0] = 0;
		rgb[1] = 0;
		rgb[2] = 0;
            }

            glTranslatef(obj.x, obj.y, z);
            glRotatef(bot_to_degrees(obj.yaw),0.0,0.0,1.0);
            glRotatef(bot_to_degrees(obj.pitch),0.0,1.0,0.0);
            glRotatef(bot_to_degrees(obj.roll),1.0,0.0,0.0);

            rgbmix[0] = rgb[0]*colmix+(1-colmix)*rgb4[0];
            rgbmix[1] = rgb[1]*colmix+(1-colmix)*rgb4[1];
            rgbmix[2] = rgb[2]*colmix+(1-colmix)*rgb4[2];

            //if (z>(self->param_time_scale-3) && z<(self->param_time_scale+3))
            {
              if (!colors) glColor4f(rgbmix[0],rgbmix[1],rgbmix[2],alpha);
              if (pass==1) glDrawArrays(mode, 1, element.npoints);
            }
            glPopMatrix();
          }
        }
      }
      if (colors) glDisableClientState(GL_COLOR_ARRAY);
    } // for each (element)
    } // for each (pass)
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisable(GL_BLEND);
    glPopAttrib();
  }

  virtual void clear() {
    for (elements_t::iterator it=elements.begin();it!=elements.end();++it) {
      my_vs_t& dst = it->second;
      delete [] dst.entries;
      delete [] dst.colors;
      delete [] dst.normals;
    }
    elements.clear();
  }
};

class TextCollection : public Collection {
public:
  typedef vs_text_collection_t my_vs_collection_t;

  typedef struct _my_vs_t{
    _my_vs_t() : collection_id(0), object_id(0) {}
    int32_t collection_id;
    int64_t object_id;
    std::string text;
  } my_vs_t;

  // index can be time stamp (64 bit!)
  typedef map<int64_t, my_vs_t> elements_t;

  elements_t elements;

  TextCollection(int id, string name, int type, bool show) : Collection(id, name, type, show)
  {
  }

  virtual ~TextCollection() {
    clear();
  }

  static int get_size(const my_vs_collection_t *msg) {
    return msg->n;
  }

  static void copy(const my_vs_collection_t *msg, int i, elements_t& dst_map) {
    my_vs_t& dst = dst_map[msg->texts[i].id];

    dst.collection_id = msg->texts[i].collection_id;
    dst.object_id = msg->texts[i].object_id;
    dst.text = std::string(msg->texts[i].text);
  }

  virtual void draw(void *_self, int64_t range_start, int64_t range_end) {
    RendererCollections *self = (RendererCollections*) _self;

    for (elements_t::iterator it = elements.begin(); it != elements.end(); it++) {
      my_vs_t & element = it->second;
      collections_t::iterator collection_it = self->collections.find(element.collection_id);
      if (collection_it != self->collections.end()) {
        ObjCollection* objs_ptr = dynamic_cast<ObjCollection*>(collection_it->second);
        if (objs_ptr == 0) continue;
        ObjCollection::elements_t& objs = objs_ptr->elements;
        ObjCollection::elements_t::iterator obj_it = objs.find(element.object_id);
        if (obj_it != objs.end()) {
          vs_obj_t& obj = obj_it->second;
          if (obj.id>=range_start && obj.id<=range_end) {
            glPushMatrix();
            double z = time_elevation_collection(self, obj.id, obj.z, collection_it->first);

            glColor3f(1,1,1);
            double pos[3];
            pos[0] = obj.x; pos[1] = obj.y; pos[2] = z;

            bot_gl_draw_text(pos, GLUT_BITMAP_HELVETICA_12, element.text.c_str(),
                             BOT_GL_DRAW_TEXT_DROP_SHADOW);


            glPopMatrix();
          }
        }
      }
    }
  }

  virtual void clear() {
    elements.clear();
  }
};


// Update min max values (only used ids from collections that are being displayed
void calculate_ranges(RendererCollections *self, int64_t& range_start, int64_t& range_end) {
  bool initialized = false;

  for (collections_t::iterator collection_it = self->collections.begin(); collection_it != self->collections.end(); collection_it++) {
    // ObjCollection?
    ObjCollection* obj_col = dynamic_cast<ObjCollection*>(collection_it->second);
    if (obj_col != NULL) {
      ObjCollection::elements_t& objs = obj_col->elements;
      if (!initialized) {
        self->obj_minid = self->obj_maxid = objs.begin()->second.id;
        initialized = true;
      }
      for (ObjCollection::elements_t::iterator it = objs.begin(); it != objs.end(); it++) {
        vs_obj_t& obj = it->second;
        if (it==objs.begin()) {
          obj_col->maxid = obj.id;
        }
        if (obj.id > self->obj_maxid) self->obj_maxid = obj.id;
        if (obj.id < self->obj_minid) self->obj_minid = obj.id;
        if (obj.id > obj_col->maxid) obj_col->maxid = obj.id;
      }
    }
  }
  double range = (double)(self->obj_maxid - self->obj_minid);
  range_start = self->obj_minid + (int64_t)(range*self->param_range_start);
  range_end   = self->obj_minid + (int64_t)(range*self->param_range_end);
}

static void draw_collections(RendererCollections *self) {
  if (!self)
    return;

  glPushMatrix();

  if (!self->param_z_up) {
    // Viewer is (x,y,z) = (forward,left,up) coordinates
    //		Collections are in (forward,right,down) coordinates
    //		We rotate 180 around x-axis
    glRotatef(180.0,1.0,0.0,0.0);
  }

  /// @todo have GROUND_LEVEL configurable
  glTranslatef(0.,0., GROUND_LEVEL);

  int64_t range_start;
  int64_t range_end;
  calculate_ranges(self, range_start, range_end);

  for (collections_t::iterator collection_it = self->collections.begin(); collection_it != self->collections.end(); collection_it++) {
    Collection* collection = collection_it->second;
    if (collection->show) {
      collection->draw(self, range_start, range_end);
    }
  }

  glPopMatrix ();
}

static void collections_draw(BotViewer *viewer, BotRenderer *renderer) {
  RendererCollections *self = (RendererCollections*) renderer->user;
  g_assert (self->viewer == viewer);
  g_mutex_lock(self->collectionsMutex);

  glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_LINE_BIT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LESS);
  glDisable (GL_LIGHTING);
  glLineWidth (2);

  draw_collections (self);

  glPopAttrib ();
  g_mutex_unlock(self->collectionsMutex);
}

static void on_checkbox_changed(GtkWidget *box, checkbox_info_t* info) {
  g_mutex_lock(info->self->collectionsMutex);

  bool is_checked = gtk_toggle_button_get_active((GtkToggleButton*)box);
  collections_t::iterator it = info->self->collections.find(info->id);
  if (it!=info->self->collections.end()) {
    it->second->show = is_checked;
  }

  g_mutex_unlock(info->self->collectionsMutex);

  bot_viewer_request_redraw (info->self->viewer);
}

static void add_checkbox(RendererCollections* self, const char* name, int id) {
  CollectionConfig & config = collectionConfig[id];
  bool isconf = config.is_configured();

  float color[4];

  if (isconf && config.has_value(std::string("Color"))) {
    std::string sColor = config.get("Color");
    sscanf(sColor.c_str(), "%f,%f,%f,%f", &(color[0]), &(color[1]), &(color[2]), &(color[3]));
  } else {
    color[0] = colors[3*(id%num_colors)];
    color[1] = colors[3*(id%num_colors)+1];
    color[2] = colors[3*(id%num_colors)+2];
    color[3] = colors[3*(id%num_colors)+3];
  } 

  char markup[50];
  PangoColor pc;
  pc.red   = color[0]*65535;
  pc.green = color[1]*65535;
  pc.blue  = color[2]*65535;
  // trick to get colored box
  gchar* pango_color = pango_color_to_string(&pc);
  snprintf(markup, sizeof(markup), "<span background=\"%s\">     </span> ", pango_color);
  GtkBox *hb = GTK_BOX (gtk_hbox_new (FALSE, 0));
  GtkWidget *label = gtk_label_new(NULL);
  gtk_label_set_markup (GTK_LABEL (label), markup);
  gtk_box_pack_start (GTK_BOX (hb), label, FALSE, FALSE, 0);
  GtkWidget *cb = gtk_check_button_new_with_label(name);
  gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (cb), TRUE);

  gtk_box_pack_start (GTK_BOX (hb), cb, FALSE, FALSE, 0);

  // @todo need to cleanup when button is destroyed
  checkbox_info_t* info = new checkbox_info_t;
  info->self = self;
  info->id = id;
  
  // added by mfallon: TODO: check if this vector needs to be cleared when doing "clear all"
  self->checkbox_ptr.push_back(cb); 
  
  g_signal_connect (G_OBJECT(cb), "toggled", G_CALLBACK (on_checkbox_changed), info);
  gtk_widget_show_all (GTK_WIDGET (hb));
  gtk_box_pack_start (GTK_BOX(self->collectionsbox), GTK_WIDGET(hb), TRUE, TRUE, 0);
}

static void collections_free(BotRenderer *renderer) {
  if (!renderer)
    return;

  RendererCollections *self = (RendererCollections*)renderer->user;
 
  if (!self)
    return;

  free (self);
}

static void reset_collections(collections_t* collections)
{
  for (collections_t::iterator it = collections->begin(); it!=collections->end(); it++) {
    Collection* collection = it->second;
    collection->clear();
  }
}
// remove all content from all collections (they stay active for now
// as otherwise they would also have to be removed from the user interface)
static void on_reset_collections_data(const lcm_recv_buf_t *rbuf, const char *channel, 
                                      const vs_reset_collections_t *msg, void *user_data) {
  RendererCollections *self = (RendererCollections*) user_data;

  g_mutex_lock(self->collectionsMutex);
  reset_collections(&self->collections);
  
  // added by mfallon:
  self->checkbox_ptr.clear();
  g_mutex_unlock(self->collectionsMutex);
}

// general method for parsing different collection messages, and
// adding to local data structures as well as GUI
template <class MyCollection>
static void on_collection_data(const lcm_recv_buf_t *rbuf, const char *channel, 
                               const typename MyCollection::my_vs_collection_t *msg, void *user_data ) {
  RendererCollections *self = (RendererCollections*) user_data;
  collections_t* collections = &self->collections;

  g_mutex_lock(self->collectionsMutex);

  // find object collection, create new one if necessary, update record
  collections_t::iterator collection_it = collections->find(msg->id);
  if (collection_it==collections->end()) {
    MyCollection* collection = new MyCollection(msg->id, msg->name, msg->type, true);
    collections->insert(make_pair(msg->id, collection));
    collection_it = collections->find(msg->id);
    // also add new menu entry for trajectory
    add_checkbox(self, msg->name, msg->id);
  }
  Collection* collection = collection_it->second;

  // update objs
  typename MyCollection::elements_t& elements = dynamic_cast<MyCollection*>(collection)->elements;
  if (msg->reset) {
    collection->clear();
  }
  for (int i=0; i<MyCollection::get_size(msg); i++) {
    MyCollection::copy(msg, i, elements);
  }
  g_mutex_unlock(self->collectionsMutex);

  bot_viewer_request_redraw (self->viewer);
}

static void on_obj_collection_data(const lcm_recv_buf_t *rbuf, const char *channel, 
                                   const vs_obj_collection_t *msg, void *user_data ) {
  on_collection_data<ObjCollection>(rbuf, channel, msg, user_data);
}

static void on_link_collection_data(const lcm_recv_buf_t *rbuf, const char *channel, 
                                    const vs_link_collection_t *msg, void *user_data ) {
  on_collection_data<LinkCollection>(rbuf, channel, msg, user_data);
}

static void on_cov_collection_data(const lcm_recv_buf_t *rbuf, const char *channel, 
                                   const vs_cov_collection_t *msg, void *user_data ) {
  on_collection_data<CovCollection>(rbuf, channel, msg, user_data);
}

static void on_points_collection_data(const lcm_recv_buf_t *rbuf, const char *channel, 
                                   const vs_point3d_list_collection_t *msg, void *user_data ) {
  on_collection_data<PointsCollection>(rbuf, channel, msg, user_data);
}

static void on_text_collection_data(const lcm_recv_buf_t *rbuf, const char *channel,
                                   const vs_text_collection_t *msg, void *user_data ) {
  on_collection_data<TextCollection>(rbuf, channel, msg, user_data);
}

static void on_collection_config(const lcm_recv_buf_t *rbuf, const char *channel,
                                 const vs_collection_config_t *msg, void *user_data ) {
  RendererCollections *self = (RendererCollections*) user_data;

  g_mutex_lock(self->collectionsMutex);

  // Find config block
  CollectionConfig & config = collectionConfig[msg->collection_id];

  // Update config
  for (int i=0; i<msg->n; i++) {
    config.set(msg->properties[i].name, msg->properties[i].value);
  }

  g_mutex_unlock(self->collectionsMutex);

}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *param, void *user_data) {
  RendererCollections *self = (RendererCollections*) user_data;

  self->param_use_time = bot_gtk_param_widget_get_bool(self->pw, PARAM_USE_TIME);
  self->param_use_time_collection = bot_gtk_param_widget_get_bool(self->pw, PARAM_USE_TIME_COLLECTION);
  self->param_time_scale = bot_gtk_param_widget_get_double(self->pw, PARAM_TIME_SCALE);
  self->param_range_start = bot_gtk_param_widget_get_double(self->pw, PARAM_RANGE_START);
  self->param_range_end = bot_gtk_param_widget_get_double(self->pw, PARAM_RANGE_END);

  self->param_alpha_points = bot_gtk_param_widget_get_double(self->pw, PARAM_ALPHA_POINTS);
  self->param_fill_scans = bot_gtk_param_widget_get_bool(self->pw, PARAM_FILL_SCANS);
  self->param_color_time = bot_gtk_param_widget_get_bool(self->pw, PARAM_COLOR_TIME);
  self->param_point_width = (int)bot_gtk_param_widget_get_double(self->pw, PARAM_POINT_WIDTH_POINTS);
  self->param_pose_width = (int)bot_gtk_param_widget_get_double(self->pw, PARAM_POSE_WIDTH_POSES);  
  
  self->param_z_up = bot_gtk_param_widget_get_bool(self->pw, PARAM_Z_UP);

  bot_viewer_request_redraw (self->viewer);
}

//static void on_clear_widget(GtkWidget *collection, RendererCollection *self)
static void on_clear_widget(GtkWidget *collection, void *user_data)
{
  RendererCollections *self = (RendererCollections*) user_data;

  gtk_widget_destroy(collection);
}

static void on_clear_button(GtkWidget *button, RendererCollections *self)
{
  g_mutex_lock(self->collectionsMutex);

  collections_t & collections = self->collections;
  for (collections_t::iterator it = collections.begin(); it!=collections.end(); it++) {
    Collection* collection = it->second;
    collection->clear();
    delete collection;
    it->second = 0;
  }
  collections.clear();
  gtk_container_foreach(GTK_CONTAINER(self->collectionsbox), on_clear_widget, self);

  self->checkbox_ptr.clear();
  
  g_mutex_unlock(self->collectionsMutex);

  bot_viewer_request_redraw (self->viewer);
}


// added by mfallon: to toggle all collections off/on
static void on_toggle_button(GtkWidget *button, RendererCollections *self)
{
  g_mutex_lock(self->collectionsMutex);
  collections_t & collections = self->collections;
  for (collections_t::iterator it = collections.begin(); it!=collections.end(); it++) {
    it->second->show = self->toggle_onoff;
  }
  g_mutex_unlock(self->collectionsMutex);

  // Force the buttons to change: (mutex must be released)
  for (int i=0;i<self->checkbox_ptr.size();i++){
    GtkWidget *button = self->checkbox_ptr[i];
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON(button), self->toggle_onoff);
  }
  self->toggle_onoff = !self->toggle_onoff;
  bot_viewer_request_redraw (self->viewer);
}

static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
  RendererCollections *self = (RendererCollections*) user_data;
  bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
  RendererCollections *self = (RendererCollections*) user_data;
  bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, RENDERER_NAME);
}

BotRenderer *renderer_collections_new (BotViewer *viewer, int render_priority) {
  RendererCollections *self = (RendererCollections*) calloc (1, sizeof (RendererCollections));
  g_assert (self);

  self->viewer = viewer;
  self->renderer.draw = collections_draw;
  self->renderer.destroy = collections_free;
  self->renderer.name = (char*)"Collections";
  self->renderer.user = self;
  self->renderer.enabled = 1;

  self->collectionsMutex = g_mutex_new();

  self->lcm = bot_lcm_get_global(NULL);

  /* subscribe to LCM streams */
  vs_obj_collection_t_subscribe(self->lcm, "OBJ_COLLECTION", on_obj_collection_data, self);
  vs_link_collection_t_subscribe(self->lcm, "LINK_COLLECTION", on_link_collection_data, self);
  vs_cov_collection_t_subscribe(self->lcm, "COV_COLLECTION", on_cov_collection_data, self);
  vs_point3d_list_collection_t_subscribe(self->lcm, "POINTS_COLLECTION", on_points_collection_data, self);
  vs_text_collection_t_subscribe(self->lcm, "TEXT_COLLECTION", on_text_collection_data, self);
  vs_reset_collections_t_subscribe(self->lcm, "RESET_COLLECTIONS", on_reset_collections_data, self);
  vs_collection_config_t_subscribe(self->lcm, "COLLECTION_CONFIG", on_collection_config, self);

  self->collections.clear();

  self->param_use_time    = PARAM_USE_TIME_DEFAULT;
  self->param_use_time_collection    = PARAM_USE_TIME_COLLECTION_DEFAULT;
  self->param_time_scale  = PARAM_TIME_SCALE_DEFAULT;
  self->param_range_start = PARAM_RANGE_START_DEFAULT;
  self->param_range_end   = PARAM_RANGE_END_DEFAULT;
  self->param_fill_scans = PARAM_FILL_SCANS_DEFAULT;
  self->param_alpha_points = PARAM_ALPHA_POINTS_DEFAULT;
  self->param_color_time = PARAM_COLOR_TIME_DEFAULT;
  self->param_point_width = PARAM_POINT_WIDTH_POINTS_DEFAULT;
  self->param_pose_width = PARAM_POSE_WIDTH_POSES_DEFAULT;
  self->param_z_up = PARAM_Z_UP_DEFAULT;

  if (viewer) {
    self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);

    self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
    GtkWidget *vbox = gtk_vbox_new (FALSE, 0);
    self->vbox = vbox;
    gtk_container_add (GTK_CONTAINER (self->renderer.widget), vbox);
    gtk_widget_show (vbox);

    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET (self->pw), 
                        FALSE, TRUE, 0);

    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_USE_TIME, PARAM_USE_TIME_DEFAULT, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_USE_TIME_COLLECTION, PARAM_USE_TIME_COLLECTION_DEFAULT, NULL);

    bot_gtk_param_widget_add_double(self->pw, PARAM_TIME_SCALE,
                                    BOT_GTK_PARAM_WIDGET_SLIDER,
                                    PARAM_TIME_SCALE_MIN, PARAM_TIME_SCALE_MAX,
                                    PARAM_TIME_SCALE_DELTA,
                                    PARAM_TIME_SCALE_DEFAULT);
    bot_gtk_param_widget_add_double(self->pw, PARAM_RANGE_START,
                                    BOT_GTK_PARAM_WIDGET_SLIDER,
                                    PARAM_RANGE_START_MIN, PARAM_RANGE_START_MAX,
                                    PARAM_RANGE_START_DELTA,
                                    PARAM_RANGE_START_DEFAULT);
    bot_gtk_param_widget_add_double(self->pw, PARAM_RANGE_END,
                                    BOT_GTK_PARAM_WIDGET_SLIDER,
                                    PARAM_RANGE_END_MIN, PARAM_RANGE_END_MAX,
                                    PARAM_RANGE_END_DELTA,
                                    PARAM_RANGE_END_DEFAULT);

    bot_gtk_param_widget_add_double(self->pw, PARAM_ALPHA_POINTS,
                                    BOT_GTK_PARAM_WIDGET_SLIDER,
                                    PARAM_ALPHA_POINTS_MIN, PARAM_ALPHA_POINTS_MAX,
                                    PARAM_ALPHA_POINTS_DELTA,
                                    PARAM_ALPHA_POINTS_DEFAULT);
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_FILL_SCANS, PARAM_FILL_SCANS_DEFAULT, NULL);
    bot_gtk_param_widget_add_double(self->pw, PARAM_POINT_WIDTH_POINTS,
                                    BOT_GTK_PARAM_WIDGET_SLIDER,
                                    PARAM_POINT_WIDTH_POINTS_MIN, PARAM_POINT_WIDTH_POINTS_MAX,
                                    PARAM_POINT_WIDTH_POINTS_DELTA,
                                    PARAM_POINT_WIDTH_POINTS_DEFAULT);
    bot_gtk_param_widget_add_double(self->pw, PARAM_POSE_WIDTH_POSES,
                                    BOT_GTK_PARAM_WIDGET_SLIDER,
                                    PARAM_POSE_WIDTH_POSES_MIN, PARAM_POSE_WIDTH_POSES_MAX,
                                    PARAM_POSE_WIDTH_POSES_DELTA,
                                    PARAM_POSE_WIDTH_POSES_DEFAULT);
    
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_COLOR_TIME, PARAM_COLOR_TIME_DEFAULT, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_Z_UP, PARAM_Z_UP_DEFAULT, NULL);

    GtkWidget *clear_button = gtk_button_new_with_label("Clear All");
    gtk_box_pack_start(GTK_BOX(self->pw), clear_button, FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(clear_button), "clicked", G_CALLBACK(on_clear_button), self);

    GtkWidget *toggle_button = gtk_button_new_with_label("Toggle All");
    gtk_box_pack_start(GTK_BOX(self->pw), toggle_button, FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(toggle_button), "clicked", G_CALLBACK(on_toggle_button), self);
    
    
    gtk_widget_show_all (GTK_WIDGET (self->pw));

    g_signal_connect (G_OBJECT (self->pw), "changed",
                      G_CALLBACK (on_param_widget_changed), self);
    
    // mfallon, save widget modes:
    g_signal_connect (G_OBJECT (viewer), "load-preferences",
                G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
                G_CALLBACK (on_save_preferences), self);

    GtkWidget *collectionsbox = gtk_vbox_new (FALSE, 0);
    self->collectionsbox = collectionsbox;
    gtk_container_add (GTK_CONTAINER (vbox), collectionsbox);
    gtk_widget_show (collectionsbox);

  }

  return &self->renderer;
}

void collections_add_renderer_to_viewer(BotViewer *viewer, int render_priority)
{
  bot_viewer_add_renderer(viewer, renderer_collections_new(viewer, render_priority), render_priority);
}

/*
 * setup_renderer:
 * Generic renderer add function for use as a dynamically loaded plugin
 */
extern "C" void add_renderer_to_plugin_viewer(BotViewer *viewer, int render_priority){
  collections_add_renderer_to_viewer(viewer, render_priority);
}

