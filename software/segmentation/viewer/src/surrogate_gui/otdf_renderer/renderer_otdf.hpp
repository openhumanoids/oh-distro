#ifndef __renderer_otdf_h__
#define __renderer_otdf_h__

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <string> 
#include <vector>
#include <math.h>
#include <fstream>
#include <map>
#include <boost/shared_ptr.hpp>

#include <errno.h>
#include <dirent.h>


#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <OpenGL/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>


#include <bot_vis/bot_vis.h>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>


#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <lcmtypes/bot_core.h>

#include <path_util/path_util.h>

#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <forward_kinematics/treefksolverposfull_recursive.hpp>
#include <otdf_parser/otdf_parser.h>
#include <otdf_parser/otdf_urdf_converter.h>


#define RENDERER_NAME "OTDF"
#define PARAM_MANAGE_INSTANCES "Manage Instances"
//#define PARAM_ADJUST_PARAM "Adjust Params"
#define PARAM_OTDF_SELECT "Template"
#define PARAM_OTDF_INSTANCE_SELECT "Instance"
#define PARAM_OTDF_ADJUST_PARAM "Adjust Params"
#define PARAM_OTDF_ADJUST_DOF "Adjust DoFs"
#define PARAM_OTDF_INSTANCE_CLEAR "Clear Instance"
#define PARAM_OTDF_INSTANCE_CLEAR_ALL "Clear All"
#define PARAM_OTDF_REACH_OBJECT_L "Reach Object w Left Arm"
#define PARAM_OTDF_REACH_OBJECT_R "Reach Object w Right Arm"
#define PARAM_INSTANTIATE "Instantiate/Fit"
#define PARAM_CLEAR "Clear All Instances"
#define DRAW_PERSIST_SEC 4
#define VARIANCE_THETA (30.0 * 180.0 / M_PI);
//Not sure why abe has this high a variance for the angle //((2*M_PI)*(2*M_PI))
#define MIN_STD 0.3
#define MAX_STD INFINITY

////////////////// THE FOLLOWING CODE WAS COPIED IN HERE TO AVOID
////////////////// DEPENDENCY WITH THE COMMON_UTILS/GEOM_UTILS POD [MFALLON]
#define GEOM_EPSILON 1e-9

using namespace std;

namespace otdf_renderer{

// ===== 2 dimensional structure =====
#ifndef _point2d_t_h
typedef struct _point2d {
  double x;
  double y;
} point2d_t;
#endif

// ===== 3 dimensional strucutres =====
// double 
typedef struct _point3d {
  double x;
  double y;
  double z;
} point3d_t;

#define point3d_as_array(p) ((double*)p)

/* The magic below allows you to use the POINT3D() macro to convert a
 * double[3] to a point3d_t.  gcc is smart -- if you try to cast anything
 * other than a point3d_t or a double[] with this macro, gcc will emit
 * a warning. */
union _point3d_any_t {
  point3d_t point;
  double array[3];
};

typedef point3d_t vec3d_t;

#define POINT3D(p) (&(((union _point3d_any_t *)(p))->point))


typedef struct _OtdfInstanceStruc {
    boost::shared_ptr<otdf::ModelInterface> _otdf_instance;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> _fksolver;
    std::vector<boost::shared_ptr<otdf::Geometry> > _link_shapes;
    std::vector<drc::link_transform_t> _link_tfs;
}OtdfInstanceStruc;   

class AffordanceCollectionListener;
class RobotStateListener;
 
typedef struct _RendererOtdf {
  BotRenderer renderer;
  BotEventHandler ehandler;
  BotViewer *viewer;
  boost::shared_ptr<lcm::LCM> lcm;

  BotGtkParamWidget *pw;

  int dragging;
  int active; //1 = relocalize, 2 = set person location
  int last_active; //1 = relocalize, 2 = set person location
  point2d_t drag_start_local;
  point2d_t drag_finish_local;

  point2d_t click_pos;
  double theta;
  double goal_std;
  int otdf_id;

  int64_t max_draw_utime;
  double circle_color[3];

  int num_otdfs;
  char ** otdf_names;
  int * otdf_nums;
  
  std::string* otdf_dir_name_ptr;
  std::vector<std::string> otdf_filenames; // itdf_template_names
  std::map<std::string, int > instance_cnt; // templateName, value.
 
  std::string* instance_selection_ptr; 
  std::map<std::string, OtdfInstanceStruc > instantiated_objects; // templatename+"ID:, object
  boost::shared_ptr<AffordanceCollectionListener> affordanceMsgHandler;
  boost::shared_ptr<RobotStateListener> robotStateListener;
}RendererOtdf;

}//end_namespace

void setup_renderer_otdf(BotViewer *viewer, int render_priority, boost::shared_ptr<lcm::LCM> lcm);


/**
 * @}
 */


#endif
