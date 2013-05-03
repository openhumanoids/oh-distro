#ifndef RENDERER_AFFORDANCES_HPP
#define RENDERER_AFFORDANCES_HPP

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
#include <boost/function.hpp>
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
#include <visualization_utils/GlKinematicBody.hpp>
#include <visualization_utils/InteractableGlKinematicBody.hpp>
#include <visualization_utils/angles.hpp>
#include <visualization_utils/eigen_kdl_conversions.hpp>
#include <visualization_utils/file_access_utils.hpp>
#include "BatchFKQueryHandler.hpp"

#define RENDERER_NAME "Affordances & StickyHands/Feet"
#define PARAM_MANAGE_INSTANCES "Manage Instances"
#define PARAM_SHOW_MESH "Show mesh"
#define PARAM_SHOW_BOUNDING_BOX "Show bounding box"
#define PARAM_REACHABILITY_FILTER "Enable Reachability Filter"
//#define PARAM_ADJUST_PARAM "Adjust Params"
#define PARAM_OTDF_SELECT "Template"
#define PARAM_OTDF_INSTANCE_SELECT "Instance"
#define PARAM_OTDF_ADJUST_PARAM "Adjust Params"
#define PARAM_OTDF_ADJUST_DOF "Adjust DoFs"
#define PARAM_OTDF_INSTANCE_CLEAR "Clear Instance"
#define PARAM_OTDF_INSTANCE_CLEAR_ALL "Clear All"
#define PARAM_INSTANTIATE "Instantiate/Fit"
#define PARAM_CLEAR "Clear All Instances"
#define PARAM_SELECTION "Enable Selection"
#define PARAM_OPT_POOL_READY "OptPool Ready"
#define PARAM_LHAND_URDF_SELECT "LHand"
#define PARAM_RHAND_URDF_SELECT "RHand"

#define DRAW_PERSIST_SEC 4
#define VARIANCE_THETA (30.0 * 180.0 / M_PI);
//Not sure why abe has this high a variance for the angle //((2*M_PI)*(2*M_PI))
#define MIN_STD 0.3
#define MAX_STD INFINITY

////////////////// THE FOLLOWING CODE WAS COPIED IN HERE TO AVOID
////////////////// DEPENDENCY WITH THE COMMON_UTILS/GEOM_UTILS POD [MFALLON]
#define GEOM_EPSILON 1e-9

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;

namespace renderer_affordances {

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


struct OtdfInstanceStruc {

    OtdfInstanceStruc()
    {
     otdf_instance_viz_object_sync = true; 
    };
    
     ~OtdfInstanceStruc()
    {

    };

    std::string otdf_type;
    int uid;
    boost::shared_ptr<otdf::ModelInterface> _otdf_instance;
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_object;
    boost::shared_ptr<collision::Collision_Detector> _collision_detector;  
    // Each object has its own collision detector for now. 
    // Otherwise, we need to manage a global collision detector by add and removing links whenever an object is deleted or added.
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3i> triangles;

    // filename of model used to generate points or triangles (alternative to points/triangles in aff message)
    std::string modelfile;

    // bounding box info relative to object
    Eigen::Vector3f boundingBoxXYZ;
    Eigen::Vector3f boundingBoxRPY;
    Eigen::Vector3f boundingBoxLWH;
    
    // otdf instance is in sync with aff server. 
    // Turn off if one needs to visualize state changes before committing to aff server.    
    bool otdf_instance_viz_object_sync;


};   

struct StickyHandStruc {

    StickyHandStruc()
    {
     grasp_status = 0;
     motion_trail_log_enabled = true;
    };
    
     ~StickyHandStruc()
    {

    };
    
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_hand;
    boost::shared_ptr<collision::Collision_Detector> _collision_detector;
    string object_name;
    string geometry_name; 
    int hand_type; //SANDIA_LEFT=0, SANDIA_RIGHT=1, SANDIA_BOTH=2, IROBOT_LEFT=3, IROBOT_RIGHT=4, IROBOT_BOTH=5;
    KDL::Frame T_geometry_hand; // this is stored in obj frame
    std::vector<std::string> joint_name;
    std::vector<double> joint_position;
    int uid;
    int opt_status;//RUNNING=0, SUCCESS=1, FAILURE=2;
    int grasp_status;//CANDIDATE=0,COMMITTED=1;
    bool motion_trail_log_enabled;
};   


struct StickyFootStruc {

    StickyFootStruc()
    {
     motion_trail_log_enabled = true;
    };
    
     ~StickyFootStruc()
    {

    };
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_foot;
    boost::shared_ptr<collision::Collision_Detector> _collision_detector;
    string object_name;
    string geometry_name; 
    int foot_type; //LEFT=0, RIGHT=1;
    KDL::Frame T_geometry_foot; // this is stored in obj frame
    std::vector<std::string> joint_name;
    std::vector<double> joint_position;
    int uid;
    bool motion_trail_log_enabled;
   // int opt_status;
}; 

class AffordanceCollectionListener;
class RobotStateListener;
class InitGraspOptPublisher;
class CandidateGraspSeedListener;
class GraspOptStatusListener;
class CandidateFootStepSeedManager;
class ReachabilityVerifier;
 
struct RendererAffordances {
  RendererAffordances()
  {
  // initializing variables to prevent memory issues due to uninitialized vars.
    viewer = NULL;
    pw = NULL;
    otdf_names = NULL;
    otdf_nums = NULL;
    dblclk_popup= NULL;
    second_stage_popup= NULL;
    
    
    showMesh = false;
    showBoundingBox = false;
    enableReachabilityFilter=false;
    debugMode=false;
    selection_hold_on = false;
    selection_enabled=false;
    clicked=false;
    dragging=false;
    show_popup_onrelease=false;
    visualize_bbox=false;
    motion_trail_log_enabled=false;
    
    ray_hit_t = 0;
    otdf_id = 0;
    num_otdfs = 0;
    alpha = 1.0;
    last_state_msg_timestamp = 0;
    lhand_urdf_id=0;
    rhand_urdf_id=0;
    
    free_running_sticky_hand_cnt = 0;
    free_running_sticky_foot_cnt = 0;
  }
  
  ~RendererAffordances(){   
    
  } 

  // core members
  BotRenderer renderer;
  BotEventHandler ehandler;
  
  // core member pointers
  BotViewer *viewer;
  BotGtkParamWidget *pw;
  GtkWidget *dblclk_popup;
  GtkWidget *second_stage_popup;
  boost::shared_ptr<lcm::LCM> lcm;

 
  // Member Classes
  // -----------------
  // LCM msg handlers and publishers
  boost::shared_ptr<AffordanceCollectionListener> affordanceMsgHandler;
  boost::shared_ptr<RobotStateListener> robotStateListener;
  boost::shared_ptr<CandidateGraspSeedListener> candidateGraspSeedListener;
  boost::shared_ptr<InitGraspOptPublisher> initGraspOptPublisher;
  boost::shared_ptr<GraspOptStatusListener> graspOptStatusListener;
  boost::shared_ptr<CandidateFootStepSeedManager> candidateFootStepSeedManager;
  boost::shared_ptr<ReachabilityVerifier> reachabilityVerifier;
  boost::shared_ptr<BatchFKQueryHandler>  dofRangeFkQueryHandler;
  
  //Member Variables 
  // -----------------

  std::map<std::string, OtdfInstanceStruc > instantiated_objects; // otdftemplatename+ object_uid
  std::map<std::string, StickyHandStruc> sticky_hands; // otdftemplatename + object_uid + geometryname + "_grasp_" + hand_uid;  
  std::map<std::string, StickyFootStruc> sticky_feet; // otdftemplatename + object_uid + geometryname + "_footstep_" + foot_uid;
  // NOTE: an otdf instance can have multiple sticky_hands/feet associated with it.
  
  OtdfInstanceStruc otdf_instance_hold;// keeps a local copy of the selected object, while making changes to it and then publishes it as an affordance.
  KDL::Frame otdf_T_world_body_hold; //store position in the world
  std::map<std::string, double> otdf_current_jointpos_hold;
  
  int free_running_sticky_hand_cnt; // never decremented, used to set uid of sticky hands which is unique in global map scope 
  int free_running_sticky_foot_cnt;
  KDL::Frame T_graspgeometry_lhandinitpos;
  KDL::Frame T_graspgeometry_rhandinitpos;

  int otdf_id;
  // otdf models
  int num_otdfs;
  char ** otdf_names;
  int * otdf_nums;
  
  int lhand_urdf_id;
  int rhand_urdf_id;
  
  // hand models
  int num_urdfs;
  char ** urdf_names;
  int * urdf_nums;
  
 // strings  
 std::string robot_name;
 std::string instance_selection; 
 std::string link_selection;
 std::string object_selection;
 std::string stickyhand_selection;
 std::string stickyfoot_selection;
 std::string marker_selection;
 std::string urdf_dir_name;
 std::vector<std::string> urdf_filenames;
 std::string otdf_dir_name;
 std::vector<std::string> otdf_filenames;
 std::vector<std::string> popup_widget_name_list;

  
  std::map<std::string, int > instance_cnt; // templateName, value. keeps track of how many times each template is instantiated. (only used for creating a local aff store)
  
  long last_state_msg_timestamp;
  float alpha;    // transparency of the object:
  
  Eigen::Vector3f ray_start;
  Eigen::Vector3f ray_end;
  Eigen::Vector3f ray_hit;
  Eigen::Vector3f ray_hit_drag;
  Eigen::Vector3f prev_ray_hit_drag;
  Eigen::Vector3f ray_hit_normal;
  Eigen::Vector3f marker_offset_on_press;// maintains this offset while dragging
  double ray_hit_t;
  
  
  // boolean flags
  bool showMesh;  // if false, draws otdf, if true, draws mesh instead
  bool showBoundingBox;
  bool enableReachabilityFilter;
  bool debugMode;
  bool selection_hold_on;
  bool selection_enabled;
  bool clicked;
  bool dragging;
  bool show_popup_onrelease;
  bool visualize_bbox;
  bool motion_trail_log_enabled;
};

// if common shape (e.g. cylinder, sphere), fill in bounding box
static void commonShapeBoundingBox(const string& otdf_type, boost::shared_ptr<otdf::ModelInterface> instance_in,
                            double* bounding_xyz, double* bounding_rpy, double* bounding_lwh)
{
   if(otdf_type == "cylinder"){  
      bounding_lwh[0] = instance_in->params_map_.find("radius")->second*2;
      bounding_lwh[1] = instance_in->params_map_.find("radius")->second*2;
      bounding_lwh[2] = instance_in->params_map_.find("length")->second;
   } else if(otdf_type == "sphere"){  
      bounding_lwh[0] = instance_in->params_map_.find("radius")->second*2;
      bounding_lwh[1] = instance_in->params_map_.find("radius")->second*2;
      bounding_lwh[2] = instance_in->params_map_.find("radius")->second*2;
   } else if(otdf_type == "TODO"){  //TODO others...
   }
}

// =================================================================================
// maintaining  OtdfInstanceStruc
  
  inline static void delete_otdf_from_affstore(string channel, string otdf_type, int uid, void *user)
  {
        RendererAffordances *self = (RendererAffordances*) user;

        // create and send delete affordance message
        drc::affordance_plus_t aff;
        aff.aff.aff_store_control = drc::affordance_t::DELETE;
        aff.aff.otdf_type = otdf_type;
        aff.aff.uid = uid;
        aff.aff.map_id = 0;
        aff.aff.nparams = 0;
        aff.aff.nstates = 0;
        aff.npoints = 0;
        aff.ntriangles = 0;
        self->lcm->publish(channel, &aff);
        cout << "Delete message sent for: " << otdf_type << "_" << uid << endl;
  }


  inline static void publish_otdf_instance_to_affstore(string channel, string otdf_type, int uid, const boost::shared_ptr<otdf::ModelInterface> instance_in,void *user)
  {
   RendererAffordances *self = (RendererAffordances*) user;
   drc::affordance_t msg;

   // get otdf from map
   stringstream nameSS;
   OtdfInstanceStruc* otdf = NULL;
   nameSS << otdf_type << "_" << uid;
   if(self->instantiated_objects.count(nameSS.str())){
     otdf = &self->instantiated_objects[nameSS.str()];
   } else{
     cout << "***** ERROR: " << nameSS.str() << " not found in instantiated_objects\n";
   }

   msg.utime = 0;
   msg.map_id = 0;
   
   
   msg.otdf_type = otdf_type;
   msg.aff_store_control = msg.UPDATE;//msg.NEW
   msg.uid = uid; 
   
    //map<string, double >::iterator obj_it = instance_in->params_map_.find("x");
   msg.origin_xyz[0] =instance_in->params_map_.find("x")->second;
   msg.origin_xyz[1] =instance_in->params_map_.find("y")->second;
   msg.origin_xyz[2] =instance_in->params_map_.find("z")->second;
   msg.origin_rpy[0] =instance_in->params_map_.find("roll")->second;
   msg.origin_rpy[1] =instance_in->params_map_.find("pitch")->second;
   msg.origin_rpy[2] =instance_in->params_map_.find("yaw")->second;
   
   double bounding_xyz[]={0,0,0};
   double bounding_rpy[]={0,0,0};
   double bounding_lwh[]={0,0,0};
  
   if(otdf){
      bounding_xyz[0] = otdf->boundingBoxXYZ[0];
      bounding_xyz[1] = otdf->boundingBoxXYZ[1];
      bounding_xyz[2] = otdf->boundingBoxXYZ[2];
      bounding_rpy[0] = otdf->boundingBoxRPY[0];
      bounding_rpy[1] = otdf->boundingBoxRPY[1];
      bounding_rpy[2] = otdf->boundingBoxRPY[2];
      bounding_lwh[0] = otdf->boundingBoxLWH[0];
      bounding_lwh[1] = otdf->boundingBoxLWH[1];
      bounding_lwh[2] = otdf->boundingBoxLWH[2];
      msg.modelfile = otdf->modelfile;
   }

   // if common shape (e.g. cylinder, sphere), fill in bounding box
   commonShapeBoundingBox(otdf_type, instance_in, bounding_xyz, bounding_rpy, bounding_lwh);

   msg.bounding_xyz[0] = bounding_xyz[0]; msg.bounding_xyz[1] = bounding_xyz[1]; msg.bounding_xyz[2] = bounding_xyz[2];
   msg.bounding_rpy[0] = bounding_rpy[0]; msg.bounding_rpy[1] = bounding_rpy[1];msg.bounding_rpy[2] = bounding_rpy[2];
   msg.bounding_lwh[0] = bounding_lwh[0]; msg.bounding_lwh[1] = bounding_lwh[1];msg.bounding_lwh[2] = bounding_lwh[2];
    
   msg.nparams =  instance_in->params_map_.size();
   typedef std::map<std::string, double > params_mapType;
   for( params_mapType::const_iterator it = instance_in->params_map_.begin(); it!=instance_in->params_map_.end(); it++)
   { 
      msg.param_names.push_back(it->first);
      msg.params.push_back(it->second);
   }

  int cnt=0;
   typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator it = instance_in->joints_.begin();it != instance_in->joints_.end(); it++)
    {     
      if(it->second->type!=(int) otdf::Joint::FIXED) {

          double pos, vel;
          instance_in->getJointState(it->first,pos,vel);
          cnt++;
          msg.state_names.push_back(it->first);
          msg.states.push_back(pos);
      }
     }
   msg.nstates =  cnt;
   cout <<"publish_otdf_instance_to_affstore: "<< msg.otdf_type << "_"<< msg.uid << ", of template :" << msg.otdf_type << endl;
   
   self->lcm->publish(channel, &msg);

  } 
  
  
  inline static void publish_new_otdf_instance_to_affstore( string channel, string otdf_type, int uid, const boost::shared_ptr<otdf::ModelInterface> instance_in,void *user)
  {
   RendererAffordances *self = (RendererAffordances*) user;
   drc::affordance_plus_t msg;

   msg.aff.utime = 0;
   msg.aff.map_id = 0;
   msg.aff.uid =0; // aff store should assign this
   
      
   msg.aff.otdf_type = otdf_type;
   msg.aff.aff_store_control = msg.aff.NEW;
   
   //map<string, double >::iterator obj_it = instance_in->params_map_.find("x");
   msg.aff.origin_xyz[0] =instance_in->params_map_.find("x")->second;
   msg.aff.origin_xyz[1] =instance_in->params_map_.find("y")->second;
   msg.aff.origin_xyz[2] =instance_in->params_map_.find("z")->second;
   msg.aff.origin_rpy[0] =instance_in->params_map_.find("roll")->second;
   msg.aff.origin_rpy[1] =instance_in->params_map_.find("pitch")->second;
   msg.aff.origin_rpy[2] =instance_in->params_map_.find("yaw")->second;
   
   double bounding_xyz[]={0,0,0};
   double bounding_rpy[]={0,0,0};
   double bounding_lwh[]={0,0,0};

   if(otdf_type == "car"){
      //TODO centralize these settings.  This is duplicated in segmentation code
      msg.aff.modelfile = "car.pcd";
      bounding_xyz[2] = 1.0;   // center of bounding box is 1m above car origin
      bounding_lwh[0] = 3.0;
      bounding_lwh[1] = 1.7;
      bounding_lwh[2] = 2.2;
   } else commonShapeBoundingBox(otdf_type, instance_in, bounding_xyz, bounding_rpy, bounding_lwh);

   msg.aff.bounding_xyz[0] = bounding_xyz[0]; msg.aff.bounding_xyz[1] = bounding_xyz[1];msg.aff.bounding_xyz[2] = bounding_xyz[2];
   msg.aff.bounding_rpy[0] = bounding_rpy[0]; msg.aff.bounding_rpy[1] = bounding_rpy[1];msg.aff.bounding_rpy[2] = bounding_rpy[2];
   msg.aff.bounding_lwh[0] = bounding_lwh[0]; msg.aff.bounding_lwh[1] = bounding_lwh[1];msg.aff.bounding_lwh[2] = bounding_lwh[2];

  
   msg.aff.nparams =  instance_in->params_map_.size();
   typedef std::map<std::string, double > params_mapType;
   for( params_mapType::const_iterator it = instance_in->params_map_.begin(); it!=instance_in->params_map_.end(); it++)
   { 
      msg.aff.param_names.push_back(it->first);
      msg.aff.params.push_back(it->second);
   }

  int cnt=0;
   typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator it = instance_in->joints_.begin();it != instance_in->joints_.end(); it++)
    {   
      if(it->second->type!=(int) otdf::Joint::FIXED) {

          double pos, vel;
          instance_in->getJointState(it->first,pos,vel);
          cnt++;
          msg.aff.state_names.push_back(it->first);
          msg.aff.states.push_back(pos);
      }
     }
   msg.aff.nstates =  cnt;
   msg.npoints = 0;
   msg.ntriangles = 0;
   cout <<"publish_otdf_instance_to_affstore: creating a new instance of template :" << msg.aff.otdf_type << endl;
    
   self->lcm->publish(channel, &msg);
  } 
  
  inline static void create_otdf_object_instance (RendererAffordances *self, bool local_testing)
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
    
    std::map<std::string, int >::iterator it;
    it= self->instance_cnt.find(self->otdf_filenames[self->otdf_id]);
    it->second = it->second + 1;
    std::stringstream oss;
    oss << self-> otdf_filenames[self->otdf_id] << "_"<< it->second-1;  // unique name, cnt starts from zero
    
    instance_struc.otdf_type=filename;//new string(filename);
    instance_struc.uid=it->second-1; // starts from zero
    //instance_struc._otdf_instance->name_ = oss.str();
    
    instance_struc._collision_detector.reset();
     // Each object has its own collision detector for now.      
    instance_struc._collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());
    instance_struc._gl_object = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody(instance_struc._otdf_instance,instance_struc._collision_detector,true,oss.str()));
    instance_struc._gl_object->set_state(instance_struc._otdf_instance);

    if(local_testing)
      self->instantiated_objects.insert(std::make_pair(oss.str(), instance_struc));
    else
    {
      publish_new_otdf_instance_to_affstore("AFFORDANCE_FIT",(instance_struc.otdf_type),0,instance_struc._otdf_instance,self); 
    }
    bot_viewer_request_redraw(self->viewer);
  } 

  inline static void update_OtdfInstanceStruc (OtdfInstanceStruc &instance_struc)
  {
    instance_struc._otdf_instance->update();
    instance_struc._gl_object->set_state(instance_struc._otdf_instance);
  }
  
    
 inline static bool otdf_instance_has_seeds(void* user, string &object_name)
 { 
    RendererAffordances *self = (RendererAffordances*) user;
    typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
    sticky_hands_map_type_::iterator hand_it = self->sticky_hands.begin();
    while (hand_it!=self->sticky_hands.end()) 
    {
       if (hand_it->second.object_name == (object_name))
       {
         return true;
       }
       
       hand_it++;
    } 

    typedef std::map<std::string, StickyFootStruc > sticky_feet_map_type_;
    sticky_feet_map_type_::iterator foot_it = self->sticky_feet.begin();
    while (foot_it!=self->sticky_feet.end()) 
    {
       if (foot_it->second.object_name == (object_name))
       {
         return true;
       }
       
       foot_it++;
    } 
    return false;
    
  }
  

//===============================================================================
// MISC. UTILS

   //-------------------------------------------------------------------------------
  inline static void get_min_dimension(boost::shared_ptr<otdf::Geometry> &link_geom, std::string &min_dimension_tag)
  {
   
    min_dimension_tag = "XY";
    enum {SPHERE, BOX, CYLINDER, MESH, TORUS};   
    int type = link_geom->type;
//    if(type == SPHERE)
//    {
//      boost::shared_ptr<otdf::Sphere> sphere(boost::shared_dynamic_cast<otdf::Sphere>(link_geom));	
//      double radius = sphere->radius;
//    }
//    else 
    if(type == BOX)
    {
      boost::shared_ptr<otdf::Box> box(boost::shared_dynamic_cast<otdf::Box>(link_geom));
      double xDim = box->dim.x;
      double yDim = box->dim.y;
      double zDim = box->dim.z;
      if(zDim<min(xDim,yDim))
        min_dimension_tag = "Z"; 
    }      
    else if(type == CYLINDER)
    {
      boost::shared_ptr<otdf::Cylinder> cyl(boost::shared_dynamic_cast<otdf::Cylinder>(link_geom));
      double radius = cyl->radius;
      double length = cyl->length;
      if(length<radius)
       min_dimension_tag = "Z";   
    }  
    else if(type == TORUS)
    {
      boost::shared_ptr<otdf::Torus> torus(boost::shared_dynamic_cast<otdf::Torus>(link_geom));
      double innerRadius = torus->tube_radius;
      double outerRadius = torus->radius;
      if(innerRadius<outerRadius)
        min_dimension_tag = "Z";  
    }      
  
  }
 //-------------------------------------------------------------------------------
  inline static void get_user_specified_hand_approach(void *user, Eigen::Vector3d objectframe_finger_dir, Eigen::Vector3d from, Eigen::Vector3d to, boost::shared_ptr<otdf::Geometry> &link_geom, KDL::Frame &T_objectgeometry_lhand, KDL::Frame &T_objectgeometry_rhand)
  {
  
   // calculate the rotation required to rotate x axis of hand to the negative ray direction. The palm face is pointing in the -x direction for the sandia hand. 
   Eigen::Vector3d ux,uy,uz;
   //x axis on sandia hand is pointing away from the palm face
   // if we define the urdf in palm frame as base then palm is facing z positive :Will scale to other hands
   ux << 1 , 0 , 0; 
   uy << 0 , 1 , 0;
   uz << 0 , 0 , 1;
  
   Eigen::Vector3d nray = -(to - from);
   nray.normalize();  // normalize
   
     // back-track from the hit pt in the approach dir by 10*t cm
   double t = 0.08;
   Eigen::Vector3d p;
   p << to[0]+t*nray[0], to[1]+t*nray[1], to[2]+t*nray[2]; 

   Eigen::Vector3d cross_prod;
   double dot_prod;
  
  // no need to normalize vectors
    nray = -(to - from);
    if(nray[0]>=0){
      dot_prod =ux.dot(nray); 
      cross_prod = ux.cross(nray);
    }
    else {
      dot_prod =ux.dot(-nray); 
      cross_prod = ux.cross(-nray);
    }
 
   // supposedly handles better aaround 180deg
   double w = (dot_prod) + ux.norm()*nray.norm();
   Eigen::Vector4f quat;
   quat << w,cross_prod[0],cross_prod[1],cross_prod[2];
   quat.normalize();
   
   if (w < 0.0001) { // vectors are 180 degrees apart
    quat << 0,-ux[2],ux[1],ux[0];
    quat.normalize();
  } 
   KDL::Frame T_objectgeometry_hand;
   T_objectgeometry_hand.p[0]=p[0];
   T_objectgeometry_hand.p[1]=p[1];
   T_objectgeometry_hand.p[2]=p[2];
   T_objectgeometry_hand.M =  KDL::Rotation::Quaternion(quat[1], quat[2], quat[3], quat[0]);

  // hand pos is calculated in positive x hemisphere only.
  // if viewpoint is in negative x hemisphere, hand is flipped about Z axis.
   if(nray[0]<0){
    KDL::Frame fliphand; 
    fliphand.p[0]=0;
    fliphand.p[1]=0;
    fliphand.p[2]=0;
    fliphand.M =  KDL::Rotation::RPY(0,0,M_PI);
    T_objectgeometry_hand = T_objectgeometry_hand*fliphand;
   }
  //-------
  // Hand is now with the palm facing the object.
  
   KDL::Frame T_rotatedhand_lhand,T_rotatedhand_rhand; //dependent on hand model and object dimensions
   T_rotatedhand_lhand.p[0]=0;
   T_rotatedhand_lhand.p[1]=0;
   T_rotatedhand_lhand.p[2]=0.125;
   T_rotatedhand_rhand.p=T_rotatedhand_lhand.p;
   
   objectframe_finger_dir.normalize(); // finger dir in object frame.
 
   Eigen::Vector3d desired_finger_dir;
   
   KDL::Frame temp_frame = T_objectgeometry_hand.Inverse();
   temp_frame.p = 0*temp_frame.p;// ignore translation while dealing with direction vectors
   rotate_eigen_vector_given_kdl_frame(objectframe_finger_dir,temp_frame,desired_finger_dir);
  
   double theta = atan2(desired_finger_dir[2],desired_finger_dir[1]);
   //depending on left or right hand, sign of rotation will change.
   std::cout<< "theta: "<< theta*(180/M_PI) << std::endl;
    
   T_rotatedhand_lhand.M =  KDL::Rotation::RPY(5*(M_PI/8)-theta,0,0);
   T_rotatedhand_rhand.M =  KDL::Rotation::RPY(3*(M_PI/8)-theta,0,0);
   T_objectgeometry_lhand = T_objectgeometry_hand *(T_rotatedhand_lhand.Inverse());//gets T_objectgeometry_rotatedhand 
   T_objectgeometry_rhand = T_objectgeometry_hand *(T_rotatedhand_rhand.Inverse());//gets T_objectgeometry_rotatedhand 
   
  } // end get_user_specified_hand_approach
  
   //-------------------------------------------------------------------------------
  inline static void get_hand_approach(Eigen::Vector3d from, Eigen::Vector3d to, boost::shared_ptr<otdf::Geometry> &link_geom, KDL::Frame &T_objectgeometry_lhand,KDL::Frame &T_objectgeometry_rhand)
  {

   // calculate the rotation required to rotate x axis of hand to the negative ray direction. The palm face is pointing in the -x direction for the sandia hand.  
       
   Eigen::Vector3d ux,uy,uz;
   ux << 1 , 0 , 0; //x axis on sandia hand is pointing away from the palm face
   uy << 0 , 1 , 0;
   uz << 0 , 0 , 1;
  
   Eigen::Vector3d nray = -(to - from);
   nray.normalize();  // normalize
   

   // back-track from the hit pt in the approach dir by 100*t cm
   double t = 0.05;
   Eigen::Vector3d p;
   p << to[0]+t*nray[0], to[1]+t*nray[1], to[2]+t*nray[2]; 
   
   // APPROACH 1 to calculate 
    Eigen::Vector3d cross_prod;
    double dot_prod;
    if(nray[0]>=0){
      dot_prod =ux.dot(nray); 
      cross_prod = ux.cross(nray);
    }
    else {
      dot_prod =ux.dot(-nray); 
      cross_prod = ux.cross(-nray);
    }

   cross_prod.normalize(); 

   double q[4];
   double axis[3]  = {cross_prod[0],cross_prod[1],cross_prod[2]};
   
    double angle = acos(dot_prod);
  bot_angle_axis_to_quat(angle,axis,q); 
  
  // APPROACH 2
  // no need to normalize vectors
    nray = -(to - from);
    if(nray[0]>=0){
      dot_prod =ux.dot(nray); 
      cross_prod = ux.cross(nray);
    }
    else {
      dot_prod =ux.dot(-nray); 
      cross_prod = ux.cross(-nray);
    }
 
   // supposedly handles better aaround 180deg
   double w = (dot_prod) + ux.norm()*nray.norm();
   Eigen::Vector4f quat;
   quat << w,cross_prod[0],cross_prod[1],cross_prod[2];
   quat.normalize();
   
   if (w < 0.0001) { // vectors are 180 degrees apart
    quat << 0,-ux[2],ux[1],ux[0];
    quat.normalize();
  } 


   KDL::Frame T_objectgeometry_hand;
   T_objectgeometry_hand.p[0]=p[0];
   T_objectgeometry_hand.p[1]=p[1];
   T_objectgeometry_hand.p[2]=p[2];
   T_objectgeometry_hand.M =  KDL::Rotation::Quaternion(q[1], q[2], q[3], q[0]);
   T_objectgeometry_hand.M =  KDL::Rotation::Quaternion(quat[1], quat[2], quat[3], quat[0]);
   
//   std::cout << "quat(x,y,z,w):  " << q[1]<< " " <<q[2]<< " "  << q[3]<< " "  << q[0]  << std::endl;
//   std::cout << "quat(x,y,z,w):  " << quat[1]<< " " <<quat[2]<< " "  << quat[3]<< " "  << quat[0]  << std::endl;
   
   
  // hand pos is calculated in positive x hemisphere only.
  // if viewpoint is in negative x hemisphere, hand is flipped about Z axis.
   if(nray[0]<0){
    KDL::Frame fliphand; 
    fliphand.p[0]=0;
    fliphand.p[1]=0;
    fliphand.p[2]=0;
    fliphand.M =  KDL::Rotation::RPY(0,0,M_PI);
    T_objectgeometry_hand = T_objectgeometry_hand*fliphand;
   }
  //-----------
  // Hand is now with the palm facing the object.
  
  //hand orientation is dependent on whether the smallest dimension along the axis of the link geometry in geom_z_axis or perpendicular to it in geom_XY plane.
  // things like steering wheel cylinder are thin in Z axis. You want to position your hand so as to pinch the thin part of the plate. Not that by default z is the key axis for urdf geometry definitions like cyl, torus.
     std::string min_dimension_tag;
     get_min_dimension(link_geom,min_dimension_tag); 
     
     
  //  if looking down at the object from the top or looking up from the bottom.
    if( fabs(uz.dot(nray))>max(fabs(ux.dot(nray)),fabs(uy.dot(nray))) )
    {
   
      double bearing_about_z = atan2(to[1],to[0]);
      if(uz.dot(nray)<0)
        bearing_about_z = M_PI-bearing_about_z;      

      KDL::Frame rotatetocenter=KDL::Frame::Identity();
      if(nray[0]<0){
        bearing_about_z += M_PI;
      }
      rotatetocenter.M =  KDL::Rotation::RPY(bearing_about_z,0,0);
      T_objectgeometry_hand = T_objectgeometry_hand*rotatetocenter;
    }//end if
      
     //depending on left or right hand, sign of rotation will change.
     KDL::Frame T_rotatedhand_lhand,T_rotatedhand_rhand; //dependent on hand model and object dimensions
     T_rotatedhand_lhand.p[0]=0;
     T_rotatedhand_lhand.p[1]=0;
     T_rotatedhand_lhand.p[2]=0.125;
     T_rotatedhand_rhand.p=T_rotatedhand_lhand.p;
     if((min_dimension_tag=="XY")&&(fabs(uz.dot(nray))<=max(fabs(ux.dot(nray)),fabs(uy.dot(nray))))){
      T_rotatedhand_lhand.M =  KDL::Rotation::RPY(-3*(M_PI/8),0,0);
      T_rotatedhand_rhand.M =  KDL::Rotation::RPY(3*(M_PI/8),0,0);
     }
     else if(min_dimension_tag=="Z"){ 
      T_rotatedhand_lhand.M =  KDL::Rotation::RPY(-M_PI/8,0,0);
      T_rotatedhand_rhand.M =  KDL::Rotation::RPY(-M_PI/8,0,0);
     }

    T_objectgeometry_lhand = T_objectgeometry_hand *(T_rotatedhand_lhand.Inverse());//T_objectgeometry_rotatedhand 
    T_objectgeometry_rhand = T_objectgeometry_hand *(T_rotatedhand_rhand.Inverse());//T_objectgeometry_rotatedhand 

  }// end get_hand_approach
  
  
 //-------------------------------------------------------------------------------  
  inline static void set_hand_init_position(void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;

    string object_geometry_name = self->link_selection; 
    string object_name_token  = self->object_selection + "_";
    size_t found = object_geometry_name.find(object_name_token);  
    string geometry_name =object_geometry_name.substr(found+object_name_token.size());
  
    
    KDL::Frame T_world_graspgeometry = KDL::Frame::Identity();
    self->T_graspgeometry_lhandinitpos = KDL::Frame::Identity();
    self->T_graspgeometry_rhandinitpos = KDL::Frame::Identity();
    //Get initial position of hand relative to object geometry.
    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(self->object_selection);
    if(!obj_it->second._gl_object->get_link_geometry_frame(geometry_name,T_world_graspgeometry))
        cerr << " failed to retrieve " << geometry_name<<" in object " << self->object_selection <<endl;
    else {
        
        KDL::Frame T_graspgeometry_world = T_world_graspgeometry.Inverse();
        Eigen::Vector3d from_geomframe,hit_pt_geomframe;
        rotate_eigen_vector_given_kdl_frame(self->ray_start,T_graspgeometry_world,from_geomframe);
        rotate_eigen_vector_given_kdl_frame(self->ray_hit,T_graspgeometry_world,hit_pt_geomframe);

        boost::shared_ptr<otdf::Geometry> link_geom;
        obj_it->second._gl_object->get_link_geometry(geometry_name,link_geom); 

        
        // when not dragging set handpose deterministically (assumes z is up). 
        // Other wise you the user specified orientation as the direction of the fingers.        
        Eigen::Vector3f diff = self->ray_hit_drag - self->ray_hit;
        double length =diff.norm();
        if ((self->dragging)&&(length>1e-3)){
         Eigen::Vector3f diff = self->ray_hit_drag - self->ray_hit; // finger direction in world frame
         Eigen::Vector3d fingerdir_geomframe;//(temp.data);// finger direction in geometry frame
         diff.normalize();
         KDL::Frame temp_frame = T_graspgeometry_world;
         temp_frame.p = 0*temp_frame.p;// ignore translation while dealing with direction vectors
         rotate_eigen_vector_given_kdl_frame(diff,temp_frame,fingerdir_geomframe);//something wrong here.
         fingerdir_geomframe.normalize();
         get_user_specified_hand_approach(self,fingerdir_geomframe,from_geomframe,hit_pt_geomframe,link_geom,self->T_graspgeometry_lhandinitpos,self->T_graspgeometry_rhandinitpos);
        }
        else{
         get_hand_approach(from_geomframe,hit_pt_geomframe,link_geom,self->T_graspgeometry_lhandinitpos,self->T_graspgeometry_rhandinitpos);

        }//end else
         
        /*std::cout << "T_objectgeometry_hand.p: " << self->T_graspgeometry_lhandinitpos.p[0] 
                                          << " " << self->T_graspgeometry_lhandinitpos.p[1] 
                                          << " " << self->T_graspgeometry_lhandinitpos.p[2] << " " << std::endl;*/
        
    }//end else
  }// end void set_hand_init_position(void *user)
  //------------------------------------------------------------------------------- 
  
  inline static void set_object_desired_state_on_marker_motion(void *user,Eigen::Vector3f start,Eigen::Vector3f dir)
  {
      RendererAffordances *self = (RendererAffordances*) user;
      double gain = 1;

      std::string instance_name=  self->object_selection;
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);

      if(!it->second._gl_object->is_future_state_changing()) {
        it->second._gl_object->set_future_state_changing(true);                      
       // clear previously accumulated motion states for all dependent bodies
        typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
        sticky_hands_map_type_::iterator hand_it = self->sticky_hands.begin();
        while (hand_it!=self->sticky_hands.end()) 
        {
           if (hand_it->second.object_name == (instance_name))
           {
              hand_it->second._gl_hand->clear_desired_body_motion_history();
           }
           hand_it++;
        }
       }//end if(!it->second._gl_object->is_future_state_changing())
    
      
      // set desired state
      KDL::Frame T_world_object_future = it->second._gl_object->_T_world_body_future;
      double currentAngle, angleTo,dtheta;       
      KDL::Frame DragRotation=KDL::Frame::Identity();       
      if(it->second._gl_object->is_bodypose_adjustment_enabled())
      {
      
        if(self->marker_selection=="markers::base_x"){
          double dx =  self->ray_hit_drag[0]-self->marker_offset_on_press[0];
          T_world_object_future.p[0] = dx;
        }
        else if(self->marker_selection=="markers::base_y"){
          double dy =  self->ray_hit_drag[1]-self->marker_offset_on_press[1];
          T_world_object_future.p[1] = dy;
        }      
        else if(self->marker_selection=="markers::base_z"){
          double dz =  self->ray_hit_drag[2]-self->marker_offset_on_press[2];
          T_world_object_future.p[2] = dz;
        }    
        else if(self->marker_selection=="markers::base_roll"){
          currentAngle = atan2(self->prev_ray_hit_drag[2]-T_world_object_future.p[2],self->prev_ray_hit_drag[1]-T_world_object_future.p[1]);
          angleTo = atan2(self->ray_hit_drag[2]-T_world_object_future.p[2],self->ray_hit_drag[1]-T_world_object_future.p[1]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          //dtheta =  atan2(sin(angleTo - currentAngle), cos(angleTo - currentAngle));
          KDL::Vector axis;
          axis[0] = 1; axis[1] = 0; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        else if(self->marker_selection=="markers::base_pitch"){ 
          currentAngle = atan2(self->prev_ray_hit_drag[0]-T_world_object_future.p[0],self->prev_ray_hit_drag[2]-T_world_object_future.p[2]);
          angleTo = atan2(self->ray_hit_drag[0]-T_world_object_future.p[0],self->ray_hit_drag[2]-T_world_object_future.p[2]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          //dtheta =  atan2(sin(angleTo - currentAngle), cos(angleTo - currentAngle));
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 1; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }    
        else if(self->marker_selection=="markers::base_yaw"){
          currentAngle = atan2(self->prev_ray_hit_drag[1]-T_world_object_future.p[1],self->prev_ray_hit_drag[0]-T_world_object_future.p[0]);
          angleTo = atan2(self->ray_hit_drag[1]-T_world_object_future.p[1],self->ray_hit_drag[0]-T_world_object_future.p[0]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 0; axis[2]=1;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        
        T_world_object_future.M  = DragRotation.M*T_world_object_future.M;  
        
        std::map<std::string, double> jointpos_in;
        jointpos_in = it->second._gl_object->_future_jointpos;
        it->second._gl_object->set_future_state(T_world_object_future,jointpos_in); 
      
      }
      else if(it->second._gl_object->is_jointdof_adjustment_enabled())
      {
        //===========================================================================
        // set joint dof

        string object_name = self->object_selection; 
        string marker_name = self->marker_selection; 
        string token  = "markers::";
        size_t found = marker_name.find(token);  
        if (found==std::string::npos)
            return;
        string joint_name =marker_name.substr(found+token.size());
        
        //std::cout <<"markername: "<< marker_name<< " mouse on joint marker: " << joint_name << std::endl;


      // Get joint marker draw frame

        
        visualization_utils::JointFrameStruct jointInfo;
        it->second._gl_object->get_joint_info(joint_name,jointInfo);
        KDL::Frame T_world_object = it->second._gl_object->_T_world_body;
        KDL::Frame T_world_object_future = it->second._gl_object->_T_world_body_future;
        
        Eigen::Vector3f joint_axis;
        if(it->second._gl_object->is_future_display_active())        
          joint_axis << jointInfo.future_axis[0],jointInfo.future_axis[1],jointInfo.future_axis[2];
        else
          joint_axis << jointInfo.axis[0],jointInfo.axis[1],jointInfo.axis[2]; // in world frame
        joint_axis.normalize();
       
        
        Eigen::Vector3f u_body_to_joint;
        KDL::Frame T_world_joint;
        T_world_joint = jointInfo.future_frame;
        u_body_to_joint[0] = T_world_object_future.p[0]-T_world_joint.p[0];
        u_body_to_joint[1] = T_world_object_future.p[1]-T_world_joint.p[1];
        u_body_to_joint[2] = T_world_object_future.p[2]-T_world_joint.p[2];
        u_body_to_joint.normalize();

        
        double normal = acos(u_body_to_joint.dot(joint_axis));
        double flipped = acos(u_body_to_joint.dot(-joint_axis));
        
        KDL::Frame T_world_jointaxis;// Axis
        T_world_jointaxis.p = jointInfo.future_frame.p;

        double theta;
        Eigen::Vector3f axis;      
        Eigen::Vector3f uz; 
        uz << 0 , 0 , 1; 
        axis = uz.cross(joint_axis);
        axis.normalize();
        theta = acos(uz.dot(joint_axis));
        KDL::Vector axis_temp;
        axis_temp[0]=axis[0];axis_temp[1]=axis[1];axis_temp[2]=axis[2];
        T_world_jointaxis.M = KDL::Rotation::Rot(axis_temp,theta); //T_axis_world
        
        KDL::Frame T_world_marker = KDL::Frame::Identity();
        KDL::Frame T_jointaxis_marker = KDL::Frame::Identity();
        double arrow_length =0.2;
        if(flipped>normal+1e-1) {
          T_jointaxis_marker.p[2] =-2*arrow_length/3;
         }
        else{
          T_jointaxis_marker.p[2] = 2*arrow_length/3;
        }
        T_world_marker = T_world_jointaxis*T_jointaxis_marker; // T_axismarker_world = T_axismarker_axis*T_axis_world
        
       // proper hit_drag point via marker plane ray intersection.
        Eigen::Vector3f plane_normal,plane_point;
          
        plane_normal = joint_axis;
        plane_point[0]=T_world_marker.p[0];
        plane_point[1]=T_world_marker.p[1];
        plane_point[2]=T_world_marker.p[2];       
        double lambda1 = dir.dot(plane_normal);
        double lambda2 = (plane_point - start).dot(plane_normal);
        double t;
    
       // check for degenerate case where ray is (more or less) parallel to plane
       if (fabs (lambda1) >= 1e-9) {
         t = lambda2 / lambda1;
          self->ray_hit_drag << start[0]+t*dir[0], start[1]+t*dir[1], start[2]+t*dir[2];  
         }
        // else  no solution        
        
        
        Eigen::Vector3f diff = self->prev_ray_hit_drag - self->ray_hit_drag; 
        if(diff.norm() > 0.05){
          self->prev_ray_hit_drag = self->ray_hit_drag; 
        }
        Eigen::Vector3f hit_markerframe,hitdrag_markerframe;
        //convert to joint dof marker frame .
        rotate_eigen_vector_given_kdl_frame(self->prev_ray_hit_drag,T_world_marker.Inverse(),hit_markerframe); 
        rotate_eigen_vector_given_kdl_frame(self->ray_hit_drag,T_world_marker.Inverse(),hitdrag_markerframe); 
 
        int type = jointInfo.type;   
        if((type==otdf::Joint::REVOLUTE)||(type==otdf::Joint::CONTINUOUS))
        {          
          double currentAngle, angleTo, dtheta;         
          currentAngle = atan2(hit_markerframe[1],hit_markerframe[0]);
          angleTo = atan2(hitdrag_markerframe[1],hitdrag_markerframe[0]);
         /* cout << "currentAngle :"<< currentAngle*(180/M_PI)
               << " angleTo :"<< angleTo*(180/M_PI) <<endl;
          cout << "radius" << sqrt(pow(hitdrag_markerframe[0],2)+pow(hitdrag_markerframe[1],2)) << endl; */
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          std::map<std::string, double> jointpos_in;
          jointpos_in = it->second._gl_object->_future_jointpos;
          jointpos_in.find(joint_name)->second = (jointpos_in.find(joint_name)->second +dtheta); 
          it->second._gl_object->set_future_state(T_world_object_future,jointpos_in);   
          bot_viewer_request_redraw(self->viewer);
         }// end revolute joints
         else if(type==otdf::Joint::PRISMATIC)
         {
          double s=1;
          if(diff.dot(joint_axis)<0)
            s = -1;
          double distance = s*diff.norm();
           std::map<std::string, double> jointpos_in;
           jointpos_in = it->second._gl_object->_future_jointpos;
           jointpos_in.find(joint_name)->second -= distance;
           it->second._gl_object->set_future_state(T_world_object_future,jointpos_in); 
            bot_viewer_request_redraw(self->viewer);  
         }
      }
      
      self->prev_ray_hit_drag = self->ray_hit_drag; 
     bot_viewer_request_redraw(self->viewer);     
  }   // end set_object_desired_state_on_marker_motion()
  

  //------------------------------------------------------------------------------- 
  
  inline static void set_object_current_state_on_marker_motion(void *user,Eigen::Vector3f start,Eigen::Vector3f dir)
  {
      RendererAffordances *self = (RendererAffordances*) user;
      double gain = 1;

      std::string instance_name=  self->object_selection;

      // set current state
      KDL::Frame T_world_object = self->otdf_instance_hold._gl_object->_T_world_body;
      
      double currentAngle, angleTo,dtheta;       
      KDL::Frame DragRotation=KDL::Frame::Identity();       
      if(self->otdf_instance_hold._gl_object->is_bodypose_adjustment_enabled())
      {
      
        if(self->marker_selection=="markers::base_x"){
          double dx =  self->ray_hit_drag[0]-self->marker_offset_on_press[0];
          T_world_object.p[0] = dx;
        }
        else if(self->marker_selection=="markers::base_y"){
          double dy =  self->ray_hit_drag[1]-self->marker_offset_on_press[1];
          T_world_object.p[1] = dy;
        }      
        else if(self->marker_selection=="markers::base_z"){
          double dz =  self->ray_hit_drag[2]-self->marker_offset_on_press[2];
          T_world_object.p[2] = dz;
        }    
        else if(self->marker_selection=="markers::base_roll"){
          currentAngle = atan2(self->prev_ray_hit_drag[2]-T_world_object.p[2],self->prev_ray_hit_drag[1]-T_world_object.p[1]);
          angleTo = atan2(self->ray_hit_drag[2]-T_world_object.p[2],self->ray_hit_drag[1]-T_world_object.p[1]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          //dtheta =  atan2(sin(angleTo - currentAngle), cos(angleTo - currentAngle));
          KDL::Vector axis;
          axis[0] = 1; axis[1] = 0; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        else if(self->marker_selection=="markers::base_pitch"){ 
          currentAngle = atan2(self->prev_ray_hit_drag[0]-T_world_object.p[0],self->prev_ray_hit_drag[2]-T_world_object.p[2]);
          angleTo = atan2(self->ray_hit_drag[0]-T_world_object.p[0],self->ray_hit_drag[2]-T_world_object.p[2]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          //dtheta =  atan2(sin(angleTo - currentAngle), cos(angleTo - currentAngle));
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 1; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }    
        else if(self->marker_selection=="markers::base_yaw"){
          currentAngle = atan2(self->prev_ray_hit_drag[1]-T_world_object.p[1],self->prev_ray_hit_drag[0]-T_world_object.p[0]);
          angleTo = atan2(self->ray_hit_drag[1]-T_world_object.p[1],self->ray_hit_drag[0]-T_world_object.p[0]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 0; axis[2]=1;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        
        T_world_object.M  = DragRotation.M*T_world_object.M; 
        
        double roll,pitch,yaw;
        T_world_object.M.GetRPY(roll,pitch,yaw);
        self->otdf_instance_hold._otdf_instance->setParam("x",T_world_object.p[0]);
        self->otdf_instance_hold._otdf_instance->setParam("y",T_world_object.p[1]);
        self->otdf_instance_hold._otdf_instance->setParam("z",T_world_object.p[2]);
        self->otdf_instance_hold._otdf_instance->setParam("roll",roll);
        self->otdf_instance_hold._otdf_instance->setParam("pitch",pitch);
        self->otdf_instance_hold._otdf_instance->setParam("yaw",yaw);
        self->otdf_instance_hold._otdf_instance->update(); 
        self->otdf_instance_hold._gl_object->set_state(self->otdf_instance_hold._otdf_instance); 

      }
     else if(self->otdf_instance_hold._gl_object->is_jointdof_adjustment_enabled())
      {
        //===========================================================================
        // set joint dof
        string object_name = self->object_selection; 
        string marker_name = self->marker_selection; 
        string token  = "markers::";
        size_t found = marker_name.find(token);  
        if (found==std::string::npos)
            return;
        string joint_name =marker_name.substr(found+token.size());
        
        //std::cout <<"markername: "<< marker_name<< " mouse on joint marker: " << joint_name << std::endl;

      // Get joint marker draw frame
        visualization_utils::JointFrameStruct jointInfo;
        self->otdf_instance_hold._gl_object->get_joint_info(joint_name,jointInfo);
        
        KDL::Frame T_world_object = self->otdf_instance_hold._gl_object->_T_world_body;
        
        Eigen::Vector3f joint_axis;
        joint_axis << jointInfo.axis[0],jointInfo.axis[1],jointInfo.axis[2];
        joint_axis.normalize();

        Eigen::Vector3f u_body_to_joint;
        u_body_to_joint[0] = T_world_object.p[0]-jointInfo.frame.p[0];
        u_body_to_joint[1] = T_world_object.p[1]-jointInfo.frame.p[1];
        u_body_to_joint[2] = T_world_object.p[2]-jointInfo.frame.p[2];
        u_body_to_joint.normalize();
        double normal = acos(u_body_to_joint.dot(joint_axis));
        double flipped = acos(u_body_to_joint.dot(-joint_axis));
        
        double theta;
        Eigen::Vector3f axis;      
        Eigen::Vector3f uz; 
        uz << 0 , 0 , 1; 
        axis = uz.cross(joint_axis);
        theta = acos(uz.dot(joint_axis));
        
        KDL::Frame T_world_marker = KDL::Frame::Identity();
        KDL::Frame T_world_jointaxis= KDL::Frame::Identity();
        T_world_jointaxis.p = jointInfo.frame.p;//?
        KDL::Vector axis_temp;
        axis_temp[0]=axis[0];axis_temp[1]=axis[1];axis_temp[2]=axis[2];
        T_world_jointaxis.M = KDL::Rotation::Rot(axis_temp,theta);
        KDL::Frame T_jointaxis_marker = KDL::Frame::Identity();
        
        double arrow_length =0.2;
        if(flipped>normal+1e-1) {
          T_jointaxis_marker.p[2] =-2*arrow_length/3; 
         }
        else{
          T_jointaxis_marker.p[2] = 2*arrow_length/3;     
        }
        T_world_marker = T_world_jointaxis*T_jointaxis_marker;
        
        
        // proper hit_drag point via marker plane ray intersection.
        Eigen::Vector3f plane_normal,plane_point;
          
        plane_normal = joint_axis;
        plane_point[0]=T_world_marker.p[0];
        plane_point[1]=T_world_marker.p[1];
        plane_point[2]=T_world_marker.p[2];       
        double lambda1 = dir.dot(plane_normal);
        double lambda2 = (plane_point - start).dot(plane_normal);
        double t;
    
       // check for degenerate case where ray is (more or less) parallel to plane
       if (fabs (lambda1) >= 1e-9) {
         t = lambda2 / lambda1;
         self->ray_hit_drag << start[0]+t*dir[0], start[1]+t*dir[1], start[2]+t*dir[2];  
         }
        // else  no solution         
             
        Eigen::Vector3f diff = self->prev_ray_hit_drag - self->ray_hit_drag; 
        if(diff.norm() > 0.05){
          self->prev_ray_hit_drag = self->ray_hit_drag; 
        }   
        
        Eigen::Vector3f hit_markerframe,hitdrag_markerframe;
        //convert to joint dof marker frame .
        rotate_eigen_vector_given_kdl_frame(self->prev_ray_hit_drag,T_world_marker.Inverse(),hit_markerframe); 
        rotate_eigen_vector_given_kdl_frame(self->ray_hit_drag,T_world_marker.Inverse(),hitdrag_markerframe); 
     
        
        int type = jointInfo.type;   
        if((type==otdf::Joint::REVOLUTE)||(type==otdf::Joint::CONTINUOUS))
        {       
          double currentAngle, angleTo, dtheta;         
          currentAngle = atan2(hit_markerframe[1],hit_markerframe[0]);
          angleTo = atan2(hitdrag_markerframe[1],hitdrag_markerframe[0]);      
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          //
          double current_pos, velocity;
          self->otdf_instance_hold._otdf_instance->getJointState(joint_name, current_pos,velocity);
          self->otdf_instance_hold._otdf_instance->setJointState(joint_name, current_pos+dtheta,velocity); 
          self->otdf_instance_hold._otdf_instance->update(); 
          self->otdf_instance_hold._gl_object->set_state(self->otdf_instance_hold._otdf_instance); 
        }// end revolute joints
        else if(type==otdf::Joint::PRISMATIC)
        {
          double s=1;
          if(diff.dot(joint_axis)<0)
            s = -1;
          double distance = s*diff.norm();
          double current_pos, velocity;
          self->otdf_instance_hold._otdf_instance->getJointState(joint_name, current_pos,velocity);
          self->otdf_instance_hold._otdf_instance->setJointState(joint_name, current_pos-distance,velocity); 
          self->otdf_instance_hold._otdf_instance->update(); 
          self->otdf_instance_hold._gl_object->set_state(self->otdf_instance_hold._otdf_instance); 
        }
      }
          
      self->prev_ray_hit_drag = self->ray_hit_drag; 
          
  }   // end set_object_current_state_on_marker_motion()
  

//-------------------------------------------------------------------------------
  inline static double get_shortest_distance_between_objects_markers_sticky_hands_and_feet(void *user,Eigen::Vector3f &from,Eigen::Vector3f &to)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    collision::Collision_Object * intersected_object = NULL;
    Eigen::Vector3f hit_pt;
    Eigen::Vector3f hit_normal;
    double shortest_distance = -1;
    self->object_selection  = " ";
    self->link_selection  = " ";
    self->stickyhand_selection  = " ";
    self->stickyfoot_selection  = " ";
    self->marker_selection = " ";
    


    if((self->otdf_instance_hold._gl_object)&&(self->selection_hold_on)) // to make sure that _gl_object is initialized 
    {
     //if marker based adjustment is enabled
     if((self->otdf_instance_hold._gl_object->is_bodypose_adjustment_enabled())||(self->otdf_instance_hold._gl_object->is_jointdof_adjustment_enabled()))
     {

       if(self->otdf_instance_hold._gl_object->is_jointdof_adjustment_enabled())
          self->otdf_instance_hold._gl_object->_collision_detector_jointdof_markers->ray_test( from, to, intersected_object,hit_pt);
       else
          self->otdf_instance_hold._gl_object->_collision_detector_floatingbase_markers->ray_test( from, to, intersected_object,hit_pt);
        
        if(intersected_object != NULL ){
            self->ray_hit = hit_pt;
            self->ray_hit_t = (hit_pt - self->ray_start).norm();
            Eigen::Vector3f diff = (from-hit_pt);
            double distance = diff.norm();
            if(shortest_distance>0) {
              if (distance < shortest_distance)
              {
                shortest_distance = distance;
                self->ray_hit = hit_pt;
                self->ray_hit_drag = hit_pt;
                self->ray_hit_t = (hit_pt - self->ray_start).norm();
                self->object_selection  =  self->otdf_instance_hold._gl_object->_unique_name;
                self->marker_selection  = string(intersected_object->id().c_str());
              }
            }
            else {
              shortest_distance = distance;
              self->ray_hit = hit_pt;
              self->ray_hit_drag = hit_pt;
              self->ray_hit_t = (hit_pt - self->ray_start).norm();
              self->object_selection  =  self->otdf_instance_hold._gl_object->_unique_name;
              self->marker_selection  = string(intersected_object->id().c_str());
             }
        }
        else {
        // clear previous selections
         string no_selection = " ";
         self->otdf_instance_hold._gl_object->highlight_link(no_selection); 
        }  
                     
      }// end if(...is_bodypose_adjustment_enabled)||...->is_jointdof_adjustment_enabled))

    }// end if((self->otdf_instance_hold._gl_object)&&(self->selection_hold_on))


    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    // loop through object list and check if ray intersect any of them.
    for(object_instance_map_type_::const_iterator it = self->instantiated_objects.begin(); it!=self->instantiated_objects.end(); it++)
    {

      if(it->second._gl_object) // to make sure that _gl_object is initialized 
      {
      
       //if marker based adjustment is enabled
       if((it->second._gl_object->is_bodypose_adjustment_enabled())||(it->second._gl_object->is_jointdof_adjustment_enabled()))
       {

         if(it->second._gl_object->is_jointdof_adjustment_enabled())
            it->second._gl_object->_collision_detector_jointdof_markers->ray_test( from, to, intersected_object,hit_pt);
         else
            it->second._gl_object->_collision_detector_floatingbase_markers->ray_test( from, to, intersected_object,hit_pt);
          
          if(intersected_object != NULL ){
              self->ray_hit = hit_pt;
              self->ray_hit_t = (hit_pt - self->ray_start).norm();
              Eigen::Vector3f diff = (from-hit_pt);
              double distance = diff.norm();
              if(shortest_distance>0) {
                if (distance < shortest_distance)
                {
                  shortest_distance = distance;
                  self->ray_hit = hit_pt;
                  self->ray_hit_drag = hit_pt;
                  self->ray_hit_t = (hit_pt - self->ray_start).norm();
                  self->object_selection  =  it->first;
                  self->marker_selection  = string(intersected_object->id().c_str());
                }
              }
              else {
                shortest_distance = distance;
                self->ray_hit = hit_pt;
                self->ray_hit_drag = hit_pt;
                self->ray_hit_t = (hit_pt - self->ray_start).norm();
                self->object_selection  =  it->first;
                self->marker_selection  = string(intersected_object->id().c_str());
               }
          }
          else {
          // clear previous selections
           string no_selection = " ";
           it->second._gl_object->highlight_link(no_selection); 
          }  
                       
        }


  /*if(!(it->second._gl_object->is_jointdof_adjustment_enabled()))
  {*/
       //it->second._gl_object->_collision_detector->num_collisions();
         it->second._gl_object->_collision_detector->ray_test( from, to, intersected_object,hit_pt,hit_normal);
        //it->second._gl_object->_collision_detector->ray_test( from, to, intersected_object,hit_pt);
               
        // Highlight all objects that intersect with ray
        if(intersected_object != NULL ){
              self->ray_hit = hit_pt;
              self->ray_hit_t = (hit_pt - self->ray_start).norm();
              Eigen::Vector3f diff = (from-hit_pt);
              double distance = diff.norm();
              if(shortest_distance>0) {
                if (distance < shortest_distance)
                {
                  shortest_distance = distance;
                  self->ray_hit = hit_pt;
                  self->ray_hit_normal = hit_normal;  
                  self->ray_hit_drag = hit_pt;
                  self->ray_hit_t = (hit_pt - self->ray_start).norm();
                  self->object_selection  =  it->first;
                  self->marker_selection  = " ";
                  self->link_selection  = string(intersected_object->id().c_str());   
          
                }
              }
              else {
                shortest_distance = distance;
                self->ray_hit = hit_pt;
                self->ray_hit_normal = hit_normal;  
                self->ray_hit_drag = hit_pt;
                self->ray_hit_t = (hit_pt - self->ray_start).norm();
                self->object_selection  =  it->first;
                self->marker_selection  = " ";
                self->link_selection = string(intersected_object->id().c_str());   
     
               }          

              intersected_object = NULL;  
        }
        else {
        // clear previous selections
         string no_selection = " ";
         it->second._gl_object->highlight_link(no_selection); 
        }  
                
      }// end if object exists
    }// end for

  //}
  
   //loop through stick-feet list and check if ray intersect any of them.
    typedef map<string, StickyFootStruc > sticky_feet_map_type_;
    for(sticky_feet_map_type_::iterator it = self->sticky_feet.begin(); it!=self->sticky_feet.end(); it++)
    {
    
          KDL::Frame T_world_graspgeometry = KDL::Frame::Identity();       
          typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
          object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(it->second.object_name);
          if(!obj_it->second._gl_object->get_link_geometry_frame(it->second.geometry_name,T_world_graspgeometry))
              cerr << " failed to retrieve " << it->second.geometry_name<<" in object " << it->second.object_name <<endl;
          else {
              KDL::Frame T_graspgeometry_world = T_world_graspgeometry.Inverse();
              Eigen::Vector3f from_geomframe,to_geomframe;
              //convert to geometry frame.
              rotate_eigen_vector_given_kdl_frame(from,T_graspgeometry_world,from_geomframe); 
              rotate_eigen_vector_given_kdl_frame(to,T_graspgeometry_world,to_geomframe); 
              Eigen::Vector3f hit_pt;
              //it->second._gl_foot->_collision_detector->num_collisions();
              it->second._gl_foot->_collision_detector->ray_test( from_geomframe, to_geomframe, intersected_object,hit_pt);

              if(intersected_object != NULL ){   
                Eigen::Vector3f diff = (from_geomframe-hit_pt);
                double distance = diff.norm();
                if(shortest_distance>0) {
                  if (distance < shortest_distance)
                  {
                    shortest_distance = distance;
                    it->second._gl_foot->enable_whole_body_selection(true);
                    self->object_selection  =  " ";
                    self->link_selection  =  " ";
                    self->marker_selection  =  " ";
                    self->stickyhand_selection  = " ";
                    self->stickyfoot_selection  = it->first;  //intersected_object->id().c_str() includes link name 
                  }
                }
                else{
                  shortest_distance = distance;
                  it->second._gl_foot->enable_whole_body_selection(true);
                  self->object_selection  =  " ";
                  self->link_selection  =  " ";
                  self->marker_selection  =  " ";
                  self->stickyhand_selection  = " ";
                  self->stickyfoot_selection  = it->first;  //intersected_object->id().c_str() includes link name                  
                }
   
                intersected_object = NULL;
              }
              else {
                // clear previous selections
                string no_selection = " ";
                it->second._gl_foot->highlight_link(no_selection); 
                it->second._gl_foot->highlight_marker(no_selection);
              }
        
         }// end if
    
    }// end for   
    
  

    //loop through stick-hands list and check if ray intersect any of them.
    typedef map<string, StickyHandStruc > sticky_hands_map_type_;
    for(sticky_hands_map_type_::iterator it = self->sticky_hands.begin(); it!=self->sticky_hands.end(); it++)
    {
    
          KDL::Frame T_world_graspgeometry = KDL::Frame::Identity();       
          typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
          object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(it->second.object_name);
          if(!obj_it->second._gl_object->get_link_geometry_frame(it->second.geometry_name,T_world_graspgeometry))
              cerr << " failed to retrieve " << it->second.geometry_name<<" in object " << it->second.object_name <<endl;
          else {
              KDL::Frame T_graspgeometry_world = T_world_graspgeometry.Inverse();
              Eigen::Vector3f from_geomframe,to_geomframe;
              //convert to geometry frame.
              rotate_eigen_vector_given_kdl_frame(from,T_graspgeometry_world,from_geomframe); 
              rotate_eigen_vector_given_kdl_frame(to,T_graspgeometry_world,to_geomframe); 
              Eigen::Vector3f hit_pt;
              //it->second._gl_hand->_collision_detector->num_collisions();
              it->second._gl_hand->_collision_detector->ray_test( from_geomframe, to_geomframe, intersected_object,hit_pt);

              if(intersected_object != NULL ){              
                Eigen::Vector3f diff = (from_geomframe-hit_pt);
                double distance = diff.norm();
                if(shortest_distance>0) {
                  if (distance < shortest_distance)
                  {
                    shortest_distance = distance;
                    it->second._gl_hand->enable_whole_body_selection(true);
                    self->object_selection  =  " ";
                    self->link_selection  =  " ";
                    self->marker_selection  =  " ";
                    self->stickyhand_selection  = it->first;  //intersected_object->id().c_str() includes link name 
                  }
                }
                else{
                  shortest_distance = distance;
                  it->second._gl_hand->enable_whole_body_selection(true);
                  self->object_selection  =  " ";
                  self->link_selection  =  " ";
                  self->marker_selection =  " ";
                  self->stickyhand_selection  = it->first;  //intersected_object->id().c_str() includes link name                  
                }
   
                intersected_object = NULL;
              }
              else {
                // clear previous selections
                string no_selection = " ";
                it->second._gl_hand->highlight_link(no_selection); 
                it->second._gl_hand->highlight_marker(no_selection);
              }
        
         }// end if
    
    }// end for
    

    
    self->prev_ray_hit_drag = self->ray_hit_drag;
    return shortest_distance;
  }
  
}//end_namespace

void setup_renderer_affordances(BotViewer *viewer, int render_priority, lcm_t* lcm);


#endif //RENDERER_AFFORDANCES_HPP
