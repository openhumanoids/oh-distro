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


#define RENDERER_NAME "Objects & StickyHands"
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
#define PARAM_PICKING "Enable Selection"
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

namespace renderer_affordances{

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
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_object;
    boost::shared_ptr<collision::Collision_Detector> _collision_detector; 
}OtdfInstanceStruc;   

typedef struct _StickyHandStruc {
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_hand;
    boost::shared_ptr<collision::Collision_Detector> _collision_detector;
    std::string object_name;
    std::string geometry_name; 
    int uid;
    int opt_status;//RUNNING=0, SUCCESS=1, FAILURE=2;
}StickyHandStruc;   

class AffordanceCollectionListener;
class RobotStateListener;
class InitGraspOptPublisher;
class CandidateGraspSeedListener;
class GraspOptStatusListener;
 
typedef struct _RendererAffordances {
  BotRenderer renderer;
  BotEventHandler ehandler;
  BotViewer *viewer;
  boost::shared_ptr<lcm::LCM> lcm;

  BotGtkParamWidget *pw;

  bool picking;
  bool clicked;
  bool visualize_bbox;
  Eigen::Vector3f ray_start;
  Eigen::Vector3f ray_end;
  std::string* link_selection;
  std::string* object_selection;

  int otdf_id;

  int num_otdfs;
  char ** otdf_names;
  int * otdf_nums;
  
  std::string* otdf_dir_name_ptr;
  std::vector<std::string> otdf_filenames; // otdf_template_names
  std::map<std::string, int > instance_cnt; // templateName, value.
  
   int lhand_urdf_id;
   int rhand_urdf_id;
  // hand models
  int num_urdfs;
  char ** urdf_names;
  int * urdf_nums;
  
  std::string* urdf_dir_name_ptr;
  std::vector<std::string> urdf_filenames; // otdf_template_names

  std::string* instance_selection_ptr; 
  std::map<std::string, OtdfInstanceStruc > instantiated_objects; // otdftemplatename+ object_uid
  
  std::map<std::string, StickyHandStruc> sticky_hands; // otdftemplatename + object_uid + geometryname + "_grasp_" + hand_uid;
  // NOTE: an otdf instance can have multiple sticky_hands associated with it.
  // this variable should ideally be in the affordance store process.
  int free_running_sticky_hand_cnt; // never decremented, used to set uid of sticky hands which is unique in global map scope 
  
  KDL::Frame T_graspgeometry_handinitpos;
  
  // LCM msg handlers and publishers
  boost::shared_ptr<AffordanceCollectionListener> affordanceMsgHandler;
  boost::shared_ptr<RobotStateListener> robotStateListener;
  boost::shared_ptr<CandidateGraspSeedListener> candidateGraspSeedListener;
  boost::shared_ptr<InitGraspOptPublisher> initGraspOptPublisher;
  boost::shared_ptr<GraspOptStatusListener> graspOptStatusListener;
 
  long last_state_msg_timestamp;
  std::string* robot_name_ptr;
  
  GtkWidget *dblclk_popup;
}RendererAffordances;


// =================================================================================
// maintaining  OtdfInstanceStruc

  static void create_otdf_object_instance (RendererAffordances *self)
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
    oss << self-> otdf_filenames[self->otdf_id] << "_"<< it->second;  

    instance_struc._collision_detector.reset();
    instance_struc._collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());
    instance_struc._gl_object = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody(instance_struc._otdf_instance,instance_struc._collision_detector,true,oss.str()));
    instance_struc._gl_object->set_state(instance_struc._otdf_instance);

    self->instantiated_objects.insert(std::make_pair(oss.str(), instance_struc));
    bot_viewer_request_redraw(self->viewer);
  } 

  static void update_OtdfInstanceStruc (OtdfInstanceStruc &instance_struc)
  {
    instance_struc._otdf_instance->update();
    instance_struc._gl_object->set_state(instance_struc._otdf_instance);
  }
  
//===============================================================================
// FILE ACCESS
  
  static bool get_xmlstring_from_file(const std::string& filename, std::string &xml_string)
  {
    // get the entire file
    std::fstream xml_file(filename.c_str(), std::fstream::in);
    if (xml_file.is_open())
    {
      while ( xml_file.good() )
      {
        std::string line;
        std::getline( xml_file, line);
        xml_string += (line + "\n");
      }
      xml_file.close();
      return true;
    }
    else
    {
     // ROS_ERROR("Could not open file [%s] for parsing.",filename.c_str());
     std::cerr << "Could not open file ["<<filename.c_str()<<"] for parsing."<< std::endl;
      return false;
    }
  }

      
  // this function should go into otdf_utils library
  static int get_OTDF_filenames_from_dir (std::string dir, std::vector<std::string> &files)
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
  
  // this function should go into otdf_utils library
  static int get_URDF_or_SDF_filenames_from_dir (std::string dir, std::vector<std::string> &files)
  {
      DIR *dp;
      struct dirent *dirp;
      if((dp  = opendir(dir.c_str())) == NULL) {
          cout << "Error(" << errno << ") opening " << dir << endl;
          return errno;
      }

      while ((dirp = readdir(dp)) != NULL) {
        std::string fn =string(dirp->d_name);
        if((fn.substr(fn.find_last_of(".") + 1) == "urdf")||(fn.substr(fn.find_last_of(".") + 1) == "sdf")) 
          files.push_back(fn.substr(0,fn.find_last_of(".")));
      }
      closedir(dp);
      return 0;
  }  
//===============================================================================
// MISC. UTILS

  static void get_min_dimension(boost::shared_ptr<otdf::Geometry> &link_geom, std::string &min_dimension_tag)
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


  static void get_approach_frame(Eigen::Vector3d from, Eigen::Vector3d to, boost::shared_ptr<otdf::Geometry> &link_geom, KDL::Frame &T_objectgeometry_hand)
  {

   // calculate the rotation required to rotate x axis of hand to the negative ray direction. The palm face is pointing in the -x direction for the sandia hand.  
       
   Eigen::Vector3d ux,uy,uz;
   ux << 1 , 0 , 0; //x axis on sandia hand is pointing away from the palm face
   uy << 0 , 1 , 0;
   uz << 0 , 0 , 1;
  
   Eigen::Vector3d nray = -(to - from);
   nray.normalize();  // normalize
   

   // back-track from the hit pt in the approach dir by 100*t cm
   double t = 0.1;
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

  
  
  std::cout << "\n nray " <<nray.transpose() << std::endl;
  std::cout << "dot_prod " <<dot_prod << std::endl;
  std::cout << "cross_prod " <<cross_prod.transpose() << std::endl;
   std::cout << "cross_prod in z " <<cross_prod[2] << std::endl;
//  std::cout << "axis(x,y,z):  "  << axis[0] << " "<< axis[1]<< " " <<axis[2] << std::endl;
//  std::cout << "quat(x,y,z,w):  " << q[1]<< " " <<q[2]<< " "  << q[3]<< " "  << q[0]  << std::endl;
//  
//  Eigen::Vector4f test;
//  test << q[0],q[1],q[2],q[3];
//  std::cout << "quat norm:  " << test.norm() << std::endl;
//   
//   
//  if (dot_prod >= 1-1e-5) { // ux is already parrallel to nray
//    q[1]=0;
//    q[2]=0;
//    q[3]=0;
//    q[0]=1;
//  }
//  else if(dot_prod<=1e-5-1) {
//     //std::cout << "dot_prod<=-0.8 " <<dot_prod << std::endl;
//    Eigen::Vector3d fallbackaxis;
//    fallbackaxis = uy.cross(ux);
//    //std::cout << "uy.cross(ux) " <<fallbackaxis.transpose()<< std::endl;
//    fallbackaxis.normalize();
//    axis[0] = fallbackaxis[0];
//    axis[1] = fallbackaxis[1];
//    axis[2] = fallbackaxis[2];
//    bot_angle_axis_to_quat(M_PI,axis,q); 
//    
//  }

//   double rpy[3];
//   bot_angle_axis_to_roll_pitch_yaw (angle,axis,rpy);
//   std::cout << "euler(rpy):  "<< rpy[0]*57.2958  << " " << rpy[1]*57.2958 << " " <<rpy[2]*57.2958  << std::endl;
   
   
   T_objectgeometry_hand.p[0]=p[0];
   T_objectgeometry_hand.p[1]=p[1];
   T_objectgeometry_hand.p[2]=p[2];
   T_objectgeometry_hand.M =  KDL::Rotation::Quaternion(q[1], q[2], q[3], q[0]);
   T_objectgeometry_hand.M =  KDL::Rotation::Quaternion(quat[1], quat[2], quat[3], quat[0]);
   
   std::cout << "quat(x,y,z,w):  " << q[1]<< " " <<q[2]<< " "  << q[3]<< " "  << q[0]  << std::endl;
   std::cout << "quat(x,y,z,w):  " << quat[1]<< " " <<quat[2]<< " "  << quat[3]<< " "  << quat[0]  << std::endl;
   
   
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
  //-------------------------------------------------------------------------------
  // Hand is now with the palm facing the object.
  
  //hand orientation is dependent on whether the smallest dimension along the axis of the link geometry in geom_z_axis or perpendicular to it in geom_XY plane.
  // things like steering wheel cylinder are thin in Z axis. You want to position your hand so as to pinch the thin part of the plate. Not that by default z is the key axis for urdf geometry definitions like cyl, torus.
     std::string min_dimension_tag;
     get_min_dimension(link_geom,min_dimension_tag); 
     
     
  //  if looking down at the object from the top or looking up from the bottom.
  // TODO: looking up from bottom doesn't work.
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
      

     KDL::Frame T_rotatedhand_hand; //dependent on hand model and object dimensions
     T_rotatedhand_hand.p[0]=0;
     T_rotatedhand_hand.p[1]=0;
     T_rotatedhand_hand.p[2]=0.125;
     if((min_dimension_tag=="XY")&&(fabs(uz.dot(nray))<=max(fabs(ux.dot(nray)),fabs(uy.dot(nray))))){
      T_rotatedhand_hand.M =  KDL::Rotation::RPY(3*(M_PI/8),0,0);
     }
     else if(min_dimension_tag=="Z"){ 
      T_rotatedhand_hand.M =  KDL::Rotation::RPY(-M_PI/8,0,0);
     }

    T_objectgeometry_hand = T_objectgeometry_hand *(T_rotatedhand_hand.Inverse());//T_objectgeometry_rotatedhand 

//    if( fabs(uz.dot(nray))>max(fabs(ux.dot(nray)),fabs(uy.dot(nray))) )
//    {
//      if(min_dimension_tag=="Z"){ 
//        KDL::Frame fliphand; 
//        fliphand.p[0]=0;
//        fliphand.p[1]=0;
//        fliphand.p[2]=0;
//        fliphand.M =  KDL::Rotation::RPY(M_PI,0,0);
//        T_objectgeometry_hand = fliphand*T_objectgeometry_hand;
//      }
//    }//end if
  }
  

  
  
}//end_namespace

void setup_renderer_affordances(BotViewer *viewer, int render_priority, lcm_t* lcm);


#endif //RENDERER_AFFORDANCES_HPP
