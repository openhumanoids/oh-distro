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

#define RENDERER_NAME "Objects & StickyHands"
#define PARAM_MANAGE_INSTANCES "Manage Instances"
#define PARAM_SHOW_MESH "Show mesh"
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
    std::string otdf_type;
    int uid;
    boost::shared_ptr<otdf::ModelInterface> _otdf_instance;
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_object;
    boost::shared_ptr<collision::Collision_Detector> _collision_detector;  
    // Each object has its own collision detector for now. 
    // Otherwise, we need to manage a global collision detector by add and removing links whenever an object is deleted or added.   

    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3i> triangles;

}OtdfInstanceStruc;   

typedef struct _StickyHandStruc {
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_hand;
    boost::shared_ptr<collision::Collision_Detector> _collision_detector;
    std::string object_name;
    std::string geometry_name; 
    int hand_type; //SANDIA_LEFT=0, SANDIA_RIGHT=1, SANDIA_BOTH=2, IROBOT_LEFT=3, IROBOT_RIGHT=4, IROBOT_BOTH=5;
    KDL::Frame T_geometry_hand; // this is stored in obj frame
    std::vector<std::string> joint_name;
    std::vector<double> joint_position;
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

  bool selection_enabled;
  bool clicked;
  bool dragging;
  bool show_popup_onrelease;
  bool visualize_bbox;
  Eigen::Vector3f ray_start;
  Eigen::Vector3f ray_end;
  Eigen::Vector3f ray_hit;
  Eigen::Vector3f ray_hit_drag;
  Eigen::Vector3f prev_ray_hit_drag;

  Eigen::Vector3f marker_offset_on_press;// maintains this offset while dragging
  double ray_hit_t;
  std::string* link_selection;
  std::string* object_selection;
  std::string* stickyhand_selection;
  std::string* marker_selection;
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

  // transparency of the object:
  float alpha;  
  
  std::string* instance_selection_ptr; 
  bool selection_hold_on;
  
  
  // TODO: make the following OtdfInstanceStruc
  boost::shared_ptr<otdf::ModelInterface> otdf_instance_hold;// keeps a local copy of the selected object, while making changes to it and then publishes it as an affordance type.
  boost::shared_ptr<visualization_utils::GlKinematicBody> gl_temp_object;
  std::string*  instance_hold_otdf_type_ptr; // std::stromg segfaults , hence using a pointer
  int instance_hold_uid;
    
  //otdf::ModelInterface otdf_instance_hold; // keeps a local copy of the selected object, while making changes to it and then publishes it as an affordance type.
  std::map<std::string, OtdfInstanceStruc > instantiated_objects; // otdftemplatename+ object_uid
  
  std::map<std::string, StickyHandStruc> sticky_hands; // otdftemplatename + object_uid + geometryname + "_grasp_" + hand_uid;
  // NOTE: an otdf instance can have multiple sticky_hands associated with it.
  // this variable should ideally be in the affordance store process.
  int free_running_sticky_hand_cnt; // never decremented, used to set uid of sticky hands which is unique in global map scope 
  
  KDL::Frame T_graspgeometry_lhandinitpos;
  KDL::Frame T_graspgeometry_rhandinitpos;
  // LCM msg handlers and publishers
  boost::shared_ptr<AffordanceCollectionListener> affordanceMsgHandler;
  boost::shared_ptr<RobotStateListener> robotStateListener;
  boost::shared_ptr<CandidateGraspSeedListener> candidateGraspSeedListener;
  boost::shared_ptr<InitGraspOptPublisher> initGraspOptPublisher;
  boost::shared_ptr<GraspOptStatusListener> graspOptStatusListener;

 
  long last_state_msg_timestamp;
  std::string* robot_name_ptr;
  
  GtkWidget *dblclk_popup;
  GtkWidget *second_stage_popup;

  bool showMesh;  // if false, draws otdf, if true, draws mesh instead
}RendererAffordances;


// =================================================================================
// maintaining  OtdfInstanceStruc

  inline static void create_otdf_object_instance (RendererAffordances *self)
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
    
    instance_struc.otdf_type=filename;
    instance_struc.uid=it->second-1; // starts from zero
    //instance_struc._otdf_instance->name_ = oss.str();
    
    instance_struc._collision_detector.reset();
     // Each object has its own collision detector for now.      
    instance_struc._collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());
    instance_struc._gl_object = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody(instance_struc._otdf_instance,instance_struc._collision_detector,true,oss.str()));
    instance_struc._gl_object->set_state(instance_struc._otdf_instance);

    self->instantiated_objects.insert(std::make_pair(oss.str(), instance_struc));
    bot_viewer_request_redraw(self->viewer);
  } 

  inline static void update_OtdfInstanceStruc (OtdfInstanceStruc &instance_struc)
  {
    instance_struc._otdf_instance->update();
    instance_struc._gl_object->set_state(instance_struc._otdf_instance);
  }
  
//===============================================================================
// FILE ACCESS
  
 /* inline static bool get_xmlstring_from_file(const std::string& filename, std::string &xml_string)
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
  inline static int get_OTDF_filenames_from_dir (std::string dir, std::vector<std::string> &files)
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
   //-------------------------------------------------------------------------------
  // this function should go into otdf_utils library
  inline static int get_URDF_or_SDF_filenames_from_dir (std::string dir, std::vector<std::string> &files)
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
  
    inline static int get_URDF_filenames_from_dir (std::string dir, std::vector<std::string> &files)
  {
      DIR *dp;
      struct dirent *dirp;
      if((dp  = opendir(dir.c_str())) == NULL) {
          cout << "Error(" << errno << ") opening " << dir << endl;
          return errno;
      }

      while ((dirp = readdir(dp)) != NULL) {
        std::string fn =string(dirp->d_name);
        if(fn.substr(fn.find_last_of(".") + 1) == "urdf") 
          files.push_back(fn.substr(0,fn.find_last_of(".")));
      }
      closedir(dp);
      return 0;
  } 
  
    */
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

    string object_geometry_name = (*self->link_selection); 
    string object_name_token  = (*self->object_selection) + "_";
    size_t found = object_geometry_name.find(object_name_token);  
    string geometry_name =object_geometry_name.substr(found+object_name_token.size());
  
    
    KDL::Frame T_world_graspgeometry = KDL::Frame::Identity();
    self->T_graspgeometry_lhandinitpos = KDL::Frame::Identity();
    self->T_graspgeometry_rhandinitpos = KDL::Frame::Identity();
    //Get initial position of hand relative to object geometry.
    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator obj_it = self->instantiated_objects.find((*self->object_selection));
    if(!obj_it->second._gl_object->get_link_geometry_frame(geometry_name,T_world_graspgeometry))
        cerr << " failed to retrieve " << geometry_name<<" in object " << (*self->object_selection) <<endl;
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

        }
         
        /*std::cout << "T_objectgeometry_hand.p: " << self->T_graspgeometry_lhandinitpos.p[0] 
                                          << " " << self->T_graspgeometry_lhandinitpos.p[1] 
                                          << " " << self->T_graspgeometry_lhandinitpos.p[2] << " " << std::endl;*/
        
    }
  }
  //------------------------------------------------------------------------------- 
  
  inline static void set_object_desired_state_on_marker_motion(void *user)
  {
      RendererAffordances *self = (RendererAffordances*) user;
      double gain = 1;

      std::string instance_name=  (*self->object_selection);
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
      
        if((*self->marker_selection)=="markers::base_x"){
          double dx =  self->ray_hit_drag[0]-self->marker_offset_on_press[0];
          T_world_object_future.p[0] = dx;
        }
        else if((*self->marker_selection)=="markers::base_y"){
          double dy =  self->ray_hit_drag[1]-self->marker_offset_on_press[1];
          T_world_object_future.p[1] = dy;
        }      
        else if((*self->marker_selection)=="markers::base_z"){
          double dz =  self->ray_hit_drag[2]-self->marker_offset_on_press[2];
          T_world_object_future.p[2] = dz;
        }    
        else if((*self->marker_selection)=="markers::base_roll"){
          currentAngle = atan2(self->prev_ray_hit_drag[2]-T_world_object_future.p[2],self->prev_ray_hit_drag[1]-T_world_object_future.p[1]);
          angleTo = atan2(self->ray_hit_drag[2]-T_world_object_future.p[2],self->ray_hit_drag[1]-T_world_object_future.p[1]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          //dtheta =  atan2(sin(angleTo - currentAngle), cos(angleTo - currentAngle));
          KDL::Vector axis;
          axis[0] = 1; axis[1] = 0; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        else if((*self->marker_selection)=="markers::base_pitch"){ 
          currentAngle = atan2(self->prev_ray_hit_drag[0]-T_world_object_future.p[0],self->prev_ray_hit_drag[2]-T_world_object_future.p[2]);
          angleTo = atan2(self->ray_hit_drag[0]-T_world_object_future.p[0],self->ray_hit_drag[2]-T_world_object_future.p[2]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          //dtheta =  atan2(sin(angleTo - currentAngle), cos(angleTo - currentAngle));
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 1; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }    
        else if((*self->marker_selection)=="markers::base_yaw"){
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

        string object_name = (*self->object_selection); 
        string marker_name = (*self->marker_selection); 
        string token  = "markers::";
        size_t found = marker_name.find(token);  
        if (found==std::string::npos)
            return;
        string joint_name =marker_name.substr(found+token.size());
        
        //std::cout <<"markername: "<< marker_name<< " mouse on joint marker: " << joint_name << std::endl;


      // Get joint marker draw frame

        
        visualization_utils::JointFrameStruct jointInfo;
        it->second._gl_object->get_joint_info(joint_name,jointInfo);
        
        KDL::Frame T_world_object_future = it->second._gl_object->_T_world_body_future;
        
        Eigen::Vector3f joint_axis;
        joint_axis << jointInfo.axis[0],jointInfo.axis[1],jointInfo.axis[2];

        Eigen::Vector3f u_body_to_joint;
        u_body_to_joint[0] = T_world_object_future.p[0]-jointInfo.future_frame.p[0];
        u_body_to_joint[1] = T_world_object_future.p[1]-jointInfo.future_frame.p[1];
        u_body_to_joint[2] = T_world_object_future.p[2]-jointInfo.future_frame.p[2];
        u_body_to_joint.normalize();
        double normal = acos(u_body_to_joint.dot(joint_axis));
        double flipped = acos(u_body_to_joint.dot(-joint_axis));
        
        double theta;
        Eigen::Vector3f axis;      
        Eigen::Vector3f uz; 
        uz << 0 , 0 , 1; 
        axis = uz.cross(joint_axis);
        theta = acos(uz.dot(joint_axis));
 
        KDL::Frame JointAxisFrame;
        JointAxisFrame.p = jointInfo.future_frame.p;//?
        KDL::Vector axis_temp;
        axis_temp[0]=axis[0];axis_temp[1]=axis[1];axis_temp[2]=axis[2];
        JointAxisFrame.M = KDL::Rotation::Rot(axis_temp,theta);
        KDL::Frame JointAxisOffset = KDL::Frame::Identity();
        
        double arrow_length =0.2;
        if(flipped>normal+1e-1) {
          JointAxisOffset.p[2] =-2*arrow_length/3;          
          JointAxisFrame = JointAxisFrame*JointAxisOffset;
         }
        else{
          JointAxisOffset.p[2] = 2*arrow_length/3;          
          JointAxisFrame = JointAxisFrame*JointAxisOffset;
        }
     
        Eigen::Vector3f hit_markerframe,hitdrag_markerframe;
        //convert to joint dof marker frame .
        rotate_eigen_vector_given_kdl_frame(self->prev_ray_hit_drag,JointAxisFrame.Inverse(),hit_markerframe); 
        rotate_eigen_vector_given_kdl_frame(self->ray_hit_drag,JointAxisFrame.Inverse(),hitdrag_markerframe); 
     
        double currentAngle, angleTo, dtheta;         
        currentAngle = atan2(hit_markerframe[1],hit_markerframe[0]);
        angleTo = atan2(hitdrag_markerframe[1],hitdrag_markerframe[0]);
        dtheta = gain*shortest_angular_distance(currentAngle,angleTo);

        std::map<std::string, double> jointpos_in;
        jointpos_in = it->second._gl_object->_future_jointpos;
        jointpos_in.find(joint_name)->second = normalize_angle_positive(jointpos_in.find(joint_name)->second +dtheta); // what about joint limits?
        
        it->second._gl_object->set_future_state(T_world_object_future,jointpos_in);   
      }
      
      self->prev_ray_hit_drag = self->ray_hit_drag; 
          
  }   // end set_object_desired_state_on_marker_motion()
  
//------------------------------------------------------------------------------- 

  inline static double get_shortest_distance_between_objects_markers_stickyhands (void *user,Eigen::Vector3f &from,Eigen::Vector3f &to)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    collision::Collision_Object * intersected_object = NULL;
    Eigen::Vector3f hit_pt;
    double shortest_distance = -1;
    (*self->object_selection)  = " ";
    (*self->link_selection)  = " ";
    (*self->stickyhand_selection)  = " ";
    (*self->marker_selection) = " ";
    

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
                  (*self->object_selection)  =  it->first;
                  (*self->marker_selection)  = string(intersected_object->id().c_str());
                }
              }
              else {
                shortest_distance = distance;
                self->ray_hit = hit_pt;
                self->ray_hit_drag = hit_pt;
                self->ray_hit_t = (hit_pt - self->ray_start).norm();
                (*self->object_selection)  =  it->first;
                (*self->marker_selection)  = string(intersected_object->id().c_str());
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
        it->second._gl_object->_collision_detector->ray_test( from, to, intersected_object,hit_pt);
   
        
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
                  self->ray_hit_drag = hit_pt;
                  self->ray_hit_t = (hit_pt - self->ray_start).norm();
                  (*self->object_selection)  =  it->first;
                  (*self->marker_selection)  = " ";
                  (*self->link_selection)  = string(intersected_object->id().c_str());   
          
                }
              }
              else {
                shortest_distance = distance;
                self->ray_hit = hit_pt;
                self->ray_hit_drag = hit_pt;
                self->ray_hit_t = (hit_pt - self->ray_start).norm();
                (*self->object_selection)  =  it->first;
                (*self->marker_selection)  = " ";
                (*self->link_selection)  = string(intersected_object->id().c_str());   
     
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
                    (*self->object_selection)  =  " ";
                    (*self->link_selection)  =  " ";
                    (*self->marker_selection)  =  " ";
                    (*self->stickyhand_selection)  = it->first;  //intersected_object->id().c_str() includes link name 
                  }
                }
                else{
                  shortest_distance = distance;
                  it->second._gl_hand->enable_whole_body_selection(true);
                  (*self->object_selection)  =  " ";
                  (*self->link_selection)  =  " ";
                  (*self->marker_selection)  =  " ";
                  (*self->stickyhand_selection)  = it->first;  //intersected_object->id().c_str() includes link name                  
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
