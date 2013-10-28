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
#include <visualization_utils/gl_draw_utils.hpp>
#include <visualization_utils/keyboard_signal_utils.hpp>
#include <visualization_utils/SelectionManager.hpp>
#include <visualization_utils/affordance_utils/aff_trigger_signal_utils.hpp>
#include <visualization_utils/affordance_utils/affordance_utils.hpp>
#include <visualization_utils/affordance_utils/affordance_seed_utils.hpp>
#include <visualization_utils/affordance_utils/BatchFKQueryHandler.hpp>
#include <visualization_utils/affordance_utils/AffordanceCollectionManager.hpp>
#include <visualization_utils/stickyhand_utils/sticky_hand_utils.hpp>
#include <visualization_utils/stickyhand_utils/StickyhandCollectionManager.hpp>
#include <visualization_utils/stickyfoot_utils/sticky_foot_utils.hpp>
#include <visualization_utils/stickyfoot_utils/StickyfootCollectionManager.hpp>
#include <visualization_utils/foviation_signal_utils.hpp>

#define RENDERER_NAME "Affordances & StickyHands/Feet"
#define PARAM_MANAGE_INSTANCES "Manage Instances"
#define PARAM_SHOW_MESH "Show mesh"
#define PARAM_SHOW_BOUNDING_BOX "Show bounding box"
#define PARAM_SHOW_TRIAD "Show triads"
#define PARAM_REACHABILITY_FILTER "Enable Reachability Filter"
//#define PARAM_ADJUST_PARAM "Adjust Params"
#define PARAM_OTDF_SELECT "Template"
#define PARAM_OTDF_INSTANCE_SELECT "Instance"
#define PARAM_OTDF_ADJUST_PARAM "Adjust Params"
#define PARAM_OTDF_ADJUST_DOF "Adjust DoFs"
#define PARAM_OTDF_FLIP_PITCH "Flip pitch"
#define PARAM_OTDF_INSTANCE_CLEAR "Clear Instance"
#define PARAM_OTDF_DELETE "Delete"
#define PARAM_OTDF_INSTANCE_CLEAR_ALL "Clear All"
#define PARAM_INSTANTIATE "Instantiate/Fit"
#define PARAM_CLEAR "Clear All Instances"
#define PARAM_SELECTION "Enable Selection"
#define PARAM_OPT_POOL_READY "OptPool Ready"
#define PARAM_SHOW_PROPOSED_MANIP_MAP "Show Proposed Manip Map"
#define PARAM_LHAND_URDF_SELECT "LHand"
#define PARAM_RHAND_URDF_SELECT "RHand"
#define PARAM_REQUEST_PTCLD_FROM_MAPS "Request PtCld"

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

namespace renderer_affordances_gui_utils{
static void spawn_mixed_seed_dblclk_popup(void* user);
}

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
    frames = NULL;
    otdf_names = NULL;
    otdf_nums = NULL;
    dblclk_popup= NULL;
    second_stage_popup= NULL;
    
    
    showMesh = false;
    showBoundingBox = false;
    showTriad = false;
    enableReachabilityFilter=false;
    debugMode=false;
    selection_hold_on = false;
    selection_enabled=false;
    clicked=false;
    dragging=false;
    show_popup_onrelease=false;
    visualize_bbox=false;
    motion_trail_log_enabled=false;
    doBatchFK = false;
    
    ray_hit_t = 0;
    joint_marker_pos_on_press = 0;
    coupled_joint_marker_pos_on_press=0;
    otdf_id = 0;
    num_otdfs = 0;
    alpha = 1.0;
    last_state_msg_timestamp = 0;
    lhand_urdf_id=0;
    rhand_urdf_id=0;
    
    active_mate_axis = 2; //MATE_X=0,MATE_Y=1,MATE_Z=2;
    active_ee= drc::ee_teleop_transform_t::RIGHT_HAND; //RIGHT by default
    
    _renderer_foviate = false;
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
    
  BotFrames *frames;
  
  
  //Member Variables 
  // -----------------
  
  //object/stickyhands/stickyfeet cache managers 
  boost::shared_ptr<AffordanceCollectionManager> affCollection; 
  boost::shared_ptr<StickyhandCollectionManager> stickyHandCollection;
  boost::shared_ptr<StickyfootCollectionManager> stickyFootCollection;
  
  
  // Member Classes
  // -----------------
  // LCM msg handlers and publishers
  boost::shared_ptr<AffordanceCollectionListener> affordanceMsgHandler;
  boost::shared_ptr<RobotStateListener> robotStateListener;
  boost::shared_ptr<CandidateGraspSeedListener> candidateGraspSeedListener;
  boost::shared_ptr<InitGraspOptPublisher> initGraspOptPublisher;
  boost::shared_ptr<GraspOptStatusListener> graspOptStatusListener;

  boost::shared_ptr<ReachabilityVerifier> reachabilityVerifier;
  boost::shared_ptr<BatchFKQueryHandler>  dofRangeFkQueryHandler;
  boost::shared_ptr<KeyboardSignalHandler> keyboardSignalHndlr;
  boost::shared_ptr<SelectionManager> seedSelectionManager;
  RendererFoviationSignalRef _rendererFoviationSignalRef;
  bool _renderer_foviate;
  
  OtdfInstanceStruc otdf_instance_hold;// keeps a local copy of the selected object, while making changes to it and then publishes it as an affordance.
  KDL::Frame otdf_T_world_body_hold; //store position in the world
  std::map<std::string, double> otdf_current_jointpos_hold;
  
  KDL::Frame T_graspgeometry_lhandinitpos_sandia;
  KDL::Frame T_graspgeometry_rhandinitpos_sandia;
  KDL::Frame T_graspgeometry_lhandinitpos_irobot;
  KDL::Frame T_graspgeometry_rhandinitpos_irobot;
  
  int otdf_id;
  // otdf models
  int num_otdfs;
  char ** otdf_names;
  int * otdf_nums;
  
  int lhand_urdf_id;
  int rhand_urdf_id;
  int active_mate_axis;
  int active_ee;
  
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
 std::vector<std::string> _planseeds;

 std::map<std::string, int > instance_cnt; // templateName, value. keeps track of how many times each template is instantiated. (only used for creating a local aff store)


  // for manip map
  std::vector<string> dof_names;
  std::vector<double> dof_min;
  std::vector<double> dof_max;
  std::map<std::string, vector<KDL::Frame> > ee_frames_map;
  std::map<std::string, vector<drc::affordance_index_t> > ee_frame_affindices_map;
  
  long last_state_msg_timestamp;
  float alpha;    // transparency of the object:
  
  Eigen::Vector3f ray_start;
  Eigen::Vector3f ray_end;
  Eigen::Vector3f ray_hit;
  Eigen::Vector3f ray_hit_drag;
  Eigen::Vector3f prev_ray_hit_drag;
  Eigen::Vector3f ray_hit_normal;
  Eigen::Vector3f marker_offset_on_press;// maintains this offset while dragging
  double joint_marker_pos_on_press;
  double coupled_joint_marker_pos_on_press; // used for plane markers
  double ray_hit_t;
  
  
  // boolean flags
  bool showMesh;  // if false, draws otdf, if true, draws mesh instead
  bool showBoundingBox;
  bool showTriad;
  bool enableReachabilityFilter;
  bool debugMode;
  bool selection_hold_on;
  bool selection_enabled;
  bool clicked;
  bool dragging;
  bool show_popup_onrelease;
  bool visualize_bbox;
  bool motion_trail_log_enabled;
  bool doBatchFK;
  
  
  AffTriggerSignalsRef affTriggerSignalsRef;
  void keyboardSignalCallback(int keyval, bool is_pressed)
  {
    /*if(is_pressed) 
    {
      cout << "RendererAffordances::KeyPress Signal Received:  Keyval: " << keyval << endl;
    }
    else {
      cout << "RendererAffordances::KeyRelease Signal Received: Keyval: " << keyval << endl;
    }*/
    
    // last key event was shift release
    // and selection cnt is greater than one
    
    if(((!is_pressed)&&(keyval == SHIFT_L||keyval == SHIFT_R ))&&(seedSelectionManager->get_selection_cnt()>1))
    {
        cout<< "Printing Selection Order On Shift KeyRelease\n";
	      seedSelectionManager->print();
        renderer_affordances_gui_utils::spawn_mixed_seed_dblclk_popup(this);
    }
    
  }
};


//===============================================================================
// MISC. UTILS

 inline static bool otdf_instance_has_seeds(void* user, string &object_name)
 { 
    RendererAffordances *self = (RendererAffordances*) user;
    bool has_hand = self->stickyHandCollection->is_parent_object(object_name);
    bool has_foot = self->stickyFootCollection->is_parent_object(object_name);
    return (has_hand||has_foot);
 }

 //-------------------------------------------------------------------------------  
  inline static void set_hand_init_position(void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    string object_geometry_name = self->link_selection; 
    string object_name_token  = self->object_selection + "_";
    size_t found = object_geometry_name.find(object_name_token);  
    string geometry_name =object_geometry_name.substr(found+object_name_token.size());
    
    self->T_graspgeometry_lhandinitpos_sandia = KDL::Frame::Identity();
    self->T_graspgeometry_rhandinitpos_sandia = KDL::Frame::Identity();
    self->T_graspgeometry_lhandinitpos_irobot = KDL::Frame::Identity();
    self->T_graspgeometry_rhandinitpos_irobot = KDL::Frame::Identity();
       
  //Get initial position of hand relative to object geometry.
    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(self->object_selection);
        
    //Get initial position of hand relative to object geometry.
    //utility function defined in stickyhand_utils library
    //#include <vizualization_utils/stickyhand_utils/sticky_hand_utils.hpp>
    bool success = get_stickyhand_init_positions(self->object_selection, geometry_name ,obj_it->second,
                                  self->ray_start,self->ray_hit,self->ray_hit_drag,self->dragging,
                                  self->T_graspgeometry_lhandinitpos_sandia,self->T_graspgeometry_rhandinitpos_sandia,true);
    success = get_stickyhand_init_positions(self->object_selection, geometry_name ,obj_it->second,
                                  self->ray_start,self->ray_hit,self->ray_hit_drag,self->dragging,
                                  self->T_graspgeometry_lhandinitpos_irobot,self->T_graspgeometry_rhandinitpos_irobot,false);
  }// end void set_hand_init_position(void *user)
  //------------------------------------------------------------------------------- 
  
  
  
  inline static void set_object_desired_state_on_marker_motion(void *user,Eigen::Vector3f start,Eigen::Vector3f dir)
  {
      RendererAffordances *self = (RendererAffordances*) user;
      double gain = 1;

      std::string instance_name=  self->object_selection;
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = self->affCollection->_objects.find(instance_name);



      if(!it->second._gl_object->is_future_state_changing()) {
        it->second._gl_object->set_future_state_changing(true); 


		//  leave the clearing to the user to do this via reset                     
       // clear previously accumulated motion states for all dependent bodies
      /*  typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
        sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.begin();
        while (hand_it!=self->stickyHandCollection->_hands.end()) 
        {
           if (hand_it->second.object_name == (instance_name))
           {
              hand_it->second._gl_hand->clear_desired_body_motion_history();
           }
           hand_it++;
        }
        
        typedef std::map<std::string, StickyFootStruc > sticky_feet_map_type_;
        sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.begin();
        while (foot_it!=self->stickyFootCollection->_feet.end()) 
        {
           std::string foot_name = std::string(foot_it->second.object_name);
           if (foot_name == (it->first))
           {
              foot_it->second._gl_foot->clear_desired_body_motion_history();
           }
           foot_it++;
        } */
       }//end if(!it->second._gl_object->is_future_state_changing())
    
      
      // set desired state
      KDL::Frame T_world_object_future = it->second._gl_object->_T_world_body_future;
      KDL::Frame T_world_marker =  (it->second._gl_object->get_floatingbasemarker_frame()); 
      KDL::Frame T_marker_world =  T_world_marker.Inverse();
      KDL::Frame T_marker_object = T_marker_world*T_world_object_future;

      Eigen::Vector3f markerframe_prev_ray_hit_drag;
      rotate_eigen_vector_given_kdl_frame(self->prev_ray_hit_drag,T_marker_world,markerframe_prev_ray_hit_drag);
      Eigen::Vector3f markerframe_ray_hit_drag;
      rotate_eigen_vector_given_kdl_frame(self->ray_hit_drag,T_marker_world,markerframe_ray_hit_drag); 
      Eigen::Vector3f worldframe_delta,markerframe_delta;
      worldframe_delta  = self->ray_hit_drag-self->marker_offset_on_press;
      rotate_eigen_vector_given_kdl_frame(worldframe_delta,T_marker_world,markerframe_delta);          
      
      double currentAngle, angleTo,dtheta;       
      KDL::Frame DragRotation=KDL::Frame::Identity();       
      if(it->second._gl_object->is_bodypose_adjustment_enabled())
      {

        std::string token  = "plane::";
        size_t found = self->marker_selection.find(token);  
        if(found!=std::string::npos)  
        {
          string plane_name="";
          string root_link_name=it->second._gl_object->get_root_link_name();
          it->second._gl_object->extract_plane_name(root_link_name,plane_name);
          size_t found2 = plane_name.find("x"); 
          bool x_plane_active = (found2!=std::string::npos);
          found2 = plane_name.find("y"); 
          bool y_plane_active = (found2!=std::string::npos);       
          found2 = plane_name.find("z"); 
          bool z_plane_active = (found2!=std::string::npos);
          if(x_plane_active){
           T_marker_object.p[0] = markerframe_delta[0];
          }
          if(y_plane_active){
            T_marker_object.p[1] = markerframe_delta[1];
          }
          if(z_plane_active){
            T_marker_object.p[2] = markerframe_delta[2];
          }               
        }
        
        if(self->marker_selection=="markers::base_x"){
          T_marker_object.p[0] = markerframe_delta[0];
        }
        else if(self->marker_selection=="markers::base_y"){
          T_marker_object.p[1] = markerframe_delta[1];
        }      
        else if(self->marker_selection=="markers::base_z"){
          T_marker_object.p[2] = markerframe_delta[2];
        }    
        else if(self->marker_selection=="markers::base_roll"){
          currentAngle = atan2(markerframe_prev_ray_hit_drag[2]-T_marker_object.p[2],markerframe_prev_ray_hit_drag[1]-T_marker_object.p[1]);
          angleTo = atan2(markerframe_ray_hit_drag[2]-T_marker_object.p[2],markerframe_ray_hit_drag[1]-T_marker_object.p[1]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 1; axis[1] = 0; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        else if(self->marker_selection=="markers::base_pitch"){ 
          currentAngle = atan2(markerframe_prev_ray_hit_drag[0]-T_marker_object.p[0],markerframe_prev_ray_hit_drag[2]-T_marker_object.p[2]);
          angleTo = atan2(markerframe_ray_hit_drag[0]-T_marker_object.p[0],markerframe_ray_hit_drag[2]-T_marker_object.p[2]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 1; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }    
        else if(self->marker_selection=="markers::base_yaw"){
          currentAngle = atan2(markerframe_prev_ray_hit_drag[1]-T_marker_object.p[1],markerframe_prev_ray_hit_drag[0]-T_marker_object.p[0]);
          angleTo = atan2(markerframe_ray_hit_drag[1]-T_marker_object.p[1],markerframe_ray_hit_drag[0]-T_marker_object.p[0]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 0; axis[2]=1;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        
        T_marker_object.M  = DragRotation.M*T_marker_object.M; 
        T_world_object_future = (T_marker_world.Inverse())*T_marker_object;
 
        std::map<std::string, double> jointpos_in;
        jointpos_in = it->second._gl_object->_future_jointpos;
        it->second._gl_object->set_future_state(T_world_object_future,jointpos_in); 
      
      }
      else if(it->second._gl_object->is_jointdof_adjustment_enabled())
      {
        // set joint dof
        string object_name = self->object_selection; 
        string marker_name = self->marker_selection; 
        string token  = "markers::";
        size_t found = marker_name.find(token);  
        if (found==std::string::npos)
            return;
        string joint_name =marker_name.substr(found+token.size());
        //std::cout <<"markername: "<< marker_name<< " mouse on joint marker: " << joint_name << std::endl;

        visualization_utils::JointFrameStruct jointInfo;
        KDL::Frame T_world_object = it->second._gl_object->_T_world_body;
        KDL::Frame T_world_object_future = it->second._gl_object->_T_world_body_future;
        Eigen::Vector3f joint_axis;
        bool flip_criteria;
        KDL::Frame T_jointaxis_marker = KDL::Frame::Identity();

        // handle planar markers differently from cartesian markers
        if(it->second._gl_object->is_planar_coupling_active(joint_name))
        {
          std::string token  = "::translate";
          size_t found = joint_name.find(token); 
          if (found!=std::string::npos)
          {
             std::string second_axis_name;
             it->second._gl_object->get_second_axis_name(marker_name,second_axis_name);
             it->second._gl_object->get_joint_info(second_axis_name,jointInfo);
			       if(it->second._gl_object->is_future_display_active())        
            	 joint_axis << jointInfo.future_axis[0],jointInfo.future_axis[1],jointInfo.future_axis[2];
          	 else
             	 joint_axis << jointInfo.axis[0],jointInfo.axis[1],jointInfo.axis[2]; 
          	 
          	 joint_axis.normalize();
             if(!it->second._gl_object->modify_joint_axis_to_plane_normal(second_axis_name,joint_axis))
               return;  
             flip_criteria = it->second._gl_object->is_joint_axis_flipped(second_axis_name);  
          }
        }
        else
        {

          it->second._gl_object->get_joint_info(joint_name,jointInfo);
		  if(it->second._gl_object->is_future_display_active())        
            joint_axis << jointInfo.future_axis[0],jointInfo.future_axis[1],jointInfo.future_axis[2];
          else
            joint_axis << jointInfo.axis[0],jointInfo.axis[1],jointInfo.axis[2]; // in world frame
          joint_axis.normalize();
           
          flip_criteria = it->second._gl_object->is_joint_axis_flipped(joint_name);
          double arrow_length =0.2;
          if(flip_criteria) {
            T_jointaxis_marker.p[2] =-2*arrow_length/3; 
           }
          else{
            T_jointaxis_marker.p[2] = 2*arrow_length/3;     
          }

        }
        
    
        Eigen::Vector3f u_body_to_joint;
        KDL::Frame T_world_joint;
        T_world_joint = jointInfo.future_frame;
        u_body_to_joint[0] = T_world_object_future.p[0]-T_world_joint.p[0];
        u_body_to_joint[1] = T_world_object_future.p[1]-T_world_joint.p[1];
        u_body_to_joint[2] = T_world_object_future.p[2]-T_world_joint.p[2];
        u_body_to_joint.normalize();
        double normal = acos(u_body_to_joint.dot(joint_axis));
        double flipped = acos(u_body_to_joint.dot(-joint_axis));
        
        double theta;
        Eigen::Vector3f axis;      
        Eigen::Vector3f uz; 
        uz << 0 , 0 , 1; 
        axis = uz.cross(joint_axis);
        axis.normalize();
        theta = acos(uz.dot(joint_axis));
        
		    KDL::Frame T_world_marker = KDL::Frame::Identity();
        KDL::Frame T_world_jointaxis;// Axis
        T_world_jointaxis.p = jointInfo.future_frame.p;
        KDL::Vector axis_temp;
        axis_temp[0]=axis[0];axis_temp[1]=axis[1];axis_temp[2]=axis[2];
        T_world_jointaxis.M = KDL::Rotation::Rot(axis_temp,theta); //T_axis_world
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

          // handle planar markers differently from cartesian markers
          if(self->otdf_instance_hold._gl_object->is_planar_coupling_active(joint_name))
          {
            std::string token  = "::translate";
            size_t found = joint_name.find(token); 
            if (found!=std::string::npos)
            {
              self->ray_hit_drag << start[0]+self->ray_hit_t*dir[0],
                    start[1]+self->ray_hit_t*dir[1],
                    start[2]+self->ray_hit_t*dir[2]; 
              Eigen::Vector3f hitpt_obj_frame=self->marker_offset_on_press;
              Eigen::Vector3f dragpt_obj_frame;
              dragpt_obj_frame << self->ray_hit_drag[0]-T_world_object_future.p[0],
		                 self->ray_hit_drag[1]-T_world_object_future.p[1],
		                 self->ray_hit_drag[2]-T_world_object_future.p[2];
              Eigen::Vector3f da= dragpt_obj_frame-hitpt_obj_frame;
              
              std::string first_axis_name,second_axis_name;
              it->second._gl_object->get_first_axis_name(marker_name,first_axis_name);
              it->second._gl_object->get_second_axis_name(marker_name,second_axis_name);
              double first_axis_pos, second_axis_pos;
              first_axis_pos=it->second._gl_object->_future_jointpos.find(first_axis_name)->second;
              second_axis_pos=it->second._gl_object->_future_jointpos.find(second_axis_name)->second;
                
              it->second._gl_object->get_joint_info(first_axis_name,jointInfo);
              if(it->second._gl_object->is_future_display_active())        
                joint_axis << jointInfo.future_axis[0],jointInfo.future_axis[1],jointInfo.future_axis[2];
              else
                joint_axis << jointInfo.axis[0],jointInfo.axis[1],jointInfo.axis[2]; 
              joint_axis.normalize();
              first_axis_pos = self->joint_marker_pos_on_press + da.dot(joint_axis);  
              
              it->second._gl_object->get_joint_info(second_axis_name,jointInfo);
              if(it->second._gl_object->is_future_display_active())        
                joint_axis << jointInfo.future_axis[0],jointInfo.future_axis[1],jointInfo.future_axis[2];
              else
                joint_axis << jointInfo.axis[0],jointInfo.axis[1],jointInfo.axis[2]; 
              joint_axis.normalize();  
              second_axis_pos = self->coupled_joint_marker_pos_on_press + da.dot(joint_axis); 

              std::map<std::string, double> jointpos_in;
              jointpos_in = it->second._gl_object->_future_jointpos;
              jointpos_in.find(first_axis_name)->second = first_axis_pos;
              jointpos_in.find(second_axis_name)->second = second_axis_pos;
              it->second._gl_object->set_future_state(T_world_object_future,jointpos_in); 
              bot_viewer_request_redraw(self->viewer); 
            }
            
          }
          else
          {
         
		        self->ray_hit_drag << start[0]+self->ray_hit_t*dir[0],
		                              start[1]+self->ray_hit_t*dir[1],
		                              start[2]+self->ray_hit_t*dir[2]; 
		        Eigen::Vector3f hitpt_obj_frame=self->marker_offset_on_press;
		        Eigen::Vector3f dragpt_obj_frame;
		        dragpt_obj_frame << self->ray_hit_drag[0]-T_world_object_future.p[0],
		                 self->ray_hit_drag[1]-T_world_object_future.p[1],
		                 self->ray_hit_drag[2]-T_world_object_future.p[2];
		        Eigen::Vector3f da= dragpt_obj_frame-hitpt_obj_frame;
		        double desired_pos = self->joint_marker_pos_on_press + da.dot(joint_axis);

		         std::map<std::string, double> jointpos_in;
		         jointpos_in = it->second._gl_object->_future_jointpos;
		         jointpos_in.find(joint_name)->second = desired_pos;
		         it->second._gl_object->set_future_state(T_world_object_future,jointpos_in); 
		         bot_viewer_request_redraw(self->viewer); 
 		       }
         }// end else PRISMATIC
      }// end else if(it->second._gl_object->is_jointdof_adjustment_enabled())
      self->prev_ray_hit_drag = self->ray_hit_drag;
      
  }   // end set_object_desired_state_on_marker_motion()
  

  //------------------------------------------------------------------------------- 
  
  inline static void set_object_current_state_on_marker_motion(void *user,Eigen::Vector3f start,Eigen::Vector3f dir)
  {
      RendererAffordances *self = (RendererAffordances*) user;
      double gain = 1;

      std::string instance_name=  self->object_selection;

      // set current state
      KDL::Frame T_world_object =  self->otdf_instance_hold._gl_object->_T_world_body;
      KDL::Frame T_marker_world =  (self->otdf_instance_hold._gl_object->get_floatingbasemarker_frame()).Inverse(); 
      KDL::Frame T_marker_object = T_marker_world*T_world_object;
      Eigen::Vector3f markerframe_prev_ray_hit_drag;
      rotate_eigen_vector_given_kdl_frame(self->prev_ray_hit_drag,T_marker_world,markerframe_prev_ray_hit_drag);
      Eigen::Vector3f markerframe_ray_hit_drag;
      rotate_eigen_vector_given_kdl_frame(self->ray_hit_drag,T_marker_world,markerframe_ray_hit_drag); 
      Eigen::Vector3f worldframe_delta,markerframe_delta;
      worldframe_delta  = self->ray_hit_drag-self->marker_offset_on_press;
      rotate_eigen_vector_given_kdl_frame(worldframe_delta,T_marker_world,markerframe_delta);       
      
      double currentAngle, angleTo,dtheta;       
      KDL::Frame DragRotation=KDL::Frame::Identity();       
      if(self->otdf_instance_hold._gl_object->is_bodypose_adjustment_enabled())
      {
        std::string token  = "plane::";
        size_t found = self->marker_selection.find(token);  
        if(found!=std::string::npos)  
        {
          string plane_name="";
          string root_link_name=self->otdf_instance_hold._gl_object->get_root_link_name();
          self->otdf_instance_hold._gl_object->extract_plane_name(root_link_name,plane_name);
          size_t found2 = plane_name.find("x"); 
          bool x_plane_active = (found2!=std::string::npos);
          found2 = plane_name.find("y"); 
          bool y_plane_active = (found2!=std::string::npos);       
          found2 = plane_name.find("z"); 
          bool z_plane_active = (found2!=std::string::npos);
          
          if(x_plane_active){
           T_marker_object.p[0] = markerframe_delta[0];
          }
          if(y_plane_active){
            T_marker_object.p[1] = markerframe_delta[1];
          }
          if(z_plane_active){
            T_marker_object.p[2] = markerframe_delta[2];
          }        
        } 
      
        if(self->marker_selection=="markers::base_x"){
          T_marker_object.p[0] = markerframe_delta[0];
        }
        else if(self->marker_selection=="markers::base_y"){
          T_marker_object.p[1] = markerframe_delta[1];
        }      
        else if(self->marker_selection=="markers::base_z"){
          T_marker_object.p[2] = markerframe_delta[2];
        }    
        else if(self->marker_selection=="markers::base_roll"){
          currentAngle = atan2(markerframe_prev_ray_hit_drag[2]-T_marker_object.p[2],markerframe_prev_ray_hit_drag[1]-T_marker_object.p[1]);
          angleTo = atan2(markerframe_ray_hit_drag[2]-T_marker_object.p[2],markerframe_ray_hit_drag[1]-T_marker_object.p[1]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 1; axis[1] = 0; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        else if(self->marker_selection=="markers::base_pitch"){ 
          currentAngle = atan2(markerframe_prev_ray_hit_drag[0]-T_marker_object.p[0],markerframe_prev_ray_hit_drag[2]-T_marker_object.p[2]);
          angleTo = atan2(markerframe_ray_hit_drag[0]-T_marker_object.p[0],markerframe_ray_hit_drag[2]-T_marker_object.p[2]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 1; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }    
        else if(self->marker_selection=="markers::base_yaw"){
          currentAngle = atan2(markerframe_prev_ray_hit_drag[1]-T_marker_object.p[1],markerframe_prev_ray_hit_drag[0]-T_marker_object.p[0]);
          angleTo = atan2(markerframe_ray_hit_drag[1]-T_marker_object.p[1],markerframe_ray_hit_drag[0]-T_marker_object.p[0]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 0; axis[2]=1;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        
        T_marker_object.M  = DragRotation.M*T_marker_object.M; 
        T_world_object = (T_marker_world.Inverse())*T_marker_object;
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
        KDL::Frame T_world_object = self->otdf_instance_hold._gl_object->_T_world_body;
        Eigen::Vector3f joint_axis;
        bool flip_criteria;
        KDL::Frame T_jointaxis_marker = KDL::Frame::Identity();
        

        // handle planar markers differently from cartesian markers
        if(self->otdf_instance_hold._gl_object->is_planar_coupling_active(joint_name))
        {
          std::string token  = "::translate";
          size_t found = joint_name.find(token); 
          if (found!=std::string::npos)
          {
             std::string second_axis_name;
             self->otdf_instance_hold._gl_object->get_second_axis_name(marker_name,second_axis_name);
             self->otdf_instance_hold._gl_object->get_joint_info(second_axis_name,jointInfo);
             joint_axis << jointInfo.axis[0],jointInfo.axis[1],jointInfo.axis[2];
             joint_axis.normalize();
             if(!self->otdf_instance_hold._gl_object->modify_joint_axis_to_plane_normal(second_axis_name,joint_axis))
               return;  
              flip_criteria = self->otdf_instance_hold._gl_object->is_joint_axis_flipped(second_axis_name);  
          }
        }
        else
        {
          self->otdf_instance_hold._gl_object->get_joint_info(joint_name,jointInfo);
          joint_axis << jointInfo.axis[0],jointInfo.axis[1],jointInfo.axis[2];
          joint_axis.normalize();
           
          flip_criteria = self->otdf_instance_hold._gl_object->is_joint_axis_flipped(joint_name);
          double arrow_length =0.2;
          if(flip_criteria) {
            T_jointaxis_marker.p[2] =-2*arrow_length/3; 
           }
          else{
            T_jointaxis_marker.p[2] = 2*arrow_length/3;     
          }
        }
        
      
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

          // handle planar markers differently from cartesian markers
          if(self->otdf_instance_hold._gl_object->is_planar_coupling_active(joint_name))
          {

            std::string token  = "::translate";
            size_t found = joint_name.find(token); 
            if (found!=std::string::npos)
            {
              self->ray_hit_drag << start[0]+self->ray_hit_t*dir[0],
                    start[1]+self->ray_hit_t*dir[1],
                    start[2]+self->ray_hit_t*dir[2]; 
              Eigen::Vector3f hitpt_obj_frame=self->marker_offset_on_press;
              Eigen::Vector3f dragpt_obj_frame;
              KDL::Frame T_world_object;
              T_world_object = self->otdf_instance_hold._gl_object->_T_world_body;
              dragpt_obj_frame << self->ray_hit_drag[0]-T_world_object.p[0],
              self->ray_hit_drag[1]-T_world_object.p[1],
              self->ray_hit_drag[2]-T_world_object.p[2];
              Eigen::Vector3f da= dragpt_obj_frame-hitpt_obj_frame;
              
              std::string first_axis_name,second_axis_name;
              self->otdf_instance_hold._gl_object->get_first_axis_name(marker_name,first_axis_name);
              self->otdf_instance_hold._gl_object->get_second_axis_name(marker_name,second_axis_name);
              double first_axis_pos, second_axis_pos,vel;
              self->otdf_instance_hold._otdf_instance->getJointState(first_axis_name, first_axis_pos,vel);     
              self->otdf_instance_hold._otdf_instance->getJointState(second_axis_name, second_axis_pos,vel);  
                
              self->otdf_instance_hold._gl_object->get_joint_info(first_axis_name,jointInfo);
              joint_axis << jointInfo.axis[0],jointInfo.axis[1],jointInfo.axis[2];
              joint_axis.normalize();   
              first_axis_pos = self->joint_marker_pos_on_press + da.dot(joint_axis);  
              
              self->otdf_instance_hold._gl_object->get_joint_info(second_axis_name,jointInfo);
              joint_axis << jointInfo.axis[0],jointInfo.axis[1],jointInfo.axis[2];
              joint_axis.normalize();   
              second_axis_pos = self->coupled_joint_marker_pos_on_press + da.dot(joint_axis); 
 
              self->otdf_instance_hold._otdf_instance->setJointState(first_axis_name, first_axis_pos,0); 
              self->otdf_instance_hold._otdf_instance->setJointState(second_axis_name, second_axis_pos,0);
            }
            self->otdf_instance_hold._otdf_instance->update(); 
            self->otdf_instance_hold._gl_object->set_state(self->otdf_instance_hold._otdf_instance); 
          }
          else
          {
            self->ray_hit_drag << start[0]+self->ray_hit_t*dir[0],
                                  start[1]+self->ray_hit_t*dir[1],
                                  start[2]+self->ray_hit_t*dir[2]; 
            Eigen::Vector3f hitpt_obj_frame=self->marker_offset_on_press;
            Eigen::Vector3f dragpt_obj_frame;
            KDL::Frame T_world_object;
              T_world_object = self->otdf_instance_hold._gl_object->_T_world_body;
              dragpt_obj_frame << self->ray_hit_drag[0]-T_world_object.p[0],
                     self->ray_hit_drag[1]-T_world_object.p[1],
                     self->ray_hit_drag[2]-T_world_object.p[2];
            Eigen::Vector3f da= dragpt_obj_frame-hitpt_obj_frame;
            double desired_pos = self->joint_marker_pos_on_press + da.dot(joint_axis);
            self->otdf_instance_hold._otdf_instance->setJointState(joint_name, desired_pos,0); 
            self->otdf_instance_hold._otdf_instance->update(); 
            self->otdf_instance_hold._gl_object->set_state(self->otdf_instance_hold._otdf_instance); 
          }
        }    
      }//end else if(it->second._gl_object->is_jointdof_adjustment_enabled())
          
      self->prev_ray_hit_drag = self->ray_hit_drag; 
  }   // end set_object_current_state_on_marker_motion()
  
//-------------------------------------------------------------------------------
 inline static void set_stickyhand_current_state_on_marker_motion(void *user,Eigen::Vector3f start,Eigen::Vector3f dir)
  {
      RendererAffordances *self = (RendererAffordances*) user;
      double gain = 1;
      
      typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.find(self->stickyhand_selection);

      // set current state
      KDL::Frame T_geometry_hand =  hand_it->second._gl_hand->_T_world_body;  // hand in aff frame
      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(string(hand_it->second.object_name));
      KDL::Frame T_world_object = obj_it->second._gl_object->_T_world_body;
      KDL::Frame T_world_geometry;
       obj_it->second._gl_object->get_link_geometry_frame(string(hand_it->second.geometry_name),T_world_geometry);
      KDL::Frame T_marker_world =  (hand_it->second._gl_hand->get_floatingbasemarker_frame()).Inverse(); // marker in world frame
      
      KDL::Frame T_marker_hand = T_marker_world*T_world_geometry*T_geometry_hand;
      
      Eigen::Vector3f markerframe_prev_ray_hit_drag;
      rotate_eigen_vector_given_kdl_frame(self->prev_ray_hit_drag,T_marker_world,markerframe_prev_ray_hit_drag);
      Eigen::Vector3f markerframe_ray_hit_drag;
      rotate_eigen_vector_given_kdl_frame(self->ray_hit_drag,T_marker_world,markerframe_ray_hit_drag); 
      Eigen::Vector3f worldframe_delta,markerframe_delta;
      worldframe_delta  = self->ray_hit_drag-self->marker_offset_on_press;
      rotate_eigen_vector_given_kdl_frame(worldframe_delta,T_marker_world,markerframe_delta);       
      
      double currentAngle, angleTo,dtheta;       
      KDL::Frame DragRotation=KDL::Frame::Identity();       
      if(hand_it->second._gl_hand->is_bodypose_adjustment_enabled())
      {
        std::string token  = "plane::";
        size_t found = self->marker_selection.find(token);  
        if(found!=std::string::npos)  
        {
          string plane_name="";
          string root_link_name=hand_it->second._gl_hand->get_root_link_name();
          hand_it->second._gl_hand->extract_plane_name(root_link_name,plane_name);
          size_t found2 = plane_name.find("x"); 
          bool x_plane_active = (found2!=std::string::npos);
          found2 = plane_name.find("y"); 
          bool y_plane_active = (found2!=std::string::npos);       
          found2 = plane_name.find("z"); 
          bool z_plane_active = (found2!=std::string::npos);
          
          if(x_plane_active){
           T_marker_hand.p[0] = markerframe_delta[0];
          }
          if(y_plane_active){
            T_marker_hand.p[1] = markerframe_delta[1];
          }
          if(z_plane_active){
            T_marker_hand.p[2] = markerframe_delta[2];
          }        
        } 
      
        if(self->marker_selection=="markers::base_x"){
          T_marker_hand.p[0] = markerframe_delta[0];
        }
        else if(self->marker_selection=="markers::base_y"){
          T_marker_hand.p[1] = markerframe_delta[1];
        }      
        else if(self->marker_selection=="markers::base_z"){
          T_marker_hand.p[2] = markerframe_delta[2];
        }    
        else if(self->marker_selection=="markers::base_roll"){
          currentAngle = atan2(markerframe_prev_ray_hit_drag[2]-T_marker_hand.p[2],markerframe_prev_ray_hit_drag[1]-T_marker_hand.p[1]);
          angleTo = atan2(markerframe_ray_hit_drag[2]-T_marker_hand.p[2],markerframe_ray_hit_drag[1]-T_marker_hand.p[1]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 1; axis[1] = 0; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        else if(self->marker_selection=="markers::base_pitch"){ 
          currentAngle = atan2(markerframe_prev_ray_hit_drag[0]-T_marker_hand.p[0],markerframe_prev_ray_hit_drag[2]-T_marker_hand.p[2]);
          angleTo = atan2(markerframe_ray_hit_drag[0]-T_marker_hand.p[0],markerframe_ray_hit_drag[2]-T_marker_hand.p[2]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 1; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }    
        else if(self->marker_selection=="markers::base_yaw"){
          currentAngle = atan2(markerframe_prev_ray_hit_drag[1]-T_marker_hand.p[1],markerframe_prev_ray_hit_drag[0]-T_marker_hand.p[0]);
          angleTo = atan2(markerframe_ray_hit_drag[1]-T_marker_hand.p[1],markerframe_ray_hit_drag[0]-T_marker_hand.p[0]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 0; axis[2]=1;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        
        T_marker_hand.M  = DragRotation.M*T_marker_hand.M; 
        T_geometry_hand = (T_marker_world*T_world_geometry).Inverse()*T_marker_hand;
       drc::joint_angles_t posture_msg;
       posture_msg.num_joints = hand_it->second.joint_position.size();
       posture_msg.joint_name = hand_it->second.joint_name;
       posture_msg.joint_position = hand_it->second.joint_position;
       string unique_name = hand_it->first;
       self->stickyHandCollection->add_or_update_sticky_hand(hand_it->second.uid,unique_name,T_geometry_hand, posture_msg);
      }
              
      self->prev_ray_hit_drag = self->ray_hit_drag; 
  }   // end set_stickyhand_current_state_on_marker_motion()
  
//-------------------------------------------------------------------------------
 inline static void set_stickyfoot_current_state_on_marker_motion(void *user,Eigen::Vector3f start,Eigen::Vector3f dir)
  {
      RendererAffordances *self = (RendererAffordances*) user;
      double gain = 1;
      
      typedef std::map<std::string, StickyFootStruc > sticky_feet_map_type_;
      sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.find(self->stickyfoot_selection);

      // set current state
      KDL::Frame T_geometry_foot =  foot_it->second._gl_foot->_T_world_body;  // foot in aff frame
      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(string(foot_it->second.object_name));
      //KDL::Frame T_world_object = obj_it->second._gl_object->_T_world_body;
      KDL::Frame T_world_geometry;
      obj_it->second._gl_object->get_link_geometry_frame(string(foot_it->second.geometry_name),T_world_geometry);
      KDL::Frame T_marker_world =  (foot_it->second._gl_foot->get_floatingbasemarker_frame()).Inverse(); // marker in world frame
      KDL::Frame T_marker_foot = T_marker_world*T_world_geometry*T_geometry_foot;
      
      Eigen::Vector3f markerframe_prev_ray_hit_drag;
      rotate_eigen_vector_given_kdl_frame(self->prev_ray_hit_drag,T_marker_world,markerframe_prev_ray_hit_drag);
      Eigen::Vector3f markerframe_ray_hit_drag;
      rotate_eigen_vector_given_kdl_frame(self->ray_hit_drag,T_marker_world,markerframe_ray_hit_drag); 
      Eigen::Vector3f worldframe_delta,markerframe_delta;
      worldframe_delta  = self->ray_hit_drag-self->marker_offset_on_press;
      rotate_eigen_vector_given_kdl_frame(worldframe_delta,T_marker_world,markerframe_delta);       
      
      double currentAngle, angleTo,dtheta;       
      KDL::Frame DragRotation=KDL::Frame::Identity();       
      if(foot_it->second._gl_foot->is_bodypose_adjustment_enabled())
      {
        std::string token  = "plane::";
        size_t found = self->marker_selection.find(token);  
        if(found!=std::string::npos)  
        {
          string plane_name="";
          string root_link_name=foot_it->second._gl_foot->get_root_link_name();
          foot_it->second._gl_foot->extract_plane_name(root_link_name,plane_name);
          size_t found2 = plane_name.find("x"); 
          bool x_plane_active = (found2!=std::string::npos);
          found2 = plane_name.find("y"); 
          bool y_plane_active = (found2!=std::string::npos);       
          found2 = plane_name.find("z"); 
          bool z_plane_active = (found2!=std::string::npos);
          
          if(x_plane_active){
           T_marker_foot.p[0] = markerframe_delta[0];
          }
          if(y_plane_active){
            T_marker_foot.p[1] = markerframe_delta[1];
          }
          if(z_plane_active){
            T_marker_foot.p[2] = markerframe_delta[2];
          }        
        } 
      
        if(self->marker_selection=="markers::base_x"){
          T_marker_foot.p[0] = markerframe_delta[0];
        }
        else if(self->marker_selection=="markers::base_y"){
          T_marker_foot.p[1] = markerframe_delta[1];
        }      
        else if(self->marker_selection=="markers::base_z"){
          T_marker_foot.p[2] = markerframe_delta[2];
        }    
        else if(self->marker_selection=="markers::base_roll"){
          currentAngle = atan2(markerframe_prev_ray_hit_drag[2]-T_marker_foot.p[2],markerframe_prev_ray_hit_drag[1]-T_marker_foot.p[1]);
          angleTo = atan2(markerframe_ray_hit_drag[2]-T_marker_foot.p[2],markerframe_ray_hit_drag[1]-T_marker_foot.p[1]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 1; axis[1] = 0; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        else if(self->marker_selection=="markers::base_pitch"){ 
          currentAngle = atan2(markerframe_prev_ray_hit_drag[0]-T_marker_foot.p[0],markerframe_prev_ray_hit_drag[2]-T_marker_foot.p[2]);
          angleTo = atan2(markerframe_ray_hit_drag[0]-T_marker_foot.p[0],markerframe_ray_hit_drag[2]-T_marker_foot.p[2]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 1; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }    
        else if(self->marker_selection=="markers::base_yaw"){
          currentAngle = atan2(markerframe_prev_ray_hit_drag[1]-T_marker_foot.p[1],markerframe_prev_ray_hit_drag[0]-T_marker_foot.p[0]);
          angleTo = atan2(markerframe_ray_hit_drag[1]-T_marker_foot.p[1],markerframe_ray_hit_drag[0]-T_marker_foot.p[0]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 0; axis[2]=1;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        
        T_marker_foot.M  = DragRotation.M*T_marker_foot.M; 
        T_geometry_foot = (T_marker_world*T_world_geometry).Inverse()*T_marker_foot;
        
        self->stickyFootCollection->add_or_update_sticky_foot(foot_it->second.uid,
                                                             foot_it->second.foot_type,
                                                             foot_it->second.object_name,
                                                             foot_it->second.geometry_name,
                                                             T_geometry_foot,
                                                             foot_it->second.joint_name,
                                                             foot_it->second.joint_position);
      }
              
      self->prev_ray_hit_drag = self->ray_hit_drag; 
  }   // end set_stickyfoot_current_state_on_marker_motion()
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
    self->affCollection->clear_highlights();
    if(!self->seedSelectionManager->is_shift_pressed())
    {
      self->stickyHandCollection->clear_highlights();
      self->stickyFootCollection->clear_highlights();

      
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
              //self->ray_hit = hit_pt;
              //self->ray_hit_t = (hit_pt - self->ray_start).norm();
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
               
               //intersected_object = NULL; 
          }
                             
        }// end if(...is_bodypose_adjustment_enabled)||...->is_jointdof_adjustment_enabled))

      }// end if((self->otdf_instance_hold._gl_object)&&(self->selection_hold_on))


      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;

      // loop through object list and check if ray intersect any of them.
      for(object_instance_map_type_::const_iterator it = self->affCollection->_objects.begin(); it!=self->affCollection->_objects.end(); it++)
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
                //self->ray_hit = hit_pt;
                //self->ray_hit_t = (hit_pt - self->ray_start).norm();
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
                 
                 //intersected_object = NULL; 
            }
                          
          }



           it->second._gl_object->_collision_detector->ray_test( from, to, intersected_object,hit_pt,hit_normal);
          // Highlight all objects that intersect with ray
          if(intersected_object != NULL ){
                //self->ray_hit = hit_pt;
                //self->ray_hit_t = (hit_pt - self->ray_start).norm();
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
     
        }// end if object exists
      }// end for
      
   }// end if(!self->seedSelectionManager->is_shift_pressed()) // if shift is pressed ignore objects 
    
    // Dont Draw Seeds If the renderer is foviated. 
    if(self->_renderer_foviate)
    {
      self->prev_ray_hit_drag = self->ray_hit_drag;
      return shortest_distance;
    }
    
   //loop through stick-feet list and check if ray intersect any of them.
    typedef map<string, StickyFootStruc > sticky_feet_map_type_;
    for(sticky_feet_map_type_::iterator it = self->stickyFootCollection->_feet.begin(); it!=self->stickyFootCollection->_feet.end(); it++)
    {
 
          if(it->second._gl_foot->is_bodypose_adjustment_enabled())
          {
            it->second._gl_foot->_collision_detector_floatingbase_markers->ray_test( from, to, intersected_object,hit_pt);
            if(intersected_object != NULL ){
                //self->ray_hit = hit_pt;
                //self->ray_hit_t = (hit_pt - self->ray_start).norm();
                Eigen::Vector3f diff = (from-hit_pt);
                double distance = diff.norm();
                if(shortest_distance>0) {
                  if (distance < shortest_distance)
                  {
                    shortest_distance = distance;
                    self->ray_hit = hit_pt;
                    self->ray_hit_drag = hit_pt;
                    self->ray_hit_t = (hit_pt - self->ray_start).norm();
                    self->object_selection  =  " ";
                    self->link_selection  =  " ";
                    self->marker_selection  = string(intersected_object->id().c_str());
                    self->stickyhand_selection  = " ";
                    self->stickyfoot_selection  = it->first;
                  }
                }
                else {
                  shortest_distance = distance;
                  self->ray_hit = hit_pt;
                  self->ray_hit_drag = hit_pt;
                  self->ray_hit_t = (hit_pt - self->ray_start).norm();
                  self->object_selection  =  " ";
                  self->link_selection  =  " ";
                  self->marker_selection  = string(intersected_object->id().c_str());
                  self->stickyhand_selection  = " ";
                  self->stickyfoot_selection  = it->first;
                 }
            }                
          }// end if(...is_bodypose_adjustment_enabled))

          KDL::Frame T_world_graspgeometry = KDL::Frame::Identity();       
          typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
          object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(it->second.object_name);
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
          
         }// end if
    
    }// end for   
    

    //loop through stick-hands list and check if ray intersect any of them.
    typedef map<string, StickyHandStruc > sticky_hands_map_type_;
    for(sticky_hands_map_type_::iterator it = self->stickyHandCollection->_hands.begin(); it!=self->stickyHandCollection->_hands.end(); it++)
    {

          if(it->second._gl_hand->is_bodypose_adjustment_enabled())
          {
            it->second._gl_hand->_collision_detector_floatingbase_markers->ray_test( from, to, intersected_object,hit_pt);
            if(intersected_object != NULL ){
                //self->ray_hit = hit_pt;
                //self->ray_hit_t = (hit_pt - self->ray_start).norm();
                Eigen::Vector3f diff = (from-hit_pt);
                double distance = diff.norm();
                if(shortest_distance>0) {
                  if (distance < shortest_distance)
                  {
                    shortest_distance = distance;
                    self->ray_hit = hit_pt;
                    self->ray_hit_drag = hit_pt;
                    self->ray_hit_t = (hit_pt - self->ray_start).norm();
                    self->object_selection  =  " ";
                    self->link_selection  =  " ";
                    self->marker_selection  = string(intersected_object->id().c_str());
                    self->stickyhand_selection  = it->first;
                    self->stickyfoot_selection  = " ";
                  }
                }
                else {
                  shortest_distance = distance;
                  self->ray_hit = hit_pt;
                  self->ray_hit_drag = hit_pt;
                  self->ray_hit_t = (hit_pt - self->ray_start).norm();
                  self->object_selection  =  " ";
                  self->link_selection  =  " ";
                  self->marker_selection  = string(intersected_object->id().c_str());
                  self->stickyhand_selection  = it->first;
                  self->stickyfoot_selection  = " ";
                 }
            }                
          }// end if(...is_bodypose_adjustment_enabled))
              
          KDL::Frame T_world_graspgeometry = KDL::Frame::Identity();       
          typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
          object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(it->second.object_name);
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
        
         }// end if
    
    }// end for
    
    self->prev_ray_hit_drag = self->ray_hit_drag;
    return shortest_distance;
  }
  
}//end_namespace

void setup_renderer_affordances(BotViewer *viewer, int render_priority, lcm_t* lcm, BotFrames *frames, KeyboardSignalRef signalRef,AffTriggerSignalsRef affTriggerSignalsRef,RendererFoviationSignalRef rendererFoviationSignalRef);
void setup_renderer_affordances(BotViewer *viewer, int render_priority, lcm_t* lcm, KeyboardSignalRef signalRef,AffTriggerSignalsRef affTriggerSignalsRef,RendererFoviationSignalRef rendererFoviationSignalRef);


#endif //RENDERER_AFFORDANCES_HPP
