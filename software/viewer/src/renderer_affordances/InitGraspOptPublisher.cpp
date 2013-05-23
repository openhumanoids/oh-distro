#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "InitGraspOptPublisher.hpp"
#include <algorithm>

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;

namespace renderer_affordances 
{
  //==================constructor / destructor

//   InitGraspOptPublisher::InitGraspOptPublisher(boost::shared_ptr<lcm::LCM> &lcm, RendererAffordances* affordance_renderer):
//     _lcm(lcm),
//     _parent_renderer(affordance_renderer)
  InitGraspOptPublisher::InitGraspOptPublisher(RendererAffordances* affordance_renderer):
    _parent_renderer(affordance_renderer)
  {
 
    _lcm = affordance_renderer->lcm; 
   
    if(!_lcm->good())
    {
      cerr << "\nLCM Not Good: Init GraspOpt Publisher" << endl;
      return;
    }

  }

  InitGraspOptPublisher::~InitGraspOptPublisher() {

  }

//-------------------------------------------------------------------------------------      
//=============message callbacks

//std::string channel = "INIT_GRASP_OPT"+next_available_opt;
//publishGraspOptControlMsg("INIT_GRASP_OPT",T_geom_approach,KDL::Frame::Identity(),0,0,0);

  void InitGraspOptPublisher::publishGraspOptControlMsg(const std::string& channel, const KDL::Frame &T_geom_lhandpose,  const KDL::Frame &T_geom_rhandpose,const int grasp_type,const int contact_mask,const int drake_control, const int uid)				 
  {
    
    drc::grasp_opt_control_t msg;
    msg.utime = _parent_renderer->last_state_msg_timestamp; // best estimate of latest sim time
    msg.robot_name = _parent_renderer->robot_name;
    msg.object_name = _parent_renderer->object_selection;
    
    string object_geometry_name = _parent_renderer->link_selection;
    string object_name_token  =msg.object_name + "_";
    size_t found = object_geometry_name.find(object_name_token);  
    string geometry_name =object_geometry_name.substr(found+object_name_token.size());
  
    msg.geometry_name = geometry_name;
    msg.unique_id = uid; 
    msg.drake_control = drake_control;
    //grasp_type and contact_mask are set by the rightclk pop up event manager
    msg.grasp_type = grasp_type;//msg.SANDIA_LEFT; // how to set this? 
    msg.contact_mask = contact_mask;//msg.ALL;
    
    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it= _parent_renderer->instantiated_objects.find(msg.object_name);
    
    
    boost::shared_ptr<otdf::Geometry> link_geom;
    if(!it->second._gl_object->get_link_geometry(msg.geometry_name,link_geom)){
     cerr <<"ERROR in InitGraspOptPublisher: " << msg.geometry_name << " not found in KinematicBody Class (_gl_object)" << endl;
     return;
    }
    else
    {
      int type = link_geom->type;
      
      enum {SPHERE, BOX, CYLINDER, MESH, TORUS}; 
      
      if(type==SPHERE)  {
        shared_ptr<otdf::Sphere> sphere(shared_dynamic_cast<otdf::Sphere>(link_geom));	
        msg.geometry_type = msg.SPHERE;
        msg.num_dims = 1;
        msg.dims.push_back(sphere->radius);

      }
      else if(type==BOX)  {
        shared_ptr<otdf::Box> box(shared_dynamic_cast<otdf::Box>(link_geom));
        msg.geometry_type = msg.BOX;
        msg.num_dims = 3;
        msg.dims.push_back(box->dim.x);
        msg.dims.push_back(box->dim.y);
        msg.dims.push_back(box->dim.z);
      }
      else if(type==CYLINDER) {
   
        shared_ptr<otdf::Cylinder> cyl(shared_dynamic_cast<otdf::Cylinder>(link_geom));
   
        msg.geometry_type = msg.CYLINDER;
        msg.num_dims = 2;
        msg.dims.push_back(cyl->radius);
        msg.dims.push_back(cyl->length);
        
      }
      else if(type==MESH) { // meshes are not supported for grasping (for the moment treat them as boxes?)
        MeshStruct mesh_struct;
        if(it->second._gl_object->get_mesh_struct(msg.geometry_name,mesh_struct))
        {
            msg.geometry_type = msg.BOX;
            msg.num_dims = 3;
            msg.dims.push_back(mesh_struct.span_x);
            msg.dims.push_back(mesh_struct.span_y);
            msg.dims.push_back(mesh_struct.span_z);
        }
      }
      else if(type==TORUS)  {
       boost::shared_ptr<otdf::Torus> torus(boost::shared_dynamic_cast<otdf::Torus>(link_geom));
        msg.geometry_type = msg.TORUS;
        msg.num_dims = 2;
        msg.dims.push_back(torus->radius);
        msg.dims.push_back(torus->tube_radius);
      }
    } // end else
    
    drc::position_3d_t lhandpose;
    lhandpose.translation.x = T_geom_lhandpose.p[0];
    lhandpose.translation.y = T_geom_lhandpose.p[1];
    lhandpose.translation.z = T_geom_lhandpose.p[2];
    T_geom_lhandpose.M.GetQuaternion(lhandpose.rotation.x, lhandpose.rotation.y, lhandpose.rotation.z, lhandpose.rotation.w);
    
    drc::position_3d_t rhandpose;
    rhandpose.translation.x = T_geom_rhandpose.p[0];
    rhandpose.translation.y = T_geom_rhandpose.p[1];
    rhandpose.translation.z = T_geom_rhandpose.p[2];
    T_geom_rhandpose.M.GetQuaternion(rhandpose.rotation.x, rhandpose.rotation.y, rhandpose.rotation.z, rhandpose.rotation.w);

    msg.l_hand_init_pose = lhandpose;
    msg.r_hand_init_pose = rhandpose;
    
    _lcm->publish(channel, &msg);

  } 

//-------------------------------------------------------------------------------------        



} //end namespace 

//struct grasp_opt_control_t 
//{
//    int64_t utime;    
//    string robot_name; // parent robot name
//    
//    // unique way to associate a grasp with a object geometry in the scene
//    // object_name + geometry name + "_grasp_" + unique_hand_id;
//    // e.g. steering_wheel_1_torus_grasp_1 where {object:"steering_wheel_1"} {geometry: "torus"}
//    string object_name; 
//    string geometry_name; 
//    int32_t  unique_id;
//    
//    int16_t drake_control;
//    const int16_t NEW=0, RESET=1, HALT=2; // is interruption for reset or halting even possible in drake?
//        
//    //The type of hand (left, right or both) to be used 
//    int16_t grasp_type; // see constants below 
//    const int16_t SANDIA_LEFT=0, SANDIA_RIGHT=1, SANDIA_BOTH=2, IROBOT_LEFT=3, IROBOT_RIGHT=4, IROBOT_BOTH=5;
//    
//    // power grasp or precision grasp, for small objects a finger grasp is required.
//    int16_t contact_mask;// see constants below 
//    const int16_t ALL=0, FINGERS_ONLY=1; 
//    
//    //The type of object to be grasped
//    int16_t geometry_type; // see constants below 
//    const int16_t SPHERE=0, CYLINDER=1, BOX=2, TORUS=4; 
//    int32_t num_dims;
//    double dims[num_dims]; // sphere: dims = [radius]; cylinder: dims = [radius,length];box: dims = [span_x,span_y,span_z]; torus: dims = [radius,tube_radius];
//    
//    // initial positions of left/right hands in object frame;
//    position_3d_t l_hand_init_pose; 
//    position_3d_t r_hand_init_pose; 
//}


