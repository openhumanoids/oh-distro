#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "InitGraspOptPublisher.hpp"
#include <algorithm>
#include "lcmtypes/drc/grasp_opt_control_t.hpp"

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;

namespace renderer_affordances 
{
  //==================constructor / destructor

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

  void InitGraspOptPublisher::publishGraspOptControlMsg(const std::string& channel, const KDL::Frame &T_geom_lhandpose,  
    const KDL::Frame &T_geom_rhandpose,const int grasp_type,const int contact_mask,
    const int drake_control, const int uid, double dilation, Eigen::Vector3f &ray_hit)
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
    msg.grasp_type = grasp_type;//msg.SANDIA_LEFT etc... defined in drc::grasp_opt_control_t and desired_grasp_state_t
    msg.contact_mask = contact_mask;//msg.ALL;
    
    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it= _parent_renderer->affCollection->_objects.find(msg.object_name);
    
    
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
        boost::shared_ptr<otdf::Sphere> sphere(shared_dynamic_cast<otdf::Sphere>(link_geom));	
        msg.geometry_type = msg.SPHERE;
        msg.num_dims = 1;
        msg.dims.push_back(dilation*sphere->radius);

      }
      else if(type==BOX)  {
        boost::shared_ptr<otdf::Box> box(shared_dynamic_cast<otdf::Box>(link_geom));
        msg.geometry_type = msg.BOX;
        msg.num_dims = 3;
        msg.dims.push_back(dilation*box->dim.x);
        msg.dims.push_back(dilation*box->dim.y);
        msg.dims.push_back(dilation*box->dim.z);
      }
      else if(type==CYLINDER) {
   
        boost::shared_ptr<otdf::Cylinder> cyl(shared_dynamic_cast<otdf::Cylinder>(link_geom));
   
        msg.geometry_type = msg.CYLINDER;
        msg.num_dims = 2;
        msg.dims.push_back(dilation*cyl->radius);
        msg.dims.push_back(dilation*cyl->length);
        
      }
      else if(type==MESH) { // meshes are not supported for grasping (for the moment treat them as boxes?)
        MeshStruct mesh_struct;
        if(it->second._gl_object->get_mesh_struct(msg.geometry_name,mesh_struct))
        {
            msg.geometry_type = msg.BOX;
            msg.num_dims = 3;
            msg.dims.push_back(dilation*mesh_struct.span_x);
            msg.dims.push_back(dilation*mesh_struct.span_y);
            msg.dims.push_back(dilation*mesh_struct.span_z);
        }
      }
      else if(type==TORUS)  {
       boost::shared_ptr<otdf::Torus> torus(boost::shared_dynamic_cast<otdf::Torus>(link_geom));
        msg.geometry_type = msg.TORUS;
        msg.num_dims = 2;
        msg.dims.push_back(dilation*torus->radius);
        msg.dims.push_back(dilation*torus->tube_radius);
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
    
    // Also send the xyz location of where the ray intersected the affordance:
    msg.ray_hit[0] = ray_hit(0);
    msg.ray_hit[1] = ray_hit(1);
    msg.ray_hit[2] = ray_hit(2);
    _lcm->publish(channel, &msg);

  } 

//-------------------------------------------------------------------------------------        

} //end namespace 

