#ifndef GL_KINEMATIC_BODY_HPP
#define GL_KINEMATIC_BODY_HPP

#include <boost/function.hpp>
#include <map>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <forward_kinematics/treefksolverposfull_recursive.hpp>
#include <otdf_parser/otdf_parser.h>
#include <otdf_parser/otdf_urdf_converter.h>

#include <lcmtypes/bot_core.hpp>
#include <bot_vis/bot_vis.h>
#include <bot_core/bot_core.h>
#include <path_util/path_util.h>
#include <bot_frames/bot_frames.h>
#include <Eigen/Dense>
#include <iostream>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include "gl_draw_utils.hpp"

namespace visualization_utils {

struct MeshStruct 
{
  GLuint displaylist;
  double span_x;
  double span_y;
  double span_z;
  double offset_x; // vertices are not always defined in local link frame. In the drc robot sdf, the vertices are defined in parent joint coordinates.
  double offset_y;
  double offset_z; 
};

struct  LinkFrameStruct
{
    std::string name;
    KDL::Frame frame;
}; 
  
class GlKinematicBody
{
  protected: 
    std::string _urdf_xml_string; 
    std::vector<std::string> _joint_names_;
    std::map<std::string, boost::shared_ptr<urdf::Link> > _links_map;
    std::map<std::string, boost::shared_ptr<otdf::Link> > _otdf_links_map;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> _fksolver;

    std::vector<std::string > _link_names;
    std::vector<boost::shared_ptr<urdf::Geometry> > _link_shapes;
    std::vector<boost::shared_ptr<otdf::Geometry> > _otdf_link_shapes; //for affordance display we want to use otdf::Geometry
    std::vector<LinkFrameStruct> _link_tfs;     
    std::map<std::string, MeshStruct > _mesh_map; // associates link name with meshstruct
  //Avoids loading the same mesh multiple times.
    std::map<std::string, MeshStruct > _mesh_model_map; // associates file name with meshstruct

  
    bool visualize_bbox; 
    bool initialized;
    bool is_otdf_instance;
   
   public:
    // Constructors and destructor
    //GlKinematicBody( const GlKinematicBody& other );// copy constructor
    // GlKinematicBody(std::string urdfFilename); 
    GlKinematicBody(std::string &urdf_xml_string);
    //GlKinematicBody(boost::shared_ptr<urdf::Model> urdf_instance);
    GlKinematicBody(boost::shared_ptr<otdf::ModelInterface> otdf_instance);
    ~ GlKinematicBody();
    
    // state can be set via robot_state_t, or urdf::Model,  or affordance_state_t,  or otdf::ModelInterface;
    void set_state(const drc::robot_state_t &msg);
    void set_state(const KDL::Frame &T_world_body, const drc::joint_angles_t &msg); 
    void run_fk_and_update_urdf_link_shapes_and_tfs(std::map<std::string, double> &jointpos_in,const KDL::Frame &T_world_body);
    
    void re_init(boost::shared_ptr<otdf::ModelInterface> otdf_instance);
    void set_state(boost::shared_ptr<otdf::ModelInterface> _otdf_instance); 
    void run_fk_and_update_otdf_link_shapes_and_tfs(std::map<std::string, double> &jointpos_in,const KDL::Frame &T_world_body);
    // void set_state(const drc::affordance_t &msg);    
    // NOTE: PROBLEMS with set_state(const drc::affordance_t &msg)
    // Params could have changed structure 
    // requires otdf_instance to resolve link patterns 
    // reparsing xml file from drc::affortance_t to generate otdf_instance can be slow.
     
    
    void draw_link(boost::shared_ptr<urdf::Geometry> link, const LinkFrameStruct &nextTf);
    void draw_link(boost::shared_ptr<otdf::Geometry> link, const LinkFrameStruct &nextTf);
    
    void draw_body (float (&c)[3], float alpha)
    {
      //glColor3f(c[0],c[1],c[2]);
      glColor4f(c[0],c[1],c[2],alpha);
      for(uint i = 0; i < _link_tfs.size(); i++)
      {
        LinkFrameStruct nextTf = _link_tfs[i];
        if(is_otdf_instance)
        {
          boost::shared_ptr<otdf::Geometry> nextLink = _otdf_link_shapes[i];
          draw_link(nextLink, nextTf);
        }
        else
        {
          boost::shared_ptr<urdf::Geometry> nextLink = _link_shapes[i];
          draw_link(nextLink, nextTf);
         }
      }
      
    };
   
      
    void draw_body_in_frame (double (&c)[3], double alpha, const KDL::Frame &T_newWorldFrame_currentWorldFrame)
    {
      //glColor3f(c[0],c[1],c[2]);
      glColor4f(c[0],c[1],c[2],alpha);
      for(uint i = 0; i < _link_tfs.size(); i++)
      {
        KDL::Frame T_currentWorldFrame_link,T_newWorldFrame_link;
        LinkFrameStruct nextTf;
        
        nextTf = _link_tfs[i];
        T_currentWorldFrame_link = _link_tfs[i].frame;
        T_newWorldFrame_link = T_newWorldFrame_currentWorldFrame*T_currentWorldFrame_link;
        nextTf.frame = T_newWorldFrame_link;
        
        if(is_otdf_instance)
        {
          boost::shared_ptr<otdf::Geometry> nextLink = _otdf_link_shapes[i];
          draw_link(nextLink, nextTf);
        }
        else
        {
          boost::shared_ptr<urdf::Geometry> nextLink = _link_shapes[i];
          draw_link(nextLink, nextTf);
        }
      }
    };


    void show_bbox(bool value)
    {
       visualize_bbox = value; 
    };
    
    int get_num_joints()
    {
      int val = _joint_names_.size();
      return val;
    };

    std::vector<LinkFrameStruct> get_link_tfs()
    {
      return _link_tfs;
    };
    std::vector<std::string > get_link_names()
    {
      return _link_names;
    };
    
    std::map<std::string, boost::shared_ptr<urdf::Link> > get_links_map()
    {
      return _links_map;
    };

    bool get_link_frame(const std::string &link_name, KDL::Frame &T_world_link);
    bool get_link_geometry(const std::string &link_name, boost::shared_ptr<urdf::Geometry> &link_geom);
    bool get_link_geometry(const std::string &link_name, boost::shared_ptr<otdf::Geometry> &link_geom);
    bool get_mesh_struct(const std::string &link_name, MeshStruct &mesh_struct);
    
    // Was protected: (mfallon changed this:
    std::string evalMeshFilePath(std::string file_path_expression);
  protected:
     std::string exec(std::string cmd);
     std::string evalROSMeshFilePath(std::string file_path_expression);  
 };
 
} // end namespace 

#endif //GL_KINEMATIC_BODY_HPP
