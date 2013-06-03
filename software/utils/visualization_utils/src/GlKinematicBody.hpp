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
    LinkFrameStruct():frame(KDL::Frame::Identity()), future_frame(KDL::Frame::Identity()) { }
    std::string name;
    KDL::Frame frame;
    KDL::Frame future_frame;
}; 


struct  JointFrameStruct
{
    JointFrameStruct():frame(KDL::Frame::Identity()), future_frame(KDL::Frame::Identity()){ }
    std::string name;
    KDL::Frame frame;
    KDL::Frame future_frame;
    KDL::Vector axis; // in world frame
    KDL::Vector future_axis; // in future_world frame
    int type;
}; 
  
class GlKinematicBody
{
  protected: 
    std::string _urdf_xml_string; 
    std::string _root_name;
    std::string _unique_root_geometry_name;
    std::vector<std::string> _joint_names_;
    std::map<std::string, boost::shared_ptr<urdf::Link> > _links_map;
    std::map<std::string, boost::shared_ptr<otdf::Link> > _otdf_links_map;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> _fksolver;

    std::vector<std::string > _link_names; 
    std::vector<boost::shared_ptr<urdf::Geometry> > _link_shapes;
    std::vector<boost::shared_ptr<otdf::Geometry> > _otdf_link_shapes; //for affordance display we want to use otdf::Geometry
    std::vector<LinkFrameStruct> _link_tfs;    
    
    // adding support for multiple visual geometries for a given link 
    std::vector<std::string > _link_geometry_names; //_link name + id;
    std::vector<LinkFrameStruct> _link_geometry_tfs;
    
    
    std::vector<std::string > _joint_names; 
    std::vector<JointFrameStruct > _joint_tfs;
     


   // std::map<std::string, MeshStruct > _mesh_map; // associates link geometry name with meshstruct
  //Avoids loading the same mesh multiple times.
   // std::map<std::string, MeshStruct > _mesh_model_map; // associates file name with meshstruct

    std::map<std::string, boost::shared_ptr<MeshStruct> > _mesh_map; // associates link geometry name with meshstruct
  //Avoids loading the same mesh multiple times.
    std::map<std::string, boost::shared_ptr<MeshStruct> > _mesh_model_map; // associates file name with meshstruct
    
 
    bool visualize_bbox;
    bool enable_blinking; 
    bool initialized;
    bool is_otdf_instance;
    bool future_state_changing;
    bool future_display_active;  // set when set_future_state is called. Cleared when _T_world_body == _T_world_body_desired;
    bool accumulate_motion_trail;

   
   public:
    // Constructors and destructor
   // GlKinematicBody( const GlKinematicBody& other );// copy constructor
    // GlKinematicBody(std::string urdfFilename); 
    GlKinematicBody(std::string &urdf_xml_string);
    //GlKinematicBody(boost::shared_ptr<urdf::Model> urdf_instance);
    GlKinematicBody(boost::shared_ptr<otdf::ModelInterface> otdf_instance);
    ~ GlKinematicBody();
    
    //object state
    KDL::Frame _T_world_body; //store position in the world
    KDL::Frame _T_world_body_future; //store position in the world
    //Also store current jointangles map.
    std::map<std::string, double> _current_jointpos;
    std::map<std::string, double> _future_jointpos;
    std::map<std::string, double> _jointlimit_min;
    std::map<std::string, double> _jointlimit_max;
    std::vector<KDL::Frame> _desired_body_motion_history;
    
    // mesh for drawing
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3i> triangles;
    bool isShowMeshSelected;

    // state can be set via robot_state_t, or urdf::Model,  or affordance_state_t,  or otdf::ModelInterface;
    void set_state(const drc::robot_state_t &msg);
    void set_state(const KDL::Frame &T_world_body, const drc::joint_angles_t &msg); 
    void set_state(const KDL::Frame &T_world_body, std::map<std::string, double> &jointpos_in);
    void run_fk_and_update_urdf_link_shapes_and_tfs(std::map<std::string, double> &jointpos_in,const KDL::Frame &T_world_body, bool update_future_frame);

    // void set_state(const drc::affordance_t &msg);    
    // NOTE: PROBLEMS with set_state(const drc::affordance_t &msg)
    // Params could have changed structure 
    // requires otdf_instance to resolve link patterns 
    // reparsing xml file from drc::affortance_t to generate otdf_instance can be slow.
    void re_init(boost::shared_ptr<otdf::ModelInterface> otdf_instance);
    void set_state(boost::shared_ptr<otdf::ModelInterface> _otdf_instance); 
    void run_fk_and_update_otdf_link_shapes_and_tfs(std::map<std::string, double> &jointpos_in,const KDL::Frame &T_world_body, bool update_future_frame);
    
    void set_future_state(const drc::robot_state_t &msg);
    void set_future_state(const KDL::Frame &T_world_body, std::map<std::string, double> &jointpos_in);// ability to visualize in space and time, should work for both otdf and urdf.

    void draw_link(boost::shared_ptr<urdf::Geometry> link,const std::string &nextTfname, const KDL::Frame &nextTfframe);
    void draw_link(boost::shared_ptr<otdf::Geometry> link,const std::string &nextTfname, const KDL::Frame &nextTfframe);
    void draw_link_current_and_future(float (&c)[3], float alpha,int link_shape_index, const LinkFrameStruct &nextTf)
    {    
    
      if(is_otdf_instance)
      {
        boost::shared_ptr<otdf::Geometry> nextLink = _otdf_link_shapes[link_shape_index];
        draw_link(nextLink,nextTf.name, nextTf.frame);
        //displays desired end state.
        if(future_display_active) {
            glColor4f(0.5,0.5,0,0.35);
            draw_link(nextLink,nextTf.name,nextTf.future_frame);
            glColor4f(c[0],c[1],c[2],alpha);
        }
      }
      else
      {
        boost::shared_ptr<urdf::Geometry> nextLink = _link_shapes[link_shape_index];
        draw_link(nextLink,nextTf.name, nextTf.frame);
        if(future_display_active) {
           //displays desired end state.
            glColor4f(0.5,0.5,0,0.35);
            draw_link(nextLink,nextTf.name,nextTf.future_frame);
            glColor4f(c[0],c[1],c[2],alpha);
        }
      }    
    };
    
    void draw_body (float (&c)[3], float alpha)
    {

     if(alpha==0)
			return;

      //glColor3f(c[0],c[1],c[2]);
      glColor4f(c[0],c[1],c[2],alpha);
      double t;
      if(enable_blinking){
        int64_t now=bot_timestamp_now();
        t=bot_timestamp_useconds(now)*1e-6;//in sec
        alpha=std::min(fabs(sin(M_PI*t)),1.0);
        glColor4f(c[0],c[1],c[2],alpha);
      }
      
      
      for(uint i = 0; i < _link_geometry_tfs.size(); i++)
      {
        LinkFrameStruct nextTf = _link_geometry_tfs[i];   
        draw_link_current_and_future(c,alpha,i,nextTf);
      }
      
    };
   
      
    void draw_body_in_frame (float (&c)[3], double alpha, const KDL::Frame &T_drawFrame_currentWorldFrame)
    {

      //glColor3f(c[0],c[1],c[2]);
      glColor4f(c[0],c[1],c[2],alpha);
      for(uint i = 0; i < _link_geometry_tfs.size(); i++)
      {
        KDL::Frame T_currentWorldFrame_link,T_drawFrame_link;
        LinkFrameStruct nextTf;
        
        nextTf = _link_geometry_tfs[i];
        T_currentWorldFrame_link = _link_geometry_tfs[i].frame;
        T_drawFrame_link = T_drawFrame_currentWorldFrame*T_currentWorldFrame_link;
        nextTf.frame = T_drawFrame_link;
        draw_link_current_and_future(c,alpha,i,nextTf);
      }

      
    };
    
    void accumulate_and_draw_motion_trail(float (&c)[3], double alpha, const KDL::Frame &T_drawFrame_accumulationFrame,const KDL::Frame &T_accumulationFrame_currentWorldFrame)
    {
       if(accumulate_motion_trail)
        {
          KDL::Frame T_accumulationFrame_body = T_accumulationFrame_currentWorldFrame*_T_world_body;

          if(_desired_body_motion_history.size()>0)
          {  
            KDL::Frame prev_T_accumulationFrame_body = _desired_body_motion_history.back();
            KDL::Vector diff = prev_T_accumulationFrame_body.p -  T_accumulationFrame_body.p;
            double distance = sqrt( diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);

            if(distance > 0.01)//>1cm
            {
             _desired_body_motion_history.push_back(T_accumulationFrame_body);
            }
          }
          else
            _desired_body_motion_history.push_back(T_accumulationFrame_body);      
        }
        draw_motion_trail(c,alpha,T_drawFrame_accumulationFrame);
    };
    
    void draw_motion_trail(float (&c)[3], double alpha, const KDL::Frame &T_drawFrame_accumulationFrame)
    {
        glColor4f(c[0],c[1],c[2],alpha);
      
       if(_desired_body_motion_history.size()>0)  {     
         glLineWidth (3.0);
          glPushMatrix();
          glBegin(GL_LINE_STRIP);
          //glPointSize(5.0f);
          //glBegin(GL_POINTS);
          for(uint i = 0; i < _desired_body_motion_history.size(); i++)
          {
             KDL::Frame nextTfframe = T_drawFrame_accumulationFrame*_desired_body_motion_history[i];
	           glVertex3f(nextTfframe.p[0], nextTfframe.p[1], nextTfframe.p[2]);
          }
          glEnd();
          glPopMatrix();
       }
    };
    
    void log_motion_trail(bool value)
    {
      accumulate_motion_trail=value;
    };
    
    bool is_motion_trail_log_active()
    {
      return accumulate_motion_trail;
    };
    
    
    void clear_desired_body_motion_history()
    {
     accumulate_motion_trail=false;
      if(_desired_body_motion_history.size()>0) {
        std::cout << "called clear_desired_body_motion_history() " << std::endl;
        _T_world_body_future = _T_world_body;
        _future_jointpos.clear();
        _future_jointpos = _current_jointpos;
        _desired_body_motion_history.clear();
      }
    };

    void disable_future_display()
    {
      future_state_changing=false;
      future_display_active = false;
      clear_desired_body_motion_history();
    };
    
    bool is_future_display_active()
    {
       return future_display_active;
    };   
    
    bool is_future_state_changing()
    {
       return future_state_changing;
    };

    void set_future_state_changing(bool value)
    {
      future_state_changing=value;
      if(value) // init to current pos on set
      {
        _T_world_body_future = _T_world_body; 
        _future_jointpos =  _current_jointpos;
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
    
    std::map<std::string, boost::shared_ptr<otdf::Link> > get_otdf_links_map()
    {
      return _otdf_links_map;
    };
    
    std::vector<LinkFrameStruct> get_link_geometry_tfs()
    {
      return _link_geometry_tfs;
    };
    std::vector<std::string > get_link_geometry_names()
    {
      return _link_geometry_names;
    };

    bool get_link_frame(const std::string &link_name, KDL::Frame &T_world_link);
    bool get_link_future_frame(const std::string &link_name, KDL::Frame &T_world_link);// if not future display not active return false    
    bool get_link_geometry_frame(const std::string &link_geometry_name, KDL::Frame &T_world_link);
    bool get_link_geometry_future_frame(const std::string &link_geometry_name, KDL::Frame &T_world_link);// if not future display not active return fals
    bool get_link_geometry(const std::string &link_geometry_name, boost::shared_ptr<urdf::Geometry> &link_geom);
    bool get_link_geometry(const std::string &link_geometry_name, boost::shared_ptr<otdf::Geometry> &link_geom);
    bool get_mesh_struct(const std::string &link_geometry_name, MeshStruct &mesh_struct);
    bool get_joint_info(const std::string &joint_name, JointFrameStruct &jointinfo_struct);
    void get_whole_body_span_dims(Eigen::Vector3f &whole_body_span,Eigen::Vector3f &offset);
    
    bool get_associated_link_name(std::string &link_geometry_name,std::string &link_name)
    {
      std::vector<std::string>::const_iterator found;
      found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), link_geometry_name);
      if (found != _link_geometry_names.end()) { // if doesnt exist then add*/
        unsigned int index = found - _link_geometry_names.begin();
        link_name = _link_names[index];
        return true;
      }
      return false;
    };
    
    void draw_whole_body_bbox(); 
    void blink(bool value){
      enable_blinking=value;
    }
    // Was protected: (mfallon changed this:
    std::string evalMeshFilePath(std::string file_path_expression, bool return_convex_hull_path =false);
  protected:    
    std::string exec(std::string cmd);
    std::string evalROSMeshFilePath(std::string file_path_expression);  
 };
 
} // end namespace 

#endif //GL_KINEMATIC_BODY_HPP
