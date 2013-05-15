#ifndef INTERACTABLE_GL_KINEMATIC_BODY_HPP
#define INTERACTABLE_GL_KINEMATIC_BODY_HPP

#include "GlKinematicBody.hpp"
#include <collision/collision_detector.h>
#include <collision/collision_object_sphere.h>
#include <collision/collision_object_cylinder.h>
#include <collision/collision_object_box.h>
#include <collision/collision_object_torus.h>
namespace visualization_utils {

 
class InteractableGlKinematicBody: public GlKinematicBody 
{

  public:
    std::string _unique_name;
    std::map<std::string,   boost::shared_ptr<collision::Collision_Object> > _collision_object_map;
    
    // create a collision detector class
    // collision::Collision_Detector _collision_detector;
    boost::shared_ptr<collision::Collision_Detector> _collision_detector; 
    
    //separate collision detector for interactable markers
    std::map<std::string,   boost::shared_ptr<collision::Collision_Object> > _markers_collision_object_map;
    std::map<std::string,   boost::shared_ptr<collision::Collision_Object> > _dofmarkers_collision_object_map;
    boost::shared_ptr<collision::Collision_Detector> _collision_detector_floatingbase_markers;
    boost::shared_ptr<collision::Collision_Detector> _collision_detector_jointdof_markers;
   

  private:   
   std::string selected_link;
   bool link_selection_enabled;
   bool whole_body_selection_enabled; 
   
   std::string selected_marker;
   bool bodypose_adjustment_enabled;
   bool jointdof_adjustment_enabled;
   bool jointdof_markers_initialized;
   
   void init_vars(void);
  public:  
   enum _bodyadjust_type
   {
     THREE_D=0, TWO_D, TWO_HALF_D,THREE_D_TRANS, THREE_D_ROT, TWO_D_TRANS, TWO_D_ROT
   } bodypose_adjustment_type; 
     
  private:  
   Eigen::Vector3f _floatingbase_offset; // visual offset of the root link, if any 
   float _floatingbase_markers_boxsize;
   Eigen::Vector2f _floatingbase_markers_torusdims;
   Eigen::Vector3i _marker_dir_flip;
   
  public:
  //copy Constructors
  InteractableGlKinematicBody( const InteractableGlKinematicBody& other, std::string unique_name); 
  InteractableGlKinematicBody( const GlKinematicBody& other,
                              boost::shared_ptr<collision::Collision_Detector> col_detector, 
                              bool enable_selection,
                              std::string unique_name);
  // create                             
  InteractableGlKinematicBody( const GlKinematicBody& other,
                              bool enable_selection,
                              std::string unique_name);                            
 // Constructors (Reuses code from GLKinematicBody Constructor)
  InteractableGlKinematicBody(std::string urdf_xml_string,
                              boost::shared_ptr<collision::Collision_Detector> col_detector, 
                              bool enable_selection, std::string unique_name);
  InteractableGlKinematicBody(std::string urdf_xml_string,
                              bool enable_selection, std::string unique_name);                            
  InteractableGlKinematicBody (boost::shared_ptr<otdf::ModelInterface> otdf_instance,
                              boost::shared_ptr<collision::Collision_Detector> col_detector,
                              bool enable_selection, std::string unique_name);
  ~InteractableGlKinematicBody();
   void init_urdf_collision_objects();
   void init_otdf_collision_objects();
   void set_state(const drc::robot_state_t &msg); 
   void set_state(const KDL::Frame &T_world_body, const drc::joint_angles_t &msg); 
   void set_state(const KDL::Frame &T_world_body, std::map<std::string, double> &jointpos_in);
   void set_state(boost::shared_ptr<otdf::ModelInterface> _otdf_instance);
   void update_urdf_collision_objects(void);
   void update_otdf_collision_objects(void);
   
   // overloaded from GLKinematicBody They call update functions for marker collision objects
   void set_future_state(const KDL::Frame &T_world_body, std::map<std::string, double> &jointpos_in);

   // double c[3] = {0.3,0.3,0.3};
   // double alpha = self->alpha;


   bool draw_mesh(int linkType);

   void draw_body (float (&c)[3], float alpha);   
   
   void draw_body_in_frame (float (&c)[3], float alpha,const KDL::Frame &T_drawFrame_currentWorldFrame)
   {
     
      glColor4f(c[0],c[1],c[2],alpha);
      for(uint i = 0; i < _link_geometry_tfs.size(); i++)
      {
        KDL::Frame T_currentWorldFrame_link,T_drawFrame_link;
        LinkFrameStruct nextTf=_link_geometry_tfs[i];
        
        T_currentWorldFrame_link = nextTf.frame;
        T_drawFrame_link = T_drawFrame_currentWorldFrame*T_currentWorldFrame_link;
        nextTf.frame = T_drawFrame_link;

        
        std::stringstream oss;
        oss << _unique_name << "_"<< nextTf.name; 
        if((link_selection_enabled)&&(selected_link == oss.str())) {
//          if((bodypose_adjustment_enabled)&&(is_otdf_instance))
//            draw_interactable_markers(_otdf_link_shapes[i],nextTf); 
//          else if((bodypose_adjustment_enabled)&&(!is_otdf_instance)) 
//            draw_interactable_markers(_link_shapes[i],nextTf);   
            
          glColor4f(0.7,0.1,0.1,alpha);         
        }
        else
           glColor4f(c[0],c[1],c[2],alpha);

        if((whole_body_selection_enabled)&&(selected_link == _unique_name)) {
          glColor4f(0.7,0.1,0.1,alpha); // whole body is selected instead of an individual link
        }
        
        GlKinematicBody::draw_link_current_and_future(c,alpha,i,nextTf);    
      
      }

   };



  
  // Interactive markers
   void init_floatingbase_marker_collision_objects(); // requires  as FK needs to be solved atleast once to derive. Called inside setstate.
   void update_floatingbase_marker_collision_objects();
   void draw_floatingbase_markers();
   
   void init_jointdof_marker_collision_objects(); // requires  as FK needs to be solved atleast once to derive. Called inside setstate.
   void update_jointdof_marker_collision_objects();
   void draw_jointdof_markers();

   void draw_interactable_markers(boost::shared_ptr<otdf::Geometry> &_link_shape, const LinkFrameStruct &link_tf);
   void draw_interactable_markers(boost::shared_ptr<urdf::Geometry> &_link_shape,const LinkFrameStruct &link_tf);
   void draw_markers(float (&pos)[3], float markersize, float inner_radius, float outer_radius); 

   
   void enable_link_selection(bool value)   {
       link_selection_enabled = value; 
   };   
   void highlight_link(std::string &link_name)   {
       selected_link = link_name; 
   };  
   
   void highlight_body(std::string &body_name)   {
       selected_link = body_name; 
   };   
   
    
   void highlight_marker(std::string &marker_name)   {
       selected_marker = marker_name; 
   };   
   void enable_bodypose_adjustment(bool value)   { 
    bodypose_adjustment_enabled = value;
    if(value)
    {
      jointdof_adjustment_enabled = false; // mutually exclusive
      if(_root_name=="world"){
        std::cerr << "ERROR: root link pose cannot be adjusted as it is fixed to the world. Enabling jointdof adjusment instead" << std::endl;
        enable_jointdof_adjustment(true);   
       }
    }
   };
   void enable_jointdof_adjustment(bool value)   { 
    jointdof_adjustment_enabled = value;
    if(value){
      bodypose_adjustment_enabled = false; // mutually exclusive
//      if(_collision_detector_jointdof_markers==NULL)
//        _collision_detector_jointdof_markers = boost::shared_ptr<collision::Collision_Detector>(new collision::Collision_Detector());
    }
   };
   
  void set_bodypose_adjustment_type(int type){
    bodypose_adjustment_type= (_bodyadjust_type)type;
    if(_collision_detector_floatingbase_markers) //  without this it SEGFAULTS If set_bodypose_adjustment_type is called before set state
    _collision_detector_floatingbase_markers->clear_collision_objects(); // reset markers
    _markers_collision_object_map.clear();
    init_floatingbase_marker_collision_objects(); 
    update_floatingbase_marker_collision_objects();  
   }
   
   bool is_bodypose_adjustment_enabled()   { 
    return bodypose_adjustment_enabled;
   };
   bool is_jointdof_adjustment_enabled()   { 
    return jointdof_adjustment_enabled;
   };
   void enable_whole_body_selection(bool value)   { 
    whole_body_selection_enabled = value;
   };
   
   void flip_trans_marker_xdir(bool value)   { 

     if(value)
      _marker_dir_flip[0] = -1;
    else
     _marker_dir_flip[0] = 1;
   };
   void flip_trans_marker_ydir(bool value)   { 
     if(value)
      _marker_dir_flip[1] = -1;
    else
     _marker_dir_flip[1] = 1;
   };
   void flip_trans_marker_zdir(bool value)   { 
     if(value)
      _marker_dir_flip[2] = -1;
    else
     _marker_dir_flip[2] = 1;
   };
};

} // end namespace 




#endif //INTERACTABLE_GL_KINEMATIC_BODY_HPP
