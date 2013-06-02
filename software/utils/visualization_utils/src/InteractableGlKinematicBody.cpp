#include "InteractableGlKinematicBody.hpp"
#include <algorithm> // using std::find

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;

// Variable initialization
void InteractableGlKinematicBody::init_vars(){
  bodypose_adjustment_enabled =false;
  jointdof_adjustment_enabled = false;
  jointdof_markers_initialized = false;  
  whole_body_selection_enabled = false;
  _jointdof_marker_filter_on = false;
  selected_link = " ";
  _floatingbase_markers_boxsize = 0;
  bodypose_adjustment_type = InteractableGlKinematicBody::THREE_D;
  _marker_dir_flip << 1,1,1;

}


// Copy constructors
InteractableGlKinematicBody::InteractableGlKinematicBody( const InteractableGlKinematicBody& other, string unique_name): 
GlKinematicBody(other),
link_selection_enabled(other.link_selection_enabled),
_unique_name(unique_name)
{ 
  _collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());
  init_vars();
  if(is_otdf_instance)
    init_otdf_collision_objects();
  else
    init_urdf_collision_objects();
    
      
}

InteractableGlKinematicBody::InteractableGlKinematicBody( const GlKinematicBody& other, shared_ptr<Collision_Detector> col_detector, bool enable_selection, string unique_name):
GlKinematicBody(other),
link_selection_enabled(enable_selection),
_collision_detector(col_detector),
_unique_name(unique_name)
{ 
  init_vars();
  if(is_otdf_instance)
    init_otdf_collision_objects();
  else
    init_urdf_collision_objects();

}

InteractableGlKinematicBody::InteractableGlKinematicBody( const GlKinematicBody& other, bool enable_selection, string unique_name):
GlKinematicBody(other),
link_selection_enabled(enable_selection),
_unique_name(unique_name)
{ 
  _collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());
  init_vars();
  if(is_otdf_instance)
    init_otdf_collision_objects();
  else
    init_urdf_collision_objects();


}


InteractableGlKinematicBody::InteractableGlKinematicBody(string urdf_xml_string,
  bool enable_selection, string unique_name):
  GlKinematicBody(urdf_xml_string), 
  link_selection_enabled(enable_selection) ,
  _unique_name(unique_name)
{  

  _collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());
  init_vars();
  init_urdf_collision_objects();
}



// constructor for plain urdf
InteractableGlKinematicBody::InteractableGlKinematicBody(string urdf_xml_string,
  shared_ptr<Collision_Detector> col_detector,
  bool enable_selection, string unique_name):
  GlKinematicBody(urdf_xml_string), 
  link_selection_enabled(enable_selection) ,
  _collision_detector(col_detector),
  _unique_name(unique_name)
{  
  init_vars();
  init_urdf_collision_objects();
}

void InteractableGlKinematicBody::init_urdf_collision_objects()
{

  // loop through all the links and add them to collision world.
  typedef map<string, shared_ptr<urdf::Link> > links_mapType;
  for(links_mapType::const_iterator it =  _links_map.begin(); it!= _links_map.end(); it++)
  { 
    if(it->second->visual)
    {
      //cout << it->first << endl;

      typedef map<string, shared_ptr<vector<shared_ptr<urdf::Visual> > > >  visual_groups_mapType;
      visual_groups_mapType::iterator v_grp_it = it->second->visual_groups.find("default");
      for (size_t iv = 0;iv < v_grp_it->second->size();iv++)
      {
         std::stringstream oss,oss2;
         oss << _unique_name << "_"<< it->first << "_" << iv; 
         oss2 << it->first << "_"<< iv; 
         string unique_geometry_name = oss2.str();
      
         //int type = it->second->visual->geometry->type;
          vector<shared_ptr<urdf::Visual> > visuals = (*v_grp_it->second);
          int type = visuals[iv]->geometry->type;

          enum {SPHERE, BOX, CYLINDER, MESH}; 

          if  (type == SPHERE)
          {
            shared_ptr<urdf::Sphere> sphere(shared_dynamic_cast<urdf::Sphere>(visuals[iv]->geometry));	
            double radius = sphere->radius;
     
            shared_ptr<Collision_Object> object_ptr(new Collision_Object_Sphere(oss.str(), radius, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
            _collision_object_map.insert(make_pair(oss.str(), object_ptr));
             
            // add a collision object to the collision detector class
            _collision_detector->add_collision_object(&*_collision_object_map.find(oss.str())->second);
          }
          else if  (type == BOX)
          {
            shared_ptr<urdf::Box> box(shared_dynamic_cast<urdf::Box>(visuals[iv]->geometry));
            Eigen::Vector3f dims;
            dims<< box->dim.x,box->dim.y,box->dim.z;
            shared_ptr<Collision_Object> object_ptr(new Collision_Object_Box(oss.str(), dims, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
            _collision_object_map.insert(make_pair(oss.str(), object_ptr));
            // add a collision object to the collision detector class
            _collision_detector->add_collision_object(&*_collision_object_map.find(oss.str())->second);
          }
          else if  (type == CYLINDER)
          {
            shared_ptr<urdf::Cylinder> cyl(shared_dynamic_cast<urdf::Cylinder>(visuals[iv]->geometry));
            double radius = cyl->radius;
            double length = cyl->length;
    //shared_ptr<Collision_Object> object_ptr(new Collision_Object_Cylinder(oss.str(),radius,length,Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
            Eigen::Vector3f dims;
            dims<< 2*radius,2*radius,length;
            shared_ptr<Collision_Object> object_ptr(new Collision_Object_Box(oss.str(), dims, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) )); 
            _collision_object_map.insert(make_pair(oss.str(), object_ptr));  
            // add a collision object to the collision detector class
            _collision_detector->add_collision_object(&*_collision_object_map.find(oss.str())->second);
       
          }
          else if  (type == MESH)
          //if  (type == MESH)
          {
            shared_ptr<urdf::Mesh> mesh(shared_dynamic_cast<urdf::Mesh>(visuals[iv]->geometry));

            //typedef std::map<std::string, MeshStruct > mesh_map_type_;
	    typedef std::map<std::string, shared_ptr<MeshStruct> > mesh_map_type_;
            mesh_map_type_::iterator mesh_map_it = _mesh_map.find(unique_geometry_name);
     
            Eigen::Vector3f dims;
            //dims<<  mesh_map_it->second.span_x,mesh_map_it->second.span_y,mesh_map_it->second.span_z;
            dims<<  mesh_map_it->second->span_x,mesh_map_it->second->span_y,mesh_map_it->second->span_z;

            shared_ptr<Collision_Object> object_ptr(new Collision_Object_Box(oss.str(), dims, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
            _collision_object_map.insert(make_pair(oss.str(), object_ptr));
             
            // add a collision object to the collision detector class
            _collision_detector->add_collision_object(&*_collision_object_map.find(oss.str())->second);
            //std::cout  << "num_colls: " << _collision_detector->num_collisions() <<  std::endl;

          }//end  if  (type == MESH)
      
      }// end for visuals in default visual group
    } // end if(it->second->visual)
  } // end for
  
}



//======================================================================================================
// constructor for otdf_instance
InteractableGlKinematicBody::InteractableGlKinematicBody (shared_ptr<otdf::ModelInterface> otdf_instance,  
  shared_ptr<Collision_Detector> col_detector, 
  bool enable_selection, string unique_name):  
  GlKinematicBody(otdf_instance), 
  link_selection_enabled(enable_selection),
 _collision_detector(col_detector),
  _unique_name(unique_name)
{  
  init_vars(); 
  init_otdf_collision_objects();
  
}

void InteractableGlKinematicBody::init_otdf_collision_objects()
{
 // loop through all the links and add them to collision world.
  typedef map<string, shared_ptr<otdf::Link> > links_mapType;
  for(links_mapType::const_iterator it =  _otdf_links_map.begin(); it!= _otdf_links_map.end(); it++)
  { 
    if(it->second->visual)
    {
      //cout << it->first << endl;
      
      typedef map<string, shared_ptr<vector<shared_ptr<otdf::Visual> > > >  visual_groups_mapType;
      visual_groups_mapType::iterator v_grp_it = it->second->visual_groups.find("default");
      for (size_t iv = 0;iv < v_grp_it->second->size();iv++)
      {
         std::stringstream oss,oss2;
         oss << _unique_name << "_"<< it->first << "_" << iv; 
         oss2 << it->first << "_"<< iv; 
         string unique_geometry_name = oss2.str(); 
      
         //int type = it->second->visual->geometry->type;
          vector<shared_ptr<otdf::Visual> > visuals = (*v_grp_it->second);
          int type = visuals[iv]->geometry->type;

          enum {SPHERE, BOX, CYLINDER, MESH, TORUS}; 
          //TODO: create collision objects for cylinder,sphere and torus.
          if  (type == SPHERE)
          {
            shared_ptr<otdf::Sphere> sphere(shared_dynamic_cast<otdf::Sphere>(visuals[iv]->geometry));	
            double radius = sphere->radius;
            shared_ptr<Collision_Object> object_ptr(new Collision_Object_Sphere(oss.str(), radius, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
            _collision_object_map.insert(make_pair(oss.str(), object_ptr));         
            // add a collision object to the collision detector class
            _collision_detector->add_collision_object(&*_collision_object_map.find(oss.str())->second);
          }
          else if  (type == BOX)
          {
            shared_ptr<otdf::Box> box(shared_dynamic_cast<otdf::Box>(visuals[iv]->geometry));
            Eigen::Vector3f dims;
            dims<< box->dim.x,box->dim.y,box->dim.z;
            shared_ptr<Collision_Object> object_ptr(new Collision_Object_Box(oss.str(), dims, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
            _collision_object_map.insert(make_pair(oss.str(), object_ptr));
            // add a collision object to the collision detector class
            _collision_detector->add_collision_object(&*_collision_object_map.find(oss.str())->second);
          }
          else if  (type == CYLINDER)
          {
            shared_ptr<otdf::Cylinder> cyl(shared_dynamic_cast<otdf::Cylinder>(visuals[iv]->geometry));
            double radius = cyl->radius;
            double length = cyl->length;
            //shared_ptr<Collision_Object> object_ptr(new Collision_Object_Cylinder(oss.str(),radius,length,Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
            
            Eigen::Vector3f dims;
            dims<< 2*radius,2*radius,length;
            shared_ptr<Collision_Object> object_ptr(new Collision_Object_Box(oss.str(), dims, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));        
            
            
            _collision_object_map.insert(make_pair(oss.str(), object_ptr));
            // add a collision object to the collision detector class
            _collision_detector->add_collision_object(&*_collision_object_map.find(oss.str())->second);
          }
          else if  (type == TORUS)
          {
            shared_ptr<otdf::Torus> tor(shared_dynamic_cast<otdf::Torus>(visuals[iv]->geometry));
            double radius = tor->radius;
            double length = tor->tube_radius;       
          //shared_ptr<Collision_Object> object_ptr(new Collision_Object_Cylinder(oss.str(),radius,length,Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
            Eigen::Vector3f dims;
            dims<< 2*radius,2*radius,length;
            shared_ptr<Collision_Object> object_ptr(new Collision_Object_Box(oss.str(), dims, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) )); 
            _collision_object_map.insert(make_pair(oss.str(), object_ptr));
            // add a collision object to the collision detector class
            _collision_detector->add_collision_object(&*_collision_object_map.find(oss.str())->second);
          }
          else if  (type == MESH)
          //if  (type == MESH)
          {
            shared_ptr<otdf::Mesh> mesh(shared_dynamic_cast<otdf::Mesh>(visuals[iv]->geometry));

            //typedef std::map<std::string, MeshStruct > mesh_map_type_;
  	    typedef std::map<std::string, shared_ptr<MeshStruct> > mesh_map_type_;
            mesh_map_type_::iterator mesh_map_it = _mesh_map.find(unique_geometry_name);
     
            Eigen::Vector3f dims;
            //dims<<  mesh_map_it->second.span_x,mesh_map_it->second.span_y,mesh_map_it->second.span_z;
            dims<<  mesh_map_it->second->span_x,mesh_map_it->second->span_y,mesh_map_it->second->span_z;

            shared_ptr<Collision_Object> object_ptr(new Collision_Object_Box(oss.str(), dims, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
            _collision_object_map.insert(make_pair(oss.str(), object_ptr));         
            // add a collision object to the collision detector class
            _collision_detector->add_collision_object(&*_collision_object_map.find(oss.str())->second);
            //std::cout  << "num_colls: " << _collision_detector->num_collisions() <<  std::endl;

          }//end  if  (type == MESH)
      }// end for visuals in default visual group
    } // end if(it->second->visual)
  } // end for

}
//======================================================================================================
InteractableGlKinematicBody::~InteractableGlKinematicBody()
{

  // clear the collision world
  if(_floatingbase_markers_boxsize!=0)
    _collision_detector_floatingbase_markers->clear_collision_objects();
  if(jointdof_markers_initialized)
    _collision_detector_jointdof_markers->clear_collision_objects();
  //_collision_detector->clear_collision_objects(); // The parent program should handle this.
}

//======================================================================================================
// update methods for plain urdf

void InteractableGlKinematicBody::set_state(const drc::robot_state_t &msg)
{
   GlKinematicBody::set_state(msg);  //code re-use
   update_urdf_collision_objects();
   if(_root_name!="world"){
      if(_floatingbase_markers_boxsize==0)
        init_floatingbase_marker_collision_objects(); //  For the first time, create the marker collision objects.
      update_floatingbase_marker_collision_objects();  
   }
   
    if(!jointdof_markers_initialized){
      init_jointdof_marker_collision_objects(); // For the first time, create the marker collision objects.
      jointdof_markers_initialized = true; 
    }
    update_jointdof_marker_collision_objects(); 
   
} // end InteractableGlKinematicBody::set_state(const drc::robot_state_t)


void InteractableGlKinematicBody::set_state(const KDL::Frame &T_world_body, const drc::joint_angles_t &msg)
{
    GlKinematicBody::set_state(T_world_body,msg);  //code re-use
    update_urdf_collision_objects();
    if(_root_name!="world"){
      if(_floatingbase_markers_boxsize==0)
        init_floatingbase_marker_collision_objects(); //  For the first time, create the marker collision objects.
      update_floatingbase_marker_collision_objects();  
   }
   
    if(!jointdof_markers_initialized){
      init_jointdof_marker_collision_objects(); // For the first time, create the marker collision objects.
      jointdof_markers_initialized = true; 
    }
    update_jointdof_marker_collision_objects();
    
} // end InteractableGlKinematicBody::set_state(const KDL::Frame, const drc::joint_angles_t)

void InteractableGlKinematicBody::set_state(const KDL::Frame &T_world_body, std::map<std::string, double> &jointpos_in)
{
    GlKinematicBody::set_state(T_world_body,jointpos_in);  //code re-use
    update_urdf_collision_objects();
    if(_root_name!="world"){
      if(_floatingbase_markers_boxsize==0)
        init_floatingbase_marker_collision_objects(); //  For the first time, create the marker collision objects.
        update_floatingbase_marker_collision_objects();  
   }
   
    if(!jointdof_markers_initialized){
      init_jointdof_marker_collision_objects(); // For the first time, create the marker collision objects.
      jointdof_markers_initialized = true; 
    }
    update_jointdof_marker_collision_objects();

}// end InteractableGlKinematicBody::set_state(const KDL::Frame &T_world_body, std::map<std::string, double> &jointpos_in)

void InteractableGlKinematicBody::update_urdf_collision_objects(void)
{
 
    typedef map<string, shared_ptr<urdf::Link> > links_mapType;
    for( links_mapType::const_iterator it =  _links_map.begin(); it!= _links_map.end(); it++)
    { 
      if(it->second->visual)
      {
      
        typedef map<string, shared_ptr<vector<shared_ptr<urdf::Visual> > > >  visual_groups_mapType;
        visual_groups_mapType::iterator v_grp_it = it->second->visual_groups.find("default");
        for (size_t iv = 0;iv < v_grp_it->second->size();iv++)
        {
            
            vector<shared_ptr<urdf::Visual> > visuals = (*v_grp_it->second);
             
            std::stringstream oss,oss2;
            oss << _unique_name << "_"<< it->first << "_" << iv; 
            oss2 << it->first << "_"<< iv; 
            string unique_geometry_name = oss2.str(); 
             
            KDL::Frame  T_world_visual;
            LinkFrameStruct state;	    

           // retrieve T_world_visual from store
           // #include <algorithm>
            std::vector<std::string>::const_iterator found;
            found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), unique_geometry_name);
            if (found != _link_geometry_names.end()) {
              unsigned int index = found - _link_geometry_names.begin();
              state=_link_geometry_tfs[index];  
              T_world_visual = state.frame;
            } 
            else 
            {  
             T_world_visual = KDL::Frame::Identity();
             std::cerr << "ERROR:"<< unique_geometry_name << " not found in _link_geometry_names" << std::endl;
            }


            int type = visuals[iv]->geometry->type;
            enum {SPHERE, BOX, CYLINDER, MESH}; 
            
            Eigen::Vector3f p;
            p[0] = state.frame.p[0];
            p[1] = state.frame.p[1];
            p[2] = state.frame.p[2];
              double x,y,z,w;
            state.frame.M.GetQuaternion(x,y,z,w);
            Eigen::Vector4f q;
            q << x, y, z, w;
            if  (type == SPHERE)
            {
              shared_ptr<Collision_Object_Sphere> downcasted_object(shared_dynamic_cast<Collision_Object_Sphere>(_collision_object_map.find(oss.str())->second));
              downcasted_object->set_transform(p,q);
            }
            else if  (type == BOX)
            {
              shared_ptr<Collision_Object_Box> downcasted_object(shared_dynamic_cast<Collision_Object_Box>(_collision_object_map.find(oss.str())->second));
              downcasted_object->set_transform(p,q);
            }
            else if  (type == CYLINDER)
            {
              //shared_ptr<Collision_Object_Cylinder> downcasted_object(shared_dynamic_cast<Collision_Object_Cylinder>(_collision_object_map.find(oss.str())->second));
              shared_ptr<Collision_Object_Box> downcasted_object(shared_dynamic_cast<Collision_Object_Box>(_collision_object_map.find(oss.str())->second));
              downcasted_object->set_transform(p,q);

            }
            else if  (type == MESH)
            {
              //MeshStruct mesh_struct = _mesh_map.find(unique_geometry_name)->second;
	      shared_ptr<MeshStruct> mesh_struct_ptr = _mesh_map.find(unique_geometry_name)->second;
              KDL::Frame T_visual_objorigin, T_world_objorigin; 

              T_visual_objorigin = KDL::Frame::Identity();
              /*T_visual_objorigin.p[0]= mesh_struct.offset_x;
              T_visual_objorigin.p[1]= mesh_struct.offset_y;
              T_visual_objorigin.p[2]= mesh_struct.offset_z;*/
	      T_visual_objorigin.p[0]= mesh_struct_ptr->offset_x;
              T_visual_objorigin.p[1]= mesh_struct_ptr->offset_y;
              T_visual_objorigin.p[2]= mesh_struct_ptr->offset_z;

              /*In drc mesh files visual origin is not geometric origin. It is implicit in the vertex units. If you want to draw like a bounding box for collision detection that assumes vertices relative to the geometric origin, you have to take into account this shift in the frame used to define the vertices with that of the geometric origin of the object.*/
              T_world_objorigin = T_world_visual*T_visual_objorigin;

              state.frame = T_world_objorigin; 

              Eigen::Vector3f p;
              p[0] = state.frame.p[0];
              p[1] = state.frame.p[1];
              p[2] = state.frame.p[2];
              double x,y,z,w;
              state.frame.M.GetQuaternion(x,y,z,w);
              Eigen::Vector4f q;
              q << x, y, z, w;

              shared_ptr<Collision_Object_Box> downcasted_object(shared_dynamic_cast<Collision_Object_Box>(_collision_object_map.find(oss.str())->second));
              downcasted_object->set_transform(p,q); 
              
            } // end if MESH
             
        }// end for geoemtries in visual group  
        
      }//if(it->second->visual)

    }//end for
}

//==================================================================================================== 	
// Update methods for otdf objects
void InteractableGlKinematicBody::set_state(boost::shared_ptr<otdf::ModelInterface> otdf_instance)
{
    GlKinematicBody::set_state(otdf_instance);  //code re-use
    _collision_detector->clear_collision_objects();
    _collision_object_map.clear();

    init_otdf_collision_objects(); // reset collision objects with updated geometries
    update_otdf_collision_objects();
    if(_root_name!="world"){
      if(_floatingbase_markers_boxsize==0)
        init_floatingbase_marker_collision_objects(); //  For the first time, create the marker collision objects.
      update_floatingbase_marker_collision_objects();  
    }
    
    if(jointdof_markers_initialized)
      _collision_detector_jointdof_markers->clear_collision_objects(); 
    _dofmarkers_collision_object_map.clear();
    init_jointdof_marker_collision_objects(); // For the first time, create the marker collision objects.
    jointdof_markers_initialized = true; 
    update_jointdof_marker_collision_objects();/**/          
    
} // end InteractableGlKinematicBody::set_state(const drc::robot_state_t &msg)

void InteractableGlKinematicBody::update_otdf_collision_objects(void)
{
 
    typedef map<string, shared_ptr<otdf::Link> > links_mapType;
    for( links_mapType::const_iterator it =  _otdf_links_map.begin(); it!= _otdf_links_map.end(); it++)
    {  		

      if(it->second->visual)
      {
      
        typedef map<string, shared_ptr<vector<shared_ptr<otdf::Visual> > > >  visual_groups_mapType;
        visual_groups_mapType::iterator v_grp_it = it->second->visual_groups.find("default");
        for (size_t iv = 0;iv < v_grp_it->second->size();iv++)
        {
            
            vector<shared_ptr<otdf::Visual> > visuals = (*v_grp_it->second);
             
            std::stringstream oss,oss2;
            oss << _unique_name << "_"<< it->first << "_" << iv; 
            oss2 << it->first << "_"<< iv; 
            string unique_geometry_name = oss2.str(); 
             
            KDL::Frame  T_world_visual;
            LinkFrameStruct state;	    

           // retrieve T_world_visual from store
           // #include <algorithm>
            std::vector<std::string>::const_iterator found;
            found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), unique_geometry_name);
            if (found != _link_geometry_names.end()) {
              unsigned int index = found - _link_geometry_names.begin();
              state=_link_geometry_tfs[index];  
              T_world_visual = state.frame;
            } 
            else 
            {  
             T_world_visual = KDL::Frame::Identity();
             std::cerr << "ERROR:"<< unique_geometry_name << " not found in _link_geometry_names" << std::endl;
            }


            int type = visuals[iv]->geometry->type;
            enum {SPHERE, BOX, CYLINDER, MESH, TORUS}; 
            
            Eigen::Vector3f p;
            p[0] = state.frame.p[0];
            p[1] = state.frame.p[1];
            p[2] = state.frame.p[2];
              double x,y,z,w;
            state.frame.M.GetQuaternion(x,y,z,w);
            Eigen::Vector4f q;
            q << x, y, z, w;
            if  (type == SPHERE)
            {
              shared_ptr<Collision_Object_Sphere> downcasted_object(shared_dynamic_cast<Collision_Object_Sphere>(_collision_object_map.find(oss.str())->second));
              downcasted_object->set_transform(p,q);
            }
            else if  (type == BOX)
            {
              shared_ptr<Collision_Object_Box> downcasted_object(shared_dynamic_cast<Collision_Object_Box>(_collision_object_map.find(oss.str())->second));
              downcasted_object->set_transform(p,q);
            }
            else if  (type == CYLINDER)
            {
              //shared_ptr<Collision_Object_Cylinder> downcasted_object(shared_dynamic_cast<Collision_Object_Cylinder>(_collision_object_map.find(oss.str())->second));
              shared_ptr<Collision_Object_Box> downcasted_object(shared_dynamic_cast<Collision_Object_Box>(_collision_object_map.find(oss.str())->second));
              downcasted_object->set_transform(p,q);

            }
            else if  (type == TORUS)
            {
              //shared_ptr<Collision_Object_Cylinder> downcasted_object(shared_dynamic_cast<Collision_Object_Cylinder>(_collision_object_map.find(oss.str())->second));
              shared_ptr<Collision_Object_Box> downcasted_object(shared_dynamic_cast<Collision_Object_Box>(_collision_object_map.find(oss.str())->second));
              downcasted_object->set_transform(p,q);
            }
            else if  (type == MESH)
            {
              //MeshStruct mesh_struct = _mesh_map.find(unique_geometry_name)->second;
	      shared_ptr<MeshStruct> mesh_struct_ptr = _mesh_map.find(unique_geometry_name)->second;

              KDL::Frame T_visual_objorigin, T_world_objorigin; 

              T_visual_objorigin = KDL::Frame::Identity();
              /*T_visual_objorigin.p[0]= mesh_struct.offset_x;
              T_visual_objorigin.p[1]= mesh_struct.offset_y;
              T_visual_objorigin.p[2]= mesh_struct.offset_z;*/
	      T_visual_objorigin.p[0]= mesh_struct_ptr->offset_x;
              T_visual_objorigin.p[1]= mesh_struct_ptr->offset_y;
              T_visual_objorigin.p[2]= mesh_struct_ptr->offset_z;

              /*In drc mesh files visual origin is not geometric origin. It is implicit in the vertex units. If you want to draw like a bounding box for collision detection that assumes vertices relative to the geometric origin, you have to take into account this shift in the frame used to define the vertices with that of the geometric origin of the object.*/
              T_world_objorigin = T_world_visual*T_visual_objorigin;

              state.frame = T_world_objorigin; 

              Eigen::Vector3f p;
              p[0] = state.frame.p[0];
              p[1] = state.frame.p[1];
              p[2] = state.frame.p[2];
              double x,y,z,w;
              state.frame.M.GetQuaternion(x,y,z,w);
              Eigen::Vector4f q;
              q << x, y, z, w;

              shared_ptr<Collision_Object_Box> downcasted_object(shared_dynamic_cast<Collision_Object_Box>(_collision_object_map.find(oss.str())->second));
              downcasted_object->set_transform(p,q); 
              
            } // end if MESH
             
        }// end for geoemtries in visual group  
        
      }//if(it->second->visual)

    }//end for
}

void InteractableGlKinematicBody::set_future_state(const drc::robot_state_t &msg)
{

  GlKinematicBody::set_future_state(msg);
  
   if(_root_name!="world"){
      if(_floatingbase_markers_boxsize==0)
        init_floatingbase_marker_collision_objects(); //  For the first time, create the marker collision objects.
      update_floatingbase_marker_collision_objects();  
   }

    if(!jointdof_markers_initialized){
      init_jointdof_marker_collision_objects(); // For the first time, create the marker collision objects.
      jointdof_markers_initialized = true; 
    }

    update_jointdof_marker_collision_objects();

}//end void GlKinematicBody::set_future_state

void InteractableGlKinematicBody::set_future_state(const KDL::Frame &T_world_body_future, std::map<std::string, double> &jointpos_in)
{

  GlKinematicBody::set_future_state(T_world_body_future,jointpos_in);
  
   if(_root_name!="world"){
      if(_floatingbase_markers_boxsize==0)
        init_floatingbase_marker_collision_objects(); //  For the first time, create the marker collision objects.
      update_floatingbase_marker_collision_objects();  
   }

    if(!jointdof_markers_initialized){
      init_jointdof_marker_collision_objects(); // For the first time, create the marker collision objects.
      jointdof_markers_initialized = true; 
    }

    update_jointdof_marker_collision_objects();

}//end void GlKinematicBody::set_future_state



//==================================================================================================== 	  
// utils for interactive markers


//----------------------------------------------------------------------------------------------------------------
// Floating Base Markers
//----------------------------------------------------------------------------------------------------------------
void InteractableGlKinematicBody::init_floatingbase_marker_collision_objects()
{
   //_collision_detector_floatingbase_markers.reset();
  _collision_detector_floatingbase_markers = shared_ptr<Collision_Detector>(new Collision_Detector()); 

  Eigen::Vector3f whole_body_span_dims,offset;
  GlKinematicBody::get_whole_body_span_dims(whole_body_span_dims,offset); //bounding box for entire body  

  //float dim[3] = {0.5*whole_body_span_dims[0]-offset[0], 0.5*whole_body_span_dims[1]-offset[1], 0.5*whole_body_span_dims[2]-offset[2]};
  float dim[3] = {0.5*whole_body_span_dims[0], 0.5*whole_body_span_dims[1], 0.5*whole_body_span_dims[2]};
  float maxspan= max(whole_body_span_dims[2],max(whole_body_span_dims[0],whole_body_span_dims[1]));
  float markersize;

  if(maxspan<0.5)
     markersize = 0.09;
  else
     markersize = 0.15;
   float maxdim= max(dim[2],max(dim[0],dim[1]));
   float rot_marker_inner_radius = maxdim+markersize;
   float rot_marker_outer_radius = rot_marker_inner_radius+0.8*markersize;
   float torus_radius = 0.5*(rot_marker_inner_radius+rot_marker_outer_radius);
   float torus_tube_radius = 0.5*(rot_marker_outer_radius-rot_marker_inner_radius);

  Eigen::Vector3f box_dims;
  box_dims<<  markersize,markersize,markersize;

   // add collision objects for floating base markers
   
   
  if ((bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D)||
    (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_D)||
    (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_HALF_D)||
    (bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D_TRANS)||
    (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_D_TRANS))
  {
     
    shared_ptr<Collision_Object> base_x_object_ptr(new Collision_Object_Box("markers::base_x", box_dims, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
    _markers_collision_object_map.insert(make_pair("markers::base_x", base_x_object_ptr));
    _collision_detector_floatingbase_markers->add_collision_object(&*base_x_object_ptr);
    
    shared_ptr<Collision_Object> base_y_object_ptr(new Collision_Object_Box("markers::base_y", box_dims, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
    _markers_collision_object_map.insert(make_pair("markers::base_y", base_y_object_ptr));
    _collision_detector_floatingbase_markers->add_collision_object(&*base_y_object_ptr);
  
  }
  if((bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D)||
      (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_HALF_D)||
      (bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D_TRANS))
  {    
    shared_ptr<Collision_Object> base_z_object_ptr(new Collision_Object_Box("markers::base_z", box_dims, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
    _markers_collision_object_map.insert(make_pair("markers::base_z", base_z_object_ptr));
    _collision_detector_floatingbase_markers->add_collision_object(&*base_z_object_ptr);
  }
  
  if((bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D)||
    (bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D_ROT))
  {    
    shared_ptr<Collision_Object> base_roll_object_ptr(new Collision_Object_Torus("markers::base_roll",torus_radius,torus_tube_radius, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
    _markers_collision_object_map.insert(make_pair("markers::base_roll", base_roll_object_ptr));
    _collision_detector_floatingbase_markers->add_collision_object(&*base_roll_object_ptr);
    
    shared_ptr<Collision_Object> base_pitch_object_ptr(new Collision_Object_Torus("markers::base_pitch",torus_radius,torus_tube_radius, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
    _markers_collision_object_map.insert(make_pair("markers::base_pitch", base_pitch_object_ptr));
    _collision_detector_floatingbase_markers->add_collision_object(&*base_pitch_object_ptr);
  }  

  if ((bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D)||
      (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_D)||
      (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_HALF_D)||
      (bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D_ROT)||
      (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_D_ROT))
  {
      
    shared_ptr<Collision_Object> base_yaw_object_ptr(new Collision_Object_Torus("markers::base_yaw", torus_radius,torus_tube_radius, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
    _markers_collision_object_map.insert(make_pair("markers::base_yaw", base_yaw_object_ptr));
    _collision_detector_floatingbase_markers->add_collision_object(&*base_yaw_object_ptr);
    
  }

 
  // Store marker dims. Will be used later for drawing.  
  _floatingbase_offset = offset;
  _floatingbase_markers_boxsize=markersize;
  _floatingbase_markers_torusdims<<  torus_radius,torus_tube_radius;

}   
//----------------------------------------------------------------------------------------------------------------
void InteractableGlKinematicBody::update_floatingbase_marker_collision_objects()
{

    Eigen::Vector3f p0,p;
    p0 << _T_world_body_future.p[0] + _floatingbase_offset[0], _T_world_body_future.p[1] + _floatingbase_offset[1], _T_world_body_future.p[2] + _floatingbase_offset[2];
    //p0 << _T_world_body.p[0] + _floatingbase_offset[0], _T_world_body.p[1] + _floatingbase_offset[1], _T_world_body.p[2] + _floatingbase_offset[2];
    Eigen::Vector4f q0,q;
    q0 << 0,0,0,1;
    float rot_marker_outer_radius = _floatingbase_markers_torusdims[0]+_floatingbase_markers_torusdims[1];
    double trans_marker_length;
    if (_floatingbase_markers_boxsize > 0.1)
      trans_marker_length = 1.2*rot_marker_outer_radius;
    else
      trans_marker_length = 1.4*rot_marker_outer_radius;  // for small objects
      
  if ((bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D)||
    (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_D)||
    (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_HALF_D)||
    (bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D_TRANS)||
    (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_D_TRANS))
  {      

    shared_ptr<Collision_Object_Box> downcasted_object1(shared_dynamic_cast<Collision_Object_Box>(_markers_collision_object_map.find("markers::base_x")->second));
    p=p0; p[0]+=_marker_dir_flip[0]*trans_marker_length;
    downcasted_object1->set_transform(p,q0);
    
    shared_ptr<Collision_Object_Box> downcasted_object2(shared_dynamic_cast<Collision_Object_Box>(_markers_collision_object_map.find("markers::base_y")->second));
    p=p0; p[1]+=_marker_dir_flip[1]*trans_marker_length;


    downcasted_object2->set_transform(p,q0);
  }  
  if((bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D)||
     (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_HALF_D)||
     (bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D_TRANS))
  {     
    shared_ptr<Collision_Object_Box> downcasted_object3(shared_dynamic_cast<Collision_Object_Box>(_markers_collision_object_map.find("markers::base_z")->second));
    p=p0;p[2]+=_marker_dir_flip[2]*trans_marker_length;
    downcasted_object3->set_transform(p,q0);
  }
    
    double axis[3];
    double q_temp[4];
  if((bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D)||
    (bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D_ROT))
  {      
    shared_ptr<Collision_Object_Torus> downcasted_object4(shared_dynamic_cast<Collision_Object_Torus>(_markers_collision_object_map.find("markers::base_roll")->second));
    axis[0] = 0; axis[1] = 1; axis[2] = 0;
    bot_angle_axis_to_quat(M_PI/2, axis,q_temp);
    q << q_temp[1],q_temp[2],q_temp[3],q_temp[0];
    downcasted_object4->set_transform(p0,q);
    
    shared_ptr<Collision_Object_Torus> downcasted_object5(shared_dynamic_cast<Collision_Object_Torus>(_markers_collision_object_map.find("markers::base_pitch")->second));
    axis[0] = 1; axis[1] = 0; axis[2] = 0;
    bot_angle_axis_to_quat (M_PI/2, axis,q_temp);
    q << q_temp[1],q_temp[2],q_temp[3],q_temp[0];
    downcasted_object5->set_transform(p0,q);
  }
  
  if ((bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D)||
      (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_D)||
      (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_HALF_D)||
      (bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D_ROT)||
      (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_D_ROT))
  {          
    shared_ptr<Collision_Object_Torus> downcasted_object6(shared_dynamic_cast<Collision_Object_Torus>(_markers_collision_object_map.find("markers::base_yaw")->second));
    downcasted_object6->set_transform(p0,q0);
  }  
}
//----------------------------------------------------------------------------------------------------------------
void InteractableGlKinematicBody::draw_floatingbase_markers()
{
   //float pos[3] = {_T_world_body.p[0] + _floatingbase_offset[0], _T_world_body.p[1] + _floatingbase_offset[1], _T_world_body.p[2] + _floatingbase_offset[2]};
   float pos[3] = {_T_world_body_future.p[0] + _floatingbase_offset[0], _T_world_body_future.p[1] + _floatingbase_offset[1], _T_world_body_future.p[2] + _floatingbase_offset[2]};
   
   float rot_marker_inner_radius = _floatingbase_markers_torusdims[0]-_floatingbase_markers_torusdims[1];
   float rot_marker_outer_radius = _floatingbase_markers_torusdims[0]+_floatingbase_markers_torusdims[1];
   draw_markers(pos,_floatingbase_markers_boxsize,rot_marker_inner_radius,rot_marker_outer_radius);
} 

//----------------------------------------------------------------------------------------------------------------
// Joint Dof Markers
//----------------------------------------------------------------------------------------------------------------
void InteractableGlKinematicBody::init_jointdof_marker_collision_objects()
{

  double dof_marker_inner_radius = 0.05;
  double dof_marker_outer_radius = 0.1;
  double torus_radius =0.5*(dof_marker_outer_radius+dof_marker_inner_radius); 
  double torus_tube_radius =0.5*(dof_marker_outer_radius-dof_marker_inner_radius);
  double trans_markersize = 0.05;
  Eigen::Vector3f box_dims;
  box_dims<<  trans_markersize,trans_markersize,trans_markersize;
      
  _collision_detector_jointdof_markers.reset();
  _collision_detector_jointdof_markers = boost::shared_ptr<collision::Collision_Detector>(new collision::Collision_Detector());

 for (size_t j = 0;j < _joint_tfs.size();j++)
  {
    JointFrameStruct jointInfo = _joint_tfs[j];
    int type = jointInfo.type;
    
    //jointInfo.name in filter list?
    std::vector<std::string>::const_iterator found;
    found = std::find (_jointdof_marker_filter.begin(), _jointdof_marker_filter.end(), jointInfo.name);
    bool infilter=(found != _link_geometry_names.end());
                
    if((!_jointdof_marker_filter_on)||(infilter))
    {
      if(((!is_otdf_instance)&&((type==urdf::Joint::REVOLUTE)||(type==urdf::Joint::CONTINUOUS)))||((is_otdf_instance)&&((type==otdf::Joint::REVOLUTE)||(type==otdf::Joint::CONTINUOUS))))
      {
          std::stringstream oss;
          oss << "markers::" <<  jointInfo.name;
          shared_ptr<Collision_Object> object_ptr(new Collision_Object_Torus(oss.str(),torus_radius,torus_tube_radius, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
          _dofmarkers_collision_object_map.insert(make_pair(oss.str(), object_ptr));
          _collision_detector_jointdof_markers->add_collision_object(&*object_ptr);
         
       }//for revolute or continuous joints
       
      if(((!is_otdf_instance)&&(type==urdf::Joint::PRISMATIC))||((is_otdf_instance)&&(type==otdf::Joint::PRISMATIC)))
      {
          std::stringstream oss;
          oss << "markers::" <<  jointInfo.name;
          shared_ptr<Collision_Object> object_ptr(new Collision_Object_Box(oss.str(),box_dims, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
          _dofmarkers_collision_object_map.insert(make_pair(oss.str(), object_ptr));
          _collision_detector_jointdof_markers->add_collision_object(&*object_ptr);

       }//for prismatic
    }//end if((!_jointdof_marker_filter_on)||(infilter)){
  }  
  
}

//----------------------------------------------------------------------------------------------------------------
void InteractableGlKinematicBody::update_jointdof_marker_collision_objects()
{
 
  Eigen::Vector3f p;
  Eigen::Vector4f q;

  for (size_t j = 0;j < _joint_tfs.size();j++)
  {
    JointFrameStruct jointInfo = _joint_tfs[j];
    Eigen::Vector3f joint_axis;
    float pos[3];
    if(future_display_active){
      pos[0] = jointInfo.future_frame.p[0];  pos[1] = jointInfo.future_frame.p[1];  pos[2] = jointInfo.future_frame.p[2];
      joint_axis << jointInfo.future_axis[0],jointInfo.future_axis[1],jointInfo.future_axis[2]; // joint axis in future_world_frame;
    }
    else {
      pos[0] = jointInfo.frame.p[0];  pos[1] = jointInfo.frame.p[1];  pos[2] = jointInfo.frame.p[2];
      joint_axis << jointInfo.axis[0],jointInfo.axis[1],jointInfo.axis[2]; // joint axis in world_frame;
    }
   joint_axis.normalize();
   int type = jointInfo.type;
    //jointInfo.name in filter list?
   std::vector<std::string>::const_iterator found;
   found = std::find (_jointdof_marker_filter.begin(), _jointdof_marker_filter.end(), jointInfo.name);
   bool infilter=(found != _jointdof_marker_filter.end());
                
   if((!_jointdof_marker_filter_on)||(infilter))
   {
   
   
      if(((!is_otdf_instance)&&(type==urdf::Joint::PRISMATIC))||((is_otdf_instance)&&(type==otdf::Joint::PRISMATIC)))
      {
          double theta;
          Eigen::Vector3f axis;      
          Eigen::Vector3f uz; uz << 0 , 0 , 1;
          double arrow_length =0.2;
          
          Eigen::Vector3f u_body_to_joint;
          //u_body_to_joint << _T_world_body.p[0]-jointInfo.frame.p[0], _T_world_body.p[1]-jointInfo.frame.p[1],_T_world_body.p[2]-jointInfo.frame.p[2]; 
          u_body_to_joint << _T_world_body_future.p[0]-jointInfo.future_frame.p[0], _T_world_body_future.p[1]-jointInfo.future_frame.p[1],_T_world_body_future.p[2]-jointInfo.future_frame.p[2];
          u_body_to_joint.normalize();
          double normal = acos(u_body_to_joint.dot(joint_axis));
          double flipped = acos(u_body_to_joint.dot(-joint_axis));
          
          axis = uz.cross(joint_axis);
          theta = acos(uz.dot(joint_axis));
            
          KDL::Frame JointAxisFrame;
          JointAxisFrame.p[0] =pos[0]; JointAxisFrame.p[1] =pos[1]; JointAxisFrame.p[2] =pos[2];
          KDL::Vector axis_temp;
          axis_temp[0]=axis[0];axis_temp[1]=axis[1];axis_temp[2]=axis[2];
          JointAxisFrame.M = KDL::Rotation::Rot(axis_temp,theta);          
        
          KDL::Frame JointAxisOffset = KDL::Frame::Identity();
         
          if(flipped>normal+1e-1) {
            JointAxisOffset.p[2] =-2*arrow_length/3;          
            JointAxisFrame = JointAxisFrame*JointAxisOffset;
            double x,y,z,w;
            JointAxisFrame.M.GetQuaternion(x,y,z,w);
            p << JointAxisFrame.p[0],JointAxisFrame.p[1],JointAxisFrame.p[2];
            q << x,y,z,w;
          }
          else{
            JointAxisOffset.p[2] = 2*arrow_length/3;          
            JointAxisFrame = JointAxisFrame*JointAxisOffset;
            double x,y,z,w;
            JointAxisFrame.M.GetQuaternion(x,y,z,w);
            p << JointAxisFrame.p[0],JointAxisFrame.p[1],JointAxisFrame.p[2];
            q << x,y,z,w;
          }
          
          std::stringstream oss;
          oss << "markers::" <<  jointInfo.name;
          shared_ptr<Collision_Object_Box> downcasted_object(shared_dynamic_cast<Collision_Object_Box>(_dofmarkers_collision_object_map.find(oss.str())->second));
          downcasted_object->set_transform(p,q);
      }//for prismatic   
       
      if(((!is_otdf_instance)&&((type==urdf::Joint::REVOLUTE)||(type==urdf::Joint::CONTINUOUS)))||((is_otdf_instance)&&((type==otdf::Joint::REVOLUTE)||(type==otdf::Joint::CONTINUOUS))))
      {
          //--get rotation in angle/axis form
          double theta;
          Eigen::Vector3f axis;      
          Eigen::Vector3f uz; uz << 0 , 0 , 1;
          double arrow_length =0.2;
          
          Eigen::Vector3f u_body_to_joint;
          //u_body_to_joint << _T_world_body.p[0]-jointInfo.frame.p[0], _T_world_body.p[1]-jointInfo.frame.p[1],_T_world_body.p[2]-jointInfo.frame.p[2]; 
          u_body_to_joint << _T_world_body_future.p[0]-jointInfo.future_frame.p[0], _T_world_body_future.p[1]-jointInfo.future_frame.p[1],_T_world_body_future.p[2]-jointInfo.future_frame.p[2];
          u_body_to_joint.normalize();
          double normal = acos(u_body_to_joint.dot(joint_axis));
          double flipped = acos(u_body_to_joint.dot(-joint_axis));
          
          
          axis = uz.cross(joint_axis);
          theta = acos(uz.dot(joint_axis));
          
          KDL::Frame JointAxisFrame;
          JointAxisFrame.p[0] =pos[0]; JointAxisFrame.p[1] =pos[1]; JointAxisFrame.p[2] =pos[2];
          KDL::Vector axis_temp;
          axis_temp[0]=axis[0];axis_temp[1]=axis[1];axis_temp[2]=axis[2];
          JointAxisFrame.M = KDL::Rotation::Rot(axis_temp,theta);
          KDL::Frame JointAxisOffset = KDL::Frame::Identity();
          
          
          if(flipped>normal+1e-1) {
  //        glTranslatef(pos[0],pos[1],pos[2]);
  //        glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]);
  //        glTranslatef(0,0,-2*arrow_length/3);
  //        gluDisk(quadric,0.05,0.1,36,1);
            JointAxisOffset.p[2] =-2*arrow_length/3;          
            JointAxisFrame = JointAxisFrame*JointAxisOffset;
            double x,y,z,w;
            JointAxisFrame.M.GetQuaternion(x,y,z,w);
            p << JointAxisFrame.p[0],JointAxisFrame.p[1],JointAxisFrame.p[2];
            q << x,y,z,w;
            
            std::stringstream oss;
            oss << "markers::" <<  jointInfo.name;
            shared_ptr<Collision_Object_Torus> downcasted_object(shared_dynamic_cast<Collision_Object_Torus>(_dofmarkers_collision_object_map.find(oss.str())->second));
            downcasted_object->set_transform(p,q);   

          }
          else{
  //          glTranslatef(pos[0],pos[1],pos[2]);
  //          glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]);
  //          glTranslatef(0,0,2*arrow_length/3);
  //          gluDisk(quadric,0.05,0.1,36,1);
            JointAxisOffset.p[2] = 2*arrow_length/3;          
            JointAxisFrame = JointAxisFrame*JointAxisOffset;
            double x,y,z,w;
            JointAxisFrame.M.GetQuaternion(x,y,z,w);
            p << JointAxisFrame.p[0],JointAxisFrame.p[1],JointAxisFrame.p[2];
            q << x,y,z,w;
            
            std::stringstream oss;
            oss << "markers::" <<  jointInfo.name;
            shared_ptr<Collision_Object_Torus> downcasted_object(shared_dynamic_cast<Collision_Object_Torus>(_dofmarkers_collision_object_map.find(oss.str())->second));
            downcasted_object->set_transform(p,q);

          }
        
       }//end if revolute or continuous joints
       
    }// end if ((!_jointdof_marker_filter_on)||(infilter))

  }//end for

}

//----------------------------------------------------------------------------------------------------------------
void InteractableGlKinematicBody::draw_jointdof_markers()
{
  double dof_marker_inner_radius = 0.05;
  double dof_marker_outer_radius = 0.1;
  double length =0.2;//=diff.norm();
  double head_width = 0.03; double head_length = 0.03;double body_width = 0.01;
  double trans_markersize = 0.05;
   glEnable(GL_LINE_SMOOTH); 
   GLUquadricObj* quadric = gluNewQuadric();
   gluQuadricDrawStyle(quadric, GLU_FILL);
   gluQuadricNormals(quadric, GLU_SMOOTH);
   gluQuadricOrientation(quadric, GLU_OUTSIDE);
   
  for (size_t j = 0;j < _joint_tfs.size();j++)
  {
    JointFrameStruct jointInfo = _joint_tfs[j];
  
    Eigen::Vector3f joint_axis;
    float pos[3];
    if(future_display_active){
      pos[0] = jointInfo.future_frame.p[0];  pos[1] = jointInfo.future_frame.p[1];  pos[2] = jointInfo.future_frame.p[2];
      joint_axis << jointInfo.future_axis[0],jointInfo.future_axis[1],jointInfo.future_axis[2]; // joint axis in future_world_frame;
    }
    else {
      pos[0] = jointInfo.frame.p[0];  pos[1] = jointInfo.frame.p[1];  pos[2] = jointInfo.frame.p[2];
      joint_axis << jointInfo.axis[0],jointInfo.axis[1],jointInfo.axis[2]; // joint axis in world_frame;
    }
    joint_axis.normalize();
    int type = jointInfo.type;
    //jointInfo.name in filter list?
   std::vector<std::string>::const_iterator found;
   found = std::find (_jointdof_marker_filter.begin(), _jointdof_marker_filter.end(), jointInfo.name);
   bool infilter=(found != _jointdof_marker_filter.end());
   if((!_jointdof_marker_filter_on)||(infilter))
   {
      if(((!is_otdf_instance)&&(type==urdf::Joint::PRISMATIC))||((is_otdf_instance)&&(type==otdf::Joint::PRISMATIC)))
      {
        //--get rotation in angle/axis form
          double theta;
          Eigen::Vector3f axis;      
          Eigen::Vector3f ux,uz; ux << 1 , 0 , 0;uz << 0 , 0 , 1;
          axis = ux.cross(joint_axis); // Required rotation to be align a arrow in the joint axis direction
          theta = acos(ux.dot(joint_axis));
               
          float c_darkgrey[3] = {0.1,0.1,0.1};
          float c_yellow[3] = {0.5,0.5,0.3};
          float c_blue[3] = {0.3,0.3,0.6};
          float c_red[3] = {0.6,0.3,0.3};

          std::stringstream oss;
          oss << "markers::" <<  jointInfo.name;

          
          Eigen::Vector3f u_body_to_joint;
          //u_body_to_joint << _T_world_body.p[0]-jointInfo.frame.p[0], _T_world_body.p[1]-jointInfo.frame.p[1],_T_world_body.p[2]-jointInfo.frame.p[2]; 
          u_body_to_joint << _T_world_body_future.p[0]-jointInfo.future_frame.p[0], _T_world_body_future.p[1]-jointInfo.future_frame.p[1],_T_world_body_future.p[2]-jointInfo.future_frame.p[2];
          u_body_to_joint.normalize();
          double normal = acos(u_body_to_joint.dot(joint_axis));
          double flipped = acos(u_body_to_joint.dot(-joint_axis));
          if(flipped>normal+1e-1) {
            glColor4f(c_darkgrey[0],c_darkgrey[1],c_darkgrey[2],1);
            glPushMatrix();
            glTranslatef(pos[0],pos[1],pos[2]);
            glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]); // Rotation from joint frame to joint axis direction 
            glTranslatef(-length/2, 0,0); 
            bot_gl_draw_arrow_3d(length,head_width, head_length,body_width);
            glPopMatrix();

            glColor4f(c_red[0],c_red[1],c_red[2],0.5);
            if(selected_marker==oss.str())
              glColor4f(0.7,0.1,0.1,1.0);
            axis = uz.cross(joint_axis);
            theta = acos(uz.dot(joint_axis));
            glPushMatrix();
            glTranslatef(pos[0],pos[1],pos[2]);
            glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]); 
            glTranslatef(0,0,-2*length/3);
            glScalef(trans_markersize,trans_markersize,trans_markersize);
            bot_gl_draw_cube();
            glPopMatrix();
          }
          else{
            glColor4f(c_darkgrey[0],c_darkgrey[1],c_darkgrey[2],1);
            glPushMatrix();
            glTranslatef(pos[0],pos[1],pos[2]);
            glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]);
            glTranslatef(length/2, 0,0);
            bot_gl_draw_arrow_3d(length,head_width, head_length,body_width);
            glPopMatrix();

            glColor4f(c_red[0],c_red[1],c_red[2],0.5);
            if(selected_marker==oss.str())
              glColor4f(0.7,0.1,0.1,1.0);
            axis = uz.cross(joint_axis);
            theta = acos(uz.dot(joint_axis));
            glPushMatrix();
            glTranslatef(pos[0],pos[1],pos[2]);
            glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]);
            glTranslatef(0,0,2*length/3);
            glScalef(trans_markersize,trans_markersize,trans_markersize);
            bot_gl_draw_cube();
            glPopMatrix();
          }
      }//for prismatic       
        
      if(((!is_otdf_instance)&&((type==urdf::Joint::REVOLUTE)||(type==urdf::Joint::CONTINUOUS)))||((is_otdf_instance)&&((type==otdf::Joint::REVOLUTE)||(type==otdf::Joint::CONTINUOUS))))
      {
      
          //--get rotation in angle/axis form
          double theta;
          Eigen::Vector3f axis;      
          Eigen::Vector3f ux,uz; ux << 1 , 0 , 0;uz << 0 , 0 , 1;
          axis = ux.cross(joint_axis); // Required rotation to be align a arrow in the joint axis direction
          theta = acos(ux.dot(joint_axis));
               
          float c_darkgrey[3] = {0.1,0.1,0.1};
          float c_yellow[3] = {0.5,0.5,0.3};
          float c_blue[3] = {0.3,0.3,0.6};
          float c_red[3] = {0.6,0.3,0.3};

          std::stringstream oss;
          oss << "markers::" <<  jointInfo.name;

          
          Eigen::Vector3f u_body_to_joint;
          //u_body_to_joint << _T_world_body.p[0]-jointInfo.frame.p[0], _T_world_body.p[1]-jointInfo.frame.p[1],_T_world_body.p[2]-jointInfo.frame.p[2]; 
          u_body_to_joint << _T_world_body_future.p[0]-jointInfo.future_frame.p[0], _T_world_body_future.p[1]-jointInfo.future_frame.p[1],_T_world_body_future.p[2]-jointInfo.future_frame.p[2];
          u_body_to_joint.normalize();
          double normal = acos(u_body_to_joint.dot(joint_axis));
          double flipped = acos(u_body_to_joint.dot(-joint_axis));
          if(flipped>normal+1e-1) {
            glColor4f(c_darkgrey[0],c_darkgrey[1],c_darkgrey[2],1);
            glPushMatrix();
            glTranslatef(pos[0],pos[1],pos[2]);
            glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]); // Rotation from joint frame to joint axis direction 
            glTranslatef(-length/2, 0,0); 
            bot_gl_draw_arrow_3d(length,head_width, head_length,body_width);
            glPopMatrix();

            glColor4f(c_red[0],c_red[1],c_red[2],0.5);
            if(selected_marker==oss.str())
              glColor4f(0.7,0.1,0.1,1.0);
            axis = uz.cross(joint_axis);
            theta = acos(uz.dot(joint_axis));
            glPushMatrix();
            glTranslatef(pos[0],pos[1],pos[2]);
            glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]); 
            glTranslatef(0,0,-2*length/3);
            gluDisk(quadric,dof_marker_inner_radius,dof_marker_outer_radius,36,1);
            glPopMatrix();
          }
          else{
            glColor4f(c_darkgrey[0],c_darkgrey[1],c_darkgrey[2],1);
            glPushMatrix();
            glTranslatef(pos[0],pos[1],pos[2]);
            glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]);
            glTranslatef(length/2, 0,0);
            bot_gl_draw_arrow_3d(length,head_width, head_length,body_width);
            glPopMatrix();

            glColor4f(c_red[0],c_red[1],c_red[2],0.5);
            if(selected_marker==oss.str())
              glColor4f(0.7,0.1,0.1,1.0);
            axis = uz.cross(joint_axis);
            theta = acos(uz.dot(joint_axis));
            glPushMatrix();
            glTranslatef(pos[0],pos[1],pos[2]);
            glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]);
            glTranslatef(0,0,2*length/3);
            gluDisk(quadric,dof_marker_inner_radius,dof_marker_outer_radius,36,1);
            glPopMatrix();
          }
        
       }//end if revolute or continuous joints
    }// end if((!_jointdof_marker_filter_on)||(infilter))
  }//end for

} 

//----------------------------------------------------------------------------------------------------------------

void InteractableGlKinematicBody::draw_interactable_markers(boost::shared_ptr<otdf::Geometry> &_link_shape,const LinkFrameStruct &link_tf)
{
  float pos[3] = {link_tf.frame.p[0],link_tf.frame.p[1],link_tf.frame.p[2]};

  float markersize = 0.15;
  int type = _link_shape->type ;
  //enum {SPHERE, BOX, CYLINDER, MESH, TORUS}; 
  if (type == otdf::Geometry::SPHERE)
  {
    boost::shared_ptr<otdf::Sphere> sphere(boost::shared_dynamic_cast<otdf::Sphere>(_link_shape));	
    float radius = sphere->radius;
    float dims[3] = {radius+2*markersize,radius+2*markersize,radius+2*markersize};
    float maxdim= max(dims[2],max(dims[0],dims[1]));
    draw_markers(pos,markersize,maxdim+markersize,maxdim+1.8*markersize);  
  }
  else if  (type == otdf::Geometry::BOX)
  {
    boost::shared_ptr<otdf::Box> box(boost::shared_dynamic_cast<otdf::Box>(_link_shape));
    float dims[3] = {0.5*box->dim.x+2*markersize,0.5*box->dim.y+2*markersize,0.5*box->dim.z+2*markersize};
    float maxdim= max(dims[2],max(dims[0],dims[1]));
    draw_markers(pos,markersize,maxdim+markersize,maxdim+1.8*markersize);  
  }
  else if  (type == otdf::Geometry::CYLINDER)
  {
    boost::shared_ptr<otdf::Cylinder> cyl(boost::shared_dynamic_cast<otdf::Cylinder>(_link_shape));
    float dims[3] = {cyl->radius+2*markersize,cyl->radius+2*markersize,0.5*cyl->length+2*markersize};
    float maxdim= max(dims[2],max(dims[0],dims[1]));
    draw_markers(pos,markersize,maxdim+markersize,maxdim+1.8*markersize);  
  }
  else if  (type == otdf::Geometry::MESH)
  {
    //std::map<std::string, MeshStruct>::const_iterator mesh_map_it;
    std::map<std::string, shared_ptr<MeshStruct> >::const_iterator mesh_map_it;
    mesh_map_it=_mesh_map.find(link_tf.name);
    if(mesh_map_it!=_mesh_map.end()) // exists in cache
    { 
      // get the vertices for mesh_map_it->second
      /*float xDim = mesh_map_it->second.span_x;
      float yDim = mesh_map_it->second.span_y;
      float zDim = mesh_map_it->second.span_z;
      float xc = mesh_map_it->second.offset_x;
      float yc = mesh_map_it->second.offset_y;
      float zc = mesh_map_it->second.offset_z;*/

      float xDim = mesh_map_it->second->span_x;
      float yDim = mesh_map_it->second->span_y;
      float zDim = mesh_map_it->second->span_z;
      float xc = mesh_map_it->second->offset_x;
      float yc = mesh_map_it->second->offset_y;
      float zc = mesh_map_it->second->offset_z;

      float dims[3] = {0.5*xDim+2*markersize,0.5*yDim+2*markersize,0.5*zDim+2*markersize};
      float newpos[3] = {pos[0]+xc,pos[1]+yc,pos[2]+zc}; // meshes have a visual offset.
      float maxdim= max(dims[2],max(dims[0],dims[1]));
      draw_markers(newpos,markersize,maxdim+markersize,maxdim+1.8*markersize);  
    }
  }
  else if  (type == otdf::Geometry::TORUS)
  {
    boost::shared_ptr<otdf::Torus> torus(boost::shared_dynamic_cast<otdf::Torus>(_link_shape));
    float dims[3] = {torus->radius+2*markersize,torus->radius+2*markersize,torus->tube_radius+2*markersize};
    float maxdim= max(dims[2],max(dims[0],dims[1]));
    draw_markers(pos,markersize,maxdim+markersize,maxdim+1.8*markersize);   
  }
}

     
//----------------------------------------------------------------------------------------------------------------   
void InteractableGlKinematicBody::draw_interactable_markers(boost::shared_ptr<urdf::Geometry> &_link_shape,const LinkFrameStruct &link_tf)
{
  float pos[3] = {link_tf.frame.p[0],link_tf.frame.p[1],link_tf.frame.p[2]};

  float markersize = 0.15;
  int type = _link_shape->type ;
  //enum {SPHERE, BOX, CYLINDER, MESH}; 
  if (type == urdf::Geometry::SPHERE)
  {
    boost::shared_ptr<urdf::Sphere> sphere(boost::shared_dynamic_cast<urdf::Sphere>(_link_shape));	
    float radius = sphere->radius;
    float dims[3] = {radius+2*markersize,radius+2*markersize,radius+2*markersize};
    float maxdim= max(dims[2],max(dims[0],dims[1]));
    draw_markers(pos,markersize,maxdim+markersize,maxdim+1.8*markersize);  
  }
  else if  (type == urdf::Geometry::BOX)
  {
    boost::shared_ptr<urdf::Box> box(boost::shared_dynamic_cast<urdf::Box>(_link_shape));
    float dims[3] = {0.5*box->dim.x+2*markersize,0.5*box->dim.y+2*markersize,0.5*box->dim.z+2*markersize};
    float maxdim= max(dims[2],max(dims[0],dims[1]));
    draw_markers(pos,markersize,maxdim+markersize,maxdim+1.8*markersize);  
  }
  else if  (type == urdf::Geometry::CYLINDER)
  {
    boost::shared_ptr<urdf::Cylinder> cyl(boost::shared_dynamic_cast<urdf::Cylinder>(_link_shape));
    float dims[3] = {cyl->radius+2*markersize,cyl->radius+2*markersize,0.5*cyl->length+2*markersize};
    float maxdim= max(dims[2],max(dims[0],dims[1]));
    draw_markers(pos,markersize,maxdim+markersize,maxdim+1.8*markersize);   
  }
  else if  (type == urdf::Geometry::MESH)
  {
    // std::map<std::string, MeshStruct>::const_iterator mesh_map_it;
    std::map<std::string, shared_ptr<MeshStruct> >::const_iterator mesh_map_it;

    mesh_map_it=_mesh_map.find(link_tf.name);
    if(mesh_map_it!=_mesh_map.end()) // exists in cache
    { 
      // get the vertices for mesh_map_it->second
      /*float xDim = mesh_map_it->second.span_x;
      float yDim = mesh_map_it->second.span_y;
      float zDim = mesh_map_it->second.span_z;
      float xc = mesh_map_it->second.offset_x;
      float yc = mesh_map_it->second.offset_y;
      float zc = mesh_map_it->second.offset_z;*/

      float xDim = mesh_map_it->second->span_x;
      float yDim = mesh_map_it->second->span_y;
      float zDim = mesh_map_it->second->span_z;
      float xc = mesh_map_it->second->offset_x;
      float yc = mesh_map_it->second->offset_y;
      float zc = mesh_map_it->second->offset_z;
      float dims[3] = {0.5*xDim+2*markersize,0.5*yDim+2*markersize,0.5*zDim+2*markersize};
      float newpos[3] = {pos[0]+xc,pos[1]+yc,pos[2]+zc}; // meshes have a visual offset.
       
      float maxdim= max(dims[2],max(dims[0],dims[1]));
      draw_markers(newpos,markersize,maxdim+markersize,maxdim+1.8*markersize);  
    }
  }

}        
//----------------------------------------------------------------------------------------------------------------
void InteractableGlKinematicBody::draw_markers(float (&pos)[3], float trans_marker_boxsize, float rot_marker_inner_radius,float rot_marker_outer_radius)
{
   glEnable(GL_LINE_SMOOTH); 
   GLUquadricObj* quadric = gluNewQuadric();
   gluQuadricDrawStyle(quadric, GLU_FILL);
   gluQuadricNormals(quadric, GLU_SMOOTH);
   gluQuadricOrientation(quadric, GLU_OUTSIDE);

   
   double trans_marker_length;
   if (trans_marker_boxsize > 0.1)
     trans_marker_length = 1.2*rot_marker_outer_radius;
   else
     trans_marker_length = 1.4*rot_marker_outer_radius;  // for small objects
     
   
    float ORG[3] = {0.0, 0.0, 0.0};
    float XP[3] = {_marker_dir_flip[0]*trans_marker_length, 0.0, 0.0};
    float YP[3] = {0.0, _marker_dir_flip[1]*trans_marker_length, 0.0};
    float ZP[3] = {0.0, 0.0, _marker_dir_flip[2]*trans_marker_length};


    glPushMatrix();
    glTranslatef(pos[0],pos[1],pos[2]);
    glLineWidth (3.0);
    glBegin (GL_LINES);
    glColor3f (1,0,0); // X axis is red.
    glVertex3fv (ORG);
    glVertex3fv (XP );
    glColor3f (0,1,0); // Y axis is green.
    glVertex3fv (ORG);
    glVertex3fv (YP );
    glColor3f (0,0,1); // z axis is blue.
    glVertex3fv (ORG);
    glVertex3fv (ZP );
    glEnd();
    glPopMatrix();


    double alpha=0.4;
    

    if ((bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D)||
        (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_D)||
        (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_HALF_D)||
        (bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D_TRANS)||
        (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_D_TRANS))
    {
    
      glPushMatrix();
      glTranslatef(pos[0]+_marker_dir_flip[0]*trans_marker_length,pos[1],pos[2]);
      glScalef(trans_marker_boxsize,trans_marker_boxsize,trans_marker_boxsize);
      glColor4f(1.0, 0.0, 0.0,alpha);
      if(selected_marker=="markers::base_x")
        glColor4f(0.7,0.1,0.1,1.0);
      bot_gl_draw_cube();
      glPopMatrix();
      
      glPushMatrix();
      glTranslatef(pos[0],pos[1]+_marker_dir_flip[1]*trans_marker_length,pos[2]);
      glColor4f(0.0, 1.0, 0.0,alpha);
      if(selected_marker=="markers::base_y")
        glColor4f(0.7,0.1,0.1,1.0);
      glScalef(trans_marker_boxsize,trans_marker_boxsize,trans_marker_boxsize);
      bot_gl_draw_cube();
      glPopMatrix();
    
    }
    
    if ((bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D)||
        (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_D)||
        (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_HALF_D)||
        (bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D_ROT)||
        (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_D_ROT))
    {
      
      glPushMatrix();
      glColor4f(0.0, 0.0, 1.0,alpha);
      if(selected_marker=="markers::base_yaw")
        glColor4f(0.7,0.1,0.1,1.0);
      glTranslatef(pos[0],pos[1],pos[2]);
      gluDisk(quadric,rot_marker_inner_radius,rot_marker_outer_radius,36,1);
      glPopMatrix();   
      
    }
    
    if((bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D)||
        (bodypose_adjustment_type == InteractableGlKinematicBody::TWO_HALF_D)||    
        (bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D_TRANS))
    {        
      glPushMatrix();
      glTranslatef(pos[0],pos[1],pos[2]+_marker_dir_flip[2]*trans_marker_length);
      glColor4f(0.0, 0.0, 1.0,alpha);
      if(selected_marker=="markers::base_z")
        glColor4f(0.7,0.1,0.1,1.0);
      glScalef(trans_marker_boxsize,trans_marker_boxsize,trans_marker_boxsize);
      bot_gl_draw_cube();
      glPopMatrix();
    }
    
    if((bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D)||
      (bodypose_adjustment_type == InteractableGlKinematicBody::THREE_D_ROT))
    {       
      glPushMatrix();
      glColor4f(1.0, 0.0, 0.0,alpha);
      if(selected_marker=="markers::base_roll")
        glColor4f(0.7,0.1,0.1,1.0);
      glTranslatef(pos[0],pos[1],pos[2]);
      glRotatef(90,0,1,0);   
      gluDisk(quadric,rot_marker_inner_radius,rot_marker_outer_radius,36,1);
      glPopMatrix();
      
      glPushMatrix();
      glColor4f(0.0, 1.0, 0.0,alpha);
      if(selected_marker=="markers::base_pitch")
        glColor4f(0.7,0.1,0.1,1.0);
      glTranslatef(pos[0],pos[1],pos[2]);
      glRotatef(90,1,0,0);
      gluDisk(quadric,rot_marker_inner_radius,rot_marker_outer_radius,36,1);
      //glutSolidTorus(rot_marker_inner_radius,rot_marker_outer_radius,36,36);
      glPopMatrix();
    }


} 

//----------------------------------------------------------------------------------------------------------------                                       

bool InteractableGlKinematicBody::draw_mesh(int linkType){
  /////////////////////////////////////////////////////////
  // draw mesh

  //--get rotation in angle/axis form
  KDL::Frame& nextTfframe = _T_world_body;
  double theta;
  double axis[3];
  double x,y,z,w;
  nextTfframe.M.GetQuaternion(x,y,z,w);
  double quat[4] = {w,x,y,z};
  bot_quat_to_angle_axis(quat, &theta, axis);

  // if Show Mesh checked and object has mesh, set drawMesh to true
  const std::vector<Eigen::Vector3i>& tri = triangles;
  const std::vector<Eigen::Vector3f>& pts = points;

  // decide whether of not to draw mesh in place of object
  bool drawMesh=false;
  if(isShowMeshSelected && pts.size()>0) drawMesh = true;
  if(linkType == otdf::Geometry::DYNAMIC_MESH) drawMesh = true;


  //cout << link->type << " " << isShowMeshSelected << " " << drawMesh << " " << pts.size() << " " << triangles.size() << endl;

  // draw triangles if available
  if(tri.size()>0 && drawMesh){
    glPushMatrix();
    //glColor4f(c[0],c[1],c[2],self->alpha);
    glTranslatef(nextTfframe.p[0], nextTfframe.p[1], nextTfframe.p[2]);
    glRotatef(theta * 180/M_PI, axis[0], axis[1], axis[2]); 
    for(size_t i=0; i<tri.size(); i++){
      glBegin(GL_POLYGON);
      for(size_t j=0; j<3; j++) glVertex3fv(pts[tri[i][j]].data());
      glEnd();
    }
    glPopMatrix();
  }

  // draw points if available and no triangles
  if(pts.size()>0 && tri.size()==0 && drawMesh){
    glPushMatrix();
    //glColor4f(c[0],c[1],c[2],self->alpha);
    glTranslatef(nextTfframe.p[0], nextTfframe.p[1], nextTfframe.p[2]);
    glRotatef(theta * 180/M_PI, axis[0], axis[1], axis[2]); 
    glEnable(GL_BLEND); //for dimming contrast
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glPointSize(2.0);
    glBegin(GL_POINTS);
    for(size_t i=0; i<pts.size(); i++) glVertex3fv(pts[i].data());
    glEnd();
    glPopMatrix();
  }

  return drawMesh;
}

//----------------------------------------------------------------------------------------------------------------                                       


void InteractableGlKinematicBody::draw_body (float (&c)[3], float alpha)
{
  glColor4f(c[0],c[1],c[2],alpha);
  double t;
  if(enable_blinking){
    int64_t now=bot_timestamp_now();
    t=bot_timestamp_useconds(now)*1e-6;//in sec
    alpha=std::min(fabs(sin(M_PI*t)),1.0);
    c[0]=0.7; c[1]=0.1; c[2]=0.1;
  }
 
  bool drawMesh = false;
  if(!_otdf_link_shapes.empty()) {
    drawMesh = draw_mesh(_otdf_link_shapes[0]->type);
  }

  for(uint i = 0; i < _link_geometry_tfs.size(); i++)
  {
    LinkFrameStruct nextTf=_link_geometry_tfs[i];
    std::stringstream oss;
    oss << _unique_name << "_"<< _link_geometry_tfs[i].name; 
    if((link_selection_enabled)&&(selected_link == oss.str())) {
//          if((bodypose_adjustment_enabled)&&(is_otdf_instance))
//            draw_interactable_markers(_otdf_link_shapes[i],_link_geometry_tfs[i]); // draws shapes and adds them to _collision_detector 
//          else if((bodypose_adjustment_enabled)&&(!is_otdf_instance)) 
//            draw_interactable_markers(_link_shapes[i],_link_geometry_tfs[i]);   
        
      glColor4f(0.7,0.1,0.1,alpha);         
    }
    else
       glColor4f(c[0],c[1],c[2],alpha);

    if((whole_body_selection_enabled)&&(selected_link == _unique_name)){
      glColor4f(0.7,0.1,0.1,alpha); // whole body is selected instead of an individual link
    } 
    //int64_t tic = bot_timestamp_now();
    GlKinematicBody::draw_link_current_and_future(c,alpha,i,nextTf);  
    //int64_t toc = bot_timestamp_now();
    //std::cout << _link_geometry_tfs[i].name << " usec: " << bot_timestamp_useconds(toc-tic) << std::endl;
    
  }
  
  //draw_whole_body_bbox();
   if((_root_name!="world")&&(bodypose_adjustment_enabled)){
    if(_floatingbase_markers_boxsize!=0){
      draw_floatingbase_markers();
    }
   }
   if(jointdof_adjustment_enabled){
     draw_jointdof_markers();
   }
   

};


//----------------------------------------------------------------------------------------------------------------                                       
 


