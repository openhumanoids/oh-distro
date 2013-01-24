#include "InteractableGlKinematicBody.hpp"
#include <algorithm> // using std::find

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;

// Copy constructors
InteractableGlKinematicBody::InteractableGlKinematicBody( const InteractableGlKinematicBody& other, string unique_name): 
GlKinematicBody(other),
link_selection_enabled(other.link_selection_enabled),
_collision_detector(other._collision_detector),
_unique_name(unique_name)
{ 
  link_adjustment_enabled =false;
  whole_body_selection_enabled = false;
  selected_link = " ";
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
  link_adjustment_enabled =false;
  whole_body_selection_enabled = false;
  selected_link = " ";
  if(is_otdf_instance)
    init_otdf_collision_objects();
  else
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
  link_adjustment_enabled =false;
  whole_body_selection_enabled = false;
  selected_link = " ";
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
      
      std::stringstream oss;
      oss << _unique_name << "_"<< it->first; 
      //
      
      int type = it->second->visual->geometry->type;

      enum {SPHERE, BOX, CYLINDER, MESH}; 

      if  (type == SPHERE)
      {
        shared_ptr<urdf::Sphere> sphere(shared_dynamic_cast<urdf::Sphere>(it->second->visual->geometry));	
        double radius = sphere->radius;
 
        shared_ptr<Collision_Object> object_ptr(new Collision_Object_Sphere(oss.str(), radius, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
        _collision_object_map.insert(make_pair(oss.str(), object_ptr));
         
        // add a collision object to the collision detector class
        _collision_detector->add_collision_object(&*_collision_object_map.find(oss.str())->second);
      }
      else if  (type == BOX)
      {
        shared_ptr<urdf::Box> box(shared_dynamic_cast<urdf::Box>(it->second->visual->geometry));
        Eigen::Vector3f dims;
        dims<< box->dim.x,box->dim.y,box->dim.z;
        shared_ptr<Collision_Object> object_ptr(new Collision_Object_Box(oss.str(), dims, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
        _collision_object_map.insert(make_pair(oss.str(), object_ptr));
        // add a collision object to the collision detector class
        _collision_detector->add_collision_object(&*_collision_object_map.find(oss.str())->second);
      }
      else if  (type == CYLINDER)
      {
        shared_ptr<urdf::Cylinder> cyl(shared_dynamic_cast<urdf::Cylinder>(it->second->visual->geometry));
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
        shared_ptr<urdf::Mesh> mesh(shared_dynamic_cast<urdf::Mesh>(it->second->visual->geometry));

      typedef std::map<std::string, MeshStruct > mesh_map_type_;
      mesh_map_type_::iterator mesh_map_it = _mesh_map.find(it->first);
 
        Eigen::Vector3f dims;
        dims<<  mesh_map_it->second.span_x,mesh_map_it->second.span_y,mesh_map_it->second.span_z;
  
        shared_ptr<Collision_Object> object_ptr(new Collision_Object_Box(oss.str(), dims, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
        _collision_object_map.insert(make_pair(oss.str(), object_ptr));
         
        // add a collision object to the collision detector class
        _collision_detector->add_collision_object(&*_collision_object_map.find(oss.str())->second);
        //std::cout  << "num_colls: " << _collision_detector->num_collisions() <<  std::endl;

      }//end  if  (type == MESH)
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
  link_adjustment_enabled =false;
  whole_body_selection_enabled = false;
  selected_link = " ";
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
      std::stringstream oss;
      oss << _unique_name << "_"<< it->first; 
      int type = it->second->visual->geometry->type;

      enum {SPHERE, BOX, CYLINDER, MESH, TORUS}; 
      //TODO: create collision objects for cylinder,sphere and torus.
      if  (type == SPHERE)
      {
        shared_ptr<otdf::Sphere> sphere(shared_dynamic_cast<otdf::Sphere>(it->second->visual->geometry));	
        double radius = sphere->radius;
        shared_ptr<Collision_Object> object_ptr(new Collision_Object_Sphere(oss.str(), radius, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
        _collision_object_map.insert(make_pair(oss.str(), object_ptr));         
        // add a collision object to the collision detector class
        _collision_detector->add_collision_object(&*_collision_object_map.find(oss.str())->second);
      }
      else if  (type == BOX)
      {
        shared_ptr<otdf::Box> box(shared_dynamic_cast<otdf::Box>(it->second->visual->geometry));
        Eigen::Vector3f dims;
        dims<< box->dim.x,box->dim.y,box->dim.z;
        shared_ptr<Collision_Object> object_ptr(new Collision_Object_Box(oss.str(), dims, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
        _collision_object_map.insert(make_pair(oss.str(), object_ptr));
        // add a collision object to the collision detector class
        _collision_detector->add_collision_object(&*_collision_object_map.find(oss.str())->second);
      }
      else if  (type == CYLINDER)
      {
        shared_ptr<otdf::Cylinder> cyl(shared_dynamic_cast<otdf::Cylinder>(it->second->visual->geometry));
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
        shared_ptr<otdf::Torus> tor(shared_dynamic_cast<otdf::Torus>(it->second->visual->geometry));
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
        shared_ptr<otdf::Mesh> mesh(shared_dynamic_cast<otdf::Mesh>(it->second->visual->geometry));

        typedef std::map<std::string, MeshStruct > mesh_map_type_;
        mesh_map_type_::iterator mesh_map_it = _mesh_map.find(it->first);
 
        Eigen::Vector3f dims;
        dims<<  mesh_map_it->second.span_x,mesh_map_it->second.span_y,mesh_map_it->second.span_z;
        
        shared_ptr<Collision_Object> object_ptr(new Collision_Object_Box(oss.str(), dims, Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) ));
        _collision_object_map.insert(make_pair(oss.str(), object_ptr));         
        // add a collision object to the collision detector class
        _collision_detector->add_collision_object(&*_collision_object_map.find(oss.str())->second);
        //std::cout  << "num_colls: " << _collision_detector->num_collisions() <<  std::endl;

      }//end  if  (type == MESH)
    } // end if(it->second->visual)
  } // end for

}
//======================================================================================================
InteractableGlKinematicBody::~InteractableGlKinematicBody()
{

  // clear the collision world
  //_collision_detector->clear_collision_objects(); // The parent program should handle this.
}

//======================================================================================================
// update methods for plain urdf

void InteractableGlKinematicBody::set_state(const drc::robot_state_t &msg)
{
    GlKinematicBody::set_state(msg);  //code re-use
    update_urdf_collision_objects();
} // end InteractableGlKinematicBody::set_state(const drc::robot_state_t)


void InteractableGlKinematicBody::set_state(const KDL::Frame &T_world_body, const drc::joint_angles_t &msg)
{
    GlKinematicBody::set_state(T_world_body,msg);  //code re-use
    update_urdf_collision_objects();
} // end InteractableGlKinematicBody::set_state(const KDL::Frame, const drc::joint_angles_t)

void InteractableGlKinematicBody::update_urdf_collision_objects(void)
{
 
    typedef map<string, shared_ptr<urdf::Link> > links_mapType;
    for( links_mapType::const_iterator it =  _links_map.begin(); it!= _links_map.end(); it++)
    {  		
      std::stringstream oss;
      oss << _unique_name << "_"<< it->first;
      if(it->second->visual)
      {
          KDL::Frame  T_world_visual;
          LinkFrameStruct state;	    

         // retrieve T_world_visual from store
         // #include <algorithm>
          std::vector<std::string>::const_iterator found;
          found = std::find (_link_names.begin(), _link_names.end(), it->first);
          if (found != _link_names.end()) {
            unsigned int index = found - _link_names.begin();
            state=_link_tfs[index];  
            T_world_visual = state.frame;
          } 
          else 
          {  
           T_world_visual = KDL::Frame::Identity();
           std::cerr << "ERROR:"<< it->first << " not found in _link_names" << std::endl;
          }

          int type = it->second->visual->geometry->type;
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
            MeshStruct mesh_struct = _mesh_map.find(it->first)->second;

            KDL::Frame T_visual_objorigin, T_world_objorigin; 

            T_visual_objorigin = KDL::Frame::Identity();
            T_visual_objorigin.p[0]= mesh_struct.offset_x;
            T_visual_objorigin.p[1]= mesh_struct.offset_y;
            T_visual_objorigin.p[2]= mesh_struct.offset_z;

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
} // end InteractableGlKinematicBody::set_state(const drc::robot_state_t &msg)

void InteractableGlKinematicBody::update_otdf_collision_objects(void)
{
  
    typedef map<string, shared_ptr<otdf::Link> > links_mapType;
    for( links_mapType::const_iterator it =  _otdf_links_map.begin(); it!= _otdf_links_map.end(); it++)
    {  		
      std::stringstream oss;
      oss << _unique_name << "_"<< it->first; 
      if(it->second->visual)
      {
          KDL::Frame  T_world_visual;
          LinkFrameStruct state;	    

         // retrieve T_world_visual from store
         // #include <algorithm>
          std::vector<std::string>::const_iterator found;
          found = std::find (_link_names.begin(), _link_names.end(), it->first);
          if (found != _link_names.end()) {
            unsigned int index = found - _link_names.begin();
            state=_link_tfs[index]; 
            T_world_visual = state.frame;
          } 
          else 
          {  
           T_world_visual = KDL::Frame::Identity();
           std::cerr << "ERROR:"<< it->first << " not found in _link_names" << std::endl;
          }

          int type = it->second->visual->geometry->type;
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
            MeshStruct mesh_struct = _mesh_map.find(it->first)->second;

            KDL::Frame T_visual_objorigin, T_world_objorigin; 

            T_visual_objorigin = KDL::Frame::Identity();
            T_visual_objorigin.p[0]= mesh_struct.offset_x;
            T_visual_objorigin.p[1]= mesh_struct.offset_y;
            T_visual_objorigin.p[2]= mesh_struct.offset_z;

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
            
            //shared_ptr<urdf::Box> box(shared_dynamic_cast<urdf::Box>(it->second->visual->geometry));
            shared_ptr<Collision_Object_Box> downcasted_object(shared_dynamic_cast<Collision_Object_Box>(_collision_object_map.find(oss.str())->second));
            downcasted_object->set_transform(p,q); 

          } // end if MESH


      }//if(it->second->visual)

    }//end for

}

bool InteractableGlKinematicBody::get_link_frame(const std::string &link_name, KDL::Frame &T_world_link)
{
   if(GlKinematicBody::get_link_frame(link_name,T_world_link))
      return true;
   else
      return false;
}  

//==================================================================================================== 	  
// drawing utils

void InteractableGlKinematicBody::draw_interactable_markers(boost::shared_ptr<otdf::Geometry> &_link_shape,const LinkFrameStruct &link_tf)
{
  double pos[3] = {link_tf.frame.p[0],link_tf.frame.p[1],link_tf.frame.p[2]};

  double markersize = 0.15;
  int type = _link_shape->type ;
  enum {SPHERE, BOX, CYLINDER, MESH, TORUS}; 
  if (type == SPHERE)
  {
    boost::shared_ptr<otdf::Sphere> sphere(boost::shared_dynamic_cast<otdf::Sphere>(_link_shape));	
    double radius = sphere->radius;
    double dims[3] = {radius+2*markersize,radius+2*markersize,radius+2*markersize};
    draw_markers(pos,dims,markersize);  
  }
  else if  (type == BOX)
  {
    boost::shared_ptr<otdf::Box> box(boost::shared_dynamic_cast<otdf::Box>(_link_shape));
    double dims[3] = {0.5*box->dim.x+2*markersize,0.5*box->dim.y+2*markersize,0.5*box->dim.z+2*markersize};
    draw_markers(pos,dims,markersize);
  }
  else if  (type == CYLINDER)
  {
    boost::shared_ptr<otdf::Cylinder> cyl(boost::shared_dynamic_cast<otdf::Cylinder>(_link_shape));
    double dims[3] = {cyl->radius+2*markersize,cyl->radius+2*markersize,0.5*cyl->length+2*markersize};
    draw_markers(pos,dims,markersize);  
  }
  else if  (type == MESH)
  {
    std::map<std::string, MeshStruct>::const_iterator mesh_map_it;
    mesh_map_it=_mesh_map.find(link_tf.name);
    if(mesh_map_it!=_mesh_map.end()) // exists in cache
    { 
      // get the vertices for mesh_map_it->second
      double xDim = mesh_map_it->second.span_x;
      double yDim = mesh_map_it->second.span_y;
      double zDim = mesh_map_it->second.span_z;
      double xc = mesh_map_it->second.offset_x;
      double yc = mesh_map_it->second.offset_y;
      double zc = mesh_map_it->second.offset_z;
      double dims[3] = {0.5*xDim+2*markersize,0.5*yDim+2*markersize,0.5*zDim+2*markersize};
      double newpos[3] = {pos[0]+xc,pos[1]+yc,pos[2]+zc}; // meshes have a visual offset.
      draw_markers(newpos,dims,markersize);  
    }
  }
  else if  (type == TORUS)
  {
    boost::shared_ptr<otdf::Torus> torus(boost::shared_dynamic_cast<otdf::Torus>(_link_shape));
    double dims[3] = {torus->radius+2*markersize,torus->radius+2*markersize,torus->tube_radius+2*markersize};
    draw_markers(pos,dims,markersize);  
  }
}
   
     
   
void InteractableGlKinematicBody::draw_interactable_markers(boost::shared_ptr<urdf::Geometry> &_link_shape,const LinkFrameStruct &link_tf)
{
  double pos[3] = {link_tf.frame.p[0],link_tf.frame.p[1],link_tf.frame.p[2]};

  double markersize = 0.15;
  int type = _link_shape->type ;
  enum {SPHERE, BOX, CYLINDER, MESH}; 
  if (type == SPHERE)
  {
    boost::shared_ptr<urdf::Sphere> sphere(boost::shared_dynamic_cast<urdf::Sphere>(_link_shape));	
    double radius = sphere->radius;
    double dims[3] = {radius+2*markersize,radius+2*markersize,radius+2*markersize};
    draw_markers(pos,dims,markersize);  
  }
  else if  (type == BOX)
  {
    boost::shared_ptr<urdf::Box> box(boost::shared_dynamic_cast<urdf::Box>(_link_shape));
    double dims[3] = {0.5*box->dim.x+2*markersize,0.5*box->dim.y+2*markersize,0.5*box->dim.z+2*markersize};
    draw_markers(pos,dims,markersize);
  }
  else if  (type == CYLINDER)
  {
    boost::shared_ptr<urdf::Cylinder> cyl(boost::shared_dynamic_cast<urdf::Cylinder>(_link_shape));
    double dims[3] = {cyl->radius+2*markersize,cyl->radius+2*markersize,0.5*cyl->length+2*markersize};
    draw_markers(pos,dims,markersize);  
  }
  else if  (type == MESH)
  {
    std::map<std::string, MeshStruct>::const_iterator mesh_map_it;
    mesh_map_it=_mesh_map.find(link_tf.name);
    if(mesh_map_it!=_mesh_map.end()) // exists in cache
    { 
      // get the vertices for mesh_map_it->second
      double xDim = mesh_map_it->second.span_x;
      double yDim = mesh_map_it->second.span_y;
      double zDim = mesh_map_it->second.span_z;
      double xc = mesh_map_it->second.offset_x;
      double yc = mesh_map_it->second.offset_y;
      double zc = mesh_map_it->second.offset_z;
      double dims[3] = {0.5*xDim+2*markersize,0.5*yDim+2*markersize,0.5*zDim+2*markersize};
      double newpos[3] = {pos[0]+xc,pos[1]+yc,pos[2]+zc}; // meshes have a visual offset.
      draw_markers(newpos,dims,markersize);  
    }
  }

}        

void InteractableGlKinematicBody::draw_markers(double (&pos)[3], double (&dim)[3], double markersize)
{
    glEnable(GL_LINE_SMOOTH); 
   GLUquadricObj* quadric = gluNewQuadric();
   gluQuadricDrawStyle(quadric, GLU_FILL);
   gluQuadricNormals(quadric, GLU_SMOOTH);
   gluQuadricOrientation(quadric, GLU_OUTSIDE);
   
    float ORG[3] = {0.0, 0.0, 0.0};
    float XP[3] = {dim[0], 0.0, 0.0};
    float YP[3] = {0.0, dim[1], 0.0};
    float ZP[3] = {0.0, 0.0, dim[2]};


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
    double maxdim= max(dim[2],max(dim[0],dim[1]));
    glPushMatrix();
    glTranslatef(pos[0]+dim[0],pos[1],pos[2]);

    glScalef(markersize,markersize,markersize);
    glColor4f(1.0, 0.0, 0.0,alpha);
    bot_gl_draw_cube();
    glPopMatrix();
    
    glPushMatrix();
    glTranslatef(pos[0],pos[1],pos[2]);
    glRotatef(90,0,1,0);   
    gluDisk(quadric,maxdim+markersize,maxdim+2*markersize,36,1);
    glPopMatrix();
    
    glPushMatrix();
    glTranslatef(pos[0],pos[1]+dim[1],pos[2]);
    glColor4f(0.0, 1.0, 0.0,alpha);
    glScalef(markersize,markersize,markersize);
    bot_gl_draw_cube();
    glPopMatrix();
    
    glPushMatrix();
    glTranslatef(pos[0],pos[1],pos[2]);
    glRotatef(90,1,0,0);
    gluDisk(quadric,maxdim+markersize,maxdim+2*markersize,36,1);
    glPopMatrix();
    
    glPushMatrix();
    glTranslatef(pos[0],pos[1],pos[2]+dim[2]);
    glColor4f(0.0, 0.0, 1.0,alpha);
    glScalef(markersize,markersize,markersize);
    bot_gl_draw_cube();
    glPopMatrix();
    
    glPushMatrix();
    glTranslatef(pos[0],pos[1],pos[2]);
    gluDisk(quadric,maxdim+markersize,maxdim+2*markersize,36,1);
    glPopMatrix();

} 
                                       
     
