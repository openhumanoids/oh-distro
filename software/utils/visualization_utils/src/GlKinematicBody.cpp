#include "GlKinematicBody.hpp"


using namespace std;
using namespace boost;
using namespace visualization_utils;

// copy constructor
/*GlKinematicBody::GlKinematicBody( const GlKinematicBody& other )
{
   //*this = other;

  urdf::Model model; 
  if (!model.initString(other._urdf_xml_string))
  {
    cerr << "ERROR: Could not generate robot model" << endl;
  }

 _joint_names_= other._joint_names_;
 _jointlimit_min=other._jointlimit_min;
 _jointlimit_max=other._jointlimit_max;
 _current_jointpos= other._current_jointpos;
 _future_jointpos= other._future_jointpos;

  // get root link
  boost::shared_ptr<const urdf::Link> root_link=model.getRoot();
  _root_name = root_link->name;
  if(!root_link->inertial){
   cerr << "WARNING: root link has no inertia, Adding small inertia" << endl;
   model.root_link_->inertial.reset(new urdf::Inertial);
   model.root_link_->inertial->mass = 0.01;
   model.root_link_->inertial->ixx = 0.01;
   model.root_link_->inertial->iyy = 0.01;
   model.root_link_->inertial->izz = 0.01;
  }
  
  // Parse KDL tree
  KDL::Tree tree;
  //if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
  if (!kdl_parser::treeFromUrdfModel(model,tree))
  {
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl; 
    return;
  }

  _fksolver = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));

  _links_map =  model.links_;
  _mesh_model_map = other._mesh_model_map;
  _mesh_map= other._mesh_map;
  _unique_root_geometry_name=other._unique_root_geometry_name;

} */

//constructor
GlKinematicBody::GlKinematicBody(string &urdf_xml_string): initialized(false),visualize_bbox(false),is_otdf_instance(false),_T_world_body(KDL::Frame::Identity()),_T_world_body_future(KDL::Frame::Identity()), future_display_active(false), accumulate_motion_trail(false),enable_blinking(false),future_state_changing(false),isShowMeshSelected(false),isMateable(false)
{  
  //cout << "GLKinematicBody Constructor" << endl;

  // Get a urdf Model from the xml string and get all the joint names.
  urdf::Model model; 
  if (!model.initString(urdf_xml_string))
  {
    cerr << "ERROR: Could not generate robot model" << endl;
  }
//  boost::shared_ptr<urdf::Model> model_ptr(new urdf::Model); 
//  model_ptr->initString(urdf_xml_string)
  
  //enum  {UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED};
  typedef map<string, shared_ptr<urdf::Joint> > joints_mapType;
  for( joints_mapType::const_iterator it = model.joints_.begin(); it!= model.joints_.end(); it++)
  { 

      string joint_name = it->first;
      size_t found = joint_name.find("mate::start"); 
      if (found!=std::string::npos){
       _mate_start_link = it->second->parent_link_name;
       isMateable = true;
       }
      found = joint_name.find("mate::end"); 
      if (found!=std::string::npos){
       _mate_end_link = it->second->child_link_name;
        isMateable = true;
       } 
  
  
    if(it->second->type!=urdf::Joint::FIXED){ // All joints that not of the type FIXED.
      _joint_names_.push_back(it->first);
      if((it->second->type==urdf::Joint::REVOLUTE)||(it->second->type==urdf::Joint::PRISMATIC)){
        if(it->second->limits){
          _jointlimit_min.insert(make_pair(it->first, it->second->limits->lower));
          _jointlimit_max.insert(make_pair(it->first, it->second->limits->upper));
        }
      }
      _current_jointpos.insert(make_pair(it->first, 0)); //initialize 
      _future_jointpos.insert(make_pair(it->first, 0));
    }
  }
  
  // get root link
  boost::shared_ptr<const urdf::Link> root_link=model.getRoot();
  _root_name = root_link->name;
  if(!root_link->inertial){
   cerr << "WARNING: root link has no inertia, Adding small inertia" << endl;
   model.root_link_->inertial.reset(new urdf::Inertial);
   model.root_link_->inertial->mass = 0.01;
   model.root_link_->inertial->ixx = 0.01;
   model.root_link_->inertial->iyy = 0.01;
   model.root_link_->inertial->izz = 0.01;
  }
  
  // Parse KDL tree
  KDL::Tree tree;
  //if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
  if (!kdl_parser::treeFromUrdfModel(model,tree))
  {
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl; 
    return;
  }

  _fksolver = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
  
  _links_map =  model.links_;
  
  typedef map<string, shared_ptr<urdf::Link> > links_mapType;
  for(links_mapType::const_iterator it =  _links_map.begin(); it!= _links_map.end(); it++)
  { 
    if(it->second->visual) // atleast one default visual tag exists
    {
      //cout << it->first << endl;
      
      typedef map<string, shared_ptr<vector<shared_ptr<urdf::Visual> > > >  visual_groups_mapType;
      visual_groups_mapType::iterator v_grp_it = it->second->visual_groups.find("default");
      for (size_t iv = 0;iv < v_grp_it->second->size();iv++)
      {
         //int type = it->second->visual->geometry->type;
           vector<shared_ptr<urdf::Visual> > visuals = (*v_grp_it->second);
           int type = visuals[iv]->geometry->type;
    
      //enum {SPHERE, BOX, CYLINDER, MESH}; 

        if  (type == urdf::Geometry::MESH)
        {
          //shared_ptr<urdf::Mesh> mesh(shared_dynamic_cast<urdf::Mesh>(it->second->visual->geometry));
          shared_ptr<urdf::Mesh> mesh(shared_dynamic_cast<urdf::Mesh>(visuals[iv]->geometry));
          
          // READ AND STORE DISPLAY LISTS IN MEMORY ONCE
          string file_path = evalMeshFilePath(mesh->filename);
          //storing wavefront_model for different file_paths to prevent repeated creation of the same wavefront model  
          
	  MeshStruct mesh_struct;
	  shared_ptr<MeshStruct> mesh_struct_ptr;

          //typedef std::map<std::string, MeshStruct> mesh_model_map_type_;
	  typedef std::map<std::string, shared_ptr<MeshStruct> > mesh_model_map_type_;
          mesh_model_map_type_::iterator mesh_model_it = _mesh_model_map.find(file_path);
          if(mesh_model_it==_mesh_model_map.end()) // does not exist
          {
            cout <<"MESH:" << file_path << endl;
            BotWavefrontModel* wavefront_model;
            wavefront_model = bot_wavefront_model_create(file_path.c_str()); 
            
            GLuint dl;
            dl = glGenLists (1);
            glNewList (dl, GL_COMPILE);
            glScalef(mesh->scale.x,mesh->scale.y,mesh->scale.z);
            //glEnable(GL_LIGHTING);
            bot_wavefront_model_gl_draw(wavefront_model);
            //glDisable(GL_LIGHTING);
            glEndList ();
           
            double minv[3];
            double maxv[3];
	    mesh_struct_ptr = shared_ptr<MeshStruct>(new MeshStruct);
            bot_wavefront_model_get_extrema(wavefront_model, minv, maxv);
            /*mesh_struct.displaylist = dl;            
            mesh_struct.span_x =mesh->scale.x*(maxv[0] - minv[0]);
            mesh_struct.span_y =mesh->scale.y*(maxv[1] - minv[1]);
            mesh_struct.span_z =mesh->scale.z*(maxv[2] - minv[2]);
            mesh_struct.offset_x = mesh->scale.x*((maxv[0] + minv[0])/2.0);
            mesh_struct.offset_y = mesh->scale.y*((maxv[1] + minv[1])/2.0);
            mesh_struct.offset_z = mesh->scale.z*((maxv[2] + minv[2])/2.0);*/
	    mesh_struct_ptr->displaylist = dl;            
            mesh_struct_ptr->span_x =mesh->scale.x*(maxv[0] - minv[0]);
            mesh_struct_ptr->span_y =mesh->scale.y*(maxv[1] - minv[1]);
            mesh_struct_ptr->span_z =mesh->scale.z*(maxv[2] - minv[2]);
            mesh_struct_ptr->offset_x = mesh->scale.x*((maxv[0] + minv[0])/2.0);
            mesh_struct_ptr->offset_y = mesh->scale.y*((maxv[1] + minv[1])/2.0);
            mesh_struct_ptr->offset_z = mesh->scale.z*((maxv[2] + minv[2])/2.0);
            bot_wavefront_model_destroy(wavefront_model);
            // remember store in memory
            //_mesh_model_map.insert(make_pair(file_path, mesh_struct));

            _mesh_model_map.insert(make_pair(file_path, mesh_struct_ptr));
          }
          else
          {
          //recall from memory if the a wavefront model for a file path exists.
           //mesh_struct = mesh_model_it->second;
	   mesh_struct_ptr = mesh_model_it->second;
          }
          
          std::stringstream oss;
          oss << it->first << "_"<< iv; 
          string unique_geometry_name = oss.str();

            
         // populate data structures
         //typedef std::map<std::string, MeshStruct> mesh_map_type_;
 	 typedef std::map<std::string, shared_ptr<MeshStruct> > mesh_map_type_;
          mesh_map_type_::iterator mesh_map_it = _mesh_map.find(unique_geometry_name);
          if(mesh_map_it==_mesh_map.end()){
            //_mesh_map.insert(make_pair(unique_geometry_name, mesh_struct));
	    _mesh_map.insert(make_pair(unique_geometry_name, mesh_struct_ptr));
 	 }

        }//end  if  (type == MESH)
        
                  
       if(it->first==_root_name){
           std::stringstream oss;
            oss << it->first << "_"<< iv; 
          _unique_root_geometry_name = oss.str();
       }
      
      }// end for all visuals in a visual group
      
    } // end if(it->second->visual)
  } // end for

}

GlKinematicBody::GlKinematicBody(boost::shared_ptr<otdf::ModelInterface> otdf_instance): initialized(false),visualize_bbox(false),is_otdf_instance(true),_T_world_body(KDL::Frame::Identity()),_T_world_body_future(KDL::Frame::Identity()), future_display_active(false),accumulate_motion_trail(false),enable_blinking(false),future_state_changing(false), isShowMeshSelected(false),isMateable(false)
{  
 //re_init(otdf_instance);
 set_state(otdf_instance); //calls re_init inside
}


//destructor
GlKinematicBody::~GlKinematicBody()
{

}

// code reuse
//GlKinematicBody::init_urdf(boost::shared_ptr<urdf::Model> urdf_model_ptr)
//{


//}

//===============================================================================================
// UPDATE METHODS
//
// ========================================================================= 


void GlKinematicBody::re_init(boost::shared_ptr<otdf::ModelInterface> otdf_instance)
{  

  _fksolver.reset();
  _otdf_links_map.clear();
  _mesh_map.clear(); 
  
  
 //enum  {UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED};
  typedef map<string, shared_ptr<otdf::Joint> > joints_mapType;
  for( joints_mapType::const_iterator it = otdf_instance->joints_.begin(); it!= otdf_instance->joints_.end(); it++)
  { 
  
    string joint_name = it->first;
    size_t found = joint_name.find("mate::start"); 
    if (found!=std::string::npos){
     _mate_start_link = it->second->parent_link_name;  
      isMateable = true;
     }
    found = joint_name.find("mate::end"); 
    if (found!=std::string::npos){
     _mate_end_link = it->second->child_link_name;  
      isMateable = true;  
     } 
    
    if(it->second->type!=otdf::Joint::FIXED){ // All joints that not of the type FIXED.
      _joint_names_.push_back(it->first);
      if((it->second->type==otdf::Joint::REVOLUTE)||(it->second->type==otdf::Joint::PRISMATIC)){
        if(it->second->limits){
          _jointlimit_min.insert(make_pair(it->first, it->second->limits->lower));
          _jointlimit_max.insert(make_pair(it->first, it->second->limits->upper));
        }
      }
    }
  }  
  typedef std::map<std::string,boost::shared_ptr<otdf::Joint_pattern> > jp_mapType;
  for (jp_mapType::iterator jp_it = otdf_instance->joint_patterns_.begin();jp_it != otdf_instance->joint_patterns_.end(); jp_it++)
  {
    // for all joints in joint pattern.
    for (unsigned int i=0; i < jp_it->second->joint_set.size(); i++)
    {
        if(jp_it->second->joint_set[i]->type!= otdf::Joint::FIXED) { // All joints that not of the type FIXED.
        _joint_names_.push_back(jp_it->second->joint_set[i]->name);
        if((jp_it->second->joint_set[i]->type==otdf::Joint::REVOLUTE)||(jp_it->second->joint_set[i]->type==otdf::Joint::PRISMATIC)){
          if(jp_it->second->joint_set[i]->limits){
            _jointlimit_min.insert(make_pair(jp_it->second->joint_set[i]->name, jp_it->second->joint_set[i]->limits->lower));
            _jointlimit_max.insert(make_pair(jp_it->second->joint_set[i]->name, jp_it->second->joint_set[i]->limits->upper));
          }
        }
      } // end if
       
    } // end for all joints in jp
  }// for all joint patterns
  
  
  boost::shared_ptr<const otdf::BaseEntity> root_link=otdf_instance->getRoot();
  _root_name = root_link->name;
  std::string _urdf_xml_string = otdf::convertObjectInstanceToCompliantURDFstring(otdf_instance);
   
  // Parse KDL tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
  {
    std::cerr << "ERROR: Failed to extract kdl tree from "  << otdf_instance->name_ << " otdf object instance "<< std::endl; 
  }

  _fksolver = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
  

  _otdf_links_map =  otdf_instance->links_; // static links
      
  //Have to handle link_patterns separately  
  typedef std::map<std::string,boost::shared_ptr<otdf::Link_pattern> > lp_mapType;
  for (lp_mapType::iterator lp_it = otdf_instance->link_patterns_.begin();lp_it != otdf_instance->link_patterns_.end(); lp_it++)
  {
    // for all joints in joint pattern.
    for (unsigned int i=0; i < lp_it->second->link_set.size(); i++)
    {
      _otdf_links_map.insert(std::make_pair(lp_it->second->link_set[i]->name,lp_it->second->link_set[i]));       
    } // end for all links in lp
  }// for all link patterns

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
      
         
        //int type = it->second->visual->geometry->type;
        vector<shared_ptr<otdf::Visual> > visuals = (*v_grp_it->second);
        int type = visuals[iv]->geometry->type;
           
        //enum {SPHERE, BOX, CYLINDER, MESH};
        if  (type == otdf::Geometry::MESH)
        {
           shared_ptr<otdf::Mesh> mesh(shared_dynamic_cast<otdf::Mesh>(visuals[iv]->geometry));

          // READ AND STORE DISPLAY LISTS IN MEMORY ONCE
          string file_path = evalMeshFilePath(mesh->filename);
                
          MeshStruct mesh_struct;
	  shared_ptr<MeshStruct> mesh_struct_ptr;
          //typedef std::map<std::string, MeshStruct> mesh_model_map_type_;
          typedef std::map<std::string, shared_ptr<MeshStruct> > mesh_model_map_type_;
          mesh_model_map_type_::iterator mesh_model_it = _mesh_model_map.find(file_path);
          if(mesh_model_it==_mesh_model_map.end()) // does not exist
          {
            cout <<"MESH:" << file_path << endl;
            BotWavefrontModel* wavefront_model;
            wavefront_model = bot_wavefront_model_create(file_path.c_str()); 
            
            GLuint dl = glGenLists (1);
            glNewList (dl, GL_COMPILE);
            //glEnable(GL_LIGHTING);
            glScalef(mesh->scale.x,mesh->scale.y,mesh->scale.z);
            bot_wavefront_model_gl_draw(wavefront_model);
            //glDisable(GL_LIGHTING);
            glEndList ();
            double minv[3];
            double maxv[3];
	    mesh_struct_ptr = shared_ptr<MeshStruct>(new MeshStruct);
            bot_wavefront_model_get_extrema(wavefront_model, minv, maxv);

           /*mesh_struct.displaylist = dl;            
            mesh_struct.span_x =mesh->scale.x*(maxv[0] - minv[0]);
            mesh_struct.span_y =mesh->scale.y*(maxv[1] - minv[1]);
            mesh_struct.span_z =mesh->scale.z*(maxv[2] - minv[2]);
            mesh_struct.offset_x = mesh->scale.x*((maxv[0] + minv[0])/2.0);
            mesh_struct.offset_y = mesh->scale.y*((maxv[1] + minv[1])/2.0);
            mesh_struct.offset_z = mesh->scale.z*((maxv[2] + minv[2])/2.0);*/
	    mesh_struct_ptr->displaylist = dl;            
            mesh_struct_ptr->span_x =mesh->scale.x*(maxv[0] - minv[0]);
            mesh_struct_ptr->span_y =mesh->scale.y*(maxv[1] - minv[1]);
            mesh_struct_ptr->span_z =mesh->scale.z*(maxv[2] - minv[2]);
	    // offset with respect to geometric center.
            mesh_struct_ptr->offset_x = mesh->scale.x*((maxv[0] + minv[0])/2.0);
            mesh_struct_ptr->offset_y = mesh->scale.y*((maxv[1] + minv[1])/2.0);
            mesh_struct_ptr->offset_z = mesh->scale.z*((maxv[2] + minv[2])/2.0);

            bot_wavefront_model_destroy(wavefront_model);
            // remember store in memory
            //_mesh_model_map.insert(make_pair(file_path, mesh_struct));
           _mesh_model_map.insert(make_pair(file_path, mesh_struct_ptr));
          }
          else
          {
          //recall from memory if the a wavefront model for a file path exists.
           //mesh_struct = mesh_model_it->second;
	   mesh_struct_ptr = mesh_model_it->second;
          }
            
          std::stringstream oss;
          oss << it->first << "_"<< iv; 
          string unique_geometry_name = oss.str();

            
         // populate data structures
          //typedef std::map<std::string, MeshStruct> mesh_map_type_;
 	  typedef std::map<std::string, shared_ptr<MeshStruct> > mesh_map_type_;
          mesh_map_type_::iterator mesh_map_it = _mesh_map.find(unique_geometry_name);
          if(mesh_map_it==_mesh_map.end()){
            //_mesh_map.insert(make_pair(unique_geometry_name, mesh_struct));   
	    _mesh_map.insert(make_pair(unique_geometry_name, mesh_struct_ptr));
	   } 

        }//end  if  (type == MESH)
        
                  
       if(it->first==_root_name){
           std::stringstream oss;
            oss << it->first << "_"<< iv; 
          _unique_root_geometry_name = oss.str();
       }

            
      }// end for visual_groups

       
    } // end if(it->second->visual)
  } // end for
    
}

void GlKinematicBody::set_state(boost::shared_ptr<otdf::ModelInterface> otdf_instance)
{
    // if future display is active
    // retain future_display_flag
    // and motion traj

    re_init(otdf_instance); // for otdf the morphology can change, so we must reinitialize

    std::map<std::string, double> jointpos_in;
    //enum  {UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED};
  
   typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
   for (joints_mapType::iterator joint = otdf_instance->joints_.begin();joint != otdf_instance->joints_.end(); joint++)
   {
     if(joint->second->type!= otdf::Joint::FIXED) { // All joints that not of the type FIXED.
          double dof_current_pos = 0; // TODO: need object's initial dof state from fitting
          double pos, vel; 
          otdf_instance->getJointState(joint->first, pos, vel); 
          dof_current_pos = pos;
          jointpos_in.insert(make_pair(joint->first, dof_current_pos));
       }
   }
    //Have to handle joint_patterns separately   
    // DoF of all joints in joint patterns.
    typedef std::map<std::string,boost::shared_ptr<otdf::Joint_pattern> > jp_mapType;
    for (jp_mapType::iterator jp_it = otdf_instance->joint_patterns_.begin();jp_it != otdf_instance->joint_patterns_.end(); jp_it++)
    {
      // for all joints in joint pattern.
      for (unsigned int i=0; i < jp_it->second->joint_set.size(); i++)
      {

        if(jp_it->second->joint_set[i]->type!= otdf::Joint::FIXED) { // All joints that not of the type FIXED.
            double dof_current_pos = 0; //TODO: need object's initial dof state from fitting
            jointpos_in.insert(make_pair(jp_it->second->joint_set[i]->name, dof_current_pos)); 
        } // end if
         
      } // end for all joints in jp
    }// for all joint patterns
    
    // should also reset links_map
    _otdf_links_map =  otdf_instance->links_; // static links
      
    //Have to handle link_patterns separately  
    typedef std::map<std::string,boost::shared_ptr<otdf::Link_pattern> > lp_mapType;
    for (lp_mapType::iterator lp_it = otdf_instance->link_patterns_.begin();lp_it != otdf_instance->link_patterns_.end(); lp_it++)
    {
      // for all joints in joint pattern.
      for (unsigned int i=0; i < lp_it->second->link_set.size(); i++)
      {
        _otdf_links_map.insert(std::make_pair(lp_it->second->link_set[i]->name,lp_it->second->link_set[i]));       
      } // end for all links in lp
    }// for all link patterns
   
    KDL::Frame T_world_body;
    T_world_body.p[0]= otdf_instance->getParam("x");
    T_world_body.p[1]= otdf_instance->getParam("y");
    T_world_body.p[2]= otdf_instance->getParam("z");
    T_world_body.M =  KDL::Rotation::RPY(otdf_instance->getParam("roll"),
                                         otdf_instance->getParam("pitch"),
                                         otdf_instance->getParam("yaw"));
   
   update_motion_trail(_T_world_body,T_world_body); // update motion trail if it exists       
                                   
   _T_world_body  = T_world_body; 
                    

  _current_jointpos.clear();
  _current_jointpos = jointpos_in;
  run_fk_and_update_otdf_link_shapes_and_tfs(_current_jointpos,_T_world_body,false);

  if(future_display_active)
    run_fk_and_update_otdf_link_shapes_and_tfs(_future_jointpos,_T_world_body_future,true);
  else{
    _T_world_body_future = _T_world_body; 
    _future_jointpos =  _current_jointpos;
  } 
}//end GlKinematicBody::set_state(boost::shared_ptr<otdf::ModelInterface> otdf_instance)

// ========================================================================= 
void GlKinematicBody::set_state(const drc::robot_state_t &msg)
{

  /*std::map<std::string, double> jointpos_in;
  for (uint i=0; i< (uint) msg.num_joints; i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(msg.joint_name[i], msg.joint_position[i])); 
  _current_jointpos.clear();
  _current_jointpos = jointpos_in;  */
  
  // dont clear old just update
   map<string, double>::iterator joint_it;
    for (uint i=0; i< (uint) msg.num_joints; i++){
       joint_it = _current_jointpos.find(msg.joint_name[i]);
       if(joint_it!=_current_jointpos.end())
          joint_it->second = msg.joint_position[i];
    }
    
  //TODO: STORE previous jointpos_in and T_world_body as private members?
  //InteractableGLKinematicBody will provide an method for interactive adjustment.  
    
  KDL::Frame T_world_body;
  T_world_body.p[0]= msg.origin_position.translation.x;
  T_world_body.p[1]= msg.origin_position.translation.y;
  T_world_body.p[2]= msg.origin_position.translation.z;		    
  T_world_body.M =  KDL::Rotation::Quaternion(msg.origin_position.rotation.x, msg.origin_position.rotation.y, msg.origin_position.rotation.z, msg.origin_position.rotation.w);
  
   update_motion_trail(_T_world_body,T_world_body); // update motion trail if it exists  
  _T_world_body  = T_world_body;  

  
  run_fk_and_update_urdf_link_shapes_and_tfs(_current_jointpos,_T_world_body,false);
  if(future_display_active)
    run_fk_and_update_urdf_link_shapes_and_tfs(_future_jointpos,_T_world_body_future,true);
  else{
    _T_world_body_future = _T_world_body; 
    _future_jointpos =  _current_jointpos;
  }  
  
}//end GlKinematicBody::set_state(const drc::robot_state_t &msg)

void GlKinematicBody::set_state(const KDL::Frame &T_world_body, const drc::joint_angles_t &msg)
{
   update_motion_trail(_T_world_body,T_world_body); // update motion trail if it exists  
  _T_world_body  = T_world_body;

  std::map<std::string, double> jointpos_in;
  for (uint i=0; i< (uint) msg.num_joints; i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(msg.joint_name[i], msg.joint_position[i]));
    
  _current_jointpos.clear();
  _current_jointpos = jointpos_in;   

  if(!is_otdf_instance)
    run_fk_and_update_urdf_link_shapes_and_tfs(_current_jointpos,_T_world_body,false);
  else
    run_fk_and_update_otdf_link_shapes_and_tfs(_current_jointpos,_T_world_body,false);
    
  if(future_display_active){
    if(!is_otdf_instance)
      run_fk_and_update_urdf_link_shapes_and_tfs(_future_jointpos,_T_world_body_future,true);
    else
      run_fk_and_update_otdf_link_shapes_and_tfs(_future_jointpos,_T_world_body_future,true);
   }
  else{
    _T_world_body_future = _T_world_body; 
    _future_jointpos =  _current_jointpos;
  }
  
  
}//end void GlKinematicBody::set_state(const KDL::Frame &T_world_body, const drc::joint_angles_t &msg)



void GlKinematicBody::set_state(const KDL::Frame &T_world_body, std::map<std::string, double> &jointpos_in)
{
   update_motion_trail(_T_world_body,T_world_body); // update motion trail if it exists  
  _T_world_body  = T_world_body;

  _current_jointpos.clear();
  _current_jointpos = jointpos_in;   


  if(!is_otdf_instance)
    run_fk_and_update_urdf_link_shapes_and_tfs(_current_jointpos,_T_world_body,false);
  else
    run_fk_and_update_otdf_link_shapes_and_tfs(_current_jointpos,_T_world_body,false);
  
  if(future_display_active){
    if(!is_otdf_instance)
      run_fk_and_update_urdf_link_shapes_and_tfs(_future_jointpos,_T_world_body_future,true);
    else
      run_fk_and_update_otdf_link_shapes_and_tfs(_future_jointpos,_T_world_body_future,true);
   }
  else{
    _T_world_body_future = _T_world_body; 
    _future_jointpos =  _current_jointpos;
  }
  
  
}//end void GlKinematicBody::set_state(const KDL::Frame &, const std::map<std::string, double> & )

void GlKinematicBody::set_future_state(const drc::robot_state_t &msg)
{

  // dont clear old just update
   map<string, double>::iterator joint_it;
    for (uint i=0; i< (uint) msg.num_joints; i++){
       joint_it = _future_jointpos.find(msg.joint_name[i]);
       if(joint_it!=_future_jointpos.end())
          joint_it->second = msg.joint_position[i];
    }
    
  //TODO: STORE previous jointpos_in and T_world_body as private members?
  //InteractableGLKinematicBody will provide an method for interactive adjustment.  
    
  KDL::Frame T_world_body_future;
  T_world_body_future.p[0]= msg.origin_position.translation.x;
  T_world_body_future.p[1]= msg.origin_position.translation.y;
  T_world_body_future.p[2]= msg.origin_position.translation.z;		    
  T_world_body_future.M =  KDL::Rotation::Quaternion(msg.origin_position.rotation.x, msg.origin_position.rotation.y, msg.origin_position.rotation.z, msg.origin_position.rotation.w);
  _T_world_body_future = T_world_body_future;
  
 if(!future_display_active){
   future_display_active = true;
  }
//  std::map<std::string, double> jointpos_in;
//  for (uint i=0; i< (uint) msg.num_joints; i++) //cast to uint to suppress compiler warning
//    jointpos_in.insert(make_pair(msg.joint_name[i], msg.joint_position[i]));     

    if(is_otdf_instance)
        run_fk_and_update_otdf_link_shapes_and_tfs(_future_jointpos,T_world_body_future,future_display_active);
    else
        run_fk_and_update_urdf_link_shapes_and_tfs(_future_jointpos,T_world_body_future,future_display_active);
  
}//end GlKinematicBody::set_state(const drc::robot_state_t &msg)

   
// space and time visualization.
void GlKinematicBody::set_future_state(const KDL::Frame &T_world_body_future, std::map<std::string, double> &jointpos_in)
{

 if(!future_display_active){
   future_display_active = true;
  }
//  std::map<std::string, double> jointpos_in;
//  for (uint i=0; i< (uint) msg.num_joints; i++) //cast to uint to suppress compiler warning
//    jointpos_in.insert(make_pair(msg.joint_name[i], msg.joint_position[i]));     
    _future_jointpos.clear();
    _future_jointpos = jointpos_in;
    _T_world_body_future = T_world_body_future;
    if(is_otdf_instance)
        run_fk_and_update_otdf_link_shapes_and_tfs(_future_jointpos,T_world_body_future,future_display_active);
    else
        run_fk_and_update_urdf_link_shapes_and_tfs(_future_jointpos,T_world_body_future,future_display_active);

}//end void GlKinematicBody::set_future_state

// ========================================================================= 

// Shared code
void GlKinematicBody::run_fk_and_update_urdf_link_shapes_and_tfs(std::map<std::string, double> &jointpos_in, const KDL::Frame &T_world_body, bool update_future_frame)
{

   //clear stored data
   if (!update_future_frame){
    _link_names.clear();
    _link_geometry_names.clear();
    _link_tfs.clear();
    _link_geometry_tfs.clear();
    _link_shapes.clear();

    _joint_names.clear();
    _joint_tfs.clear();
    }
    
    // enforce joint limits
    typedef map<string, double > jointpos_mapType;      
    for( jointpos_mapType::iterator it =  jointpos_in.begin(); it!=jointpos_in.end(); it++)
    { 
       map<string,double>::const_iterator joint_it;
       joint_it=_jointlimit_min.find(it->first);
       if(joint_it!=_jointlimit_min.end())
        it->second = std::max(joint_it->second,it->second);
       joint_it=_jointlimit_max.find(it->first);
       if(joint_it!=_jointlimit_max.end())
        it->second = std::min(joint_it->second,it->second); 
    }
    
    std::map<std::string, KDL::Frame > cartpos_out;
    

    // Calculate forward position kinematics
    bool kinematics_status;
    bool flatten_tree=true; // determines the absolute transforms with respect to robot origin. 
                    //Otherwise returns relative transforms between joints. 

    kinematics_status = _fksolver->JntToCart(jointpos_in,cartpos_out,flatten_tree);

    if(kinematics_status>=0){
      // cout << "Success!" <<endl;
    }else{
      cerr << "Error: could not calculate forward kinematics!" <<endl;
      return;
    }


    typedef map<string, shared_ptr<urdf::Link> > links_mapType;      
    for( links_mapType::const_iterator it =  _links_map.begin(); it!= _links_map.end(); it++)
    {  		
    
       map<string, KDL::Frame>::const_iterator transform_it;
       transform_it=cartpos_out.find(it->first);
    
    // iterate over add all child joints and store joint origins and axes that are not of type fixed.
    //  std::vector<boost::shared_ptr<Joint> > child_joints;
      for (size_t i = 0; i < it->second->child_joints.size();i++)
      {
        if(it->second->child_joints[i]->type!=urdf::Joint::FIXED)
        {
          KDL::Frame T_parentlink_jointorigin;
          T_parentlink_jointorigin.p[0]= it->second->child_joints[i]->parent_to_joint_origin_transform.position.x;
          T_parentlink_jointorigin.p[1]= it->second->child_joints[i]->parent_to_joint_origin_transform.position.y;
          T_parentlink_jointorigin.p[2]= it->second->child_joints[i]->parent_to_joint_origin_transform.position.z;
          double x,y,z,w;       
          x= it->second->child_joints[i]->parent_to_joint_origin_transform.rotation.x;
          y= it->second->child_joints[i]->parent_to_joint_origin_transform.rotation.y;
          z= it->second->child_joints[i]->parent_to_joint_origin_transform.rotation.z; 
          w= it->second->child_joints[i]->parent_to_joint_origin_transform.rotation.w; 
          T_parentlink_jointorigin.M = KDL::Rotation::Quaternion(x,y,z,w);

          if(transform_it!=cartpos_out.end())// fk cart pos exists
          {
            KDL::Frame T_body_parentlink =  transform_it->second;
            KDL::Frame T_world_jointorigin =T_world_body*T_body_parentlink*T_parentlink_jointorigin;
            JointFrameStruct jointInfo;
            jointInfo.name=it->second->child_joints[i]->name;
            jointInfo.parent_link_name=it->second->child_joints[i]->parent_link_name;
            jointInfo.child_link_name=it->second->child_joints[i]->child_link_name;
            if (!update_future_frame){
              jointInfo.frame=T_world_jointorigin;
              jointInfo.axis[0]=it->second->child_joints[i]->axis.x;
              jointInfo.axis[1]=it->second->child_joints[i]->axis.y;
              jointInfo.axis[2]=it->second->child_joints[i]->axis.z;
              jointInfo.axis= T_world_body.M*T_body_parentlink.M*jointInfo.axis;
              jointInfo.axis.Normalize();
              jointInfo.type=it->second->child_joints[i]->type;
              
              //store
              _joint_names.push_back(jointInfo.name);
              _joint_tfs.push_back(jointInfo);
              }
              else{ // update future frame
                std::vector<std::string>::const_iterator found;
                found = std::find (_joint_names.begin(), _joint_names.end(), jointInfo.name);
                if (found != _joint_names.end()) {
                    unsigned int index = found - _joint_names.begin();
                    _joint_tfs[index].future_frame = T_world_jointorigin;
                    jointInfo.future_axis[0]=it->second->child_joints[i]->axis.x;
                    jointInfo.future_axis[1]=it->second->child_joints[i]->axis.y;
                    jointInfo.future_axis[2]=it->second->child_joints[i]->axis.z;
                    _joint_tfs[index].future_axis= T_world_body.M*T_body_parentlink.M*jointInfo.future_axis;
                    _joint_tfs[index].future_axis.Normalize();
                }
              } // end if  (!update_future_frame)     
            
          }
          else if(it->first ==_root_name){
            KDL::Frame T_body_parentlink =  KDL::Frame::Identity();
            KDL::Frame T_world_jointorigin =T_world_body*T_body_parentlink*T_parentlink_jointorigin;

            JointFrameStruct jointInfo;
            jointInfo.name=it->second->child_joints[i]->name;
            jointInfo.parent_link_name=it->second->child_joints[i]->parent_link_name;
            jointInfo.child_link_name=it->second->child_joints[i]->child_link_name;
            if (!update_future_frame){
              jointInfo.frame=T_world_jointorigin;
              jointInfo.axis[0]=it->second->child_joints[i]->axis.x;
              jointInfo.axis[1]=it->second->child_joints[i]->axis.y;
              jointInfo.axis[2]=it->second->child_joints[i]->axis.z;
              jointInfo.axis= T_world_body.M*T_body_parentlink.M*jointInfo.axis;
              jointInfo.axis.Normalize();
              jointInfo.type=it->second->child_joints[i]->type;
              
              //store
              _joint_names.push_back(jointInfo.name);
              _joint_tfs.push_back(jointInfo);
              }
              else{ // update future frame
                std::vector<std::string>::const_iterator found;
                found = std::find (_joint_names.begin(), _joint_names.end(), jointInfo.name);
                if (found != _joint_names.end()) {
                    unsigned int index = found - _joint_names.begin();
                    _joint_tfs[index].future_frame = T_world_jointorigin;
                    jointInfo.future_axis[0]=it->second->child_joints[i]->axis.x;
                    jointInfo.future_axis[1]=it->second->child_joints[i]->axis.y;
                    jointInfo.future_axis[2]=it->second->child_joints[i]->axis.z;
                    _joint_tfs[index].future_axis= T_world_body.M*T_body_parentlink.M*jointInfo.future_axis;
                    _joint_tfs[index].future_axis.Normalize();
                }
              } // end if  (!update_future_frame)    
          }
           
        }// end if (it->second->child_joints[i]->type!=urdf::Joint::FIXED)
      }// end for all child joints
    

      if(it->second->visual)
      {  

        typedef map<string, shared_ptr<vector<shared_ptr<urdf::Visual> > > >  visual_groups_mapType;
        visual_groups_mapType::iterator v_grp_it = it->second->visual_groups.find("default");
        for (size_t iv = 0;iv < v_grp_it->second->size();iv++)
        {
          vector<shared_ptr<urdf::Visual> > visuals = (*v_grp_it->second);
          
          urdf::Pose visual_origin = visuals[iv]->origin;
          KDL::Frame T_parentjoint_visual, T_body_parentjoint, T_body_visual, T_world_visual, T_world_parentjoint;


          //map<string, KDL::Frame>::const_iterator transform_it;
          //transform_it=cartpos_out.find(it->first);

          // usually find fails if base_link has a visual element.
          // Kdl based FK ignores the root link which must be set to body pose
          // manually.
          if(transform_it!=cartpos_out.end())// fk cart pos exists
          {

            T_body_parentjoint= transform_it->second;



            T_parentjoint_visual.p[0]=visual_origin.position.x;
            T_parentjoint_visual.p[1]=visual_origin.position.y;
            T_parentjoint_visual.p[2]=visual_origin.position.z;
            T_parentjoint_visual.M =  KDL::Rotation::Quaternion(visual_origin.rotation.x, visual_origin.rotation.y, visual_origin.rotation.z, visual_origin.rotation.w);

            T_body_visual  = T_body_parentjoint*T_parentjoint_visual;

            T_world_visual = T_world_body*T_body_visual;
            T_world_parentjoint = T_world_body*T_body_parentjoint;
             //T_world_visual  = T_world_camera*T_camera_body*T_body_visual;
             
             
            std::stringstream oss;
            oss << it->first << "_"<< iv; 
            string unique_geometry_name = oss.str();

            LinkFrameStruct geometry_state,link_state;	    
            if (!update_future_frame){
              geometry_state.name = unique_geometry_name;
              geometry_state.frame = T_world_visual;
              link_state.name = it->first;
              link_state.frame = T_world_parentjoint;
            }

              shared_ptr<urdf::Geometry> geom =  visuals[iv]->geometry;
              //---store
              if (!update_future_frame){
                /*std::vector<std::string>::const_iterator found;
                found = std::find (_link_names.begin(), _link_names.end(), it->first);
                if (found == _link_names.end()) { // if doesnt exist then add*/
                  _link_names.push_back(it->first);  
                  _link_tfs.push_back(link_state);;
                //}
                _link_geometry_names.push_back(unique_geometry_name);
                _link_shapes.push_back(geom);
                _link_geometry_tfs.push_back(geometry_state);
              }
              else{ // update future frame
                std::vector<std::string>::const_iterator found;
                found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), unique_geometry_name);
                if (found != _link_geometry_names.end()) {
                    unsigned int index = found - _link_geometry_names.begin();
                    _link_tfs[index].future_frame = T_world_parentjoint;
                    _link_geometry_tfs[index].future_frame = T_world_visual; 
                }
              }


            //cout << "translation  : " << endl;
            //cout << "\t .x  : " << state.tf.translation.x << endl;
            //cout << "\t .y  : " << state.tf.translation.y << endl;
            //cout << "\t .z  : " << state.tf.translation.z << endl;
            //cout << "quaternion" << endl;
            //cout << "\t .x  : " << state.tf.rotation.x << endl;
            //cout << "\t .y  : " << state.tf.rotation.y << endl;
            //cout << "\t .z  : " << state.tf.rotation.z << endl;
            //cout << "\t .w  : " << state.tf.rotation.w << endl;   
            //cout << "\n"<< endl;

          }//if(transform_it!=cartpos_out.end())
          else // root link has visual element (but is not part of fk output)
          {
              urdf::Pose visual_origin = visuals[iv]->origin;
              T_body_visual.p[0]=visual_origin.position.x;
              T_body_visual.p[1]=visual_origin.position.y;
              T_body_visual.p[2]=visual_origin.position.z;
              T_body_visual.M =  KDL::Rotation::Quaternion(visual_origin.rotation.x, visual_origin.rotation.y, visual_origin.rotation.z, visual_origin.rotation.w);


              T_world_visual = T_world_body*T_body_visual;
              T_world_parentjoint = T_world_body;
              
              std::stringstream oss;
              oss << it->first << "_"<< iv; 
              string unique_geometry_name = oss.str();
              shared_ptr<urdf::Geometry> geom =  visuals[iv]->geometry;
              LinkFrameStruct geometry_state,link_state;	
              if (!update_future_frame){
                geometry_state.name = unique_geometry_name;
                geometry_state.frame = T_world_visual;
                link_state.name = it->first;
                link_state.frame = T_world_parentjoint;
              }

                //---store
                 if (!update_future_frame){
                 /* std::vector<std::string>::const_iterator found;
                  found = std::find (_link_names.begin(), _link_names.end(), it->first);
                  if (found == _link_names.end()) { // if doesnt exist then add*/
                    _link_names.push_back(it->first);  
                    _link_tfs.push_back(link_state);;
                  //}
                  _link_geometry_names.push_back(unique_geometry_name);  
                  _link_shapes.push_back(geom);
                  _link_geometry_tfs.push_back(geometry_state);
                }
                else{// update future frame
                  std::vector<std::string>::const_iterator found;
                  found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), unique_geometry_name);
                  if (found != _link_geometry_names.end()) {
                    unsigned int index = found - _link_geometry_names.begin();
                    _link_tfs[index].future_frame = T_world_parentjoint;
                    _link_geometry_tfs[index].future_frame = T_world_visual;
                  }
                }

    
         } //end if(transform_it!=cartpos_out.end())
           
        }// end for visual groups   

      }//if(it->second->visual)

    }//end for


}// end run_fk_and_update_urdf_link_shapes_and_tfs

// ========================================================================= 
void GlKinematicBody::run_fk_and_update_otdf_link_shapes_and_tfs(std::map<std::string, double> &jointpos_in,const KDL::Frame &T_world_body, bool update_future_frame)
{

   //clear stored data
   if (!update_future_frame){
    _link_names.clear();
    _link_geometry_names.clear();
    _link_tfs.clear();
    _link_geometry_tfs.clear();
    _otdf_link_shapes.clear();
    
    _joint_names.clear();
    _joint_tfs.clear();    
    }
    
    
    // enforce joint limits
    typedef map<string, double > jointpos_mapType;      
    for( jointpos_mapType::iterator it =  jointpos_in.begin(); it!=jointpos_in.end(); it++)
    { 
       map<string,double>::const_iterator joint_it;
       joint_it=_jointlimit_min.find(it->first);
       if(joint_it!=_jointlimit_min.end())
        it->second = std::max(joint_it->second,it->second);
       joint_it=_jointlimit_max.find(it->first);
       if(joint_it!=_jointlimit_max.end())
        it->second = std::min(joint_it->second,it->second); 
    }    

    std::map<std::string, KDL::Frame > cartpos_out;

    // Calculate forward position kinematics
    bool kinematics_status;
    bool flatten_tree=true; // determines the absolute transforms with respect to robot origin. 
                    //Otherwise returns relative transforms between joints. 

    kinematics_status = _fksolver->JntToCart(jointpos_in,cartpos_out,flatten_tree);

    if(kinematics_status>=0){
      // cout << "Success!" <<endl;
    }else{
      cerr << "Error: could not calculate forward kinematics!" <<endl;
      return;
    }

    typedef map<string, shared_ptr<otdf::Link> > links_mapType;      
    for( links_mapType::const_iterator it =  _otdf_links_map.begin(); it!= _otdf_links_map.end(); it++)
    {  		
      map<string, KDL::Frame>::const_iterator transform_it;
      transform_it=cartpos_out.find(it->first);

    // iterate over add all child joints and store joint origins and axes that are not of type fixed.
    //  std::vector<boost::shared_ptr<Joint> > child_joints;
      for (size_t i = 0; i < it->second->child_joints.size();i++)
      {
        if(it->second->child_joints[i]->type!=otdf::Joint::FIXED)
        {
          KDL::Frame T_parentlink_jointorigin;
          T_parentlink_jointorigin.p[0]= it->second->child_joints[i]->parent_to_joint_origin_transform.position.x;
          T_parentlink_jointorigin.p[1]= it->second->child_joints[i]->parent_to_joint_origin_transform.position.y;
          T_parentlink_jointorigin.p[2]= it->second->child_joints[i]->parent_to_joint_origin_transform.position.z;
          double x,y,z,w;       
          x= it->second->child_joints[i]->parent_to_joint_origin_transform.rotation.x;
          y= it->second->child_joints[i]->parent_to_joint_origin_transform.rotation.y;
          z= it->second->child_joints[i]->parent_to_joint_origin_transform.rotation.z;
          w= it->second->child_joints[i]->parent_to_joint_origin_transform.rotation.w;  
          T_parentlink_jointorigin.M = KDL::Rotation::Quaternion(x,y,z,w);

          if(transform_it!=cartpos_out.end())// fk cart pos exists
          {
            KDL::Frame T_body_parentlink =  transform_it->second;
            KDL::Frame T_world_jointorigin =T_world_body*T_body_parentlink*T_parentlink_jointorigin;
            
            JointFrameStruct jointInfo;
            jointInfo.name=it->second->child_joints[i]->name;
            jointInfo.parent_link_name=it->second->child_joints[i]->parent_link_name;
            jointInfo.child_link_name=it->second->child_joints[i]->child_link_name;
            if (!update_future_frame){
              jointInfo.frame=T_world_jointorigin;
              jointInfo.axis[0]=it->second->child_joints[i]->axis.x;
              jointInfo.axis[1]=it->second->child_joints[i]->axis.y;
              jointInfo.axis[2]=it->second->child_joints[i]->axis.z;       
              // axis is specified in joint frame 
             jointInfo.axis= T_world_body.M*T_body_parentlink.M*T_parentlink_jointorigin.M*jointInfo.axis;
              //or is axis specified in parent link frame
              // jointInfo.axis= T_world_body.M*T_body_parentlink.M*jointInfo.axis;   
              jointInfo.axis.Normalize();
              
              jointInfo.type=it->second->child_joints[i]->type;
              
              //store
              _joint_names.push_back(jointInfo.name);
              _joint_tfs.push_back(jointInfo);
              }
              else{ // update future frame
                std::vector<std::string>::const_iterator found;
                found = std::find (_joint_names.begin(), _joint_names.end(), jointInfo.name);
                if (found != _joint_names.end()) {
                    unsigned int index = found - _joint_names.begin();
                    _joint_tfs[index].future_frame = T_world_jointorigin;
                    jointInfo.future_axis[0]=it->second->child_joints[i]->axis.x;
                    jointInfo.future_axis[1]=it->second->child_joints[i]->axis.y;
                    jointInfo.future_axis[2]=it->second->child_joints[i]->axis.z;
                   
                   // axis is specified in parent link frame of the joint
                  _joint_tfs[index].future_axis= T_world_body.M*T_body_parentlink.M*T_parentlink_jointorigin.M*jointInfo.future_axis;
                   //or is axis specified in body frame
                   // _joint_tfs[index].future_axis= T_world_body.M*T_body_parentlink.M*jointInfo.future_axis;
                   _joint_tfs[index].future_axis.Normalize();
                    
                }
              } // end if  (!update_future_frame)        
          
            
          }
          else if(it->first ==_root_name){
            KDL::Frame T_body_parentlink =  KDL::Frame::Identity();
            KDL::Frame T_world_jointorigin =T_world_body*T_body_parentlink*T_parentlink_jointorigin;

            JointFrameStruct jointInfo;
            jointInfo.name=it->second->child_joints[i]->name;
            jointInfo.parent_link_name=it->second->child_joints[i]->parent_link_name;
            jointInfo.child_link_name=it->second->child_joints[i]->child_link_name;
            if (!update_future_frame){
              jointInfo.frame=T_world_jointorigin;
              jointInfo.axis[0]=it->second->child_joints[i]->axis.x;
              jointInfo.axis[1]=it->second->child_joints[i]->axis.y;
              jointInfo.axis[2]=it->second->child_joints[i]->axis.z;
              // axis is specified in parent link frame of the joint
              jointInfo.axis= T_world_body.M*T_body_parentlink.M*T_parentlink_jointorigin.M*jointInfo.axis;
              //or is axis specified in body frame
              //jointInfo.axis= T_world_body.M*T_body_parentlink.M*jointInfo.axis; 
              jointInfo.axis.Normalize();
              jointInfo.type=it->second->child_joints[i]->type;
              
              //store
              _joint_names.push_back(jointInfo.name);
              _joint_tfs.push_back(jointInfo);
              }
              else{ // update future frame
                std::vector<std::string>::const_iterator found;
                found = std::find (_joint_names.begin(), _joint_names.end(), jointInfo.name);
                if (found != _joint_names.end()) {
                    unsigned int index = found - _joint_names.begin();
                    _joint_tfs[index].future_frame = T_world_jointorigin;
                    jointInfo.future_axis[0]=it->second->child_joints[i]->axis.x;
                    jointInfo.future_axis[1]=it->second->child_joints[i]->axis.y;
                    jointInfo.future_axis[2]=it->second->child_joints[i]->axis.z;
                    _joint_tfs[index].future_axis= T_world_body.M*T_body_parentlink.M*T_parentlink_jointorigin.M*jointInfo.future_axis;
                    
                   //or is axis specified in body frame
                  // _joint_tfs[index].future_axis= T_world_body.M*T_body_parentlink.M*jointInfo.future_axis;
                   _joint_tfs[index].future_axis.Normalize();
                }
              } // end if  (!update_future_frame)     
          }
           
        }// end if (it->second->child_joints[i]->type!=urdf::Joint::FIXED)
      }// end for all child joints

      if(it->second->visual)
      {  
        typedef map<string, shared_ptr<vector<shared_ptr<otdf::Visual> > > >  visual_groups_mapType;
        visual_groups_mapType::iterator v_grp_it = it->second->visual_groups.find("default");

        for (size_t iv = 0;iv < v_grp_it->second->size();iv++)
        {
          vector<shared_ptr<otdf::Visual> > visuals = (*v_grp_it->second);
          
          otdf::Pose visual_origin = visuals[iv]->origin;
          KDL::Frame T_parentjoint_visual, T_body_parentjoint, T_body_visual, T_world_visual, T_world_parentjoint;

          //map<string, KDL::Frame>::const_iterator transform_it;
          //transform_it=cartpos_out.find(it->first);

          // usually find fails if base_link has a visual element.
          // Kdl based FK ignores the root link which must be set to body pose
          // manually.
          if(transform_it!=cartpos_out.end())// fk cart pos exists
          {
            T_body_parentjoint = transform_it->second;

            T_parentjoint_visual.p[0]=visual_origin.position.x;
            T_parentjoint_visual.p[1]=visual_origin.position.y;
            T_parentjoint_visual.p[2]=visual_origin.position.z;
            T_parentjoint_visual.M =  KDL::Rotation::Quaternion(visual_origin.rotation.x, visual_origin.rotation.y, visual_origin.rotation.z, visual_origin.rotation.w);

            T_body_visual  = T_body_parentjoint*T_parentjoint_visual;

            T_world_visual = T_world_body*T_body_visual;
            T_world_parentjoint = T_world_body*T_body_parentjoint;
             //T_world_visual  = T_world_camera*T_camera_body*T_body_visual;

            std::stringstream oss;
            oss << it->first << "_"<< iv; 
            string unique_geometry_name = oss.str();

            LinkFrameStruct geometry_state,link_state;	
            if (!update_future_frame){
              geometry_state.name = unique_geometry_name;
              geometry_state.frame = T_world_visual;
              link_state.name = it->first;
              link_state.frame = T_world_parentjoint;
            }

                  
            shared_ptr<otdf::Geometry> geom =  visuals[iv]->geometry;
            //---store
             if (!update_future_frame){
               /*std::vector<std::string>::const_iterator found;
                found = std::find (_link_names.begin(), _link_names.end(), it->first);
                if (found == _link_names.end()) { // if doesnt exist then add*/
                _link_names.push_back(it->first);  
                _link_tfs.push_back(link_state);
                //}
              _link_geometry_names.push_back(unique_geometry_name);   
              _otdf_link_shapes.push_back(geom);
              _link_geometry_tfs.push_back(geometry_state);
            }
            else{
              std::vector<std::string>::const_iterator found;
              found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), unique_geometry_name);
              if (found != _link_geometry_names.end()) {
                  unsigned int index = found - _link_geometry_names.begin();
                  _link_tfs[index].future_frame = T_world_parentjoint;  
                  _link_geometry_tfs[index].future_frame = T_world_visual; 
              }
            }

          }//if(transform_it!=cartpos_out.end())
          else // root link has visual element (but is not part of fk output)
          {
              otdf::Pose visual_origin = visuals[iv]->origin;
              T_body_visual.p[0]=visual_origin.position.x;
              T_body_visual.p[1]=visual_origin.position.y;
              T_body_visual.p[2]=visual_origin.position.z;
              T_body_visual.M =  KDL::Rotation::Quaternion(visual_origin.rotation.x, visual_origin.rotation.y, visual_origin.rotation.z, visual_origin.rotation.w);


              T_world_visual = T_world_body*T_body_visual;
              T_world_parentjoint = T_world_body;
              
              std::stringstream oss;
              oss << it->first << "_"<< iv; 
              string unique_geometry_name = oss.str(); 
              shared_ptr<otdf::Geometry> geom =  visuals[iv]->geometry;
              
              LinkFrameStruct geometry_state,link_state;	
              if (!update_future_frame){
                geometry_state.name = unique_geometry_name;
                geometry_state.frame = T_world_visual;
                link_state.name = it->first;
                link_state.frame = T_world_parentjoint;
              }

              //---store
               if (!update_future_frame){
               
                /*std::vector<std::string>::const_iterator found;
                found = std::find (_link_names.begin(), _link_names.end(), it->first);
                if (found == _link_names.end()) { // if doesnt exist then add*/
                _link_names.push_back(it->first);  
                _link_tfs.push_back(link_state);
                //}
                _link_geometry_names.push_back(unique_geometry_name);
                _otdf_link_shapes.push_back(geom);
               
                _link_geometry_tfs.push_back(geometry_state);
              }
              else{
                std::vector<std::string>::const_iterator found;
                found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), unique_geometry_name);
                if (found != _link_geometry_names.end()) {
                    unsigned int index = found - _link_geometry_names.begin();
                    _link_tfs[index].future_frame = T_world_parentjoint;
                    _link_geometry_tfs[index].future_frame = T_world_visual;
                }
              }

           } //end if(transform_it!=cartpos_out.end())
             
        }// end for visual groups    
      }//if(it->second->visual)
    }//end for

}// end run_fk_and_update_otdf_link_shapes_and_tfs

//===============================================================================================
// ACCESS METHODS
//

bool GlKinematicBody::get_link_geometry_frame(const std::string &link_geometry_name, KDL::Frame &T_world_link)
{
      LinkFrameStruct state;	    

     // retrieve T_world_link from store
      std::vector<std::string>::const_iterator found;
      found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), link_geometry_name);
      if (found != _link_geometry_names.end()) {
        unsigned int index = found - _link_geometry_names.begin();
        state = _link_geometry_tfs[index];  
        T_world_link= state.frame;       
        return true;
      } 
      else 
      {  
       T_world_link = KDL::Frame::Identity();
       // std::cerr << "ERROR:"<< link_geometry_name << " not found in _link_geometry_names" << std::endl;
       return false;
      }
}  


bool GlKinematicBody::get_link_geometry_future_frame(const std::string &link_geometry_name, KDL::Frame &T_world_link)
{
      LinkFrameStruct state;	    

     // retrieve T_world_link from store
      std::vector<std::string>::const_iterator found;
      found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), link_geometry_name);
      if (found != _link_geometry_names.end()) {
        unsigned int index = found - _link_geometry_names.begin();
        state = _link_geometry_tfs[index]; 
        if(future_display_active){
          T_world_link= state.future_frame;       
          return true;
        }
        else
          return false;
      } 
      else 
      {  
       T_world_link = KDL::Frame::Identity();
       std::cerr << "ERROR in GlKinematicBody::get_link_geometry_future_frame:"<< link_geometry_name << " not found in _link_geometry_names" << std::endl;
       return false;
      }
} 

bool GlKinematicBody::get_link_frame(const std::string &link_name, KDL::Frame &T_world_link)
{
      LinkFrameStruct state;	    

     // retrieve T_world_link from store
      std::vector<std::string>::const_iterator found;
      found = std::find (_link_names.begin(), _link_names.end(), link_name);
      if (found != _link_names.end()) {
        unsigned int index = found - _link_names.begin();
        state = _link_tfs[index];  
        T_world_link= state.frame; 
        return true;
      } 
      else 
      {  
       T_world_link = KDL::Frame::Identity();
       // std::cerr << "ERROR:"<< link_name << " not found in _link_names" << std::endl;
       return false;
      }
}  


bool GlKinematicBody::get_link_future_frame(const std::string &link_name, KDL::Frame &T_world_link)
{
      LinkFrameStruct state;	    

     // retrieve T_world_link from store
      std::vector<std::string>::const_iterator found;
      found = std::find (_link_names.begin(), _link_names.end(), link_name);
      if (found != _link_names.end()) {
        unsigned int index = found - _link_names.begin();
        state = _link_tfs[index]; 
        if(future_display_active){
          T_world_link= state.future_frame;       
          return true;
        }
        else
          return false;
      } 
      else 
      {  
       T_world_link = KDL::Frame::Identity();
       std::cerr << "ERROR in GlKinematicBody::get_link_future_frame:"<< link_name << " not found in _link_names" << std::endl;
       return false;
      }
}  

//bool GlKinematicBody::get_link_geometry(const std::string &link_geometry_name, boost::shared_ptr<urdf::Geometry> &link_geom)
//{
//    typedef map<string, shared_ptr<urdf::Link> > links_mapType; 
//    links_mapType::const_iterator link_it =  _links_map.find(link_geometry_name);
//    if (link_it != _links_map.end()) {
//      link_geom = link_it->second->visual->geometry; // TODO: check if this is somehow affected by visual groups
//      return true;
//    }
//    else 
//     return false; 

//}

//bool GlKinematicBody::get_link_geometry(const std::string &link_geometry_name, boost::shared_ptr<otdf::Geometry> &link_geom)
//{
//    typedef map<string, shared_ptr<otdf::Link> > links_mapType; 
//    links_mapType::const_iterator link_it =  _otdf_links_map.find(link_geometry_name);
//    if (link_it != _otdf_links_map.end()) {
//      link_geom = link_it->second->visual->geometry; // TODO: check if this is somehow affected by visual groups, only the default visual geometry is returned.
//      return true;
//    }
//    else 
//     return false; 
//}

bool GlKinematicBody::get_link_geometry(const std::string &link_geometry_name, boost::shared_ptr<urdf::Geometry> &link_geom)
{

    std::vector<std::string>::const_iterator found;
    found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), link_geometry_name);
    if (found != _link_geometry_names.end()) {
        unsigned int index = found - _link_geometry_names.begin();
        link_geom = _link_shapes[index]; 
        return true;
    }
    else 
     return false; 

}


bool GlKinematicBody::get_link_geometry(const std::string &link_geometry_name, boost::shared_ptr<otdf::Geometry> &link_geom)
{
    std::vector<std::string>::const_iterator found;
    found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), link_geometry_name);
    if (found != _link_geometry_names.end()) {
        unsigned int index = found - _link_geometry_names.begin();
        link_geom = _otdf_link_shapes[index]; 
        return true;
    }
    else 
     return false; 
}


bool GlKinematicBody::get_joint_info(const std::string &joint_name, JointFrameStruct &jointinfo_struct)
{
    std::vector<std::string>::const_iterator found;
    found = std::find (_joint_names.begin(), _joint_names.end(), joint_name);
    if (found != _joint_names.end()) {
        unsigned int index = found - _joint_names.begin();
        jointinfo_struct = _joint_tfs[index]; 
        return true;
    }
    else 
     return false; 
}

bool GlKinematicBody::get_current_joint_pos(const std::string &joint_name, double &pos)
{
    pos= 0;
    std::map<std::string, double >::const_iterator jointpos_map_it;
    jointpos_map_it = _current_jointpos.find(joint_name); 
    if(jointpos_map_it!=_current_jointpos.end()) 
    { 
      pos = jointpos_map_it->second;
      return true;
    }
    else 
     return false; 
}

bool GlKinematicBody::get_future_joint_pos(const std::string &joint_name, double &pos)
{
    pos= 0;
    std::map<std::string, double >::const_iterator jointpos_map_it;
    jointpos_map_it = _current_jointpos.find(joint_name); 
    if(jointpos_map_it!=_current_jointpos.end()) 
    { 
      pos = jointpos_map_it->second;
      return true;
    }
    else 
     return false; 
}



bool GlKinematicBody::get_mesh_struct(const std::string &link_geometry_name, MeshStruct &mesh_struct) 
{
  //std::map<std::string, MeshStruct>::const_iterator mesh_map_it;
  std::map<std::string, shared_ptr<MeshStruct> >::const_iterator mesh_map_it;
  mesh_map_it = _mesh_map.find(link_geometry_name);
  if(mesh_map_it!=_mesh_map.end()) 
  { 
   //mesh_struct = mesh_map_it->second;
    mesh_struct = *mesh_map_it->second;
    return true;
  }
  else 
   return false; 
}


//===============================================================================================
// DRAWING METHODS
//
void GlKinematicBody::draw_link(shared_ptr<urdf::Geometry> link, const std::string &nextTfname, const KDL::Frame &nextTfframe)
{

  //--get rotation in angle/axis form
  double theta;
  double axis[3];
  double x,y,z,w;
  nextTfframe.M.GetQuaternion(x,y,z,w);
  double quat[4] = {w,x,y,z};
  bot_quat_to_angle_axis(quat, &theta, axis);
  
 GLUquadricObj* quadric = gluNewQuadric();
 gluQuadricDrawStyle(quadric, GLU_FILL);
 gluQuadricNormals(quadric, GLU_SMOOTH);
 gluQuadricOrientation(quadric, GLU_OUTSIDE);

 int type = link->type ;
  //enum {SPHERE, BOX, CYLINDER, MESH}; 
  if (type == urdf::Geometry::SPHERE)
    {
      shared_ptr<urdf::Sphere> sphere(shared_dynamic_cast<urdf::Sphere>(link));	
      double radius = sphere->radius;
       glPushMatrix();
       glTranslatef(nextTfframe.p[0], nextTfframe.p[1], nextTfframe.p[2]);
       glutSolidSphere(radius,36,36); 
	    //drawSphere(6,  radius);
       glPopMatrix();
    
    }
  else if  (type == urdf::Geometry::BOX)
    {
    shared_ptr<urdf::Box> box(shared_dynamic_cast<urdf::Box>(link));
    double xDim = box->dim.x;
    double yDim = box->dim.y;
    double zDim = box->dim.z;
  //todo
    glPushMatrix();
        //size cuboid
    
        // move base up so that bottom face is at origin
     // glTranslatef(0,0.5,0.0); 
     glTranslatef(nextTfframe.p[0],nextTfframe.p[1],nextTfframe.p[2]);

     glRotatef(theta * 180/3.141592654, 
       	 axis[0], axis[1], axis[2]); 
     glScalef(xDim,yDim,zDim);
     bot_gl_draw_cube();
        //cube();
    glPopMatrix();
  

  }else if  (type == urdf::Geometry::CYLINDER){
    shared_ptr<urdf::Cylinder> cyl(shared_dynamic_cast<urdf::Cylinder>(link));

    glPushMatrix();
    double v[] = {0,0, -cyl->length/2.0};
    double result[3];
    bot_quat_rotate_to(quat,v,result);

    // Translate tf origin to cylinder centre
    glTranslatef(result[0],result[1],result[2]); 

    glTranslatef(nextTfframe.p[0],nextTfframe.p[1],nextTfframe.p[2]);

    glRotatef(theta * 180/3.141592654, 
    axis[0], axis[1], axis[2]); 

    gluCylinder(quadric,
      cyl->radius,
      cyl->radius,
      (double) cyl->length,
      36,
      1);

    //gluDeleteQuadric(quadric);
    glPopMatrix();

    // drawing two disks to make a SOLID cylinder
    glPushMatrix();  

    v[2] = -(cyl->length/2.0);
    bot_quat_rotate_to(quat,v,result);

    // Translate tf origin to cylinder centre
    glTranslatef(result[0],result[1],result[2]); 
    glTranslatef(nextTfframe.p[0],nextTfframe.p[1],nextTfframe.p[2]);
      glRotatef(theta * 180/3.141592654, 
      axis[0], axis[1], axis[2]); 
    gluDisk(quadric,
      0,
      cyl->radius,
      36,
      1);
    glPopMatrix();
    glPushMatrix(); 

    v[2] = (cyl->length/2.0);
    bot_quat_rotate_to(quat,v,result);

    // Translate tf origin to cylinder centre
    glTranslatef(result[0],result[1],result[2]); 
    glTranslatef(nextTfframe.p[0],nextTfframe.p[1],nextTfframe.p[2]);
    glRotatef(theta * 180/3.141592654, 
      axis[0], axis[1], axis[2]); 
    gluDisk(quadric,
      0,
      cyl->radius,
      36,
      1);
    glPopMatrix();

    //cout << "CYLINDER"<< endl;
    //cout << "radius : "<<  cyl->radius << endl;
    //cout << "length : "<<  cyl->length << endl;
    // drawBox(radius,length, it->second -> visual->origin);
  }
  else if  (type == urdf::Geometry::MESH)
    {
    
    //shared_ptr<urdf::Mesh> mesh(shared_dynamic_cast<urdf::Mesh>(link));
    
     /* size_t found1;
      found1=nextTf.link_name.find("r_");
    if (found1!=std::string::npos)
      {*/
        glPushMatrix();
        
        glTranslatef(nextTfframe.p[0],nextTfframe.p[1],nextTfframe.p[2]);
        glRotatef(theta * 180/3.141592654, 
        axis[0], axis[1], axis[2]); 


        //std::map<std::string, MeshStruct>::const_iterator mesh_map_it;
        std::map<std::string, shared_ptr<MeshStruct> >::const_iterator mesh_map_it;
        mesh_map_it=_mesh_map.find(nextTfname);
        if(mesh_map_it!=_mesh_map.end()) // exists in cache
        { 
         
          if(!visualize_bbox)
          {
            //cout << mesh_map_it->first <<  " dl: " << mesh_map_it->second.displaylist<< endl;
            //glCallList (mesh_map_it->second.displaylist); 
	    glCallList (mesh_map_it->second->displaylist); 
            //cout << glGetError() << endl;
          }
          else 
          {
            // get the vertices for mesh_map_it->second
            /*double xDim = mesh_map_it->second.span_x;
            double yDim = mesh_map_it->second.span_y;
            double zDim = mesh_map_it->second.span_z;
            double xc = mesh_map_it->second.offset_x;
            double yc = mesh_map_it->second.offset_y;
            double zc = mesh_map_it->second.offset_z;;*/
            double xDim = mesh_map_it->second->span_x;
            double yDim = mesh_map_it->second->span_y;
            double zDim = mesh_map_it->second->span_z;
            double xc = mesh_map_it->second->offset_x;
            double yc = mesh_map_it->second->offset_y;
            double zc = mesh_map_it->second->offset_z;
            
            //glCallList (mesh_map_it->second.displaylist);
            glCallList (mesh_map_it->second->displaylist); 

            glPopMatrix(); 
            // (27th Nov- Steven and Sisir)
            // THE DRC SDF defines the visual origin internally within the mesh vertices, which is wierd.
            // The visual origin itself is set to 0,0,0
            // So if you are drawing a simple geometry you need to get the visual origin from the average of the extrema vertices
            //============================================
            glPushMatrix();
            glTranslatef(nextTfframe.p[0],nextTfframe.p[1],nextTfframe.p[2]);
            glRotatef(theta * 180/3.141592654, 
            axis[0], axis[1], axis[2]); 
            glTranslatef(xc, yc, zc);
            glScalef(xDim,yDim,zDim);
            bot_gl_draw_cube_frame();
            
          }	
        }

    glPopMatrix();

   // }// end if (found1!=std::string::npos)
  }
  else {
  //cout << "UNKNOWN"<< endl;
  }

  gluDeleteQuadric(quadric);
}
// ======================================================================================================
// overloaded draw_link method for otdf geometries
void GlKinematicBody::draw_link(shared_ptr<otdf::Geometry> link,const std::string &nextTfname, const KDL::Frame &nextTfframe)
{

  //--get rotation in angle/axis form
  double theta;
  double axis[3];
  double x,y,z,w;
  nextTfframe.M.GetQuaternion(x,y,z,w);
  double quat[4] = {w,x,y,z};
  bot_quat_to_angle_axis(quat, &theta, axis);

 GLUquadricObj* quadric = gluNewQuadric();
 gluQuadricDrawStyle(quadric, GLU_FILL);
 gluQuadricNormals(quadric, GLU_SMOOTH);
 gluQuadricOrientation(quadric, GLU_OUTSIDE);

  ///////////////////////////////////////////////////////////////
  // draw object

 int type = link->type ;
 //enum {SPHERE, BOX, CYLINDER, MESH, TORUS}; 
  if (type == otdf::Geometry::SPHERE)
    {
      shared_ptr<otdf::Sphere> sphere(shared_dynamic_cast<otdf::Sphere>(link));	
      double radius = sphere->radius;
       glPushMatrix();
       glTranslatef(nextTfframe.p[0], nextTfframe.p[1], nextTfframe.p[2]);
       glutSolidSphere(radius,36,36);
	     //drawSphere(6,  radius);
       glPopMatrix();
    
    }
  else if  (type == otdf::Geometry::BOX)
    {
    shared_ptr<otdf::Box> box(shared_dynamic_cast<otdf::Box>(link));
    double xDim = box->dim.x;
    double yDim = box->dim.y;
    double zDim = box->dim.z;
  //todo
    glPushMatrix();
        //size cuboid
    
        // move base up so that bottom face is at origin
     // glTranslatef(0,0.5,0.0); 
     glTranslatef(nextTfframe.p[0], nextTfframe.p[1], nextTfframe.p[2]);

     glRotatef(theta * 180/3.141592654, 
       	 axis[0], axis[1], axis[2]); 
     glScalef(xDim,yDim,zDim);
         bot_gl_draw_cube();
        //cube();
    glPopMatrix();
  

  }else if  (type == otdf::Geometry::CYLINDER){
    shared_ptr<otdf::Cylinder> cyl(shared_dynamic_cast<otdf::Cylinder>(link));

    glPushMatrix();
    double v[] = {0,0, -cyl->length/2.0};
    double result[3];
    bot_quat_rotate_to(quat,v,result);

    // Translate tf origin to cylinder centre
    glTranslatef(result[0],result[1],result[2]); 

    glTranslatef(nextTfframe.p[0], nextTfframe.p[1], nextTfframe.p[2]);

    glRotatef(theta * 180/3.141592654, 
    axis[0], axis[1], axis[2]); 

    gluCylinder(quadric,
      cyl->radius,
      cyl->radius,
      (double) cyl->length,
      36,
      1);

    //gluDeleteQuadric(quadric);
    glPopMatrix();

    // drawing two disks to make a SOLID cylinder
    glPushMatrix();  

    v[2] = -(cyl->length/2.0);
    bot_quat_rotate_to(quat,v,result);

    // Translate tf origin to cylinder centre
    glTranslatef(result[0],result[1],result[2]); 
    glTranslatef(nextTfframe.p[0], nextTfframe.p[1], nextTfframe.p[2]);
    glRotatef(theta * 180/3.141592654, 
      axis[0], axis[1], axis[2]); 
    gluDisk(quadric,
      0,
      cyl->radius,
      36,
      1);
    glPopMatrix();
    glPushMatrix(); 

    v[2] = (cyl->length/2.0);
    bot_quat_rotate_to(quat,v,result);

    // Translate tf origin to cylinder centre
    glTranslatef(result[0],result[1],result[2]); 
    glTranslatef(nextTfframe.p[0], nextTfframe.p[1], nextTfframe.p[2]);
    glRotatef(theta * 180/3.141592654, 
      axis[0], axis[1], axis[2]); 
    gluDisk(quadric,
      0,
      cyl->radius,
      36,
      1);
    glPopMatrix();

    //cout << "CYLINDER"<< endl;
    //cout << "radius : "<<  cyl->radius << endl;
    //cout << "length : "<<  cyl->length << endl;
    // drawBox(radius,length, it->second -> visual->origin);
  }
  else if  (type == otdf::Geometry::MESH)
    {
    //cout << "MESH: " << nextTfname << endl;
    //shared_ptr<otdf::Mesh> mesh(shared_dynamic_cast<otdf::Mesh>(link));
    
     /* size_t found1;
      found1=nextTf.link_name.find("r_");
    if (found1!=std::string::npos)
      {*/
        glPushMatrix();
        
        glTranslatef(nextTfframe.p[0], nextTfframe.p[1], nextTfframe.p[2]);
        
        glRotatef(theta * 180/3.141592654, 
        axis[0], axis[1], axis[2]); 


        //std::map<std::string, MeshStruct>::const_iterator mesh_map_it;
        std::map<std::string, shared_ptr<MeshStruct> >::const_iterator mesh_map_it;
        mesh_map_it=_mesh_map.find(nextTfname);
        if(mesh_map_it!=_mesh_map.end()) // exists in cache
        { 
          if(!visualize_bbox)
          {
            //glCallList (mesh_map_it->second.displaylist);
	    glCallList (mesh_map_it->second->displaylist);
          }
          else 
          {
            // get the vertices for mesh_map_it->second
            /*double xDim = mesh_map_it->second.span_x;
            double yDim = mesh_map_it->second.span_y;
            double zDim = mesh_map_it->second.span_z;
            double xc = mesh_map_it->second.offset_x;
            double yc = mesh_map_it->second.offset_y;
            double zc = mesh_map_it->second.offset_z;*/
            double xDim = mesh_map_it->second->span_x;
            double yDim = mesh_map_it->second->span_y;
            double zDim = mesh_map_it->second->span_z;
            double xc = mesh_map_it->second->offset_x;
            double yc = mesh_map_it->second->offset_y;
            double zc = mesh_map_it->second->offset_z;
            
            
            //glCallList (mesh_map_it->second.displaylist);
            glCallList (mesh_map_it->second->displaylist); 

            // (27th Nov- Steven and Sisir)
            // THE DRC SDF defines the visual origin internally within the mesh vertices, which is wierd.
            // The visual origin itself is set to 0,0,0
            // So if you are drawing a simple geometry you need to get the visual origin from the average of the extrema vertices
            //============================================
            glTranslatef(xc, yc, zc);
            glScalef(xDim,yDim,zDim);
            bot_gl_draw_cube_frame();
            
          }	
        }

    glPopMatrix();

   // }// end if (found1!=std::string::npos)
  }
  else if  (type == otdf::Geometry::DYNAMIC_MESH)
  {
    //NOTE: this is now drawn by renderer_affordances.cpp
    
    /*
    boost::shared_ptr<otdf::DynamicMesh> dm(boost::shared_dynamic_cast<otdf::DynamicMesh>(link));
    glPushMatrix();
    glTranslatef(nextTfframe.p[0], nextTfframe.p[1], nextTfframe.p[2]);
    glRotatef(theta * 180/M_PI, axis[0], axis[1], axis[2]); 
    for(int i=0; i<dm->triangles.size(); i++){
      glBegin(GL_POLYGON);
      for(int j=0; j<3; j++) glVertex3fv(dm->points[dm->triangles[i][j]].data());
      glEnd();
    }
    glPopMatrix();
    */
  }
  else if  (type == otdf::Geometry::TORUS)
  {
    boost::shared_ptr<otdf::Torus> torus(boost::shared_dynamic_cast<otdf::Torus>(link));
    double innerRadius = torus->tube_radius;
    double outerRadius = torus->radius;
   
  //todo
    glPushMatrix();
        //size cuboid
    
      // move base up so that bottom face is at origin
    glTranslatef(nextTfframe.p[0], nextTfframe.p[1], nextTfframe.p[2]);
    glRotatef(theta * 180/3.141592654, 
       	 axis[0], axis[1], axis[2]); 
    glutSolidTorus(innerRadius,outerRadius,36,36); 
    // glutWireTorus(innerRadius,outerRadius,8,8); 
 
    glPopMatrix();

  }
  else {
  //cout << "UNKNOWN"<< endl;
  }

  gluDeleteQuadric(quadric);
}


void GlKinematicBody::draw_whole_body_bbox()
{
 
  Eigen::Vector3f whole_body_span_dims,offset;
  get_whole_body_span_dims(whole_body_span_dims,offset); //bounding box for entire body
  glPushMatrix();
  glTranslatef(_T_world_body.p[0],_T_world_body.p[1],_T_world_body.p[2]);// world frame to body frame
  //glTranslatef(xc,yc,zc);
  glTranslatef(offset[0],offset[1],offset[2]);
  glScalef(whole_body_span_dims[0],whole_body_span_dims[1],whole_body_span_dims[2]);
  bot_gl_draw_cube_frame();
  glPopMatrix();/**/

}


//===============================================================================================
// MISC. UTILITIES 
//
 std::string GlKinematicBody::evalMeshFilePath(std::string file_path_expression, bool return_convex_hull_path)
  {
    std::string result = "";
    std::string package_path = std::string(getModelsPath()) + "/mit_gazebo_models/"; // getModelsPath gives /drc/software/build/models/
    std::string token_str1 ("package://");
    std::string token_str2 (".");
    size_t found1, found2;
    std::string  file_path= "";
    found1=file_path_expression.find(token_str1);
    if (found1!=std::string::npos) // if string contains package:// as a substring 
    {  
    found2=file_path_expression.find(token_str2);
      if (return_convex_hull_path){
        file_path = package_path + file_path_expression.substr(found1+token_str1.size(),found2-found1-token_str1.size())+"_chull.obj"; 
      }else{
        file_path = package_path + file_path_expression.substr(found1+token_str1.size(),found2-found1-token_str1.size())+".obj"; 
      }
    }
    else
    file_path = file_path_expression;


    return file_path;
  }

//-------------------------------------------------------------------------------------
  std::string GlKinematicBody::exec(std::string cmd) 
  {
      FILE* pipe = popen(cmd.c_str(), "r");
      if (!pipe) return "ERROR";
      char buffer[128];
      std::string result = "";
      while(!feof(pipe)) {
      	if(fgets(buffer, 128, pipe) != NULL)
      		result += buffer;
      }
      pclose(pipe);
      return result;
  }
//-------------------------------------------------------------------------------------
  std::string GlKinematicBody::evalROSMeshFilePath(std::string file_path_expression) 
  {
      std::string result = "";
      std::string token_str1 ("package://");
      std::string token_str2 ("/");
      size_t found1, found2;
      std::string  file_path= "";
      found1=file_path_expression.find(token_str1);
      if (found1!=std::string::npos) // if string contains package:// as a substring 
      {
       	found2 = file_path_expression.find(token_str2,found1+token_str1.size()+1);
       	std::string package_name = file_path_expression.substr(found1+token_str1.size(),found2-found1-token_str1.size());

       	std::string cmd = "rospack find " + package_name;  
       	std::string  package_path = exec(cmd);

       	file_path = package_path.substr(0,package_path.size()-1) + 	    file_path_expression.substr(found2); 	
      }
      else
      	file_path = file_path_expression;

      return file_path;
  }
  
//-------------------------------------------------------------------------------------  

void GlKinematicBody::get_whole_body_span_dims(Eigen::Vector3f &whole_body_span_dims,Eigen::Vector3f &offset)
{

 float global_min_x,global_max_x,global_min_y,global_max_y,global_min_z,global_max_z;
 global_min_x=0;  global_max_x=0;  global_min_y=0;  global_max_y=0;  global_min_z=0;  global_max_z=0;  
 whole_body_span_dims<<1,1,1;
 offset<<0,0,0;
 
 double xc = 0;   double yc = 0;   double zc = 0;
 for (size_t i = 0;i < _link_geometry_tfs.size();i++){
 
    KDL::Frame link_tf = _link_geometry_tfs[i].frame; // in world frame
    
    float min_x,max_x,min_y,max_y,min_z,max_z; // get span in link frame
    min_x=0;  max_x=0;  min_y=0;  max_y=0;  min_z=0;  max_z=0;  
    if(!is_otdf_instance)
    {
      int type = _link_shapes[i]->type ;

      if (type == urdf::Geometry::SPHERE)
      {
        boost::shared_ptr<urdf::Sphere> sphere(boost::shared_dynamic_cast<urdf::Sphere>(_link_shapes[i]));	
        double radius = sphere->radius;
        min_x = -radius; min_y = -radius; min_z = -radius;
        max_x =  radius; max_y =  radius; max_z =  radius;
      }
      else if  (type == urdf::Geometry::BOX)
      {
        boost::shared_ptr<urdf::Box> box(boost::shared_dynamic_cast<urdf::Box>(_link_shapes[i]));
        min_x = -0.5*box->dim.x; min_y = -0.5*box->dim.y; min_z = -0.5*box->dim.z;
        max_x =  0.5*box->dim.x; max_y =  0.5*box->dim.y; max_z =  0.5*box->dim.z;
      }
      else if  (type == urdf::Geometry::CYLINDER)
      {
        boost::shared_ptr<urdf::Cylinder> cyl(boost::shared_dynamic_cast<urdf::Cylinder>(_link_shapes[i]));
        min_x = -cyl->radius; min_y = -cyl->radius; min_z = -0.5*cyl->length;
        max_x =  cyl->radius; max_y =  cyl->radius; max_z =  0.5*cyl->length;
      }
      else if  (type == urdf::Geometry::MESH)
      {
        //std::map<std::string, MeshStruct>::const_iterator mesh_map_it;
	std::map<std::string, shared_ptr<MeshStruct> >::const_iterator mesh_map_it;
        mesh_map_it=_mesh_map.find(_link_geometry_tfs[i].name);
        if(mesh_map_it!=_mesh_map.end()) // exists in cache
        { 
          // get the vertices for mesh_map_it->second
          /*double xDim = mesh_map_it->second.span_x;
          double yDim = mesh_map_it->second.span_y;
          double zDim = mesh_map_it->second.span_z;
           xc = mesh_map_it->second.offset_x;
           yc = mesh_map_it->second.offset_y;
           zc = mesh_map_it->second.offset_z;*/

          double xDim = mesh_map_it->second->span_x;
          double yDim = mesh_map_it->second->span_y;
          double zDim = mesh_map_it->second->span_z;
           xc = mesh_map_it->second->offset_x;
           yc = mesh_map_it->second->offset_y;
           zc = mesh_map_it->second->offset_z;
          min_x = -0.5*xDim+xc; min_y = -0.5*yDim+yc; min_z = -0.5*zDim+zc;
          max_x =  0.5*xDim+xc; max_y =  0.5*yDim+yc; max_z =  0.5*zDim+zc;
        }
      }
    }
    else {
      int type = _otdf_link_shapes[i]->type ;
      
      if (type == otdf::Geometry::SPHERE)
      {
        boost::shared_ptr<otdf::Sphere> sphere(boost::shared_dynamic_cast<otdf::Sphere>(_otdf_link_shapes[i]));	
        double radius = sphere->radius;
        min_x = -radius; min_y = -radius; min_z = -radius;
        max_x =  radius; max_y =  radius; max_z =  radius;
      }
      else if  (type == otdf::Geometry::BOX)
      {
        boost::shared_ptr<otdf::Box> box(boost::shared_dynamic_cast<otdf::Box>(_otdf_link_shapes[i]));
        min_x = -0.5*box->dim.x; min_y = -0.5*box->dim.y; min_z = -0.5*box->dim.z;
        max_x =  0.5*box->dim.x; max_y =  0.5*box->dim.y; max_z =  0.5*box->dim.z;
      }
      else if  (type == otdf::Geometry::CYLINDER)
      {
        boost::shared_ptr<otdf::Cylinder> cyl(boost::shared_dynamic_cast<otdf::Cylinder>(_otdf_link_shapes[i]));
        min_x = -cyl->radius; min_y = -cyl->radius; min_z = -0.5*cyl->length;
        max_x =  cyl->radius; max_y =  cyl->radius; max_z =  0.5*cyl->length;
      }
      else if  (type == otdf::Geometry::MESH)
      {
        //std::map<std::string, MeshStruct>::const_iterator mesh_map_it;
	std::map<std::string, shared_ptr<MeshStruct> >::const_iterator mesh_map_it;
        mesh_map_it=_mesh_map.find(_link_geometry_tfs[i].name);
        if(mesh_map_it!=_mesh_map.end()) // exists in cache
        { 
          // get the vertices for mesh_map_it->second
          /*double xDim = mesh_map_it->second.span_x;
          double yDim = mesh_map_it->second.span_y;
          double zDim = mesh_map_it->second.span_z;
           xc = mesh_map_it->second.offset_x;
           yc = mesh_map_it->second.offset_y;
           zc = mesh_map_it->second.offset_z;*/
          double xDim = mesh_map_it->second->span_x;
          double yDim = mesh_map_it->second->span_y;
          double zDim = mesh_map_it->second->span_z;
           xc = mesh_map_it->second->offset_x;
           yc = mesh_map_it->second->offset_y;
           zc = mesh_map_it->second->offset_z;
          min_x = -0.5*xDim+xc; min_y = -0.5*yDim+yc; min_z = -0.5*zDim+zc;
          max_x =  0.5*xDim+xc; max_y =  0.5*yDim+yc; max_z =  0.5*zDim+zc;
        }
      }
      else if  (type == otdf::Geometry::TORUS)
      {
        boost::shared_ptr<otdf::Torus> torus(boost::shared_dynamic_cast<otdf::Torus>(_otdf_link_shapes[i]));
        min_x = -torus->radius; min_y = -torus->radius; min_z = -torus->tube_radius;
        max_x =  torus->radius; max_y =  torus->radius; max_z =  torus->tube_radius;
      }
      
    
    }// end if(!is_otdf_instance)    
      
    // corners in link frame
    Eigen::Matrix<float, 3, 8> corners;
    corners << min_x,min_x,min_x,min_x,max_x,max_x,max_x,max_x,
               min_y,min_y,max_y,max_y,min_y,min_y,max_y,max_y,
               min_z,max_z,min_z,max_z,min_z,max_z,min_z,max_z;

    // transform eight corners to body frame
     KDL::Vector corner, rotated_corner;
   for(uint k=0;k<8;k++){
       corner[0]=corners(0,k);corner[1]=corners(1,k);corner[2]=corners(2,k);
       rotated_corner=(link_tf*corner)-_T_world_body.p; //T_world_link*T_link_corner - T_world_body
       if(rotated_corner[0]<=global_min_x)
          global_min_x = rotated_corner[0];
       if(rotated_corner[1]<=global_min_y)
          global_min_y = rotated_corner[1];
       if(rotated_corner[2]<=global_min_z)
          global_min_z = rotated_corner[2];   
       if(rotated_corner[0]>=global_max_x)
          global_max_x = rotated_corner[0];
       if(rotated_corner[1]>=global_max_y)
          global_max_y = rotated_corner[1];
       if(rotated_corner[2]>=global_max_z)
          global_max_z = rotated_corner[2]; 
    }

 }// end for _link_geometry_tfs

  whole_body_span_dims[0]= global_max_x - global_min_x;
  whole_body_span_dims[1]= global_max_y - global_min_y;
  whole_body_span_dims[2]= global_max_z - global_min_z;
  
  offset[0] = 0.5*(global_max_x + global_min_x);
  offset[1] = 0.5*(global_max_y + global_min_y);
  offset[2] = 0.5*(global_max_z + global_min_z);
  
}
 
