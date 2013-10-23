
#include "StickyfootCollectionManager.hpp"
#include <tinyxml.h>
using namespace visualization_utils;
using namespace collision;

//==================constructor / destructor

StickyfootCollectionManager::StickyfootCollectionManager(boost::shared_ptr<lcm::LCM> &lcm):_lcm(lcm), _foot_urdfs_found(false),free_running_sticky_foot_cnt(0)
{
    //lcm ok?
    if(!_lcm->good()) {
        cerr << "\nLCM Not Good: StickyfootCollectionManager" << endl;
        return;
    }
    _feet.clear();
    _T_groundframe_bodyframe_left = KDL::Frame::Identity();
    _T_groundframe_bodyframe_right = KDL::Frame::Identity();

    _left_foot_name ="l_foot";
    _right_foot_name = "r_foot";
    if(!load_foot_urdfs()){
        cerr << "##### ERROR: #####" <<  " sticky_foot urdfs not found in StickyfootCollectionManager.cpp. Disabling StickyfootCollection Manager. Please update your models folder" << endl;    
        return;
    }
    _foot_urdfs_found= true;
    _base_gl_foot_left =  boost::shared_ptr<GlKinematicBody>(new GlKinematicBody(_left_urdf_xml_string));     
    _base_gl_foot_right =  boost::shared_ptr<GlKinematicBody>(new GlKinematicBody(_right_urdf_xml_string));    

    std::map<std::string, double> jointpos_in;
    jointpos_in =  _base_gl_foot_left->_current_jointpos;
    _base_gl_foot_left->set_state(_base_gl_foot_left->_T_world_body,jointpos_in); // set to initialized values.
    jointpos_in.clear();
    jointpos_in =  _base_gl_foot_right->_current_jointpos;
    _base_gl_foot_right->set_state(_base_gl_foot_left->_T_world_body,jointpos_in); // set to initialized values.  


    Eigen::Vector3f whole_body_span;
    Eigen::Vector3f offset;
    MeshStruct mesh_struct;
    bool val;
    _base_gl_foot_left->get_whole_body_span_dims(whole_body_span,offset);

    val =_base_gl_foot_left->get_mesh_struct("l_talus_0", mesh_struct);
    _T_groundframe_bodyframe_left.p[2] = whole_body_span[2]-0.5*(mesh_struct.span_z);    

    _base_gl_foot_right->get_whole_body_span_dims(whole_body_span,offset);
    val = _base_gl_foot_right->get_mesh_struct("r_talus_0", mesh_struct); 
    _T_groundframe_bodyframe_right.p[2] = whole_body_span[2]-0.5*(mesh_struct.span_z);      

}

StickyfootCollectionManager::~StickyfootCollectionManager() {

}


//------------------------------------------------------------------------------ 

bool  StickyfootCollectionManager::load_foot_urdfs()
{   

    string urdf_models_path = string(getModelsPath()) + "/mit_gazebo_models/mit_robot_feet/"; // getModelsPath gives /drc/software/build/models/
    cout << "searching for foot urdf files in: "<< (urdf_models_path) << endl;
    vector<string> urdf_files = vector<string>();
    int res = get_URDF_filenames_from_dir(urdf_models_path, urdf_files);  

 
    if(res==0)  //urdf found
        cout << "found " << urdf_files.size() << " " << urdf_files[0] << " " << urdf_files[1] << " files"<< endl;
    else{
        cerr << "ERROR: no urdf files found in: "<< (urdf_models_path) << endl;
        return false;
    } 
    
    std::vector<std::string>::const_iterator found;
    found = std::find (urdf_files.begin(), urdf_files.end(), _left_foot_name); 
    if(found !=  urdf_files.end()) {
        unsigned int index = found - urdf_files.begin();
        std::stringstream oss;
        oss << urdf_models_path << urdf_files[index] << ".urdf" ;
        get_xmlstring_from_file(oss.str(), _left_urdf_xml_string);
    }
    else {
        cerr <<"ERROR: " << _left_foot_name  << ".urdf not found"<< endl;
        return false;
    }

    found = std::find (urdf_files.begin(), urdf_files.end(), _right_foot_name);  
    if(found !=  urdf_files.end()) {
        unsigned int index = found - urdf_files.begin();
        std::stringstream oss;
        oss << urdf_models_path << urdf_files[index] << ".urdf" ;
        get_xmlstring_from_file(oss.str(), _right_urdf_xml_string);
    }
    else {
        cerr <<"ERROR:" << _right_foot_name  << ".urdf not found"<< endl;
        return false;
    }
    return true;
}


//-------------------------------------------------------------------------------------------

void StickyfootCollectionManager::add_or_update_sticky_foot(int uid,int foot_type, string& object_name, string& geometry_name, 
                                                             KDL::Frame &T_world_foot, vector<string> &joint_names,vector<double> &joint_positions)		
{

    if(!_foot_urdfs_found) {
        cerr << "##### ERROR: #####" <<  " sticky_foot urdfs not found in CandidateFootStepSeedManager.cpp. Cannot spawn sticky feet" << endl;    
        return;    
    }
    
    string  unique_foot_name;
    std::stringstream oss;
    if(foot_type==0)
        oss << object_name <<"_"<< geometry_name << "_lfootstep_" << uid;
    else
        oss << object_name <<"_"<< geometry_name << "_rfootstep_" << uid;
    unique_foot_name = oss.str(); 

    typedef std::map<std::string,StickyFootStruc> sticky_feet_map_type;
    sticky_feet_map_type::iterator it = _feet.find(unique_foot_name);
    if(it ==_feet.end() ) { // not in cache
 
        StickyFootStruc sticky_foot_struc;
        sticky_foot_struc.object_name = object_name;//.c_str() ;
        sticky_foot_struc.geometry_name = geometry_name;//.c_str(); if created here as na c str it goes out of scope when called from top level functions, hence using string.
        sticky_foot_struc.uid = uid; 
        sticky_foot_struc._collision_detector.reset();
        // should we have a global collision detector and add and remove objects to it as we create and delete objects and foots. We would have to manually add and remove multiple links to the collision detector.??
        // Each foot has its own collision detector for now.
        sticky_foot_struc._collision_detector = boost::shared_ptr<Collision_Detector>(new Collision_Detector()); 
        sticky_foot_struc.foot_type=foot_type;
  
        if(foot_type==0)// LEFT
                sticky_foot_struc._gl_foot = boost::shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody((*_base_gl_foot_left),sticky_foot_struc._collision_detector,true,unique_foot_name));
        else if(foot_type==1)// RIGHT
                sticky_foot_struc._gl_foot = boost::shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody((*_base_gl_foot_right),sticky_foot_struc._collision_detector,true,unique_foot_name));
  
        drc::joint_angles_t posture_msg;
        posture_msg.num_joints = joint_names.size();
        posture_msg.joint_name = joint_names;
        posture_msg.joint_position = joint_positions;
  
        sticky_foot_struc._gl_foot->set_state(T_world_foot, posture_msg);
        sticky_foot_struc.foot_type = foot_type;
        sticky_foot_struc.T_geometry_foot = T_world_foot;

        sticky_foot_struc.joint_name = joint_names;
        sticky_foot_struc.joint_position = joint_positions;
        _feet.insert(make_pair(unique_foot_name, sticky_foot_struc));
    }
    else {
        drc::joint_angles_t posture_msg;
        posture_msg.num_joints= joint_names.size();
        posture_msg.joint_name= joint_names;
        posture_msg.joint_position= joint_positions;

        it->second._gl_foot->set_state(T_world_foot, posture_msg);
        it->second.T_geometry_foot = T_world_foot; 
        it->second.joint_position = joint_positions;
    }
}
//-------------------------------------------------------------------------------------------
// returns true if a given object name has dependent sticky feet
bool StickyfootCollectionManager::is_parent_object(std::string& object_name) 
{
  sticky_feet_map_type_::iterator foot_it = _feet.begin();
  while (foot_it!=_feet.end()) 
  {
     if (foot_it->second.object_name == (object_name))
     {
       return true;
     }
     
     foot_it++;
  } 
  return false;
}
//-------------------------------------------------------------------------------------------
bool StickyfootCollectionManager::remove(std::string& id) {
    sticky_feet_map_type_::iterator foot_it = _feet.find(id);
    if(foot_it!=_feet.end()){
        _feet.erase(foot_it);
        return true;
    }    
    return false;
}
//-------------------------------------------------------------------------------------------
bool StickyfootCollectionManager::remove_selected(boost::shared_ptr<visualization_utils::SelectionManager>  &selectionManager) {

 sticky_feet_map_type_::iterator foot_it = _feet.begin();
  while (foot_it!=_feet.end()) 
  {
     string id = foot_it->first;
     if(selectionManager->is_selected(id))
     {
         selectionManager->remove(id); 
        _feet.erase(foot_it++);
     }
     else
        foot_it++;
  } 
}
//-------------------------------------------------------------------------------------------
void StickyfootCollectionManager::remove_seeds(std::string& obj_id,boost::shared_ptr<visualization_utils::AffordanceCollectionManager>  &affCollectionManager) {

  sticky_feet_map_type_::iterator foot_it = _feet.begin();
  while (foot_it!=_feet.end()) 
  {
     string id = foot_it->first;
     if (foot_it->second.object_name == obj_id)
     { 
        _feet.erase(foot_it++);
     }
     else
        foot_it++;
  }
}
//-------------------------------------------------------------------------------------------
bool StickyfootCollectionManager::store(std::string& id,bool unstore,
                                         boost::shared_ptr<visualization_utils::AffordanceCollectionManager>  &affCollectionManager) {
    sticky_feet_map_type_::iterator foot_it = _feet.find(id);
    if(foot_it!=_feet.end()){
      object_instance_map_type_::iterator obj_it = affCollectionManager->_objects.find(string(foot_it->second.object_name));
        // get otdf name
        std::string otdf_models_path = std::string(getModelsPath()) + "/otdf/"; 
        std::string filepath =  otdf_models_path + obj_it->second.otdf_type +".otdf";

        // get variables to create grasp_seed xml
        GraspSeed graspSeed;
        graspSeed.appType = GraspSeed::FOOT;
        KDL::Frame& geo = foot_it->second.T_geometry_foot;
        graspSeed.xyz[0] = geo.p.x(), graspSeed.xyz[1] = geo.p.y(), graspSeed.xyz[2] = geo.p.z();
        geo.M.GetRPY(graspSeed.rpy[0],graspSeed.rpy[1],graspSeed.rpy[2]);
        graspSeed.geometry_name = foot_it->second.geometry_name;
        graspSeed.grasp_type = foot_it->second.foot_type;
        graspSeed.joint_names = foot_it->second.joint_name;
        graspSeed.joint_positions = foot_it->second.joint_position;
        if(unstore) 
          graspSeed.unstoreFromOtdf(filepath);
        else        
          graspSeed.writeToOtdf(filepath);
        return true;
    }    
    return false;
}
//-------------------------------------------------------------------------------------------
bool StickyfootCollectionManager::store_selected(boost::shared_ptr<visualization_utils::SelectionManager>  &selectionManager,bool unstore,
                                         boost::shared_ptr<visualization_utils::AffordanceCollectionManager>  &affCollectionManager) {
  for(sticky_feet_map_type_::iterator it = _feet.begin(); it!=_feet.end(); it++)
  {
      string id = it->first;
      if(selectionManager->is_selected(id)){
        store(id,unstore,affCollectionManager);
      }
  }// end for 
}
//-------------------------------------------------------------------------------------------
void StickyfootCollectionManager::load_stored(OtdfInstanceStruc& instance_struc)
{

    std::vector<GraspSeed>& list = instance_struc._otdf_instance->graspSeedList_;
    
    for(int i=0; i<list.size(); i++) {

       if(list[i].appType == GraspSeed::FOOT)
       {
        std::stringstream objname;
        objname << instance_struc.otdf_type << "_"<< instance_struc.uid;
        int uid =  free_running_sticky_foot_cnt++;
        double*xyz = list[i].xyz;
        KDL::Rotation rot(KDL::Rotation::RPY(list[i].rpy[0],list[i].rpy[1],
                                             list[i].rpy[2]));
        KDL::Frame frame(rot, KDL::Vector(xyz[0],xyz[1],xyz[2]));
        string obj_name = objname.str();
        
        add_or_update_sticky_foot(uid,list[i].grasp_type,obj_name,
                                 list[i].geometry_name,frame,list[i].joint_names,
                                 list[i].joint_positions);	
      }

   }// end for

}// end method

//-------------------------------------------------------------------------------------------
void StickyfootCollectionManager:: get_motion_history_bnds_of_seeds(string object_name, int &max_motion_history_size,int &min_motion_history_size)
{
  // Publish time indexed ee motion constraints from associated sticky feet 
  for(sticky_feet_map_type_::const_iterator foot_it = _feet.begin(); foot_it!=_feet.end(); foot_it++)
  {
    string host_name = foot_it->second.object_name;
    if (host_name == (object_name))
    {
      int num_frames =  foot_it->second._gl_foot->_desired_body_motion_history.size();
      max_motion_history_size = std::max(max_motion_history_size,num_frames);
      min_motion_history_size = std::min(min_motion_history_size,num_frames);
    }    
  }
}

//-------------------------------------------------------------------------------------------  
void StickyfootCollectionManager::get_motion_constraints(string object_name, OtdfInstanceStruc& obj, bool is_retractable,
                                                  map<string, vector<KDL::Frame> > &ee_frames_map, 
                                                  map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                                  map<string, vector<double> > &joint_pos_map,
                                                  map<string, vector<int64_t> > &joint_pos_timestamps_map,
                                                  int max_num_frames)
                                                  
{

  for(sticky_feet_map_type_::const_iterator foot_it = _feet.begin(); foot_it!=_feet.end(); foot_it++)
  {
      string host_name = foot_it->second.object_name;
      if (host_name == (object_name))
      {

          string ee_name;
          if(foot_it->second.foot_type==0)
              ee_name ="l_foot";    
          else if(foot_it->second.foot_type==1)   
              ee_name ="r_foot";    
          else
              cout << "unknown foot_type in StickyfootCollectionManager::get_motion_constraints\n";  
 
          // if ee_name already exists in ee_frames_map, redundant ee_frames
          // e.g two right sticky hands on the same object.
          map<string, vector<KDL::Frame> >::const_iterator ee_it = ee_frames_map.find(ee_name);
          if(ee_it!=ee_frames_map.end()){
              cerr<<" ERROR: Cannot of two seeds of the same ee. Please consider deleting redundant seeds\n";
              return;   
          }      

          //======================     
          //KDL::Frame  T_geometry_ee = KDL::Frame::Identity(); 
          //if(!foot_it->second._gl_foot->get_link_frame(ee_name,T_geometry_ee))
          //    cout <<"ERROR: ee link "<< ee_name << " not found in sticky foot urdf"<< endl;

          KDL::Frame T_world_object = obj._gl_object->_T_world_body;

          int num_frames =  std::min(max_num_frames,(int)foot_it->second._gl_foot->_desired_body_motion_history.size());
          vector<KDL::Frame> T_world_ee_frames;
          vector<int64_t> frame_timestamps;
          for(uint i = 0; i < (uint) num_frames; i++)
          {
              KDL::Frame T_object_foot = foot_it->second._gl_foot->_desired_body_motion_history[i];
              KDL::Frame T_world_foot = T_world_object*T_object_foot;
              KDL::Frame T_world_ee = T_world_foot;//TODO: Eventually will be in object frame
              T_world_ee_frames.push_back(T_world_ee);
              int64_t timestamp=(int64_t)i*1000000;
              frame_timestamps.push_back(timestamp);   
          }
          //=================== 

          ee_frames_map.insert(make_pair(ee_name, T_world_ee_frames));
          ee_frame_timestamps_map.insert(make_pair(ee_name, frame_timestamps));    
      } // end if (host_name == (object_name))
  } // end for sticky feet
}



//-------------------------------------------------------------------------------------------

void StickyfootCollectionManager::get_pose_constraints(string object_name, OtdfInstanceStruc& obj, bool to_future_state,
                                                  map<string, vector<KDL::Frame> > &ee_frames_map, 
                                                  map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                                  map<string, vector<double> > &joint_pos_map,
                                                  map<string, vector<int64_t> > &joint_pos_timestamps_map)
                                                  
{

// Publish time indexed ee motion constraints from associated sticky feet 
      typedef map<string, StickyFootStruc > sticky_feet_map_type_;
      for(sticky_feet_map_type_::const_iterator foot_it = _feet.begin(); foot_it!=_feet.end(); foot_it++)
      {
          string host_name = foot_it->second.object_name;
          if (host_name == (object_name))
              {
   
                  string ee_name;
                  if(foot_it->second.foot_type==0)
                      ee_name ="l_foot";    
                  else if(foot_it->second.foot_type==1)   
                      ee_name ="r_foot";    
                  else
                      cout << "unknown foot_type in publish_pose_goal\n";  
         
                  // if ee_name already exists in ee_frames_map, redundant ee_frames
                  // e.g two right sticky hands on the same object.
                  map<string, vector<KDL::Frame> >::const_iterator ee_it = ee_frames_map.find(ee_name);
                  if(ee_it!=ee_frames_map.end()){
                      cerr<<" ERROR: Cannot of two seeds of the same ee. Please consider deleting redundant seeds\n";
                      return;   
                  }      
    
                  //======================     
                  KDL::Frame  T_geometry_foot = KDL::Frame::Identity(); 
                  if(!foot_it->second._gl_foot->get_link_frame(ee_name,T_geometry_foot))
                     cout <<"ERROR: ee link "<< ee_name << " not found in sticky foot urdf"<< endl;


                  KDL::Frame T_world_object = KDL::Frame::Identity();
                  KDL::Frame T_world_geometry = KDL::Frame::Identity();
                  if(!to_future_state){
                      T_world_object = obj._gl_object->_T_world_body;
                    // the object might have moved.
                     if(!obj._gl_object->get_link_geometry_frame(string(foot_it->second.geometry_name),T_world_geometry))
                        cerr << " failed to retrieve " << foot_it->second.geometry_name<<" in object " << foot_it->second.object_name <<endl;
                  }
                  else {
                    T_world_object = obj._gl_object->_T_world_body_future;
                    // the object might have moved.
                     if(!obj._gl_object->get_link_geometry_future_frame(string(foot_it->second.geometry_name),T_world_geometry))
                        cerr << " failed to retrieve " << foot_it->second.geometry_name<<" in object " << foot_it->second.object_name <<endl;        
                  }

                  int num_frames =  1;
                  vector<KDL::Frame> T_world_ee_frames;
                  vector<int64_t> frame_timestamps;
                  for(uint i = 0; i < (uint) num_frames; i++)
                  {
                      KDL::Frame  T_world_ee = T_world_geometry*T_geometry_foot;  
                      T_world_ee_frames.push_back(T_world_ee);
                      int64_t timestamp=(int64_t)i*1000000;
                      frame_timestamps.push_back(timestamp);   
                  }
                  //=================== 
      
                  ee_frames_map.insert(make_pair(ee_name, T_world_ee_frames));
                  ee_frame_timestamps_map.insert(make_pair(ee_name, frame_timestamps));    
              } // end if (host_name == (object_name))
      } // end for sticky feet
      
 }
//-------------------------------------------------------------------------------------------  
void StickyfootCollectionManager::get_selected_pose_constraints(boost::shared_ptr<visualization_utils::AffordanceCollectionManager>  &affCollectionManager, bool to_future_state,
                                                  boost::shared_ptr<visualization_utils::SelectionManager>  &selectionManager,
                                                  map<string, vector<KDL::Frame> > &ee_frames_map, 
                                                  map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                                  map<string, vector<double> > &joint_pos_map,
                                                  map<string, vector<int64_t> > &joint_pos_timestamps_map,
                                                  bool is_time_ordered)
                                                  
{



  for(sticky_feet_map_type_::const_iterator foot_it = _feet.begin(); foot_it!=_feet.end(); foot_it++)
  {
      string host_name = foot_it->second.object_name;
      object_instance_map_type_::const_iterator obj_it = affCollectionManager->_objects.find(host_name);
      string id = foot_it->first;

      if ((selectionManager->is_selected(id))&&(obj_it!=affCollectionManager->_objects.end()))
      {      
          string ee_name;
          if(foot_it->second.foot_type==0)
              ee_name ="l_foot";    
          else if(foot_it->second.foot_type==1)   
              ee_name ="r_foot";    
          else
              cout << "unknown foot_type in StickyfootCollectionManager::get_selected_motion_constraints\n";  
 
          //======================     
          KDL::Frame  T_geometry_foot = KDL::Frame::Identity(); 
          if(!foot_it->second._gl_foot->get_link_frame(ee_name,T_geometry_foot))
             cout <<"ERROR: ee link "<< ee_name << " not found in sticky foot urdf"<< endl;


          KDL::Frame T_world_object = KDL::Frame::Identity();
          KDL::Frame T_world_geometry = KDL::Frame::Identity();
          if(!to_future_state){
              T_world_object = obj_it->second._gl_object->_T_world_body;
            // the object might have moved.
             if(!obj_it->second._gl_object->get_link_geometry_frame(string(foot_it->second.geometry_name),T_world_geometry))
                cerr << " failed to retrieve " << foot_it->second.geometry_name<<" in object " << foot_it->second.object_name <<endl;
          }
          else {
            T_world_object = obj_it->second._gl_object->_T_world_body_future;
            // the object might have moved.
             if(!obj_it->second._gl_object->get_link_geometry_future_frame(string(foot_it->second.geometry_name),T_world_geometry))
                cerr << " failed to retrieve " << foot_it->second.geometry_name<<" in object " << foot_it->second.object_name <<endl;        
          }

          int num_frames =  1;
          vector<KDL::Frame> T_world_ee_frames;
          vector<int64_t> frame_timestamps;
          for(uint i = 0; i < (uint) num_frames; i++)
          {
              KDL::Frame  T_world_ee = T_world_geometry*T_geometry_foot;  
              T_world_ee.p[2] = NAN;
              T_world_ee_frames.push_back(T_world_ee);
              double nmr = (selectionManager->get_selection_order(id)-1);
              double dmr = (selectionManager->get_selection_cnt()-1);
              int64_t timestamp;
              if(is_time_ordered)
                timestamp = (int64_t)((nmr/dmr)*1000000);
              else
                timestamp=(int64_t)i*1000000;
              frame_timestamps.push_back(timestamp);   
          }
          //=================== 
          std::stringstream oss;
          oss << ee_name << "::" << selectionManager->get_selection_order(id);
          std::string unique_ee_name = oss.str();
          ee_frames_map.insert(make_pair(unique_ee_name, T_world_ee_frames));
          ee_frame_timestamps_map.insert(make_pair(unique_ee_name, frame_timestamps));    
      } // end if (host_name == (object_name))
  } // end for sticky feet
}
 
//-------------------------------------------------------------------------------------------  

void StickyfootCollectionManager::get_aff_indexed_ee_constraints(string& object_name,OtdfInstanceStruc& obj,
                                                                 boost::shared_ptr<BatchFKQueryHandler>  &dofRangeFkQueryHandler,
                                                                 map<string, vector<KDL::Frame> > &ee_frames_map,
                                                                 map<string, vector<drc::affordance_index_t> > &ee_frame_affindices_map) 
{
  for(sticky_feet_map_type_::const_iterator foot_it = _feet.begin(); foot_it!=_feet.end(); foot_it++) {
      string host_name = foot_it->second.object_name;
      if (host_name == (object_name)) {
          
          string ee_name;
          if(foot_it->second.foot_type==0)
              ee_name ="l_foot";    
          else if(foot_it->second.foot_type==1)   
              ee_name ="r_foot";    
          else
              cout << "unknown foot_type in StickyfootCollectionManager::get_aff_indexed_ee_constraints\n";  
          
          // if ee_name already exists in ee_frames_map, redundant ee_frames
          // e.g two right sticky hands on the same object.
          map<string, vector<KDL::Frame> >::const_iterator ee_it = ee_frames_map.find(ee_name);
          if(ee_it!=ee_frames_map.end()){
              cerr<<" ERROR: Cannot have two seeds of the same ee on the same affordance. Please consider deleting redundant seeds\n";
              return;   
          }      
          
          KDL::Frame  T_geometry_ee = KDL::Frame::Identity(); 
          if(!foot_it->second._gl_foot->get_link_frame(ee_name,T_geometry_ee))
              cout <<"ERROR: ee link "<< ee_name << " not found in sticky foot urdf"<< endl;
          
          
          vector<KDL::Frame> T_world_geometry_frames;
          string seed_geometry_name = foot_it->second.geometry_name;
          dofRangeFkQueryHandler->getLinkFrames(seed_geometry_name,T_world_geometry_frames);
          std::string dof_name;
          vector<double> dof_values;  
          bool isdofset = dofRangeFkQueryHandler->getAssociatedDoFNameAndVal(seed_geometry_name,dof_name,dof_values);
          if(isdofset) { // dof range was set
              int num_of_incs = T_world_geometry_frames.size();
              vector<KDL::Frame> T_world_ee_frames;
              vector<drc::affordance_index_t> frame_affindices;
              for(size_t i=0;i<(size_t)num_of_incs;i++){
                  KDL::Frame  T_world_ee = T_world_geometry_frames[i]*T_geometry_ee;     
                  T_world_ee_frames.push_back(T_world_ee);
                  
                  drc::affordance_index_t aff_index;
                  aff_index.utime=(int64_t)i;
                  aff_index.aff_type = obj.otdf_type; 
                  aff_index.aff_uid = obj.uid; 
                  aff_index.num_ees =1; 
                  aff_index.ee_name.push_back(ee_name); 
                  aff_index.dof_name.push_back(dof_name);     
                  aff_index.dof_value.push_back(dof_values[i]); 
                  frame_affindices.push_back(aff_index);
              } 
              
              ee_frames_map.insert(make_pair(ee_name, T_world_ee_frames));
              ee_frame_affindices_map.insert(make_pair(ee_name, frame_affindices)); 
          } // end if isdofset  
      } // end if (host_name == (object_name))
  } // end sticky feet
} // end method
//-------------------------------------------------------------------------------------------

KDL::Vector StickyfootCollectionManager::get_contact_mask_offset(foot_contact_mask_type_t contact_mask)
{
   KDL::Vector contact_mask_offset;
   contact_mask_offset[0]=0;
   contact_mask_offset[1]=0;
   contact_mask_offset[2]=0;
   // <collision group="toe">
  //<origin rpy="0 0 0" xyz="0.178 0 -0.081119"/>
 //<collision group="heel">
 // <origin rpy="0 0 0" xyz="-0.082 0 -0.081119"/>
 
 double foot_length = (0.178+0.082);
  if(contact_mask==ORG)
  {
   contact_mask_offset[0]=0;
  }
  else if(contact_mask==HEEL)
  {
    contact_mask_offset[0]=-(0.082-0.25*foot_length);
  }
  else if(contact_mask==TOE)
  {
    contact_mask_offset[0]=0.178-0.25*foot_length;
  }
  else if(contact_mask==MID)
  {
    contact_mask_offset[0]=0.5*(0.178-0.082);
  }
  else
  {
    cerr <<  "unknown contact mask in seed_foot. Defaulting to foot origin projection ORG\n";
  }
  return contact_mask_offset;
}   
//-------------------------------------------------------------------------------------------
void StickyfootCollectionManager::seed_foot(OtdfInstanceStruc& obj,std::string &object_name,std::string &geometry_name,int foot_type,foot_contact_mask_type_t contact_mask, Eigen::Vector3f &ray_hit_drag,Eigen::Vector3f &ray_hit,Eigen::Vector3f &ray_hit_normal)
{
    free_running_sticky_foot_cnt++;
    int uid = free_running_sticky_foot_cnt;
       
    Eigen::Vector3f eVx,eVy,eVz,diff;
    diff=ray_hit_drag-ray_hit;
    cout << diff.norm() << endl;
    if(diff.norm()< 0.001)
        diff << 0,0,1;

    diff.normalize();
    eVz = ray_hit_normal; eVz.normalize();
    eVy = eVz.cross(diff);  eVy.normalize();
    eVx = eVy.cross(eVz); eVx.normalize();

    KDL::Vector Vx,Vy,Vz;
    Vx[0]= eVx[0];Vx[1]= eVx[1];Vx[2]= eVx[2];
    Vy[0]= eVy[0];Vy[1]= eVy[1];Vy[2]= eVy[2];
    Vz[0]= eVz[0];Vz[1]= eVz[1];Vz[2]= eVz[2];
    
    
    KDL::Frame T_world_footcontact= KDL::Frame::Identity();
    T_world_footcontact.p[0] = ray_hit[0];
    T_world_footcontact.p[1] = ray_hit[1];
    T_world_footcontact.p[2] = ray_hit[2];

    // For the car (pedals) we offset the sticky foot to prevent the pedal from being depressed
    /*double delta = 0.025;
    T_world_footcontact.p[0] = ray_hit[0] + delta * eVz[0];
    T_world_footcontact.p[1] = ray_hit[1] + delta * eVz[1];
    T_world_footcontact.p[2] = ray_hit[2] + delta * eVz[2];*/

    KDL::Rotation tempM(Vx,Vy,Vz);
    T_world_footcontact.M = tempM;

    KDL::Frame T_world_objectgeometry = KDL::Frame::Identity(); 
    if(!obj._gl_object->get_link_geometry_frame(geometry_name,T_world_objectgeometry))
        cerr << " ERROR: failed to retrieve " << geometry_name <<" in object " << object_name <<endl;

    KDL::Frame T_objectgeometry_footcontact = (T_world_objectgeometry.Inverse())*T_world_footcontact;
    KDL::Frame T_objectgeometry_foot,T_contactframe_footframe;
    
    if(foot_type==1)  {
      T_contactframe_footframe = _T_groundframe_bodyframe_right;
    }
    else    {
      T_contactframe_footframe = _T_groundframe_bodyframe_left;
    }
    
    KDL::Vector contact_mask_offset = get_contact_mask_offset(contact_mask);
    T_contactframe_footframe.p -=contact_mask_offset;
    T_objectgeometry_foot = T_objectgeometry_footcontact*(T_contactframe_footframe);
    
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    
    if(foot_type==1){
      joint_names.push_back("r_leg_aky");
      joint_names.push_back("r_leg_akx");
    }
    else {
      joint_names.push_back("l_leg_aky");
      joint_names.push_back("l_leg_akx");    
    }
    
    joint_positions.push_back(0);
    joint_positions.push_back(0); 

    add_or_update_sticky_foot(uid,foot_type,object_name,geometry_name, T_objectgeometry_foot,joint_names,joint_positions);	
}    
//---------------------------------------------------------------------------------
void StickyfootCollectionManager::clear_highlights()
{
  for(sticky_feet_map_type_::iterator it = _feet.begin(); it!=_feet.end(); it++)
  {
      string no_selection = " ";
        it->second._gl_foot->highlight_link(no_selection); 
        it->second._gl_foot->highlight_marker(no_selection);  
  }// end for 
}
//-------------------------------------------------------------------------------------------

void StickyfootCollectionManager::highlight_selected(boost::shared_ptr<visualization_utils::SelectionManager>  &selectionManager)
{
 for(sticky_feet_map_type_::iterator it = _feet.begin(); it!=_feet.end(); it++)
  {
      string no_selection = " ";
      string id = it->first;
      if(selectionManager->is_selected(id)){
        it->second._gl_foot->enable_whole_body_selection(true); 
        it->second._gl_foot->highlight_link(id); 
      }
  }// end for 
}
