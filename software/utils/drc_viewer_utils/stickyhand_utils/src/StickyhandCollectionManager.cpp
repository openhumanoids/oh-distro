
#include "StickyhandCollectionManager.hpp"
#include <tinyxml.h>
using namespace visualization_utils;
using namespace collision;

//==================constructor / destructor

StickyhandCollectionManager::StickyhandCollectionManager(boost::shared_ptr<lcm::LCM> &lcm):
  _lcm(lcm),free_running_sticky_hand_cnt(0),_urdf_found(false)
{
  _urdf_dir_name = string(getModelsPath()) + "/mit_gazebo_models/mit_robot_hands/"; // getModelsPath gives /drc/software/build/models/
  cout << "StickyhandCollectionManager::searching for hand urdf files in: "<<  _urdf_dir_name << endl;
  get_URDF_filenames_from_dir(_urdf_dir_name.c_str(),_urdf_filenames);
  
  //lcm ok?
  if(!_lcm->good())
  {
    cerr << "\nLCM Not Good: Candidate Grasp Seed Listener" << endl;
    return;
  }
  _hands.clear();
  drc::desired_grasp_state_t msg;
  _handtype_id_map.clear();
  _handtype_id_map[msg.SANDIA_LEFT]= 0; // maps handtype to id of glhand list
  _handtype_id_map[msg.SANDIA_RIGHT]=1;
  _handtype_id_map[msg.IROBOT_LEFT]= 2; // maps handtype to id of glhand list
  _handtype_id_map[msg.IROBOT_RIGHT]=3;
   // load and pre-parse base_gl_hands for left and right hand
  boost::shared_ptr<GlKinematicBody> new_hand;
  _urdf_found=true;
  for (map<int,int>::iterator it=_handtype_id_map.begin(); it!=_handtype_id_map.end(); ++it)
  { 
   _urdf_found = _urdf_found && load_hand_urdf(it->first);
   if(_urdf_found){
   new_hand =  boost::shared_ptr<GlKinematicBody>(new GlKinematicBody(_urdf_xml_string));
    _gl_hand_list.push_back(new_hand);
   }
  }
}
//-------------------------------------------------------------------------------------------  
StickyhandCollectionManager::~StickyhandCollectionManager() {

}

//-------------------------------------------------------------------------------------------
bool StickyhandCollectionManager::is_parent_object(std::string& object_name)
{
  sticky_hands_map_type_::iterator hand_it = _hands.begin();
  while (hand_it!=_hands.end()) 
  {
     if (hand_it->second.object_name == (object_name))
     {
       return true;
     }
     
     hand_it++;
  } 
  return false;
}
//-------------------------------------------------------------------------------------------
bool StickyhandCollectionManager::remove(std::string& id) {
    sticky_hands_map_type_::iterator hand_it = _hands.find(id);
    if(hand_it!=_hands.end()){
        _hands.erase(hand_it);
        return true;
    }
    return false;
}
//-------------------------------------------------------------------------------------------
bool StickyhandCollectionManager::remove_selected(boost::shared_ptr<visualization_utils::SelectionManager>  &selectionManager) {

  sticky_hands_map_type_::iterator hand_it = _hands.begin();
  while (hand_it!=_hands.end()) 
  {
     string id = hand_it->first;
     if(selectionManager->is_selected(id))
     { 
         selectionManager->remove(id);
        _hands.erase(hand_it++);
     }
     else
        hand_it++;
  }
}

//-------------------------------------------------------------------------------------------
void StickyhandCollectionManager::remove_seeds(std::string& obj_id,boost::shared_ptr<visualization_utils::AffordanceCollectionManager>  &affCollectionManager) {

  sticky_hands_map_type_::iterator hand_it = _hands.begin();
  while (hand_it!=_hands.end()) 
  {
     string id = hand_it->first;
     if (hand_it->second.object_name == obj_id)
     { 
        _hands.erase(hand_it++);
     }
     else
        hand_it++;
  }
}

//-------------------------------------------------------------------------------------------
bool StickyhandCollectionManager::store(std::string& id,bool unstore,
                                         boost::shared_ptr<visualization_utils::AffordanceCollectionManager>  &affCollectionManager) {
    sticky_hands_map_type_::iterator hand_it = _hands.find(id);
    if(hand_it!=_hands.end()){
      object_instance_map_type_::iterator obj_it = affCollectionManager->_objects.find(string(hand_it->second.object_name));
      // get otdf name
      std::string otdf_models_path = std::string(getModelsPath()) + "/otdf/"; 
      std::string filepath =  otdf_models_path + obj_it->second.otdf_type +".otdf";

      // get variables to create grasp_seed xml
      GraspSeed graspSeed;
      graspSeed.appType = GraspSeed::HAND;
      KDL::Frame& geo = hand_it->second.T_geometry_hand;
      graspSeed.xyz[0] = geo.p.x(), graspSeed.xyz[1] = geo.p.y(), graspSeed.xyz[2] = geo.p.z();
      geo.M.GetRPY(graspSeed.rpy[0],graspSeed.rpy[1],graspSeed.rpy[2]);
      graspSeed.geometry_name = hand_it->second.geometry_name;
      graspSeed.grasp_type = hand_it->second.hand_type;
      graspSeed.joint_names = hand_it->second.joint_name;
      graspSeed.joint_positions = hand_it->second.joint_position;
      if(unstore) 
        graspSeed.unstoreFromOtdf(filepath);
      else        
        graspSeed.writeToOtdf(filepath);
      return true;
    }    
    return false;
}
//-------------------------------------------------------------------------------------------
bool StickyhandCollectionManager::store_selected(boost::shared_ptr<visualization_utils::SelectionManager>  &selectionManager,bool unstore,
                                         boost::shared_ptr<visualization_utils::AffordanceCollectionManager>  &affCollectionManager) {
  for(sticky_hands_map_type_::iterator it = _hands.begin(); it!=_hands.end(); it++)
  {
      string id = it->first;
      if(selectionManager->is_selected(id)){
        store(id,unstore,affCollectionManager);
      }
  }// end for 
}  
//-------------------------------------------------------------------------------------------  
void StickyhandCollectionManager::load_stored(OtdfInstanceStruc& instance_struc)
{

    std::vector<GraspSeed>& list = instance_struc._otdf_instance->graspSeedList_;
    
    for(int i=0; i<list.size(); i++) {
        if(list[i].appType == GraspSeed::HAND){
          std::stringstream objname;
          objname << instance_struc.otdf_type << "_"<< instance_struc.uid; 
          int uid =free_running_sticky_hand_cnt++;

          drc::position_3d_t pose;
          pose.translation.x = list[i].xyz[0];
          pose.translation.y = list[i].xyz[1];
          pose.translation.z = list[i].xyz[2];
          KDL::Rotation rot(KDL::Rotation::RPY(list[i].rpy[0],list[i].rpy[1],list[i].rpy[2]));
          rot.GetQuaternion(pose.rotation.x, pose.rotation.y,
                            pose.rotation.z, pose.rotation.w);

          drc::desired_grasp_state_t msg;
          msg.utime = -1;
          msg.robot_name = "TODO";
          msg.object_name = objname.str();
          msg.geometry_name = list[i].geometry_name;
          msg.unique_id = uid;
          msg.grasp_type = list[i].grasp_type;
          msg.power_grasp = false;
          msg.num_r_joints = 0;
          msg.num_l_joints = 0;
          if(msg.grasp_type == msg.SANDIA_LEFT || msg.grasp_type == msg.SANDIA_BOTH 
            || msg.grasp_type == msg.IROBOT_LEFT || msg.grasp_type == msg.IROBOT_BOTH){
            msg.num_l_joints = list[i].joint_positions.size();
            msg.l_hand_pose = pose;
            msg.l_joint_name = list[i].joint_names;
            msg.l_joint_position = list[i].joint_positions;
          }else if(msg.grasp_type == msg.SANDIA_RIGHT || msg.grasp_type == msg.SANDIA_BOTH 
            || msg.grasp_type == msg.IROBOT_RIGHT || msg.grasp_type == msg.IROBOT_BOTH){
            msg.num_r_joints = list[i].joint_positions.size();
            msg.r_hand_pose = pose;
            msg.r_joint_name = list[i].joint_names;
            msg.r_joint_position = list[i].joint_positions;
          }else{
            cout << "add_new_otdf_object_instance error. grasp_type not recognized: " << msg.grasp_type << endl;
          }
          msg.optimization_status = drc::desired_grasp_state_t::SUCCESS;
          _lcm->publish("CANDIDATE_GRASP",&msg);
        }
    }

}

//-------------------------------------------------------------------------------------------
void StickyhandCollectionManager::add_or_update(const drc::desired_grasp_state_t* msg)		
{

    _object_name = msg->object_name;
    _geometry_name = msg->geometry_name;
    _grasp_type = msg->grasp_type;
    _optimization_status = msg->optimization_status;
   
     KDL::Frame T_graspgeometry_lhand,T_graspgeometry_rhand, T_world_lhand,T_world_rhand;
    //NOTE: the msg received from grasp opt is in a base frame and not l_hand/r_hand frame.
    // For visualization of sticky hands it is important to keep in base frame as we are using the 
    // same urdf that we used for optimization.
          
    T_graspgeometry_lhand.p[0] = msg->l_hand_pose.translation.x;
    T_graspgeometry_lhand.p[1] = msg->l_hand_pose.translation.y;
    T_graspgeometry_lhand.p[2] = msg->l_hand_pose.translation.z;
    T_graspgeometry_lhand.M = KDL::Rotation::Quaternion( msg->l_hand_pose.rotation.x, msg->l_hand_pose.rotation.y, msg->l_hand_pose.rotation.z, msg->l_hand_pose.rotation.w );
    
    T_graspgeometry_rhand.p[0] = msg->r_hand_pose.translation.x;
    T_graspgeometry_rhand.p[1] = msg->r_hand_pose.translation.y;
    T_graspgeometry_rhand.p[2] = msg->r_hand_pose.translation.z;
    T_graspgeometry_rhand.M = KDL::Rotation::Quaternion( msg->r_hand_pose.rotation.x, msg->r_hand_pose.rotation.y, msg->r_hand_pose.rotation.z, msg->r_hand_pose.rotation.w );
    
    KDL::Frame T_world_graspgeometry = KDL::Frame::Identity();  // take into account the world location of the graspgeometry later as the object may move.    
    T_world_lhand = T_world_graspgeometry*T_graspgeometry_lhand;
    T_world_rhand = T_world_graspgeometry*T_graspgeometry_rhand;
    
    std::string  unique_hand_name;
    if((msg->grasp_type  == msg->SANDIA_LEFT)||(msg->grasp_type  == msg->IROBOT_LEFT))
    {
      drc::joint_angles_t posture_msg;
      posture_msg.utime = msg->utime;
      posture_msg.robot_name= msg->robot_name;
      posture_msg.num_joints= msg->num_l_joints;
      posture_msg.joint_name= msg->l_joint_name;
      posture_msg.joint_position= msg->l_joint_position;
      std::stringstream oss;
      oss << msg->object_name <<"_"<< msg->geometry_name << "_lgrasp_" << msg->unique_id;
      unique_hand_name = oss.str(); 
      
      add_or_update_sticky_hand(msg->unique_id,unique_hand_name, T_world_lhand, posture_msg);
    
    }
    else if((msg->grasp_type  == msg->SANDIA_RIGHT)||(msg->grasp_type  == msg->IROBOT_RIGHT))
    {
      
      drc::joint_angles_t posture_msg;
      posture_msg.utime = msg->utime;
      posture_msg.robot_name= msg->robot_name;
      posture_msg.num_joints= msg->num_r_joints;
      posture_msg.joint_name= msg->r_joint_name;
      posture_msg.joint_position= msg->r_joint_position;
      std::stringstream oss;
      oss << msg->object_name <<"_"<< msg->geometry_name  << "_rgrasp_" << msg->unique_id;
      unique_hand_name = oss.str(); 
      
      add_or_update_sticky_hand(msg->unique_id,unique_hand_name, T_world_rhand, posture_msg);
    
    }
    else if((msg->grasp_type  == msg->SANDIA_BOTH)||(msg->grasp_type  == msg->IROBOT_BOTH))
    {
      drc::joint_angles_t posture_msg;
      posture_msg.utime = msg->utime;
      posture_msg.robot_name= msg->robot_name;
      posture_msg.num_joints= msg->num_l_joints;
      posture_msg.joint_name= msg->l_joint_name;
      posture_msg.joint_position= msg->l_joint_position;
      std::stringstream oss;
      oss << msg->object_name <<"_"<< msg->geometry_name << "_lgrasp_" << msg->unique_id;
      unique_hand_name = oss.str(); 

      add_or_update_sticky_hand(msg->unique_id, unique_hand_name, T_world_lhand, posture_msg);
      
      posture_msg.num_joints= msg->num_r_joints;
      posture_msg.joint_name= msg->r_joint_name;
      posture_msg.joint_position= msg->r_joint_position;
      oss << msg->object_name <<"_"<< msg->geometry_name  << "_rgrasp_" << msg->unique_id;
      unique_hand_name = oss.str(); 

      add_or_update_sticky_hand(msg->unique_id,unique_hand_name, T_world_rhand, posture_msg);
      
    }

}
//-------------------------------------------------------------------------------------------
void StickyhandCollectionManager::add_or_update_sticky_hand(int uid, string& unique_hand_name, KDL::Frame &T_world_hand, drc::joint_angles_t &posture_msg)		
{

  
    sticky_hands_map_type_::iterator it = _hands.find(unique_hand_name);
    if(it ==_hands.end() ) // not in cache
    {
     
      StickyHandStruc sticky_hand_struc;
      sticky_hand_struc.object_name = _object_name;//.c_str();
      sticky_hand_struc.geometry_name = _geometry_name;//.c_str();
      sticky_hand_struc.uid = uid; 
      sticky_hand_struc.opt_status = _optimization_status;
      sticky_hand_struc._collision_detector.reset();
      // should we have a global collision detector and add and remove objects to it as we create and delete objects and hands. We would have to manually add and remove multiple links to the collision detector.??
      // Each hand has its own collision detector for now.
      sticky_hand_struc._collision_detector = boost::shared_ptr<Collision_Detector>(new Collision_Detector()); 
        
     map<int,int>::iterator hand_it = _handtype_id_map.find(_grasp_type);
     if(hand_it!=_handtype_id_map.end())//exists in cache
      {
        sticky_hand_struc._gl_hand = boost::shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody((*_gl_hand_list[hand_it->second]),sticky_hand_struc._collision_detector,true,unique_hand_name));
      }
      else {
        bool urdf_found=load_hand_urdf(_grasp_type); // gets the urdf string from file
        sticky_hand_struc._gl_hand = boost::shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody (_urdf_xml_string,sticky_hand_struc._collision_detector,true,unique_hand_name)); 
      }
      
      sticky_hand_struc._gl_hand->set_state(T_world_hand, posture_msg);
      sticky_hand_struc.hand_type = _grasp_type;
      sticky_hand_struc.T_geometry_hand = T_world_hand;
       

      /*double ro,pi,ya;  
      T_world_hand.M.GetRPY(ro,pi,ya);
      cout <<"adding sticky hand \n T_geometry_hand : "<< endl;
      cout <<"roll "<<ro*(180/M_PI) << endl;
      cout <<"pitch "<<pi*(180/M_PI) << endl;
      cout <<"yaw "<<ya*(180/M_PI) << endl;*/

      sticky_hand_struc.joint_name = posture_msg.joint_name;
      sticky_hand_struc.joint_position = posture_msg.joint_position;
      sticky_hand_struc.optimized_T_geometry_hand = T_world_hand;
      sticky_hand_struc.optimized_joint_position = posture_msg.joint_position;
      _hands.insert(make_pair(unique_hand_name, sticky_hand_struc));
    }
    else {
      cout <<"update sticky hand \n";
      it->second._gl_hand->set_state(T_world_hand, posture_msg);
      cout <<"end update sticky hand \n";
      it->second.T_geometry_hand = T_world_hand; 
      it->second.joint_position = posture_msg.joint_position;
      it->second.optimized_T_geometry_hand = T_world_hand;
      it->second.optimized_joint_position = posture_msg.joint_position;
    }
}
//-------------------------------------------------------------------------------------------
bool StickyhandCollectionManager::load_hand_urdf(int grasp_type)
{
  drc::desired_grasp_state_t msg;
  std::stringstream oss;
  std::vector<std::string>::const_iterator found;
  std::string filename, ext;
  if (grasp_type  == msg.SANDIA_LEFT){
    filename ="sandia_hand_left";
    ext=".urdf";
  }
  else if (grasp_type  == msg.SANDIA_RIGHT){
    filename ="sandia_hand_right";
    ext=".urdf";
  }
  else if (grasp_type  == msg.IROBOT_LEFT){
    filename ="irobot_hand_left";
    ext=".urdf";
  }
  else if (grasp_type  == msg.IROBOT_RIGHT){
    filename ="irobot_hand_right";
    ext=".urdf";
  }
  found = std::find(_urdf_filenames.begin(), _urdf_filenames.end(), filename);
  if (found != _urdf_filenames.end()) {
    cout << "\npre-loading hand urdf: " << filename << " for sticky hands rendering" << endl;
    cout << "--------------------------------------------------------------------" << endl;
    unsigned int index = found - _urdf_filenames.begin();
    std::string filename=_urdf_filenames[index];  
    oss << _urdf_dir_name << filename << ext;
    std::string xml_string;
    get_xmlstring_from_file(oss.str(),xml_string);
    _urdf_xml_string = xml_string;
  } 
  else{
     cerr << "ERROR: " << filename << " not found in CandidateGraspSeedListener.cpp" << endl;
     return false;
   }
  return true;
 }

//-------------------------------------------------------------------------------------------
void StickyhandCollectionManager::get_motion_constraints(string object_name, OtdfInstanceStruc& obj, bool is_retractable,
                                                  map<string, vector<KDL::Frame> > &ee_frames_map, 
                                                  map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                                  map<string, vector<double> > &joint_pos_map,
                                                  map<string, vector<int64_t> > &joint_pos_timestamps_map)
                                                  
{
    // Publish time indexed ee motion constraints from associated sticky hands 
    for(sticky_hands_map_type_::const_iterator hand_it = _hands.begin(); hand_it!=_hands.end(); hand_it++)
    {
      string host_name = hand_it->second.object_name;
      if (host_name == (object_name))
      {
          string ee_name;
          if(hand_it->second.hand_type==drc::desired_grasp_state_t::SANDIA_LEFT)
              ee_name ="left_palm";    
          else if(hand_it->second.hand_type==drc::desired_grasp_state_t::SANDIA_RIGHT)   
              ee_name ="right_palm";    
          else if(hand_it->second.hand_type==drc::desired_grasp_state_t::IROBOT_LEFT)
              ee_name ="left_base_link";    
          else if(hand_it->second.hand_type==drc::desired_grasp_state_t::IROBOT_RIGHT)   
              ee_name ="right_base_link";    
          else
              cout << "unknown hand_type in on_otdf_dof_range_widget_popup_close\n";   

          // if ee_name already exists in ee_frames_map, redundant ee_frames
          // e.g two right sticky hands on the same object.
          map<string, vector<KDL::Frame> >::const_iterator ee_it = ee_frames_map.find(ee_name);
          if(ee_it!=ee_frames_map.end()){
              cerr<<" ERROR: Cannot of two seeds of the same ee. Please consider deleting redundant seeds\n";
              return;   
          }

          //======================     
          KDL::Frame  T_geometry_hand = hand_it->second.T_geometry_hand;
          KDL::Frame  T_geometry_palm = KDL::Frame::Identity(); 
          if(!hand_it->second._gl_hand->get_link_frame(ee_name,T_geometry_palm))
              cout <<"ERROR: ee link "<< ee_name << " not found in sticky hand urdf"<< endl;
          KDL::Frame T_hand_palm = T_geometry_hand.Inverse()*T_geometry_palm; // offset

          KDL::Frame T_world_object = obj._gl_object->_T_world_body;

          int num_frames =  hand_it->second._gl_hand->_desired_body_motion_history.size();
          vector<KDL::Frame> T_world_ee_frames;
          vector<int64_t> frame_timestamps;
          for(uint i = 0; i < (uint) num_frames; i++) {
              KDL::Frame T_object_hand = hand_it->second._gl_hand->_desired_body_motion_history[i];
              KDL::Frame T_world_hand = T_world_object*T_object_hand;
              KDL::Frame T_world_ee = T_world_hand*T_hand_palm;//T_world_palm ; TODO: Eventually will be in object frame
              T_world_ee_frames.push_back(T_world_ee);
              int64_t timestamp=(int64_t)i*1000000;
              frame_timestamps.push_back(timestamp);   
          }
          // append reverse motion with a small back up palm offset
          if(is_retractable){
              for(uint i = 0; i < (uint) num_frames; i++) {
                  KDL::Frame T_object_hand = hand_it->second._gl_hand->_desired_body_motion_history[num_frames-1-i];                   
                  KDL::Frame T_world_hand = T_world_object*T_object_hand;
                  KDL::Frame T_world_ee = T_world_hand*T_hand_palm;//T_world_palm ; TODO: Eventually will be in object frame                             
                  KDL::Frame T_palm_hand = T_geometry_palm.Inverse()*T_geometry_hand; //this should be T_palm_base    
                  KDL::Vector handframe_offset;
                  handframe_offset[0]=0.05;handframe_offset[1]=0;handframe_offset[2]=0;
                  KDL::Vector palmframe_offset= T_palm_hand*handframe_offset;
                  KDL::Vector worldframe_offset=T_world_ee.M*palmframe_offset;
                  T_world_ee.p += worldframe_offset;   
                  T_world_ee_frames.push_back(T_world_ee);
                  int64_t timestamp=(int64_t)(num_frames+i)*1000000;
                  frame_timestamps.push_back(timestamp);   
              } 
          }
          //===================         

          ee_frames_map.insert(make_pair(ee_name, T_world_ee_frames));
          ee_frame_timestamps_map.insert(make_pair(ee_name, frame_timestamps));   
          // hand_it->second.joint_name;  
          // hand_it->second.joint_position; 

            for(uint k = 0; k< (uint) hand_it->second.joint_name.size(); k++) 
            {
              std::string joint_name = hand_it->second.joint_name[k];
              vector<double> joint_pos;
              vector<int64_t> joint_pos_timestamps;
              uint i=0;
              //for(uint i = 0; i < (uint) num_frames; i++)
              //{ 
              joint_pos.push_back(hand_it->second.joint_position[k]);
              int64_t timestamp=(int64_t)i*1000000;
              joint_pos_timestamps.push_back(timestamp);
              //}
              // append reverse motion with a small back up palm offset
              if(is_retractable){
                //for(uint i = 0; i < (uint) num_frames; i++)
                    //{
                    //joint_pos.push_back(0.5*hand_it->second.joint_position[k]); % dont open fully, just loosen hold until half way
                    joint_pos.push_back(0.0);

                    int64_t timestamp=(int64_t)(num_frames+i)*1000000;
                    joint_pos_timestamps.push_back(timestamp);   
                    //} 
                }
                joint_pos_map.insert(make_pair(joint_name,joint_pos));
                joint_pos_timestamps_map.insert(make_pair(joint_name, joint_pos_timestamps));  
            }

      } // end if (host_name == (it->first))
    } // end for sticky hands       

};

//-------------------------------------------------------------------------------------------
void StickyhandCollectionManager::get_pose_constraints(string object_name, OtdfInstanceStruc& obj, bool to_future_state,
                                                  map<string, vector<KDL::Frame> > &ee_frames_map, 
                                                  map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                                  map<string, vector<double> > &joint_pos_map,
                                                  map<string, vector<int64_t> > &joint_pos_timestamps_map)
                                                  
{

      for(sticky_hands_map_type_::const_iterator hand_it = _hands.begin(); hand_it!=_hands.end(); hand_it++)
      {
          string host_name = hand_it->second.object_name;
          if (host_name == (object_name))
              {
                  string ee_name;
                  if(hand_it->second.hand_type==drc::desired_grasp_state_t::SANDIA_LEFT)
                      ee_name ="left_palm";    
                  else if(hand_it->second.hand_type==drc::desired_grasp_state_t::SANDIA_RIGHT)   
                      ee_name ="right_palm";  
                  else if(hand_it->second.hand_type==drc::desired_grasp_state_t::IROBOT_LEFT)
                      ee_name ="left_base_link";    
                  else if(hand_it->second.hand_type==drc::desired_grasp_state_t::IROBOT_RIGHT)   
                      ee_name ="right_base_link";                          
                  else
                      cout << "unknown hand_type in StickyhandCollectionManager::get_pose_constraints\n";   
         
                  // if ee_name already exists in ee_frames_map, redundant ee_frames
                  // e.g two right sticky hands on the same object.
                  map<string, vector<KDL::Frame> >::const_iterator ee_it = ee_frames_map.find(ee_name);
                  if(ee_it!=ee_frames_map.end()){
                      cerr<<" ERROR: Cannot of two seeds of the same ee. Please consider deleting redundant seeds\n";
                      return;   
                  }
     
                  //======================     
                  KDL::Frame  T_geometry_hand = hand_it->second.T_geometry_hand;
                  KDL::Frame  T_geometry_palm = KDL::Frame::Identity(); 
                  if(!hand_it->second._gl_hand->get_link_frame(ee_name,T_geometry_palm))
                      cout <<"ERROR: ee link "<< ee_name << " not found in sticky hand urdf"<< endl;
                  KDL::Frame T_hand_palm = T_geometry_hand.Inverse()*T_geometry_palm; // offset

                  KDL::Frame T_world_object = KDL::Frame::Identity();
                  KDL::Frame T_world_graspgeometry = KDL::Frame::Identity();
                  if(!to_future_state){
                      T_world_object = obj._gl_object->_T_world_body;
                    // the object might have moved.
                     if(!obj._gl_object->get_link_geometry_frame(string(hand_it->second.geometry_name),T_world_graspgeometry))
                        cerr << " failed to retrieve " << hand_it->second.geometry_name<<" in object " << hand_it->second.object_name <<endl;
                  }
                  else {
                    T_world_object = obj._gl_object->_T_world_body_future;
                    // the object might have moved.
                     if(!obj._gl_object->get_link_geometry_future_frame(string(hand_it->second.geometry_name),T_world_graspgeometry))
                        cerr << " failed to retrieve " << hand_it->second.geometry_name<<" in object " << hand_it->second.object_name <<endl;        
                  }

                  int num_frames = 1;
                  vector<KDL::Frame> T_world_ee_frames;
                  vector<int64_t> frame_timestamps;
                  for(uint i = 0; i < (uint) num_frames; i++) {
                      KDL::Frame  T_world_ee = T_world_graspgeometry*T_geometry_palm;   
                      KDL::Frame T_palm_hand = T_geometry_palm.Inverse()*T_geometry_hand; //this should be T_palm_base    
                      KDL::Vector handframe_offset;
                      handframe_offset[0]=0.01;handframe_offset[1]=0;handframe_offset[2]=0;
                      KDL::Vector palmframe_offset= T_palm_hand*handframe_offset;
                      KDL::Vector worldframe_offset=T_world_ee.M*palmframe_offset;
                      T_world_ee.p += worldframe_offset;                    
                      
                      T_world_ee_frames.push_back(T_world_ee);
                      int64_t timestamp=(int64_t)i*1000000;
                      frame_timestamps.push_back(timestamp);   
                  }
                  //===================         

                  ee_frames_map.insert(make_pair(ee_name, T_world_ee_frames));
                  ee_frame_timestamps_map.insert(make_pair(ee_name, frame_timestamps));   
                  // hand_it->second.joint_name;  
                  // hand_it->second.joint_position; 
     
                    for(uint k = 0; k< (uint) hand_it->second.joint_name.size(); k++) 
                    {
                      std::string joint_name = hand_it->second.joint_name[k];
                      vector<double> joint_pos;
                      vector<int64_t> joint_pos_timestamps;
                      uint i=0;
                      joint_pos.push_back(hand_it->second.joint_position[k]);
                      int64_t timestamp=(int64_t)i*1000000;
                      joint_pos_timestamps.push_back(timestamp);
                      joint_pos_map.insert(make_pair(joint_name,joint_pos));
                      joint_pos_timestamps_map.insert(make_pair(joint_name, joint_pos_timestamps));  
                    }
     
              } // end if (host_name == (it->first))
      } // end for sticky hands
      
}// end method
//-------------------------------------------------------------------------------------------

void StickyhandCollectionManager::get_time_ordered_pose_constraints(boost::shared_ptr<visualization_utils::AffordanceCollectionManager>  &affCollectionManager, bool to_future_state,
                                                  boost::shared_ptr<visualization_utils::SelectionManager>  &selectionManager,
                                                  map<string, vector<KDL::Frame> > &ee_frames_map, 
                                                  map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                                  map<string, vector<double> > &joint_pos_map,
                                                  map<string, vector<int64_t> > &joint_pos_timestamps_map)
                                                  
{

      for(sticky_hands_map_type_::const_iterator hand_it = _hands.begin(); hand_it!=_hands.end(); hand_it++)
      {
          string host_name = hand_it->second.object_name;
          object_instance_map_type_::const_iterator obj_it = affCollectionManager->_objects.find(host_name);
          string id = hand_it->first;
          if ((selectionManager->is_selected(id))&&(obj_it!=affCollectionManager->_objects.end()))
          {
              string ee_name;
              if(hand_it->second.hand_type==drc::desired_grasp_state_t::SANDIA_LEFT)
                  ee_name ="left_palm";    
              else if(hand_it->second.hand_type==drc::desired_grasp_state_t::SANDIA_RIGHT)   
                  ee_name ="right_palm";  
              else if(hand_it->second.hand_type==drc::desired_grasp_state_t::IROBOT_LEFT)
                  ee_name ="left_base_link";    
              else if(hand_it->second.hand_type==drc::desired_grasp_state_t::IROBOT_RIGHT)   
                  ee_name ="right_base_link";     
              else
                  cout << "unknown hand_type in StickyhandCollectionManager::get_time_ordered_pose_constraints\n";   
      
             
              //======================     
              KDL::Frame  T_geometry_hand = hand_it->second.T_geometry_hand;
              KDL::Frame  T_geometry_palm = KDL::Frame::Identity(); 
              if(!hand_it->second._gl_hand->get_link_frame(ee_name,T_geometry_palm))
                  cout <<"ERROR: ee link "<< ee_name << " not found in sticky hand urdf"<< endl;
              KDL::Frame T_hand_palm = T_geometry_hand.Inverse()*T_geometry_palm; // offset

              KDL::Frame T_world_object = KDL::Frame::Identity();
              KDL::Frame T_world_graspgeometry = KDL::Frame::Identity();
              if(!to_future_state){
                  T_world_object = obj_it->second._gl_object->_T_world_body;
                // the object might have moved.
                 if(!obj_it->second._gl_object->get_link_geometry_frame(string(hand_it->second.geometry_name),T_world_graspgeometry))
                    cerr << " failed to retrieve " << hand_it->second.geometry_name<<" in object " << hand_it->second.object_name <<endl;
              }
              else {
                T_world_object = obj_it->second._gl_object->_T_world_body_future;
                // the object might have moved.
                 if(!obj_it->second._gl_object->get_link_geometry_future_frame(string(hand_it->second.geometry_name),T_world_graspgeometry))
                    cerr << " failed to retrieve " << hand_it->second.geometry_name<<" in object " << hand_it->second.object_name <<endl;        
              }

              int num_frames = 1;
              vector<KDL::Frame> T_world_ee_frames;
              vector<int64_t> frame_timestamps;
              for(uint i = 0; i < (uint) num_frames; i++) {
                  KDL::Frame  T_world_ee = T_world_graspgeometry*T_geometry_palm;   
                  KDL::Frame T_palm_hand = T_geometry_palm.Inverse()*T_geometry_hand; //this should be T_palm_base    
                  KDL::Vector handframe_offset;
                  handframe_offset[0]=0.01;handframe_offset[1]=0;handframe_offset[2]=0;
                  KDL::Vector palmframe_offset= T_palm_hand*handframe_offset;
                  KDL::Vector worldframe_offset=T_world_ee.M*palmframe_offset;
                  T_world_ee.p += worldframe_offset;                    
                  
                  T_world_ee_frames.push_back(T_world_ee);
                  double nmr = (selectionManager->get_selection_order(id)-1);
                  double dmr = (selectionManager->get_selection_cnt()-1);
                  int64_t timestamp = (int64_t)((nmr/dmr)*1000000);
                  frame_timestamps.push_back(timestamp);   
              }
              //===================         

              std::stringstream oss;
              oss << ee_name << "::" << selectionManager->get_selection_order(id);
              std::string unique_ee_name = oss.str();
              ee_frames_map.insert(make_pair(unique_ee_name, T_world_ee_frames));
              ee_frame_timestamps_map.insert(make_pair(unique_ee_name, frame_timestamps));   
              // hand_it->second.joint_name;  
              // hand_it->second.joint_position; 

                for(uint k = 0; k< (uint) hand_it->second.joint_name.size(); k++) 
                {
                  std::string joint_name = hand_it->second.joint_name[k];
                  std::stringstream oss2;
                  oss2 << joint_name << "::" << selectionManager->get_selection_order(id);
                  std::string unique_joint_name = oss2.str();
                  vector<double> joint_pos;
                  vector<int64_t> joint_pos_timestamps;
                  joint_pos.push_back(hand_it->second.joint_position[k]);
                  double nmr = (selectionManager->get_selection_order(id)-1);
                  double dmr = (selectionManager->get_selection_cnt()-1);
                  int64_t timestamp = (int64_t)((nmr/dmr)*1000000);
                  joint_pos_timestamps.push_back(timestamp);
                  joint_pos_map.insert(make_pair(unique_joint_name,joint_pos));
                  joint_pos_timestamps_map.insert(make_pair(unique_joint_name, joint_pos_timestamps));  
                }

          } // end if (host_name == (object_na,me))
      } // end for sticky hands
      
}// end method
//-------------------------------------------------------------------------------------------
void StickyhandCollectionManager::get_aff_indexed_ee_constraints(string& object_name, OtdfInstanceStruc& obj,
                                                                 boost::shared_ptr<BatchFKQueryHandler>  &dofRangeFkQueryHandler,
                                                                 map<string, vector<KDL::Frame> > &ee_frames_map,
                                                                 map<string, vector<drc::affordance_index_t> > &ee_frame_affindices_map)
{
  for(sticky_hands_map_type_::const_iterator hand_it = _hands.begin(); hand_it!=_hands.end(); hand_it++) 
  {
    
      string host_name = hand_it->second.object_name;
      if (host_name == (object_name)) {

          string ee_name;
          if(hand_it->second.hand_type==drc::desired_grasp_state_t::SANDIA_LEFT)
              ee_name ="left_palm";    
          else if(hand_it->second.hand_type==drc::desired_grasp_state_t::SANDIA_RIGHT)   
              ee_name ="right_palm";  
          else if(hand_it->second.hand_type==drc::desired_grasp_state_t::IROBOT_LEFT)
              ee_name ="left_base_link";    
          else if(hand_it->second.hand_type==drc::desired_grasp_state_t::IROBOT_RIGHT)   
              ee_name ="right_base_link";      
          else
              cout << "unknown hand_type in on_otdf_dof_range_widget_popup_close\n";   
          
          // if ee_name already exists in ee_frames_map, redundant ee_frames
          // e.g two right sticky hands on the same object.
          map<string, vector<KDL::Frame> >::const_iterator ee_it = ee_frames_map.find(ee_name);
          if(ee_it!=ee_frames_map.end()){
              cerr<<" ERROR: Cannot have two seeds of the same ee on the same affordance. Please consider deleting redundant seeds\n";
              return;   
          }
          
          KDL::Frame  T_geometry_ee = KDL::Frame::Identity(); 
          if(!hand_it->second._gl_hand->get_link_frame(ee_name,T_geometry_ee))
              cout <<"ERROR: ee link "<< ee_name << " not found in sticky hand urdf"<< endl;
          
          
          vector<KDL::Frame> T_world_geometry_frames;        
          string seed_geometry_name = hand_it->second.geometry_name;
          dofRangeFkQueryHandler->getLinkFrames(seed_geometry_name,T_world_geometry_frames);
          std::string dof_name;
          vector<double> dof_values;  
          bool isdofset =dofRangeFkQueryHandler->getAssociatedDoFNameAndVal(seed_geometry_name,dof_name,dof_values);
          
          // dof range was set
          if(isdofset) {
              int num_of_incs = T_world_geometry_frames.size();
              vector<KDL::Frame> T_world_ee_frames;
              vector<drc::affordance_index_t> frame_affindices;
              for(size_t i=0;i<(size_t)num_of_incs;i++) {
                  KDL::Frame  T_world_ee = T_world_geometry_frames[i]*T_geometry_ee;     
                  T_world_ee_frames.push_back(T_world_ee);
                  drc::affordance_index_t aff_index;
                  aff_index.utime=(int64_t)i;
                  aff_index.aff_type = obj.otdf_type; 
                  aff_index.aff_uid = obj.uid;   
                  aff_index.num_ees = 1;  
                  aff_index.ee_name.push_back(ee_name); 
                  aff_index.dof_name.push_back(dof_name);     
                  aff_index.dof_value.push_back(dof_values[i]); 
                  
                  frame_affindices.push_back(aff_index);
              } 
              ee_frames_map.insert(make_pair(ee_name, T_world_ee_frames));
              ee_frame_affindices_map.insert(make_pair(ee_name, frame_affindices));   
          }// end if dofset
      } // end if (host_name == (object_name))
  }// end for sticky hands   
} // end method

//-------------------------------------------------------------------------------------------
void StickyhandCollectionManager::clear_highlights()
{
  for(sticky_hands_map_type_::iterator it = _hands.begin(); it!=_hands.end(); it++)
  {
      string no_selection = " ";
      it->second._gl_hand->highlight_link(no_selection); // clear selection 
      it->second._gl_hand->highlight_marker(no_selection);  
  }// end for 
} 
//-------------------------------------------------------------------------------------------

void StickyhandCollectionManager::highlight_selected(boost::shared_ptr<visualization_utils::SelectionManager>  &selectionManager)
{
 for(sticky_hands_map_type_::iterator it = _hands.begin(); it!=_hands.end(); it++)
  {
      string no_selection = " ";
      string id = it->first;
      if(selectionManager->is_selected(id)){
        it->second._gl_hand->enable_whole_body_selection(true); 
        it->second._gl_hand->highlight_link(id); 
        
      }
  }// end for 
}
