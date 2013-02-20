#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "CandidateGraspSeedListener.hpp"
#include <algorithm>

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;

namespace renderer_affordances 
{
  //==================constructor / destructor

//   CandidateGraspSeedListener::CandidateGraspSeedListener(boost::shared_ptr<lcm::LCM> &lcm, RendererAffordances* affordance_renderer):
//     _lcm(lcm),
//     _parent_renderer(affordance_renderer)
  CandidateGraspSeedListener::CandidateGraspSeedListener(RendererAffordances* affordance_renderer):
    _parent_renderer(affordance_renderer)
  {
 
    _lcm = affordance_renderer->lcm; 
    //_collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());
    
    //lcm ok?
    if(!_lcm->good())
    {
      cerr << "\nLCM Not Good: Robot State Handler" << endl;
      return;
    }
    
    
    drc::desired_grasp_state_t msg;
    _handtype_id_map.clear();
    _handtype_id_map[msg.SANDIA_LEFT]=0; // maps handtype to id of glhand list
    _handtype_id_map[msg.SANDIA_RIGHT]=1;
     // load and pre-parse base_gl_hands for left and right hand
    shared_ptr<GlKinematicBody> new_hand;
    bool urdf_found=true;
    for (map<int,int>::iterator it=_handtype_id_map.begin(); it!=_handtype_id_map.end(); ++it)
    { 
     urdf_found = urdf_found && load_hand_urdf(it->first);
     if(urdf_found){
     new_hand =  shared_ptr<GlKinematicBody>(new GlKinematicBody(_urdf_xml_string));
      _gl_hand_list.push_back(new_hand);
     }
    }
    if(urdf_found){
    //Subscribe to CANDIDATE_GRASP_SEED 
    _lcm->subscribe("CANDIDATE_GRASP_SEED", &CandidateGraspSeedListener::handleDesiredGraspStateMsg, this); 
    }
    else{
     cerr << "##### ERROR: #####" <<  " sticky_hand urdfs not found in CandidateGraspSeedListener.cpp. Disabling candidate GraspSeedListener in renderer_affordances. Please update your models folder" << endl;    
    }
  }
 
  CandidateGraspSeedListener::~CandidateGraspSeedListener() {
    // _collision_detector->clear_collision_objects(); 
  }

//-------------------------------------------------------------------------------------      
//=============message callbacks

  void CandidateGraspSeedListener::handleDesiredGraspStateMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::desired_grasp_state_t* msg)						 
  {
    
    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = _parent_renderer->instantiated_objects.find(msg->object_name);

    if(it == _parent_renderer->instantiated_objects.end()) {
       cerr << " object_name " <<msg->object_name << " specified in CANDIDATE_GRASP_SEED channel is not available in cache." << endl; 
       return;
    }    
   
    KDL::Frame T_world_graspgeometry = KDL::Frame::Identity();  // take into account the world location of the graspgeometry later as the object may move.
    _object_name = msg->object_name;
    _geometry_name = msg->geometry_name;
    _grasp_type = msg->grasp_type;
    _optimization_status = msg->optimization_status;
    KDL::Frame T_graspgeometry_lhand,T_graspgeometry_rhand, T_world_lhand,T_world_rhand;
    
    T_graspgeometry_lhand.p[0] = msg->l_hand_pose.translation.x;
    T_graspgeometry_lhand.p[1] = msg->l_hand_pose.translation.y;
    T_graspgeometry_lhand.p[2] = msg->l_hand_pose.translation.z;
    T_graspgeometry_lhand.M = KDL::Rotation::Quaternion( msg->l_hand_pose.rotation.x, msg->l_hand_pose.rotation.y, msg->l_hand_pose.rotation.z, msg->l_hand_pose.rotation.w );
    
    T_graspgeometry_rhand.p[0] = msg->r_hand_pose.translation.x;
    T_graspgeometry_rhand.p[1] = msg->r_hand_pose.translation.y;
    T_graspgeometry_rhand.p[2] = msg->r_hand_pose.translation.z;
    T_graspgeometry_rhand.M = KDL::Rotation::Quaternion( msg->r_hand_pose.rotation.x, msg->r_hand_pose.rotation.y, msg->r_hand_pose.rotation.z, msg->r_hand_pose.rotation.w );
    
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
      
      //Request redraw
      bot_viewer_request_redraw(_parent_renderer->viewer);

  } // end handleMessage

//-------------------------------------------------------------------------------------        


  void CandidateGraspSeedListener::add_or_update_sticky_hand(int uid, string& unique_hand_name, KDL::Frame &T_world_hand, drc::joint_angles_t &posture_msg)		
  {
    
      typedef std::map<std::string,StickyHandStruc> sticky_hands_map_type;
      sticky_hands_map_type::iterator it = _parent_renderer->sticky_hands.find(unique_hand_name);
      if(it ==_parent_renderer->sticky_hands.end() ) // not in cache
      {
       
        StickyHandStruc sticky_hand_struc;
        sticky_hand_struc.object_name = _object_name ;
        sticky_hand_struc.geometry_name = _geometry_name;
        sticky_hand_struc.uid = uid; 
        sticky_hand_struc.opt_status = _optimization_status;
        sticky_hand_struc._collision_detector.reset();
        // should we have a global collision detector and add and remove objects to it as we create and delete objects and hands. We would have to manually add and remove multiple links to the collision detector.??
        // Each hand has its own collision detector for now.
        sticky_hand_struc._collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector()); 
          
       map<int,int>::iterator hand_it = _handtype_id_map.find(_grasp_type);
       if(hand_it!=_handtype_id_map.end())//exists in cache
        {
          sticky_hand_struc._gl_hand = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody((*_gl_hand_list[hand_it->second]),sticky_hand_struc._collision_detector,true,unique_hand_name));
        }
        else {
          bool urdf_found=load_hand_urdf(_grasp_type); // gets the urdf string from file
          sticky_hand_struc._gl_hand = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody (_urdf_xml_string,sticky_hand_struc._collision_detector,true,unique_hand_name)); 
      }
        
        sticky_hand_struc._gl_hand->set_state(T_world_hand, posture_msg);
        sticky_hand_struc.hand_type = _grasp_type;
        sticky_hand_struc.T_geometry_hand = T_world_hand;
        sticky_hand_struc.joint_name = posture_msg.joint_name;
        sticky_hand_struc.joint_position = posture_msg.joint_position;
        _parent_renderer->sticky_hands.insert(make_pair(unique_hand_name, sticky_hand_struc));
      }
      else {
        it->second._gl_hand->set_state(T_world_hand, posture_msg);
        it->second.T_geometry_hand = T_world_hand; 
        it->second.joint_position = posture_msg.joint_position;
      }
  }
  
  bool CandidateGraspSeedListener::load_hand_urdf(int grasp_type)
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
    else if ((grasp_type  == msg.IROBOT_LEFT)||(grasp_type  == msg.IROBOT_RIGHT)){
      filename ="irobot_hand";
      ext=".sdf";
    }
    found = std::find(_parent_renderer->urdf_filenames.begin(), _parent_renderer->urdf_filenames.end(), filename);
    if (found != _parent_renderer->urdf_filenames.end()) {
      cout << "\npre-loading hand urdf: " << filename << " for sticky hands rendering" << endl;
      cout << "--------------------------------------------------------------------" << endl;
      unsigned int index = found - _parent_renderer->urdf_filenames.begin();
      std::string filename=_parent_renderer->urdf_filenames[index];  
      oss << (*_parent_renderer->urdf_dir_name_ptr)  << filename << ext;
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



} //end namespace


