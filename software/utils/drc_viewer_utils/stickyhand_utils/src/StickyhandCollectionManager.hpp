#ifndef STICKY_HAND_COLLECTION_MANAGER_HPP
#define STICKY_HAND_COLLECTION_MANAGER_HPP


#include "sticky_hand_utils.hpp"
#include <visualization_utils/affordance_utils/BatchFKQueryHandler.hpp>
#include <visualization_utils/SelectionManager.hpp>
using namespace std;

namespace visualization_utils
{
  typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
  // wraps cache of sticky_hands
  class StickyhandCollectionManager 
  {
    public:
     StickyhandCollectionManager(boost::shared_ptr<lcm::LCM> &lcm);
     ~StickyhandCollectionManager();
    
      
    std::map<std::string, StickyHandStruc> _hands; 
    
      // never decremented, used to set uid of sticky hands which is unique
      int free_running_sticky_hand_cnt; 
      void load_stored(OtdfInstanceStruc& instance_struc);
       // returns true if a given object name has dependent sticky hands
      bool is_parent_object(std::string& object_name);
      void add_or_update(const drc::desired_grasp_state_t* msg);		
      void add_or_update_sticky_hand(int uid, std::string& unique_hand_name, KDL::Frame &T_world_hand, drc::joint_angles_t &posture_msg);		
      bool is_urdf_found(){return _urdf_found;};
      
      // Used to generate desired ee constraints for manip plans. 
      // Generates and returns ee constraints for all associated seeds  of a given parent object
      void get_motion_constraints(string object_name, OtdfInstanceStruc& obj, bool is_retractable,
                                map<string, vector<KDL::Frame> > &ee_frames_map, 
                                map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                map<string, vector<double> > &joint_pos_map,
                                map<string, vector<int64_t> > &joint_pos_timestamps_map);     
                                
      // Used to generate desired ee constraints for pose goals. 
      // Generates and returns ee constraints for all associated seeds of a given parent object                             
      void get_pose_constraints(string object_name, OtdfInstanceStruc& obj, bool to_future_state,
                                map<string, vector<KDL::Frame> > &ee_frames_map, 
                                map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                map<string, vector<double> > &joint_pos_map,
                                map<string, vector<int64_t> > &joint_pos_timestamps_map);
                                
     // Used to generate desired ee constraints for manip maps. 
     // Generates and returns ee constraints for all associated seeds of a given parent object that are indexed with
     // given dof value.                                
      void get_aff_indexed_ee_constraints(string& object_name,OtdfInstanceStruc& obj,
                                          boost::shared_ptr<visualization_utils::BatchFKQueryHandler>  &dofRangeFkQueryHandler,
                                          map<string, vector<KDL::Frame> > &ee_frames_map,
                                          map<string, vector<drc::affordance_index_t> > &ee_frame_affindices_map); 
                                          
      
      void clear_highlights();
      void highlight_selected(boost::shared_ptr<visualization_utils::SelectionManager>  &selectionManager);                                                                          
    private:	
      boost::shared_ptr<lcm::LCM> _lcm;
      std::string _urdf_dir_name;
      std::vector<std::string> _urdf_filenames;
      std::string _urdf_xml_string;      
      bool load_hand_urdf(int grasp_type);
      std::vector< boost::shared_ptr<visualization_utils::GlKinematicBody> >  _gl_hand_list;
      std::map<int,int> _handtype_id_map;
      bool _urdf_found;
      
     std::string _object_name;
		 std::string _geometry_name;
		 int _grasp_type;
		 int _optimization_status;// RUNNING=0, SUCCESS=1, FAILURE=2;
  };


}//end_namespace


#endif //STICKY_HAND_COLLECTION_MANAGER_HPP

