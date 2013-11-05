#ifndef STICKY_HAND_COLLECTION_MANAGER_HPP
#define STICKY_HAND_COLLECTION_MANAGER_HPP


#include "sticky_hand_utils.hpp"
#include <visualization_utils/affordance_utils/AffordanceCollectionManager.hpp>
#include <visualization_utils/affordance_utils/BatchFKQueryHandler.hpp>
#include <visualization_utils/SelectionManager.hpp>
#include <otdf_parser/otdf_parser.h>
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
      
      
      // motion constraints requires equal number of samples for multiple seeds.    
      // used to get the motion history bounds of different seeds (history size can differ for left hand and right hand)
      // Will be used later in get_motion_constraints to remove unassociated constraints.
      void get_motion_history_bnds_of_seeds(string object_name, int &max_motion_history_size,int &min_motion_history_size);
      
      // Used to generate desired ee constraints for manip plans. 
      // Generates and returns ee constraints for all associated seeds  of a given parent object
      void get_motion_constraints(string object_name, OtdfInstanceStruc& obj, bool is_retractable,
                                map<string, vector<KDL::Frame> > &ee_frames_map, 
                                map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                map<string, vector<double> > &joint_pos_map,
                                map<string, vector<int64_t> > &joint_pos_timestamps_map,
                                int max_num_frames,double retracting_offset);     
                                
      // Used to generate desired ee constraints for pose goals. 
      // Generates and returns ee constraints for all associated seeds of a given parent object                             
      void get_pose_constraints(string object_name, OtdfInstanceStruc& obj, bool to_future_state,
                                map<string, vector<KDL::Frame> > &ee_frames_map, 
                                map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                map<string, vector<double> > &joint_pos_map,
                                map<string, vector<int64_t> > &joint_pos_timestamps_map);
                                
     // Ordered constraints via shift+select mechanism.
     void get_selected_pose_constraints(boost::shared_ptr<visualization_utils::AffordanceCollectionManager>  &affCollectionManager, bool to_future_state,
                            boost::shared_ptr<visualization_utils::SelectionManager>  &selectionManager,
                            map<string, vector<KDL::Frame> > &ee_frames_map, 
                            map<string, vector<int64_t> > &ee_frame_timestamps_map,
                            map<string, vector<double> > &joint_pos_map,
                            map<string, vector<int64_t> > &joint_pos_timestamps_map,
                            bool is_time_ordered);                                  
                                
     // Used to generate desired ee constraints for manip maps. 
     // Generates and returns ee constraints for all associated seeds of a given parent object that are indexed with
     // given dof value.                                
      void get_aff_indexed_ee_constraints(string& object_name,OtdfInstanceStruc& obj,
                                          boost::shared_ptr<visualization_utils::BatchFKQueryHandler>  &dofRangeFkQueryHandler,
                                          map<string, vector<KDL::Frame> > &ee_frames_map,
                                          map<string, vector<drc::affordance_index_t> > &ee_frame_affindices_map); 
                                          
      
      void clear_highlights();
      void highlight_selected(boost::shared_ptr<visualization_utils::SelectionManager>  &selectionManager); 
      bool remove(string &id); 
      bool remove_selected(boost::shared_ptr<visualization_utils::SelectionManager>  &selectionManager); 
      bool store(string &id, bool unstore,
                 boost::shared_ptr<visualization_utils::AffordanceCollectionManager>  &affCollectionManager);  
      bool store_selected(boost::shared_ptr<visualization_utils::SelectionManager>  &selectionManager,bool unstore,
                          boost::shared_ptr<visualization_utils::AffordanceCollectionManager>  &affCollectionManager);      
     void remove_seeds(std::string& obj_id,
                      boost::shared_ptr<visualization_utils::AffordanceCollectionManager>  &affCollectionManager);                                                                       
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

