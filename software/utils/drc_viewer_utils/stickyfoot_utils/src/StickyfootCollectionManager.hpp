#ifndef STICKY_FOOT_COLLECTION_MANAGER_HPP
#define STICKY_FOOT_COLLECTION_MANAGER_HPP


#include <math.h>
#include "sticky_foot_utils.hpp"
#include <visualization_utils/affordance_utils/AffordanceCollectionManager.hpp>
#include <visualization_utils/affordance_utils/BatchFKQueryHandler.hpp>
#include <visualization_utils/SelectionManager.hpp>
#include <otdf_parser/otdf_parser.h>

using namespace std;

namespace visualization_utils
{
  typedef std::map<std::string, StickyFootStruc > sticky_feet_map_type_;
  // wraps cache of sticky_feet
  class StickyfootCollectionManager 
  {
    public:
     StickyfootCollectionManager(boost::shared_ptr<lcm::LCM> &lcm);
     ~StickyfootCollectionManager();
    
    std::string _urdf_xml_string; // of the foot/feet  
    std::map<std::string, StickyFootStruc> _feet; 
    // never decremented, used to set uid of sticky foots which is unique 
    int free_running_sticky_foot_cnt; 
    
    void load_stored(OtdfInstanceStruc& instance_struc);
    // returns true if a given object name has dependent sticky feet
    bool is_parent_object(std::string& object_name);
    void add_or_update_sticky_foot(int uid,int foot_type, string& object_name, string& geometry_name,
                                   KDL::Frame &T_world_foot, vector<string> &joint_names,
                                   vector<double> &joint_positions);
                                   
    KDL::Frame _T_groundframe_bodyframe_left; 
    KDL::Frame _T_groundframe_bodyframe_right; 
    
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
                          bool is_time_ordered,bool use_nan_for_z);                                
                              
    // Used to generate desired ee constraints for manip maps. 
    // Generates and returns ee constraints for all associated seeds of a given parent object that are indexed with
    // given dof value.                                
    void get_aff_indexed_ee_constraints(string& object_name,OtdfInstanceStruc& obj,
                                        boost::shared_ptr<BatchFKQueryHandler>  &dofRangeFkQueryHandler,
                                        map<string, vector<KDL::Frame> > &ee_frames_map,
                                        map<string, vector<drc::affordance_index_t> > &ee_frame_affindices_map); 
    
    // Seeds foot given normal at point of dbl click. Drag direction gives foot direction.
    // Foot frame Z direction should point towards normal.  
    void seed_foot(OtdfInstanceStruc& obj,std::string &object_name,std::string &geometry_name,int foot_type,foot_contact_mask_type_t contact_mask, Eigen::Vector3f &ray_hit_drag,Eigen::Vector3f &ray_hit,Eigen::Vector3f &ray_hit_normal);
      
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
    bool load_foot_urdfs();
    bool _foot_urdfs_found;
    std::string _left_foot_name; // foot ee names
    std::string _right_foot_name;
    std::string _left_urdf_xml_string;
    std::string _right_urdf_xml_string;
		boost::shared_ptr<visualization_utils::GlKinematicBody> _base_gl_foot_left;
    boost::shared_ptr<visualization_utils::GlKinematicBody> _base_gl_foot_right;   
    
    KDL::Vector get_contact_mask_offset(foot_contact_mask_type_t contact_mask); 
  };


}//end_namespace


#endif //STICKY_FOOT_COLLECTION_MANAGER_HPP

