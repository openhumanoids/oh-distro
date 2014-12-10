#ifndef AFFORDANCE_COLLECTION_MANAGER_HPP
#define AFFORDANCE_COLLECTION_MANAGER_HPP

#include "affordance_utils.hpp"

using namespace std;

namespace visualization_utils
{
  typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
  // wraps aff store messaging and the cache of affordances
  class AffordanceCollectionManager 
  {
    public:
     AffordanceCollectionManager(boost::shared_ptr<lcm::LCM> &lcm);
     ~AffordanceCollectionManager();
    
    
    // utility methods
     void delete_otdf_from_affstore(string channel, string otdf_type, int uid);
     void update_mate_joints_in_affstore(string channel, string otdf_type, int uid, const boost::shared_ptr<otdf::ModelInterface> instance_in, KDL::Frame &T_mate_start_mate_end);
     void update_object_pose_in_affstore(string channel, string otdf_type, int uid, const boost::shared_ptr<otdf::ModelInterface> instance_in, KDL::Frame &T_world_object);
     void publish_otdf_instance_to_affstore(string channel, string otdf_type, int uid, const boost::shared_ptr<otdf::ModelInterface> instance_in); 
     void publish_new_otdf_instance_to_affstore( string channel, string otdf_type, int uid, const boost::shared_ptr<otdf::ModelInterface> instance_in); 

     
     std::vector<std::string> _otdf_filenames;
     std::map<std::string, int > _instance_cnt;
     std::map<std::string, OtdfInstanceStruc> _objects; // pass a reference from construction?
     
     //  methods
    bool add(std::string &filename, const drc::affordance_t &aff, OtdfInstanceStruc &instance_struc);
    void update(const drc::affordance_t &aff);
    void create(std::string &filename, bool local_testing);
    void clear_highlights();

    private:
      boost::shared_ptr<lcm::LCM> _lcm;
      
      // Misc. Functions
      //-----------------------------      
      // xyz_ypr is already in message, no need to duplicate it   
      bool isRedundantParam(const std::string& param)
      {
        if(param=="x" || param=="y" || param=="z" || 
           param=="yaw" || param=="pitch" || param=="roll") return true;
        else return false;
      };
      
      // if common shape (e.g. cylinder, sphere), fill in bounding box
      void commonShapeBoundingBox(const string& otdf_type, boost::shared_ptr<otdf::ModelInterface> instance_in,
                                  double* bounding_xyz, double* bounding_rpy, double* bounding_lwh)
      {
         if(otdf_type == "cylinder")
         {  
            bounding_lwh[0] = instance_in->params_map_.find("radius")->second*2;
            bounding_lwh[1] = instance_in->params_map_.find("radius")->second*2;
            bounding_lwh[2] = instance_in->params_map_.find("length")->second;
         } 
         else if(otdf_type == "sphere")
         {  
            bounding_lwh[0] = instance_in->params_map_.find("radius")->second*2;
            bounding_lwh[1] = instance_in->params_map_.find("radius")->second*2;
            bounding_lwh[2] = instance_in->params_map_.find("radius")->second*2;
         } 
         else if(otdf_type == "TODO")
         {  //TODO others...
         
         }
      };
      
      
  
  };


}//end_namespace


#endif //AFFORDANCE_COLLECTION_MANAGER_HPP

