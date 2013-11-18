
#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "AffordanceCollectionListener.hpp"

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <drc_utils/PointConvert.h>

#include <affordance/AffordanceUtils.hpp>
#include <renderer_affordances/CandidateGraspSeedListener.hpp>
#include <visualization_utils/stickyfoot_utils/sticky_foot_utils.hpp>

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;
using namespace drc::PointConvert;

namespace renderer_affordances
{

  //==================constructor / destructor

//   AffordanceCollectionListener::AffordanceCollectionListener(boost::shared_ptr<lcm::LCM> &lcm, RendererAffordances* affordance_renderer):
//     _lcm(lcm),
//     _parent_affordance_renderer(affordance_renderer)
  AffordanceCollectionListener::AffordanceCollectionListener( RendererAffordances* affordance_renderer):
    _parent_affordance_renderer(affordance_renderer),_last_affcoll_msg_system_timestamp(0)
  {
    _lcm = affordance_renderer->lcm;
    _last_affcoll_msg_system_timestamp = bot_timestamp_now();
 
    //lcm ok?
    if(!_lcm->good())
      {
	      cerr << "\nLCM Not Good: Robot State Handler" << endl;
	      return;
      }


    _lcm->subscribe("AFFORDANCE_COLLECTION", &AffordanceCollectionListener::handleAffordanceCollectionMsg, this); 
    _lcm->subscribe("AFFORDANCE", &AffordanceCollectionListener::handleAffordanceMsg, this); 
    _lcm->subscribe("AFFORDANCE_PLUS_COLLECTION", &AffordanceCollectionListener::handleAffordancePlusCollectionMsg, this); 
    _lcm->subscribe("AFFORDANCE_PLUS", &AffordanceCollectionListener::handleAffordancePlusMsg, this); 
    _lcm->subscribe("AFF_TRIGGERED_CANDIDATE_STICKY_FEET",&AffordanceCollectionListener::handleAffordanceTriggeredCandidateStickyFeetMsg, this); 

  }
  


  AffordanceCollectionListener::~AffordanceCollectionListener() {}

 // =======================================================================  
 //  message callbacks
 // =======================================================================   

// checks to see the affordances are in the parent renderer object list. If it is pre-existing 
// it is updated and if its new a new instance is created.
void AffordanceCollectionListener::handleAffordanceCollectionMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::affordance_collection_t* msg)						 
  {
  
   int64_t now =bot_timestamp_now();
   double dt = (now- _last_affcoll_msg_system_timestamp )/1000000.0;// timestamps are in usec
    if(dt< 2) // 2Hz Sync
      return;
  
    //cout << "Ok!: " << oss.str() << endl;
   std::vector<std::string> active_object_names;
    for (size_t i=0; i< (size_t)msg->naffs; i++)
    {
      const drc::affordance_t aff = msg->affs[i];
      
      std::stringstream oss;
      oss << aff.otdf_type << "_"<< aff.uid; 
      active_object_names.push_back(oss.str());  
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = _parent_affordance_renderer->affCollection->_objects.find(oss.str());
       
       if (it!=_parent_affordance_renderer->affCollection->_objects.end()) {
          //exists so update
          //cout <<"updated_otdf_object_instance: "<< aff.otdf_type <<  " " <<  aff.uid << endl;
          _parent_affordance_renderer->affCollection->update(aff);
       }      
       else {
          // add new otdf instance
          std::string filename = aff.otdf_type;//get_filename(aff.otdf_id);
          cout <<"add new otdf instance: "<< aff.otdf_type << "_"<< aff.uid << ", of template :" << filename << endl;
          //std::transform(filename.begin(), filename.end(), filename.begin(), ::tolower);
           OtdfInstanceStruc instance_struc;
          bool success=_parent_affordance_renderer->affCollection->add(filename, aff,instance_struc);
          if((instance_struc._otdf_instance)&&(success)){
           _parent_affordance_renderer->stickyHandCollection->load_stored(instance_struc);
           _parent_affordance_renderer->stickyFootCollection->load_stored(instance_struc);
          }
       }
    }

    //cycle through object in affCollection,
    // and remove all affordances that are not in affCollection but exist in the renderer.
     removeRedundantObjectsFromCache(active_object_names);
    
    _last_affcoll_msg_system_timestamp = now;

  } // end handleMessage

  void AffordanceCollectionListener::handleAffordancePlusCollectionMsg(const lcm::ReceiveBuffer* rbuf,
                                                                   const string& chan, 
                                                                   const drc::affordance_plus_collection_t* msg)						 
  {

    // call handleAffordancePlusMsg for each affordance in collection
    set<pair<string,int> > currentAffs;
    std::vector<std::string> active_object_names;
    for (size_t i=0; i< (size_t)msg->naffs; i++) {
      const drc::affordance_plus_t& aff = msg->affs_plus[i];
      handleAffordancePlusMsg(rbuf, chan, &aff);
      currentAffs.insert(make_pair(aff.aff.otdf_type, aff.aff.uid));
       std::stringstream oss;
      oss << aff.aff.otdf_type << "_"<< aff.aff.uid; 
      active_object_names.push_back(oss.str());  
    }

    //cycle through object in affCollection,
    // and remove all affordances that are not in affCollection but exist in the renderer.
     removeRedundantObjectsFromCache(active_object_names);
  }

   // =======================================================================  
   
  void AffordanceCollectionListener::handleAffordanceMsg(const lcm::ReceiveBuffer* rbuf, const string& channel, 
					       const  drc::affordance_t* msg) 
  {
    //cout << "Received affordance: " << msg->otdf_type << " with uid: " << msg->uid << endl;
       std::stringstream oss;
       oss << msg->otdf_type << "_"<< msg->uid; 
    
       typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
       object_instance_map_type_::iterator it = _parent_affordance_renderer->affCollection->_objects.find(oss.str());
       if (it!=_parent_affordance_renderer->affCollection->_objects.end()) {
          //exists so update
         //cout <<"update_otdf_object_instance" << endl;
         _parent_affordance_renderer->affCollection->update(*msg);
       }      
       else{
          // add new otdf instance
          cout <<"add new otdf instance" << endl;
          std::string filename = msg->otdf_type;//get_filename(msg->otdf_id);
          
          OtdfInstanceStruc instance_struc;
          bool success=_parent_affordance_renderer->affCollection->add(filename, (*msg),instance_struc);
          if((instance_struc._otdf_instance)&&(success)){
           _parent_affordance_renderer->stickyHandCollection->load_stored(instance_struc);
           _parent_affordance_renderer->stickyFootCollection->load_stored(instance_struc);
          }
       }
      //Request redraw
      bot_viewer_request_redraw(_parent_affordance_renderer->viewer);    
  } 

  void AffordanceCollectionListener::handleAffordancePlusMsg(const lcm::ReceiveBuffer* rbuf, const string& channel, 
                                                             const  drc::affordance_plus_t* msg)
  {
    // handle affordance_t within affordance_plus_t message
    handleAffordanceMsg(rbuf,channel,&msg->aff);

    //////////////////////////
    // handle plus features

    // retrieve otdf object associated with current message
    std::stringstream oss;
    oss << msg->aff.otdf_type << "_"<< msg->aff.uid;
    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = _parent_affordance_renderer->affCollection->_objects.find(oss.str());
    if (it!=_parent_affordance_renderer->affCollection->_objects.end()) {

      // if no modelfile, copy points and triangles
      if(msg->aff.modelfile.empty()){
        // copy triangles and points
        convertVec3(msg->points, it->second._gl_object->points);
        convertVec3(msg->triangles, it->second._gl_object->triangles);
      }

      // copy bounding box
      convert3(msg->aff.bounding_xyz, it->second.boundingBoxXYZ);
      convert3(msg->aff.bounding_rpy, it->second.boundingBoxRPY);
      convert3(msg->aff.bounding_lwh, it->second.boundingBoxLWH);

      // if modelfile has changed, read in file
      if(msg->aff.modelfile != it->second.modelfile){
        // read in model
        vector<vector<float> > points;
        vector<vector<int> > triangles;
        string file = getenv("HOME") + string("/drc/software/models/otdf/") + msg->aff.modelfile;
        AffordanceUtils affutils;
        affutils.getModelAsLists(file, points, triangles);
        
        convertVec3(points, it->second._gl_object->points);
        convertVec3(triangles, it->second._gl_object->triangles);

        it->second.modelfile = msg->aff.modelfile;
      }

      // find links of type DynamicMesh and copy points and triangles into it.
      /*
      std::vector<boost::shared_ptr<otdf::Link> > links;
      it->second._otdf_instance->getLinks(links);
      for(int i=0; i<links.size(); i++){
        if(links[i]->visual && links[i]->visual->geometry){
          //cout << links[i]->visual->geometry->type << endl;
          boost::shared_ptr<otdf::DynamicMesh> dmesh = 
            dynamic_pointer_cast<otdf::DynamicMesh>(links[i]->visual->geometry);
          
          if(dmesh){
            dmesh->points = msg->points;
            dmesh->triangles = msg->triangles;
          }
        }
      }
      */
      
    } else{
      // object should always exist, so we should never get here
      cout << "*** ERROR handleAffordancePlusMsg: shouldn't get here\n";
    }

  }
  
  void AffordanceCollectionListener::handleAffordanceTriggeredCandidateStickyFeetMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel,  
                                const  drc::traj_opt_constraint_t* msg)
 {
    for (size_t j=0;j<msg->num_links;j++)
    {
      int foot_type = 0;
      if(msg->link_name[j]=="r_foot")
      {
       foot_type = 1;
      }
      std::string object_name =msg->robot_name;
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it= _parent_affordance_renderer->affCollection->_objects.find(object_name);

      std::vector<LinkFrameStruct> link_geometry_tfs = it->second._gl_object->get_link_geometry_tfs();
      
      std::string geometry_name;
      KDL::Frame T_world_objectgeometry;
      if(!it->second._gl_object->get_root_link_geometry_name(geometry_name))// get the root geometry if it exists
      {
          geometry_name =link_geometry_tfs[0].name; // else take the first one in the list. note:: link_geometry_tfs is alphabetized.
          T_world_objectgeometry=link_geometry_tfs[0].frame;
      }
      else
      {
          T_world_objectgeometry=it->second._gl_object->_T_world_body; // root link
      }
      KDL::Frame T_world_aff=it->second._gl_object->_T_world_body;
      KDL::Frame T_objectgeometry_aff = T_world_objectgeometry.Inverse()*T_world_aff;
      
      KDL::Frame T_aff_foot,T_objectgeometry_foot;
      T_aff_foot.p[0]=msg->link_origin_position[j].translation.x;
      T_aff_foot.p[1]=msg->link_origin_position[j].translation.y;
      T_aff_foot.p[2]=msg->link_origin_position[j].translation.z;
      double x,y,z,w;
      x = msg->link_origin_position[j].rotation.x;
      y = msg->link_origin_position[j].rotation.y;
      z = msg->link_origin_position[j].rotation.z;
      w = msg->link_origin_position[j].rotation.w;
      T_aff_foot.M = KDL::Rotation::Quaternion(x,y,z,w);
      T_objectgeometry_foot=T_objectgeometry_aff*T_aff_foot;
      
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
      _parent_affordance_renderer->stickyFootCollection->free_running_sticky_foot_cnt++;
      int uid = _parent_affordance_renderer->stickyFootCollection->free_running_sticky_foot_cnt;
      _parent_affordance_renderer->stickyFootCollection->add_or_update_sticky_foot(uid,foot_type,object_name,geometry_name,T_objectgeometry_foot,joint_names,joint_positions);   
      
      bool store=false;
      if(store) // seed and autostore? or store manually?
      {
        string  unique_foot_name;
        std::stringstream oss;
        if(foot_type==0)
            oss << object_name <<"_"<< geometry_name << "_lfootstep_" << uid;
        else
            oss << object_name <<"_"<< geometry_name << "_rfootstep_" << uid;
        unique_foot_name = oss.str(); 
        _parent_affordance_renderer->stickyFootCollection->store(unique_foot_name,false,_parent_affordance_renderer->affCollection); 
      }

    }
 
 }
 // =======================================================================
 void AffordanceCollectionListener::removeRedundantObjectsFromCache(std::vector<std::string> &active_object_names)
 {  

    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = _parent_affordance_renderer->affCollection->_objects.begin();
    while (it!=_parent_affordance_renderer->affCollection->_objects.end())
    {
     std::vector<std::string>::const_iterator found;
     std::string instance_name = it->first;
     found = std::find (active_object_names.begin(), active_object_names.end(), instance_name);
     if (found == active_object_names.end())  // remove if not active
     {
       _parent_affordance_renderer->affCollection->_objects.erase(it++);     
       //remove dependants
       typedef map<string, StickyHandStruc > sticky_hands_map_type_;
        sticky_hands_map_type_::iterator hand_it = _parent_affordance_renderer->stickyHandCollection->_hands.begin();
        while (hand_it!=_parent_affordance_renderer->stickyHandCollection->_hands.end()) 
        {
          string hand_name = string(hand_it->second.object_name);
          if (hand_name == string(instance_name))
          {
            if(_parent_affordance_renderer->stickyhand_selection==hand_it->first){
              _parent_affordance_renderer->seedSelectionManager->remove(_parent_affordance_renderer->stickyhand_selection);
              _parent_affordance_renderer->stickyhand_selection = " ";
            }
            _parent_affordance_renderer->stickyHandCollection->_hands.erase(hand_it++);
          }
          else
            hand_it++;
        } // end while
        typedef map<string, StickyFootStruc > sticky_feet_map_type_;
        sticky_feet_map_type_::iterator foot_it = _parent_affordance_renderer->stickyFootCollection->_feet.begin();
        while (foot_it!=_parent_affordance_renderer->stickyFootCollection->_feet.end()) 
        {
          string foot_name = string(foot_it->second.object_name);
          if (foot_name == string(instance_name))
          {
            if(_parent_affordance_renderer->stickyfoot_selection==foot_it->first)
            {
              _parent_affordance_renderer->seedSelectionManager->remove(_parent_affordance_renderer->stickyfoot_selection);                            
              _parent_affordance_renderer->stickyfoot_selection = " ";
             }
            _parent_affordance_renderer->stickyFootCollection->_feet.erase(foot_it++);
          }
          else
            foot_it++;
         } // end while 
     } // end if (found == active_object_names.end())
     else
      it++;
    } // end while   
 }
// =======================================================================


} //namespace affordance_renderer

