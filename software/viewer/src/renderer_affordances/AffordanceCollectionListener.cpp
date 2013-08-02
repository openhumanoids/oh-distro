
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
    if(dt< 2) // 1Hz Sync
      return;
  
    //cout << "Ok!: " << oss.str() << endl;
    for (size_t i=0; i< (size_t)msg->naffs; i++)
    {
      const drc::affordance_t aff = msg->affs[i];
      
      std::stringstream oss;
      oss << aff.otdf_type << "_"<< aff.uid; 
        
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
    _last_affcoll_msg_system_timestamp = now;

  } // end handleMessage

  void AffordanceCollectionListener::handleAffordancePlusCollectionMsg(const lcm::ReceiveBuffer* rbuf,
                                                                   const string& chan, 
                                                                   const drc::affordance_plus_collection_t* msg)						 
  {

    // call handleAffordancePlusMsg for each affordance in collection
    set<pair<string,int> > currentAffs;
    for (size_t i=0; i< (size_t)msg->naffs; i++) {
      const drc::affordance_plus_t& aff = msg->affs_plus[i];
      handleAffordancePlusMsg(rbuf, chan, &aff);
      currentAffs.insert(make_pair(aff.aff.otdf_type, aff.aff.uid));
    }

    /* TODO: put back in after solving sticky hands crash issue. 
    // delete any affordance not in message
    if(!_parent_affordance_renderer->debugMode){
      std::map<std::string, OtdfInstanceStruc>::iterator it;
      std::map<std::string, OtdfInstanceStruc>& objs = _parent_affordance_renderer->affCollection->_objects;
      for(it = objs.begin(); it!=objs.end(); ){
        // if not in collection, erase
        if(currentAffs.find(make_pair(it->second.otdf_type, it->second.uid)) == currentAffs.end()) {
          // clear selection if selected
          const char *instance_name;
          instance_name = bot_gtk_param_widget_get_enum_str( _parent_affordance_renderer->pw, PARAM_OTDF_INSTANCE_SELECT );
          if(instance_name && _parent_affordance_renderer->object_selection==string(instance_name)){
            _parent_affordance_renderer->link_selection = " ";
            _parent_affordance_renderer->object_selection = " ";
          }  
          // clear sticky hands if selected  
          typedef map<string, StickyHandStruc > sticky_hands_map_type_;
          sticky_hands_map_type_::iterator hand_it = _parent_affordance_renderer->sticky_hands.begin();
          while (hand_it!=_parent_affordance_renderer->sticky_hands.end()) {
            string hand_name = string(hand_it->second.object_name);
            if (instance_name && hand_name == string(instance_name)) {
              if(_parent_affordance_renderer->stickyhand_selection==hand_it->first)
                 _parent_affordance_renderer->stickyhand_selection = " ";
              _parent_affordance_renderer->sticky_hands.erase(hand_it++);
            } else hand_it++;
          } 
          // erase object
          objs.erase(it++);
          // trigger redraw
          bot_viewer_request_redraw(_parent_affordance_renderer->viewer);
        } else ++it;   // otherwise go to next object
      }
    }*/

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

// =======================================================================


} //namespace affordance_renderer

