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


  CandidateGraspSeedListener::CandidateGraspSeedListener(RendererAffordances* affordance_renderer):
    _parent_renderer(affordance_renderer)
  {
 
    _lcm = affordance_renderer->lcm; 
    //_collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());
    
    //lcm ok?
    if(!_lcm->good())
    {
      cerr << "\nLCM Not Good: Candidate Grasp Seed Listener" << endl;
      return;
    }

    if(_parent_renderer->stickyHandCollection->is_urdf_found()){
    //Subscribe to CANDIDATE_GRASP 
    _lcm->subscribe("CANDIDATE_GRASP", &CandidateGraspSeedListener::handleDesiredGraspStateMsg, this); 
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
    object_instance_map_type_::iterator it = _parent_renderer->affCollection->_objects.find(msg->object_name);

    if(it == _parent_renderer->affCollection->_objects.end()) {
       cerr << " object_name " <<msg->object_name << " specified in CANDIDATE_GRASP channel is not available in cache." << endl; 
       return;
    }    
   
    _parent_renderer->stickyHandCollection->add_or_update(msg);
      
     //Request redraw
     bot_viewer_request_redraw(_parent_renderer->viewer);

  } // end handleMessage
  


//-------------------------------------------------------------------------------------        

} //end namespace


