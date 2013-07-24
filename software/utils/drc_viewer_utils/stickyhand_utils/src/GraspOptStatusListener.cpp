#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "GraspOptStatusListener.hpp"
#include <algorithm>

using namespace std;
using namespace boost;

namespace renderer_affordances 
{
  //==================constructor / destructor

  GraspOptStatusListener::GraspOptStatusListener(RendererAffordances* affordance_renderer):
    _parent_renderer(affordance_renderer),
    _worker_pool_ready(false),
    _num_workers(0) 
  {
    _last_statusmsg_stamp = bot_timestamp_now();
    _lcm = affordance_renderer->lcm; 
    
    //lcm ok?
    if(!_lcm->good())
    {
      cerr << "\nLCM Not Good: Robot State Handler" << endl;
      return;
    }
    _worker_availability.clear();
    //Subscribe to GRASP_OPT_STATUS 
    _lcm->subscribe("GRASP_OPT_STATUS", &GraspOptStatusListener::handleGraspOptStatusMsg, this); 

  }

  GraspOptStatusListener::~GraspOptStatusListener() {

  }

//-------------------------------------------------------------------------------------      
//=============message callbacks

  void GraspOptStatusListener::handleGraspOptStatusMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::grasp_opt_status_t* msg)						 
  {
    _last_statusmsg_stamp =  bot_timestamp_now();//msg->utime; use system time hear to maintain heart beat from drake
    _worker_pool_ready = msg->matlab_pool_ready;
    _num_workers= msg->num_matlab_workers;
    if(_worker_availability.size()==0){
     _worker_availability.resize(_num_workers,false);// initialize all to false.
     _worker_reservation.resize(_num_workers,false);// initialize all to -1.
     _worker_associated_handuid.resize(_num_workers,-1);
     }
    else{
 	   _worker_availability.resize(_num_workers);
 	   _worker_reservation.resize(_num_workers);
 	   _worker_associated_handuid.resize(_num_workers);
 	   }
 	    	   
 	   _worker_availability[msg->worker_id-1] = msg->worker_available;
 	   if(msg->worker_available==false)
 	    _worker_reservation[msg->worker_id-1] =false; // unreserve as it is engaged

	  bot_gtk_param_widget_set_bool(_parent_renderer->pw,PARAM_OPT_POOL_READY,_worker_pool_ready); // TODO: need a time-out

  } // end handleMessage

//-------------------------------------------------------------------------------------  
// utils      

int GraspOptStatusListener::getNextAvailableOptChannelId()
{
  for (uint i=0;i<_worker_availability.size();i++)
  {
     if((_worker_availability[i])&&(!_worker_reservation[i])){ // get the next available channel that has not been reserved.
         return i+1;  
      }
  } 
  return -1; // no opt channel available 
}

bool GraspOptStatusListener::reserveOptChannel(int OptChannel, int uid)
{

    if(OptChannel<=_num_workers) {
     _worker_reservation[OptChannel-1] = true; // now its reserved    
     _worker_associated_handuid[OptChannel-1]= uid;
     return true;
     }

   return false;
}

bool GraspOptStatusListener::unreserveOptChannel(int OptChannel)
{
    if(OptChannel<=_num_workers) {
     _worker_reservation[OptChannel-1] = false; // now its unreserved
     return true;
     }

     return false;
}

void GraspOptStatusListener::getAllReservedOptChannelIdsandHandUids(std::vector<int> &OptChannelIdList, std::vector<int> &ChannelHandUidList)
{ 
  OptChannelIdList.clear();
  ChannelHandUidList.clear();
  for (size_t i=0;i<_worker_availability.size();i++)
  {
   if(_worker_availability[i]){ // get the next available channel that has not been reserved.
       OptChannelIdList.push_back(i+1); 
       ChannelHandUidList.push_back(_worker_associated_handuid[i]); 
    }
  } 
}

int GraspOptStatusListener::getReservedOptChannelIdforHandUid(int hand_uid)
{
  for (size_t i=0;i<_worker_associated_handuid.size();i++)
  {
    if(_worker_associated_handuid[i]==hand_uid)  {
      return i+1;
    }
  }
  return -1; // no opt channel reserved 
}

} //end namespace


