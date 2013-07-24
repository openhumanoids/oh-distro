#ifndef RENDERER_AFFORDANCES_GRASPOPTSTATUSLISTENER_HPP
#define RENDERER_AFFORDANCES_GRASPOPTSTATUSLISTENER_HPP


#include "renderer_affordances.hpp" // has definition of RendererAffordances struc

namespace renderer_affordances
{

// Sample usage
// GraspOptStatusListener obj = new GraspOptStatusListener();
// if(obj.isOptPoolReady()){
//    int id =  obj.getNextAvailableOptChannelId();
//    if((id!=-1)&&(obj.reserveOptChannel(id)))
//       publish on INIT_GRASP_SEED;
// }
	  
  class GraspOptStatusListener
  {

  private:
    boost::shared_ptr<lcm::LCM> _lcm; 
    RendererAffordances* _parent_renderer; // maintains the list of objects.  

    //----------------constructor/destructor
  public:
    GraspOptStatusListener(RendererAffordances* affordance_renderer);
    ~GraspOptStatusListener();
  
    

    //-------------message callback
  private:
    void handleGraspOptStatusMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::grasp_opt_status_t* msg);
			      
  private: 
 	   //Reservation Process:
 	   //User reserves the next available channel reservation an publishes a msg to initialize opt.
 	   //When opt starts, a status msg is received which clears reservation.
 	   std::vector<bool> _worker_availability; // set by status messages from drake.
 	   std::vector<bool> _worker_reservation;  // set by user. There can be lag between reservation and status actually changing.
 	   std::vector<int>  _worker_associated_handuid; // Associated hand_uid on reserved channel.
 	   int _num_workers;
 	   bool _worker_pool_ready;
 	   
 	public:
	  int getNextAvailableOptChannelId(void);
	  void getAllReservedOptChannelIdsandHandUids(std::vector<int> &OptChannelList, std::vector<int> &ChannelHandUidList);
	  int getReservedOptChannelIdforHandUid(int hand_uid); // used to reseed a hand_uid.
	  bool reserveOptChannel(int OptChannel, int hand_uid);
	  bool unreserveOptChannel(int OptChannel);
	  bool isOptPoolReady(void){
	   return _worker_pool_ready;
	  };
 	
 	public:     		      
		 int64_t  _last_statusmsg_stamp;	      

  }; //class GraspOptStatusListener


} //end namespace


#endif //RENDERER_AFFORDANCES_GRASPOPTSTATUSLISTENER_HPP
