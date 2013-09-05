#ifndef AFF_TRIGGER_SIGNAL_UTILS_HPP
#define AFF_TRIGGER_SIGNAL_UTILS_HPP

#include <iostream>
#include <boost/signals2.hpp> 
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <kdl/frames.hpp>

using namespace std;
namespace visualization_utils {


    typedef boost::signals2::signal<void(string,KDL::Frame)> AffTriggerPlanStoreSignal; 
    typedef boost::signals2::signal<void(string,KDL::Frame,string)> AffTriggerPlanLoadSignal; 
    struct AffTriggerSignals
    {
      AffTriggerPlanStoreSignal plan_store; // Signals  plan renderer to store current plan in the specified affordance's xml file. Stores the plan in affordance frame.
      AffTriggerPlanLoadSignal plan_load;
    };
    typedef boost::shared_ptr<AffTriggerSignals> AffTriggerSignalsRef;
    
    enum aff_trigger_type
    {
      PLAN_STORE=0,PLAN_LOAD, UNKNOWN
    }; 
       
    class AffTriggerSignalsHandler 
    {
      public:
        AffTriggerSignalsHandler(AffTriggerSignalsRef signalCollectionRef, boost::function<void(aff_trigger_type,string,KDL::Frame,string)> func):_user_callback(func)
        {
          _connection1 =  signalCollectionRef->plan_store.connect(boost::bind(&AffTriggerSignalsHandler::callback,this,PLAN_STORE,_1,_2," "));
          _connection2 =  signalCollectionRef->plan_load.connect(boost::bind(&AffTriggerSignalsHandler::callback,this,PLAN_LOAD,_1,_2,_3));
        };
        
        ~AffTriggerSignalsHandler(){
          _connection1.disconnect();
          _connection2.disconnect();
        }

        void callback(aff_trigger_type type,std::string otdf_id, KDL::Frame T_world_aff,std::string plan_id)
        {
          /*if(type==PLAN_STORE)
            cout<< otdf_id << " got triggered to store currently active plan"<< endl;
          else
            cerr<<  " unknown trigger "<< endl;*/
          
          if(!_user_callback.empty())
             _user_callback(type,otdf_id,T_world_aff,plan_id); 
          else
             cerr << "ERROR in visualization_utils::AffTriggerSignalsHandler:  user callback function reference is empty!" << endl;
        }
        
        private:
        boost::signals2::connection _connection1; 
        boost::signals2::connection _connection2; 
        boost::function<void (aff_trigger_type,string,KDL::Frame,string)> _user_callback; 
    };

} // end namespace 


#endif //AFF_TRIGGER_SIGNAL_UTILS_HPP
/* usage

AffTriggerSignalsRef _affTriggerSignalsRef = AffTriggerSignalsRef(new AffTriggerSignals()); 

(*_affTriggerSignalsRef).plan_store("cylinder",KDL::Frame::Identity()); // emit global keyboard signal


*/
