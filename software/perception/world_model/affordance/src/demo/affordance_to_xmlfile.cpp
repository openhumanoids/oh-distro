#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <otdf_lcm_utils/otdf_lcm_utils.h>
#include <ConciseArgs>


using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void affHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::affordance_plus_collection_t* msg);   

    std::string image_channel_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_):
    lcm_(lcm_), image_channel_(image_channel_){

  lcm_->subscribe( "AFFORDANCE_PLUS_COLLECTION" ,&Pass::affHandler,this);
  
}

void Pass::affHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::affordance_plus_collection_t* msg){

  std::string urdf_xml_string;
  otdf::AffordanceLcmMsgToUrdfString(msg->affs_plus[0].aff, urdf_xml_string);

  std::cout << "got affs\n";
  std::cout << urdf_xml_string << "\n";
}


int main(int argc, char ** argv) {
  string channel = "dsfsd";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(channel, "c", "channel","channel");
  opt.parse();
  std::cout << "channel: " << channel << "\n";  

  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,channel);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}

