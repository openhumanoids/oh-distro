#include <stdio.h>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include <ConciseArgs>

using namespace std;

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~App(){
    }
    
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;

    void sandiaRawHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::raw_sandia_hand_t* msg);    
};   

App::App(boost::shared_ptr<lcm::LCM> &lcm_):
   lcm_(lcm_){

  lcm_->subscribe("SANDIA_LEFT_RAW",&App::sandiaRawHandler,this);  
}




void App::sandiaRawHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::raw_sandia_hand_t* msg){
 /* std::cout << msg->utime 
            << ", " << msg->mobo.finger_currents[0]
            << ", " << msg->mobo.finger_currents[1]
            << ", " << msg->mobo.finger_currents[2]            
            << ", " << msg->mobo.finger_currents[3] << "\n";
 */
  
}
  
 
int main(int argc, char *argv[]){
/*  string role = "robot";
  bool labels = false;
  bool triads = false;
  bool standalone_head = false;
  bool ground_height = false;
  bool bdi_motion_estimate = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(role, "r", "role","Role - robot or base");
  opt.add(triads, "t", "triads","Frame Triads - show no not");
  opt.add(labels, "l", "labels","Frame Labels - show no not");
  opt.add(ground_height, "g", "ground", "Publish the grounded foot pose");
  opt.add(standalone_head, "s", "standalone_head","Standalone Sensor Head");
  opt.add(bdi_motion_estimate, "b", "bdi","Use POSE_BDI to make frames [Temporary!]");
  opt.parse();
  if (labels){ // require triads if labels is to be published
    triads=true;
  }
  
  std::cout << "triads: " << triads << "\n";
  std::cout << "labels: " << labels << "\n";
  std::cout << "role: " << role << "\n";
*/

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  App app(lcm);
  while(0 == lcm->handle());
  return 0;
}