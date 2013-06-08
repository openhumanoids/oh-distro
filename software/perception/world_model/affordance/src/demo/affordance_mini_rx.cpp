#include <stdio.h>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <sys/time.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/drc/affordance_mini_collection_t.hpp"
#include "lcmtypes/bot_core.hpp"


#include <ConciseArgs>
using namespace std;

class App
{
public:
  App(boost::shared_ptr<lcm::LCM> &_lcm);
  ~App() {}
  boost::shared_ptr<lcm::LCM> _lcm;
  void handleAMBO(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const  drc::affordance_mini_collection_t * msg);

  drc::affordance_plus_t getCarAffordancePlus(std::string filename, float xyz[], float rpy[], int uid);
  drc::affordance_plus_t getCylinderAffordancePlus(std::string filename, float xyz[], float rpy[], int uid, float c_length, float c_radius);
  
  
private:
  
};

App::App(boost::shared_ptr<lcm::LCM> &_lcm): _lcm(_lcm){
  
  _lcm->subscribe("AFFORDANCE_MINI_BOT_OVERWRITE", &App::handleAMBO, this); 
 
  
  
}


drc::affordance_plus_t App::getCarAffordancePlus(std::string filename, float xyz[], float rpy[], int uid){ 
  drc::affordance_plus_t p;
  
  drc::affordance_t a;
  a.utime =0;
  a.map_id =0;
  a.uid =uid;
  a.otdf_type ="car";
  a.aff_store_control = drc::affordance_t::NEW;

  a.nparams =0;
  a.nstates =0;

  a.origin_xyz[0]=xyz[0]; a.origin_xyz[1]=xyz[1]; a.origin_xyz[2]=xyz[2]; 
  a.origin_rpy[0]=rpy[0]; a.origin_rpy[1]=rpy[1]; a.origin_rpy[2]=rpy[2]; 
  
  a.bounding_xyz[0]=0.0; a.bounding_xyz[1]=0; a.bounding_xyz[2]=1.0; 
  a.bounding_rpy[0]=0.0; a.bounding_rpy[1]=0.0; a.bounding_rpy[2]=0.0; 
  a.bounding_lwh[0]=3.0;       a.bounding_lwh[1]=1.7;      a.bounding_lwh[2]=2.2;
  
  a.modelfile = filename;
  p.aff = a;
  
  p.npoints=0;//points.size(); 
  p.ntriangles =0;//p.triangles.size();
  
  return p;
}




drc::affordance_plus_t App::getCylinderAffordancePlus(std::string filename, float xyz[], float rpy[], int uid, float c_length, float c_radius){ 
  drc::affordance_plus_t p;
  
  drc::affordance_t a;
  a.utime =0;
  a.map_id =0;
  a.uid =uid;
  a.otdf_type ="cylinder";
  a.aff_store_control = drc::affordance_t::NEW;

  a.params.push_back(c_length);
  a.params.push_back(c_radius);
  a.params.push_back(1.0);
  a.param_names.push_back("length");
  a.param_names.push_back("radius");
  a.param_names.push_back("mass");
  a.nparams = a.params.size();    
  
  a.nstates =0;

  a.origin_xyz[0]=xyz[0]; a.origin_xyz[1]=xyz[1]; a.origin_xyz[2]=xyz[2]; 
  a.origin_rpy[0]=rpy[0]; a.origin_rpy[1]=rpy[1]; a.origin_rpy[2]=rpy[2]; 
  
  a.bounding_xyz[0]=0.0; a.bounding_xyz[1]=0; a.bounding_xyz[2]=0.0; 
  a.bounding_rpy[0]=0.0; a.bounding_rpy[1]=0.0; a.bounding_rpy[2]=0.0; 
  a.bounding_lwh[0]=c_radius*2;       a.bounding_lwh[1]=c_radius*2;      a.bounding_lwh[2]=c_length;
  
  a.modelfile = ""; // none
  p.aff = a;
  
  p.npoints=0;//points.size(); 
  p.ntriangles =0;//p.triangles.size();
  
  return p;
}




void App::handleAMBO(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const  drc::affordance_mini_collection_t * msg)  {
  std::cout << "got Add Mini Bot 0verwrite\n";
  
  drc::affordance_plus_collection_t pcoll;
  
  for (size_t i=0; i < msg->affs.size(); i++){
    drc::affordance_mini_t mini = msg->affs[i];
    if (mini.type == drc::affordance_mini_t::CAR){
      std::cout << i << ": " << mini.uid << " car\n";
      drc::affordance_plus_t plus = getCarAffordancePlus("car.pcd", mini.origin_xyz, mini.origin_rpy, mini.uid );
      pcoll.affs_plus.push_back( plus );
    }else if (mini.type == drc::affordance_mini_t::CYLINDER){
      std::cout << i << ": " << mini.uid << " cylinder\n";
      float c_length=0, c_radius=0;
      for (size_t i=0; i < mini.param_names.size(); i++){
        if (mini.param_names[i] == drc::affordance_mini_t::LENGTH){
          c_length = mini.params[i];
        }else if (mini.param_names[i] == drc::affordance_mini_t::RADIUS){
          c_radius = mini.params[i];          
        }else{
          std::cout << "parameter not understood ########################"; 
        }
      }
      drc::affordance_plus_t plus = getCylinderAffordancePlus("", mini.origin_xyz, mini.origin_rpy, mini.uid , c_length, c_radius);
      pcoll.affs_plus.push_back( plus );
    }else{
      std::cout << i << ": " << mini.uid << " not understood #####################\n";
    }
  }
  
  pcoll.naffs = pcoll.affs_plus.size();
  
  _lcm->publish("AFFORDANCE_PLUS_BOT_OVERWRITE", &pcoll );
  
}


int main (int argc, char ** argv){
  /*
  ConciseArgs parser(argc, argv, "aff-mini-rx");
  int mode=0;
  bool verbose=false;
  parser.add(mode, "m", "mode", "Mode [0,1]");
  parser.add(verbose, "v", "verbose", "Verbose");
  parser.parse();
  cout << "mode is: " << mode << "\n"; 
  cout << "verbose is: " << verbose << "\n"; 
  */
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  App app(lcm);
  cout << "App ready"<< endl;
  while(0 == lcm->handle());
  return 0;
}