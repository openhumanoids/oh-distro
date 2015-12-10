// simple test utility to save an octree published by the maps server
// to a .bt file

#include <ConciseArgs>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/map_octree_t.hpp>
#include <octomap/octomap.h>
using namespace octomap;

struct AppConfig
{
  bool repeat;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, AppConfig app_cfg);
    
    ~App(){
    }        
    
    AppConfig app_cfg_;
    
    boost::shared_ptr<lcm::LCM> lcm_;
    
    void octreeHandler(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  drc::map_octree_t* msg);
    
    OcTree* tree_;  

  private:
    
};

App::App(boost::shared_ptr< lcm::LCM >& lcm_, AppConfig app_cfg_) : lcm_(lcm_), 
         app_cfg_(app_cfg_){
  
  std::cout << "Subscribing to octomap on MAP_OCTREE\n";
  lcm_->subscribe("MAP_OCTREE",&App::octreeHandler,this);


  tree_  = new OcTree(1); // resolution reset else where
}

void App::octreeHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::map_octree_t* msg){
  std::cout << "MAP_OCTREE received\n";

  // TODO: Currently not handling transform, assuming identity transform

  std::stringstream datastream;
  datastream.write((const char*) msg->data.data(), msg->num_bytes);
  tree_ = new octomap::OcTree(1); //resolution will be set by data from message
  tree_->readBinary(datastream);

  std::stringstream s;
  s <<  "/tmp/map_octomap.bt" ;
  printf("Saving MAP_OCTREE to: %s\n", s.str().c_str());
  tree_->writeBinary(s.str().c_str());

  exit(-1);
}

int main(int argc, char ** argv) {
  AppConfig app_cfg;
  app_cfg.repeat = false;
  
  ConciseArgs opt(argc, (char**)argv);
  opt.parse();  
    
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App* app= new App(lcm, app_cfg);
  
  std::cout << "Started commsThread\n";
  while( 0==lcm->handle() );

  return 0;
}
