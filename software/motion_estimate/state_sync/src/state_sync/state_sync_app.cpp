#include <state_sync/state_sync.hpp>
#include <ConciseArgs>

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, boost::shared_ptr<StateSyncConfig> &cl_cfg_);

    ~App(){
    }

    state_sync* ss_;

  private:
    boost::shared_ptr<StateSyncConfig> cl_cfg_;
    boost::shared_ptr<lcm::LCM> lcm_;

};

App::App(boost::shared_ptr<lcm::LCM> &lcm_, 
                       boost::shared_ptr<StateSyncConfig> &cl_cfg_):
                       lcm_(lcm_), cl_cfg_(cl_cfg_){

  ss_ = new state_sync (lcm_, cl_cfg_);
}

int
main(int argc, char ** argv){
  boost::shared_ptr<StateSyncConfig> cl_cfg(new StateSyncConfig() );  
  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg->standalone_head, "l", "standalone_head","Standalone Head");
  opt.add(cl_cfg->standalone_hand, "f", "standalone_hand","Standalone Hand");
  opt.add(cl_cfg->bdi_motion_estimate, "b", "bdi","Use POSE_BDI to make EST_ROBOT_STATE");
  opt.add(cl_cfg->simulation_mode, "s", "simulation","Simulation mode - output TRUE RS");
  opt.add(cl_cfg->output_channel, "o", "output_channel","Output Channel for robot state msg");
  opt.add(cl_cfg->publish_pose_body, "p", "publish_pose_body","Publish POSE_BODY when in BDI mode");
  opt.parse();
  
  std::cout << "standalone_head: " << cl_cfg->standalone_head << "\n";
  std::cout << "publish_pose_body: " << cl_cfg->publish_pose_body << "\n";

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  App app(lcm, cl_cfg);
  while(0 == lcm->handle());
  return 0;
}

