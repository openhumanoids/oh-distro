#include <estimate/leg_odometry.hpp>
#include <path_util/path_util.h>
#include <ConciseArgs>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

struct CommandLineConfig
{
    std::string param_file;
    std::string urdf_file;
    std::string in_log_name;
    std::string out_log_name;
    bool read_lcmlog;
    int64_t begin_timestamp;
    int64_t end_timestamp;
    bool republish_incoming;
    int processing_rate;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_subscribe_, boost::shared_ptr<lcm::LCM> &lcm_publish_, const CommandLineConfig& cl_cfg_);
    
    ~App(){
    }

  private:
    boost::shared_ptr<lcm::LCM> lcm_subscribe_, lcm_publish_;
    BotParam* botparam_;
    boost::shared_ptr<ModelClient> model_;
    
    const CommandLineConfig cl_cfg_;
    leg_odometry* leg_odo_;
    
    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);
    void viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg);
    
    // logging:
    BotFrames* frames_;
    bot::frames* frames_cpp_;
    //void openLogFile();
    //std::ofstream logfile_;
    //void terminate();    
    
    
};
    
App::App(boost::shared_ptr<lcm::LCM> &lcm_subscribe_,  boost::shared_ptr<lcm::LCM> &lcm_publish_, const CommandLineConfig& cl_cfg_):
          lcm_subscribe_(lcm_subscribe_), lcm_publish_(lcm_publish_), cl_cfg_(cl_cfg_){

  if (cl_cfg_.param_file == ""){
    botparam_ = bot_param_new_from_server(lcm_subscribe_->getUnderlyingLCM(), 0);
  }else{
    //std::string param_file = "drc_robot_02.cfg";            
    std::string param_file_full = std::string(getConfigPath()) +'/' + std::string(cl_cfg_.param_file);
    botparam_ = bot_param_new_from_file(param_file_full.c_str());
  }
  // TODO: not sure what do do here ... what if i want the frames from the file?
  //frames_ = bot_frames_new(NULL, botparam_);
  frames_ = bot_frames_get_global(lcm_subscribe_->getUnderlyingLCM(), botparam_);
  frames_cpp_ = new bot::frames(frames_);  
  
  
  
  
  
  if (cl_cfg_.urdf_file == ""){           
    model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_subscribe_->getUnderlyingLCM(), 0));
  }else{
    //std::string urdf_file = "model_LH_RH.urdf";            
    std::string urdf_file_full = std::string(getModelsPath()) +"/mit_gazebo_models/mit_robot/" + std::string(cl_cfg_.urdf_file);
    model_ = boost::shared_ptr<ModelClient>(new ModelClient( urdf_file_full  ));
  }            
            
  leg_odo_ = new leg_odometry(lcm_subscribe_, lcm_publish_, botparam_, model_);

  // Overwrite the Default parameters
  string init_mode = bot_param_get_str_or_fail(botparam_, "state_estimator.legodo_driven_process.initialization_mode");
  std::cout << "Overwriting leg odom init mode:: " << init_mode << "\n";
  leg_odo_->setInitializationMode( init_mode );
  string leg_odo_mode = bot_param_get_str_or_fail(botparam_, "state_estimator.legodo_driven_process.integration_mode");
  std::cout << "Overwriting leg odom integration mode:: " << leg_odo_mode << "\n";
  leg_odo_->setLegOdometryMode( leg_odo_mode );
  
  

  lcm_subscribe_->subscribe("EST_ROBOT_STATE",&App::robotStateHandler,this);  
  
  if ( cl_cfg_.republish_incoming){
    lcm_subscribe_->subscribe("VICON_BODY|VICON_FRONTPLATE",&App::viconHandler,this);  
  }
  // openLogFile();
  
}

/*
void App::openLogFile(){
  time_t rawtime;
  struct tm * timeinfo;  
  char buffer [80];
  time (&rawtime);
  timeinfo = localtime (&rawtime);

  strftime (buffer,80,"/tmp/se-leg-estimate-result-%Y-%m-%d-%H-%M.txt",timeinfo);
  std::string filename = buffer;
  
  std::cout << "Opening Logfile: "<< filename << "\n";
  logfile_.open ( filename.c_str() );
}

void App::terminate(){
  std::cout << "Closing Logfile: " << "\n";
  logfile_.close();
}
*/

void App::robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  if ( cl_cfg_.begin_timestamp > -1){
    if (msg->utime <  cl_cfg_.begin_timestamp ){
      double seek_seconds = (cl_cfg_.begin_timestamp - msg->utime)*1E-6;
      std::cout << msg->utime << " too early | seeking " << seek_seconds    << "secs, to " << cl_cfg_.begin_timestamp << "\n";
      return;
    }
  }
  if ( cl_cfg_.end_timestamp > -1){
    if (msg->utime >  cl_cfg_.end_timestamp ){
      //terminate();
      std::cout << msg->utime << " finishing\n";
      exit(-1);
      return;
    }    
  }
  
  
  if (cl_cfg_.republish_incoming){
    // Don't publish then working live:
    bot_core::pose_t bdipose = getRobotStatePoseAsBotPose(msg);
    lcm_publish_->publish("POSE_BDI", &bdipose);
  }  
  
  
  Eigen::Isometry3d world_to_body_bdi;
  world_to_body_bdi.setIdentity();
  world_to_body_bdi.translation()  << msg->pose.translation.x, msg->pose.translation.y, msg->pose.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->pose.rotation.w, msg->pose.rotation.x, 
                                               msg->pose.rotation.y, msg->pose.rotation.z);
  world_to_body_bdi.rotate(quat); 
  
  leg_odo_->setPoseBDI( world_to_body_bdi ); 
  leg_odo_->setFootForces(msg->force_torque.l_foot_force_z,msg->force_torque.r_foot_force_z);
  leg_odo_->updateOdometry(msg->joint_name, msg->joint_position,
                           msg->joint_velocity, msg->joint_effort, msg->utime);
  
  Eigen::Isometry3d world_to_body = leg_odo_->getRunningEstimate();
  bot_core::pose_t pose_msg = getPoseAsBotPose(world_to_body, msg->utime);
  lcm_publish_->publish("POSE_BODY_ALT", &pose_msg );   
  
  // Publish Pose as a robot state:
  drc::robot_state_t state_out_msg;
  state_out_msg = *msg;
  insertPoseInRobotState(state_out_msg, world_to_body);
  lcm_publish_->publish("EST_ROBOT_STATE_COMPRESSED_LOOPBACK", &state_out_msg );   

  //logfile_ <<  1 << ", " << msg->utime << ", " << print_Isometry3d(world_to_body_bdi) << "\n";  
  //logfile_ <<  2 << ", " << msg->utime << ", " << print_Isometry3d(world_to_body) << "\n";      
}


void App::viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg){

  Eigen::Isometry3d worldvicon_to_frontplate_vicon;
  worldvicon_to_frontplate_vicon.setIdentity();
  worldvicon_to_frontplate_vicon.translation()  << msg->trans[0], msg->trans[1] , msg->trans[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->quat[0], msg->quat[1], 
                                               msg->quat[2], msg->quat[3]);
  worldvicon_to_frontplate_vicon.rotate(quat); 
  
  // Apply the body to frontplate transform
  Eigen::Isometry3d frontplate_vicon_to_body_vicon;
  frames_cpp_->get_trans_with_utime( "body_vicon" , "frontplate_vicon", msg->utime, frontplate_vicon_to_body_vicon);    
  Eigen::Isometry3d worldvicon_to_body_vicon = worldvicon_to_frontplate_vicon* frontplate_vicon_to_body_vicon;
  
  bot_core::pose_t pose_msg = getPoseAsBotPose(worldvicon_to_body_vicon, msg->utime);
  lcm_publish_->publish("POSE_VICON", &pose_msg );  
  
  //logfile_ <<  0 << ", " << msg->utime << ", " << print_Isometry3d(worldvicon_to_body_vicon) << "\n";    
}


int 
main(int argc, char ** argv){
  CommandLineConfig cl_cfg;
  cl_cfg.urdf_file = "";
  cl_cfg.param_file = "";
  cl_cfg.in_log_name = "";
  cl_cfg.out_log_name = "";
  cl_cfg.read_lcmlog = false;
  cl_cfg.begin_timestamp = -1;
  cl_cfg.end_timestamp = -1;
  cl_cfg.republish_incoming = false;
  cl_cfg.processing_rate = 1;
  
  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg.urdf_file, "U", "urdf_file","urdf_file");
  opt.add(cl_cfg.param_file, "P", "param_file","param_file");
  opt.add(cl_cfg.in_log_name, "L", "in_log_name","in_log_name");
  opt.add(cl_cfg.out_log_name, "l", "out_log_name","out_log_name");
  opt.add(cl_cfg.begin_timestamp, "bt", "begin_timestamp","Run estimation from this timestamp");
  opt.add(cl_cfg.end_timestamp, "et", "end_timestamp","End estimation at this timestamp");  
  opt.add(cl_cfg.republish_incoming, "r", "republish_incoming","Republish Incoming Messages");  
  opt.add(cl_cfg.processing_rate, "pr", "processing_rate","Processing Rate from a log [0=ASAP, 1=realtime]");    
  opt.parse();
  
  std::stringstream lcmurl_in;
  if (cl_cfg.in_log_name == "" ){
    lcmurl_in << "";
  }else{
    cl_cfg.read_lcmlog = true;
    lcmurl_in << "file://" << cl_cfg.in_log_name << "?speed=" << cl_cfg.processing_rate;// + "&start_timestamp=";// + begin_timestamp;
  }
  boost::shared_ptr<lcm::LCM> lcm_subscribe(new lcm::LCM(lcmurl_in.str()) );
  
  std::stringstream lcmurl_out;  
  if (cl_cfg.out_log_name == "" ) {
    lcmurl_out << ""; // mfallon publish back to lcm if run from log
  }else{
    printf("publishing into log file: %s\n", cl_cfg.out_log_name.c_str());
    lcmurl_out << "file://" << cl_cfg.out_log_name << "?mode=w";
  } 
  boost::shared_ptr<lcm::LCM> lcm_publish(new lcm::LCM(lcmurl_out.str()) );
  
  if(!lcm_subscribe->good())
    return 1;  
  if(!lcm_publish->good())
    return 1;  
  
  if (lcm_publish != lcm_subscribe && cl_cfg.republish_incoming) {
    cl_cfg.republish_incoming = true;
  }else{
    // Over-rule if requesting republish 
    cl_cfg.republish_incoming = false;
  }
  
  App app(lcm_subscribe, lcm_publish, cl_cfg);
  while(0 == lcm_subscribe->handle());
  return 0;
}
