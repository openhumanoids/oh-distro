#include <mav_state_est/lcm_front_end.hpp>
#include <mav_state_est/sensor_handlers.hpp>
#include <mav_state_est/rbis_initializer.hpp>
#include <mav_state_est/mav_state_est.hpp>
#include <mav_state_est/gpf/rbis_gpf_update.hpp>

// #include <mav_state_est/gpf-rgbd-lib/rbis_gpf_update.hpp>
#include <mav_state_est/mav-est-fovis/rbis_fovis_update.hpp>
#include <mav_state_est/mav-est-legodo/rbis_legodo_update.hpp>
#include <mav_state_est/mav-est-legodo/rbis_legodo_external_update.hpp>
#include <mav_state_est/pose_meas.hpp>

#include <path_util/path_util.h>


#include <lcmtypes/drc/utime_t.hpp>


#include <ConciseArgs>

using namespace std;
using namespace MavStateEst;

static void shutdown_module(int unused __attribute__((unused)))
{
  fprintf(stderr, "shutting down!\n");
  bot_tictoc_print_stats(BOT_TICTOC_AVG);
  exit(1);
}


class StandingPrep{
public:
    StandingPrep(boost::shared_ptr<lcm::LCM> &lcm_):lcm_(lcm_){
      lcm_->subscribe( "STATE_EST_READY" ,&StandingPrep::navReadyHandler,this);
      ready_ =false;
    }
    ~StandingPrep(){}    
    void listenForReady(){
      std::cout << "Waiting for MAV_STATE_EST_READY before starting\n";
      std::cout << "Send this message when the robot is standing still\n";
      while(0 == lcm_->handle()  && (!ready_) );
    } 
private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool ready_;
    void navReadyHandler(const lcm::ReceiveBuffer* rbuf, 
                           const std::string& channel, const  drc::utime_t* msg){
      std::cout << "Received Ready\n";
      ready_ = true;
    }
};





class App {
public:
  App(int argc, char ** argv) :
      opt(argc, argv)
  {

    string in_log_fname = "";
    string out_log_fname = "";
    string param_file = "";
    string urdf_file = "";
    string override_str = "";
    output_likelihood_filename = "";
    string begin_timestamp= "0";
    smooth_at_end = false;
    double processing_rate = 1; // real time
    bool wait_for_viewer_ready = false;

    ConciseArgs opt(argc, argv);
    opt.add(in_log_fname, "L", "in_log_name", "Run state estimation directly on this log");
    opt.add(out_log_fname, "l", "out_log_name", "publish into this log");
    opt.add(param_file, "P", "param_file", "Pull params from this file instead of LCM");
    opt.add(override_str, "O", "override_params",
        "Override the parameters in BotParam with a '|' seperated list of key=value pairs");
    opt.add(smooth_at_end, "S", "smooth",
        "Run Kalman smoothing and publish poses at end - only works when running from log");
    opt.add(output_likelihood_filename, "M", "meas_like", "save the measurement likelihood to this file");
    opt.add(begin_timestamp, "t", "begin_timestamp", "Run estimation from this timestamp"); // mfallon
    opt.add(urdf_file, "U", "urdf_file", "Pull params from this file instead of LCM"); // mfallon
    opt.add(processing_rate, "pr", "processing_rate","Processing Rate from a log [0=ASAP, 1=realtime]");        
    opt.add(wait_for_viewer_ready, "v", "viewer_ready","Wait for trigger from Viewer to start");
    opt.parse();

    
    if (wait_for_viewer_ready){// Listen for a message from the viewer to start
      boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
      StandingPrep prep(lcm);
      prep.listenForReady();
    }
    
    std::string param_file_full = std::string(getConfigPath()) +'/' + std::string(param_file);    
    if (param_file.empty()) { // get param from lcm
      param_file_full = "";
    }
 

    relaunch_app = false; // processing from log, exit at the end
    if (in_log_fname.empty()) { 
      relaunch_app = true; // listening live, relaunch at exit
    }

 
    //create front end
    front_end = new LCMFrontEnd(in_log_fname, out_log_fname, param_file_full , 
                                override_str,begin_timestamp, processing_rate);
    rbis_initializer = new RBISInitializer(front_end, RBISInitializer::getDefaultState(front_end->param),
        RBISInitializer::getDefaultCov(front_end->param));

    vicon_handler = NULL;
    pose_meas_handler = NULL;
    gps_handler = NULL;
    scan_matcher_handler = NULL;
    laser_gpf_handler = NULL;
    //rgbd_gpf_handler = NULL;
    indexed_measurement_handler = new IndexedMeasurementHandler();
    optical_flow_handler = NULL;
    init_message_handler = NULL;

    if (rbis_initializer->initializingWith("init_message")) {
      init_message_handler = new InitMessageHandler();
      rbis_initializer->addSensor("init_message", &MavStateEst::InitMessageHandler::processMessageInit,
          init_message_handler);
    }

    if (front_end->isActive("viewer") || rbis_initializer->initializingWith("viewer")) {
      front_end->addSensor("viewer", &MavStateEst::IndexedMeasurementHandler::processMessage,
          indexed_measurement_handler);
      rbis_initializer->addSensor("viewer", &MavStateEst::IndexedMeasurementHandler::processMessageInit,
          indexed_measurement_handler);
    }

    if (front_end->isActive("ins") || rbis_initializer->initializingWith("ins")) {
      ins_handler = new InsHandler(front_end->param, front_end->frames);

      // TODO: enable easy switching between inertial sensors:
      if( ins_handler->channel =="MICROSTRAIN_INS"){
        front_end->addSensor("ins", &MavStateEst::InsHandler::processMessage, ins_handler);
        rbis_initializer->addSensor("ins", &MavStateEst::InsHandler::processMessageInit, ins_handler);
      }else if( ins_handler->channel =="ATLAS_IMU_BATCH"){
        front_end->addSensor("ins", &MavStateEst::InsHandler::processMessageAtlas, ins_handler);
        rbis_initializer->addSensor("ins", &MavStateEst::InsHandler::processMessageInitAtlas, ins_handler);
      }else if( ins_handler->channel =="ATLAS_IMU_BATCH_FILTERED"){
        std::cout << "Filtered IMU expected\n";
        // messages will not be incorporated twice, either this or the other - but not both
        front_end->addSensor("ins", &MavStateEst::InsHandler::processMessageAtlas, ins_handler);
        rbis_initializer->addSensor("ins", &MavStateEst::InsHandler::processMessageInitAtlas, ins_handler);
      }
    }

    if (front_end->isActive("gps") || rbis_initializer->initializingWith("gps")) {
      gps_handler = new GpsHandler(front_end->param);
      front_end->addSensor("gps", &MavStateEst::GpsHandler::processMessage, gps_handler);
      rbis_initializer->addSensor("gps", &MavStateEst::GpsHandler::processMessageInit, gps_handler);
    }

    if (front_end->isActive("vicon") || rbis_initializer->initializingWith("vicon")) {
      vicon_handler = new ViconHandler(front_end->param, front_end->frames);
      front_end->addSensor("vicon", &MavStateEst::ViconHandler::processMessage, vicon_handler);
      rbis_initializer->addSensor("vicon", &MavStateEst::ViconHandler::processMessageInit, vicon_handler);
    }

    if (front_end->isActive("pose_meas") || rbis_initializer->initializingWith("pose_meas")) {
      // Correct the pose in manner similar to vicon corrections: typically so as to init the estimator at BDI's position
      pose_meas_handler = new PoseMeasHandler(front_end->param, front_end->frames);
      front_end->addSensor("pose_meas", &MavStateEst::PoseMeasHandler::processMessage, pose_meas_handler);
      rbis_initializer->addSensor("pose_meas", &MavStateEst::PoseMeasHandler::processMessageInit, pose_meas_handler);
    }

    if (front_end->isActive("optical_flow")) {
      optical_flow_handler = new OpticalFlowHandler(front_end->param, front_end->frames);
      front_end->addSensor("optical_flow", &MavStateEst::OpticalFlowHandler::processMessage, optical_flow_handler);
    }

    if (front_end->isActive("scan_matcher")) {
      scan_matcher_handler = new ScanMatcherHandler(front_end->param);
      front_end->addSensor("scan_matcher", &MavStateEst::ScanMatcherHandler::processMessage, scan_matcher_handler);
    }

    if (front_end->isActive("laser_gpf")) {
      laser_gpf_handler = new LaserGPFHandler(front_end->lcm_pub->getUnderlyingLCM(), front_end->param,
          front_end->frames);
      front_end->addSensor("laser_gpf", &MavStateEst::LaserGPFHandler::processMessage, laser_gpf_handler);
    }

    if (front_end->isActive("laser_gpf_out_of_process")) {
      front_end->addSensor("laser_gpf_out_of_process", &IndexedMeasurementHandler::processMessage,
          indexed_measurement_handler);
    }

    //if (front_end->isActive("rgbd_gpf")) {
    //  rgbd_gpf_handler = new RgbdGPFHandler(front_end->lcm_pub->getUnderlyingLCM(), front_end->param,
    //      front_end->frames);
    //  front_end->addSensor("rgbd_gpf", &MavStateEst::RgbdGPFHandler::processMessage, rgbd_gpf_handler);
    //}


    if (front_end->isActive("fovis")) {
      fovis_handler = new FovisHandler(front_end->lcm_recv, front_end->lcm_pub, front_end->param);
      front_end->addSensor("fovis", &MavStateEst::FovisHandler::processMessage, fovis_handler);
    }



    if (front_end->isActive("legodo")) {
      // legodo_handler = new LegOdoHandler(front_end->param);

      ModelClient* model;
      if (urdf_file == ""){           
        model = new ModelClient(  front_end->lcm_recv->getUnderlyingLCM(), 0);
      }else{
        //std::string urdf_file = "model_LH_RH.urdf";            
        std::string urdf_filename_full = std::string(getModelsPath()) +"/mit_gazebo_models/mit_robot/" + std::string( urdf_file );
        model = new ModelClient( urdf_filename_full );
      }  
      legodo_handler = new LegOdoHandler(front_end->lcm_recv, front_end->lcm_pub, front_end->param, model, front_end->frames);

      front_end->addSensor("legodo", &MavStateEst::LegOdoHandler::processMessage, legodo_handler);
    }

    if (front_end->isActive("legodo_external")) {
      legodo_external_handler = new LegOdoExternalHandler(front_end->lcm_recv, front_end->lcm_pub, front_end->param);
      front_end->addSensor("legodo_external", &MavStateEst::LegOdoExternalHandler::processMessage, legodo_external_handler);
    }
    
    
    
    restart_sub =  front_end->lcm_recv->subscribe( "STATE_EST_RESTART" ,&App::restartHandler,this);

  }

  void restartHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::utime_t* msg){
    // exit 
    front_end->exit_estimator = true; 
    
    front_end->lcm_recv->unsubscribe( restart_sub );
  }
  
  
  void run()
  {
    //initialization
    RBIS init_state;
    RBIM init_cov;
    rbis_initializer->initialize(init_state, init_cov);

    MavStateEstimator * state_est = new MavStateEstimator(
        new RBISResetUpdate(init_state, init_cov, MavStateEst::RBISUpdateInterface::reset, init_state.utime),
        front_end->param);

    front_end->setStateEstimator(state_est);

    if (smooth_at_end) {
      front_end->smooth(ins_handler->dt);
    }
    else {
      front_end->run();
    }

    front_end->outputLogLikeLihood(output_likelihood_filename);
  }
  ConciseArgs opt;

  LCMFrontEnd * front_end;
  RBISInitializer * rbis_initializer;

  ViconHandler * vicon_handler;
  PoseMeasHandler * pose_meas_handler;
  InsHandler * ins_handler;
  GpsHandler * gps_handler;
  LaserGPFHandler * laser_gpf_handler;
  //RgbdGPFHandler * rgbd_gpf_handler;
  FovisHandler * fovis_handler;
  LegOdoHandler * legodo_handler;
  LegOdoExternalHandler * legodo_external_handler;
  IndexedMeasurementHandler * indexed_measurement_handler;
  ScanMatcherHandler * scan_matcher_handler;
  OpticalFlowHandler * optical_flow_handler;
  InitMessageHandler * init_message_handler;

  bool filter_initialized;
  MavStateEstimator * state_estimator;
  int64_t history_span;

  string output_likelihood_filename;
  bool smooth_at_end;
  
  lcm::Subscription * restart_sub;
  
  bool relaunch_app;
};



bool launchApp(int argc, char **argv){
  
  App * app = new App(argc, argv);
  app->run();
  
  bool relaunch_app = app->relaunch_app;
  
  delete app->front_end;
  delete app;
  
  return relaunch_app;
}



int main(int argc, char **argv)
{
  signal(SIGINT, shutdown_module);
  
  bool relaunch_app = true;
  while (relaunch_app){ // mfallon: added infinite loop, application continually restarts
    relaunch_app = launchApp(argc, argv);
  }

  shutdown_module(1);
  return 0;
}
