
struct PFGraspOptions { 
    bool vDEBUG;
    float vSCALE;
    int vAFFORDANCE_ID;
    std::string vCHANNEL, vAFFORDANCE_CHANNEL;

    PFGraspOptions () : 
        vCHANNEL(std::string("CAMERALEFT")), vAFFORDANCE_CHANNEL(std::string("CAMERALEFT_MASKZIPPED")), 
        vSCALE(1.f), vAFFORDANCE_ID(64), vDEBUG(false) {}
};

class PFGrasp{
  public:
    PFGrasp(boost::shared_ptr<lcm::LCM> &lcm_);
    ~PFGrasp() {}
    
  private:
    std::shared_ptr<drc::BotWrapper> mBotWrapper;
    PFGraspOptions _options;
    
    // Parameters for the camera
    cameraParams camera_params;
    String cameraChannelName;
    
    // Img, affordance image
    cv::Mat img, aff_img;
    
    // TLD Tracker
    TLDTracker* tracker;
    
    // Image warpper
    ImageWarper* warper;
    
    // handle reset, run-one-iteration
    void commandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::command_t* msg);
    
    // get image from hand camera 
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::image_t* msg);
    
    // get segment from track segmenter
    void segmentHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::image_t* msg);
}

PFGrasp::PFGrasp(boost::shared_ptr<lcm::LCM> &lcm_): lcm_(lcm_) {
  //param = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 1);
  //frames = bot_frames_get_global(lcm->getUnderlyingLCM(), param);
  mBotWrapper = new BotWrapper();
  
  // Camera Params
  camera_params = CameraParams(mBotWrapper->getBotParam(), "cameras." + _options.vCHANNEL + ".intrinsic_cal");
  
  // subscribing to image, segmenter, commands
  lcm_->subscribe(cameraChannelName.c_str(), &PFGrasp::imageHandler, this);
  lcm_->subscribe(segmenterChannelName.c_str(), &PFGrasp::imageHandler, this);
  lcm_->subscribe(commandChannelName.c_str(), &PFGrasp::imageHandler, this);
  
  // Initialize image IO utils  ?????????????????
  imgutils_aff = new image_io_utils( mBotWrapper->getLcm(), camera_params.width, camera_params.height);

  // Initialize TLD tracker
  tracker = new TLDTracker(camera_params.width, camera_params.height, _options.vSCALE);
  
  // Initialize Image warper
  warper = new ImageWarper();
}



int main(int argc,char** argv) {
  
  PFGraspOptions options;
  
  ConciseArgs opt(argc, (char**)argv);
  opt.add(options.vCHANNEL, "c", "camera-channel","Camera Channel [CAMERALEFT]");
  opt.add(options.vAFFORDANCE_CHANNEL, "a", "affordance-channel","Affordance Channel [CAMERALEFT_MASKZIPPED]");
  opt.add(options.vSCALE, "s", "scale","Scale Factor");
  opt.parse();
  
  PFGrasp pfgrasp();
  
  cout << "\npfgrasp ready\n";
  pfgrasp.start();
  return 0;
}
