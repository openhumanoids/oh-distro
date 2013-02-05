// Listen to features, images and registeration triggers
// - when features received, find corresponding image from buffer of images and pair
// - when trigger is received, these two become the reference
// - for subsequent feature message, match the features and image with the reference and publish.

#include <iostream>
#include <Eigen/Dense>
#include <deque>

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>

#include <dirent.h>


#include "registeration.hpp"

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/drc_lcmtypes.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/registeration.hpp>


#define DO_TIMING_PROFILE TRUE
#include <ConciseArgs>


using namespace std;
using namespace Eigen;
using namespace cv;
using namespace boost;
using namespace boost::assign; // bring 'operator+()' into scope

class RegApp{
  public:
    RegApp(boost::shared_ptr<lcm::LCM> &publish_lcm, std::string camera_);
    
    ~RegApp(){
    }
    
    void doReg();
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void featuresHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  reg::features_t* msg);
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    void registerCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::map_params_t* msg);
    std::vector<ImageFeature> getFeaturesFromLCM(const  reg::features_t* msg);
    std::string camera_;

    pointcloud_vis* pc_vis_;
    
    Reg::Ptr reg;
    std::vector<ImageFeature> cur_features_;
    bot_core::image_t cur_image_;
    
    std::vector<ImageFeature> ref_features_;
    bot_core::image_t ref_image_;
    
    bool registeration_active_ ; // are we currently doing registeration
    
    deque< bot_core::image_t  > * image_queue_;
};



RegApp::RegApp(boost::shared_ptr<lcm::LCM> &lcm_, std::string camera_):          
    lcm_(lcm_),camera_(camera_), registeration_active_(false){

  lcm_->subscribe("FEATURES",&RegApp::featuresHandler,this);  
  lcm_->subscribe(camera_ ,&RegApp::imageHandler,this);  
  
  // In progress:
  lcm_->subscribe("MAP_CREATE",&RegApp::registerCommandHandler,this);  

  reg = Reg::Ptr (new Reg (lcm_));
  
  image_queue_ = new deque< bot_core::image_t > ();
  
}

void RegApp::doReg(){
  // grey assumed:
  Mat ref_img = Mat::zeros( ref_image_.height , ref_image_.width ,CV_8UC1); // h,w
  ref_img.data = ref_image_.data.data();
  Mat cur_img = Mat::zeros( cur_image_.height , cur_image_.width ,CV_8UC1); // h,w
  cur_img.data = cur_image_.data.data();
    
  //imwrite("ref.png",ref_img);
  //imwrite("cur.png",cur_img);
      
  //FrameMatchPtr match =  
  reg->align_images(ref_img, cur_img, ref_features_, cur_features_, ref_image_.utime, cur_image_.utime );
}

std::vector<ImageFeature> RegApp::getFeaturesFromLCM(const  reg::features_t* msg){
  std::vector<ImageFeature> features;
    
  for (int i =0; i < msg->nfeatures; i++){ 
    ImageFeature f;
    
    f.track_id = msg->features[i].track_id;
    f.uv[0] = msg->features[i].uv[0];
    f.uv[1] = msg->features[i].uv[1];
    f.base_uv[0] = msg->features[i].base_uv[0];
    f.base_uv[1] = msg->features[i].base_uv[1];
    f.uvd[0] = msg->features[i].uvd[0];
    f.uvd[1] = msg->features[i].uvd[1];
    f.uvd[2] = msg->features[i].uvd[2];
    f.xyz[0] = msg->features[i].xyz[0];
    f.xyz[1] = msg->features[i].xyz[1];
    f.xyz[2] = msg->features[i].xyz[2];
    f.xyzw[0] = msg->features[i].xyzw[0];
    f.xyzw[1] = msg->features[i].xyzw[1];
    f.xyzw[2] = msg->features[i].xyzw[2];
    f.xyzw[3] = msg->features[i].xyzw[3];
    // color left out for now
    
    /*
    cout << line << " is line\n";
    cout << "i: " << i <<"\n";
    cout << "f.track_id: " << f.track_id <<"\n";
    cout << "f.uv: " << f.uv[0] << " "<< f.uv[1] <<"\n";
    cout << "f.base_uv: " << f.base_uv[0] << " "<< f.base_uv[1] <<"\n";
    cout << "f.uvd: " << f.uvd[0] << " "<< f.uvd[1]<< " "<< f.uvd[2]<<"\n";
    cout << "f.xyz: " << f.xyz[0] << " "<< f.xyz[1]<< " "<< f.xyz[2]<<"\n";
    cout << "f.xyzw: " << f.xyzw[0] << " "<< f.xyzw[1]<< " "<< f.xyzw[2]<< " "<< f.xyzw[3]<<"\n";
    cout << "f.color: " << (int)f.color[0] << " "<< (int)f.color[1] << " "<< (int)f.color[2] <<"\n";
      */
    features.push_back(f);
  }
  
  cout << "in: " << msg->nfeatures << " | out: "<< features.size() << "\n"; 
  return features;  
}

void RegApp::featuresHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  reg::features_t* msg){
  cout << "got features @ "<< msg->utime<<"\n";
  
  // now find the corresponding image and pair it
  bool feat_image_matched_ = false;
  for(int i=0; i < image_queue_->size(); i++){
    cout << image_queue_->at(i).utime << " " << i << "\n";
    if (msg->utime == image_queue_->at(i).utime){
      cout << "image and features matched: " << i << " " << image_queue_->at(i).utime << "\n"; 
      cur_features_ = getFeaturesFromLCM(msg);
      cur_image_ = image_queue_->at(i);
      feat_image_matched_ = true;
      break;
    }
  }
  
  
  // If the registartation have been activated, do it live:
  if (registeration_active_ && feat_image_matched_ ){
    doReg();
  }else{
   cout << "dyuck\n"; 
  }
}


void RegApp::imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg){
  cout << "got image @ "<< msg->utime <<"\n";
  
  // Keep a buffer/deque of left images - should only require a few images
  bot_core::image_t msg_cpy = *msg;
  image_queue_->push_back(  msg_cpy );
  
  //cout << "image_queue_ size: " << image_queue_->size() << "\n";
  if( image_queue_->size() > 10){
    image_queue_->pop_front();
  }
}


void RegApp::registerCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::map_params_t* msg){
  // TODO: add check that we've seen last_features
  cout << "got command\n";
  
  ref_features_ = cur_features_;
  ref_image_ = cur_image_;
  // set the reference image
  cout << "Will now register to " << ref_features_.size() << " features\n";
  
  
  
  registeration_active_ = true;
}



int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "registeration-app");
  string camera="CAMLCM_IMAGE_GRAY_LEFT";
  parser.add(camera, "c", "camera", "Camera channel");
  parser.parse();
  cout << camera << " is camera\n"; 
  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  RegApp app(lcm, camera);
  cout << "registeration is ready" << endl << endl;
  while(0 == lcm->handle());
  return 0;
}
