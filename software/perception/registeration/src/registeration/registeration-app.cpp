#include <iostream>
#include <Eigen/Dense>

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
    std::vector<ImageFeature> last_features_;
    std::vector<ImageFeature> ref_features_;
    bool registeration_active_ ; // are we currently doing registeration
};



RegApp::RegApp(boost::shared_ptr<lcm::LCM> &lcm_, std::string camera_):          
    lcm_(lcm_),camera_(camera_), registeration_active_(false){

  lcm_->subscribe("FEATURES",&RegApp::featuresHandler,this);  
  lcm_->subscribe("CAMERA",&RegApp::imageHandler,this);  
  
  // In progress:
  lcm_->subscribe("MAP_CREATE",&RegApp::registerCommandHandler,this);  

  reg = Reg::Ptr (new Reg (lcm_));
}

void RegApp::doReg(){
  std::vector<string> futimes;
  std::vector<string> utimes_strings;
  
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (".")) != NULL) {
    /* print all the files and directories within directory */
    while ((ent = readdir (dir)) != NULL) {
      string fname = ent->d_name;
      if(fname.size() > 5){
        if (fname.compare(fname.size()-4,4,"feat") == 0){ 
          printf ("%s\n", ent->d_name);
          futimes.push_back( fname.substr(0,21) );
          utimes_strings.push_back( fname.substr(5,16) );
        }
      }
    }
    closedir (dir);
  } else {
    /* could not open directory */
    perror ("");
    exit(-1);
  }
  
  string temp_string;
  
  istringstream temp_buffer( temp_string );
  string main_fname;
  temp_buffer >> main_fname; 
  
  string temp = main_fname.substr(5,16);
  istringstream temp_buffer2( temp);
  int64_t main_utime;
  temp_buffer2 >> main_utime ; 
  
  cout << main_fname << " fname\n";
  cout << main_utime << " utime\n";
  
  stringstream ifile0, featfile0;
  ifile0 << main_fname << "_left.png";
  featfile0 << main_fname << ".feat";
  cv::Mat img0 = cv::imread( ifile0.str(), CV_LOAD_IMAGE_GRAYSCALE );
  std::vector<ImageFeature> features0;
  // read_features(featfile0.str(), features0);
  
  
  for (size_t i= 0 ; i<  futimes.size(); i++){
    istringstream buffer(utimes_strings[i]);
    int64_t utime;
    buffer >> utime; 

    cout << i << " count\n";
    cout << "doing: " << i << " - "<<futimes[i] <<"\n";
    cout << " and   " << utimes_strings[i] <<"\n";
    cout << " utime " << utime << "\n";
    stringstream ifile1, featfile1;
    ifile1 << futimes[i] << "_left.png";
    featfile1 << futimes[i] << ".feat";

    /// 1. Read in imgs and featues:
    cv::Mat img1 = cv::imread( ifile1.str(), CV_LOAD_IMAGE_GRAYSCALE );
  
    std::vector<ImageFeature> features1;
    //read_features(featfile1.str(), features1);
    
    //FrameMatchPtr match =  
    reg->align_images(img0, img1, features0, features1, main_utime, utime );

    
    int incoming;
    cin >> incoming;
    cout << "end\n";
  }

  
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
  cout << "got features\n";
  
  last_features_ = getFeaturesFromLCM(msg);
  // now find the corresponding image and pair it
  
  // If the registartation have been activated, do it live:
  if (registeration_active_ ){
    //doReg();
  }
}


void RegApp::imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg){
  cout << "got image\n";
  
  // Keep a buffer/deque of left images - should only require a few images
}


void RegApp::registerCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::map_params_t* msg){
  // TODO: add check that we've seen last_features
  cout << "got command\n";
  
  ref_features_ = last_features_;
  // set the reference image
  cout << "Will now register to " << ref_features_.size() << " features\n";
  
  registeration_active_ = true;
}



int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "registeration-app");
  string camera="CAMERALEFT";
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
