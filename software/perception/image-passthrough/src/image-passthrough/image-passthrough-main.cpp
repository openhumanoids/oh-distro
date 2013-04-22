#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <image-passthrough/image-passthrough-app.hpp>

#include <ConciseArgs>
using namespace std;


class Main{
  public:
    Main(int argc, char** argv, boost::shared_ptr<lcm::LCM> &publish_lcm, 
         std::string camera_channel_, int output_color_mode_, bool use_convex_hulls, string camera_frame);
    
    ~Main(){
    }

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);  
    Pass::Ptr pass;
};
    
    
Main::Main(int argc, char** argv, boost::shared_ptr<lcm::LCM> &lcm_, std::string camera_channel,
    int output_color_mode, bool use_convex_hulls, std::string camera_frame): lcm_(lcm_){
     
  pass = Pass::Ptr (new Pass (argc, argv, lcm_, camera_channel, output_color_mode, use_convex_hulls,
                    camera_frame));
  lcm_->subscribe(camera_channel,&Main::imageHandler,this);  
}

void Main::imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg){
  int64_t msg_time = msg->utime;
  if (pass->createMask(msg_time) ){
    pass->sendOutput(msg_time);  
  }
}

int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  string camera_channel="CAMERALEFT";
  int output_color_mode=1; // 0 =rgb, 1=grayscale mask, 2=binary black/white grayscale mask
  bool use_convex_hulls=false;
  string camera_frame = "left_camera_optical_frame";
  parser.add(camera_channel, "c", "camera_channel", "Camera channel");
  parser.add(camera_frame, "f", "camera_frame", "Camera frame");
  parser.add(output_color_mode, "o", "output_color_mode", "0rgb |1grayscale |2b/w");
  parser.add(use_convex_hulls, "u", "use_convex_hulls", "Use convex hull models");
  parser.parse();
  cout << camera_channel << " is camera_channel\n"; 
  cout << camera_frame << " is camera_frame\n"; 
  cout << output_color_mode << " is output_color_mode\n"; 
  cout << use_convex_hulls << " is use_convex_hulls\n"; 
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Main main(argc,argv, lcm, camera_channel,output_color_mode, use_convex_hulls, camera_frame);
  cout << "image-passthrough ready" << endl << endl;
  while(0 == lcm->handle());
  return 0;
}