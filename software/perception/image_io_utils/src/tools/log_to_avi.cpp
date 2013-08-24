// DRC-tags
// - detects tags (using april tags)
// - pairs detections with affordances in a library
// - updates the affordances in the affordances in the store
//
// TODO: read library from file
// TODO: add rate limiting (process is at about 15Hz)

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>

#include <lcm/lcm-cpp.hpp>

#include <opencv2/opencv.hpp>
#include <lcmtypes/bot_core.hpp>

#include <ConciseArgs>

using namespace std;


cv::VideoWriter outputVideo;                    // Open the output


class Tags{
  public:
    Tags(boost::shared_ptr<lcm::LCM> &lcm_, string camera_chan_);
    
    ~Tags(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    string camera_chan_;
    std::string camera_frame_;
    int width_, height_;
    double fx_, fy_, cx_, cy_;
    
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
};

Tags::Tags(boost::shared_ptr<lcm::LCM> &lcm_, string camera_chan_):
    lcm_(lcm_), camera_chan_(camera_chan_){
 
  lcm_->subscribe( camera_chan_ ,&Tags::imageHandler,this);
  
   cv::Size size2 = cv::Size(640,480);  

   int codec = CV_FOURCC('M', 'J', 'P', 'G');
  cv::VideoWriter aoutputVideo = cv::VideoWriter("video.avi",codec,15.0,size2,true);  
  
  if (!aoutputVideo.isOpened())
  {
    cout  << "Could not open the output video for write" << endl;
  //  exit(-1);
  }    
  
}
    

int counter =0;
void Tags::imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg){
  std::cout << msg->utime << " chans\n";
  
  cv::Mat img(cv::Size(640,480),CV_8UC3);
  //img.data = msg->data;
  
  outputVideo << img;

  counter++;
  if (counter > 100){
    outputVideo.release();

    exit(-1);
  }
}



int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "drc-tags");
  string camera_chan="WEBCAM";
  parser.add(camera_chan, "v", "camera_chan", "Camera Channel");
  parser.parse();
  cout << camera_chan << " is camera_chan\n";
  
  std::string filename = "/home/mfallon/data/atlas/2013-08-20-atlas-walking/lcmlog-2013-08-20.07_log03";
  std::string filename_url = "file://" + filename;
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM ( filename_url    )  );
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Tags app(lcm,camera_chan);
  cout << "Ready to find tags" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}