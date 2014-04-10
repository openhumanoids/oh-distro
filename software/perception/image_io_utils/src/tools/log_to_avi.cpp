// Simple tool to parse a log file and dump out an avi file

#include <signal.h>
#include <stdlib.h>

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>

#include <lcm/lcm-cpp.hpp>

#include <opencv2/opencv.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/multisense.hpp>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

#include <ConciseArgs>

using namespace std;

cv::VideoWriter output_video;

struct CommandLineConfig
{
    string codec;
    std::string camera_chan;
    int frames;
    string logfile;
    string outputfile;
    int fps;
};


class Tags{
  public:
    Tags(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_);
    
    ~Tags(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    string camera_chan_;
    std::string camera_frame_;
    int width_, height_;
    const CommandLineConfig& cl_cfg_;
    int codec_val_;
    
    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg);   
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    void doEncode();
    
    bot_core::image_t img_;  
    image_io_utils*  imgutils_; 
    uint8_t* img_buf_; 
    
    int64_t utime_init_;
};

Tags::Tags(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_):
    lcm_(lcm_), cl_cfg_(cl_cfg_){
 
  if ((cl_cfg_.camera_chan == "CAMERA")){// ||  (cl_cfg_.camera_chan == "CAMERA_LEFT") ){
    lcm_->subscribe( cl_cfg_.camera_chan ,&Tags::multisenseHandler,this);
  }else{
    lcm_->subscribe( cl_cfg_.camera_chan ,&Tags::imageHandler,this); 
  }
  
  if (cl_cfg_.codec  == "flv1"){
    codec_val_ = CV_FOURCC('F','L','V','1');
  }else if (cl_cfg_.codec == "divx"){
    codec_val_ = CV_FOURCC('D','I','V','X');
  }else if (cl_cfg_.codec == "h263"){
    codec_val_ = CV_FOURCC('U','2','6','3');
  }else { // uncompressed
    codec_val_ =0; 
  }
   
  // left these numbers very large:
  img_buf_= (uint8_t*) malloc(3* 1524  * 1544);
  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), 
                                  1524, 
                                  3*1544 );  
}
    

int counter =0;

void Tags::multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg){
  img_= msg->images[0];    
  doEncode();
}

void Tags::imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg){
  img_= *msg;
  doEncode();
}
  
void Tags::doEncode(){  
  if (counter ==0){
    utime_init_ = img_.utime;
    
    cv::Size size2 = cv::Size( img_.width, img_.height);  
    output_video = cv::VideoWriter( cl_cfg_.outputfile ,codec_val_, ((float)cl_cfg_.fps) ,size2,true);  
    
    if (!output_video.isOpened()){
      cout  << "Could not open the output to write:" << endl;
      cout  << cl_cfg_.outputfile << "\n";
      exit(-1);
    }    
  }
  
  if (counter%30 == 0){
    char buff[100];
    sprintf(buff, "%f", (img_.utime - utime_init_) /1E6);
    string temp = buff;
    
    std::cout << (counter+1) << " frames | " << cl_cfg_.camera_chan
            << " " << cl_cfg_.codec << " | " << temp  << " log time "
            << img_.width << " width " << img_.height << " height\n";
  }

  imgutils_->decodeImageToRGB( &img_,  img_buf_ );
  cv::Mat img(cv::Size( img_.width,img_.height),CV_8UC3);
  img.data = img_buf_;
  cv::cvtColor(img, img, CV_BGR2RGB);
  output_video << img;

  counter++;
  
  if (cl_cfg_.frames == -1){
  }else if (counter > cl_cfg_.frames){
    output_video.release();
    exit(-1);
  }
}


void my_handler(int s){
  printf("Caught signal %d | finishing video file\n",s);
  output_video.release();

  exit(1); 
}


int main( int argc, char** argv ){

  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);  

  CommandLineConfig cl_cfg;
  cl_cfg.codec ="divx";
  cl_cfg.camera_chan = "WEBCAM";  
  cl_cfg.frames = -1;
  cl_cfg.outputfile = "";
  cl_cfg.fps = 60;
  
  cout << "Convert images within LCM log to avi\n";
  cout << "Example Usage:\n";
  cout << "  lcm-log-to-avi -m WEBCAM -l logfile\n";
  cout << "  lcm-log-to-avi -m CAMERA -l logfile\n";
  cout << "  lcm-log-to-avi -m CAMERACHEST_LEFT -l logfile\n";
  ConciseArgs parser(argc, argv, "");
  parser.add(cl_cfg.camera_chan, "m", "camera_chan", "Camera Channel e.g. WEBCAM, CAMERA, CAMERACHEST_LEFT");
  parser.add(cl_cfg.codec, "c", "codec", "Codec: none, divx, flv1, h264");
  parser.add(cl_cfg.frames, "f", "frames", "No. of Frames to process (-1 = all)");
  parser.add(cl_cfg.logfile, "l", "logfile", "Path to the LCM log");
  parser.add(cl_cfg.outputfile, "o", "outputfile", "Output AVI file (empty means use logfile.avi)");
  parser.add(cl_cfg.fps, "s", "fps", "Frame rate of output video");
  parser.parse();
  cout << cl_cfg.codec << " is codec\n";
  cout << cl_cfg.camera_chan << " is camera_chan\n";
  cout << cl_cfg.frames << " is frames\n";
  cout << cl_cfg.logfile << " is logfile\n";
  if (cl_cfg.outputfile.empty()){
    cl_cfg.outputfile = string(cl_cfg.logfile + ".avi");
  }
  cout << cl_cfg.outputfile << " is outputfile\n";
  
  std::string filename_url = "file://" + cl_cfg.logfile + "?speed=0"; // as quick as it can
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM ( filename_url    )  );
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Tags app(lcm,cl_cfg);
  cout << "Ready to find tags" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
