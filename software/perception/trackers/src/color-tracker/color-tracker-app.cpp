#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <ConciseArgs>

using namespace cv;
using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_,
      int resize_, int jpeg_quality_, int mode , int downsample_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   

    void sendOutput();

    int jpeg_quality_;

    int width_;
    int height_;
    int counter_;
    int resize_;
    std::string image_channel_;

    int mode_;
    int downsample_;

    image_io_utils*  imgutils_;
  
    bot_core::image_t last_img_;  
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_, 
           int resize_, int jpeg_quality_, int mode_, int downsample_):
    lcm_(lcm_), resize_(resize_), jpeg_quality_(jpeg_quality_), mode_(mode_),
    image_channel_(image_channel_), downsample_(downsample_){

  lcm_->subscribe( image_channel_ ,&Pass::imageHandler,this);

  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), width_, height_ );
  width_ = 800;
  height_ =800;
  counter_=0;
  last_img_.utime=0; // used to indicate no message recieved yet
}





IplImage* GetThresholdedImage(IplImage* img){
  // Convert the image into an HSV image
  IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
  cvCvtColor(img, imgHSV, CV_BGR2HSV);

  IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);

  // Values 20,100,100 to 30,255,255 working perfect for yellow at around 6pm
  //cvInRangeS(imgHSV, cvScalar(112, 100, 100), cvScalar(124, 255, 255), imgThreshed);

  // Yellow:
  //cvInRangeS(imgHSV, cvScalar(0, 100, 100), cvScalar(30, 255, 255), imgThreshed);
  // Orange (tropicana:
  //cvInRangeS(imgHSV, cvScalar(10, 50, 50), cvScalar(15, 255, 255), imgThreshed);
  // red bowl:
  //cvInRangeS(imgHSV, cvScalar(0, 50, 50), cvScalar(8, 255, 255), imgThreshed);
  // green (top of lemon juice):
  cvInRangeS(imgHSV, cvScalar(55, 50, 50), cvScalar(65, 255, 255), imgThreshed);

  cvReleaseImage(&imgHSV);

  return imgThreshed;
}




void Pass::sendOutput(){
  if (last_img_.utime==0){     return;   } // if no msg recieved then ignore output command
    
    Mat src= Mat::zeros( last_img_.height,last_img_.width  ,CV_8UC3);
    src.data = last_img_.data.data();


  IplImage* frame = new IplImage(src);

  // If this is the first frame, we need to initialize it
//  if(imgScribble == NULL){
//    imgScribble = cvCreateImage(cvGetSize(frame), 8, 3);
//  }

  // Holds the yellow thresholded image (yellow = white, rest = black)
  IplImage* imgColorThresh = GetThresholdedImage(frame);

  // Calculate the moments to estimate the position of the ball
  CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
  cvMoments(imgColorThresh, moments, 1);

  // The actual moment values
  double moment10 = cvGetSpatialMoment(moments, 1, 0);
  double moment01 = cvGetSpatialMoment(moments, 0, 1);
  double area = cvGetCentralMoment(moments, 0, 0);

  // Holding the last and current ball positions
  static int posX = 0;
  static int posY = 0;

  int lastX = posX;
  int lastY = posY;

  posX = moment10/area;
  posY = moment01/area;

  // Print it out for debugging purposes
  cout << area << "\n";
  printf("position (%d,%d)\n", posX, posY);

  // We want to draw a line only if its a valid position
  if (area> 30){
    if(lastX>0 && lastY>0 && posX>0 && posY>0){
      // Draw a yellow line from the previous point to the current point
     cvLine(frame, cvPoint(posX, posY), cvPoint(posX, posY), cvScalar(0,255,255), 5);
     // cvLine(imgScribble, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,255,255), 5);
    }
  }


  cv::Mat imgMat(frame);


  imgutils_->sendImage(imgMat.data, last_img_.utime, last_img_.width, 
                        last_img_.height, 3, string(image_channel_ + "_TRACKING")  );
  
}

void Pass::imageHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_core::image_t* msg){
  counter_++;
  if (counter_%30 ==0){ cout << counter_ << " | " << msg->utime << "\n";   }  
  if (width_ != msg->width){
    cout << "incoming width " << msg->width << " doesn't match assumed width " << width_ << "\n";
    cout << "returning cowardly\n";
    return;
  }

  last_img_= *msg;  
  sendOutput();
}



int main(int argc, char ** argv) {

  int jpeg_quality = 50;
  string channel = "CAMERALEFT";
  int resize = 4;
  int mode=0;
  int downsample = -1;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(jpeg_quality, "j", "jpeg_quality","jpeg_quality");
  opt.add(channel, "c", "channel","channel");
  opt.add(resize, "r", "resize","resize image by this factor");
  opt.add(mode, "m", "mode","0=rgbinJPEGREDUCEDOUT 1=zipinGRAYOUT");
  opt.add(downsample, "d", "downsample","Downsample Factor");
  opt.parse();
  std::cout << "jpeg_quality: " << jpeg_quality << "\n";  
  std::cout << "channel: " << channel << "\n";  
  std::cout << "resize: " << resize << "\n";
  std::cout << "mode: " << mode << "\n";    
  std::cout << "downsample: output 1 in every [" << downsample << "] frames\n";
  std::cout << "            (set as -1 to output none regularly)\n";    
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,channel, resize, jpeg_quality, mode, downsample);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
