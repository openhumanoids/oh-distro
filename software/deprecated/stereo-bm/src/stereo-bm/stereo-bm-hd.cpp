#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "stereo-bm.hpp"
#include <ConciseArgs>

using namespace cv;
using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_, float scale_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   

    std::string image_channel_;
    StereoB*  stereob_;
    
    image_io_utils*  imgutils_;    
    
    std::vector <cv::Mat> disps_;    
    
    int16_t* img_buffer_;
    int img_buffer_size_;
    
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_, float scale):
    lcm_(lcm_), image_channel_(image_channel_){

  lcm_->subscribe( image_channel_ ,&Pass::imageHandler,this);
  stereob_ = new StereoB(lcm_);
  stereob_->setScale(scale);
  
  int w =800;
  int h=800;
  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), w, 2*h); // extra space for stereo tasks
  
  
    img_buffer_size_ = 800 * 800 * sizeof(int16_t) * 4;
    img_buffer_= new int16_t[img_buffer_size_];  // x4 was used for zlib in kinect_lcm
  
}


double getMedian(vector<short> scores){
  double median;
  size_t size = scores.size();

  sort(scores.begin(), scores.end());

  if (size  % 2 == 0)
  {
      median = (scores[size / 2 - 1] + scores[size / 2]) / 2;
  }
  else 
  {
      median = scores[size / 2];
  }

  return median;
}

double getMean(vector<short> scores){
  short sum = std::accumulate(scores.begin(), scores.end(), 0.0);
  double mean = (double) sum / scores.size();
  return mean;
}

void Pass::imageHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_core::image_t* msg){
  cv::Mat left_img, right_img;

  int w = msg->width;
  int h = msg->height/2;
  
  if (left_img.empty() || left_img.rows != h || left_img.cols != w)
        left_img.create(h, w, CV_8UC1);
  if (right_img.empty() || right_img.rows != h || right_img.cols != w)
        right_img.create(h, w, CV_8UC1);

  imgutils_->decodeStereoImage(msg, left_img.data, right_img.data);
  stereob_->doStereoB(left_img, right_img);
  
  disps_.push_back( stereob_->getDisparityMat() );
  
  

    if (disps_.size() > 30){
    std::cout << "got 10\n";

    //cv::Mat img(disps_[0].size(),disps_[0].type());
    cv::Mat img = disps_[0];


    std::cout << disps_[0].size().width << " | "
    << disps_[0].size().height << "\n";

    std::cout << img.size().width << " | "
    << img.size().height << "\n";


    
    
    int ndisps = disps_.size();
    for (int i=0; i < ( 800 ); i++ ){
      for (int k=0; k < ( 800 ); k++ ){
        std::vector <short> samples;
        std::stringstream ss;
        
        for (size_t j=0; j < ndisps ; j++){
          int16_t* data = (int16_t*) disps_[j].data;
          samples.push_back( (short) data[i*800+k] );
          ss << (int) data[i*800+k] << " ";
        }

        //img.data[i] = (int) round(getMean(samples));      
        short val = (short) round(getMedian(samples));

        img_buffer_[i*800 + k] = val;
        ss <<  " | "<< (short) val <<" | " << (i*800+k) ;
        if ( (i*800+k) % 5000 ==0){
          std::cout << ss.str() << "\n";
        }

      }
    }
    
    img.data= (uint8_t*) img_buffer_;// (short)val;


    stereob_->setDisparityMat(img);
    imwrite("blah.png",img);
    stereob_->sendRangeImage(msg->utime);
    disps_.clear();
    std::cout  << "done\n";
    
  }
  
  
}

int main(int argc, char ** argv) {
  string channel = "CAMERA";
  float scale = 0.25;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(channel, "c", "channel","channel");
  opt.add(scale, "s", "scale","scale");
  opt.parse();
  std::cout << "channel: " << channel << "\n";  
  std::cout << "scale: " << scale << "\n";  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,channel,scale);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}