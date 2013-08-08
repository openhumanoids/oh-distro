#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <boost/shared_ptr.hpp>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/multisense.hpp>

#include <ConciseArgs>

using namespace std;
using namespace cv;



class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg);   
    
    void doGradient(cv::Mat &src, cv::Mat &dst);


};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_): lcm_(lcm_){

  lcm_->subscribe( "CAMERA" ,&Pass::multisenseHandler,this);

    
}



void Pass::doGradient(cv::Mat &src, cv::Mat &dst){
  

  
}


void Pass::multisenseHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  multisense::images_t* msg){
  cout << "incoming width " << msg->images[0].width << "\n";
  int h = msg->images[1].height;
  int w = msg->images[1].width;
//  cv::Mat warped_tile(800, 800, CV_16U , warped_tile_data);

  cv::Mat disparity_orig_temp = cv::Mat::zeros(h,w,CV_16UC1); // h,w
  memcpy ( disparity_orig_temp.data , msg->images[1].data.data() , h*w*sizeof(short) ) ;    
  
  cv::Mat src = cv::Mat::zeros(h,w,CV_16UC1); // h,w
  cv::Mat dst = cv::Mat::zeros(h,w,CV_16UC1); // h,w
  
  int morph_elem = 0;
  int morph_size = 2;
  int morph_operator = 2;
  // Since MORPH_X : 2,3,4,5 and 6
  int operation = morph_operator + 2;

  std::cout << morph_elem << " " << morph_size << " " << operation << "\n";
  Mat element = getStructuringElement( morph_elem, 
                   Size( 2*morph_size + 1, 2*morph_size+1 ), 
                   Point( morph_size, morph_size ) );

  /// Apply the specified morphology operation
  morphologyEx( disparity_orig_temp, dst, operation, element );
  
  
  cv::imwrite("test.png", disparity_orig_temp);
  cv::imwrite("test_out.png", dst);
  //cv::Mat_<float> disparity(header.height, header.width, 
    // const_cast<float*>(reinterpret_cast<const float*>(imageDataP)));
  
  cv::imshow( "Gray image", disparity_orig_temp );

  imshow( "Grad", dst );  
  
   waitKey(0);      
    
    
}


int main(int argc, char ** argv) {
  
//   string lidar_channel = "SCAN";
//   ConciseArgs opt(argc, (char**)argv);
//   opt.add(lidar_channel, "l", "lidar_channel","lidar_channel");
//   opt.parse();
//   std::cout << "lidar_channel: " << lidar_channel << "\n"; 

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
