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



void removeEdges(cv::Mat &src) {
  // 1 Convert to 16S as morphologyEx doesnt support 16U
  Mat src_16s;  
  src.convertTo(src_16s, CV_16S);

  // 2 Determain Morpological Gradient:
  int morph_elem = 0;
  int morph_size = 2; // is too weak
  int morph_operator = 4;
  Mat element = getStructuringElement( morph_elem, 
            Size( 2*morph_size + 1, 2*morph_size+1 ),
            Point( morph_size, morph_size ) );
  Mat dst;
  morphologyEx( src_16s, dst, morph_operator, element );
  //imwrite("test_morph_gray.png",dst);

  // 3 Apply a binary threshold on the gradient and remove high gradient elements
  short unsigned int grad_threshold = 254;
  for(int i=0; i<dst.rows; i++)
    for(int j=0; j<dst.cols; j++) 
      if (  dst.at<short unsigned int>(i,j) > grad_threshold)
        src.at<short unsigned int>(i,j)  = 0;
}




void Pass::multisenseHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  multisense::images_t* msg){
  int h = msg->images[1].height;
  int w = msg->images[1].width;

  cv::Mat disparity_orig_temp = cv::Mat::zeros(h,w,CV_16UC1); // h,w
  memcpy ( disparity_orig_temp.data , msg->images[1].data.data() , h*w*sizeof(short) ) ;    
  removeEdges(disparity_orig_temp);
  
  // form new image message
  bot_core::image_t disp_msg;
  disp_msg.utime = msg->utime;
  disp_msg.width = msg->images[1].width;
  disp_msg.height = msg->images[1].height;
  disp_msg.row_stride = 2*msg->images[1].width;
  disp_msg.pixelformat = msg->images[1].pixelformat;
  disp_msg.nmetadata = 0;
  disp_msg.data.resize( h*w*sizeof(short) );
  memcpy(&disp_msg.data[ 0 ], disparity_orig_temp.data,  h*w*sizeof(short)  );
  disp_msg.size = disp_msg.data.size() ;
  
  multisense::images_t msg_out = *msg;
  msg_out.images[1] = disp_msg;
  lcm_->publish("CAMERA_SGBM", &msg_out);
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
