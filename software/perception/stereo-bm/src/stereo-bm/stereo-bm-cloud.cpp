#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pointcloud_tools/pointcloud_lcm.hpp> // create point clouds
#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

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

    pointcloud_lcm* pc_lcm_;      
    image_io_utils*  imgutils_;

    cv::Mat_<double> Q_;
    
    int width_;
    int height_;
    uint8_t* left_buf_;
    uint8_t* right_buf_;
    uint8_t* images_buf_;    
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_, float scale):
    lcm_(lcm_), image_channel_(image_channel_), Q_(4,4,0.0){

  lcm_->subscribe( image_channel_ ,&Pass::imageHandler,this);
  stereob_ = new StereoB(lcm_);
  stereob_->setScale(scale);
  
  height_=800;
  width_=800;
  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), width_, 2*height_); // extra space for stereo tasks


  if (1==0){
    /////// From original simulation
    Q_(0,0) = Q_(1,1) = 1.0;  
    //double Tx = baseline();
    Q_(3,2) = 14.2914745276283;//1.0 / Tx;
    Q_(0,3) = -512.5;//-right_.cx();
    Q_(1,3) = -272.5;//-right_.cy();
    Q_(2,3) = 610.1778;//right_.fx();
    Q_(3,3) = 0;// (512.0 - 272.0)/0.07;//(right_.cx() - left_.cx()) / Tx; 
  }else{
    // 800x800 simulation
    Q_(0,0) = Q_(1,1) = 1.0;  
    //double Tx = baseline();
    Q_(3,2) = 14.2914745276283;//1.0 / Tx;
    Q_(0,3) = -400.5;//-right_.cx();
    Q_(1,3) = -400.5;//-right_.cy();
    Q_(2,3) = 476.7014;//right_.fx();
    Q_(3,3) = 0;// (512.0 - 272.0)/0.07;//(right_.cx() - left_.cx()) / Tx; 
    
  }

  int decimate_ =2;
  pc_lcm_ = new pointcloud_lcm( lcm_->getUnderlyingLCM() );
  pc_lcm_->set_decimate( decimate_ );  
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
  
//  memcpy(left_img.data,  msg->data.data() , msg->size/2);
//  memcpy(right_img.data,  msg->data.data() + msg->size/2 , msg->size/2);
  stereob_->doStereoB(left_img, right_img);
  stereob_->sendRangeImage(msg->utime);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pc_lcm_->unpack_multisense(stereob_->getDisparity(), stereob_->getColor(), h, w, Q_, cloud);

  cout << "points: " << cloud->points.size() << "\n";
  pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
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
