// Tool to take a multisense stereo image (images_t)
// - use disparity image and left and create a point cloud

#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ConciseArgs>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "lcmtypes/bot_core.hpp"
#include <multisense_utils/multisense_utils.hpp> // create point clouds

using namespace cv;
using namespace std;

class image_tool{
  public:
    image_tool(boost::shared_ptr<lcm::LCM> &lcm_, int decimate_);
    ~image_tool(){}
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    
    void disparityHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg); 
    mutable pcl::PCLPointCloud2 cloud_out;
    
    cv::Mat_<double> Q_;
    int decimate_;
    multisense_utils* ms_utils_;
};    

image_tool::image_tool(boost::shared_ptr<lcm::LCM> &lcm_,
                       int decimate_):
      lcm_(lcm_), decimate_(decimate_), Q_(4,4,0.0){
        
  lcm_->subscribe( "CAMERA",&image_tool::disparityHandler,this);

  Q_(0,0) = Q_(1,1) = 1.0;  
  Q_(3,2) = 14.26672796348671; //1.0 / baseline;
  Q_(0,3) = -512; //-stereo_params_.right.cx;
  Q_(1,3) = -512;//-stereo_params_.right.cy;
  Q_(2,3) = 591.909423828125;// stereo_params_.right.fx;
  Q_(3,3) = 0;//(stereo_params_.right.cx - stereo_params_.left.cx ) / baseline;  
  
  std::cout << Q_ << " is reprojectionMatrix\n";  
  
  ms_utils_ = new multisense_utils();
  ms_utils_->set_decimate( decimate_ );
}

void image_tool::disparityHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg){
  cout << msg->utime << " | "<< msg->images[0].width <<" x "<< msg->images[0].height <<"\n";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  ms_utils_->unpack_multisense(msg,Q_,cloud);  
  
  pcl::PCDWriter writer;
  std::stringstream pcd_fname;
  pcd_fname << "/tmp/multisense.pcd";
  std::cout << pcd_fname.str() << " written\n";
  writer.write (pcd_fname.str() , *cloud, false);  
}

int main(int argc, char ** argv) {
  ConciseArgs parser(argc, argv, "registeration-app");
  int decimate = 8;
  parser.add(decimate, "d", "decimate", "Decimation of data");
  parser.parse();
  cout << decimate << " is decimate\n"; 

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  image_tool app(lcm,decimate);
  cout << "Ready image tool" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
