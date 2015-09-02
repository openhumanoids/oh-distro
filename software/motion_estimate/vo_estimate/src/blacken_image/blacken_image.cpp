#include <stdio.h>
#include <inttypes.h>
#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core.hpp"
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <ConciseArgs>
using namespace cv;
using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string input_channel_, std::string output_channel_, int row_remove_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool verbose_;
    int row_remove_;
    std::string input_channel_;
    std::string output_channel_;
    image_io_utils*  imgutils_;
    uint8_t* img_buf_; 
    uint8_t* rgb_compress_buffer_;
    
    void imagesHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg);
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string input_channel_, std::string output_channel_, int row_remove_):
    lcm_(lcm_), verbose_(verbose_), 
    input_channel_(input_channel_), output_channel_(output_channel_),  row_remove_(row_remove_){
  lcm_->subscribe( input_channel_ ,&Pass::imagesHandler,this);

  // left these numbers very large:
  img_buf_= (uint8_t*) malloc(3* 1524  * 1544);
  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), 
                                  1524, 
                                  3*1544 );  

  rgb_compress_buffer_= (uint8_t*) malloc(3* 1524  * 1544);

  cout << "Finished setting up\n";
}


void Pass::imagesHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg){
  std::cout << "got " << channel << "\n";

  int w = msg->images[0].width;
  int h = msg->images[0].height;
  int n_colors =3;
  
  imgutils_->decodeImageToRGB( & msg->images[0], img_buf_);
  cv::Mat img(cv::Size( w, h),CV_8UC3);
  img.data = img_buf_;
  std::cout << row_remove_ << "\n";
  rectangle( img, Point( 0, row_remove_ ) , Point( 1024, 1024 ), Scalar( 0, 0, 0 ), -1, 4 );
  int compressed_size =  w*h*n_colors;//image_buf_size;
  int compression_status = jpeg_compress_8u_rgb  (img.data, w, h, w*n_colors,
                                                     rgb_compress_buffer_, &compressed_size, 75);

  bot_core::image_t msgout_small;
  msgout_small.utime = msg->utime;
  msgout_small.width = w;
  msgout_small.height = h;
  msgout_small.row_stride = n_colors*w;
  msgout_small.size = compressed_size;
  msgout_small.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG;
  msgout_small.data.resize(compressed_size);
  memcpy(msgout_small.data.data(), rgb_compress_buffer_, compressed_size);
  msgout_small.nmetadata =0;
  // lcm_->publish("MMM", &msgout_small);

  bot_core::images_t msgo;
  msgo.utime = msg->utime;
  msgo.n_images = 2;
  msgo.images.resize(2);
  msgo.image_types.resize(2);
  msgo.image_types[0] = bot_core::images_t::LEFT;
  msgo.image_types[1] = bot_core::images_t::DISPARITY_ZIPPED;
  msgo.images[0] = msgout_small;
  msgo.images[1] = msg->images[1];
  lcm_->publish( output_channel_ , &msgo);
}


int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "blacken-image");
  bool verbose=false;
  int row_remove = 725;
  string input_channel="CAMERA";
  string output_channel="CAMERA_BLACKENED";
  parser.add(verbose, "v", "verbose", "Verbosity");
  parser.add(input_channel, "l", "input_channel", "Incoming channel");
  parser.add(output_channel, "o", "output_channel", "Output channel");
  parser.add(row_remove, "r", "row_remove", "Row to remove [1024 is none]");
  parser.parse();
  cout << verbose << " is verbose\n";
  cout << input_channel << " is input_channel\n";
  cout << output_channel << " is output_channel\n";
  cout << row_remove << " is row_remove\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,verbose,input_channel, output_channel, row_remove);
  cout << "Ready to convert from imu to pose" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
