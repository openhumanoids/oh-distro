#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <ConciseArgs>
#include <zlib.h>
#include <lcmtypes/multisense.hpp>

#include <multisense_image_utils/multisense_image_utils.hpp>

using namespace std;

struct CommandLineConfig
{
    float depthThresh;
    int sizeThresh;
    std::string input;
    std::string output;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_);

    ~App(){
    }
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CommandLineConfig cl_cfg_;
    int width_, height_;
    int local_img_buffer_size_;
    uint8_t* local_img_buffer_;

    void msHandler(const lcm::ReceiveBuffer* rbuf,
                           const std::string& channel, const  multisense::images_t* msg);
    void unzipImage(const bot_core::image_t *msg, cv::Mat& disparityMat);

    multisense_image_utils miu_;
};

App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_):
    lcm_(lcm_), cl_cfg_(cl_cfg_){
  lcm_->subscribe( cl_cfg_.input ,&App::msHandler,this);

  width_ = 1024;
  height_ = 1024;


  local_img_buffer_= new uint8_t[local_img_buffer_size_];  // x4 was used for zlib in kinect_lcm

}


// assumes a zipped gray scale image: CPP CPP CPP CPP
void App::unzipImage(const bot_core::image_t *msg, cv::Mat& disparityMat){

  int w = 1024;
  int h = 1024;

  std::vector<uint8_t> buf(w*h*2);
  unsigned long len = buf.size();
  uncompress(buf.data(), &len, msg->data.data(), msg->data.size());
  disparityMat = cv::Mat(h, w, CV_16UC1, buf.data()).clone();

  return;
}

void App::msHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg){
  std::cout << "a\n";

  float k00 = 591.909423828125; // focal length x;
  float baseline = 0.0700931567882511;
  float mDisparityFactor = 1/k00/baseline;
  uint16_t thresh = 16/mDisparityFactor/cl_cfg_.depthThresh;


  cv::Mat disparityMat;
  unzipImage(&msg->images[1], disparityMat);
  miu_.removeSmall(disparityMat, thresh, cl_cfg_.sizeThresh);


  int width =1024;
  int height = 1024;
  int utime =0;
  int n_colors =2;
  int isize= 1024* 1024;

  int uncompressed_size = isize*n_colors;
  unsigned long compressed_size = width* height* sizeof(int16_t) * 4;;

  std::vector<uint8_t> buf(width*height*4);
  compress2(buf.data() , &compressed_size, (const Bytef*) disparityMat.data, uncompressed_size,
            Z_BEST_SPEED);

  bot_core::image_t image;
  image.utime =utime;
  image.width =width;
  image.height=height;
  image.row_stride =n_colors*width; // this is useless if doing zip
  image.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;// This label will be invalid...
  image.size = (int) compressed_size;
  image.data = buf;
  image.nmetadata =0;
  //image.metadata = NULL;

  multisense::images_t out;
  out.utime =0;
  out.n_images = 2;
  out.image_types = {0,5};
  out.images.push_back( msg->images[0] );
  out.images.push_back( image );

  lcm_->publish( cl_cfg_.output, &out);
}


int main(int argc, char ** argv) {
  CommandLineConfig cl_cfg;
  cl_cfg.depthThresh = 10.0f; //m
  cl_cfg.sizeThresh = 100; // pixels
  cl_cfg.input = "CAMERA";
  cl_cfg.output = "CAMERA_OUT";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg.depthThresh, "d", "depthThresh","depth in m from camera");
  opt.add(cl_cfg.sizeThresh, "s", "sizeThresh","pixel size of cluster to remove");
  opt.add(cl_cfg.input, "i", "input","Camera input channel");
  opt.add(cl_cfg.output, "o", "output","Camera output channel");
  opt.parse();

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  App app(lcm, cl_cfg);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
