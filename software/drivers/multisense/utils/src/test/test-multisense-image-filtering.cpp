#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <ConciseArgs>
#include <zlib.h>
#include <lcmtypes/multisense.hpp>

#include <multisense_image_utils/multisense_image_utils.hpp>

#include <chrono>

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

    void msHandler(const lcm::ReceiveBuffer* rbuf,
                           const std::string& channel, const  multisense::images_t* msg);

    multisense_image_utils miu_;
};

App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_):
    lcm_(lcm_), cl_cfg_(cl_cfg_){
  lcm_->subscribe( cl_cfg_.input ,&App::msHandler,this);
}

void App::msHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg){
  int n_bytes =2;
  int width =msg->images[0].width;
  int height = msg->images[0].height;

  // 1. Uncompress Depth Image:
  std::vector<uint8_t> buf(width*height*n_bytes);
  unsigned long len = buf.size();
  uncompress(buf.data(), &len, msg->images[1].data.data(), msg->images[1].data.size());
  cv::Mat disparityMat = cv::Mat( height, width, CV_16UC1, buf.data()).clone();

  // 2. Remove small components (parameters used here only for easy mapping)
  float k00 = 591.909423828125; // focal length x;
  float baseline = 0.0700931567882511;
  float mDisparityFactor = 1/k00/baseline;
  auto t1 = std::chrono::high_resolution_clock::now();
  //uint16_t thresh = 16/mDisparityFactor/cl_cfg_.depthThresh;
  //miu_.removeSmall(disparityMat, thresh, cl_cfg_.sizeThresh);
  double thresh =
    std::abs(16*k00*baseline*(1.0/2 - 1.0/(2+cl_cfg_.depthThresh)));
  miu_.removeSpeckles(disparityMat, thresh, cl_cfg_.sizeThresh);
  auto t2 = std::chrono::high_resolution_clock::now();
  auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1);
  std::cout << "processed frame in " << dt.count() << " ms" << std::endl;

  // 3. Recompress:
  int uncompressed_size = width*height*n_bytes;
  unsigned long compressed_size = width* height* sizeof(int16_t) * 4;;
  std::vector<uint8_t> buf2(width*height*4);
  compress2(buf2.data() , &compressed_size, (const Bytef*) disparityMat.data, uncompressed_size,
            Z_BEST_SPEED);

  // 4. Publish
  multisense::image_t image;
  image.utime =msg->utime;
  image.width =width;
  image.height=height;
  image.row_stride =n_bytes*width; // this is useless if doing zip
  image.pixelformat = multisense::image_t::PIXEL_FORMAT_RGB;// This label will be invalid...
  image.size = (int) compressed_size;
  image.data = buf2;
  image.nmetadata =0;
  //image.metadata = NULL;

  multisense::images_t out;
  out.utime =msg->utime;
  out.n_images = 2;
  out.image_types = msg->image_types;
  out.images.push_back( msg->images[0] );
  out.images.push_back( image );

  lcm_->publish( cl_cfg_.output, &out);
}


int main(int argc, char ** argv) {
  CommandLineConfig cl_cfg;
  cl_cfg.depthThresh = 100.0f; //m
  cl_cfg.sizeThresh = 4000; // pixels
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
