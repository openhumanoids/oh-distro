#ifndef STEREO_B_
#define STEREO_B_

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <bot_param/param_client.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

class StereoB
{
  public:
    typedef boost::shared_ptr<StereoB> Ptr;
    typedef boost::shared_ptr<const StereoB> ConstPtr;
        
    StereoB (boost::shared_ptr<lcm::LCM> &lcm_, std::string lcm_channel = "CAMERA");

    ~StereoB() {
    }

    void doStereoB(cv::Mat& left_stereo, cv::Mat& right_stereo);
    void sendRangeImage(int64_t utime);

    uint8_t* getDisparity(){
      return _disp.data;
    };
    uint8_t* getColor(){ // assumed to be grey
      return _left.data; 
    }
    
    cv::Mat getDisparityMat(){
      return _disp;
    };
    
    void setDisparityMat(cv::Mat &disp_in ){
      _disp = disp_in;
    };

    void setScale(float scale_in){
      vSCALE = scale_in;
    }
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    BotParam* botparam_;

    float vSCALE;
    cv::Mat _disp;
    cv::Mat _left;
};




#endif
