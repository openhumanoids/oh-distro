#ifndef COLOR_THRESHOLD_
#define COLOR_THRESHOLD_

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

class ColorThreshold
{
  public:
    typedef boost::shared_ptr<ColorThreshold> Ptr;
    typedef boost::shared_ptr<const ColorThreshold> ConstPtr;
        
    ColorThreshold (boost::shared_ptr<lcm::LCM> &lcm_, 
                               int width_, int height_, 
                               double fx_, double fy_, double cx_, double cy_);

    ~ColorThreshold() {
    }

    std::vector<float> colorThreshold(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts, uint8_t* img_data,
                                        Eigen::Isometry3d local_to_camera, int64_t current_utime);
    
    IplImage* GetThresholdedImage(IplImage* img);
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    pointcloud_vis* pc_vis_;
    int mode_;

    int width_;
    int height_;
    double fx_, fy_, cx_, cy_;    

    image_io_utils*  imgutils_;  
};




#endif
