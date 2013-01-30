#ifndef registeration_HPP_
#define registeration_HPP_

#include <lcm/lcm-cpp.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <bot_lcmgl_client/lcmgl.h>

#include <pointcloud_tools/pointcloud_lcm.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp>


struct ImageFeature{
  int track_id;
  Eigen::Vector2d uv; ///< unrectified, distorted, orig. coords
  Eigen::Vector2d base_uv; ///< unrectified, distorted, base level
  Eigen::Vector3d uvd; ///< rectified, undistorted, base level
  Eigen::Vector3d xyz;
  Eigen::Vector4d xyzw;
  uint8_t color[3];

  // @todo what more is needed?
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

enum{SUCCESS, INSUFFICIENT_INLIERS};

struct FrameMatch{
  std::vector<int> featuresA_indices;
  std::vector<int> featuresB_indices;

  std::vector<ImageFeature> featuresA;
  std::vector<ImageFeature> featuresB;
  
  int estimation_status; // 0 = sucess, 0 = too few matches, 1 = too few inliers
  Eigen::Isometry3d delta; // A->B transform : where is B relative to A

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef boost::shared_ptr<FrameMatch> FrameMatchPtr;


class Reg
{
  public:
    typedef boost::shared_ptr<Reg> Ptr;
    typedef boost::shared_ptr<const Reg> ConstPtr;
    
    Reg (boost::shared_ptr<lcm::LCM> &lcm_);
    
    void align_images(cv::Mat &img0, cv::Mat &img1, 
                           std::vector<ImageFeature> &features0, std::vector<ImageFeature> &features1,
                           int64_t utime0, int64_t utime1);
    
    
    void draw_both_reg(std::vector<ImageFeature> features0,    std::vector<ImageFeature> features1,
                           Eigen::Isometry3d pose0,   Eigen::Isometry3d pose1);
    void draw_reg(std::vector<ImageFeature> features,    int status, Eigen::Isometry3d pose);
    
    void send_both_reg(std::vector<ImageFeature> features0,    std::vector<ImageFeature> features1,
        Eigen::Isometry3d pose0,   Eigen::Isometry3d pose1,
        int64_t utime0, int64_t utime1           );
    
    void send_both_reg_inliers(std::vector<ImageFeature> features0,    std::vector<ImageFeature> features1,
        Eigen::Isometry3d pose0,   Eigen::Isometry3d pose1,
        std::vector<int> feature_inliers0,    std::vector<int> feature_inliers1 ,
        int64_t utime0, int64_t utime1);
    
    void send_lcm_image(cv::Mat &img, std::string channel );

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bot_lcmgl_t* lcmgl_;

    pointcloud_vis* pc_vis_;
    
};

#endif
