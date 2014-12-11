#ifndef TLDTracker_H_
#define TLDTracker_H_

// LCM includes
#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include <unistd.h>
#include <iomanip>
#include <glib.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "tld/TLD.h"

using namespace cv;


struct CameraParams { 
    int width, height;
    float fx, fy, cx, cy, k1, k2, k3, p1, p2;
    CameraParams () {}
    CameraParams (BotParam* param, const std::string key_prefix_str) { 
        width = bot_param_get_int_or_fail(param, (key_prefix_str+".width").c_str());
        height = bot_param_get_int_or_fail(param,(key_prefix_str+".height").c_str());
  
        double vals[5];
        bot_param_get_double_array_or_fail(param, (key_prefix_str+".pinhole").c_str(), vals, 5);
        fx = vals[0];
        fy = vals[1];
        cx = vals[3];
        cy = vals[4];
        if (3 == bot_param_get_double_array(param, (key_prefix_str+".distortion_k").c_str(), vals, 3)) {
            k1 = vals[0];
            k2 = vals[1];
            k3 = vals[2];
        }
        if (2 == bot_param_get_double_array(param, (key_prefix_str+".distortion_p").c_str(), vals, 2)) {
	        p1 = vals[0];
	        p2 = vals[1];
        }
    }
};



class TLDTracker { 
 public: 
    cv::Mat object_roi;

    // Image Size
    cv::Size img_size; 

    // Tracker scale 
    float scale_factor;

    // Gaussian blur
    bool apply_gaussian_filter;

    // Main TLD tracker
    tld::TLD* tld;

    // Current estimate from Detector
    cv::Rect currBB;

    // Predicted estimate from Median Flow Tracker
    cv::Rect predBB;

    // Prediction window of object
    cv::Rect pred_win;

    // Current confidence
    float confidence; 

    // Detection valid
    bool detection_valid;

    // has selection been initialized:
    bool initialized;

private: 
    cv::Rect scaleUp(cv::Rect& rect_in);
    cv::Rect scaleDown(cv::Rect& rect_in);

 public: 
    TLDTracker(int _width, int _height, float _scale);
    ~TLDTracker();
    void internal_init(); 
    bool initialize(cv::Mat& img, const cv::Mat& mask);
    bool initialize(cv::Mat& img, cv::Rect selection);
    void preprocess_image(cv::Mat& img, cv::Mat& _img);
    // bool getMaskInitialized(){ return mask_initialized_; };
    
    std::vector<float> update(cv::Mat& img, std::vector< Eigen::Vector3d > & pts,
        Eigen::Isometry3d local_to_camera); 
    bool computeMaskROI(const cv::Mat& img, const cv::Mat& mask, cv::Rect& roi);
    void showTLDInfo(cv::Mat& img);
    void addOverlay(cv::Mat& img, int idx, cv::Scalar color);
};

#endif /* TLDTracker_H_ */
