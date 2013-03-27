#ifndef HISTTRACKER_H_
#define HISTTRACKER_H_

// LCM includes
#include <lcm/lcm.h>
#include <bot_core/bot_core.h>

#include <unistd.h>
#include <iomanip>
#include <glib.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <Eigen/StdVector>

using namespace cv;

struct HistogramInfo { 
    int size, bin_dev;
    int binW;

    float ranges[2];
    const float* pranges;

    cv::Mat histogram;
    cv::Mat unimodal_histogram;
    cv::Mat histogram_img; 
    cv::Mat unimodal_histogram_img;

    cv::Mat buf;

    void init () { 
        // Images
        histogram_img = Mat::zeros(200, 320, CV_8UC3), 
            unimodal_histogram_img = Mat::zeros(200, 320, CV_8UC3);

        // setup Hist dims
        binW = histogram_img.cols / size;
        buf = cv::Mat(1, size, CV_8UC3);
        for( int i = 0; i < size; i++ )
            buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./size), 255, 255);
        cvtColor(buf, buf, CV_HSV2BGR);
    }

    void computeUnimodalHistogram() { 
        // Find top hist bin and only set new hist with smaller variance
        cv::Point max_idx;
        minMaxLoc(histogram, 0, 0, 0, &max_idx);

        // Clone 
        unimodal_histogram = histogram.clone();

        // Prune other colors away from max_idx
        for( int i = 0; i < size; i++ )
            if (abs(max_idx.y-i)>bin_dev)
                unimodal_histogram.at<float>(i) = 0.f ;            
        return;
    }

    void createHistogramImage() { 
        // Draw Hist
        for( int i = 0; i < size; i++ ) {
            int val = saturate_cast<int>(histogram.at<float>(i)*histogram_img.rows);
            cv::rectangle( histogram_img, Point(i*binW,histogram_img.rows),
                       Point((i+1)*binW,histogram_img.rows - val), 
                       Scalar(buf.at<Vec3b>(i)), -1, 8 );
        }
        //Draw Good Hist
        for( int i = 0; i < size; i++ ) {
            int val = saturate_cast<int>(unimodal_histogram.at<float>(i)*unimodal_histogram_img.rows);
            cv::rectangle( unimodal_histogram_img, Point(i*binW,unimodal_histogram_img.rows),
                       Point((i+1)*binW,unimodal_histogram_img.rows - val),
                       Scalar(buf.at<Vec3b>(i)), -1, 8 );
        }
    }

    void backProjectUnimodal(cv::Mat& img, cv::Mat& out) { 
        calcBackProject(&img, 1, 0, unimodal_histogram, out, &pranges);
    }

    void backProject(cv::Mat& img, cv::Mat& out) { 
        calcBackProject(&img, 1, 0, histogram, out, &pranges);
    }

};

class HistTracker { 
 private: 
  HistogramInfo hue_info, val_info, sat_info;
    cv::Mat object_roi;
    cv::Mat backproj, belief;

    // Prediction window of object
    cv::Rect pred_win;

    enum TrackingMode { DEFAULT };
    TrackingMode vTRACKING_MODE; 
    
    // has mask been initialized:
    bool mask_initialized_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    

 public: 
    pthread_mutex_t mutex;
 public: 
    HistTracker(); 
    HistTracker(int mode);
    ~HistTracker();
    void internal_init(); 
    bool initialize(const cv::Mat& img, const cv::Mat& mask);
    bool getMaskInitialized(){ return mask_initialized_; };
    
    void update_prediction(const cv::Mat& img);
    std::vector<float> update(cv::Mat& img, float scale, std::vector< Eigen::Vector3d > & pts,
        Eigen::Isometry3d local_to_camera); 
    cv::Mat get_belief(); 
    bool computeMaskROI(const cv::Mat& img, const cv::Mat& mask, cv::Rect& roi);
    void showHistogramInfo(cv::Mat& img);
    void plot_histogram(char* win); 
};

#endif /* HISTTRACKER_H_ */
