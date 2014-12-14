#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <opencv2/opencv.hpp>
#include <trackers/histogram-tracker.hpp>

class ObjectSegmenter { 
public:
    bool initialized; 

private: 
    // Color info
    HistogramInfo hue_info, val_info, sat_info; 
    cv::EM gmm_model;

    cv::Rect p_detect;

    cv::Mat_<float> hist_bp;
public: 
    ObjectSegmenter(); 
    ~ObjectSegmenter();

    int find_largest_contour(const vector<vector<Point> >& contours, vector<Vec4i>& hierarchy);
    cv::Mat1b find_largest_contour_mask(cv::Mat1b& mask);

    void init_hsv_histogram();
    void initialize_histogram(const cv::Mat& hsv, std::vector<cv::Point>& pts);
    void initialize_GMM(const cv::Mat& img, std::vector<cv::Point>& pts);
    void initialize(const cv::Mat& img, std::vector<cv::Point>& pts);
    void belief_hist(const cv::Mat& hsv, cv::Mat_<float>& bp);
    void predictGMM(cv::Mat& img, cv::Mat& bp);
    cv::Mat1b segment(const cv::Mat& img);
    cv::Mat1b track(const cv::Mat& img);

    enum pr_type { SEGMENT, TRACK };
    cv::Mat1b process(const cv::Mat& img, pr_type type);



};
