#include "HistTracker.hpp"

HistTracker::HistTracker() { 
    vTRACKING_MODE = DEFAULT; 
    internal_init(); 
}

HistTracker::HistTracker(int mode) { 
    vTRACKING_MODE = DEFAULT; 
    internal_init(); 
}

HistTracker::~HistTracker() {
}

void
HistTracker::internal_init() { 

    // Hue bins, params etc. 
    hue_info.size = 40, val_info.size = 40;
    hue_info.bin_dev = 5, val_info.bin_dev = 5;

    hue_info.ranges[0] = 0, hue_info.ranges[1] = 180;
    val_info.ranges[0] = 0, val_info.ranges[1] = 255;

    hue_info.pranges = hue_info.ranges;
    val_info.pranges = val_info.ranges;

    hue_info.init();
    val_info.init();
    return;
}

bool 
HistTracker::computeMaskROI(const cv::Mat& img, const cv::Mat& mask) { 
    std::vector<std::vector<cv::Point> > contours;
    findContours(mask, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    int minx = img.cols, miny = img.rows; 
    int maxx = -1, maxy = -1;
    for (int j=0; j<contours.size(); j++) { 
        for (int k=0; k<contours[j].size(); k++) { 
            int x = contours[j][k].x; 
            int y = contours[j][k].y; 
            if (x < minx) minx = x; 
            if (y < miny) miny = y; 
            if (x > maxx) maxx = x; 
            if (y > maxy) maxy = y; 
        }
    }
    int rx = minx - 5, ry = miny - 5; 
    int rw = maxx-minx + 10, rh = maxy-miny + 10;
    if (!(rx >=0 && rx + rw < img.cols && 
          ry >=0 && ry + rh < img.rows) || rh <= 0 || rw <= 0)
        return false;

    cv::Mat roi = cv::Mat(img, cv::Rect(rx, ry, rw, rh));
    roi.copyTo(object_roi);
    return true;
}

bool
HistTracker::initialize(const cv::Mat& img, const cv::Mat& mask) {

    // Compute mask roi for debug
    if (!computeMaskROI(img, mask)) 
        return false;

    // Convert to HSV space
    cv::Mat hsv; 
    cvtColor(img, hsv, CV_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    assert(channels.size() == 3);
    cv::Mat hue = channels[0]; 
    cv::Mat val = channels[2];
    // std::cerr << "hue: " << hue << std::endl;

    // Calculate Histogram
    calcHist(&hue, 1, 0, mask, hue_info.histogram, 1, &hue_info.size, &hue_info.pranges); 
    calcHist(&val, 1, 0, mask, val_info.histogram, 1, &val_info.size, &val_info.pranges); 

    // std::cerr << "hue hist: " << hue_info.histogram << " " << std::endl;
    // std::cerr << "val hist: " << val_info.histogram << " " << std::endl;

    normalize(hue_info.histogram, hue_info.histogram, 0, 1, CV_MINMAX);
    normalize(val_info.histogram, val_info.histogram, 0, 1, CV_MINMAX);

    // Compute unimodal histogram
    hue_info.computeUnimodalHistogram();
    val_info.computeUnimodalHistogram();

    // Create debug histograms
    hue_info.createHistogramImage();
    val_info.createHistogramImage();

    // cv::Mat display = img.clone();
    // showHistogramInfo(display);

    // cv::imshow("Initialize Histogram Tracker", display);
    return true;
}

void
HistTracker::update_prediction(const cv::Mat& img) { 
    

    return;
}

bool
HistTracker::update(cv::Mat& img, float scale) { 
    if (hue_info.histogram.empty() || val_info.histogram.empty()) return false;
 
    // Downsample image
    cv::Mat _img; 
    if (scale == 1.f)
        _img = img;
    else
        cv::resize(img, _img, cv::Size(), scale, scale, cv::INTER_LINEAR);

    // Convert to HSV space
    cv::Mat hsv; 
    cvtColor(_img, hsv, CV_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    assert(channels.size() == 3);
    cv::Mat hue = channels[0]; 
    cv::Mat val = channels[2];
    // std::cerr << "hue: " << hue << std::endl;

    // Calculate likelihood
    cv::Mat hue_bp, val_bp;
    hue_info.backProjectUnimodal(hue, hue_bp);
    val_info.backProjectUnimodal(val, val_bp);

    cv::Mat _bp;
    multiply(hue_bp, val_bp, _bp);

    // Upsample image
    cv::Mat bp;
    if (scale == 1.f)
        bp = _bp;
    else
        cv::resize(_bp, bp, cv::Size(), 1.f/scale, 1.f/scale, cv::INTER_LINEAR);

    // cv::imshow( "Hue Belief", hue_bp * 255); 
    // cv::imshow( "Val Belief", val_bp * 255); 
    showHistogramInfo(img);

    cv::Mat bp3;
    cv::cvtColor(bp * 200, bp3, CV_GRAY2BGR);
    addWeighted(img, 0.5, bp3, 0.5, 0, img); 

    return true;
}

cv::Mat 
HistTracker::get_belief() { 
    return belief; 
}

void 
HistTracker::showHistogramInfo(cv::Mat& img) { 
    float scale = 0.2;
    int w = img.cols, h = img.rows;
    int sw = scale * w, sh = scale * h; 
    
    int off = 5;
    float ow = object_roi.cols, oh = object_roi.rows;
    cv::Mat obj_roi = cv::Mat(img, cv::Rect(w-2*sw-3*off-ow,off,ow,oh));
    cv::Mat hue_roi = cv::Mat(img, cv::Rect(w-2*sw-2*off,off,sw,sh));
    cv::Mat val_roi = cv::Mat(img, cv::Rect(w-sw-off,off,sw,sh));

    if (!hue_info.histogram.empty()) { 
        cv::Mat pip_hist, pip_val, pip_obj;
        cv::resize(hue_info.unimodal_histogram_img, pip_hist, hue_roi.size());
        cv::resize(val_info.unimodal_histogram_img, pip_val, val_roi.size());

        addWeighted(hue_roi, 0.5, pip_hist, 0.5, 0, hue_roi);
        addWeighted(val_roi, 0.5, pip_val, 0.5, 0, val_roi);
        addWeighted(obj_roi, 0.5, object_roi, 0.5, 0, obj_roi);
    }
    return;
}

void HistTracker::plot_histogram(char* win) { 
    // cv::imshow(win, hist_img); 
    return;
} 
