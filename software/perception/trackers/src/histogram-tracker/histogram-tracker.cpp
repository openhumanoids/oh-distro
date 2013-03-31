#include "histogram-tracker.hpp"

HistogramTracker::HistogramTracker() { 
    vTRACKING_MODE = DEFAULT; 
    internal_init(); 
  mask_initialized_=false;
  
  
          fx_ = 610.1778;
          fy_ = 610.1778;
          cx_ = 512.5;
          cy_ = 272.5;
  
}

HistogramTracker::HistogramTracker(int mode) { 
    vTRACKING_MODE = DEFAULT; 
    internal_init(); 
  mask_initialized_=false;
}

HistogramTracker::~HistogramTracker() {
}

void
HistogramTracker::internal_init() { 

    // Hue bins, params etc. 
  hue_info.size = 40, val_info.size = 40, sat_info.size = 40;
  hue_info.bin_dev = 5, val_info.bin_dev = 5, sat_info.bin_dev = 5;

  hue_info.ranges[0] = 0, hue_info.ranges[1] = 180;
  val_info.ranges[0] = 10, val_info.ranges[1] = 255;
  sat_info.ranges[0] = 10, sat_info.ranges[1] = 255;

  hue_info.pranges = hue_info.ranges;
  val_info.pranges = val_info.ranges;
  sat_info.pranges = sat_info.ranges;

  hue_info.init();
  val_info.init();
  sat_info.init();

  // prediction window
  pred_win = cv::Rect(0,0,0,0);

  return;
}

bool 
HistogramTracker::computeMaskROI(const cv::Mat& img, const cv::Mat& mask, cv::Rect& roi) { 
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
    
    // Populate ROI for initializing prediction window for camshift
    roi = cv::Rect(rx, ry, rw, rh);

    cv::Mat roi_im = cv::Mat(img, roi);
    roi_im.copyTo(object_roi);
    return true;
}

bool
HistogramTracker::initialize(const cv::Mat& img, const cv::Mat& mask) {
  
  hue_info = HistogramInfo();
  val_info = HistogramInfo();
  sat_info = HistogramInfo();
  object_roi = cv::Mat();
  internal_init();

    // Compute mask roi for debug
    if (!computeMaskROI(img, mask, pred_win)) 
        return false;

    // Convert to HSV space
    cv::Mat hsv; 
    cvtColor(img, hsv, CV_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    assert(channels.size() == 3);
    cv::Mat hue = channels[0]; 
    cv::Mat val = channels[1];
    cv::Mat sat = channels[2];
    // std::cerr << "hue: " << hue << std::endl;

    // Calculate Histogram
    calcHist(&hue, 1, 0, mask, hue_info.histogram, 1, &hue_info.size, &hue_info.pranges); 
    calcHist(&val, 1, 0, mask, val_info.histogram, 1, &val_info.size, &val_info.pranges); 
    calcHist(&sat, 1, 0, mask, sat_info.histogram, 1, &sat_info.size, &sat_info.pranges); 

    // std::cerr << "hue hist: " << hue_info.histogram << " " << std::endl;
    // std::cerr << "val hist: " << val_info.histogram << " " << std::endl;

    normalize(hue_info.histogram, hue_info.histogram, 0, 1, CV_MINMAX);
    normalize(val_info.histogram, val_info.histogram, 0, 1, CV_MINMAX);
    normalize(sat_info.histogram, sat_info.histogram, 0, 1, CV_MINMAX);

    // Compute unimodal histogram
    hue_info.computeUnimodalHistogram();
    val_info.computeUnimodalHistogram();
    sat_info.computeUnimodalHistogram();

    // Create debug histograms
    hue_info.createHistogramImage();
    val_info.createHistogramImage();
    sat_info.createHistogramImage();

    // cv::Mat display = img.clone();
    // showHistogramInfo(display);

    // cv::imshow("Initialize Histogram Tracker", display);
    mask_initialized_ = true;

    // Once mask is inialized, find the 
    std::cerr << "Initialized: Prediction window " << pred_win.tl() << "->" << pred_win.br() << std::endl;

    return true;
}

void
HistogramTracker::update_prediction(const cv::Mat& img) { 
    return;
}

std::vector<float>
HistogramTracker::update(cv::Mat& img, float scale, std::vector< Eigen::Vector3d > & pts,
        Eigen::Isometry3d local_to_camera) { 
    // Assign uniform likelihoods:
    std::vector<float> loglikelihoods;
    loglikelihoods.assign ( pts.size() ,0);    
  
    if (hue_info.histogram.empty() || val_info.histogram.empty()) return loglikelihoods;
 
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
    cv::Mat_<float> hue = channels[0]; 
    cv::Mat_<float> val = channels[1];
    cv::Mat_<float> sat = channels[2];
    // std::cerr << "hue: " << hue << std::endl;

    // Calculate likelihood
    cv::Mat hue_bp, val_bp, sat_bp;
    hue_info.backProject(hue, hue_bp);
    val_info.backProject(val, val_bp);
    sat_info.backProject(sat, sat_bp);
    //hue_info.backProjectUnimodal(hue, hue_bp);
    //val_info.backProjectUnimodal(val, val_bp);


    cv::Mat_<float> _bp;
    multiply(hue_bp, val_bp, _bp);
    multiply(sat_bp, _bp, _bp);
    normalize(_bp, _bp, 0, 1, CV_MINMAX);
    
    // for (int y=0; y<_bp.rows; y++)  
    //   for (int x=0; x<_bp.cols; x++)
    // 	std::cerr << _bp(y,x) << std::endl
          
    // // Apply bell curved mapping instead of linear scale
    // float sigma2 = 0.5;
    // cv::Mat_<float> bp_xmu = (1.f - _bp);
    // multiply(bp_xmu, bp_xmu, bp_xmu);
    // cv::exp(-bp_xmu / (sigma2), _bp);

    cv::Mat_<float> _bp8 = _bp * 255;

    cv::Mat bp8;
    _bp8.convertTo(bp8, CV_8UC1);

    
    // Upsample image
    cv::Mat bp;
    if (scale == 1.f)
        bp = bp8;
    else
        cv::resize(bp8, bp, cv::Size(), 1.f/scale, 1.f/scale, cv::INTER_LINEAR);

    // 1. Project particles into camera frame:
    Eigen::Affine3d transform;
    transform.setIdentity();
    Eigen::Translation3d translation(local_to_camera.translation());
    Eigen::Quaterniond quat(local_to_camera.rotation());
    transform = transform * translation * quat;
    for (size_t i = 0; i < pts.size (); ++i){
      pts[i] = transform*pts[i];
    }    

    // 2. Estimate mean position and bounds of object from PF
    if (pts.size()) { 
        cv::Point2f est_obj_pos(0.f, 0.f); 
        for (int j=0; j<pts.size(); j++) { 
            int u = floor( ((pts[j][0] * fx_)/pts[j][2]) + cx_);
            int v = floor( ((pts[j][1] * fy_)/pts[j][2]) + cy_);
            est_obj_pos.x += u, est_obj_pos.y += v;
        }
        pred_win.x = est_obj_pos.x * 1.f / pts.size(), 
            pred_win.y = est_obj_pos.y * 1.f / pts.size();
        //std::cerr << "Prior rect from PF : ";
    } else {
        //std::cerr << "Prior rect from Prev Estimate : ";
    }
    //std::cerr << pred_win.tl() << "->" << pred_win.br() << std::endl;            

    // Visualization:
    cv::Mat bp3;
    cv::cvtColor(bp8, bp3, CV_GRAY2BGR);
    addWeighted(img, 0.4, bp3, 0.6, 0, img); 
    // cv::imshow( "Hue Belief", hue_bp); 
    // cv::imshow( "Val Belief", val_bp); 
    // cv::imshow( "Sat Belief", val_bp); 

    // Done with using the input image
    if (pred_win.x < 0 || pred_win.x > _bp.cols || pred_win.y < 0 || pred_win.y > _bp.rows) {
        std::string str("=== > Prior suggests object is not in the scene, skipping tracking "); 
        std::cerr << str << std::endl;
        putText(img, cv::format("%s", str.c_str()), Point(5,20), 0, 0.5, cv::Scalar(200, 200, 200), 1);
        return loglikelihoods;
    }

    // 3. Performs mean shift on the back projection, and recomputes window size
    // updated window size can be used for depth estimation
    cv::RotatedRect est_win = cv::CamShift(_bp, pred_win,
                                           TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
    if( pred_win.area() <= 1 ) {
        int cols = _bp.cols, rows = _bp.rows, r = (std::min(cols, rows) + 5)/6;
        pred_win = Rect(pred_win.x - r, pred_win.y - r,
                        pred_win.x + r, pred_win.y + r) &
            cv::Rect(0, 0, cols, rows);
    }
    int u_estimated = est_win.center.x; 
    int v_estimated = est_win.center.y;

    // Check % increase in width/height (rule out possibilities)
    //std::cerr << "Posterior rect from CAMSHIFT : " 
    //          << est_win.boundingRect().tl() << "->" << est_win.boundingRect().br() << std::endl;
         
    // Update prediction for next update to be current estimate
    // Otherwise, use prediction from incoming particle predictions
    pred_win = est_win.boundingRect();
    //std::cout << u_estimated << " and " << v_estimated << "\n";
      
    // 4. Determine Likelihood in Image space:
    for (size_t i=0; i< pts.size(); i++) {
        // u = pt.x fx/pt.z   ... project point to pixel
        Eigen::Vector3d pt1 = pts[i];
        int u = floor( ((pt1[0] * fx_)/pt1[2]) + cx_);
        int v = floor( ((pt1[1] * fy_)/pt1[2]) + cy_);
        int dist = sqrt( pow( u - u_estimated ,2) + pow( v - v_estimated ,2) );
        // Crude Binary Likelihood:
        if (dist < 13){
            loglikelihoods[i] =1; //was 1
        }
    }        


    // Particles/Posterior Visualization:
    for (size_t i=0; i< pts.size(); i++) { // draw particles on image
      Eigen::Vector3d pt1 = pts[i];
      int u = floor( ((pt1[0] * fx_)/pt1[2])  +  512.5);
      int v = floor( ((pt1[1] * fy_)/pt1[2]) + 272.5);
      Point center( u, v );
      circle( img, center, 2, Scalar(0,255,0), -1, 8, 0 );
    }      
    Point center( u_estimated, v_estimated );
    circle( img, center, 10, Scalar(0,0,255), 1, 8, CV_AA );

    // Draw estimated window 
    ellipse( img, est_win, cv::Scalar(0,255,0), 1, CV_AA );    

    // Draw predicted window by PF pts
    cv::rectangle( img, pred_win.tl(), pred_win.br(), cv::Scalar(0,0,255), 1, 8 );
  
    showHistogramInfo(img);

    return loglikelihoods;
}

cv::Mat 
HistogramTracker::get_belief() { 
    return belief; 
}

void 
HistogramTracker::showHistogramInfo(cv::Mat& img) { 
    float scale = 0.2;
    int w = img.cols, h = img.rows;
    int sw = scale * w, sh = scale * h; 
    
    int off = 5;
    float ow = object_roi.cols, oh = object_roi.rows;
    cv::Mat obj_roi = cv::Mat(img, cv::Rect(w-3*sw-3*off-ow,off,ow,oh));
    cv::Mat hue_roi = cv::Mat(img, cv::Rect(w-3*sw-3*off,off,sw,sh));
    cv::Mat val_roi = cv::Mat(img, cv::Rect(w-2*sw-2*off,off,sw,sh));
    cv::Mat sat_roi = cv::Mat(img, cv::Rect(w-sw-off,off,sw,sh));

    if (!hue_info.histogram.empty()) { 
      cv::Mat pip_hist, pip_val, pip_sat, pip_obj;
        cv::resize(hue_info.histogram_img, pip_hist, hue_roi.size());
        cv::resize(val_info.histogram_img, pip_val, val_roi.size());
	cv::resize(sat_info.histogram_img, pip_sat, sat_roi.size());

        addWeighted(hue_roi, 0.5, pip_hist, 0.5, 0, hue_roi);
        addWeighted(val_roi, 0.5, pip_val, 0.5, 0, val_roi);
        addWeighted(sat_roi, 0.5, pip_sat, 0.5, 0, sat_roi);
        addWeighted(obj_roi, 0.5, object_roi, 0.5, 0, obj_roi);
    }
    return;
}

void HistogramTracker::plot_histogram(char* win) { 
    // cv::imshow(win, hist_img); 
    return;
} 
