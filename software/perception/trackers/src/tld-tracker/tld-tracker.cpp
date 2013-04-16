#include "tld-tracker.hpp"

TLDTracker::TLDTracker(int _width, int _height, float _scale) { 
    tld = NULL;
    scale_factor = _scale;
    initialized=false;
    detection_valid = false;
    img_size = cv::Size(_width, _height);
    internal_init(); 
}

TLDTracker::~TLDTracker() {
    delete tld;
}


void
TLDTracker::internal_init() { 
    // Main tld instance
    if (tld) delete tld;
    tld = new tld::TLD();

    // Preprocessing settings
    apply_gaussian_filter = false;

    // Initialize tracker settings
    tld->alternating = false;
    tld->learningEnabled = true;
    tld->trackerEnabled = true;
    tld->scale_factor = scale_factor;

    // Cascade settings
    tld::DetectorCascade* dc = tld->detectorCascade;

    dc->imgWidth = img_size.width * scale_factor;
    dc->imgHeight = img_size.height * scale_factor;
    dc->imgWidthStep = img_size.width * scale_factor;
            
    dc->varianceFilter->enabled = true;
    dc->ensembleClassifier->enabled = true;
    dc->nnClassifier->enabled = true;

    // classifier
    dc->useShift = true; 
    dc->shift = 0.04;
    dc->minScale = -10;
    dc->maxScale = 10;
    dc->minSize = 10; // 25
    dc->numTrees = 10;
    dc->numFeatures = 10;
    dc->nnClassifier->thetaTP = 0.65;
    dc->nnClassifier->thetaFP = 0.5;


    currBB = cv::Rect(0,0,0,0); 
    predBB = cv::Rect(0,0,0,0);

    // prediction window
    pred_win = cv::Rect(0,0,0,0);

  return;
}

bool 
TLDTracker::computeMaskROI(const cv::Mat& img, const cv::Mat& mask, cv::Rect& roi) { 
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
    int rx = minx-10, ry = miny-10; 
    int rw = maxx-minx + 20, rh = maxy-miny + 20;
    cv::Rect _roi(rx, ry, rw, rh);
    if (rw < 0 || rh < 0) return false;
    if ( _roi.area() <= 0 || _roi.tl().x <= 0 || _roi.tl().y <= 0 || 
         _roi.br().x >= img.cols || _roi.br().y >= img.rows)
        return false;
    
    // Populate ROI for initializing prediction window for camshift
    roi = _roi;

    return true;
}

void 
TLDTracker::preprocess_image(cv::Mat& img, cv::Mat& _img) { 
    // Convert to grayscale
    if (img.channels() == 3)
        cv::cvtColor(img, _img, CV_BGR2GRAY);
    else
        _img = img;
    
    // Downsample image
    if (scale_factor != 1.f)
        cv::resize(_img, _img, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);

    // Apply gaussian smoothing
    if (apply_gaussian_filter)
        cv::boxFilter(_img, _img, _img.depth(), cv::Size(3,3));
    return;
}

bool 
TLDTracker::initialize(cv::Mat& img, const cv::Mat& mask) { 
    // Compute mask roi for debug
    if (!computeMaskROI(img, mask, pred_win)) 
        return false;
    
    initialize(img, pred_win);
    return true;
}

bool
TLDTracker::initialize(cv::Mat& img, cv::Rect selection) {
 
    internal_init();

    cv::Mat roi_im = cv::Mat(img, selection);
    roi_im.copyTo(object_roi);

    // Convert to grayscale and downsample if needed
    cv::Mat _img;
    preprocess_image(img, _img);

    // Scale selection
    selection = scaleDown(selection);

    // tld tracker selection
    tld->selectObject(_img, &selection);

    pred_win = selection;
    initialized = true;

    // Once mask is inialized, find the 
    std::cerr << "Initialized: Prediction window " << pred_win.tl() << "->" << pred_win.br() << std::endl;

    return true;
}

cv::Rect
TLDTracker::scaleUp(cv::Rect& rect_in) { 
    cv::Point2f c = 0.5 * (rect_in.tl() + rect_in.br()) * (1.f / scale_factor);
    float w = rect_in.width * 1.f / scale_factor, h = rect_in.height * 1.f / scale_factor;
    cv::Rect rect_out(c.x-w/2, c.y-h/2, rect_in.width * 1.f / scale_factor, rect_in.height * 1.f / scale_factor);
    return rect_out;
}

cv::Rect
TLDTracker::scaleDown(cv::Rect& rect_in) { 
    cv::Point2f c = 0.5 * (rect_in.tl() + rect_in.br()) * scale_factor;
    float w = rect_in.width * scale_factor, h = rect_in.height * scale_factor;
    cv::Rect rect_out(c.x - w/2, c.y - h/2, rect_in.width * scale_factor, rect_in.height * scale_factor);   
    return rect_out;
}

std::vector<float>
TLDTracker::update(cv::Mat& img, std::vector< Eigen::Vector3d > & pts,
        Eigen::Isometry3d local_to_camera) { 

    // Assign uniform likelihoods:
    std::vector<float> loglikelihoods;
    loglikelihoods.assign ( pts.size() ,0);    
  
    if (!initialized) return loglikelihoods;
    
    // Convert to grayscale and downsample if needed
    cv::Mat _img;
    preprocess_image(img, _img);
    
    // Main TLD process image
    detection_valid = tld->processImage(_img);

    // Scale up response, and propagate variables
    if (detection_valid) { 
        currBB = scaleUp(*tld->currBB);
        // predBB = scaleUp(*tld->medianFlowTracker->trackerBB);
        confidence = tld->currConf;
    }

    // show TLD info
    showTLDInfo(img);

    return loglikelihoods;
}

void 
TLDTracker::showTLDInfo(cv::Mat& img) { 
    cv::Mat display = img.clone();

    float scale = 0.2;
    int w = display.cols, h = display.rows;
    int sw = scale * w, sh = scale * h; 
    
    int off = 5;
    float ow = object_roi.cols, oh = object_roi.rows;
    cv::Mat obj_roi = cv::Mat(display, cv::Rect(w-ow,off,ow,oh));

    if (!object_roi.empty()) { 
        addWeighted(obj_roi, 0.5, object_roi, 1.0, 0, obj_roi);
    }

    if (detection_valid) { 
        cv::rectangle(display, currBB.tl(), currBB.br(), cv::Scalar(0,255,0), 1, 8);
        cv::rectangle(display, predBB.tl(), predBB.br(), cv::Scalar(255,255,0), 1, 8);
    }
    cv::imshow("TLD", display);
    return;
}
