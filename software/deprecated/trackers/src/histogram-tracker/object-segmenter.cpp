#include "object-segmenter.hpp"

const int SEG_SIZE = 10;

ObjectSegmenter::ObjectSegmenter() { 
    init_hsv_histogram();
    initialized = false;
}

ObjectSegmenter::~ObjectSegmenter() { 
    
}

void ObjectSegmenter::init_hsv_histogram() { 
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

    return;
}

void ObjectSegmenter::initialize_histogram(const cv::Mat& hsv, std::vector<cv::Point>& pts) { 

    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    assert(channels.size() == 3);
    cv::Mat hue = channels[0]; 
    cv::Mat val = channels[1];
    cv::Mat sat = channels[2];
    // std::cerr << "hue: " << hue << std::endl;

    cv::Mat1b mask_ = cv::Mat1b::zeros(hsv.size()); 
    
    hue_info.histogram = cv::Mat();
    sat_info.histogram = cv::Mat();
    val_info.histogram = cv::Mat();

    for (int j=0; j<pts.size(); j++) { 

        cv::Mat1b mask = mask_.clone();
        cv::Point tl(pts[j].x-SEG_SIZE/2, pts[j].y-SEG_SIZE/2);
        cv::Point br(pts[j].x+SEG_SIZE/2, pts[j].y+SEG_SIZE/2);
        // cv::rectangle(mask, tl, br, cv::Scalar::all(255), -1, CV_AA);
        cv::circle(mask, pts[j], SEG_SIZE, cv::Scalar::all(255), -1, CV_AA);

        cv::Mat hhist = hue_info.histogram.clone(); 
        cv::Mat vhist = sat_info.histogram.clone(); 
        cv::Mat shist = val_info.histogram.clone(); 

        // Calculate Histogram
        calcHist(&hue, 1, 0, mask, hhist, 1, &hue_info.size, &hue_info.pranges); 
        calcHist(&val, 1, 0, mask, vhist, 1, &val_info.size, &val_info.pranges); 
        calcHist(&sat, 1, 0, mask, shist, 1, &sat_info.size, &sat_info.pranges); 

        normalize(hhist, hhist, 0, 1, CV_MINMAX);
        normalize(vhist, vhist, 0, 1, CV_MINMAX);
        normalize(shist, shist, 0, 1, CV_MINMAX);

        if (hue_info.histogram.empty())
            hue_info.histogram = hhist.clone();
        else 
            hue_info.histogram = hue_info.histogram + hhist;

        if (val_info.histogram.empty())
            val_info.histogram = vhist.clone();
        else
            val_info.histogram = val_info.histogram + vhist;

        if (sat_info.histogram.empty())
            sat_info.histogram = shist.clone();
        else
            sat_info.histogram = val_info.histogram + shist;
    }

    normalize(hue_info.histogram, hue_info.histogram, 0, 1, CV_MINMAX);
    normalize(val_info.histogram, val_info.histogram, 0, 1, CV_MINMAX);
    normalize(sat_info.histogram, sat_info.histogram, 0, 1, CV_MINMAX);

    // Compute unimodal histogram
    hue_info.computeUnimodalHistogram();
    val_info.computeUnimodalHistogram();
    sat_info.computeUnimodalHistogram();

    return;
}

void ObjectSegmenter::initialize_GMM(const cv::Mat& img, std::vector<cv::Point>& pts) { 

    // Create Mask
    cv::Mat1b mask = cv::Mat1b::zeros(img.size());
    for (int j=0; j<pts.size(); j++) { 
        cv::Point tl(pts[j].x-SEG_SIZE/2, pts[j].y-SEG_SIZE/2);
        cv::Point br(pts[j].x+SEG_SIZE/2, pts[j].y+SEG_SIZE/2);
        cv::rectangle(mask, tl, br, cv::Scalar::all(255), -1, CV_AA);
    }

    int nsamples = countNonZero(mask);
    cv::Mat src_samples(nsamples, 3, CV_32FC1);

    cv::Mat src_32f; 
    img.convertTo(src_32f, CV_32F, 1.f / 255.0); 

    int count = 0;
    for(int y=0; y<img.rows; y++) {
        Vec3f* row = src_32f.ptr<Vec3f>(y);
        uchar* mask_row = mask.ptr<uchar>(y);
        for(int x=0; x<img.cols; x++) {
            if(mask_row[x] > 0) {
                src_samples.at<Vec3f>(count++,0) = row[x];
            }
        }
    }
    
    // Create GMM with pts.size() mixtures
    double st = bot_timestamp_now();
    printf("===> GMM %s \n", gmm_model.train(src_samples) ? "GOOD" : "BAD");
    printf("===> GMM Trained (%i mixtures: %i samples): %4.2f ms\n", pts.size(), nsamples, (bot_timestamp_now() - st) * 1e-3); 

}

void ObjectSegmenter::initialize(const cv::Mat& img, std::vector<cv::Point>& pts) { 

    initialized = false;

    // Blur Image
    cv::Mat imgb; 
    cv::blur(img, imgb, cv::Size(5,5));


    // Segmentation in HSV space
    cv::Mat hsv; 
    cvtColor(imgb, hsv, CV_BGR2HSV);

    initialize_histogram(hsv, pts); 

    // GMM: updates em model
    // Train GMM model with pts as seeds
    gmm_model = cv::EM(2, EM::COV_MAT_GENERIC);
    initialize_GMM(hsv, pts);

    // Reset detection
    p_detect = cv::Rect(pts.end()[-1].x-SEG_SIZE/2,pts.end()[-1].y-SEG_SIZE/2,SEG_SIZE,SEG_SIZE);
    p_detect &= cv::Rect(0,0,img.cols,img.rows);

    initialized = true;
    return;
}

void ObjectSegmenter::belief_hist(const cv::Mat& hsv_, cv::Mat_<float>& bp) { 

    float scale = 0.5;
    cv::Mat hsv;
    cv::resize(hsv_, hsv, cv::Size(), scale, scale, cv::INTER_AREA);

    // HSV: backprojection
    std::vector<cv::Mat> channels;
    cv::split(hsv_, channels);
    assert(channels.size() == 3);
    cv::Mat_<float> hue = channels[0]; 
    cv::Mat_<float> val = channels[1];
    cv::Mat_<float> sat = channels[2];
 
    // Calculate likelihood
    cv::Mat_<float> hue_bp, val_bp, sat_bp;
    hue_info.backProject(hue, hue_bp);
    normalize(hue_bp, hue_bp, 0, 1, CV_MINMAX);
    val_info.backProject(val, val_bp);
    normalize(val_bp, val_bp, 0, 1, CV_MINMAX);
    // sat_info.backProject(sat, sat_bp);
    // normalize(sat_bp, sat_bp, 0, 1, CV_MINMAX);

    // Compute compound belief (H/V values)
    multiply(hue_bp, val_bp, bp); // look at only hue, and val
    normalize(bp, bp, 0, 1, CV_MINMAX);

    // cv::resize(bp, bp, cv::Size(), 1.f/scale, 1.f/scale, cv::INTER_AREA);

    // if (options.vDEBUG) { 
        // cv::imshow("hue", hue_bp);
        // cv::imshow("sat", sat_bp);
        // cv::imshow("val", val_bp);
        // cv::imshow("bp", bp);
    // }

    return;
}

static bool contour_size_compare(const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) { 
    return lhs.second < rhs.second;
}

cv::Mat1b ObjectSegmenter::find_largest_contour_mask(cv::Mat1b& mask) { 

    cv::Mat1b out = cv::Mat1b::zeros(mask.size());

    // Find contours for the mask
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    cv::Size size(mask.cols, mask.rows);
    cv::findContours(mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE); 
    if (!contours.size()) return out;

    int largest_contour_id = find_largest_contour(contours, hierarchy);
    drawContours(out, contours, largest_contour_id, cv::Scalar(255), CV_FILLED, 8, hierarchy); 

    return out;
}

int ObjectSegmenter::find_largest_contour(const vector<vector<Point> >& contours, vector<Vec4i>& hierarchy) { 

    // Sort contours with max area
    std::vector<std::pair<int, int> > contour_sizes; 
    for (int idx=0; idx>=0; idx = hierarchy[idx][0])
        contour_sizes.push_back(std::make_pair(idx, cv::contourArea(contours[idx])));
    std::vector<std::pair<int, int> >::iterator it = 
        std::max_element(contour_sizes.begin(), contour_sizes.end(), contour_size_compare);

    int largest_contour_id = it->first;
    return largest_contour_id;
}

cv::Mat1b ObjectSegmenter::process(const cv::Mat& img, pr_type type) { 

    // Blur Image
    cv::Mat imgb; 
    // cv::medianBlur(img, imgb, 5);
    cv::blur(img, imgb, cv::Size(5,5));

    // Segmentation in HSV space
    cv::Mat hsv, lab; 
    cvtColor(imgb, hsv, CV_BGR2HSV);

    // // Resize for speed
    // if (options.vSCALE != 1.f)
    //     cv::resize(hsv, hsv, cv::Size(), options.vSCALE, options.vSCALE, cv::INTER_AREA);

    // 1st step: Belief with HISTOGRAM BACK-PROJECTION
    double tic = bot_timestamp_now();
    hist_bp = cv::Mat_<float>(); 
    belief_hist(hsv, hist_bp);
    printf("===> HIST: %4.2f ms\n", (bot_timestamp_now() - tic) * 1e-3);

    // Foreground prediction mask
    cv::Mat1b fg = hist_bp>=0.5;
    // cv::imshow("fg", fg);

    if (type == pr_type::SEGMENT) { 
        // Should've been set by initialize
        assert(p_detect.area() > 1);
    } else { 
        // If previous detection is invalid (set to full image)
        if( p_detect.area() <= 1 ) {
            int cols = hist_bp.cols, rows = hist_bp.rows, r = (MIN(cols, rows) + 5)/6;
            p_detect = cv::Rect(p_detect.x - r, p_detect.y - r,
                                p_detect.x + r, p_detect.y + r) &
                Rect(0, 0, cols, rows);
        } 
    }


    // Run for several iterations before it converges
    cv::Mat dst = img.clone();
    cv::RotatedRect trackBox;
    if (type == pr_type::SEGMENT) {  
        for (int j=0; j<10; j++) {
            trackBox = cv::CamShift(hist_bp, p_detect,
                                                    cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 1 ));
            p_detect = trackBox.boundingRect(); 
        }
    } else { 
        trackBox = cv::CamShift(hist_bp, p_detect,
                                cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
    }

    // Output mask (if zeros, couldn't estimate bounding box)
    cv::Mat1b out_mask = cv::Mat1b::zeros(img.size());    

    // If valid detection
    if( p_detect.area() > 1 ) {
        p_detect = trackBox.boundingRect(); 
        p_detect &= cv::Rect(0,0,img.cols,img.rows);
        cv::rectangle(dst, p_detect.tl(), p_detect.br(), cv::Scalar(0,255,0), 1, CV_AA);

        printf("===> Predicting GMM model\n");

        // Bounding box with some margin
        cv::Rect bb = p_detect;
        bb.x -= 4;
        bb.y -= 4;
        bb.width += 8; 
        bb.height += 8; 
        bb &= cv::Rect(0,0,img.cols,img.rows);

        // ROI
        cv::Mat roi(hsv, bb);

        // Predict based on trained model
        cv::Mat_<float> bp_gmm_roi;
        double st1 = bot_timestamp_now();
        predictGMM(roi, bp_gmm_roi);

        // Dilate with 50% conf. mask image
        cv::Mat1b bp_gmm_mask_roi = bp_gmm_roi >=0.5;
        cv::dilate(bp_gmm_mask_roi, bp_gmm_mask_roi, cv::Mat(), cv::Point(-1,-1), 2);        
        bp_gmm_mask_roi = find_largest_contour_mask(bp_gmm_mask_roi);
        cv::erode(bp_gmm_mask_roi, bp_gmm_mask_roi, cv::Mat(), cv::Point(-1,-1), 1);        

        // Draw mask on full image
        roi = cv::Mat(out_mask, bb);
        bp_gmm_mask_roi.copyTo(roi);

        printf("===> GMM: %4.2f ms\n", (bot_timestamp_now() - st1) * 1e-3);
    } 

    return out_mask;
}


cv::Mat1b ObjectSegmenter::track(const cv::Mat& img) { 
    return process(img, pr_type::TRACK);
}

cv::Mat1b ObjectSegmenter::segment(const cv::Mat& img) { 
    return process(img, pr_type::SEGMENT);
}


void ObjectSegmenter::predictGMM(cv::Mat& img, cv::Mat& bp) { 
    cv::Mat src_32f; 
    img.convertTo(src_32f, CV_32F, 1.f / 255.0); 

    bp = cv::Mat_<float>::zeros(img.size());

    cv::Mat sample(1, 3, CV_32FC1);

    
    double st = bot_timestamp_now();
    cv::Mat pr; 
    int count = 0; 
    for(int y=0; y<img.rows; y++) {
        Vec3f* row = src_32f.ptr<Vec3f>(y);
        float* pbp = bp.ptr<float>(y);
        for(int x=0; x<img.cols; x++) {
            sample.at<Vec3f>(0,0) = row[x];
            Vec2d res = gmm_model.predict(sample, pr);
            pbp[x] = exp(res[0]);
        }
    }

    printf("===> GMM Predict %4.2f ms\n", (bot_timestamp_now() - st) * 1e-3); 
    return;
}


