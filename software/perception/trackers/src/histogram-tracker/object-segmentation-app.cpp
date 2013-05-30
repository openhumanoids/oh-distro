#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <lcmtypes/perception_image_roi_t.h>

#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
// #include <particle/particle_filter.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <ConciseArgs>

// OpenCV, and Histogram tracker includes
#include <opencv2/opencv.hpp>
#include <trackers/histogram-tracker.hpp>

using namespace cv;

const int WINDOW_WIDTH = 800; 
const int WINDOW_HEIGHT = 800; 
const int SEG_SIZE = 10;
#define WINDOW_NAME "Object Segmenter"

struct ObjectSegmentationOptions { 
    bool vDEBUG;
    std::string vCHANNEL;

    ObjectSegmentationOptions () : 
        vCHANNEL(std::string("CAMERALEFT")), vDEBUG(false) {}
};
ObjectSegmentationOptions options;

struct state_t { 
    lcm_t* lcm;
    pthread_t lcm_thread;
    pthread_mutex_t img_mutex;

    BotParam* param;
    BotFrames* frames;

    // Img
    cv::Mat img;

    bool needs_update;
    std::vector<cv::Point> bg_pts, fg_pts;

    // Color info
    // HistogramInfo hue_info, val_info, sat_info; 

    // utimes for image
    int64_t img_utime;

    int counter;

    // void init_hsv_histogram() { 

    //     // Hue bins, params etc. 
    //     hue_info.size = 40, val_info.size = 40, sat_info.size = 40;
    //     hue_info.bin_dev = 5, val_info.bin_dev = 5, sat_info.bin_dev = 5;

    //     hue_info.ranges[0] = 0, hue_info.ranges[1] = 180;
    //     val_info.ranges[0] = 10, val_info.ranges[1] = 255;
    //     sat_info.ranges[0] = 10, sat_info.ranges[1] = 255;

    //     hue_info.pranges = hue_info.ranges;
    //     val_info.pranges = val_info.ranges;
    //     sat_info.pranges = sat_info.ranges;

    //     hue_info.init();
    //     val_info.init();
    //     sat_info.init();
    // }

    state_t () {
        // LCM, BotFrames, BotParam inits        
        lcm =  bot_lcm_get_global(NULL);
        param = bot_param_new_from_server(lcm, 1);
        frames = bot_frames_get_global (lcm, param);

        // init_hsv_histogram();

        img_mutex = PTHREAD_MUTEX_INITIALIZER;

        needs_update = true;

        counter = 0; 
    }

    ~state_t () { 
        lcm_destroy(lcm);
    }
};
state_t* state = NULL; 

void* lcm_thread_handler(void *l) {
    state_t* state = (state_t*)l;
    while(1)
        lcm_handle(state->lcm);
}

void
decode_image(const bot_core_image_t * msg, cv::Mat& img)
{
    int ch = msg->row_stride / (msg->width); 
    if (img.empty() || img.rows != msg->height || img.cols != msg->width)
        if (ch == 3) 
            img.create(msg->height, msg->width, CV_8UC3);
        else 
            img.create(msg->height, msg->width, CV_8UC1);
    //std::cerr << "msg: " << ch << " " << msg->row_stride << " " << msg->width << "x" << msg->height << std::endl;
        
    // extract image data
    switch (msg->pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
        memcpy(img.data, msg->data, sizeof(uint8_t) * msg->width * msg->height * 3);
        cv::cvtColor(img, img, CV_RGB2BGR);
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
        // for some reason msg->row_stride is 0, so we use msg->width instead.
        if (ch == 3) { 
            jpeg_decompress_8u_rgb(msg->data,
                                   msg->size,
                                   img.data,
                                   msg->width,
                                   msg->height,
                                   msg->width * ch);
            cv::cvtColor(img, img, CV_RGB2BGR);
        } else {  
            jpeg_decompress_8u_gray(msg->data,
                                    msg->size,
                                    img.data,
                                    msg->width,
                                    msg->height,
                                    msg->width);
        }
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY:
        memcpy(img.data, msg->data, sizeof(uint8_t) * msg->width * msg->height);
        break;
    default:
        fprintf(stderr, "Unrecognized image format\n");
        break;
    }
    return;
}

struct MouseEvent {
    MouseEvent() { event = -1; buttonState = 0; }
    cv::Point pt;
    int event;
    int buttonState;
};
MouseEvent mouse;

static void onMouse(int event, int x, int y, int flags, void* userdata) {
    MouseEvent* data = (MouseEvent*)userdata;

    switch (event) {
    case CV_EVENT_RBUTTONDOWN:
        break;
    case CV_EVENT_RBUTTONUP:
        state->bg_pts.push_back(cv::Point(x,y));
        state->needs_update = true;
        break;
    case CV_EVENT_LBUTTONDOWN:
        break;
    case CV_EVENT_LBUTTONUP:
        state->fg_pts.push_back(cv::Point(x,y));
        state->needs_update = true;
        break;
    }
    return;
}

//This colors the segmentations
static void floodFillPostprocess( Mat& img, const Scalar& colorDiff=Scalar::all(1) )
{
    CV_Assert( !img.empty() );
    RNG rng = theRNG();
    Mat mask( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0) );
    for( int y = 0; y < img.rows; y++ )
    {
        for( int x = 0; x < img.cols; x++ )
        {
            if( mask.at<uchar>(y+1, x+1) == 0 )
            {
                Scalar newVal( rng(256), rng(256), rng(256) );
                floodFill( img, mask, Point(x,y), newVal, 0, colorDiff, colorDiff );
            }
        }
    }
}

void trainGMM(cv::EM& model, cv::Mat& img, std::vector<cv::Point>& pts) { 

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
    printf("===> GMM %s \n", model.train(src_samples) ? "GOOD" : "BAD");
    printf("===> GMM Trained (%i mixtures: %i samples): %4.2f ms\n", pts.size(), nsamples, (bot_timestamp_now() - st) * 1e-3); 
}

void predictGMM(cv::EM& model, cv::Mat& img, cv::Mat& bp) { 
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
            Vec2d res = model.predict(sample, pr);
            pbp[x] = exp(res[0]);
        }
    }

    printf("===> GMM Predict %4.2f ms\n", (bot_timestamp_now() - st) * 1e-3); 
    return;
}

//--------------------------------------------
// Callbacks
//--------------------------------------------
static void on_image_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const bot_core_image_t *msg, 
                            void *user_data ) {
    // std::cerr << "on_image_frame " << std::endl;
    if (!msg->width || !msg->height) return;
    
    pthread_mutex_lock(&state->img_mutex);
    state_t* state = (state_t*) user_data; 
    if (state->img.empty() || state->img.rows != msg->height || state->img.cols != msg->width) { 
        if (msg->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY) { 
            std::cerr << "ERROR: Incoming image is grayscale!! Cannot perform color tracking!!!" << std::endl;
            assert(0);
        } else { 
            std::cerr << "One time creation of image" << std::endl;
            state->img.create(msg->height, msg->width, CV_8UC3);
        }
    }
    decode_image(msg, state->img);    
    pthread_mutex_unlock(&state->img_mutex);
    state->img_utime = msg->utime; 

    // Blur Image
    cv::Mat img; 
    cv::blur(state->img, img, cv::Size(5,5));

    double tic = bot_timestamp_now(); 

    // Return if no seeds
    if (!state->needs_update || !state->fg_pts.size())
        return;
    std::cerr << "Updating ===> " << std::endl;

    // HSV conversion
    cv::Mat hsv, lab; 
    cvtColor(img, hsv, CV_BGR2HSV);

    // Train GMM model with fg_pts.size() seeds
    cv::EM model(state->fg_pts.size(), EM::COV_MAT_GENERIC);
    trainGMM(model, hsv, state->fg_pts);
    
    // Predict based on trained model
    cv::Mat_<float> bp;
    predictGMM(model, hsv, bp);

    // Threshold image based on 80% conf.
    cv::Mat1b fgd_mask = (bp >= 0.8) * 255;
    // cv::imshow("bp", bp);

    // Find contours within the response image
    vector<vector<Point> > contours, filtered_contours;
    vector<Vec4i> hierarchy;
    findContours( fgd_mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

    // Output
    cv::Mat dst = Mat::zeros(img.size(), CV_8UC3);
    if( contours.size() == 0 ) return;

    // iterate through all the top-level contours,
    // draw the CC if the seed is within the contour
    Scalar color( 0, 255, 255 );
    RNG rng = theRNG();
    for (int j=0; j<state->fg_pts.size(); j++) { 
        int idx = 0;
        for( ; idx >= 0; idx = hierarchy[idx][0] ) {
            const vector<Point>& c = contours[idx];
            // std::cerr << "distance: " << pointPolygonTest(c, state->fg_pts[j], true) << " " << c[0] << state->fg_pts[j] << std::endl;
            if (pointPolygonTest(c, state->fg_pts[j], true) >= 0) { 
                drawContours( dst, contours, idx, color, CV_FILLED, 8, hierarchy );
            }
        }
    }
    cv::imshow("Segmentation", dst * 0.7 + state->img * 0.3);
    std::cerr << "Done updating" << std::endl;
    state->needs_update = false;
    return;
}

void  INThandler(int sig) {
    printf("Exiting\n");
    exit(0);
}


int main(int argc, char ** argv) {
    cout << "============= QUICK MODES ===================\n";
    cout << "drc-object-segmentation -c CAMERALEFT\n";
    cout << "=============================================\n";

    ConciseArgs opt(argc, (char**)argv);
    opt.add(options.vCHANNEL, "c", "camera-channel","Camera Channel [CAMERALEFT]");
    opt.add(options.vDEBUG, "d", "debug","Debug mode");
    opt.parse();
  
    std::cerr << "===========  Object Segmenter ============" << std::endl;
    std::cerr << "=> CAMERA CHANNEL : " << options.vCHANNEL << std::endl;
    std::cerr << "=> DEBUG : " << options.vDEBUG << std::endl;
    std::cerr << "=> Note: Hit 'c' to capture mask" << std::endl;
    std::cerr << "===============================================" << std::endl;
  
    // Install signal handler to free data.
    signal(SIGINT, INThandler);

    // Param server, botframes
    state = new state_t();

    printf("starting lcm thread\n");
    // pthread_create(&(state->lcm_thread), NULL, lcm_thread_handler, state);

    cv::namedWindow( WINDOW_NAME );
    cv::setMouseCallback( WINDOW_NAME, onMouse, &mouse);

    // Subscriptions
    bot_core_image_t_subscribe(state->lcm, options.vCHANNEL.c_str(), on_image_frame, (void*)state);

    while(1) { 
        unsigned char c = cv::waitKey(1) & 0xff;
	lcm_handle(state->lcm);
        if (c == 'q') { 
            break;
        } else if ( c == 'c' ) { 
            cerr << "Capturing image" << endl;
            cv::imwrite("capture.png", state->img);
        } else if ( c == 'r' ) { 
            state->fg_pts.clear(); 
            state->bg_pts.clear();
        }
            

        // pthread_mutex_lock(&state->img_mutex);
        // UI handling 
        if (!state->img.empty()) { 
            cv::Mat display;
            cv::resize(state->img.clone(), display, cv::Size(WINDOW_WIDTH,WINDOW_HEIGHT)); 

            for (int j=0; j<state->bg_pts.size(); j++) {
                cv::Point tl(state->bg_pts[j].x-SEG_SIZE/2, state->bg_pts[j].y-SEG_SIZE/2);
                cv::Point br(state->bg_pts[j].x+SEG_SIZE/2, state->bg_pts[j].y+SEG_SIZE/2);
                // cv::rectangle(display, tl, br, cv::Scalar(0,0,255), 1, CV_AA);
                cv::circle(display, state->bg_pts[j], 5, cv::Scalar(0,255,0), 1, CV_AA);
            }
            for (int j=0; j<state->fg_pts.size(); j++) {
                cv::Point tl(state->fg_pts[j].x-SEG_SIZE/2, state->fg_pts[j].y-SEG_SIZE/2);
                cv::Point br(state->fg_pts[j].x+SEG_SIZE/2, state->fg_pts[j].y+SEG_SIZE/2);
                // cv::rectangle(display, tl, br, cv::Scalar(0,255,0), 1, CV_AA);
                cv::circle(display, state->fg_pts[j], 5, cv::Scalar(0,255,0), 1, CV_AA);
            }

            imshow(WINDOW_NAME, display);
        }
        // pthread_mutex_unlock(&state->img_mutex);
    }

    // if (state) delete state; 
    return 0;
}



// void initialize_histogram_from_seeds(const cv::Mat& img, std::vector<cv::Point>& pts) { 

//     cv::Mat hsv; 
//     cvtColor(img, hsv, CV_BGR2HSV);

//     std::vector<cv::Mat> channels;
//     cv::split(hsv, channels);
//     assert(channels.size() == 3);
//     cv::Mat hue = channels[0]; 
//     cv::Mat val = channels[1];
//     cv::Mat sat = channels[2];
//     // std::cerr << "hue: " << hue << std::endl;

//     cv::Mat1b mask_ = cv::Mat1b::zeros(img.size()); 
    
//     state->hue_info.histogram = cv::Mat();
//     state->sat_info.histogram = cv::Mat();
//     state->val_info.histogram = cv::Mat();

//     for (int j=0; j<pts.size(); j++) { 

//         cv::Mat1b mask = mask_.clone();
//         cv::Point tl(pts[j].x-SEG_SIZE/2, pts[j].y-SEG_SIZE/2);
//         cv::Point br(pts[j].x+SEG_SIZE/2, pts[j].y+SEG_SIZE/2);
//         cv::rectangle(mask, tl, br, cv::Scalar::all(255), -1, CV_AA);

//         cv::Mat hhist = state->hue_info.histogram.clone(); 
//         cv::Mat vhist = state->sat_info.histogram.clone(); 
//         cv::Mat shist = state->val_info.histogram.clone(); 

//         // Calculate Histogram
//         calcHist(&hue, 1, 0, mask, hhist, 1, &state->hue_info.size, &state->hue_info.pranges); 
//         calcHist(&val, 1, 0, mask, vhist, 1, &state->val_info.size, &state->val_info.pranges); 
//         calcHist(&sat, 1, 0, mask, shist, 1, &state->sat_info.size, &state->sat_info.pranges); 

//         normalize(hhist, hhist, 0, 1, CV_MINMAX);
//         normalize(vhist, vhist, 0, 1, CV_MINMAX);
//         normalize(shist, shist, 0, 1, CV_MINMAX);

//         if (state->hue_info.histogram.empty())
//             state->hue_info.histogram = hhist.clone();
//         else 
//             state->hue_info.histogram = state->hue_info.histogram + hhist;

//         if (state->val_info.histogram.empty())
//             state->val_info.histogram = vhist.clone();
//         else
//             state->val_info.histogram = state->val_info.histogram + vhist;

//         if (state->sat_info.histogram.empty())
//             state->sat_info.histogram = shist.clone();
//         else
//             state->sat_info.histogram = state->val_info.histogram + shist;
//     }

//     normalize(state->hue_info.histogram, state->hue_info.histogram, 0, 1, CV_MINMAX);
//     normalize(state->val_info.histogram, state->val_info.histogram, 0, 1, CV_MINMAX);
//     normalize(state->sat_info.histogram, state->sat_info.histogram, 0, 1, CV_MINMAX);

//     // Compute unimodal histogram
//     state->hue_info.computeUnimodalHistogram();
//     state->val_info.computeUnimodalHistogram();
//     state->sat_info.computeUnimodalHistogram();

//     return;
// }

// void prepare_mask(cv::Mat& mask, std::vector<cv::Point>& pts, int val) { 
//     for (int j=0; j<pts.size(); j++) {
//         cv::Point tl(pts[j].x-SEG_SIZE/2, pts[j].y-SEG_SIZE/2);
//         cv::Point br(pts[j].x+SEG_SIZE/2, pts[j].y+SEG_SIZE/2);
//         cv::rectangle(mask, tl, br, cv::Scalar(val), -1, CV_AA);
//     }
//     return;
// }



    // // Initialize H,S,V bins for backprojection
    // initialize_histogram_from_seeds(img, state->fg_pts);


    // std::vector<cv::Mat> channels;
    // cv::split(hsv, channels);
    // assert(channels.size() == 3);
    // cv::Mat_<float> hue = channels[0]; 
    // cv::Mat_<float> val = channels[1];
    // cv::Mat_<float> sat = channels[2];
 
    // // Calculate likelihood
    // cv::Mat_<float> hue_bp, val_bp, sat_bp;
    // state->hue_info.backProject(hue, hue_bp);
    // state->val_info.backProject(val, val_bp);
    // state->sat_info.backProject(sat, sat_bp);

    // // Compute compound belief
    // cv::Mat_<float> bp;
    // multiply(hue_bp, val_bp, bp);
    // multiply(sat_bp, bp, bp);
    // normalize(bp, bp, 0, 1, CV_MINMAX);

    // // FG
    // cv::Mat pr_fg;
    // bitwise_and(bp >= 0.5, bp < 0.8, pr_fg);

    // // BG
    // cv::Mat pr_bg = (bp < 0.05);
    // erode(pr_bg, pr_bg, cv::Mat(), cv::Point(-1,-1), 3, BORDER_CONSTANT);

    // // Reset mask
    // cv::Mat1b mask = cv::Mat1b::zeros(state->img.size());
    // mask = GC_PR_BGD;

    // // Prepare the FG mask
    // // Determine FG, BG, PR_FG, and PR_BG
    // uchar* maskp = (uchar*) mask.data;
    // const uchar* pr_fgp = (const uchar*) pr_fg.data;
    // const uchar* pr_bgp = (const uchar*) pr_bg.data;
    // const float* bpp = (const float*) bp.data;
    // for (int j=0; j<bp.rows * bp.cols; j++) { 
    //     if (bpp[j] >= 0.8) maskp[j] = GC_FGD; 
    //     if (pr_fgp[j]) maskp[j] = GC_PR_FGD; 
    //     if (pr_bgp[j]) maskp[j] = GC_PR_BGD;
    // }

    // cv::imshow("bp", bp);
    // // cv::imshow("fg", mask == GC_FGD); 
    // // cv::imshow("pr_fg", mask == GC_PR_FGD);
    // // cv::imshow("pr_bg", mask == GC_PR_BGD);

    // if (!countNonZero(mask == GC_FGD)) { 
    //     std::cerr << "Not enough color information: Removing annotation" << std::endl; 
    //     state->fg_pts.pop_back();
    //     return; 
    // }

    // const int iters = 6;
    // cv::Mat bgdModel, fgdModel;
    // grabCut(hsv, mask, cv::Rect(), bgdModel, fgdModel, iters, GC_INIT_WITH_MASK);

    // cv::Mat fgd_mask;
    // bitwise_or(mask == GC_PR_FGD, mask == GC_FGD, fgd_mask);




    // cv::imshow("gc_pr_fg", mask == GC_PR_FGD);
    // cv::imshow("gc_pr_bg", mask == GC_PR_BGD);
    // cv::imshow("gc_bg", mask == GC_BGD);
    // cv::imshow("gc_fg", mask == GC_FGD);
    // cv::Mat1b omask = (mask == GC_PR_FGD) | (mask == GC_FGD);
    // cv::Mat3b omask3;
    // cv::cvtColor(omask, omask3, CV_GRAY2BGR);

    // if (++state->counter == 10) { 
    //     printf("===> BG CUT: %4.2f ms\n", (bot_timestamp_now() - tic) * 1e-3); 
    //     state->counter = 0;
    // }
    // cv::imshow("Camera", img*.5 + omask3 * 0.5);
