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
#include <lcmtypes/drc_point_list_t.h>

#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
// #include <particle/particle_filter.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <ConciseArgs>

#include <trackers/object-segmenter.hpp>

using namespace cv;


#define WINDOW_NAME "Object Segmenter"

struct ObjectSegmentationOptions { 
    bool vDEBUG;
    float vSCALE;
    std::string vCHANNEL;

    ObjectSegmentationOptions () : 
        vCHANNEL(std::string("CAMERALEFT")), vDEBUG(false), vSCALE(1.f) {}
};
ObjectSegmentationOptions options;

struct state_t { 
    lcm_t* lcm;
    pthread_t lcm_thread;
    pthread_mutex_t img_mutex;
    pthread_mutex_t buffer_mutex;

    BotParam* param;
    BotFrames* frames;
    image_io_utils*  imgutils_;

    cv::Rect p_detect;

    // Img
    cv::Mat img;

    std::map<int64_t, cv::Mat> buffer_map; // all the local images
    std::deque<int64_t> buffer_utimes; // all the local image utimes
    int64_t buffer_timespan; // max length of buffer in seconds

    std::map<int64_t, cv::Mat> tx_buffer_map; // all the transmitted images
    std::deque<int64_t> tx_buffer_utimes;  // transmitted image utimes

    std::deque<std::pair<int64_t, std::vector<cv::Point> > > segment_pts_queue;
    
    bool needs_update;
    std::vector<cv::Point> fg_pts;

    // Object Segmenter
    ObjectSegmenter segmenter; 

    // utimes for image
    int64_t img_utime;

    int counter;

    void init_hsv_histogram() { 
    }

    state_t () {
        // LCM, BotFrames, BotParam inits        
        lcm =  bot_lcm_get_global(NULL);
        param = bot_param_new_from_server(lcm, 1);
        frames = bot_frames_get_global (lcm, param);

        img_mutex = PTHREAD_MUTEX_INITIALIZER;
        buffer_mutex = PTHREAD_MUTEX_INITIALIZER;

        imgutils_ = NULL;
        
        needs_update = true;
        buffer_timespan = 5 * 1e6; // 5 seconds (50 imgs, 6 tx imgs; check)

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
    if (ch != 3) { 
        std::cerr << " ERROR: REQUIRE COLOR IMAGES. GrayScale Image received!" << std::endl;
        return;
    }
    assert(ch == 3);
    if (img.empty() || img.rows != msg->height || img.cols != msg->width)
        img.create(msg->height, msg->width, CV_8UC3);
        
    // extract image data
    switch (msg->pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
        memcpy(img.data, msg->data, sizeof(uint8_t) * msg->width * msg->height * 3);
        cv::cvtColor(img, img, CV_RGB2BGR);
        break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
        // for some reason msg->row_stride is 0, so we use msg->width instead.
        jpeg_decompress_8u_rgb(msg->data,
                               msg->size,
                               img.data,
                               msg->width,
                               msg->height,
                               msg->width * ch);
            cv::cvtColor(img, img, CV_RGB2BGR);
        break;
    default:
        fprintf(stderr, "Unrecognized image format\n");
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




static void on_segment (const lcm_recv_buf_t *rbuf, const char *channel,
                            const drc_point_list_t *msg, 
                            void *user_data ) {
    if (!msg->n_points || state->img.empty()) return;

    std::cerr << " ====> Received " << int(msg->n_points) << " annotations" << std::endl;
    std::cerr << " ====> looking for " << msg->utime << std::endl;
    // Clear fg_pts
    state->fg_pts.clear();

    for (int j=0; j<msg->n_points; j++) {  
        int cx = (int)msg->uv[j][0]; 
        int cy = (int)msg->uv[j][1]; 
        std::cerr << "pts: " << cx << "," << cy << std::endl;
        state->fg_pts.push_back(cv::Point(cx,cy));
    }

    pthread_mutex_lock(&state->buffer_mutex);

    // Push to front of queue
    state->segment_pts_queue.push_front(std::make_pair(msg->utime, state->fg_pts));
    
    pthread_mutex_unlock(&state->buffer_mutex);

    return;
}

//--------------------------------------------
// Callbacks
//--------------------------------------------
static void on_image_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const bot_core_image_t *msg, 
                            void *user_data ) {
    if (options.vDEBUG)
        std::cerr << "\n\non_image_frame " << std::endl;

    if (!msg->width || !msg->height) return;

    // Main on_image
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


    //--------------------------------------------
    // Handle buffer
    //--------------------------------------------
    pthread_mutex_lock(&state->buffer_mutex);
    // UPDATE: Both buffer_utimes and buffer_map
    // On new image, add to buffer_utimes

    if (state->buffer_map.find(msg->utime) == state->buffer_map.end()) { 
        state->buffer_utimes.push_front(msg->utime);
        state->buffer_map[msg->utime] = state->img.clone();
    } else { 
        std::cerr << "ALREADY IN MAP; SKIPPING" << std::endl;
    }
    
    // Check if state->buffer_queue is within max limit
    if (state->buffer_utimes.size()) { 
        // Remove last elements if old
        while (state->buffer_utimes.size()) { 
            int64_t oldest = state->buffer_utimes.back();
            if (msg->utime - oldest > state->buffer_timespan) { 
                state->buffer_utimes.pop_back();
                state->buffer_map.erase(oldest);
            } else 
                break;           
        }
    }

    // Update: tx_buffer
    // Remove stale images in tx_buffer
    std::vector<int64_t> erase_utimes;
    for (std::map<int64_t,cv::Mat>::iterator it = state->tx_buffer_map.begin(); 
         it != state->tx_buffer_map.end(); it++) { 
        int64_t utime = it->first;
        if (msg->utime - utime > state->buffer_timespan) { 
            erase_utimes.push_back(utime);
        }
    }
    // Remove buffers from tx
    for (int j=0; j<erase_utimes.size(); j++) 
        state->tx_buffer_map.erase(erase_utimes[j]);

    // Update: tx_utimes
    // Check if state->tx_buffer_queue is within max limit
    if (state->tx_buffer_utimes.size()) { 

        // Remove last elements if old
        while (state->tx_buffer_utimes.size()) { 
            if (msg->utime - state->tx_buffer_utimes.back() > state->buffer_timespan)
                state->tx_buffer_utimes.pop_back();
            else 
                break;
        }
    }
    
    // Add to tx_buffer_map
    for (int j=0; j<state->tx_buffer_utimes.size(); j++) { 
        int64_t utime = state->tx_buffer_utimes[j];
        // If in buffer but not in tx_buffer
        if (state->buffer_map.find(utime) != state->buffer_map.end() && 
            state->tx_buffer_map.find(utime) == state->tx_buffer_map.end()) { 
            state->tx_buffer_map[utime] = state->buffer_map[utime];
        }
    }

    // Find the corresponding img with the msg->utime
    for (int j=0; j<state->segment_pts_queue.size(); j++) { 
        int64_t utime = state->segment_pts_queue[j].first;
        if (state->tx_buffer_map.find(utime) != state->tx_buffer_map.end()) {

            cv::Mat img = state->tx_buffer_map[utime];
            std::cerr << "************************ FOUND IMAGE " << utime << "************************" << std::endl;

            // Initialize segmenter
            state->segmenter.initialize(img, state->segment_pts_queue[j].second);

            state->segment_pts_queue.clear();
            std::cerr << "Removing rest of the annotations" << std::endl;
        }  else { 
            std::cerr << "######################## FAILED TO FIND IMAGE " << utime << "########################" << std::endl;
        }
    }

    if (options.vDEBUG) { 
        if (state->buffer_utimes.size()) 
            std::cerr << "img: st: " << state->buffer_utimes.front() << " " << state->buffer_utimes.back() << std::endl;

        std::cerr << "buffer_map sz: " << state->buffer_map.size() << std::endl;
        std::cerr << "buffer_utimes sz: " << state->buffer_utimes.size() << std::endl;

        std::cerr << "tx_buffer_map sz: " << state->tx_buffer_map.size() << std::endl;
        std::cerr << "tx_buffer_utimes sz: " << state->tx_buffer_utimes.size() << std::endl;

        for (std::map<int64_t, cv::Mat>::iterator it = state->tx_buffer_map.begin(); 
             it != state->tx_buffer_map.end(); it++) { 
            std::cerr << "tx: " << it->first << std::endl;
        }
    }

    pthread_mutex_unlock(&state->buffer_mutex);


    std::map<int64_t, cv::Mat> buffer_map; // all the local images
    std::deque<int64_t> buffer_utimes; // all the local image utimes
    int64_t buffer_timespan; // max length of buffer in seconds

    std::map<int64_t, cv::Mat> tx_buffer_map; // all the transmitted images
    std::deque<int64_t> tx_buffer_utimes;  // transmitted image utimes


    //--------------------------------------------
    // Main tracking
    //--------------------------------------------

    // Start clock
    double st = bot_timestamp_now();

    // Blur Image
    cv::Mat img; 
    cv::blur(state->img, img, cv::Size(5,5));

    // Return if no seeds
    if (!state->segmenter.initialized)
        return;
    std::cerr << "Updating ===> " << std::endl;

    // Segment image
    double tic = bot_timestamp_now(); 
    cv::Mat1b object_mask = state->segmenter.track(img); 
    printf("===> TRACK: %4.2f ms\n", (bot_timestamp_now() - st) * 1e-3);
    // Debug mask 
    if (options.vDEBUG) { 
        cv::Mat dst = Mat::zeros(img.size(), CV_8UC3);
        cvtColor(object_mask, dst, CV_GRAY2BGR);
        // dst *= Vec3b(0,1,1);
        cv::imshow("Segmentation", dst * 0.6 + state->img * 0.4);
    }
     
    // Send mask image
    if (!state->imgutils_) { 
        state->imgutils_ = new image_io_utils( state->lcm, object_mask.cols, object_mask.rows );
    } 

    if (options.vDEBUG) 
        state->imgutils_->sendImage(object_mask.data, state->img_utime, object_mask.cols, 
                                    object_mask.rows, 1, string(options.vCHANNEL + "_MASK_TEST"));
    state->imgutils_->sendImageZipped(object_mask.data, state->img_utime, object_mask.cols, 
                                      object_mask.rows, 1, string(options.vCHANNEL + "_MASKZIPPED_TEST"));



    std::cerr << "Done updating" << std::endl;
    state->needs_update = false;

    return;
}

static void on_tx_image_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const bot_core_image_t *msg, 
                            void *user_data ) {

    pthread_mutex_lock(&state->buffer_mutex);

    std::set<int64_t> tx_set(state->tx_buffer_utimes.begin(), state->tx_buffer_utimes.end());
    if (tx_set.find(msg->utime) == tx_set.end()) {     
        // On new tx_image, add to tx_buffer_utimes
        state->tx_buffer_utimes.push_front(msg->utime);
    } else { 
        std::cerr << "ALREADY IN TX MAP; SKIPPING" << std::endl;
    }        


    if (options.vDEBUG) {
        if (state->tx_buffer_utimes.size()) 
            std::cerr << "tx: st: " << state->tx_buffer_utimes.front() << " " << state->tx_buffer_utimes.back() << std::endl;

        std::cerr << "on-tx  buffer_map sz: " << state->buffer_map.size() << std::endl;
        std::cerr << "on-tx  buffer_utimes sz: " << state->buffer_utimes.size() << std::endl;

        std::cerr << "on-tx  tx_buffer_map sz: " << state->tx_buffer_map.size() << std::endl;
        std::cerr << "on-tx  tx_buffer_utimes sz: " << state->tx_buffer_utimes.size() << std::endl;
    }

    pthread_mutex_unlock(&state->buffer_mutex);
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
    opt.add(options.vSCALE, "s", "scale","Scale Factor");
    opt.parse();
  
    std::cerr << "===========  Object Segmenter ============" << std::endl;
    std::cerr << "=> CAMERA CHANNEL : " << options.vCHANNEL << std::endl;
    std::cerr << "=> DEBUG : " << options.vDEBUG << std::endl;
    std::cerr << "=> SCALE : " << options.vSCALE << std::endl;
    std::cerr << "=> Note: Hit 'c' to capture mask" << std::endl;
    std::cerr << "===============================================" << std::endl;
  
    // Install signal handler to free data.
    signal(SIGINT, INThandler);

    // Param server, botframes
    state = new state_t();

    // Subscriptions
    bot_core_image_t_subscribe(state->lcm, options.vCHANNEL.c_str(), on_image_frame, (void*)state);
    bot_core_image_t_subscribe(state->lcm, std::string(options.vCHANNEL + "_TX").c_str(), on_tx_image_frame, (void*)state);
    drc_point_list_t_subscribe(state->lcm, "OBJECT_SEGMENTATION_ANNOTATION", on_segment, (void*)state);

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
        }
            

        // UI handling 
        if (options.vDEBUG) { 
            if (!state->img.empty()) { 
                cv::Mat display = state->img.clone();
                for (int j=0; j<state->fg_pts.size(); j++) {
                    cv::circle(display, state->fg_pts[j], 5, cv::Scalar(0,255,0), 1, CV_AA);
                }
                imshow(WINDOW_NAME, display);
            }
        }
    }

    return 0;
}



// void prepare_mask(cv::Mat& mask, std::vector<cv::Point>& pts, int val) { 
//     for (int j=0; j<pts.size(); j++) {
//         cv::Point tl(pts[j].x-SEG_SIZE/2, pts[j].y-SEG_SIZE/2);
//         cv::Point br(pts[j].x+SEG_SIZE/2, pts[j].y+SEG_SIZE/2);
//         cv::rectangle(mask, tl, br, cv::Scalar(val), -1, CV_AA);
//     }
//     return;
// }



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

    // // Threshold image based on 80% conf.
    // cv::Mat1b fgd_mask = (bp >= 0.5) * 255;

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
