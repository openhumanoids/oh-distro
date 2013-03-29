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
#include "histogram-tracker.hpp"
#define AFFORDANCE_OFFSET 64


struct HistogramTrackerOptions { 
    bool vDEBUG;
    float vSCALE;
    int vAFFORDANCE_ID;
    std::string vCHANNEL, vAFFORDANCE_CHANNEL;

    HistogramTrackerOptions () : 
        vCHANNEL(std::string("CAMERALEFT")), vAFFORDANCE_CHANNEL(std::string("CAMERALEFT_MASKZIPPED")), 
        vSCALE(1.f), vAFFORDANCE_ID(64), vDEBUG(false) {}
};
HistogramTrackerOptions options;

struct state_t { 
    lcm_t* lcm;

    BotParam* param;
    BotFrames* frames;

    // image IO util for affordances
    image_io_utils* imgutils_aff;

    // Img, affordance image
    cv::Mat img, aff_img;

    // utimes for image, and affordance image
    int64_t img_utime, aff_utime;

    HistogramTracker* tracker;
    
    int counter;

    state_t () {
        // LCM, BotFrames, BotParam inits        
        lcm =  bot_lcm_get_global(NULL);
        param = bot_param_new_from_server(lcm, 1);
        frames = bot_frames_get_global (lcm, param);

        // Initialize image IO utils
        imgutils_aff = new image_io_utils( lcm, 1024, 544);
        tracker = new HistogramTracker();
        counter = 0; 
    }

    ~state_t () { 
        lcm_destroy(lcm);
        delete imgutils_aff;
        delete tracker;
    }
};
state_t* state = NULL; 

void
decode_image(const bot_core_image_t * msg, cv::Mat& img)
{
    if (img.empty() || img.rows != msg->height || img.cols != msg->width)
        img.create(msg->height, msg->width, CV_8UC3);

  // extract image data
  // TODO add support for raw RGB
  switch (msg->pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
        memcpy(img.data, msg->data, sizeof(uint8_t) * msg->width * msg->height * 3);
        cv::cvtColor(img, img, CV_RGB2BGR);
      break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
      // for some reason msg->row_stride is 0, so we use msg->width instead.
      jpeg_decompress_8u_gray(msg->data,
                              msg->size,
                              img.data,
                              msg->width,
                              msg->height,
                              msg->width);
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

//--------------------------------------------
// Callbacks
//--------------------------------------------
static void on_affordance_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const bot_core_image_t *msg, 
                            void *user_data ) {

    // std::cerr << "on_affordance_frame" << std::endl;
    if (!msg->width || !msg->height) return;

    state_t* state = (state_t*) user_data; 

    // Data lies 
    uint8_t* data = (uint8_t*) state->imgutils_aff->unzipImage(msg);
    state->aff_img = cv::Mat( msg->height , msg->width , CV_8UC1, data);
    state->aff_utime = msg->utime; 

    if (options.vDEBUG)
        imshow("Affordance Image", state->aff_img);
    return;
}

static void on_segmentation_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const perception_image_roi_t *msg, 
                            void *user_data ) {
    
    std::cerr << "SEGMENTATION msg: " << msg->utime << " " << msg->roi.x << " " << msg->roi.y 
	      << " " << msg->roi.width << " " << msg->roi.height << std::endl;

    state_t* state = (state_t*) user_data; 
    Rect selection(msg->roi.x, msg->roi.y, msg->roi.width, msg->roi.height);
    cv::Mat1b mask = cv::Mat1b::zeros( state->img.rows , state->img.cols);

    // Assign to mask
    for (int y=msg->roi.y; y<msg->roi.y+msg->roi.height; y++)  
      for (int x=msg->roi.x; x<msg->roi.x+msg->roi.width; x++) 
	mask(y,x) = 255;

    state->aff_utime = msg->utime; 

    // Ensure tracker is initialized    
    if (!state->tracker) { 
      std::cerr << "Tracker Not Initialized!" << std::endl;
      assert(0);
    }

    // Initialize with image and mask
    state->tracker->initialize(state->img, mask);

    if (options.vDEBUG) { 
        cv::Mat maskd = mask.clone();
        cv::imshow("Captured Image", state->img);
        cv::imshow("Captured Mask", maskd);
    }

    return;
}


static void on_image_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const bot_core_image_t *msg, 
                            void *user_data ) {
    // std::cerr << "on_image_frame " << std::endl;
    if (!msg->width || !msg->height) return;
    
    state_t* state = (state_t*) user_data; 
    if (!state->tracker) return;

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
    state->img_utime = msg->utime; 

    cv::Mat display = state->img.clone();

    double tic = bot_timestamp_now(); 
    
    // mfallon added
    std::vector< Eigen::Vector3d > pts;
    Eigen::Isometry3d local_to_camera;
    
    state->tracker->update(display, options.vSCALE, pts, local_to_camera);
    if (++state->counter == 10) { 
        printf("===> HISTOGRAM BACKPROJECTION: %4.2f ms\n", (bot_timestamp_now() - tic) * 1e-3); 
        state->counter = 0;
    }

    cv::imshow("Camera", display);
    return;
}

void  INThandler(int sig) {
    printf("Exiting\n");
    exit(0);
}


int main(int argc, char ** argv) {
    cout << "============= QUICK MODES ===================\n";
    cout << "drc-histogram-tracker -s 0.25 -c CAMERALEFT\n";
    cout << "=============================================\n";

    ConciseArgs opt(argc, (char**)argv);
    opt.add(options.vCHANNEL, "c", "camera-channel","Camera Channel [CAMERALEFT]");
    opt.add(options.vAFFORDANCE_CHANNEL, "a", "affordance-channel","Affordance Channel [CAMERALEFT_MASKZIPPED]");
    opt.add(options.vSCALE, "s", "scale","Scale Factor");
    opt.add(options.vAFFORDANCE_ID, "i", "id","Affordance ID");
    opt.add(options.vDEBUG, "d", "debug","Debug mode");
    opt.parse();
  
    std::cerr << "===========  Histogram Tracker ============" << std::endl;
    std::cerr << "=> CAMERA CHANNEL : " << options.vCHANNEL << std::endl;
    std::cerr << "=> AFFORDANCE CHANNEL : " << options.vAFFORDANCE_CHANNEL << std::endl;
    std::cerr << "=> SCALE : " << options.vSCALE << std::endl;
    std::cerr << "=> AFFORDANCE ID : " << options.vAFFORDANCE_ID << std::endl;
    std::cerr << "=> DEBUG : " << options.vDEBUG << std::endl;
    std::cerr << "=> Note: Hit 'c' to capture mask" << std::endl;
    std::cerr << "===============================================" << std::endl;
  
    // Install signal handler to free data.
    signal(SIGINT, INThandler);

    // Param server, botframes
    state = new state_t();

    // Subscriptions
    bot_core_image_t_subscribe(state->lcm, options.vCHANNEL.c_str(), on_image_frame, (void*)state);
    bot_core_image_t_subscribe(state->lcm, options.vAFFORDANCE_CHANNEL.c_str(), on_affordance_frame, (void*)state);
    perception_image_roi_t_subscribe(state->lcm, "TLD_OBJECT_ROI", on_segmentation_frame, (void*)state);

    // Main lcm handle
    while(1) { 
        unsigned char c = cv::waitKey(1) & 0xff;
        lcm_handle(state->lcm);
        if (c == 'q') {
            break;  
        } else if ( c == 'c' ) { 
            if (state->aff_img.empty()) { 
                std::cerr << "AFFORDANCE IMAGE UNAVAILABLE" << std::endl;
                continue;
            }
            std::cerr << "Capturing Affordance : " << options.vAFFORDANCE_ID << std::endl;
            cv::Mat1b mask = (state->aff_img == AFFORDANCE_OFFSET + options.vAFFORDANCE_ID);
            if (!state->tracker) { 
                std::cerr << "Tracker Not Initialized!" << std::endl;
                assert(0);
            }
            if (options.vDEBUG) { 
                cv::Mat1b maskd = mask.clone();
                cv::imshow("Captured Image", state->img);
                cv::imshow("Captured Mask", maskd);
            }
            // Initialize with image and mask
            state->tracker->initialize(state->img, mask);
        }
    }

    return 0;
}
