/*
TLD's work well w/ features (keypts) low-level features

- DoT works well with textureless objects (contoures)
- Use both to bootstrap a contour and features based reconstruction 
- Does going to 3D from contour make tracking possible ??

*/
#define WINDOW_NAME "TLD Tracker"

#include <set>

// LCM includes
#include <perception_opencv_utils/calib_utils.hpp>
#include <perception_opencv_utils/imshow_utils.hpp>
#include <perception_opencv_utils/math_utils.hpp>
#include <perception_opencv_utils/plot_utils.hpp>
#include <perception_opencv_utils/color_utils.hpp>
#include <perception_opencv_utils/imgproc_utils.hpp>

#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include <lcmtypes/perception_image_roi_t.h>
#include <lcmtypes/perception_pointing_vector_t.h>

#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <ConciseArgs>

#include "tld-tracker.hpp"
// #include <pthread.h>

// #define SHOW_ALL_RECTS_BY_ONE 0
// #define BUFFER_TIME_SIZE 3 // seconds

#define AFFORDANCE_OFFSET 64
using namespace cv;

struct TLDTrackerOptions { 
    bool vDEBUG;
    float vSCALE;
    int vAFFORDANCE_ID;
    std::string vCHANNEL, vAFFORDANCE_CHANNEL;

    TLDTrackerOptions () : 
        vCHANNEL(std::string("CAMERALEFT")), vAFFORDANCE_CHANNEL(std::string("CAMERALEFT_MASKZIPPED")), 
        vSCALE(1.f), vAFFORDANCE_ID(64), vDEBUG(false) {}
};
TLDTrackerOptions options;

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

    // Parameters for the camera
    CameraParams camera_params;

    // Main Tracker
    TLDTracker* tracker;
    
    int counter;

    state_t (const TLDTrackerOptions& _options) {
        // LCM, BotFrames, BotParam inits
        lcm =  bot_lcm_get_global(NULL);
        param = bot_param_new_from_server(lcm, 1);
        frames = bot_frames_get_global (lcm, param);

        // Camera Params
        camera_params = CameraParams(param, "cameras." + _options.vCHANNEL + ".intrinsic_cal");

        // Initialize image IO utils
        imgutils_aff = new image_io_utils( lcm, camera_params.width, camera_params.height);

        // Initialize TLD tracker
        tracker = new TLDTracker(camera_params.width, camera_params.height, _options.vSCALE);

        // Counter for debug prints
        counter = 0; 
    }

    ~state_t () { 
        delete imgutils_aff;
        // delete tracker;
        lcm_destroy(lcm);
    }
};
state_t* state = NULL; 

void
decode_image(const bot_core_image_t * msg, cv::Mat& img)
{
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
    return;
}


static void on_segmentation_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const perception_image_roi_t *msg, 
                            void *user_data ) {

    std::cerr << "SEGMENTATION msg: " << msg->utime << " " << msg->roi.x << " " << msg->roi.y 
	      << " " << msg->roi.width << " " << msg->roi.height << std::endl;

    state_t* state = (state_t*) user_data; 
    float sw = state->camera_params.width, sh = state->camera_params.height;
    Rect selection(msg->roi.x * sw, msg->roi.y * sh, msg->roi.width * sw, msg->roi.height * sh);
    state->aff_utime = msg->utime; 

    if (msg->feature_id < 0) {
        std::cerr << "INVALID feature_id" << std::endl;
        return;
    }
        

    // Ensure tracker is initialized
    if (!state->tracker) { 
      std::cerr << "Tracker Not Initialized!" << std::endl;
      assert(0);
    }

    // Initialize with image and mask
    state->tracker->initialize(state->img, selection);

    if (options.vDEBUG) { 
        cv::Mat imgd = state->img.clone();
        rectangle(imgd, selection.tl(), selection.br(), CV_RGB(255, 0, 0), 2); 
        // cv::imshow("Captured Image", state->img);
        cv::imshow("Captured Image ROI", imgd);
    }

}


static void on_image_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const bot_core_image_t *msg, 
                            void *user_data ) {

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


    double tic = bot_timestamp_now(); 

    // Update step
    std::vector< Eigen::Vector3d > pts;
    Eigen::Isometry3d local_to_camera;

    if (state->tracker->initialized) 
        state->tracker->update(state->img, pts, local_to_camera);

    if (++state->counter == 10) { 
        printf("===> TLD TRACKER: %4.2f ms\n", (bot_timestamp_now() - tic) * 1.f * 1e-3); 
        state->counter = 0;
    }

    // Update bearing vector
    if (state->tracker->detection_valid) { 
        const cv::Rect& currBB = state->tracker->currBB;
        float roix = currBB.x + currBB.width/2, roiy = currBB.y + currBB.height/2;
        float a,b;
        float u = -(state->camera_params.cx - roix);
        float v = -(state->camera_params.cy - roiy);
        if (!state->camera_params.fx && !state->camera_params.fy)
            a = b = 0;
        else 
            a = u*1.f/state->camera_params.fx, b = v*1.f/state->camera_params.fy;
        perception_pointing_vector_t bearing_vec;
        bearing_vec.utime = state->img_utime;
        bearing_vec.pos[0] = 0, 
            bearing_vec.pos[1] = 0, 
            bearing_vec.pos[2] = 0;

        bearing_vec.vec[0] = a, 
            bearing_vec.vec[1] = b, 
            bearing_vec.vec[2] = 1;
        perception_pointing_vector_t_publish(state->lcm, "OBJECT_BEARING", &bearing_vec);
    }

    // Viz
    // cv::Mat display = state->img.clone();
    if (state->tracker->detection_valid) { 
        const cv::Rect& currBB = state->tracker->currBB;
        std::cerr << "TLD currBB: " << currBB.tl() << " " << currBB.br() << std::endl;
    }

    // Viz if not initialized
    if (!state->tracker->initialized && !state->aff_img.empty() && !state->img.empty()) { 
        cv::Mat maskd; cvtColor(state->aff_img, maskd, CV_GRAY2BGR);
        addWeighted(maskd, 0.7, state->img, 0.3, 0, maskd);
        cv::imshow("TLD", maskd);
    }
        
    return;
}

void  INThandler(int sig) {
    printf("Exiting\n");
    if (state) delete state;
    exit(0);
}


int main(int argc, char** argv)
{
    cout << "============= QUICK MODES ===================\n";
    cout << "drc-tld-tracker -s 0.25 -c CAMERALEFT\n";
    cout << "=============================================\n";

    ConciseArgs opt(argc, (char**)argv);
    opt.add(options.vCHANNEL, "c", "camera-channel","Camera Channel [CAMERALEFT]");
    opt.add(options.vAFFORDANCE_CHANNEL, "a", "affordance-channel","Affordance Channel [CAMERALEFT_MASKZIPPED]");
    opt.add(options.vSCALE, "s", "scale","Scale Factor");
    opt.add(options.vAFFORDANCE_ID, "i", "id","Affordance ID");
    opt.add(options.vDEBUG, "d", "debug","Debug mode");
    opt.parse();

    std::cerr << "===========  TLD Tracker ============" << std::endl;
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
    state = new state_t(options);

#if 0
    std::cerr << "reading from model" << std::endl;
    tldtracker->readFromFile("model");
    tldtracker->learningEnabled = false;
    trackObject = 1;
#endif

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
                cv::Mat maskd; cvtColor(mask.clone(), maskd, CV_GRAY2BGR);
                addWeighted(maskd, 0.5, state->img, 0.5, 0, maskd);
                cv::imshow("Captured Image mask", maskd);
            }
            // Initialize with image and mask
            state->tracker->initialize(state->img, mask);
        // } else if (c == 'l') { 
        //     tldtracker->learningEnabled = !tldtracker->learningEnabled;
        //     printf("LearningEnabled: %d\n", tldtracker->learningEnabled);
        // } else if (c == 'a') {
        //     tldtracker->alternating = !tldtracker->alternating;
        //     printf("alternating: %d\n", tldtracker->alternating);
        // } else if (c == 'r') { 
        //     std::cerr << "reading from model" << std::endl;
        //     tldtracker->readFromFile("model");
        //     tldtracker->learningEnabled = false;
        //     // htrackObject = 1;
        // } else if (c == 'w') { 
        //     std::cerr << "exiting: writing to model" << std::endl;
        //     tldtracker->writeToFile("model");
        }
    }

    if (state) delete state;
    return 0;
}


// =======================================================================================
// Queue Handling
// =======================================================================================
    // pthread_mutex_lock(&buffer_mutex);
    // bool found = false;
    // for (int j=0; j<cbuffer.size()-1; j++) { 
    //     if (cbuffer[j].first >= msg->utime && cbuffer[j+1].first < msg->utime) { 
    //         found = true;

    //         std::cerr << "Init TLD with " << cbuffer[j].first << std::endl;
	    
    //         Mat gray; 
    //         cvtColor(cbuffer[j].second, gray, CV_BGR2GRAY);

    //         // tld tracker selection
    //         Rect selection(msg->roi.x, msg->roi.y, msg->roi.width, msg->roi.height);
    //         tldtracker->selectObject(gray, &selection);
	    
    //         Mat img = cbuffer[j].second.clone();
    //         rectangle(img, selection, CV_RGB(255, 0, 0), 2); 
    //         opencv_utils::imshow("first frame", img);

    //         // Found frame: removing the older frames
    //         cbuffer.erase(cbuffer.begin() + j + 1, cbuffer.end());
    //         std::cerr << "Found image" << msg->utime << "=" << cbuffer[j].first << "~" << cbuffer[j+1].first << std::endl;
    //         break;
    //     }
    // }
    // pthread_mutex_unlock(&buffer_mutex);



    // if (!initDone) { 
    //     pthread_mutex_lock(&buffer_mutex);
    //     // Push to circular buffer
    //     cbuffer.push_front(std::make_pair(msg->utime, img));

    //     // Circular buffer implementation to allow for delay in user-segmentation
    //     if (cbuffer.size()) { 
    //         const std::pair<int64_t, Mat>& latest = cbuffer.front();

    //         // Keep only BUFFER_TIME_SIZE seconds of buffer
    //         int jtime;
    //         for (jtime=0; jtime<cbuffer.size(); jtime++) { 
    //             if (latest.first - cbuffer[jtime].first > BUFFER_TIME_SIZE * 1e6)
    //                 break;
    //         }
    //         if (jtime < cbuffer.size()) { 
    //             // std::cerr << "Clearing up " << cbuffer.size() - jtime << std::endl;
    //             cbuffer.erase(cbuffer.begin() + jtime, cbuffer.end());
    //         }
    //     }
    //     pthread_mutex_unlock(&buffer_mutex);
    // } else {

    //     pthread_mutex_lock(&buffer_mutex);
    //     // Push to circular buffer
    //     cbuffer.push_front(std::make_pair(msg->utime, img));

    //     if (cbuffer.size() > 1) { 
    //         // Once the buffer is clean
    //         int j = 0, count = 2;
    //         while (j < count && cbuffer.size()) { 
    //             std::pair<int64_t, Mat>& bufpair = cbuffer.back();
    //             // Track each of the remaining buffers before going live
    //             tld_track(bufpair.second);
    // 		cbuffer.pop_back();
    // 		std::cerr << "Pending frames : " << cbuffer.size() << std::endl;
		
    //         }
    //     } else if (cbuffer.size() == 1) { 
    //         std::cerr << "Live tracking " << msg->utime << std::endl;

    //         std::pair<int64_t, Mat>& bufpair = cbuffer.back();
    //         tld_track(bufpair.second);

    //         if(tldtracker->currBB != NULL) {
    //             float roix = tldtracker->currBB->x + tldtracker->currBB->width/2, roiy = tldtracker->currBB->y + tldtracker->currBB->height/2;
    //             float a,b;
    //             float u = -(camera_params.cx - roix);
    //             float v = -(camera_params.cy - roiy);
    //             if (!camera_params.fx && !camera_params.fy)
    //                 a = b = 0;
    //             else 
    //                 a = u*1.f/camera_params.fx, b = v*1.f/camera_params.fy;
    //             perception_pointing_vector_t bearing_vec;
    //     	bearing_vec.utime = bufpair.first;
    //             bearing_vec.pos[0] = 0, 
    //                 bearing_vec.pos[1] = 0, 
    //                 bearing_vec.pos[2] = 0;

    //             bearing_vec.vec[0] = a, 
    //                 bearing_vec.vec[1] = b, 
    //                 bearing_vec.vec[2] = 1;
    //             perception_pointing_vector_t_publish(state->lcm, "OBJECT_BEARING", &bearing_vec);
    //         }
            

    //         cbuffer.pop_back();
    //     } else 
    //         std::cerr << "Empty buffer: " << cbuffer.size() << std::endl;
    //     pthread_mutex_unlock(&buffer_mutex);
        
    // }
    // std::cerr << "Buffer size: " << cbuffer.size() << std::endl;

    // opencv_utils::imshow(WINDOW_NAME, img);
