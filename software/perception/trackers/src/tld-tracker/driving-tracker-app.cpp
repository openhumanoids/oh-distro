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

#include <lcmtypes/perception_image_roi_t.h>
#include <lcmtypes/perception_pointing_vector_t.h>

#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <ConciseArgs>

#include "tld-tracker.hpp"

#include <GL/gl.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <bot_lcmgl_client/lcmgl.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>
// #include <pthread.h>

// #define SHOW_ALL_RECTS_BY_ONE 0
// #define BUFFER_TIME_SIZE 3 // seconds

#define AFFORDANCE_OFFSET 64
using namespace cv;

struct DrivingTrackerOptions { 
    bool vDEBUG;
    float vSCALE;
    std::string vCHANNEL;

    DrivingTrackerOptions () : 
        vCHANNEL(std::string("CAMERALEFT")), vSCALE(0.5f), vDEBUG(false) {}
};
DrivingTrackerOptions options;

struct state_t { 
    lcm_t* lcm;
    bot_lcmgl_t* lcmgl;

    BotParam* param;
    BotFrames* frames;
    bot::frames frames_cpp;

    // Img
    cv::Mat img;

    // utimes for image
    int64_t img_utime;

    // Parameters for the camera
    CameraParams camera_params;

    // Main Tracker
    std::map<int64_t, TLDTracker*> tracker_map;
    
    int counter;

    std::vector<cv::Scalar> id_colors; 

    state_t (const DrivingTrackerOptions& _options) {
        // LCM, BotFrames, BotParam inits
        lcm =  bot_lcm_get_global(NULL);
        lcmgl = NULL;

        param = bot_param_new_from_server(lcm, 1);
        frames = bot_frames_get_global (lcm, param);

        // Camera Params
        camera_params = CameraParams(param, "cameras." + _options.vCHANNEL + ".intrinsic_cal");

        // Initialize TLD tracker
        // tracker = new TLDTracker(camera_params.width, camera_params.height, _options.vSCALE);
        for (int j=0; j<4; j++) { 
            cv::RNG rng(id_colors.size());
            id_colors.push_back(cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)));
        }

        // Counter for debug prints
        counter = 0; 
    }

    ~state_t () { 
        lcm_destroy(lcm);
        // delete tracker;
        for (std::map<int64_t, TLDTracker*>::iterator it = tracker_map.begin(); 
             it != tracker_map.end(); it++) 
            delete it->second;
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
static void on_segmentation_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const perception_image_roi_t *msg, 
                            void *user_data ) {

    std::cerr << "OBJECT: " << msg->object_id 
              << " SEGMENTATION msg: " << msg->utime << " " << msg->roi.x << " " << msg->roi.y 
	      << " " << msg->roi.width << " " << msg->roi.height << std::endl;

    state_t* state = (state_t*) user_data; 
    Rect selection(msg->roi.x, msg->roi.y, msg->roi.width, msg->roi.height);

    // Initialize tracker for object_id
    TLDTracker* tr = new TLDTracker(state->camera_params.width, state->camera_params.height, options.vSCALE);

    // Save tracker to tracker_map
    state->tracker_map[msg->object_id] = tr;        

    // Ensure tracker is initialized
    if (!tr) { 
      std::cerr << "Tracker Not Initialized!" << std::endl;
      assert(0);
    }

    // Initialize with image and mask
    tr->initialize(state->img, selection);

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

    // Bot trans for rigid-body transform from head to local frame
    std::vector< Eigen::Vector3d > pts;
    Eigen::Isometry3d cam_to_frame;
    state->frames_cpp.get_trans_with_utime(state->frames, "CAMERA", "local", msg->utime, cam_to_frame); 

    // LCM gl draw bearing vector
    if (!state->lcmgl)
        state->lcmgl = bot_lcmgl_init(state->lcm,"object-bearing-vector");

    if(state->lcmgl){
        // bot_lcmgl_point_size(state->lcmgl, 4);
        bot_lcmgl_line_width(state->lcmgl, 2);
        bot_lcmgl_begin(state->lcmgl, GL_LINES);
    }

    
    // Main debug overlay
    cv::Mat display = state->img.clone();

    // For each object detected
    int idx = 0;
    for (std::map<int64_t, TLDTracker*>::iterator it = state->tracker_map.begin(); 
         it != state->tracker_map.end(); it++) { 
        TLDTracker* tr = it->second;

        double tic = bot_timestamp_now(); 
        if (!tr) return;

        if (tr->initialized) 
            tr->update(state->img, pts, cam_to_frame);

        if (++state->counter == 10) { 
            printf("===> TLD TRACKER: %4.2f ms\n", (bot_timestamp_now() - tic) * 1.f * 1e-3); 
            state->counter = 0;
        }

        // Update bearing vector
        if (tr->detection_valid) { 
            const cv::Rect& currBB = tr->currBB;
            std::cerr << "TLD currBB: " << currBB.tl() << " " << currBB.br() << std::endl;
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

            Eigen::Vector4d p0 = cam_to_frame * Eigen::Vector4d(0, 0, 0, 1);
            Eigen::Vector4d p1 = cam_to_frame * Eigen::Vector4d(a, b, 1, 1);

            if(state->lcmgl){
                float b = state->id_colors[idx][0] * 1.f / 255.f;
                float g = state->id_colors[idx][1] * 1.f / 255.f;
                float r = state->id_colors[idx][2] * 1.f / 255.f;

                bot_lcmgl_color3f(state->lcmgl, r, g, b);
                bot_lcmgl_vertex3f(state->lcmgl, p0[0], p0[1], p0[2]);
                bot_lcmgl_vertex3f(state->lcmgl, p1[0], p1[1], p1[2]);
            }

            perception_pointing_vector_t_publish(state->lcm, "OBJECT_BEARING", &bearing_vec);
        }

        // Display overlay
        if (tr->initialized)
            tr->addOverlay(display, idx, state->id_colors[idx]); idx++;
    }
    cv::imshow("Driving Tracker", display);

    // Finish up drawing
    if(state->lcmgl){
        bot_lcmgl_end(state->lcmgl);
        bot_lcmgl_switch_buffer(state->lcmgl);
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
    cout << "drc-driving-tracker -s 0.5 -c CAMERALEFT\n";
    cout << "Note: Make sure drc-tld-segmenter is providing ROI segmentation\n";
    cout << "=============================================\n";

    ConciseArgs opt(argc, (char**)argv);
    opt.add(options.vCHANNEL, "c", "camera-channel","Camera Channel [CAMERALEFT]");
    opt.add(options.vSCALE, "s", "scale","Scale Factor");
    opt.add(options.vDEBUG, "d", "debug","Debug mode");
    opt.parse();

    std::cerr << "===========  TLD Driving Tracker ============" << std::endl;
    std::cerr << "=> CAMERA CHANNEL : " << options.vCHANNEL << std::endl;
    std::cerr << "=> SCALE : " << options.vSCALE << std::endl;
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
    perception_image_roi_t_subscribe(state->lcm, "TLD_OBJECT_ROI", on_segmentation_frame, (void*)state);

    // Main lcm handle
    while(1) { 
        unsigned char c = cv::waitKey(1) & 0xff;
	lcm_handle(state->lcm);
        if (c == 'q') { 
            break;      
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

