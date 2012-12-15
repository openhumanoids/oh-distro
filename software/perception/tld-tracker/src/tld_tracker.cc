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

#include "kinect_opencv_utils.h"
#include <lcmtypes/kinect_image_msg_t.h>

#include <lcmtypes/kinect_frame_msg_t.h>
#include <kinect/kinect-utils.h>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include "tld/TLD.h"
#include <lcmtypes/perception_image_roi_t.h>
#include <lcmtypes/perception_pointing_vector_t.h>
#include <pthread.h>

// #include <pcl/registration/correspondence_rejection_sample_consensus.h>

#define SHOW_ALL_RECTS_BY_ONE 0
#define BUFFER_TIME_SIZE 3 // seconds

using namespace cv;

typedef struct _state_t state_t;
struct _state_t {
    lcm_t *lcm;
    GMainLoop *mainloop;
    BotParam *param;
    BotFrames *frames;
};
state_t * state = NULL;

struct CameraParams { 
    int width, height;
    float fx, fy, cx, cy, k1, k2, k3, p1, p2;
};
CameraParams camera_params;

bool initLearning = false;
bool initDone = false;
tld::TLD* tldtracker;

pthread_mutex_t buffer_mutex;
std::deque<std::pair<int64_t, Mat> > cbuffer;

static void usage(const char* progname)
{
  fprintf (stderr, "Usage: %s [options]\n"
                   "\n"
                   "Options:\n"
                   "  -l URL    Specify LCM URL\n"
                   "  -h        This help message\n", 
                   g_path_get_basename(progname));
  exit(1);
}

void tld_track(Mat& frame) { 
    Mat img = frame.clone();

    Mat gray; 
    cvtColor(img, gray, CV_BGR2GRAY);

    IplImage img_ipl(img);
    tldtracker->processImage(&img_ipl);
    int confident = (tldtracker->currConf >= .5) ? 1 : 0;

    CvScalar yellow = CV_RGB(255,255,0);
    CvScalar blue = CV_RGB(0,0,255);
    CvScalar black = CV_RGB(0,0,0);
    CvScalar white = CV_RGB(255,255,255);
        
    if(tldtracker->currBB != NULL) {
        CvScalar rectangleColor = (confident) ? blue : yellow;
        cvRectangle(&img_ipl, tldtracker->currBB->tl(), tldtracker->currBB->br(), rectangleColor, 2, 2, 0);
                
        // plot the tracker bounding box that optical flow predicts
        Rect* trackerBB = tldtracker->medianFlowTracker->trackerBB;
        if (trackerBB != NULL)
            cvRectangle(&img_ipl, trackerBB->tl(), trackerBB->br(), yellow, 2, 2, 0);
                
        opencv_utils::imshow("result", img);
    }
}


int envoy_decompress_bot_core_image_t(const bot_core_image_t * jpeg_image, bot_core_image_t *uncomp_image)
{
  //copy metadata
  uncomp_image->utime = jpeg_image->utime;
  if (uncomp_image->metadata != NULL)
    bot_core_image_metadata_t_destroy(uncomp_image->metadata);
  uncomp_image->nmetadata = jpeg_image->nmetadata;
  if (jpeg_image->nmetadata > 0) {
    uncomp_image->metadata = bot_core_image_metadata_t_copy(jpeg_image->metadata);
  }
  
  uncomp_image->width = jpeg_image->width;
  uncomp_image->height = jpeg_image->height;
  
  uncomp_image->row_stride = jpeg_image->width * 3;//jpeg_image->row_stride;

  int depth=0;
  if (uncomp_image->width>0)
      depth= uncomp_image->row_stride / (double)uncomp_image->width;

  int ret;

  if(uncomp_image->data == NULL){
      int uncomp_size = (uncomp_image->width) * (uncomp_image->height) * depth;
      uncomp_image->data = (unsigned char *) realloc(uncomp_image->data, uncomp_size * sizeof(uint8_t));
  }

  ret = jpeg_decompress_8u_rgb(jpeg_image->data, 
                               jpeg_image->size, uncomp_image->data, 
                               uncomp_image->width, 
                               uncomp_image->height, 
                               uncomp_image->row_stride);
  
  uncomp_image->row_stride = uncomp_image->width * depth;
  uncomp_image->size = uncomp_image->width *uncomp_image->height* depth;
  if (depth == 1) {
    uncomp_image->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
  }
  else if (depth == 3) {
    uncomp_image->pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB; //TODO: lets just assume its rgb... dunno what else to do :-/
  }
  return ret;
}

void  INThandler(int sig) {
    printf("Exiting\n");
}


static void on_segmentation_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const perception_image_roi_t *msg, 
                            void *user_data ) {
    
    std::cerr << "SEGMENTATION msg: " << msg->utime << " " << msg->roi.x << " " << msg->roi.y << 
        msg->roi.width << " " << msg->roi.height << std::endl;

    pthread_mutex_lock(&buffer_mutex);
    bool found = false;
    for (int j=0; j<cbuffer.size()-1; j++) { 
        if (cbuffer[j].first >= msg->utime && cbuffer[j+1].first < msg->utime) { 
            found = true;

            std::cerr << "Init TLD with " << cbuffer[j].first << std::endl;
            opencv_utils::imshow("first frame", cbuffer[j].second);

            Mat gray; 
            cvtColor(cbuffer[j].second, gray, CV_BGR2GRAY);

            // tld tracker selection
            Rect selection(msg->roi.x, msg->roi.y, msg->roi.width, msg->roi.height);
            tldtracker->selectObject(gray, &selection);

            // Found frame: removing the older frames
            cbuffer.erase(cbuffer.begin() + j + 1, cbuffer.end());
            std::cerr << "Found image" << msg->utime << "=" << cbuffer[j].first << "~" << cbuffer[j+1].first << std::endl;
            break;
        }
    }
    pthread_mutex_unlock(&buffer_mutex);

    initDone = true;

}



static void on_image_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const bot_core_image_t *msg, 
                            void *user_data ) {

    if (!msg->width || !msg->height)
        return;
        
    Mat img(msg->height, msg->width, CV_8UC3);
    static bot_core_image_t * localFrame=(bot_core_image_t *) calloc(1,sizeof(bot_core_image_t));
    if (msg->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG) { 
        //create space for decompressed image
        envoy_decompress_bot_core_image_t(msg,localFrame);
        memcpy( img.data, localFrame->data, sizeof(uint8_t)*msg->width*msg->height*3);            
    } else { 
        //uncompressed, just copy pointer
        memcpy(img.data, msg->data, sizeof(uint8_t)*msg->width*msg->height*3);            
    }
    cvtColor(img, img, CV_RGB2BGR);
    resize(img, img, Size(640,480));


    if (!initDone) { 
        pthread_mutex_lock(&buffer_mutex);
        // Push to circular buffer
        cbuffer.push_front(std::make_pair(msg->utime, img));

        // Circular buffer implementation to allow for delay in user-segmentation
        if (cbuffer.size()) { 
            const std::pair<int64_t, Mat>& latest = cbuffer.front();

            // Keep only BUFFER_TIME_SIZE seconds of buffer
            int jtime;
            for (jtime=0; jtime<cbuffer.size(); jtime++) { 
                if (latest.first - cbuffer[jtime].first > BUFFER_TIME_SIZE * 1e6)
                    break;
            }
            if (jtime < cbuffer.size()) { 
                // std::cerr << "Clearing up " << cbuffer.size() - jtime << std::endl;
                cbuffer.erase(cbuffer.begin() + jtime, cbuffer.end());
            }
        }
        pthread_mutex_unlock(&buffer_mutex);
    } else {

        pthread_mutex_lock(&buffer_mutex);
        // Push to circular buffer
        cbuffer.push_front(std::make_pair(msg->utime, img));

        if (cbuffer.size() > 1) { 
            // Once the buffer is clean
            int j = 0, count = 2;
            while (j < count && cbuffer.size()) { 
                std::pair<int64_t, Mat>& bufpair = cbuffer.back();
                // Track each of the remaining buffers before going live
                tld_track(bufpair.second);
                cbuffer.pop_back();
            }
        } else if (cbuffer.size() == 1) { 
            // std::cerr << "Live tracking " << msg->utime << std::endl;

            std::pair<int64_t, Mat>& bufpair = cbuffer.back();
            tld_track(bufpair.second);

            if(tldtracker->currBB != NULL) {
                float roix = tldtracker->currBB->x, roiy = tldtracker->currBB->y;
                float a,b;
                float u = -(camera_params.cx - roix);
                float v = -(camera_params.cy - roiy);
                if (!camera_params.fx && !camera_params.fy)
                    a = b = 0;
                else 
                    a = u*1.f/camera_params.fx, b = v*1.f/camera_params.fy;
                perception_pointing_vector_t bearing_vec;
                bearing_vec.pos[0] = 0, 
                    bearing_vec.pos[1] = 0, 
                    bearing_vec.pos[2] = 0;

                bearing_vec.vec[0] = a, 
                    bearing_vec.vec[1] = b, 
                    bearing_vec.vec[2] = 1;
                perception_pointing_vector_t_publish(state->lcm, "OBJECT_BEARING", &bearing_vec);
            }
            

            cbuffer.pop_back();
        } else 
            std::cerr << "Empty buffer: " << cbuffer.size() << std::endl;
        pthread_mutex_unlock(&buffer_mutex);
        
    }
    opencv_utils::imshow(WINDOW_NAME, img);
    return;
}


int main(int argc, char** argv)
{
    // g_thread_init(NULL);
    setlinebuf (stdout);
    state = (state_t*) calloc(1, sizeof(state_t));

    // Param server, botframes
    state->lcm =  bot_lcm_get_global(NULL);
    // state->mainloop = g_main_loop_new( NULL, FALSE );  
    state->param = bot_param_new_from_server(state->lcm, 1);
    state->frames = bot_frames_get_global (state->lcm, state->param);

    std::string key_prefix_str = "cameras.CAMERALEFT.intrinsic_cal";
    camera_params.width = bot_param_get_int_or_fail(state->param, (key_prefix_str+".width").c_str());
    camera_params.height = bot_param_get_int_or_fail(state->param,(key_prefix_str+".height").c_str());
    camera_params.fx = bot_param_get_double_or_fail(state->param, (key_prefix_str+".fx").c_str());
    camera_params.fy = bot_param_get_double_or_fail(state->param, (key_prefix_str+".fy").c_str());
    camera_params.cx = bot_param_get_double_or_fail(state->param, (key_prefix_str+".cx").c_str());
    camera_params.cy = bot_param_get_double_or_fail(state->param, (key_prefix_str+".cy").c_str());
    camera_params.k1 = bot_param_get_double_or_fail(state->param, (key_prefix_str+".k1").c_str());
    camera_params.k2 = bot_param_get_double_or_fail(state->param, (key_prefix_str+".k2").c_str());
    camera_params.k3 = bot_param_get_double_or_fail(state->param, (key_prefix_str+".k3").c_str());
    camera_params.p1 = bot_param_get_double_or_fail(state->param, (key_prefix_str+".p1").c_str());
    camera_params.p2 = bot_param_get_double_or_fail(state->param, (key_prefix_str+".p2").c_str());


    // New TLD Tracker
    tldtracker = new tld::TLD();

    tldtracker->detectorCascade->imgWidth = WIDTH;
    tldtracker->detectorCascade->imgHeight = HEIGHT;
    tldtracker->detectorCascade->imgWidthStep = WIDTH;

    tldtracker->alternating = false;
    tldtracker->learningEnabled = true;

    tld::DetectorCascade* detectorCascade = tldtracker->detectorCascade;
    detectorCascade->varianceFilter->enabled = true;
    detectorCascade->ensembleClassifier->enabled = true;
    detectorCascade->nnClassifier->enabled = true;

    // classifier
    detectorCascade->useShift = true; 
    detectorCascade->shift = 0.1;
    detectorCascade->minScale = -10;
    detectorCascade->maxScale = 10;
    detectorCascade->minSize = 25;
    detectorCascade->numTrees = 10;
    detectorCascade->numFeatures = 10;
    detectorCascade->nnClassifier->thetaTP = 0.65;
    detectorCascade->nnClassifier->thetaFP = 0.5;

#if 0
    std::cerr << "reading from model" << std::endl;
    tldtracker->readFromFile("model");
    tldtracker->learningEnabled = false;
    trackObject = 1;
#endif

    bot_core_image_t_subscribe(state->lcm, "CAMERALEFT", on_image_frame, (void*)state);
    perception_image_roi_t_subscribe(state->lcm, "TLD_OBJECT_ROI", on_segmentation_frame, (void*)state);

    // Install signal handler to free data.
    signal(SIGINT, INThandler);

    fprintf(stderr, "Starting Main Loop\n");

    int lcm_fd = lcm_get_fileno(state->lcm);
    fd_set fds;
    // wait a limited amount of time for an incoming message
    struct timeval timeout = { 
        0,  // seconds
        10000   // microseconds
    };

    while(1) { 
        unsigned char c = cv::waitKey(10) & 0xff;

        FD_ZERO(&fds);
        FD_SET(lcm_fd, &fds);

        // setup the LCM file descriptor for waiting.
        int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);
        if(status && FD_ISSET(lcm_fd, &fds)) {
            // LCM has events ready to be processed.
            lcm_handle(state->lcm);
        }

        if (c == 'q') { 
            break;      
        } else if (c == 'l') { 
            tldtracker->learningEnabled = !tldtracker->learningEnabled;
            printf("LearningEnabled: %d\n", tldtracker->learningEnabled);
        } else if (c == 'a') {
            tldtracker->alternating = !tldtracker->alternating;
            printf("alternating: %d\n", tldtracker->alternating);
        } else if (c == 'r') { 
            std::cerr << "reading from model" << std::endl;
            tldtracker->readFromFile("model");
            tldtracker->learningEnabled = false;
            // htrackObject = 1;
        } else if (c == 'w') { 
            std::cerr << "exiting: writing to model" << std::endl;
            tldtracker->writeToFile("model");
        }
    }

    lcm_destroy(state->lcm);

    
    return 0;
}
