/*
TLD's work well w/ features (keypts) low-level features

- DoT works well with textureless objects (contoures)
- Use both to bootstrap a contour and features based reconstruction 
- Does going to 3D from contour make tracking possible ??

*/
#define WINDOW_NAME "TLD Segmenter"
#include <set>

// LCM includes
// #include <perception_opencv_utils/calib_utils.hpp>
// #include <perception_opencv_utils/imshow_utils.hpp>
// #include <perception_opencv_utils/math_utils.hpp>
// #include <perception_opencv_utils/plot_utils.hpp>
// #include <perception_opencv_utils/color_utils.hpp>
// #include <perception_opencv_utils/imgproc_utils.hpp>

#include "kinect_opencv_utils.h"
#include <lcmtypes/kinect_image_msg_t.h>

// #include <lcmtypes/kinect_frame_msg_t.h>
// #include <kinect/kinect-utils.h>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include <lcmtypes/perception_image_roi_t.h>

#define SHOW_ALL_RECTS_BY_ONE 0
int MAX_IMAGE_WIDTH = 0;
int MAX_IMAGE_HEIGHT = 0;

using namespace cv;

typedef struct _state_t state_t;
struct _state_t {
    lcm_t *lcm;
    GMainLoop *mainloop;
    BotParam *param;
    BotFrames *frames;
};
state_t * state = NULL;

struct MouseEvent {
    MouseEvent() { event = -1; buttonState = 0; }
    Point pt;
    int event;
    int buttonState;
};
MouseEvent mouse;

// UI selection of desired object
Rect selection;
Point origin;
bool selectObject = false;

int64_t last_utime; 
Mat last_img; 

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


static void onMouse(int event, int x, int y, int flags, void* userdata) {
    MouseEvent* data = (MouseEvent*)userdata;
    if (selectObject) {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
        selection &= Rect(0, 0, MAX_IMAGE_WIDTH, MAX_IMAGE_HEIGHT);
    }

    switch (event) {
    case CV_EVENT_LBUTTONDOWN:
        origin = Point(x, y);
        selection = Rect(x, y, 0, 0);
        selectObject = true;
        break;
    case CV_EVENT_LBUTTONUP:
        selectObject = false;
        std::cerr << "SEND selection: " << last_utime << " - " << selection.x << " " << selection.y << " " << selection.width << " " << selection.height << std::endl;

        perception_image_roi_t img_selection;
        img_selection.utime = last_utime;
        img_selection.roi.x = selection.x;
        img_selection.roi.y = selection.y;
        img_selection.roi.width = selection.width;
        img_selection.roi.height = selection.height;
        perception_image_roi_t_publish(state->lcm, "TLD_OBJECT_ROI", &img_selection);

        break;
    }
    return;
}

void  INThandler(int sig)
{
    // lcm_destroy(state->lcm);
    printf("Exiting\n");
    exit(0);
}

static void on_image_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const bot_core_image_t *msg, 
                            void *user_data ) {
    
    if (!msg->width || !msg->height)
        return;

    if (!MAX_IMAGE_WIDTH || !MAX_IMAGE_HEIGHT) { 
        MAX_IMAGE_WIDTH = msg->width;
        MAX_IMAGE_HEIGHT = msg->height;
    }

    Mat img(msg->height, msg->width, CV_8UC3);
    if (!selectObject) { 
        // std::cerr << msg->height << " " << msg->width << " " << msg->utime << std::endl;
    
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

        // int npixels = msg->height * msg->width;

        last_utime = msg->utime;
    } else 
        img = last_img;
        
    // Clone image before drawing rectangle
    last_img = img.clone();

    if (selectObject && selection.width > 0 && selection.height > 0) {
        Mat roi(img, selection);
        rectangle(img, selection, cv::Scalar(0,255,255), 2);
        // bitwise_not(roi, roi);
    }

    imshow(WINDOW_NAME, img);    
    return;
}


int main(int argc, char** argv)
{
    // g_thread_init(NULL);
    setlinebuf (stdout);
    state = (state_t*) calloc(1, sizeof(state_t));

    // Param server, botframes
    state->lcm =  bot_lcm_get_global(NULL);
    state->param = bot_param_new_from_server(state->lcm, 1);
    state->frames = bot_frames_get_global (state->lcm, state->param);

    namedWindow( WINDOW_NAME );
    bot_core_image_t_subscribe(state->lcm, "CAMERALEFT", on_image_frame, (void*)state);
    setMouseCallback( WINDOW_NAME, onMouse, &mouse);

    
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

        if (c == 'q') break;      
        
    }

    lcm_destroy(state->lcm);

    
    return 0;
}
