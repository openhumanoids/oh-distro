/*
TLD's work well w/ features (keypts) low-level features

- DoT works well with textureless objects (contoures)
- Use both to bootstrap a contour and features based reconstruction 
- Does going to 3D from contour make tracking possible ??

*/

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

// #include <pcl/registration/correspondence_rejection_sample_consensus.h>

#define SHOW_ALL_RECTS_BY_ONE 0
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
int trackObject = 0;
bool initLearning = false;

int mser_delta = 5;
int mser_area_threshold = 101;
int mser_min_margin = 3;
int mser_max_variation = 25;
int mser_min_diversity = 2;

// Tld tracker
tld::TLD* tldtracker;
cv::Mat1b currMask;
cv::Rect currROI;

Mat prev_gray;

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

struct Frame { 
    kinect_data kinect;
    cv::Rect BB;
};

cv::Mat cvtToGray(const Mat& img) { 
    std::vector<Mat1b> channels;
    cv::split(img,channels);
    
    Mat gray;
    addWeighted( channels[2], 0.5, channels[1], 0.5, 0, gray );
    return gray;
}

void mser_features(Mat& img) { 
    Mat gray;
    cvtColor(img, gray, CV_BGR2GRAY);
    
    int delta = 3; 
    int max_area = 14400;
    int min_area = 20;
    int max_evolution = 50;
    double area_threshold = mser_area_threshold * 1.f / 100;
    double min_margin = mser_min_margin * 1.f / 1000;
    double max_variation = mser_max_variation * 1.f / 100;
    double min_diversity = mser_min_diversity * 1.f / 10;
    int edge_blur_size = 5;
    
    Mat mser_img = img.clone();
    MSER mser(delta, min_area, max_area, max_variation, min_diversity, 
              max_evolution, area_threshold, min_margin, edge_blur_size);
    
    vector<vector< Point> > msers;
    mser(gray, msers);

    if (!msers.size())
        return;

    std::vector<Scalar> colors(msers.size());
    opencv_utils::fillColors(colors);

    for (int j=0; j<msers.size(); j++) 
        for (int k=0; k<msers[j].size(); k++) 
            circle( mser_img, msers[j][k], 2, colors[j], -1, 8);


    imshow("mser", mser_img);
}

void tld_track(Frame& frame) { 
    Mat img = frame.kinect.getRGB().clone();
    // lk_features(img);

    if (trackObject) { 
        if (trackObject < 0) { 
            opencv_utils::imshow("first frame", img);
            trackObject = 1;

            // tld tracker selection
            tldtracker->selectObject(frame.kinect.getGray().clone(), &selection);
        } else  {
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
                    cvRectangle(&img_ipl, trackerBB->tl(), trackerBB->br(), white, 2, 2, 0);
                
                // good features within the bounding box
                if (trackerBB != NULL) { 
                    std::vector<Point2f> corners;
                    Rect r(*trackerBB);

                    Mat roi_orig(frame.kinect.getRGB().clone(), r);

                    int rx = r.x + r.width / 2;
                    int ry = r.y + r.height / 2;

                    r.width *= 2;
                    r.height *= 2;

                    r.x = rx - r.width / 2;
                    r.y = ry - r.height / 2;

                    r &= Rect(0, 0, img.cols, img.rows);

                    Mat gray = frame.kinect.getGray();
                    // Mat roi(gray, r);

                    Mat roi_color; 
                    Mat roi(frame.kinect.getRGB().clone(), r);
                    cvtColor(roi, roi_color, CV_BGR2Lab);

                    if (!prev_gray.empty()) { 
                        r.width = prev_gray.cols;
                        r.height = prev_gray.rows;
                    }

                    cv::Mat patch = tldtracker->currPosPatch();
                    if (!patch.empty()) { 
                        cv::Mat3b patch3;
                        cvtColor(patch, patch3, CV_GRAY2BGR);
                        resize(patch3, patch3, roi_orig.size());
                        addWeighted( roi_orig, 0.2, patch3, 0.8, 0, patch3 );
                        
                        opencv_utils::imshow("roi_orig+patch", patch3);
                    }
                    prev_gray = gray.clone();
                    imshow("roi", roi);


                    // Compute MSER Features on the ROI
                    mser_features(roi);

                }
            }
        
            // printf("Current conf: %3.2f\n", tldtracker->currConf);
        }
        opencv_utils::imshow("result", img);
    }
    // find_corr_bb_in_bumblebee(frame, *(tldtracker->currBB));
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
        selection &= Rect(0, 0, WIDTH, HEIGHT);
    }

    switch (event) {
    case CV_EVENT_LBUTTONDOWN:
        origin = Point(x, y);
        selection = Rect(x, y, 0, 0);
        selectObject = true;
        break;
    case CV_EVENT_LBUTTONUP:
        selectObject = false;
        trackObject = -1;
        break;
    }
    return;
}

void  INThandler(int sig)
{
    // lcm_destroy(state->lcm);
    printf("Exiting\n");
    // exit(0);
}

static void on_kinect_image_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const kinect_frame_msg_t *msg, 
                            void *user_data ) {
    
    Frame frame;
    frame.kinect.update(msg);

    tld_track(frame);

    Mat img = frame.kinect.getRGB().clone();
    if (selectObject && selection.width > 0 && selection.height > 0) {
        Mat roi(img, selection);
        bitwise_not(roi, roi);
    }

    imshow("Kinect RGB", img);    
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

    // Subscribe to the kinect image
    namedWindow( "Kinect RGB" );
    kinect_frame_msg_t_subscribe(state->lcm, "KINECT_FRAME", on_kinect_image_frame, (void*)state);
    setMouseCallback("Kinect RGB", onMouse, &mouse);

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
            trackObject = 1;
        } else if (c == 'w') { 
            std::cerr << "exiting: writing to model" << std::endl;
            tldtracker->writeToFile("model");
        }


    }

    lcm_destroy(state->lcm);

    
    return 0;
}
