#define WINDOW_NAME "TLD Tracker"

#include <set>

// // LCM includes
// #include <perception_opencv_utils/calib_utils.hpp>
#include <perception_opencv_utils/imshow_utils.hpp>
#include <perception_opencv_utils/math_utils.hpp>
#include <perception_opencv_utils/plot_utils.hpp>
#include <perception_opencv_utils/color_utils.hpp>
// #include <perception_opencv_utils/imgproc_utils.hpp>

// #include "kinect_opencv_utils.h"
#include <lcmtypes/kinect_image_msg_t.h>

#include <lcmtypes/kinect_frame_msg_t.h>
#include <kinect/kinect-utils.h>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include <image_utils/jpeg.h>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

#include "tld/TLD.h"

// #include "MRPTTracker.hpp"
// #include "HistTracker.hpp"
// #include "linemod.hpp"

#include <lcmtypes/perception_image_roi_t.h>
#include <lcmtypes/perception_pointing_vector_t.h>
#include <pthread.h>

#include <lcmtypes/multisense.h>

#include <lcmtypes/vs_obj_collection_t.h>
#include <lcmtypes/vs_obj_t.h>
#include <lcmtypes/vs_link_collection_t.h>
#include <lcmtypes/vs_link_t.h>
#include <lcmtypes/vs_cov_collection_t.h>
#include <lcmtypes/vs_cov_t.h>
#include <lcmtypes/vs_point3d_list_collection_t.h>
#include <lcmtypes/vs_point3d_list_t.h>
#include <lcmtypes/vs_reset_collections_t.h>
#include <lcmtypes/vs_collection_config_t.h>
#include <lcmtypes/vs_text_collection_t.h>
#include <lcmtypes/vs_text_t.h>

#include <ConciseArgs>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#define SHOW_ALL_RECTS_BY_ONE 0
#define BUFFER_TIME_SIZE 1 // seconds

// Variables for Multi-object tracker
bool vDEBUG = false;
int vMIN_DIM = 10; 
int vSTEREO_MODE = 0; 
int vAFFORDANCE_ID = 0; 
int vAFFORDANCE_MODE = 1; 
float vSCALE = 0.25f;
float vDETECT_FPS = 2.f;
float vTRACKER_FPS = 2.f;
std::string vAFFORDANCE_CHANNEL = "CAMERALEFT_MASKZIPPED";

struct state_t {
    lcm_t *lcm;
    GMainLoop *mainloop;
    BotParam *param;
    BotFrames *frames;
    image_io_utils*  imgutils;

    cv::Mat aff_img; 
    int64_t aff_utime; 

    cv::Mat img, gray;
    cv::Mat_<uint16_t> depth16; 
    cv::Mat_<uint8_t> depth8; 
    cv::Mat_<float> depth; 
};
state_t * state = NULL;

struct CameraParams { 
    int width, height;
    double fx, fy, cx, cy, k1, k2, k3, p1, p2;
    double D[4], K[9], R[9], t[3]; 
    cv::Mat_<double> Q_; 
    CameraParams() { 
        D[0] = D[1] = D[2] = D[3] = 0; 
        K[0] = K[4] = K[8] = 1;
        K[1] = K[2] = K[3] = K[5] = K[6] = K[7] = 0; 
        R[0] = R[4] = R[8] = 1;
        R[1] = R[2] = R[3] = R[5] = R[6] = R[7] = 0; 
        t[0] = t[1] = t[2] = 0; 

        /////// From Carnegie's driver
        Q_ = cv::Mat_<double>(4,4,0.0);
        Q_(0,0) = 1.0;
        Q_(1,1) = 1.0;  
        //double Tx = baseline();
        Q_(3,2) = 14.2914745276283;//1.0 / Tx;
        Q_(0,3) = -512.0;//-right_.cx();
        Q_(1,3) = -272.0;//-right_.cy();
        Q_(2,3) = 606.0344848632812;//right_.fx();
        Q_(3,3) = 0;// (512.0 - 272.0)/0.07;//(right_.cx() - left_.cx()) / Tx;  
    }

    // figure out distortions
    void setIntrinsics(double _fx, double _fy, double _cx, double _cy, double _k1, double _k2, 
                       double _k3, double _p1, double _p2) { 
        fx = _fx, fy = _fy, cx = _cx, cy = _cy; 
        k1 = _k1, k2 = _k2, k3 = _k3, p1 = _p1, p2 = _p2; 
        
        K[0] = fx, K[4] = fy; 
        K[2] = cx, K[5] = cy; 
    }

    void setExtrinsics(double* _R, double* _t) { 
        memcpy(R, _R, 9 * sizeof(double)); 
        memcpy(t, _t, 3 * sizeof(double));
    }


    // Assuming that tne points3d are in the left camera's frame of reference, 
    // left is fixed at [I 0] and right is defined w.r.t left from the stereo extrinsic params
    std::vector<cv::Point3f> 
    backproject_points(std::vector<cv::Point2f>& pts, std::vector<double>& depths, bool normalized) { 
        std::vector<cv::Point3f> pts3d(pts.size()); 

        if (normalized) {
            for (int j=0;j<pts.size();++j) {
                double x[] = {pts[j].x,pts[j].y};
                cv::Point3f& m = pts3d[j];
                double t2 = depths[j] * x[0] - t[0];
                double t5 = depths[j] * x[1] - t[1];
                double t7 = depths[j] - t[2];
                m.x = R[0] * t2 + R[3] * t5 + R[6] * t7;
                m.y = R[1] * t2 + R[4] * t5 + R[7] * t7;
                m.z = R[2] * t2 + R[5] * t5 + R[8] * t7;
            }
        } else {
            for (int j=0;j<pts.size();++j) {
                double x[] = {pts[j].x,pts[j].y};
                cv::Point3f& m = pts3d[j];

                double t1 = 0.1e1 / K[0];
                double t7 = depths[j] * x[0] - K[0] * t[0] - K[1] * t[1] - K[2] * t[2];
                double t10 = 0.1e1 / K[4];
                double t11 = t1 * t10;
                double t18 = depths[j] * x[1] - K[4] * t[1] - K[5] * t[2];
                double t22 = K[1] * K[5] - K[2] * K[4];
                double t24 = 0.1e1 / K[8];
                double t25 = t11 * t24;
                double t28 = t10 * t24;
                double t33 = depths[j] - K[8] * t[2];
                m.x = R[0] * t1 * t7 + (-R[0] * K[1] * t11 + R[3] * t10) * t18 + 
                    (R[0] * t22 * t25 - R[3] * K[5] * t28 + R[6] * t24) * t33;
                m.y = R[1] * t1 * t7 + (-R[1] * K[1] * t11 + R[4] * t10) * t18 + 
                    (R[1] * t22 * t25 - R[4] * K[5] * t28 + R[7] * t24) * t33;
                m.z = R[2] * t1 * t7 + (-R[2] * K[1] * t11 + R[5] * t10) * t18 + 
                    (R[2] * t22 * t25 - R[5] * K[5] * t28 + R[8] * t24) * t33;
            }
        }
        return pts3d;
    }

    std::vector<cv::Point2f> 
    project_points(std::vector<cv::Point3f>& pts3d) { 
        std::vector<cv::Point2f> pts; 
        cv::projectPoints(pts3d, cv::Mat_<double>(3,3,(double*)R), cv::Mat_<double>(3,1,(double*)t), 
                          cv::Mat_<double>(3,3,(double*)K), cv::Mat_<double>(4,1,(double*)D), pts);
        return pts;        
    }

    cv::Point3f 
    reprojectTo3D(cv::Point2f pt, double d) { 
        /* float fx_inv = 1.f / K[0], fy_inv = 1.f / K[4];  */
        /* cv::Point3f pt3d( -(cx  * fx_inv - pt.x * fx_inv) * depth,  */
        /*                   -(cy * fy_inv - pt.y * fy_inv) * depth,  */
        /*                   depth); */
        double p[] = { pt.x, pt.y, d, 1 }; 
        cv::Mat_<double> p_(4,1,&p[0]); 
        // std::cerr << "p_: " << p_ << " " << d << std::endl;
        cv::Mat_<double> res = Q_ * p_ * 1.f / 1000.f; 
        // std::cerr << "RES: " << res << std::endl;
        return cv::Point3f(res(0,0)/res(3,0), res(1,0)/res(3,0), res(2,0)/res(3,0)); 
    }
};
std::map<std::string, CameraParams> camera_params;

struct TimedRect : public cv::Rect { 
    int64_t utime; 
    TimedRect () : cv::Rect(), utime(0) {}
    TimedRect (int64_t _utime, int x, int y, int w, int h) 
        : utime(_utime), cv::Rect(x,y,w,h) {}
};


std::map<int32_t, TimedRect> affordance_roi_map; 
// std::map<int32_t, HistTracker*> color_tracker_map; 
// std::map<int32_t, cv::linemod2::Detector> object_detector_map;


std::map<int32_t, tld::DetectorCascade*> object_detector_map; 
struct TLDInfo { 
    tld::TLD* tld; 
    std::string cam; 
    bool inited; 
    int64_t init_utime, last_processed; 
    int32_t object_id; 
    int32_t feature_id; 
    
private: 
    tld::TLD* init_tracker(int32_t OBJECT_ID, int32_t FEAT_ID) { 
        tld::TLD* tr; 

        // New TLD Tracker
        tr = new tld::TLD(); 
        tr->alternating = false;
        tr->learningEnabled = true;
        tr->trackerEnabled = false;
        tr->scale_factor = vSCALE;

        // Per-object detector (independent of cameras/views/instances)
        if (object_detector_map.find(OBJECT_ID) == object_detector_map.end()) { 
            tld::DetectorCascade* detectorCascade = tr->detectorCascade;

            int factor = (1.f / vSCALE);
            detectorCascade->imgWidth = camera_params[cam].width/factor;
            detectorCascade->imgHeight = camera_params[cam].height/factor;
            detectorCascade->imgWidthStep = camera_params[cam].width/factor;
            
            detectorCascade->varianceFilter->enabled = true;
            detectorCascade->ensembleClassifier->enabled = true;
            detectorCascade->nnClassifier->enabled = true;

            // classifier
            detectorCascade->useShift = true; 
            detectorCascade->shift = 0.04;
            detectorCascade->minScale = -10;
            detectorCascade->maxScale = 10;
            detectorCascade->minSize = 10; // 25
            detectorCascade->numTrees = 10;
            detectorCascade->numFeatures = 10;
            detectorCascade->nnClassifier->thetaTP = 0.65;
            detectorCascade->nnClassifier->thetaFP = 0.5;
            object_detector_map[OBJECT_ID] = detectorCascade; 
        } else {
            // tr->trackerEnabled = false;
            tr->detectorCascade->release();
            tr->detectorCascade = object_detector_map[OBJECT_ID];
        }

        // #if 0
        //     std::cerr << "reading from model" << std::endl;
        //     tr->readFromFile("model");
        //     tr->learningEnabled = false;
        //     trackObject = 1;
        // #endif
            
        return tr;
    }

public: 
    TLDInfo () : tld(NULL), object_id(-1), feature_id(-1) {};
    TLDInfo (int _init_utime, int32_t _object_id, int32_t _feature_id) : 
    init_utime(_init_utime), object_id(_object_id), feature_id(_feature_id), cam("CAMERALEFT") {
        std::cerr << "TLD CONSTRUCTOR *********************" << std::endl;
        tld = init_tracker(_object_id, _feature_id); 
        inited = true;
        last_processed = 0;
    }
    TLDInfo (const TLDInfo& tinfo) { 
        std::cerr << "TLD CONSTRUCTOR *********************" << std::endl;
        tld = tinfo.tld; 
        init_utime = tinfo.init_utime;
        last_processed = tinfo.last_processed;
        inited = tinfo.inited; 
        object_id = tinfo.object_id; 
        feature_id = tinfo.feature_id; 
        cam = tinfo.cam; 
    }
    ~TLDInfo() { 
        if (tld) delete tld; 
    }
};

// struct LINEMODOptions { 
//     bool learning_enabled;
//     bool show_match_results; 
//     int num_classes; 
//     int matching_threshold;
// };
 
// struct LINEMODInfo { 
//     cv::Ptr<cv::linemod2::Detector> detector; 
//     std::string cam; 

//     bool inited;

//     int64_t init_utime; 
//     int32_t object_id; 
//     int32_t feature_id; 

//     std::string filename;
    
// private: 
//     cv::Ptr<cv::linemod2::Detector> init_tracker(int32_t OBJECT_ID, int32_t FEAT_ID) {
//         // New LINEMOD Tracker
//         // cv::Ptr<cv::linemod2::Detector> tr = cv::linemod2::getDefaultLINE(); // 
//         cv::Ptr<cv::linemod2::Detector> tr = cv::linemod2::getDefaultLINEMOD(); 
//         return tr;
//     }

// public: 
//     LINEMODInfo () : detector(NULL), object_id(-1), feature_id(-1), filename("linemod_templates.yml") {};
//     LINEMODInfo (int _init_utime, int32_t _object_id, int32_t _feature_id) : 
//     init_utime(_init_utime), object_id(_object_id), feature_id(_feature_id), 
//     cam("CAMERALEFT"), filename("linemod_templates.yml") {
//         std::cerr << "LINEMOD CONSTRUCTOR *********************" << std::endl;
//         detector = init_tracker(_object_id, _feature_id); 
//         inited = true;
//     }
//     LINEMODInfo (const LINEMODInfo& tinfo) { 
//         std::cerr << "LINEMOD CONSTRUCTOR *********************" << std::endl;
//         detector = tinfo.detector; 
//         init_utime = tinfo.init_utime;
//         inited = tinfo.inited; 
//         object_id = tinfo.object_id; 
//         feature_id = tinfo.feature_id; 
//         cam = tinfo.cam;
//         filename = tinfo.filename;
//     }
//     ~LINEMODInfo() { 
//         // if (detector) detector; 
//     }
// };


typedef std::map<std::pair<int64_t, int64_t>, TLDInfo*> TLDTrackerMap;
typedef std::map<std::pair<int64_t, int64_t>, TLDInfo*>::iterator TLDTrackerMapIt; 
TLDTrackerMap tld_tracker_map; 

// std::map<int32_t, LINEMODInfo*> linemod_tracker_map; 

// typedef std::map<std::pair<int64_t, int64_t>, spvision::MRPTTracker*> MRPTTrackerMap;
// typedef std::map<std::pair<int64_t, int64_t>, spvision::MRPTTracker*>::iterator MRPTTrackerMapIt; 
// MRPTTrackerMap mrpt_tracker_map; 

pthread_mutex_t buffer_mutex;
typedef std::pair<cv::Mat, cv::Mat> image_pair;
std::deque<std::pair<int64_t, image_pair> > cbuffer;

std::deque<cv::Mat> depth_buffer;

struct stereo_roi { 
    cv::Rect left; 
    cv::Rect right; 
}; 
std::map<int64_t, stereo_roi> stereo_roi_map; 

inline bool is_nullrect(const cv::Rect& r) { 
    return (r.x < 0 || r.y < 0 || r.width <= 0 || r.height <= 0); 
}

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
}

// //--------------------------------------------
// // Feature Extractor, Descriptor, and Matcher
// //--------------------------------------------
// struct ObjectMatcher { 
//     cv::Ptr<FeatureDetector> detector;
//     cv::Ptr<DescriptorExtractor> extractor; 
//     cv::Ptr<DescriptorMatcher> matcher; 
//     cv::Ptr<BOWTrainer> vocab;
// };
// ObjectMatcher obj_matcher;

void publish_camera_frame(int64_t utime); 
void set_camera_params(std::string cam); 
void process_left_and_depth_images(int64_t utime, const cv::Mat& img, const cv::Mat& depth);
void run_tracker();
void affordance_capture(state_t* state);
bool viz_tracker();
void pipe_to_buffer(int64_t utime, image_pair& img);
void select_object(int64_t utime, int32_t object_id, int32_t feature_id, cv::Rect& roi);

// Adapted from cv_timer in cv_utilities
class Timer
{
public:
  Timer() : start_(0), time_(0) {}

  void start()
  {
    start_ = cv::getTickCount();
  }

  void stop()
  {
    CV_Assert(start_ != 0);
    int64 end = cv::getTickCount();
    time_ += end - start_;
    start_ = 0;
  }

  double time()
  {
    double ret = time_ / cv::getTickFrequency();
    time_ = 0;
    return ret;
  }

private:
  int64 start_, time_;
};

// void drawResponse(const std::vector<cv::linemod2::Template>& templates,
//                   int num_modalities, cv::Mat& dst, cv::Point offset, int T)
// {
//   static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255),
//                                         CV_RGB(0, 255, 0),
//                                         CV_RGB(255, 255, 0),
//                                         CV_RGB(255, 140, 0),
//                                         CV_RGB(255, 0, 0) };

//   for (int m = 0; m < num_modalities; ++m)
//   {
//     // NOTE: Original demo recalculated max response for each feature in the TxT
//     // box around it and chose the display color based on that response. Here
//     // the display color just depends on the modality.
//     cv::Scalar color = COLORS[m];

//     for (int i = 0; i < (int)templates[m].features.size(); ++i)
//     {
//       cv::linemod2::Feature f = templates[m].features[i];
//       cv::Point pt(f.x + offset.x, f.y + offset.y);
//       cv::circle(dst, pt, T / 2, color);
//     }
//   }
// }


char* esc_red = "\033[0;31m";
char* esc_green = "\033[0;32m";
char* esc_brown = "\033[0;33m";
char* esc_blue = "\033[0;34m";
char* esc_magenta = "\033[0;35m";
char* esc_cyan = "\033[0;36m";
char* esc_lightgray = "\033[0;37m";
char* esc_yellow = "\033[1;33m";
char* esc_def = "\033\[0m";
