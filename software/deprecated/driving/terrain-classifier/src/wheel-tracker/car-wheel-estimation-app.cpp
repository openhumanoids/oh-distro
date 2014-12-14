/*
One time estimation of the car mask
*/

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
#include <lcmtypes/drc_driving_wheel_state_t.h>

#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <ConciseArgs>

#include <GL/gl.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <bot_lcmgl_client/lcmgl.h>

#include <stereo-bm/stereo-bm.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include <trackers/histogram-tracker.hpp>

int MAX_IMAGE_WIDTH = 0;
int MAX_IMAGE_HEIGHT = 0;

#define WINDOW_NAME "Wheel estimator"
#define AFFORDANCE_OFFSET 64
using namespace cv;

struct CarWheelEstimatorOptions { 
    bool vDEBUG;
    float vSCALE;
    std::string vCHANNEL;
    std::string vSTEREO_CHANNEL;

    CarWheelEstimatorOptions () : 
        vCHANNEL(std::string("CAMERALEFT")), vSTEREO_CHANNEL(std::string("CAMERA")), vSCALE(0.25f), vDEBUG(false) {}
};
CarWheelEstimatorOptions options;

typedef std::vector<cv::Point> contour_t;
struct state_t { 
    lcm_t* lcm;
    boost::shared_ptr<lcm::LCM> lcm_; 
    bot_lcmgl_t* lcmgl;

    pthread_t lcm_thread;

    BotParam* param;
    // BotFrames* frames;

    // Stereo img
    cv::Mat img, left, right;

    cv::Mat p_img, p_desc;
    std::vector<cv::KeyPoint> p_kpts;

    // utimes for image
    int64_t img_utime, stereo_utime;

    // Stereo BM library
    StereoB* stereoBM; 

    int counter;

    std::map<std::string, contour_t> wheel_model; 

    // UI selection of desired object
    Rect selection;
    Point origin, destination;
    bool selectObject;
    drc_driving_wheel_state_t* wheel_segment;

    double mean_wheel_depth; 

    std::vector<cv::Scalar> id_colors; 

    // Color info for hood
    HistogramInfo hue_info, val_info, sat_info; 
    cv::Mat hood_bp; 
    int hood_height; 

    void construct_dashbd_histogram() { 
        hood_height = -1;
        mean_wheel_depth = 0;

        // Hue bins, params etc. 
        hue_info.size = 20, val_info.size = 20, sat_info.size = 20;
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

        // Hardcoded values for car mask
        cv::RNG rng; 
        cv::Mat3b dashbd = cv::Mat3b::zeros(100,100); 
        rng.fill(dashbd, cv::RNG::NORMAL, cv::Scalar(110, 200, 200), cv::Scalar(10, 40, 40));

        std::vector<cv::Mat> channels;
        cv::split(dashbd, channels);
        assert(channels.size() == 3);
        cv::Mat hue = channels[0]; 
        cv::Mat val = channels[1];
        cv::Mat sat = channels[2];
        // std::cerr << "hue: " << hue << std::endl;

        // Calculate Histogram
        calcHist(&hue, 1, 0, cv::Mat(), hue_info.histogram, 1, &hue_info.size, &hue_info.pranges); 
        calcHist(&val, 1, 0, cv::Mat(), val_info.histogram, 1, &val_info.size, &val_info.pranges); 
        calcHist(&sat, 1, 0, cv::Mat(), sat_info.histogram, 1, &sat_info.size, &sat_info.pranges); 

        // std::cerr << "hue hist: " << hue_info.histogram << " " << std::endl;
        // std::cerr << "val hist: " << val_info.histogram << " " << std::endl;

        normalize(hue_info.histogram, hue_info.histogram, 0, 1, CV_MINMAX);
        normalize(val_info.histogram, val_info.histogram, 0, 1, CV_MINMAX);
        normalize(sat_info.histogram, sat_info.histogram, 0, 1, CV_MINMAX);

        // Compute unimodal histogram
        hue_info.computeUnimodalHistogram();
        val_info.computeUnimodalHistogram();
        sat_info.computeUnimodalHistogram();

        cv::cvtColor(dashbd, dashbd, CV_HSV2BGR);
        cv::imshow("rng", dashbd);

    }

    state_t (const CarWheelEstimatorOptions& _options) {
        // LCM, BotFrames, BotParam inits
        lcm =  bot_lcm_get_global(NULL);
        lcm_ = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
        lcmgl = NULL;

        param = bot_param_new_from_server(lcm, 1);
        // frames = bot_frames_get_global (lcm, param);

        // Init stereoBM
        stereoBM = new StereoB(lcm_);
        stereoBM->setScale(_options.vSCALE);
            

        // Init histogram info
        // construct_dashbd_histogram();
        wheel_segment = NULL;

        // Counter for debug prints
        counter = 0; 
    }


    ~state_t () { 
        // lcm_destroy(lcm);
    }
};
state_t* state = NULL; 

void
decode_image(const bot_core_image_t * msg, cv::Mat& img)
{
    int ch = msg->row_stride / msg->width;
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
        fprintf(stderr, "Unrecognized image format : %d\n", (int) msg->pixelformat);
      break;
  }
  return;
}

void
decode_stereo_image(const bot_core_image_t * msg, cv::Mat& left, cv::Mat& right)
{

    cv::Mat1b img(msg->height, msg->width);
      
  // extract image data
  switch (msg->pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
        // fprintf(stderr, "rgb : %d\n", (int) msg->pixelformat);

        if (left.empty() || right.empty()) { 
            left.create(msg->height/2, msg->width, CV_8UC3);
            right.create(msg->height/2, msg->width, CV_8UC3);
        }
        memcpy(left.data, msg->data, msg->size / 2);
        memcpy(right.data, msg->data + msg->size / 2, msg->size / 2);
        cv::cvtColor(left, left, CV_RGB2BGR);
        cv::cvtColor(right, right, CV_RGB2BGR);
      break;
  case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
      // fprintf(stderr, "mjpeg : %d\n", (int) msg->pixelformat);
      // for some reason msg->row_stride is 0, so we use msg->width instead.
      jpeg_decompress_8u_gray(msg->data,
                              msg->size,
                              img.data,
                              msg->width,
                              msg->height,
                              msg->width);
      if (left.empty() || right.empty()) { 
          left.create(msg->height/2, msg->width, CV_8UC1);
          right.create(msg->height/2, msg->width, CV_8UC1);
      }
      memcpy(left.data,  img.data , msg->width * msg->height / 2);
      memcpy(right.data,  img.data + msg->width * msg->height / 2 , msg->width * msg->height / 2);
      break;
  case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY:
      // fprintf(stderr, "gray : %d\n", (int) msg->pixelformat);
      if (left.empty() || right.empty()) { 
          left.create(msg->height/2, msg->width, CV_8UC1);
          right.create(msg->height/2, msg->width, CV_8UC1);
      }
      memcpy(left.data,  msg->data , msg->size/2);
      memcpy(right.data,  msg->data + msg->size/2 , msg->size/2);
      break;
    default:
        fprintf(stderr, "Unrecognized image format : %d\n", (int) msg->pixelformat);
        //fprintf(stderr, "Unrecognized image format\n");
      break;
  }
  return;
}

struct SegmentInfo { 
    std::pair<cv::Vec3f, cv::Vec3f> edgevar;
};



static void simpleMatching( Ptr<DescriptorMatcher>& descriptorMatcher,
                     const Mat& descriptors1, const Mat& descriptors2,
                     vector<DMatch>& matches12 )
{
    vector<DMatch> matches;
    descriptorMatcher->match( descriptors1, descriptors2, matches12 );
}

static void crossCheckMatching( Ptr<DescriptorMatcher>& descriptorMatcher,
                         const Mat& descriptors1, const Mat& descriptors2,
                         vector<DMatch>& filteredMatches12, int knn=1 )
{
    filteredMatches12.clear();
    vector<vector<DMatch> > matches12, matches21;
    descriptorMatcher->knnMatch( descriptors1, descriptors2, matches12, knn );
    descriptorMatcher->knnMatch( descriptors2, descriptors1, matches21, knn );
    for( size_t m = 0; m < matches12.size(); m++ )
    {
        bool findCrossCheck = false;
        for( size_t fk = 0; fk < matches12[m].size(); fk++ )
        {
            DMatch forward = matches12[m][fk];

            for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
            {
                DMatch backward = matches21[forward.trainIdx][bk];
                if( backward.trainIdx == forward.queryIdx )
                {
                    filteredMatches12.push_back(forward);
                    findCrossCheck = true;
                    break;
                }
            }
            if( findCrossCheck ) break;
        }
    }
}


//Copy (x,y) location of descriptor matches found from KeyPoint data structures into Point2f vectors
static void matches2points(const vector<DMatch>& matches, const vector<KeyPoint>& kpts_train,
                    const vector<KeyPoint>& kpts_query, vector<Point2f>& pts_train, vector<Point2f>& pts_query)
{
  pts_train.clear();
  pts_query.clear();
  pts_train.reserve(matches.size());
  pts_query.reserve(matches.size());
  for (size_t i = 0; i < matches.size(); i++)
  {
    const DMatch& match = matches[i];
    pts_query.push_back(kpts_query[match.queryIdx].pt);
    pts_train.push_back(kpts_train[match.trainIdx].pt);
  }

}

void track_wheel_features(const cv::Mat& _img, cv::Mat mask = cv::Mat()) { 

    double st = bot_timestamp_now();
    cv::Mat img;
    if (_img.channels() == 3)
        cvtColor(_img, img, CV_BGR2GRAY);
    else
        img = _img.clone(); 

    //--------------------------------------------
    // Gaussian Blur
    //--------------------------------------------
    cv::blur(img, img, cv::Size(7,7));

    //--------------------------------------------
    // Mask from hood
    //--------------------------------------------
    if (mask.empty() && state->hood_height >=0) { 
        mask = cv::Mat1b::ones(img.size());
        cv::Rect rect(0,0,img.cols,state->hood_height);
        cv::Mat roi = mask(rect);
        roi = cv::Scalar(0,0,0);
    }

    //--------------------------------------------
    // Feature Detector, Extractor and Matcher
    //--------------------------------------------
    Ptr<FeatureDetector> detector = FeatureDetector::create( "GFTT" );
    Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create( "FREAK" );
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create( "BruteForce-Hamming" ); // 
    // std::cerr << detector.empty() << " " << extractor.empty() << " " << matcher.empty() << std::endl;
    // if( detector.empty() || extractor.empty() || matcher.empty()  )  {
    //     cout << "Can not create detector or descriptor exstractor or descriptor matcher of given types" << endl;
    // }

    std::vector<cv::KeyPoint> kpts;
    detector->detect( img, kpts, mask );

    cv::Mat desc;
    extractor->compute( img, kpts, desc );

    cv::Mat output;
    if (img.channels() == 3) output = img.clone();
    else cv::cvtColor(img, output, CV_GRAY2BGR);

    // Compute and draw matches
    if (kpts.size() && state->p_kpts.size()) { 
        std::vector<cv::DMatch> matches;
        matcher->match(desc, state->p_desc, matches);

        std::vector<cv::Point2f> p_pts, pts;
        matches2points(matches, state->p_kpts, kpts, p_pts, pts); 

        if ((p_pts.size() == pts.size()) && p_pts.size() >= 4 && pts.size() >= 4) { 
            // cout << "< Computing homography (RANSAC)..." << p_pts.size() << " " << pts.size() << endl;
            Mat_<uchar> status(pts.size(), 1);
            cv::Mat_<double> H12 = cv::findHomography( cv::Mat(p_pts), cv::Mat(pts), CV_RANSAC, 2.f, status);

            double x = state->origin.x, y = state->origin.y; 
            double Z = 1.f / (H12(2,0) * x + H12(2,1) * y + H12(2,2));
            double px = (int)((H12(0,0)*x + H12(0,1)*y + H12(0,2))*Z);
            double py = (int)((H12(1,0)*x + H12(1,1)*y + H12(1,2))*Z);
            cv::Point tf_origin(px,py);

            x = state->destination.x, y = state->destination.y; 
            Z = 1.f / (H12(2,0) * x + H12(2,1) * y + H12(2,2));
            px = (int)((H12(0,0)*x + H12(0,1)*y + H12(0,2))*Z);
            py = (int)((H12(1,0)*x + H12(1,1)*y + H12(1,2))*Z);
            cv::Point tf_destination(px,py);            

            cv::Point2f tfvec(tf_destination.y-tf_origin.y, tf_destination.x-tf_origin.x);
            tfvec *= 1.f / cv::norm(tfvec);
            double angle1 = atan2(tfvec.y,tfvec.x);

            cv::Point2f ovec(state->destination.y-state->origin.y, state->destination.x-state->origin.x);
            ovec *= 1.f / cv::norm(ovec);
            double angle2 = atan2(ovec.y,ovec.x);
            // std::cerr << "angle: " << angle2 * 180 / CV_PI  << " (" << angle2 << ") " << 
            //     state->wheel_segment->theta * 180 / CV_PI << " (" << state->wheel_segment->theta << ") " << std::endl;
            
            double dangle = angle2-angle1; // ccw +ve
            // if (dangle > CV_PI) dangle -= CV_PI; 
            // if (dangle < -CV_PI) dangle += CV_PI;

            float sx = 1.f / state->img.cols ; 
            float sy = 1.f / state->img.rows ;
            drc_driving_wheel_state_t wheel_state; 
            wheel_state.utime = state->img_utime;
            wheel_state.u = tf_origin.x * sx;
            wheel_state.v = tf_origin.y * sy;
            wheel_state.theta = dangle;
            drc_driving_wheel_state_t_publish(state->lcm, "WHEEL_STATE_ESTIMATE", &wheel_state);

            if (options.vDEBUG) { 
                std::cerr << "angle: " << angle1 * 180 / CV_PI  << " (" << angle1 << ") " << 
                    angle2 * 180 / CV_PI << " (" << angle2 << ") " << 
                    dangle * 180 / CV_PI << " (" << dangle << ") " << std::endl;

                float sz = 150;
                cv::Point origin_ = cv::Point(tf_origin.x,tf_origin.y);
                cv::Point destination_ = cv::Point(tf_origin.x + sin(state->wheel_segment->theta-dangle)*sz,tf_origin.y + cos(state->wheel_segment->theta-dangle)*sz);

                // cv::Mat output = state->img.clone();
                cv::circle(output, origin_, 20, cv::Scalar(0,255,0), 2, CV_AA);
                cv::line(output, origin_, destination_, cv::Scalar(0,255,255), 2, CV_AA);

                // cv::circle(output, tf_origin, 20, cv::Scalar(0,255,0), 2, CV_AA);
                // cv::line(output, tf_origin, tf_destination, cv::Scalar(0,255,255), 2, CV_AA);
            }


            if (options.vDEBUG) { 
                cv::Mat img_match; 
                cv::drawMatches(img, kpts, state->p_img, state->p_kpts, matches, img_match, cv::Scalar::all(-1), Scalar::all(-1), status);
                cv::imshow("matches", img_match);
            }

            
        }
    }

    // One time copy keypts
    if (state->p_img.empty()) { 
        state->p_img = img.clone();
        state->p_kpts = kpts;
        state->p_desc = desc.clone();
    }
    if (options.vDEBUG)
        cv::imshow("kpts", output);

    if (state->counter%100==0) printf("time: wheel_tracking %4.3f\n", (bot_timestamp_now() - st) * 1e-3 ); 
    state->counter++;
    return;

}

void determine_dashboard(const cv::Mat& _img) { 
    // Convert to HSV space
    cv::Mat hsv; 
    cvtColor(_img, hsv, CV_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    assert(channels.size() == 3);
    cv::Mat_<float> hue = channels[0]; 
    cv::Mat_<float> val = channels[1];
    cv::Mat_<float> sat = channels[2];
    // std::cerr << "hue: " << hue << std::endl;

    // Calculate likelihood
    cv::Mat hue_bp, val_bp, sat_bp;
    state->hue_info.backProjectUnimodal(hue, hue_bp);
    state->val_info.backProjectUnimodal(val, val_bp);
    state->sat_info.backProjectUnimodal(sat, sat_bp);

    multiply(hue_bp, val_bp, state->hood_bp);
    multiply(sat_bp, state->hood_bp, state->hood_bp);
    normalize(state->hood_bp, state->hood_bp, 0, 1, CV_MINMAX);

    // cv::imshow("bp", state->hood_bp); 

    // AND operator for bp mask and depth mask
    cv::Mat display = _img.clone();
    if (!state->hood_bp.empty()) { 
        int min_row = _img.rows; 
        cv::Mat hood_th = state->hood_bp >= 0.5;
        for (int i=0; i<hood_th.rows; i++) { 
            for (int j=0; j<hood_th.cols; j++) { 
                if (hood_th.at<uchar>(i,j)) {
                    if (i < min_row) { min_row = i;}
                }
            }
        }
        cv::line(display, cv::Point(0,min_row), cv::Point(display.cols-1,min_row), cv::Scalar(200,200,200), 2, CV_AA);
        state->hood_height = min_row;
        // cv::imshow("display", display);
    }
    return;
}


static void on_segment (const lcm_recv_buf_t *rbuf, const char *channel,
                            const drc_driving_wheel_state_t *msg, 
                            void *user_data ) {

    std::cerr << "RECV selection: " << msg->utime << " - " << 
        msg->u << " " << msg->v << " " << msg->theta << std::endl;

    if (!MAX_IMAGE_WIDTH || !MAX_IMAGE_HEIGHT)
        return;

    if (state->wheel_segment)
        drc_driving_wheel_state_t_destroy(state->wheel_segment);
    state->wheel_segment = drc_driving_wheel_state_t_copy(msg);

    float sz = 150; 
    float cx = msg->u * MAX_IMAGE_WIDTH;
    float cy = msg->v * MAX_IMAGE_HEIGHT;
    float dx = cx + sin(msg->theta)*sz;
    float dy = cy + cos(msg->theta)*sz;
    
    state->origin = cv::Point(cx,cy);
    state->destination = cv::Point(dx,dy);

    // Reset
    state->p_img = cv::Mat(); 
    state->p_kpts.clear();
    state->p_desc = cv::Mat();

    // cv::Mat output = state->img.clone();
    // cv::circle(output, state->origin, 20, cv::Scalar(0,255,0), 2, CV_AA);
    // cv::line(output, state->origin, state->destination, cv::Scalar(0,255,255), 2, CV_AA);
    // cv::imshow("test", output);

    state->selection = Rect(cx-10, cy-10, 20, 20);
    state->selection &= Rect(0, 0, MAX_IMAGE_WIDTH, MAX_IMAGE_HEIGHT);

    return;
}

static void on_image_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const bot_core_image_t *msg, 
                            void *user_data ) {

    if (!msg->width || !msg->height) return;
    if (!MAX_IMAGE_WIDTH || !MAX_IMAGE_HEIGHT) { 
        MAX_IMAGE_WIDTH = msg->width;
        MAX_IMAGE_HEIGHT = msg->height;
    }
    
    state_t* state = (state_t*) user_data; 
    decode_image(msg, state->img);    
    state->img_utime = msg->utime; 

    return;
}

static bool contour_size_compare(const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) { 
    return lhs.second < rhs.second;
}


static void on_stereo_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const bot_core_image_t *msg, 
                            void *user_data ) {

    if (!msg->width || !msg->height) return;
    
    state_t* state = (state_t*) user_data; 
    decode_stereo_image(msg, state->left, state->right);
    state->stereo_utime = msg->utime; 

    cv::Mat left = state->left.clone(), right = state->right.clone(); 
    state->stereoBM->doStereoB(left, right);

    cv::Mat gray, display;
    if (state->left.channels() == 3) { 
        display = state->left.clone();
        cv::cvtColor(display, gray, CV_BGR2GRAY);
    } else { 
        gray = state->left.clone();
        cv::cvtColor(gray, display, CV_GRAY2BGR);
    }
    cv::Mat disp = cv::Mat(state->left.rows, state->left.cols, CV_16SC1, state->stereoBM->getDisparity()); 

    if (!state->selection.area())
        return;

    // std::cerr << "selection: " << state->selection.tl() << " "<< state->selection.br() << std::endl;
    if (!state->mean_wheel_depth) { 

        // Estimate depth and threshold out the wheel
        cv::Mat_<float> droi = cv::Mat(disp, state->selection).clone();
        cv::Mat_<float> data = cv::Mat_<float>::zeros(droi.cols * droi.rows, 2); 
        for (int j=0; j<droi.rows * droi.cols; j++) 
            data.at<float>(j,0) = droi.at<float>(j);

        cv::Mat labels,centers;
        cv::kmeans(data, 2, 
                   labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
                   10, KMEANS_PP_CENTERS, centers);

        // std::cerr << "data: " << droi << std::endl;
        std::cerr << "centers: " << centers << std::endl;

        // Depth Mode-seeking
        std::vector<int> votes(centers.rows, 0); 
        for (int j=0; j<labels.rows; j++)
            votes[labels.at<int>(j)]++;
        cv::Point maxIdx(-1,-1); 
        cv::minMaxLoc(votes, 0, 0, 0, &maxIdx);

        state->mean_wheel_depth = centers.at<float>(maxIdx.x, 0);
    }            

    float d_offset = 100;
    cv::Mat_<uchar> mask_disp = (disp > state->mean_wheel_depth - 2 * d_offset) & 
        (disp < state->mean_wheel_depth + 2 * d_offset);

    // Find contours for the mask
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    cv::Size size(mask_disp.cols, mask_disp.rows);
    cv::findContours(mask_disp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE); 

    // Sort contours with max area
    std::vector<std::pair<int, int> > contour_sizes; 
    for (int idx=0; idx>=0; idx = hierarchy[idx][0])
        contour_sizes.push_back(std::make_pair(idx, cv::contourArea(contours[idx])));
    std::vector<std::pair<int, int> >::iterator it = 
        std::max_element(contour_sizes.begin(), contour_sizes.end(), contour_size_compare);

    int largest_contour_id = it->first;
    std::vector<cv::Point> largest_contour = contours[largest_contour_id];

    // Draw Contours from depth
    cv::Mat zmat = cv::Mat::zeros(size, CV_8U);
    cv::Mat3b zmat2 = cv::Mat3b::zeros(size);
    drawContours(zmat, contours, largest_contour_id, cv::Scalar(255), CV_FILLED, 8, hierarchy); 
    cv::dilate(zmat, zmat, cv::Mat(), cv::Point(-1,-1), 3);

    // Track Features on Wheel
    track_wheel_features(state->left, zmat); 
    if (options.vDEBUG)
        cv::imshow("zmat", zmat);

    return;
}



void  INThandler(int sig) {
    printf("Exiting\n");
    if (state) delete state;
    exit(0);
}


int main(int argc, char** argv)
{
    std::cout << "============= QUICK MODES ===================\n";
    std::cout << "drc-wheel-estimation -s 0.5 -c CAMERALEFT\n";
    std::cout << "=============================================\n";

    ConciseArgs opt(argc, (char**)argv);
    opt.add(options.vCHANNEL, "c", "camera-channel","Camera Channel [CAMERALEFT]");
    opt.add(options.vCHANNEL, "t", "stereo-channel","Camera Channel [CAMERA]");
    opt.add(options.vSCALE, "s", "scale","Scale Factor");
    opt.add(options.vDEBUG, "d", "debug","Debug mode");
    opt.parse();

    std::cerr << "===========  DRC Wheel Estimation ============" << std::endl;
    std::cerr << "=> CAMERA CHANNEL : " << options.vCHANNEL << std::endl;
    std::cerr << "=> STEREO CAMERA CHANNEL : " << options.vSTEREO_CHANNEL << std::endl;
    std::cerr << "=> SCALE : " << options.vSCALE << std::endl;
    std::cerr << "=> DEBUG : " << options.vDEBUG << std::endl;
    std::cerr << "===============================================" << std::endl;

    // Install signal handler to free data.
    signal(SIGINT, INThandler);

    // Param server, botframes
    state = new state_t(options);

    // Subscriptions
    bot_core_image_t_subscribe(state->lcm, options.vCHANNEL.c_str(), on_image_frame, (void*)state);
    bot_core_image_t_subscribe(state->lcm, options.vSTEREO_CHANNEL.c_str(), on_stereo_frame, (void*)state);
    drc_driving_wheel_state_t_subscribe(state->lcm, "WHEEL_STATE_SEGMENT", on_segment, (void*)state);

    // Main lcm handle
    while(1) { 
        lcm_handle(state->lcm);
        unsigned char c = cv::waitKey(1) & 0xff;
        if (c == 'q') { 
            break;      
        } 
        // // pthread_mutex_lock(&state->img_mutex);
        // // UI handling 
        // if (options.vDEBUG) { 
        //     if (!state->img.empty()) { 
        //         cv::Mat display;
        //         cv::resize(state->img.clone(), display, cv::Size(MAX_IMAGE_WIDTH,MAX_IMAGE_HEIGHT)); 
        //         if (state->selection.width > 0 && state->selection.height > 0) {
                
        //             cv::Mat roi(display, state->selection);
        //             rectangle(display, state->selection, cv::Scalar(0,255,255), 2);
        //             cv::circle(display, state->origin, 20, cv::Scalar(0,255,0), 1, CV_AA);
        //             cv::line(display, state->origin, state->destination, cv::Scalar(0,255,255), 1, CV_AA);
        //             // bitwise_not(roi, roi);
        //         }
        //         imshow(WINDOW_NAME, display);
        //     }
        // }
        // // pthread_mutex_unlock(&state->img_mutex);
    }

    if (state) delete state;
    return 0;
}



    // // Find wheel center and fit ellipse
    // cv::RotatedRect wrect = fitEllipse(largest_contour); 
    // cv::ellipse( zmat2, wrect, cv::Scalar(0,255,0), 1, CV_AA );    

    // // Perform distance transform on the center of the wheel
    // int voronoiType = 0;
    // int maskSize = voronoiType >= 0 ? CV_DIST_MASK_5 : CV_DIST_MASK_5;
    // int distType = voronoiType >= 0 ? CV_DIST_L2 : CV_DIST_L1;

    // cv::Mat dist, distlabels, dist8u; 
    // distanceTransform( zmat, dist, distlabels, distType, maskSize, voronoiType );
    // cv::Mat_<uchar> borderlabels = cv::Mat_<uchar>::zeros(distlabels.size());

    // // Find the border in labels
    // for (int i=1; i<distlabels.rows-1; i++) { 
    //     const int* ll = (const int*)distlabels.ptr(i);
    //     const int* llu = (const int*)distlabels.ptr(i-1);
    //     const int* llb = (const int*)distlabels.ptr(i+1);
    //     for (int j=1; j<distlabels.cols-1; j++) { 
    //         int llj = ll[j];
    //         if ((llj != ll[j-1]) || (llj != ll[j+1]) ||
    //             (llj != llu[j-1]) || (llj != llu[j]) || (llj != llu[j+1]) ||
    //             (llj != llb[j-1]) || (llj != llb[j]) || (llj != llb[j+1]))
    //             borderlabels(i,j) = 255;
    //     }
    // }

    // // Perform probabilistic hough transform
    // std::vector<cv::Vec4i> wheel_lines;
    // cv::HoughLinesP(borderlabels, wheel_lines, 1, CV_PI/180, 50, 30, 10); 
    // // std::cerr << "wheel_lines: " << wheel_lines.size() << std::endl;
    // // std::cerr << "wrect: " << wrect.center << std::endl;
        
    // std::vector<cv::Point2f> spoke_angles; 
    // for( size_t i = 0; i < wheel_lines.size(); i++ ) {
    //     cv::Vec4i l = wheel_lines[i];

    //     cv::Point2f p1 = cv::Point(l[0], l[1]); 
    //     cv::Point2f p2 = cv::Point(l[2], l[3]); 

    //     cv::Point2f v = p1 - p2; 
    //     float norm = cv::norm(v); 
    //     if (norm > 0) v *= 1.f / norm; 

    //     cv::Point2f pc = p1 + (v.dot(wrect.center-p1))*v;
    //     float d = cv::norm(pc-wrect.center);
    //     if (d > 50) continue;

    //     if (v.y < 0) { v.x = -v.x; v.y = fabs(v.y); }
    //     spoke_angles.push_back(v);

    //     cv::line( borderlabels, p1, p2, cv::Scalar(128), 2, CV_AA);
    //     cv::circle( borderlabels, pc, 4, cv::Scalar(128), 1, CV_AA);
    //     cv::circle( borderlabels, wrect.center, 8, cv::Scalar(100), 1, 8, 0 );
    // }

    // // float wheel_angle = 0;
    // // bool wheel_angle_good = false;
    // // if (spoke_angles.size() > 2) { 

    // //     // CosineDistanceSimilarity<float> dist_func(2); 
    // //     // cv::Mat(spoke_angles)
    // //     cv::Mat spoke_labels, spoke_centers;
    // //     cv::kmeans(cv::Mat(spoke_angles), 3, 
    // //                spoke_labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
    // //                10, KMEANS_PP_CENTERS, spoke_centers);


    // //     std::cerr << "spokes: " << cv::Mat(spoke_angles) << std::endl;
    // //     std::cerr << "spokes lab: " << spoke_labels << std::endl;
    // //     std::cerr << "spokes centers: " << spoke_centers << std::endl;

    // //     // Depth Mode-seeking
    // //     cv::Point maxIdx(-1,-1); 
    // //     cv::minMaxLoc(spoke_centers, 0, 0, 0, &maxIdx);
    // //     std::cerr << "maxidx: " << maxIdx.y << std::endl;

    // //     for (int j=0; j<spoke_labels.rows; j++)
    // //         if (spoke_labels.at<int>(j) == maxIdx.y)
    // //             spoke_angles[j] = spoke_angles[j] - 180;

    // //     std::cerr << "spokes: " << cv::Mat(spoke_angles) << std::endl;

    // //     cv::kmeans(cv::Mat(spoke_angles), 2, 
    // //                spoke_labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
    // //                10, KMEANS_PP_CENTERS, spoke_centers);

    // //     std::cerr << "spokes lab: " << spoke_labels << std::endl;
    // //     std::cerr << "spokes centers: " << spoke_centers << std::endl;


    // //     // int count0 = 0, count1 = 0; 
    // //     // for (int j=0; j<spoke_labels.rows; j++) 
    // //     //     if (spoke_labels.at<int>(j) == 1) count1++;
    // //     //     else count0++;
    // //     // if (count0 > count1) { 
    // //     //     // wheel_angle = spoke_centers.at<float>(0);
    // //     //     wheel_angle = atan2(spoke_centers.at<float>(0,1), spoke_centers.at<float>(0,0));
    // //     //     wheel_angle_good = true; 
    // //     // } else if (count1 > count0) { 
    // //     //     // wheel_angle = spoke_centers.at<float>(1);
    // //     //     wheel_angle = atan2(spoke_centers.at<float>(1,1), spoke_centers.at<float>(1,0));
    // //     //     wheel_angle_good = true; 
    // //     // }

    // //     std::cerr << "wheel angle: " << wheel_angle_good << " " << wheel_angle << std::endl;

    // //     // if (wheel_angle_good) { 
    // //     //     cv::Point2f wheeltip = wrect.center;
    // //     //     wheeltip.x += cos(wheel_angle) * wrect.size.width/2;
    // //     //     wheeltip.y += sin(wheel_angle) * wrect.size.width/2;
    // //     //     cv::line( borderlabels, wrect.center, wheeltip, cv::Scalar(128), 2, CV_AA);
    // //     // }
    // // }
    // cv::imshow("border", borderlabels);
    // // for (int idx=largest_contour_id; idx<largest_contour_id+1; idx++) { 
    // //     std::vector<cv::Point>& contourj = contours[idx]; 
    // //     for (int j=0; j<contourj.size(); j++)
    // //         cv::circle(zmat2, contourj[j], 1, cv::Scalar(0,255,0)); 
    // // }

    // // Find all contours with the largest component as their parent
    // std::vector<std::pair<int, int> > contour_children; 
    // // std::cerr << "largest: " << largest_contour_id << std::endl;
    // for (int idx=0; idx<contours.size(); idx++) { 
    //     // std::cerr << "hier: " << idx << " " << cv::Mat(hierarchy[idx]) << " " << contours[idx].size() << std::endl;
    //     if (hierarchy[idx][3] == largest_contour_id)
    //         contour_children.push_back(std::make_pair(idx, contours[idx].size()));
    // }


    // std::vector<std::vector<cv::Point> > inner_contours;
    // it = std::max_element(contour_children.begin(), contour_children.end(), contour_size_compare);
    // int cidx = it->first;
    // // drawContours(zmat2, contours, cidx, colors[count], CV_FILLED, 8, hierarchy); 

    // std::vector<cv::Point>& contourj = contours[cidx]; 
    // cv::RotatedRect crect = fitEllipse(contourj); 
    // cv::Point2f p1 = wrect.center;
    // cv::Point2f p2 = crect.center;
    // cv::Point2f p12vec = cv::Point2f(p2.x-p1.x,p2.y-p1.y);
            
    // float p12norm = cv::norm(p12vec);
    // if (p12norm) p12vec *= 1.f / p12norm;

    // // cv::line(zmat2, wrect.center, wrect.center + p12vec * wrect.size.width * 0.5, cv::Scalar(200,0,200), 2, CV_AA); 

    // int spoke_lines_count = 0; 
    // cv::Point2f mean_spoke_vec(0,0);
    // for (int j=0; j<spoke_angles.size(); j++) { 
    //     if (fabs(p12vec.dot(spoke_angles[j])) > 0.8) { 
    //         mean_spoke_vec.x += spoke_angles[j].x;
    //         mean_spoke_vec.y += spoke_angles[j].y;
    //         // cv::line(zmat2, wrect.center, wrect.center + spoke_angles[j] * wrect.size.width * 0.5, 
    //         //          cv::Scalar(0,200,0), 2, CV_AA); 
    //         spoke_lines_count++;
    //     }
    // }

    // if (spoke_lines_count) { 
    //     mean_spoke_vec.x /= spoke_lines_count; 
    //     mean_spoke_vec.y /= spoke_lines_count; 
    //     if (mean_spoke_vec.dot(p12vec) > 0)
    //         cv::line(zmat2, wrect.center, wrect.center + mean_spoke_vec * wrect.size.width * 0.5, 
    //                  cv::Scalar(0,200,200), 2, CV_AA); 
    //     else 
    //         cv::line(zmat2, wrect.center, wrect.center - mean_spoke_vec * wrect.size.width * 0.5, 
    //                  cv::Scalar(0,200,200), 2, CV_AA); 

    // }
    // // cv::imshow("zmat2", zmat2);
    // addWeighted(display, 0.3, zmat2, 1.0, 0, display);
    // cv::imshow("Display", display);
    // // int ndisps = ((state->left.size().width/8) + 15) & -16;
    // // cv::Mat disp8; 
    // // disp.convertTo(disp8, CV_8U, 1.f);
    // // cv::imshow("disp", disp);
    // // state->stereoBM->sendRangeImage(msg->utime);


// void circleRANSAC(cv::vector<cv::Point>& pts, 
//                   std::vector<Vec3f> &circles, double circle_threshold, double min_radius, int numIterations) {
    
//     // Points. 
//     cv::vector<cv::Point2d> points(pts.size()); 
//     for (int j=0; j<pts.size(); j++) 
//         points[j] = cv::Point2d(pts[j].x,pts[j].y);


//     // 4 point objects to hold the random samples
//     Point2d pointA, pointB, pointC, pointD;

//     // distances between points
//     double AB, BC, CA, DC;

//     // varibales for line equations y = mx + b
//     double m_AB, b_AB, m_BC, b_BC;

//     // varibles for line midpoints
//     double XmidPoint_AB, YmidPoint_AB, XmidPoint_BC, YmidPoint_BC;

//     // variables for perpendicular bisectors
//     double m2_AB, m2_BC, b2_AB, b2_BC;

//     // RANSAC
//     cv::RNG rng; 
//     int min_point_separation = 10; // change to be relative to image size?
//     int colinear_tolerance = 1; // make sure points are not on a line
//     int radius_tolerance = 3; // change to be relative to image size?
//     int points_threshold = 10; //should always be greater than 4
//     //double min_circle_separation = 10; //reject a circle if it is too close to a previously found circle
//     //double min_radius = 10.0; //minimum radius for a circle to not be rejected

//     int x,y;
//     Point2d center;
//     double radius;

//     // Iterate
//     for(int iteration = 0; iteration < numIterations; iteration++) {
//         //std::cout << "RANSAC iteration: " << iteration << std::endl;

//         // get 4 random points
//         pointA = points[rng.uniform((int)0, (int)points.size())];
//         pointB = points[rng.uniform((int)0, (int)points.size())];
//         pointC = points[rng.uniform((int)0, (int)points.size())];
//         pointD = points[rng.uniform((int)0, (int)points.size())];

//         // calc lines
//         AB = norm(pointA - pointB);
//         BC = norm(pointB - pointC);
//         CA = norm(pointC - pointA);
//         DC = norm(pointD - pointC);

//         // one or more random points are too close together
//         if(AB < min_point_separation || BC < min_point_separation || CA < min_point_separation || DC < min_point_separation) continue;

//         //find line equations for AB and BC
//         //AB
//         m_AB = (pointB.y - pointA.y) / (pointB.x - pointA.x + 0.000000001); //avoid divide by 0
//         b_AB = pointB.y - m_AB*pointB.x;

//         //BC
//         m_BC = (pointC.y - pointB.y) / (pointC.x - pointB.x + 0.000000001); //avoid divide by 0
//         b_BC = pointC.y - m_BC*pointC.x;


//         //test colinearity (ie the points are not all on the same line)
//         if(abs(pointC.y - (m_AB*pointC.x + b_AB + colinear_tolerance)) < colinear_tolerance) continue;

//         //find perpendicular bisector
//         //AB
//         //midpoint
//         XmidPoint_AB = (pointB.x + pointA.x) / 2.0;
//         YmidPoint_AB = m_AB * XmidPoint_AB + b_AB;
//         //perpendicular slope
//         m2_AB = -1.0 / m_AB;
//         //find b2
//         b2_AB = YmidPoint_AB - m2_AB*XmidPoint_AB;

//         //BC
//         //midpoint
//         XmidPoint_BC = (pointC.x + pointB.x) / 2.0;
//         YmidPoint_BC = m_BC * XmidPoint_BC + b_BC;
//         //perpendicular slope
//         m2_BC = -1.0 / m_BC;
//         //find b2
//         b2_BC = YmidPoint_BC - m2_BC*XmidPoint_BC;

//         //find intersection = circle center
//         x = (b2_AB - b2_BC) / (m2_BC - m2_AB);
//         y = m2_AB * x + b2_AB;	
//         center = Point2d(x,y);
//         radius = cv::norm(center - pointB);

//         // /// geometry debug image
//         // if(false)
//         //     {
//         //         Mat debug_image = edges.clone();
//         //         cvtColor(debug_image, debug_image, CV_GRAY2RGB);

//         //         Scalar pink(255,0,255);
//         //         Scalar blue(255,0,0);
//         //         Scalar green(0,255,0);
//         //         Scalar yellow(0,255,255);
//         //         Scalar red(0,0,255);

//         //         // the 3 points from which the circle is calculated in pink
//         //         circle(debug_image, pointA, 3, pink);
//         //         circle(debug_image, pointB, 3, pink);
//         //         circle(debug_image, pointC, 3, pink);

//         //         // the 2 lines (blue) and the perpendicular bisectors (green)
//         //         line(debug_image,pointA,pointB,blue);
//         //         line(debug_image,pointB,pointC,blue);
//         //         line(debug_image,Point(XmidPoint_AB,YmidPoint_AB),center,green);
//         //         line(debug_image,Point(XmidPoint_BC,YmidPoint_BC),center,green);

//         //         circle(debug_image, center, 3, yellow); // center
//         //         circle(debug_image, center, radius, yellow);// circle

//         //         // 4th point check
//         //         circle(debug_image, pointD, 3, red);

//         //         imshow("ransac debug", debug_image);
//         //         waitKey(0);
//         //     }

//         if (radius < min_radius) continue;

//         //check if the 4 point is on the circle
//         if(abs(cv::norm(pointD - center) - radius) > radius_tolerance) continue;

//         // vote
//         std::vector<int> votes;
//         std::vector<int> no_votes;
//         for(int i = 0; i < (int)points.size(); i++) 
//             {
//                 double vote_radius = norm(points[i] - center);

//                 if(abs(vote_radius - radius) < radius_tolerance) 
//                     {
//                         votes.push_back(i);
//                     }
//                 else
//                     {
//                         no_votes.push_back(i);
//                     }
//             }

//         // check votes vs circle_threshold
//         if( (float)votes.size() / (2.0*CV_PI*radius) >= circle_threshold )
//             {
//                 circles.push_back(Vec3f(x,y,radius));

//                 // // voting debug image
//                 // if(false)
//                 //     {
//                 //         Mat debug_image2 = edges.clone();
//                 //         cvtColor(debug_image2, debug_image2, CV_GRAY2RGB);

//                 //         Scalar yellow(0,255,255);
//                 //         Scalar green(0,255,0);

//                 //         circle(debug_image2, center, 3, yellow); // center
//                 //         circle(debug_image2, center, radius, yellow);// circle

//                 //         // draw points that voted
//                 //         for(int i = 0; i < (int)votes.size(); i++)
//                 //             {
//                 //                 circle(debug_image2, points[votes[i]], 1, green);
//                 //             }

//                 //         imshow("ransac debug", debug_image2);
//                 //         waitKey(0);
//                 //     }

//                 // remove points from the set so they can't vote on multiple circles
//                 std::vector<Point2d> new_points;
//                 for(int i = 0; i < (int)no_votes.size(); i++)
//                     {
//                         new_points.push_back(points[no_votes[i]]);
//                     }
//                 points.clear();
//                 points = new_points;		
//             }

//         // stop RANSAC if there are few points left
//         if((int)points.size() < points_threshold)
//             break;
//     }

//     return;
// }



// //This colors the segmentations
// static void floodFillPostprocess( Mat& img, const Scalar& colorDiff=Scalar::all(1) )
// {
//     CV_Assert( !img.empty() );
//     RNG rng = theRNG();
//     Mat mask( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0) );
//     for( int y = 0; y < img.rows; y++ )
//     {
//         for( int x = 0; x < img.cols; x++ )
//         {
//             if( mask.at<uchar>(y+1, x+1) == 0 )
//             {
//                 Scalar newVal( rng(256), rng(256), rng(256) );
//                 floodFill( img, mask, Point(x,y), newVal, 0, colorDiff, colorDiff );
//             }
//         }
//     }
// }

// void compute_edges(cv::Mat& _img) { 

//     /// Generate grad_x and grad_y
//     int scale = 1;
//     int delta = 0;
//     int ddepth = CV_32F;
//     Mat grad, grad_x, grad_y, abs_grad_x, abs_grad_y;

//     // Convert to HSV space for sobel detection
//     cv::Mat img = _img.clone();
//     cv::Mat imgb;
//     cv::Mat display = img.clone();
//     cv::medianBlur(img, imgb, 7);

//     cv::blur(img, img, cv::Size(3,3));

//     cv::Mat3b imgb_th = imgb >= 200;
//     cv::Rect rect(0,0,imgb.cols,state->hood_height);
//     cv::Mat roi = imgb_th(rect);
//     roi = cv::Scalar(0,0,0);
//     cv::imshow("img-blur", imgb_th);

//     // cv::Mat res; 
//     // pyrMeanShiftFiltering( img, res, 10, 30, 3 );
//     // floodFillPostprocess( res, Scalar::all(2) );
//     // imshow( "meanshift", res );


//     cv::Mat hsv;
//     cv::cvtColor(img, hsv, CV_BGR2HSV);
 
//     /// Gradient X
//     cv::Sobel( img, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );   
//     cv::convertScaleAbs( grad_x, abs_grad_x );

//     /// Gradient Y  
//     cv::Sobel( img, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );   
//     cv::convertScaleAbs( grad_y, abs_grad_y );

//     /// Total Gradient (approximate)
//     cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

//     // cv::cvtColor(grad, grad, CV_HSV2BGR);
//     std::vector<cv::Mat> channels; 
//     cv::split(grad, channels);
//     assert(channels.size() == 3);

//     // Find max of R,G,B channels
//     cv::Mat max_grad; 
//     cv::max(channels[0], channels[1], max_grad); 
//     cv::max(max_grad, channels[2], max_grad);

//     // Find the edges
//     cv::Mat edges; 
//     cv::Canny(max_grad, edges, 240, 255, 3); 

//     vector<vector<Point> > contours, filtered_contours;
//     vector<Vec4i> hierarchy;

//     cv::Mat1b imgb_th1b; 
//     cv::cvtColor(imgb_th, imgb_th1b, CV_BGR2GRAY);
//     findContours( imgb_th1b, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

//     Mat dst = Mat::zeros(img.size(), CV_8UC3);
//     Mat filtered_edges = Mat::zeros(img.size(), CV_8UC1);
//     if( !contours.empty() && !hierarchy.empty() ) {
//         // iterate through all the top-level contours,
//         // draw each connected component with its own random color
//         int idx = 0;
//         for( ; idx >= 0; idx = hierarchy[idx][0] ) {
//             Scalar color( (rand()&255), (rand()&255), (rand()&255) );
//             drawContours( dst, contours, idx, color, CV_FILLED, 8, hierarchy );                
//             drawContours( filtered_edges, contours, idx, cv::Scalar(255), 1, 8, hierarchy );                

//             cv::Point2f mu_pt(0.f,0.f);
//             for (int j=0; j<contours[idx].size(); j++) 
//                 mu_pt.x += contours[idx][j].x, mu_pt.y += contours[idx][j].y;
//             mu_pt *= 1.f / contours[idx].size();

//             cv::putText(filtered_edges, cv::format("%i", idx), mu_pt, 0, .3, cv::Scalar(200), 1);
//         }
//     }
//     std::cerr << "filtered contours: " << filtered_contours.size() << std::endl;
//     cv::imshow("filtered_edges", filtered_edges);

//     // Perform probabilistic hough transform
//     std::vector<cv::Vec4i> wheel_lines;
//     cv::HoughLinesP(filtered_edges, wheel_lines, 1, CV_PI/180, 10, 20, 20); 
//     // std::cerr << "wheel_lines: " << wheel_lines.size() << std::endl;
//     // std::cerr << "wrect: " << wrect.center << std::endl;
        
//     std::vector<cv::Point2f> spoke_angles; 
//     for( size_t i = 0; i < wheel_lines.size(); i++ ) {
//         cv::Vec4i l = wheel_lines[i];

//         cv::Point2f p1 = cv::Point(l[0], l[1]); 
//         cv::Point2f p2 = cv::Point(l[2], l[3]); 

//         cv::Point2f v = p1 - p2; 
//         float norm = cv::norm(v); 
//         if (norm > 0) v *= 1.f / norm; 

//         // cv::Point2f pc = p1 + (v.dot(wrect.center-p1))*v;
//         // float d = cv::norm(pc-wrect.center);
//         // if (d > 50) continue;

//         // if (v.y < 0) { v.x = -v.x; v.y = fabs(v.y); }
//         // spoke_angles.push_back(v);

//         cv::line( dst, p1, p2, cv::Scalar(255,255,255), 2, CV_AA);
//         // cv::circle( dst, pc, 4, cv::Scalar(128,128,128), 1, CV_AA);
//         // cv::circle( dst, wrect.center, 8, cv::Scalar(100,100,100), 1, 8, 0 );
//     }


//     cv::imshow("CComps", dst);

//     // // Perform probabilistic hough transform
//     // for (int j=0; j<filtered_contours.size(); j++) { 
//     //     std::vector<cv::Vec3f> circles;
//     //     circleRANSAC(filtered_contours[j], circles, .1, 20, 100); 
//     //     std::cerr << "circles: " << circles.size() << std::endl;

//     //     /// Draw the circles detected
//     //     for( size_t i = 0; i < circles.size(); i++ ) {
//     //         cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//     //         int radius = cvRound(circles[i][2]);
//     //         // circle center
//     //         cv::circle( display, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
//     //         // circle outline
//     //         cv::circle( display, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
//     //     }
//     // }



//     // // Compute the variation of color across the edge
//     // const float off = 30; // in pixels

//     // std::vector<int> test(0);
//     // std::vector<SegmentInfo> line_segments_info(lines.size()); 
//     // for( size_t i = 0; i < lines.size(); i++ ) {
//     //     cv::Vec4i l = lines[i];

//     //     cv::Point2f p1 = cv::Point(l[0], l[1]); 
//     //     cv::Point2f p2 = cv::Point(l[2], l[3]); 

//     //     cv::Point2f v = p1 - p2; 
//     //     if (fabs(v.x) > fabs(v.y)) v.x = -v.x;
//     //     else v.y = -v.y;
//     //     float norm = cv::norm(v); 
//     //     if (norm > 0) v *= 1.f / norm; 

//     //     cv::Point2f pl1 = p1 + off * v; 
//     //     cv::Point2f pr1 = p1 - off * v; 

//     //     cv::Point2f pl2 = p2 + off * v; 
//     //     cv::Point2f pr2 = p2 - off * v; 

//     //     cv::Point2f cl = (p1 + p2)*0.5 + off * v; 
//     //     cv::Point2f cr = (p1 + p2)*0.5 - off * v; 

//     //     int rw = 20, rh = 20; 
//     //     cv::Rect r1(cl.x-rw/2, cl.y-rh/2, rw, rh);
//     //     r1 &= cv::Rect(0, 0, img.cols, img.rows);
//     //     cv::Mat patch1 = cv::Mat(hsv, r1);
//     //     cv::Scalar _patch_mean1 = cv::mean(patch1);
//     //     cv::Vec3f patch_mean1(_patch_mean1[0], _patch_mean1[1], _patch_mean1[2]);

//     //     cv::Rect r2(cr.x-rw/2, cr.y-rh/2, rw, rh);
//     //     r2 &= cv::Rect(0, 0, img.cols, img.rows);
//     //     cv::Mat patch2 = cv::Mat(hsv, r2);
//     //     cv::Scalar _patch_mean2 = cv::mean(patch2);
//     //     cv::Vec3f patch_mean2(_patch_mean2[0], _patch_mean2[1], _patch_mean2[2]);

//     //     cv::Vec3f d1 = state->patch_mean1 - patch_mean1; 
//     //     cv::Vec3f d2 = state->patch_mean1 - patch_mean2; 
//     //     if (cv::norm(d1) < cv::norm(d2)) 
//     //         d2 = state->patch_mean2 - patch_mean2; 
//     //     else
//     //         d1 = state->patch_mean2 - patch_mean1;

//     //     cv::Vec3f col = d1+d2;
//     //     col[0] = fabs(col[0]), col[1] = fabs(col[1]), col[2] = fabs(col[2]);

//     //     if (cv::norm(d1) + cv::norm(d2) < 20)
//     //         cv::line( display, p1, p2, cv::Scalar(0,0,255), 1, CV_AA);
//     //     cv::line( display, cl, cr, cv::Scalar(col), 1, CV_AA);
//     //     cv::rectangle(display, r1.tl(), r1.br(), cv::Scalar(0,255,255), 1, CV_AA);
//     //     cv::rectangle(display, r2.tl(), r2.br(), cv::Scalar(0,255,255), 1, CV_AA);
//     //     // cv::line( display, pl1, pl2, cv::Scalar(patch_mean1), 3, CV_AA);
//     //     // cv::line( display, pr1, pr2, cv::Scalar(patch_mean2), 3, CV_AA);
//     //     // cv::line( display, pl2, pr2, cv::Scalar(0,255,255), 1, CV_AA);
//     //     cv::putText(display, cv::format("%f %f", cv::norm(d1), cv::norm(d2)),
//     //                 (p1 + p2) * 0.5 , 0, .3, cv::Scalar(200,200,200), 1);
//     // }


//     cv::imshow("max_grad", max_grad);
//     cv::imshow("edges", edges);

//     cv::imshow("grad", grad);
//     cv::imshow("img", display);
//     return;
// }
