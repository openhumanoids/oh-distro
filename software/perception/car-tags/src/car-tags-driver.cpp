#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <AprilTags/apriltag.h>
#include <AprilTags/common/image_u8.h>

#include <AprilTags/tag36h11.h>
#include <AprilTags/tag36h10.h>
#include <AprilTags/tag36artoolkit.h>
#include <AprilTags/tag25h9.h>
#include <AprilTags/tag25h7.h>

#include <AprilTags/common/zarray.h>
#include <AprilTags/common/getopt.h>

#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/drc/robot_state_t.hpp>
#include <lcmtypes/bot_core/images_t.hpp>
#include <lcmtypes/bot_core/image_t.hpp>

#include <vector>
#include <iostream>

#include <Eigen/Dense>

struct TagMatch { 
    cv::Point2d p0, p1, p2, p3;
    Eigen::Matrix3d H;
};

class AprilTagDetector {
    public:
    AprilTagDetector(getopt_t *options) : getopt(options) {
        tf = tag36h11_create();
        tf->black_border = getopt_get_int(options, "border");
        td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);

        td->quad_decimate = getopt_get_double(getopt, "decimate");
        td->quad_sigma = getopt_get_double(getopt, "blur");
        td->nthreads = getopt_get_int(getopt, "threads");
        td->debug = getopt_get_bool(getopt, "debug");
        td->refine_edges = getopt_get_bool(getopt, "refine-edges");
        td->refine_decode = getopt_get_bool(getopt, "refine-decode");
        td->refine_pose = getopt_get_bool(getopt, "refine-pose");

        quiet = getopt_get_bool(getopt, "quiet");
    }

    ~AprilTagDetector() {

        apriltag_detector_destroy(td);
        tag36h11_destroy(tf);
    }

    std::vector<TagMatch> detectTags(image_u8_t *im) {

        const int hamm_hist_max = 10;

        
        int hamm_hist[hamm_hist_max];
        memset(hamm_hist, 0, sizeof(hamm_hist));
        //image_u8_t *im = image_u8_create_from_pnm(path);

        zarray_t *detections = apriltag_detector_detect(td, im);
        printf("Detections: %d\n", zarray_size(detections));
        std::vector<TagMatch> tag_matches;

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            if (!quiet)
                printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, goodness %8.3f, margin %8.3f\n",
                       i, det->family->d*det->family->d, det->family->h, det->id, det->hamming, det->goodness, det->decision_margin);

            for (int x = 0; x < 3 ; x++) {
                image_u8_draw_line(im, det->p[x][0], det->p[x][1], det->p[x+1][0], det->p[x+1][1], 255, 10);
            }
            TagMatch tag_match;
            tag_match.p0 = cv::Point2d(det->p[0][0], det->p[0][1]);
            tag_match.p1 = cv::Point2d(det->p[1][0], det->p[1][1]);
            tag_match.p2 = cv::Point2d(det->p[2][0], det->p[2][1]);
            tag_match.p3 = cv::Point2d(det->p[3][0], det->p[3][1]);
            Eigen::Map<Eigen::Matrix3d> H_map(det->H->data);
            tag_match.H = H_map.transpose();
            tag_matches.push_back(tag_match);

            //image_u8_draw_line(im, det->p[3][0], det->p[3][1], det->p[0][0], det->p[0][1], 255, 10);
            //image_u8_write_pnm(im, "test.pnm");
            hamm_hist[det->hamming]++;
        }

        apriltag_detections_destroy(detections);

        if (!quiet) {
            timeprofile_display(td->tp);
            printf("nedges: %d, nsegments: %d, nquads: %d\n", td->nedges, td->nsegments, td->nquads);
        }

        if (!quiet)
            printf("Hamming histogram: ");

        for (int i = 0; i < hamm_hist_max; i++)
            printf("%5d", hamm_hist[i]);

        if (quiet) {
            printf("%12.3f", timeprofile_total_utime(td->tp) / 1.0E3);
        }

        printf("\n");
        return tag_matches;
    }
    private:
    int quiet;
    apriltag_family_t *tf;
    apriltag_detector_t *td;
    getopt_t *getopt;
};


class CameraListener {
    public:

    void setDetector(AprilTagDetector* detector) {
        mDetector = detector;
    }

    void setup() {
        mBotWrapper.reset(new drc::BotWrapper());
        mLcmWrapper.reset(new drc::LcmWrapper(mBotWrapper->getLcm()));
        mLcmWrapper->get()->subscribe("CAMERA", &CameraListener::onCamera, this);
    }  
    
    void start() {
        mLcmWrapper->startHandleThread(true);
    }

    void onCamera(const lcm::ReceiveBuffer* buffer, const std::string& channel,
                const bot_core::images_t* msg) {
        printf("got %d images\n", msg->n_images);
        cv::Mat image;
        decodeImage(msg, image);
        image_u8_t *image_u8 = fromCvMat(image);
        
        std::vector<TagMatch> tags = mDetector->detectTags(image_u8);
        cv::cvtColor(image, image, CV_GRAY2RGB);
        for (int i = 0; i < tags.size(); i++) { 

            cv::line(image, tags[i].p0, tags[i].p1, cv::Scalar(255,0,0), 2, CV_AA);
            cv::line(image, tags[i].p1, tags[i].p2, cv::Scalar(0,255,0), 2, CV_AA);
            cv::line(image, tags[i].p2, tags[i].p3, cv::Scalar(0,0,255), 2, CV_AA);
            cv::line(image, tags[i].p3, tags[i].p0, cv::Scalar(0,0,255), 2, CV_AA);
            

            Eigen::Vector3d x_axis(2,0,1);
            Eigen::Vector3d y_axis(0,2,1);
            Eigen::Vector3d origin(0,0,1);

            Eigen::Vector3d px = tags[i].H * x_axis;
            Eigen::Vector3d py = tags[i].H * y_axis;
            Eigen::Vector3d o  = tags[i].H * origin;

            px/= px[2];
            py/= py[2];
            o/= o[2];

            cv::line(image, cv::Point2d(o[0], o[1]), cv::Point2d(px[0], px[1]), cv::Scalar(255,0,255), 1, CV_AA);
            cv::line(image, cv::Point2d(o[0], o[1]), cv::Point2d(py[0], py[1]), cv::Scalar(255,255,0), 1, CV_AA);

            std::cout << tags[i].H << std::endl;
        }
        cv::imshow("detections", image);
        cv::waitKey(1);
        image_u8_destroy(image_u8);
    }

    void decodeImage(const bot_core::images_t* msg, cv::Mat & decoded_image) {
        bot_core::image_t* leftImage = NULL;
        for (int i = 0; i < msg->n_images; ++i) {
          if (msg->image_types[i] == bot_core::images_t::LEFT) {
            leftImage = (bot_core::image_t*)(&msg->images[i]);
            decoded_image = leftImage->pixelformat == leftImage->PIXEL_FORMAT_MJPEG ?
                cv::imdecode(cv::Mat(leftImage->data), -1) :
                cv::Mat(leftImage->height, leftImage->width, CV_8UC1, leftImage->data.data());
                if (decoded_image.channels() > 1) {
                    cv::cvtColor(decoded_image, decoded_image, CV_RGB2GRAY);
                }
                break;
          }
        }
    }
    
    image_u8_t *fromCvMat(const cv::Mat & img) { 
        image_u8_t *image_u8 = image_u8_create_alignment(img.cols, img.rows, img.step);
        int size = img.total() * img.elemSize();
        memcpy(image_u8->buf, img.data, size * sizeof(uint8_t));
        return image_u8;
    }

    private:
    AprilTagDetector *mDetector;
    drc::LcmWrapper::Ptr mLcmWrapper;
    drc::BotWrapper::Ptr mBotWrapper;
};



int main(int argc, char *argv[])
{
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

    if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }  

    AprilTagDetector tag_detector(getopt);
    CameraListener camera_listener;
    camera_listener.setup();
    camera_listener.setDetector(&tag_detector);
    camera_listener.start();

    return 0;
}
