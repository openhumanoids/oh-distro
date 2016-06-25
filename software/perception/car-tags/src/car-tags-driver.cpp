#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <AprilTags/apriltag.h>
#include <AprilTags/common/image_u8.h>
#include <AprilTags/tag36h11.h>
#include <AprilTags/common/zarray.h>
#include <AprilTags/common/getopt.h>

#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/bot_core/images_t.hpp>
#include <lcmtypes/bot_core/image_t.hpp>

#include <bot_core/camtrans.h>
#include <bot_param/param_util.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <vector>
#include <iostream>

#include <Eigen/Dense>

struct TagMatch {
    int id; 
    cv::Point2d p0, p1, p2, p3;
    Eigen::Matrix3d H;
};

Eigen::Isometry3d getRelativeTransform(TagMatch const& match, Eigen::Matrix3d const & K, double tag_size) 
{
  std::vector<cv::Point3f> objPts;
  std::vector<cv::Point2f> imgPts;
  double s = tag_size/2.;
  objPts.push_back(cv::Point3f(-s,-s, 0));
  objPts.push_back(cv::Point3f( s,-s, 0));
  objPts.push_back(cv::Point3f( s, s, 0));
  objPts.push_back(cv::Point3f(-s, s, 0));


  imgPts.push_back(match.p0);
  imgPts.push_back(match.p1);
  imgPts.push_back(match.p2);
  imgPts.push_back(match.p3);

  cv::Mat rvec, tvec;
  cv::Matx33f cameraMatrix(
                           K(0,0), 0, K(0,2),
                           0, K(1,1), K(1,2),
                           0,  0,  1);

  cv::Vec4f distParam(0,0,0,0); 
  cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
  cv::Matx33d r;
  cv::Rodrigues(rvec, r);
  Eigen::Matrix3d wRo;
  wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

  Eigen::Isometry3d T; 
  T.linear() = wRo;
  T.translation() << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  return T;
}


bot_core::rigid_transform_t encodeLCMFrame(Eigen::Isometry3d const & frame) 
{
    Eigen::Vector3d t(frame.translation());
    Eigen::Quaterniond r(frame.rotation());

    bot_core::rigid_transform_t msg;
    msg.quat[0] = r.w();
    msg.quat[1] = r.x();
    msg.quat[2] = r.y();
    msg.quat[3] = r.z();

    msg.trans[0] = t[0];
    msg.trans[1] = t[1];
    msg.trans[2] = t[2];

    return msg;
}


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
        tag_size = getopt_get_double(getopt, "size");
    }

    ~AprilTagDetector() {

        apriltag_detector_destroy(td);
        tag36h11_destroy(tf);
    }

    std::vector<TagMatch> detectTags(image_u8_t *im) {

        const int hamm_hist_max = 10;

        int hamm_hist[hamm_hist_max];
        memset(hamm_hist, 0, sizeof(hamm_hist));
        zarray_t *detections = apriltag_detector_detect(td, im);

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
            tag_match.id = det->family->d*det->family->d;
            tag_match.p0 = cv::Point2d(det->p[0][0], det->p[0][1]);
            tag_match.p1 = cv::Point2d(det->p[1][0], det->p[1][1]);
            tag_match.p2 = cv::Point2d(det->p[2][0], det->p[2][1]);
            tag_match.p3 = cv::Point2d(det->p[3][0], det->p[3][1]);

            Eigen::Map<Eigen::Matrix3d> H_map(det->H->data);
            tag_match.H = H_map.transpose();
            tag_matches.push_back(tag_match);
            hamm_hist[det->hamming]++;
        }

        apriltag_detections_destroy(detections);

        if (!quiet) {
            timeprofile_display(td->tp);
            printf("nedges: %d, nsegments: %d, nquads: %d\n", td->nedges, td->nsegments, td->nquads);
            printf("Hamming histogram: ");
            for (int i = 0; i < hamm_hist_max; i++)
                printf("%5d", hamm_hist[i]);
            printf("%12.3f", timeprofile_total_utime(td->tp) / 1.0E3);
            printf("\n");
        }
        
        return tag_matches;
    }
    
    double getTagSize() const {
        return tag_size;
    }

    private:
    int quiet;
    double tag_size;
    apriltag_family_t *tf;
    apriltag_detector_t *td;
    getopt_t *getopt;
};


class CameraListener {
    public:
        std::string publish_img_with_matches_channel;

    void setDetector(AprilTagDetector* detector) {
        mDetector = detector;
    }

    bool setup(bool show_window, bool publish_img_with_matches = false) {
        mBotWrapper.reset(new drc::BotWrapper());

        while (!mBotWrapper->getBotParam()) {
            std::cout << "Re-trying ... " << std::endl;
            mBotWrapper->setDefaults();
        }
        

        mLcmWrapper.reset(new drc::LcmWrapper(mBotWrapper->getLcm()));
        mLcmWrapper->get()->subscribe("CAMERA", &CameraListener::onCamera, this);

        mCamTransLeft = bot_param_get_new_camtrans(mBotWrapper->getBotParam(),"CAMERA_LEFT");
        
        K = Eigen::Matrix3d::Identity();

        K(0,0) = bot_camtrans_get_focal_length_x(mCamTransLeft);
        K(1,1) = bot_camtrans_get_focal_length_y(mCamTransLeft);
        K(0,2) = bot_camtrans_get_principal_x(mCamTransLeft);
        K(1,2) = bot_camtrans_get_principal_y(mCamTransLeft);

        mPublishImageWithMatches = publish_img_with_matches;

        mShowWindow = show_window;
        return true;
    }  
    
    void start() {
        mLcmWrapper->startHandleThread(true);
    }

    void onCamera(const lcm::ReceiveBuffer* buffer, const std::string& channel,
                const bot_core::images_t* msg) {
        cv::Mat image;
        decodeImage(msg, image);
        image_u8_t *image_u8 = fromCvMat(image);
        
        std::vector<TagMatch> tags = mDetector->detectTags(image_u8);
        cv::cvtColor(image, image, CV_GRAY2RGB);
        for (int i = 0; i < tags.size(); i++) { 

            if (mShowWindow) {
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
            }

            Eigen::Isometry3d tag_to_camera = getRelativeTransform(tags[i], K, mDetector->getTagSize());
            bot_core::rigid_transform_t tag_to_camera_msg = encodeLCMFrame(tag_to_camera);
            tag_to_camera_msg.utime = msg->utime;
            mLcmWrapper->get()->publish("APRIL_TAG_TO_CAMERA_LEFT", &tag_to_camera_msg);
            break;
        }
        if (mShowWindow) {
            cv::imshow("detections", image);
            cv::waitKey(1);
        }
        if (mPublishImageWithMatches) {
            bot_core::image_t img_with_matches;
            img_with_matches.utime = msg->utime;
            img_with_matches.width = image.size().width;
            img_with_matches.height = image.size().height;
            img_with_matches.row_stride = 3 * img_with_matches.width;
            img_with_matches.nmetadata = 0;
            std::vector<int> params;
            params.push_back(cv::IMWRITE_JPEG_QUALITY);
            params.push_back(90);
            img_with_matches.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
            cv::imencode(".jpg", image, img_with_matches.data, params);
            img_with_matches.size = img_with_matches.data.size();
            mLcmWrapper->get()->publish(publish_img_with_matches_channel.c_str(), &img_with_matches);
        }
        
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
    bool mShowWindow;
    bool mPublishImageWithMatches;
    AprilTagDetector *mDetector;
    drc::LcmWrapper::Ptr mLcmWrapper;
    drc::BotWrapper::Ptr mBotWrapper;
    BotCamTrans* mCamTransLeft;
    Eigen::Matrix3d K;
};



int main(int argc, char *argv[])
{
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'w', "window", 1, "Show the detected tags in a window");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");
    getopt_add_double(getopt, 's', "size", "0.1735", "Physical side-length of the tag (meters)");
    getopt_add_bool(getopt, 'p', "publish-img-with-match", 0, "Publish image with overlayed match(es)");
    getopt_add_string(getopt, 'c', "publish-img-with-match-channel", "CAMERA_APRIL_TAG_MATCHES", "LCM channel for image with overlayed match(es)");
    

    if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }  

    AprilTagDetector tag_detector(getopt);
    CameraListener camera_listener;

    camera_listener.publish_img_with_matches_channel = getopt_get_string(getopt, "publish-img-with-match-channel");

    if (camera_listener.setup(getopt_get_bool(getopt, "window"), getopt_get_bool(getopt, "publish-img-with-match"))) {
        camera_listener.setDetector(&tag_detector);
        camera_listener.start();
    }

    return 0;
}
