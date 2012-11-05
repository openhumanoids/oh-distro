
#ifndef KINECT_OPENCV_UTILS_H_
#define KINECT_OPENCV_UTILS_H_

#include <assert.h>
#include <math.h>
#include <algorithm>
#include <iostream>

// Opencv includes
#include <opencv2/opencv.hpp>

#include <lcm/lcm.h>
#include <lcmtypes/kinect_skeleton_msg_t.h>
#include <lcmtypes/kinect_link_msg_t.h>
#include <lcmtypes/kinect_frame_msg_t.h>
#include <bot_core/bot_core.h>
#include <image_utils/jpeg.h>
#include <image_utils/pixels.h>

#include <zlib.h>
#include <sys/time.h>

#define WIDTH 640
#define HEIGHT 480
#define BOX_W 50
// ===================== kinect_data ========================
// kinect_data struct for acquiring images
// simplifies getting rgb, gray images including 
// other derivates without having to do redundant 
// computation
// ==========================================================
static cv::Rect fake_box(320,240,BOX_W,BOX_W);
static float dx=10, dy=10;

struct kinect_data {
public: 
    lcm_t* lcm;

    int width, height;
    double timestamp;
    bool fake_kinect;
    // cv::Rect fake_box;
    cv::RNG rng;
    
    double fx_inv, cx, cy;
    bool calib_set;
    
    // Skeleton
    kinect_skeleton_msg_t* skeleton_msg;

// RGB,D images & Disparity Map
    cv::Mat3b rgb;
    cv::Mat_<uint16_t> depth;

private: 
    
    cv::Mat_<cv::Vec2f> disparity_map;

    // Image Derivatives
    cv::Mat3b rgb_blur, hls;
    cv::Mat1b gray, gray_blur;

    // Relevant lcm/image information
    int npixels;
    int8_t depth_data_format;

    bool got_gray, got_gray_blur, got_rgb_blur, got_hls;

    // Calibration
    static const float fxtoz = 1.11147, fytoz = 0.8336;

public: 
kinect_data(bool _fake_kinect = false) : timestamp(0), width(0), height(0), npixels(0), depth_data_format(0), lcm(NULL), 
        got_gray(false), got_gray_blur(false), got_rgb_blur(false), got_hls(false), 
        skeleton_msg(NULL), calib_set(false), fake_kinect(_fake_kinect) { 
    // fake_box = cv::Rect(320,240,30,30);
}

    void set_calib(float depth_fx, float depth_cx, float depth_cy) { 
        fx_inv = 1.f / depth_fx, cx = depth_cx, cy = depth_cy;
        calib_set = true;
    }

    ~kinect_data() { 
        if (skeleton_msg)
            kinect_skeleton_msg_t_destroy(skeleton_msg);
        //std::cerr << "Destroying lcm handler safely" << std::endl;
    }

    void create_disparity_map() { 
        // Create disparity map for x,y,disp to XYZ
        if (!disparity_map.data) { 
            disparity_map.create(height, width);
            // std::cerr << "Creating disparity map" << std::endl;
            // Full disparity map

            if (calib_set) { 
                for (int i = 0; i < height; i++)
                    for(int j = 0; j < width; j++) 
                        disparity_map(i,j)[0] = (j * fx_inv - cx * fx_inv), 
                            disparity_map(i,j)[1] = (i * fx_inv - cy * fx_inv);
            } else { 
                for (int i = 0; i < height; i++)
                    for(int j = 0; j < width; j++) 
                        disparity_map(i,j)[0] = (j * 1.f / width - 0.5) * fxtoz, 
                            disparity_map(i,j)[1] = (i * 1.f / height - 0.5) * fytoz;
            }
        }
    }

    void update_rgb(const kinect_frame_msg_t* msg) { 
        if (!rgb.data) { 
            assert(width == WIDTH && height == HEIGHT);
            rgb.create(height, width);
        }

        if (fake_kinect) { 
            rgb = cv::Mat3b::zeros(height, width);

            int w = BOX_W;
            int excess = fake_box.x + w - width + 1;
            if (excess > 0) {
                fake_box.x = width - 1 - excess - w;
                dx *= -1;
            } else if (fake_box.x < 0) {
                fake_box.x = - fake_box.x;
                dx *= -1;
            }

            fake_box.y += dy;
            excess = fake_box.y + w - height + 1;
            if (excess > 0) {
                fake_box.y = height - 1 - excess - w;
                dy *= -1;
            } else if (fake_box.y < 0) {
                fake_box.y = - fake_box.y;
                dy *= -1;
            }

            rectangle(rgb, fake_box, cv::Scalar(255,255,255), -1);
        } else { 
            memcpy(rgb.data, msg->image.image_data, sizeof(uint8_t)*npixels*3);
            cvtColor(rgb, rgb, CV_BGR2RGB);
        }


        got_hls = false;
        got_gray = false;
        got_gray_blur = false;
        got_rgb_blur = false;
    }

    void update_rgb(const bot_core_image_t* msg) { 
        if (!rgb.data) { 
            assert(width == WIDTH && height == HEIGHT);
            rgb.create(height, width);
        }

        if (fake_kinect) { 
            rgb = cv::Mat3b::zeros(height, width);

            int w = BOX_W;
            int excess = fake_box.x + w - width + 1;
            if (excess > 0) {
                fake_box.x = width - 1 - excess - w;
                dx *= -1;
            } else if (fake_box.x < 0) {
                fake_box.x = - fake_box.x;
                dx *= -1;
            }

            fake_box.y += dy;
            excess = fake_box.y + w - height + 1;
            if (excess > 0) {
                fake_box.y = height - 1 - excess - w;
                dy *= -1;
            } else if (fake_box.y < 0) {
                fake_box.y = - fake_box.y;
                dy *= -1;
            }

            rectangle(rgb, fake_box, cv::Scalar(255,255,255), -1);
        } else { 
            memcpy(rgb.data, msg->data, sizeof(uint8_t)*npixels*3);
            cvtColor(rgb, rgb, CV_BGR2RGB);
        }

        got_hls = false;
        got_gray = false;
        got_gray_blur = false;
        got_rgb_blur = false;
    }


    void update_depth(const kinect_frame_msg_t* msg) { 
        if (!depth.data) { 
            assert(width == WIDTH && height == HEIGHT);
            depth.create(height, width);
            create_disparity_map();
        }
        if(msg->depth.compression == KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
            memcpy(depth.data, (uint16_t*) msg->depth.depth_data, sizeof(uint16_t)*npixels);
        } else if (msg->depth.compression == KINECT_DEPTH_MSG_T_COMPRESSION_ZLIB) {
            printf("decompressing\n");
            unsigned long dlen = msg->depth.uncompressed_size;
            uncompress((uint8_t*)depth.data, &dlen, msg->depth.depth_data, msg->depth.depth_data_nbytes);
        }
    }

    void update(const kinect_frame_msg_t* msg) { 
        
        timestamp = msg->timestamp;
        
        // Assuming depth image size and RGB image size are the same
        width = msg->depth.width;
        height = msg->depth.height;
        npixels = width * height;
        depth_data_format = msg->depth.depth_data_format;

        assert(msg->depth.width == msg->image.width && msg->depth.height == msg->image.height);

        update_rgb(msg);
        update_depth(msg);
    }

    void update(const bot_core_image_t* msg) { 
        
        timestamp = msg->utime;
        
        // Assuming depth image size and RGB image size are the same
        width = msg->width;
        height = msg->height;
        npixels = width * height;
        depth_data_format = 0;

        update_rgb(msg);
    }

    const cv::Mat3b& getRGB() { 
        return rgb; 
    }

    const cv::Mat_<uint16_t>& getDepth() { 
        return depth; 
    }

    const cv::Mat3b& getBlurredRGB() { 
        if (!got_rgb_blur) { 
            // blur(rgb, rgb_blur, cv::Size(5,5), cv::Point(-1,-1));
            // medianBlur(rgb, rgb_blur, 5);
            bilateralFilter(rgb, rgb_blur, 5, 10, 10);
            got_rgb_blur = true;
        }
        return rgb_blur;
    }

    const cv::Mat3b& getHLS() { 
        if (!got_hls) { 
            if (!got_rgb_blur) { 
                getBlurredRGB();
            }
            cvtColor(rgb_blur, hls, CV_BGR2HLS);
            got_hls = true;
        }
        return hls;
    }
    
    const cv::Mat1b& getGray() { 
        if (!got_gray) { 
            cvtColor(rgb, gray, CV_RGB2GRAY);
            got_gray = true;
            got_gray_blur = false;
        }
        return gray;
    }

    const cv::Mat1b& getBlurredGray() { 
        if (!got_gray)
            getGray();
        if (!got_gray_blur) { 
            blur(gray, gray_blur, cv::Size(3,3), cv::Point(-1,-1));
            //medianBlur(gray, gray_blur, 5);
            got_gray_blur = true;
        }
        return gray_blur;
    }

    void showRGB() { 
        imshow("Kinect RGB", rgb);
    }

    void showDepth() { 
        double min = 0, max = 0;
        minMaxLoc(depth, &min, &max);
        float scale = 1.f / (max - min);

        cv::Mat_<float> depthf = cv::Mat_<float>::zeros(depth.size());
        depthf = (depth - min) * scale;

        imshow("Kinect Depth", depthf);
    }

    inline cv::Point3f getXYZ(cv::Point2f& p) const { 
        double d = getDepth(int(p.y),int(p.x));
        cv::Vec2f dxy = disparity_map(int(p.y),int(p.x));
        return cv::Point3f(dxy[0]*d, dxy[1]*d, d);
    }

    inline cv::Vec3f getXYZ(int y, int x) const { 
        double d = getDepth(y,x);
        cv::Vec2f dxy = disparity_map(y,x);
        return cv::Vec3f(dxy[0]*d, dxy[1]*d, d);
    }

    inline double getDepth(int y, int x) const {
        assert (depth_data_format == KINECT_DEPTH_MSG_T_DEPTH_MM);
        return depth(y, x) / 1000.0;
    }

    inline cv::Vec3b getScalarRGB(int y, int x) const {
        return rgb(y, x);
    }

    inline cv::Vec2f getDisparity(int y, int x) const {
        return disparity_map(y,x);
    }

    inline bool isLive() { return (!(rgb.empty() && depth.empty())); } 
};
// ========================================================== // 

static inline float median(std::vector<float> &v) {
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
}

static int64_t
timestamp_us (void)
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

#endif /* KINECT_OPENCV_UTILS_H_ */
