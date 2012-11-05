#ifndef IMSHOW_UTILS_H
#define IMSHOW_UTILS_H

#include <opencv2/opencv.hpp>

#include <pthread.h>
#include <deque>
#include <math.h>
#include <stdio.h>
#include <bot_core/bot_core.h>
#include <map>



namespace opencv_utils {

    struct image_info { 
        cv::Rect roi;
        cv::Mat img;
        std::string name;
        
        double area;
        bool placed;
        image_info() : placed(false), name(""), area(0), img(cv::Mat()) {}
        image_info(const std::string& _name, const cv::Mat& _img) : name(_name), placed(false), area(_img.rows * _img.cols) { 
            if (_img.channels() == 3)
                img = _img.clone();
            else 
                cvtColor(_img, img, CV_GRAY2BGR);
        }
    };

    class OpenCVImageViewer { 

        cv::Mat3b frame;

        int tile_width, tile_height;
        int im_w, im_h;
        
    public: 
        std::map<std::string, image_info> image_map;
        OpenCVImageViewer();
        ~OpenCVImageViewer();
        void configure_grid();
        void draw_image_to_frame(cv::Mat& frame, image_info& im_info);
        // void button_callback(int b, void* data);
        void imshow(const std::string& name, const cv::Mat& img);
        void displayStatusBar(const std::string& text, int delayms) { 
            cv::displayStatusBar("OpenCV Viewer", text, delayms);
        }
        void displayOverlay(const std::string& text, int delayms) { 
            cv::displayOverlay("OpenCV Viewer", text, delayms);
        }

    };
    
    static OpenCVImageViewer viewer;
    // static MouseEvent mouse;

    static void imshow(const std::string& name, const cv::Mat& img) {
        // std::cerr << "viewer show" << std::endl;
        if (&viewer)
            viewer.imshow(name, img);
        // cv::imshow(name, img);
    }
    static void displayStatusBar(const std::string& text, int delayms) { 
        if (&viewer)
            viewer.displayStatusBar(text, delayms);
    }
    static void displayOverlay(const std::string& text, int delayms) { 
        if (&viewer)
            viewer.displayOverlay(text, delayms);
    }

    cv::Mat drawCorrespondences(const cv::Mat& img1, const std::vector<cv::Point2f>& features1, 
                                const cv::Mat& img2, const std::vector<cv::Point2f>& features2);

}
#endif
