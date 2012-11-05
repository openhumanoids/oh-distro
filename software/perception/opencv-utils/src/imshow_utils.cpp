#include <stdio.h>
#include "imshow_utils.hpp"
#include <sys/time.h>

using namespace cv;

namespace opencv_utils { 
#define WIDTH 1600
#define HEIGHT 900


OpenCVImageViewer::OpenCVImageViewer() { 
    cv::namedWindow("OpenCV Viewer", CV_GUI_EXPANDED);
    // setMouseCallback("OpenCV Viewer", on_mouse, &mouse);
    // setWindowProperty("OpenCV Viewer", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    frame = cv::Mat3b(HEIGHT, WIDTH);
    
}

OpenCVImageViewer::~OpenCVImageViewer() { 
}

static void button_callback(int b, void* data) {
    OpenCVImageViewer* viewer = (OpenCVImageViewer*)data;
    // std::cerr << viewer->image_map.size() << std::endl;
    // std::cerr << "button callback" << std::endl;
}

static bool image_info_sort_asc(const image_info* lhs, const image_info* rhs) { 
    return lhs->area > rhs->area;
}

static std::ostream& operator<<(std::ostream& os, cv::Rect& r) { 
    os << "[" << r.x << "," << r.y << "," << r.width << "," << r.height << "]";
    return os;
}

void OpenCVImageViewer::draw_image_to_frame(cv::Mat& frame, image_info& im_info) { 
    // Set the full ROI to zeros
    cv::Mat roi(frame, im_info.roi);
    roi = cv::Scalar::all(0);

    // Preserve aspect ratio
    int im_tw, im_th;
    double s1 = im_h * 1.f / im_info.img.rows;
    double s2 = im_w * 1.f / im_info.img.cols;
    double s = std::min(s1, s2);
    im_tw = int(s*im_info.img.cols), im_th = int(s*im_info.img.rows);

    cv::Rect roi_rect(im_info.roi.x+im_info.roi.width/2-im_tw/2, 
                      im_info.roi.y+im_info.roi.height/2-im_th/2, 
                      im_tw, im_th);
    // roi_rect &= im_info.roi;
    roi = cv::Mat(frame, roi_rect);

    cv::resize(im_info.img, roi, cv::Size(im_tw, im_th));    

    cv::rectangle(frame, Point(im_info.roi.x, im_info.roi.y+im_info.roi.height-15), 
                  Point(im_info.roi.x+im_info.roi.width, im_info.roi.y+im_info.roi.height), cv::Scalar(20,20,20), CV_FILLED);
    cv::putText(frame, cv::format("%s", im_info.name.c_str()),
                Point2f(im_info.roi.x+5,im_info.roi.y+im_info.roi.height-5), 0, 0.30, 
                cv::Scalar(255,255,255),1);
    cv::rectangle(frame, Point(im_info.roi.x, im_info.roi.y), 
                  Point(im_info.roi.x+im_info.roi.width, im_info.roi.y+im_info.roi.height), cv::Scalar(55,55, 55), 1);

}

void OpenCVImageViewer::configure_grid() { 
    // One time
    tile_height = std::max(int(std::sqrt(image_map.size())), 1);
    tile_width = image_map.size() / tile_height + (image_map.size() % tile_height != 0);

    im_w = WIDTH / tile_width;
    im_h = HEIGHT / tile_height;

    int j = 0;
    frame = Vec3b(0,0,0);
    for (std::map<std::string, image_info>::iterator it=image_map.begin(); it!=image_map.end(); it++) { 
        const std::string& name = it->first;
        image_info& im_info = it->second;

        int x = j % tile_width;
        int y = j / tile_width;

        im_info.roi = cv::Rect(x*im_w, y*im_h, im_w, im_h);
        im_info.placed = true;

        draw_image_to_frame(frame, im_info);

        j++;
    }
    
    return;
}

void OpenCVImageViewer::imshow(const std::string& name, const cv::Mat& img) { 
    // std::cerr << "opencv_utils::imshow" << std::endl;
    if (image_map.find(name) == image_map.end()) { 
        image_map[name] = image_info(name.c_str(), img.clone());
        // cv::createButton(name,button_callback,this,CV_CHECKBOX,1);
        // std::cerr << "Adding: " << name << std::endl;

        // Configure grid
        configure_grid();

    } else { 
        image_info& im_info = image_map[name];
        im_info.img = img.clone();
        
        draw_image_to_frame(frame, im_info);
    }

    if (image_map.find(name) != image_map.end() && !frame.empty())
        cv::imshow("OpenCV Viewer", frame);
}

Mat drawCorrespondences(const Mat& img1, const vector<Point2f>& features1, 
                        const Mat& img2, const vector<Point2f>& features2) {
    Mat part, img_corr(Size(img1.cols + img2.cols, MAX(img1.rows, img2.rows)), CV_8UC3);
    img_corr = Scalar::all(0);
    part = img_corr(Rect(0, 0, img1.cols, img1.rows));
    img1.copyTo(part); // cvtColor(img1, part, COLOR_GRAY2RGB);
    part = img_corr(Rect(img1.cols, 0, img2.cols, img2.rows));
    img1.copyTo(part); // cvtColor(img1, part, COLOR_GRAY2RGB);

    for (size_t i = 0; i < features1.size(); i++) {
        circle(img_corr, features1[i], 3, CV_RGB(255, 0, 0));
    }

    for (size_t i = 0; i < features2.size(); i++) {
        Point pt(cvRound(features2[i].x + img1.cols), cvRound(features2[i].y));
        circle(img_corr, pt, 3, Scalar(0, 0, 255));
        // line(img_corr, features1[desc_idx[i].trainIdx].pt, pt, Scalar(0, 255, 0));
    }

    return img_corr;
}
}
