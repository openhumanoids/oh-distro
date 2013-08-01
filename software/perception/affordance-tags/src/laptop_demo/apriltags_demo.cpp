/**
 * @file april_tags.cpp
 * @brief Example application for April tags library
 * @author: Michael Kaess
 *
 */

#include <cstring>

#include <opencv2/opencv.hpp>

#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>

using namespace std;


const char* window_name = "apriltags_demo";


// draw April tag detection on actual image
void draw_detection(cv::Mat& image, const AprilTags::TagDetection& detection) {
  // use corner points detected by line intersection
  std::pair<float, float> p1 = detection.p[0];
  std::pair<float, float> p2 = detection.p[1];
  std::pair<float, float> p3 = detection.p[2];
  std::pair<float, float> p4 = detection.p[3];

  // plot outline
  cv::line(image, cv::Point2f(p1.first, p1.second), cv::Point2f(p2.first, p2.second), cv::Scalar(255,0,0,0) );
  cv::line(image, cv::Point2f(p2.first, p2.second), cv::Point2f(p3.first, p3.second), cv::Scalar(0,255,0,0) );
  cv::line(image, cv::Point2f(p3.first, p3.second), cv::Point2f(p4.first, p4.second), cv::Scalar(0,0,255,0) );
  cv::line(image, cv::Point2f(p4.first, p4.second), cv::Point2f(p1.first, p1.second), cv::Scalar(255,0,255,0) );

  // mark center
  cv::circle(image, cv::Point2f(detection.cxy.first, detection.cxy.second), 8, cv::Scalar(0,0,255,0), 2);

  // print ID
  std::ostringstream strSt;
  strSt << "#" << detection.id;
  cv::putText(image, strSt.str(),
              cv::Point2f(detection.cxy.first + 10, detection.cxy.second + 10),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
}

int main(int argc, char* argv[]) {
  int device_id;
  if (argc==1) {
    device_id = 0;
  } else {
    device_id = atoi(argv[1]);
  }

  // find any available camera (laptop camera, web cam etc)
  cv::VideoCapture cap(device_id);
  if(!cap.isOpened()) {
    cerr << "ERROR: Can't find video device " << device_id << "\n";
    return -1;
  }

  // mfallon: modification to work with lenovo laptops out of the box:
//  if (cap.get(CV_CAP_PROP_FRAME_WIDTH) < 640) {
    // some cams provide a small image by default, try to get a larger one
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
 // }

  // determines which family of April tags is detected
  AprilTags::TagDetector tag_detector(AprilTags::tagCodes36h11);

  cv::namedWindow(window_name, 1);
  cv::Mat color;
  cv::Mat gray;
  while (true) {

    // capture frame
    cap >> color;

    cv::cvtColor(color, gray, CV_BGR2GRAY);

    std::vector<AprilTags::TagDetection> detections = tag_detector.extractTags(gray);

    // print out detections
    cout << detections.size() << " tags detected:" << endl;
    for (int i=0; i<detections.size(); i++) {
      cout << "  Id: " << detections[i].id << " -- "
           << "  Hamming distance: " << detections[i].hammingDistance << endl;

      // also highlight in the image
      draw_detection(color, detections[i]);

      // recovering the relative pose requires camera calibration;
      const double tag_size = 0.166; // real side length in meters of square black frame
      const double fx = 600; // camera focal length
      const double fy = 600;
      const double px = gray.cols/2; // camera principal point
      const double py = gray.rows/2;
      Eigen::Matrix4d T = detections[i].getRelativeTransform(tag_size, fx, fy, px, py);
      // note that for SLAM application it is better to use
      // reprojection error of corner points, as the noise in this
      // relative pose is very non-Gaussian; see iSAM source code for
      // suitable factors
    }

    imshow(window_name, color);

    // exit if any key pressed
    if (cv::waitKey(10) >= 0) break;
  }

  return 0;
}
