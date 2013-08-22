/**
 * @file april_tags.cpp
 * @brief Example application for April tags library
 * @author: Michael Kaess
 *
 * Opens the first available camera (typically a built in camera in a
 * laptop) and continuously detects April tags in the incoming
 * images. Detections are both visualized in the live image and shown
 * in the text console. Optionally allows selecting of a specific
 * camera in case multiple ones are present and specifying image
 * resolution as long as supported by the camera. Also includes the
 * option to send tag detections via a serial port, for example when
 * running on a Raspberry Pi that is connected to an Arduino.
 */

using namespace std;


#include <iostream>
#include <cstring>
#include <vector>
#include <sys/time.h>

const string usage = "\n"
  "Usage:\n"
  "  apriltags_demo [OPTION...] [deviceID]\n"
  "\n"
  "Options:\n"
  "  -h  -?          Show help options\n"
  "  -d              disable graphics\n"
  "  -C <bbxhh>      Tag family (default 36h11)\n"
  "  -F <fx>         Focal length in pixels\n"
  "  -W <width>      Image width (default 640, availability depends on camera)\n"
  "  -H <height>     Image height (default 480, availability depends on camera)\n"
  "  -S <size>       Tag size (square black frame) in meters\n"
  "  -E <exposure>   Manually set camera exposure (default auto; range 0-10000)\n"
  "  -G <gain>       Manually set camera gain (default auto; range 0-255)\n"
  "  -B <brightness> Manually set the camera brightness (default 128; range 0-255)\n"
  "\n";

const string intro = "\n"
    "April tags test code\n"
    "(C) 2012-2013 Massachusetts Institute of Technology\n"
    "Michael Kaess\n"
    "\n";



#define EXPOSURE_CONTROL // only works in Linux

#ifdef EXPOSURE_CONTROL
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#endif

// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"



// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;



using namespace cv;


cv::VideoWriter outputVideo;                    // Open the output



const char* window_name = "apriltags_demo";


// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}


class Demo {


  bool m_draw; // draw image and April tag detections?

  int m_width; // image size in pixels
  int m_height;
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  int m_deviceId; // camera id (in case of multiple cameras)
  cv::VideoCapture m_cap;

  int m_exposure;
  int m_gain;
  int m_brightness;


public:

  // default constructor
  Demo() :
    // default settings, most can be modified through command line options (see below)

    m_draw(true),

    m_width(640),
    m_height(480),
    m_tagSize(0.166),
    m_fx(600),
    m_fy(600),
    m_px(m_width/2),
    m_py(m_height/2),

    m_exposure(-1),
    m_gain(-1),
    m_brightness(-1),

    m_deviceId(0)
  {}

  void parseOptions(int argc, char* argv[]);
  
  void setup();
  
  void loop();



}; // Demo



// parse command line options to change default behavior
void Demo::parseOptions(int argc, char* argv[]) {
  int c;
  while ((c = getopt(argc, argv, ":h?adC:F:H:S:W:E:G:B:")) != -1) {
    // Each option character has to be in the string in getopt();
    // the first colon changes the error character from '?' to ':';
    // a colon after an option means that there is an extra
    // parameter to this option; 'W' is a reserved character
    switch (c) {
    case 'h':
    case '?':
      cout << intro;
      cout << usage;
      exit(0);
      break;
    case 'd':
      m_draw = false;
      break;
    case 'F':
      m_fx = atof(optarg);
      m_fy = m_fx;
      break;
    case 'H':
      m_height = atoi(optarg);
      m_py = m_height/2;
      break;
    case 'S':
      m_tagSize = atof(optarg);
      break;
    case 'W':
      m_width = atoi(optarg);
      m_px = m_width/2;
      break;
    case 'E':
      m_exposure = atoi(optarg);
      break;
    case 'G':
      m_gain = atoi(optarg);
      break;
    case 'B':
      m_brightness = atoi(optarg);
      break;
    case ':': // unknown option, from getopt
      cout << intro;
      cout << usage;
      exit(1);
      break;
    }
  }

  if (argc == optind + 1) {
    m_deviceId = atoi(argv[optind]);
  }
}

void Demo::setup() {


  // find and open a USB camera (built in laptop camera, web cam etc)
  m_cap = cv::VideoCapture(m_deviceId);
      if(!m_cap.isOpened()) {
    cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
    exit(1);
  }
  m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
  m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
  cout << "Camera successfully opened (ignore error messages above...)" << endl;
  cout << "Actual resolution: "
      << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
      << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

  // prepare window for drawing the camera images
  if (m_draw) {
  //  cv::namedWindow(window_name, 1);
  }
  
  
  /*
     VideoCapture capture(1); // open the default camera
            if( !capture.isOpened() )  {
                   printf("Camera failed to open!\n");
                   exit(-1);
           }*/
           
           Mat frame; 
           m_cap >> frame; // get first frame for size
           
           // record video
            outputVideo = VideoWriter("RobotVideo.avi", -1, 30, frame.size(), true);  
  
//  std::string output_name = "output.avi";
  
//  cv::Size S = cv::Size(m_width,  m_height);  
  
//  outputVideo.open(output_name, CV_FOURCC('D','I','V','X') , 10, S, true); // also ex=-1  // 10fps
  //record.open(output_name, CV_FOURCC('P','I','M','1') , 30, S, true); // also ex=-1  // 10fps

  if (!outputVideo.isOpened())
  {
    cout  << "Could not open the output video for write" << endl;
  //  exit(-1);
  }  

}


// The processing loop where images are retrieved, tags detected,
// and information about detections generated
void Demo::loop() {

  cv::Mat image;
  cv::Mat image_gray;

  int frame = 0;
  double last_t = tic();
  while (true) {

    // capture frame
    m_cap >> image;

    // alternative way is to grab, then retrieve; allows for
    // multiple grab when processing below frame rate - v4l keeps a
    // number of frames buffered, which can lead to significant lag
    //      m_cap.grab();
    //      m_cap.retrieve(image);

    // detect April tags (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    

    // show the current image including any detections
    if (m_draw) {
      imshow(window_name, image); // OpenCV call
    }

    // print out the frame rate at which image frames are being processed
    frame++;
    if (frame % 10 == 0) {
      double t = tic();
      cout << "  " << 10./(t-last_t) << " fps" << endl;
      last_t = t;
    }

    
    
    outputVideo << image;
    
    
    
    // exit if any key is pressed
    if (cv::waitKey(1) >= 0) break;
  }
}



// here is were everything begins
int main(int argc, char* argv[]) {
  Demo demo;

  // process command line options
  demo.parseOptions(argc, argv);

  // setup image source, window for drawing...
  demo.setup();

  // the actual processing loop where tags are detected and visualized
  demo.loop();

  return 0;
}
