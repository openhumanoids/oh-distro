#ifndef OPENCV_UTILS_H
#define OPENCV_UTILS_H

#include <opencv2/opencv.hpp>

// #include <opencv/cv.h>
// #include <opencv/highgui.h>
#include <pthread.h>
#include <deque>
#include <math.h>
#include <stdio.h>
#include <bot_core/bot_core.h>

namespace opencv_utils {
#ifndef PI
#define PI 3.14159265358979323846
#endif

/*
 * make opencv just die nicely and TELL YOU WHERE it dies grrrrrr
 */
void setupOpencvErrorHandler();

/*
 * Creates an opencv image header for a bot_core_image_t structure
 * FYI: THIS FUNCTION DOES NOT COPY THE DATA!
 */
static inline const cv::Mat get_opencv_header_for_bot_core_image_t(const bot_core_image_t * c3d_im)
{
  if (c3d_im->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGR || c3d_im->pixelformat
      == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB) {
    return cv::Mat(c3d_im->height, c3d_im->width, CV_8UC3, c3d_im->data);
  }
  else if (c3d_im->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY) {
    return cv::Mat(c3d_im->height, c3d_im->width, CV_8UC1, c3d_im->data);
  }
  else if (c3d_im->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_FLOAT_GRAY32) {
    return cv::Mat(c3d_im->height, c3d_im->width, CV_32FC1, c3d_im->data);
  }
  else {
    fprintf(stderr, "ERROR: invalid bot_core_image_t pixel format!\n");
    assert(false);
    return cv::Mat(0, 0, 0, NULL); //make compiler happy
  }
}

/*
 * Creates a bot_core_image_t structure around an opencv image
 * FYI: THIS FUNCTION DOES NOT COPY THE DATA!
 */
static inline const bot_core_image_t get_bot_core_image_t_from_opencv(const cv::Mat & cv_in_mat)
{
  bot_core_image_t c3d_im;
  memset(&c3d_im, 0, sizeof(c3d_im));
  c3d_im.height = cv_in_mat.rows;
  c3d_im.width = cv_in_mat.cols;
  c3d_im.row_stride = cv_in_mat.step;
  c3d_im.size = c3d_im.row_stride * c3d_im.height;
  c3d_im.data = cv_in_mat.data;

  switch (cv_in_mat.type()) {
  case CV_8UC3:
    {
      c3d_im.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGR; //opencv images are usually bgr
      break;
    }
  case CV_8UC1:
    {
      c3d_im.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
      break;
    }
  case CV_32FC1:
    {
      c3d_im.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_FLOAT_GRAY32;
      break;
    }
  default:
    {
      fprintf(stderr, "ERROR: invalid opencv image type!\n");
      assert(false);
      break;
    }
  }
  return c3d_im;
}

//void allocateOnDemand(IplImage **img, CvSize size, int depth, int channels);
inline static void allocateOnDemand(IplImage **img, CvSize size, int depth, int channels)
{
  if (*img != NULL)
    return;

  *img = cvCreateImage(size, depth, channels);

  if (*img == NULL) {
    fprintf(stderr, "Error: Couldn't allocate image. Out of memory?\n");
    exit(-1);
  }
}

inline static void allocateOrResizeMat(CvMat **mat, int height, int width, int type)
{
  if (*mat == NULL) {
    *mat = cvCreateMat(height, width, type);
  }
  else {
    CvSize s = cvGetSize(*mat);
    if (s.height != height || s.width != width) {
      //need to resize matrix
      cvReleaseMat(mat);
      *mat = cvCreateMat(height, width, type);
    }
  }

  if (*mat == NULL) {
    fprintf(stderr, "Error: Couldn't allocate matrix. Out of memory?\n");
    exit(-1);
  }
}

inline static void allocateOrResizeMat(CvMat **mat, CvSize size, int type)
{
  allocateOrResizeMat(mat, size.height, size.width, type);
}

inline static void allocateOrResizeCloneMat(CvMat **mat, CvMat * toClone)
{
  CvSize s = cvGetSize(toClone);
  allocateOrResizeMat(mat, s.height, s.width, toClone->type);
}

inline static void allocateOrResizeImage(IplImage **im, int height, int width, int type, int nchan)
{
  if (*im == NULL) {
    *im = cvCreateImage(cvSize(width, height), type, nchan);
  }
  else {
    CvSize s = cvGetSize(*im);
    if (s.height != height || s.width != width) {
      //need to resize matrix
      cvReleaseImage(im);
      *im = cvCreateImage(cvSize(width, height), type, nchan);
    }
  }

  if (*im == NULL) {
    fprintf(stderr, "Error: Couldn't allocate matrix. Out of memory?\n");
    exit(-1);
  }
}

inline static void allocateOrResizeImage(IplImage **im, CvSize size, int type, int nchan)
{
  allocateOrResizeImage(im, size.height, size.width, type, nchan);
}

inline static void allocateOrResizeCloneImage(IplImage **im, const IplImage * toClone)
{
  allocateOrResizeImage(im, toClone->height, toClone->width, toClone->depth, toClone->nChannels);
}

inline static CvScalar CV_HLS(double h, double l, double s)
{
  //h hue is 0-360, 0 is red, 120 is green, 240 is blue
  //l lightness is 0-1
  //s saturation is 0-1
  float rgb[3];
  CvMat cv_rgb = cvMat(1, 1, CV_32FC3, rgb);
  float hls[3];
  CvMat cv_hls = cvMat(1, 1, CV_32FC3, hls);

  hls[0] = h;
  hls[1] = l;
  hls[2] = s;
  cvCvtColor(&cv_hls, &cv_rgb, CV_HLS2RGB);
  return CV_RGB((int)rgb[0]*255, (int)rgb[1]*255,(int)rgb[2]*255);

}

inline static CvScalar getJetColor(int ind)
{

  if (ind < 0)
    ind = 0;
  if (ind > 255)
    ind = 255;
  float * color = bot_color_util_jet((double) ind / 255.0);
  int R = color[0] * 255.0;
  int G = color[1] * 255.0;
  int B = color[2] * 255.0;
  ;
  CvScalar c = CV_RGB(R,G,B);
  return c;
}

void drawArrow(CvArr* drawFrame, CvPoint2D32f base, CvPoint2D32f tip, double scaleFactor, int line_thickness,
    CvScalar line_color);
void drawCov(CvArr * drawFrame, CvPoint center, double sigmaX, double sigmaXY, double sigmaY, double sigmaT,
    double scaleFactorXY, double scaleFactorT, int line_thickness, CvScalar line_color1, CvScalar line_color2);

int createGaussianKernels(double sigma, unsigned char ** square_kernel, unsigned char ** line_kernel);
void drawLine_squareKernel(CvMat * im, CvPoint p1, CvPoint p2, unsigned int kernel_size, unsigned char * square_kern);
void drawLine_lineKernel(CvMat * im, CvPoint p1, CvPoint p2, unsigned int kernel_size, unsigned char * square_kernel,
    unsigned char * line_kernel, unsigned char * diag_line_kernel);
void writeCvMatToMatlab(FILE * f, const char * variableName, const CvMat* mat);
void writeIntToMatlab(FILE * f, const char * variableName, const int data);
void writeDoubleToMatlab(FILE * f, const char * variableName, const double data);

//TODO: would probably be better if this was C
class DisplayFrame {
public:
  DisplayFrame(const char * name, const CvArr * frame_, int displayWidth)
  {
    CvMat tmpMat;
    CvMat * frame_p = cvGetMat(frame_, &tmpMat);
    int maxdim = std::max(frame_p->width, frame_p->height);

    if (displayWidth == maxdim || displayWidth <= 0) {
      frame = cvCloneMat(frame_p);
    }
    else {
      float scaleFactor = (float) displayWidth / (float) maxdim;
      frame = cvCreateMat(frame_p->height * scaleFactor, frame_p->width * scaleFactor, frame_p->type);
      cvResize(frame_, frame);
    }
    displayName = strdup(name);
    //    fprintf(stderr, "%s added\n", displayName);
  }
  ~DisplayFrame()
  {
    cvReleaseMat(&frame);
    free(displayName);
    //    fprintf(stderr, "%s released\n", displayName);
  }
  void display()
  {
    //    fprintf(stderr,"%s display\n",displayName);
    cvNamedWindow(displayName, CV_WINDOW_AUTOSIZE);
    cvShowImage(displayName, frame);
  }

private:
  char * displayName;
  CvMat * frame;
};

class DisplayWrapper {
public:
  static bool start();
  static void lock();
  static void unlock();

  static void waitForFrame();
  static void announceFrame();

  static void display(const char * name, const CvArr * frame, int displayWidth = 800);
  static void display(const char * name, const cv::Mat & frame, int displayWidth = 800)
  {
    CvMat cvmat = frame;
    display(name, &cvmat, displayWidth);
  }
  //  static inline void display(char * name, IplImage * frame){
  //  }

private:
  static std::deque<DisplayFrame *> m_displayDeque;
  static pthread_cond_t m_dequeCond;
  static pthread_mutex_t m_mutex, m_dequeLock;
  static pthread_t m_displayThread;
  static void * m_displayThread_function(void*);
  static bool m_threadRunning;

};

}

#endif
