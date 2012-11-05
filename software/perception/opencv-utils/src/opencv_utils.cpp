#include <stdio.h>
#include "opencv_utils.hpp"
#include <sys/time.h>

using namespace opencv_utils;

static int opencv_abort_error_handler(int status, char const* func_name, char const* err_msg, char const* file_name, int line,
    void*)
{
  fprintf(stderr, "ERROR: in %s (%s:%d)  Message: %s \n", func_name, file_name, line, err_msg);
  assert(false);
}
void setupOpencvErrorHandler()
{
  cvRedirectError(opencv_abort_error_handler);
}

void drawArrow(CvArr * drawFrame, CvPoint2D32f base, CvPoint2D32f tip, double scaleFactor, int line_thickness,
    CvScalar line_color)
{

  //		 Let's make the flow field look nice with arrows.

  //		 The arrows will be a bit too short for a nice visualization because of the high framerate
  //		 * (ie: there's not much motion between the frames).  So let's lengthen them by a factor of 3.
  //
  CvPoint p, q;
  p.x = (int) base.x;
  p.y = (int) base.y;
  q.x = (int) tip.x;
  q.y = (int) tip.y;

  double angle;
  angle = atan2((double) p.y - q.y, (double) p.x - q.x);
  double hypotenuse;
  hypotenuse = sqrt((p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));

  //		 Here we lengthen the arrow by a factor of three.
  q.x = (int) (p.x - scaleFactor * hypotenuse * cos(angle));
  q.y = (int) (p.y - scaleFactor * hypotenuse * sin(angle));

  //		 Now we draw the main line of the arrow.
  //		 "currFrame" is the frame to draw on.
  //		 * "p" is the point where the line begins.
  //		 * "q" is the point where the line stops.
  //		 * "CV_AA" means antialiased drawing.
  //		 * "0" means no fractional bits in the center cooridinate or radius.

  cvLine(drawFrame, p, q, line_color, line_thickness, CV_AA, 0);
  //		 Now draw the tips of the arrow.  I do some scaling so that the
  //		 * tips look proportional to the main line of the arrow.
  //
  p.x = (int) (q.x + 14 * cos(angle + PI / 4));
  p.y = (int) (q.y + 14 * sin(angle + PI / 4));
  cvLine(drawFrame, p, q, line_color, line_thickness, CV_AA, 0);
  p.x = (int) (q.x + 14 * cos(angle - PI / 4));
  p.y = (int) (q.y + 14 * sin(angle - PI / 4));
  cvLine(drawFrame, p, q, line_color, line_thickness, CV_AA, 0);

}

void drawCov(CvArr * drawFrame, CvPoint center, double sigmaX, double sigmaXY, double sigmaY, double sigmaT,
    double scaleFactorXY, double scaleFactorT, int line_thickness, CvScalar line_color1, CvScalar line_color2)
{
  double sigma[4] = { sigmaX, sigmaXY, sigmaXY, sigmaY };
  double evecs[4];
  double evals[4];
  double prod[4];

  CvMat cv_sigma = cvMat(2, 2, CV_64FC1, sigma);
  CvMat cv_evecs = cvMat(2, 2, CV_64FC1, evecs);
  CvMat cv_evals = cvMat(2, 2, CV_64FC1, evals);
  CvMat cv_prod = cvMat(2, 2, CV_64FC1, prod);
  cvSVD(&cv_sigma, &cv_evals, &cv_evecs);
  cvPow(&cv_evals, &cv_evals, 1. / 2.);
  cvTranspose(&cv_evecs, &cv_evecs);
  cvMatMul(&cv_evals, &cv_evecs, &cv_prod);
  int nSegments = 360;
  double th = -PI;
  double xy[2] = { cos(th), sin(th) };
  CvMat cv_xy = cvMat(1, 2, CV_64FC1, xy);
  cvMatMul(&cv_xy, &cv_prod, &cv_xy);
  CvPoint p0 = { center.x + scaleFactorXY * xy[0], center.y + scaleFactorXY * xy[1] };
  for (int i = 0; i < nSegments - 1; i++) {
    th += 2* PI / nSegments;
    xy[0] = cos(th);
    xy[1] = sin(th);
    cvMatMul(&cv_xy, &cv_prod, &cv_xy);
    CvPoint p1 = { center.x + scaleFactorXY * xy[0], center.y + scaleFactorXY * xy[1] };

    CvScalar color;
    if (fabs(th) < scaleFactorT * sigmaT)
      color = line_color2;
    else
      color = line_color1;
    cvLine(drawFrame, p0, p1, color, line_thickness);
    p0 = p1;
  }

  //  cvEllipse(drawFrame, center, cvSize(evals[0] * scaleFactorXY, evals[3] * scaleFactorXY), atan2(prod[2], prod[0]),
  //      -180, 180, line_color1, line_thickness);
  //
  //  cvEllipse(drawFrame, center, cvSize(evals[0] * scaleFactorXY, evals[3] * scaleFactorXY), atan2(prod[2], prod[0]),
  //      -sigmaT * scaleFactorT, sigmaT * scaleFactorT, line_color2, line_thickness);

}

int createGaussianKernels(double sigma, unsigned char ** square_kernel, unsigned char ** line_kernel)
{
  //find an appropriate kernel_size for this sigma
  unsigned int kernel_size = 0;
  int val = 255;
  while (val > 32) {
    kernel_size++;
    val = (int) cvRound(255.0 * exp(-1.0 / (sigma * sigma) * (kernel_size * kernel_size)));
  }
  kernel_size = 2* kernel_size + 1; //we only accounted for half +1 to make size odd


  *square_kernel = (unsigned char *) calloc(kernel_size * kernel_size, sizeof(unsigned char));
  *line_kernel = (unsigned char *) calloc(kernel_size, sizeof(unsigned char));

  unsigned char * square_kernel_p = *square_kernel;
  unsigned char * line_kernel_p = *line_kernel;
  for (unsigned int i = 0; i < kernel_size; i++) {
    for (unsigned int j = 0; j < kernel_size; j++) {
      int offset = kernel_size / 2;
      double x = (double) i - (double) offset;
      double y = (double) j - (double) offset;
      int val = (int) cvRound(255.0 * exp(-1.0 / (sigma * sigma) * (x * x + y * y)));
      square_kernel_p[j * kernel_size + i] = (unsigned char) val;

      //      printf("%u ", (unsigned int) square_kernel_p[j * kernel_size + i]);
    }
    //    printf("\n");
  }
  //  printf("\n");

  for (unsigned int i = 0; i < kernel_size; i++) {
    line_kernel_p[i] = square_kernel_p[kernel_size / 2 * kernel_size + i];
    //    printf("%u ", (unsigned int) line_kernel_p[i]);
  }
  //  printf("\n");

  return kernel_size;
}

void drawLine_squareKernel(CvMat * im, CvPoint p1, CvPoint p2, unsigned int kernel_size, unsigned char * square_kern)
{
  static CvMat cv_kern = cvMat(kernel_size, kernel_size, CV_8UC1, square_kern);
  CvSize imSize = cvGetSize(im);
  uchar* imDataP;
  int imStep;
  cvGetRawData(im, &imDataP, &imStep);

  CvLineIterator iterator;
  CvRect window_rect = { 0, 0, kernel_size, kernel_size };
  CvMat window;
  int count = cvInitLineIterator(im, p1, p2, &iterator, 8, 0);
  for (int i = 0; i < count; i++) {
    {
      unsigned int offset, x, y;
      /* assume that ROI is not set, otherwise need to take it into account. */
      offset = iterator.ptr - imDataP;
      y = offset / imStep;
      x = (offset - y * imStep) / (sizeof(uchar) /* size of pixel */);

      window_rect.x = x - kernel_size / 2;
      window_rect.y = y - kernel_size / 2;
      if (window_rect.x < 0 || window_rect.y < 0 || window_rect.x + window_rect.width > imSize.width || window_rect.y
          + window_rect.height > imSize.height) {
        continue; //out of bounds, skip this pixel
      }
      cvGetSubRect(im, &window, window_rect);
      cvMax(&cv_kern, &window, &window);

      CV_NEXT_LINE_POINT(iterator);

    }
  }

}

void drawLine_lineKernel(CvMat * im, CvPoint p1, CvPoint p2, unsigned int kernel_size, unsigned char * square_kern,
    unsigned char * line_kernel, unsigned char * diag_line_kernel)
{
  static CvMat cv_square_kern = cvMat(kernel_size, kernel_size, CV_8UC1, square_kern);
  CvRect window_rect = { 0, 0, kernel_size, kernel_size };
  CvMat window;
  CvMat cv_diag_line_kernel;
  CvSize imSize = cvGetSize(im);
  //draw the two endpoints with the square kernel
  //first endpoint
  window_rect.x = p1.x - (kernel_size / 2);
  window_rect.y = p1.y - (kernel_size / 2);
  if (window_rect.x < 0 || window_rect.y < 0 || window_rect.x + window_rect.width > imSize.width || window_rect.y
      + window_rect.height > imSize.height) {
    printf("endpoint is out of bounds!\n"); //out of bounds, skip this endpoint
  }
  else {
    cvGetSubRect(im, &window, window_rect);
    cvMax(&cv_square_kern, &window, &window);
  }

  //second endpoint
  window_rect.x = p2.x - (kernel_size / 2);
  window_rect.y = p2.y - (kernel_size / 2);
  if (window_rect.x < 0 || window_rect.y < 0 || window_rect.x + window_rect.width > imSize.width || window_rect.y
      + window_rect.height > imSize.height) {
    printf("endpoint is out of bounds!\n"); //out of bounds, skip this endpoint
  }
  else {
    cvGetSubRect(im, &window, window_rect);
    cvMax(&cv_square_kern, &window, &window);
  }
  bool hor;
  int pointOffsetX, pointOffsetY;
  int stretchedKernSize;
  double strechFactor;
  CvPoint d = { p2.x - p1.x, p2.y - p1.y };
  if (d.x == 0 && d.y == 0) {
    return;
  }

  double lineAngle = atan2(d.y, d.x);
  if (abs(d.x) > abs(d.y)) {
    hor = false;
    strechFactor = 1.0 / fabs(cos(lineAngle));
    stretchedKernSize = ((int) (kernel_size * strechFactor) / 2) * 2 + 1; //make it odd
    //we're gonna use a vertical kernel
    window_rect.width = 1;
    window_rect.height = stretchedKernSize;
    pointOffsetX = 0;
    pointOffsetY = stretchedKernSize / 2;
    cv_diag_line_kernel = cvMat(stretchedKernSize, 1, CV_8UC1, diag_line_kernel);
  }
  else {
    hor = true;
    //we're gonna use a horizontal kernel
    strechFactor = 1.0 / fabs(cos(PI / 2 - lineAngle));
    stretchedKernSize = ((int) (kernel_size * strechFactor) / 2) * 2 + 1; //make it odd
    window_rect.width = stretchedKernSize;
    window_rect.height = 1;
    pointOffsetX = stretchedKernSize / 2;
    pointOffsetY = 0;
    cv_diag_line_kernel = cvMat(1, stretchedKernSize, CV_8UC1, diag_line_kernel);
  }
  //  fprintf(stderr,"d.x=%d d.y=%d hor = %d, stretchedKernSize = %d, strechFactor=%f lineAngle=%f\n",d.x,d.y,hor,stretchedKernSize,strechFactor,lineAngle);

  //put the values into the stretched kernel
  int kern_ind;
  for (int i = 0; i < stretchedKernSize / 2; i++) {
    kern_ind = (int) (i * (double) kernel_size / ((double) stretchedKernSize));
    diag_line_kernel[i] = line_kernel[kern_ind];
    diag_line_kernel[(stretchedKernSize - 1) - i] = line_kernel[kern_ind];
  }
  diag_line_kernel[stretchedKernSize / 2] = line_kernel[kernel_size / 2];//middle value since width is odd

  uchar* imDataP;
  int imStep;
  cvGetRawData(im, &imDataP, &imStep);

  CvLineIterator iterator;
  int count = cvInitLineIterator(im, p1, p2, &iterator, 8, 0);
  for (int i = 0; i < count - 1; i++) {
    {
      CV_NEXT_LINE_POINT(iterator);
      unsigned int offset, x, y;
      /* assume that ROI is not set, otherwise need to take it into account. */
      offset = iterator.ptr - imDataP;
      y = offset / imStep;
      x = (offset - y * imStep) / (sizeof(uchar) /* size of pixel */);
      window_rect.x = x - pointOffsetX;
      window_rect.y = y - pointOffsetY;
      if (window_rect.x < 0 || window_rect.y < 0 || window_rect.x + window_rect.width > imSize.width || window_rect.y
          + window_rect.height > imSize.height) {
        continue; //out of bounds, skip this pixel
      }
      cvGetSubRect(im, &window, window_rect);
      CvScalar pixelVal = cvGet2D(im, y, x);
      if (pixelVal.val[0] > 225) {
        //    printf("point %d was not already hit... val=%.0f \t",i,pixelVal.val[0]);
        continue;
      }

      cvMax(&cv_diag_line_kernel, &window, &window);

    }
  }

}

void writeCvMatToMatlab(FILE * f, const char * variableName, const CvMat* mat)
{
  fprintf(f, "\n%s = [\n", variableName);
  CvSize size = cvGetSize(mat);

  for (int i = 0; i < size.height; i++) {
    for (int j = 0; j < size.width; j++) {
      if ((cvGetElemType(mat) == CV_32FC3) | (cvGetElemType(mat) == CV_64FC3)) {
        CvScalar tmp = cvGet2D(mat, i, j);
        fprintf(f, "%.24f, %.24f , %.24f ", tmp.val[0], tmp.val[1], tmp.val[2]);
      }
      else if ((cvGetElemType(mat) == CV_32FC2) | (cvGetElemType(mat) == CV_64FC2)) {
        CvScalar tmp = cvGet2D(mat, i, j);
        fprintf(f, "%.24f, %.24f ", tmp.val[0], tmp.val[1]);
      }
      else if ((cvGetElemType(mat) == CV_32FC1) | (cvGetElemType(mat) == CV_64FC1)) {
        CvScalar tmp = cvGet2D(mat, i, j);
        fprintf(f, "%.24f ", tmp.val[0]);
      }
      else {
        CvScalar tmp = cvGet2D(mat, i, j);
        fprintf(f, "%.24f ", tmp.val[0]);
      }

    }
    fprintf(f, ";\n");
  }

  fprintf(f, "];\n\n");
  fflush(f);

}

void writeIntToMatlab(FILE * f, const char * variableName, const int data)
{
  fprintf(f, "\n%s = %d;\n", variableName, data);
}

void writeDoubleToMatlab(FILE * f, const char * variableName, const double data)
{
  fprintf(f, "\n%s = %f;\n", variableName, data);
}

//static vars for the DisplayWrapper

std::deque<DisplayFrame *> DisplayWrapper::m_displayDeque;
pthread_mutex_t DisplayWrapper::m_mutex;
pthread_cond_t DisplayWrapper::m_dequeCond;
pthread_mutex_t DisplayWrapper::m_dequeLock;
pthread_t DisplayWrapper::m_displayThread;
bool DisplayWrapper::m_threadRunning = false;

bool DisplayWrapper::start()
{
  if (m_threadRunning)
    return false;
  pthread_mutex_init(&m_mutex, 0);
  pthread_mutex_init(&m_dequeLock, 0);
  pthread_cond_init(&m_dequeCond, NULL);
  m_threadRunning = true;
  pthread_create(&m_displayThread, 0, m_displayThread_function, 0);
  return true;
}

void DisplayWrapper::lock()
{
  //cerr <<"LOCK" << endl;
  pthread_mutex_lock(&m_dequeLock);
}

void DisplayWrapper::unlock()
{
  //cerr <<"UNLOCK" << endl;
  pthread_mutex_unlock(&m_dequeLock);
}

void DisplayWrapper::waitForFrame()
{
  //only wait for a short period of time...

  struct timespec next_timeout;
  bot_timespec_now(&next_timeout);
  bot_timespec_addms(&next_timeout, 250);
  //  printf("sleep\n");
  pthread_cond_timedwait(&m_dequeCond, &m_dequeLock, &next_timeout);
  //  printf("wake\n");
  //  pthread_cond_wait(&m_dequeCond, &m_dequeLock);
}
void DisplayWrapper::announceFrame()
{
  pthread_cond_broadcast(&m_dequeCond);
}

void DisplayWrapper::display(const char * name, const CvArr * frame,int displayWidth)
{
  if (!m_threadRunning)
    start();
  lock();
  m_displayDeque.push_front(new DisplayFrame(name, frame,displayWidth));
  unlock();
  announceFrame();

}
void * DisplayWrapper::m_displayThread_function(void*)
{
  while (true) {
    lock();
    if (m_displayDeque.empty())
      waitForFrame();
    while (!m_displayDeque.empty()) {
      m_displayDeque.back()->display();
      delete m_displayDeque.back();
      m_displayDeque.pop_back();
    }
    unlock();
    cvWaitKey(100);
  }
  return 0;
}
