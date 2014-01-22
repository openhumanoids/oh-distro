/**
 * SECTION:scanmatch
 * @title: ScanMatchingOpencvUtils
 * @short_description: Convenience functions for working with openCV
 * @include: bot/scanmatch/ScanMatchingOpencvUtils.hpp
 *
 * Utilities and convenience functions for working with openCV
 *
 *
 * Linking: -lscanmatch
 * namespace: scanmatch
 */

#ifndef SM_OPENCV_UTILS_H
#define SM_OPENCV_UTILS_H

//THESE FUNCTIONS COPIED FROM carmen3d_opencv_utils... KEEP THEM IN SYNC!

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pthread.h>
#include <deque>
#include <math.h>
#include <stdio.h>

namespace scanmatch
{

#ifndef PI
#define PI 3.14159265358979323846
#endif

/**
 * make opencv just die nicely and TELL YOU WHERE it dies grrrrrr
 */
void
sm_setupOpencvErrorHandler();

/**
 * sm_allocateOrResizeMat:
 * allocate or resize the input CvMat to the designated height,width, and type
 * the effect on previous data is undefined.
 */
inline static void
sm_allocateOrResizeMat(CvMat **mat, int height, int width, int type)
{
    if (*mat == NULL) {
        *mat = cvCreateMat(height, width, type);
    } else {
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

/**
 * sm_allocateOrResizeMat:
 * allocate or resize the input CvMat to the designated height,width, and type
 * the effect on previous data is undefined.
 */
inline static void
sm_allocateOrResizeMat(CvMat **mat, CvSize size, int type)
{
    sm_allocateOrResizeMat(mat, size.height, size.width, type);
}

/**
 * sm_allocateOrResizeCloneMat:
 * allocate or resize the input CvMat to be the same height,width, and type as the Cvmat to be cloned
 * the effect on previous data is undefined.
 */
inline static void
sm_allocateOrResizeCloneMat(CvMat **mat, CvMat * toClone)
{
    CvSize s = cvGetSize(toClone);
    sm_allocateOrResizeMat(mat, s.height, s.width, toClone->type);
}

/**
 * sm_drawArrow:
 * draw an arrow pattern
 */
void
sm_drawArrow(CvArr* drawFrame, CvPoint2D32f base, CvPoint2D32f tip,
        double scaleFactor, int line_thickness, CvScalar line_color);

/**
 * sm_drawCov:
 * draw am error ellipse for visualizing the covariance
 */
void
sm_drawCov(CvArr * drawFrame, CvPoint center, double sigmaX, double sigmaXY,
        double sigmaY, double sigmaT, double scaleFactorXY,
        double scaleFactorT, int line_thickness, CvScalar line_color1,
        CvScalar line_color2);

/**
 * sm_createGaussianKernels:
 * create the kernels used for drawing "blurred" lines.
 * These are used to render the laser likelihoods in the occupancy map.
 */
int
sm_createGaussianKernels(double sigma, unsigned char ** square_kernel,
        unsigned char ** line_kernel);

/**
 * sm_drawLine_squareKernel:
 * draw the blurred line by sliding a square kernel. Slower, but more accurate
 */
void
sm_drawLine_squareKernel(CvMat * im, CvPoint p1, CvPoint p2,
        unsigned int kernel_size, unsigned char * square_kern);

/**
 * sm_drawLine_lineKernel:
 * draw the blurred line by sliding a 1-pixel wide cross section of the kernel.
 * fast, and almost the same as drawing with the square_kernel. This is what is currently in use
 * for rendering the laser likelihood map.
 */
void
sm_drawLine_lineKernel(CvMat * im, CvPoint p1, CvPoint p2,
        unsigned int kernel_size, unsigned char * square_kernel,
        unsigned char * line_kernel, unsigned char * diag_line_kernel,
        bool addKernel = false);

/**
 * sm_CV_HLS:
 * workalike function for the CV_RGB function
 */
inline static CvScalar
sm_CV_HLS(double h, double l, double s)
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
    return CV_RGB((int) rgb[0] * 255, (int) rgb[1] * 255, (int) rgb[2] * 255);

}

//TODO: would probably be better if this was C
/**
 * sm_opencvDisplayWrapper:
 * class for making the openCV display routines operate in a seperate thread.
 * does NOT play nicely with the gmainloop!
 */
class sm_opencvDisplayWrapper
{
public:
    /**
     * display:
     * display the contents of frame in the window named "name"
     * this can simply replace calls to:
     *  cvNamedWindow(name, CV_WINDOW_AUTOSIZE);
     cvShowImage(name, frame);
     *
     * @name: name of the openCV display window
     * @frame: data to be displayed
     */
    static void
    display(const char * name, CvArr * frame);

private:
    static bool
    start();
    static void
    lock();
    static void
    unlock();

    static void
    waitForFrame();
    static void
    announceFrame();

    class sm_opencvDisplayFrame
    {
    public:
        sm_opencvDisplayFrame(const char * name, CvArr * frame_)
        {
            CvMat tmpMat;
            frame = cvCloneMat(cvGetMat(frame_, &tmpMat));
            displayName = strdup(name);
            //    fprintf(stderr, "%s added\n", displayName);
        }
        ~sm_opencvDisplayFrame()
        {
            cvReleaseMat(&frame);
            free(displayName);
            //    fprintf(stderr, "%s released\n", displayName);
        }
        void
        display()
        {
            //    fprintf(stderr,"%s display\n",displayName);
            cvNamedWindow(displayName, CV_WINDOW_AUTOSIZE);
            cvShowImage(displayName, frame);
        }

    private:
        char * displayName;
        CvMat * frame;
    };

    static std::deque<sm_opencvDisplayFrame *> m_displayDeque;
    static pthread_cond_t m_dequeCond;
    static pthread_mutex_t m_mutex, m_dequeLock;
    static pthread_t m_displayThread;
    static void *
    m_displayThread_function(void*);
    static bool m_threadRunning;

};

}

#endif
