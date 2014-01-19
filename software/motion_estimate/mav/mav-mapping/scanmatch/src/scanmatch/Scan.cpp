/*
 * Scan.cpp
 *
 *  Created on: Aug 23, 2009
 *      Author: abachrac
 */
#include "Scan.hpp"
#include "ScanMatchingOpencvUtils.hpp"

using namespace std;
using namespace scanmatch;

Scan::Scan() :
    points(NULL), ppoints(NULL), numPoints(0), utime(-1), laser_type(
            SM_DUMMY_LASER), drawIm(NULL), displayIm(NULL), squareDisplayIm(
            NULL)
{
    memset(&T, 0, sizeof(T));
}

Scan::Scan(int numP, smPoint * points_, ScanTransform T_,
        sm_laser_type_t laser_type_, int64_t utime_, bool buildContours) :
    numPoints(numP), utime(utime_), laser_type(laser_type_), drawIm(NULL),
            displayIm(NULL), squareDisplayIm(NULL)
{
    points = (smPoint *) malloc(numPoints * sizeof(smPoint));
    memcpy(points, points_, numPoints * sizeof(smPoint));
    ppoints = (smPoint *) malloc(numPoints * sizeof(smPoint));
    memset(&T, 0, sizeof(T));
    applyTransform(T_);
    if (buildContours) {
        ContourExtractor * cextractor = new ContourExtractor(laser_type);
        sm_tictoc("findContours");
        cextractor->findContours(ppoints, numPoints, contours);
        sm_tictoc("findContours"); //      s->drawContours();
        delete cextractor;
    }

}

Scan::~Scan()
{
    for (unsigned i = 0; i < contours.size(); i++)
        delete (contours[i]);
    contours.clear();
    if (ppoints != NULL)
        free(ppoints);
    if (points != NULL)
        free(points);
    cvReleaseMat(&drawIm);
    cvReleaseMat(&displayIm);
    cvReleaseMat(&squareDisplayIm);
}

CvMat*
Scan::drawContours(int maxDrawDim, double pixPerMeter, smPoint * point1,
        smPoint* point2, double arrowScalFactor)
{
    if (contours.empty())
        return NULL;

    double minx = DBL_MAX, miny = DBL_MAX;
    double maxx = DBL_MIN, maxy = DBL_MIN;
    for (unsigned int i = 0; i < contours.size(); i++) {
        for (unsigned int p = 0; p < contours[i]->points.size(); p++) {
            minx = fmin(minx, contours[i]->points[p].x);
            miny = fmin(miny, contours[i]->points[p].y);
            maxx = fmax(maxx, contours[i]->points[p].x);
            maxy = fmax(maxy, contours[i]->points[p].y);
        }
    }
    double margin = fmax(.5, 0.1 * fmax(maxx - minx, maxy - miny));

    minx -= margin;
    maxx += margin;
    miny -= margin;
    maxy += margin;

    int height = (int) (pixPerMeter * (maxy - miny));
    int width = (int) (pixPerMeter * (maxx - minx));
    //    printf("minx=%.3f, maxx=%.3f, miny=%.3f, maxy=%.3f, width=%d, height=%d\n", minx, maxx, miny, maxy, width, height);

    sm_allocateOrResizeMat(&drawIm, height, width, CV_8UC3);
    cvSet(drawIm, cvScalar(255, 255, 255));

    //draw the contours

    for (unsigned int i = 0; i < contours.size(); i++) {
        CvScalar color = sm_CV_HLS(rand() % 360, .5, 1);
        //      CvScalar color = CV_RGB(0,0,255);

        //      printf("HLS = (%f,%f,%f) \t RGB = (%f,%f,%f)\n", hls[0], hls[1], hls[2], rgb[0] * 255, rgb[1] * 255, rgb[2] * 255);
        for (unsigned int p = 0; p < contours[i]->points.size() - 1; p++) {
            CvPoint
                    p0 =
                        { (int) ((contours[i]->points[p].x - minx)
                                * pixPerMeter),
                                (int) ((contours[i]->points[p].y - miny)
                                        * pixPerMeter) };
            CvPoint p1 =
                { (int) ((contours[i]->points[p + 1].x - minx) * pixPerMeter),
                        (int) ((contours[i]->points[p + 1].y - miny)
                                * pixPerMeter) };
            cvLine(drawIm, p0, p1, color, 5);
            cvCircle(drawIm, p0, 10, color, -1);
            cvCircle(drawIm, p1, 10, color, -1);
            //        cvCircle(drawIm, p0, 2, CV_RGB(255,0,0), -1);
            //        cvCircle(drawIm, p1, 2, CV_RGB(255,0,0), -1);
        }

    }
    //draw the original points
    for (unsigned int i = 0; i < numPoints; i += 3) {
        CvPoint p =
            { (int) ((ppoints[i].x - minx) * pixPerMeter), (int) ((ppoints[i].y
                    - miny) * pixPerMeter) };
        cvCircle(drawIm, p, 5, CV_RGB(255,0,0), -1);
    }

    if (point1 != NULL) {
        CvPoint p1 =
            { (int) ((point1->x - minx) * pixPerMeter), (int) ((point1->y
                    - miny) * pixPerMeter) };
        cvCircle(drawIm, p1, 5, CV_RGB(0,0,0), -1);
    }
    if (point1 != NULL && point2 != NULL) {
        CvPoint2D32f p1 =
            { ((point1->x - minx) * pixPerMeter), ((point1->y - miny)
                    * pixPerMeter) };
        CvPoint2D32f p2 =
            { ((point2->x - minx) * pixPerMeter), ((point2->y - miny)
                    * pixPerMeter) };
        sm_drawArrow(drawIm, p1, p2, arrowScalFactor, 2, CV_RGB(0,255,255));
    }

    double AR = (double) height / (double) width;
    int displayHeight, displayWidth;
    if (AR <= 1) {
        displayWidth = maxDrawDim;
        displayHeight = (int) (((double) maxDrawDim) * AR);
    } else {
        displayWidth = (int) (((double) maxDrawDim) * 1 / AR);
        displayHeight = maxDrawDim;
    }
    sm_allocateOrResizeMat(&displayIm, displayHeight, displayWidth, CV_8UC3);
    cvResize(drawIm, displayIm);

    cvFlip(displayIm, displayIm);

    sm_allocateOrResizeMat(&squareDisplayIm, maxDrawDim, maxDrawDim, CV_8UC3);
    cvSet(squareDisplayIm, CV_RGB(255, 255, 255));
    CvSize displaySize = cvGetSize(displayIm);
    //put the map etc in the fullGui
    CvMat subGui;
    CvRect subGuiRect;
    subGuiRect.height = displaySize.height;
    subGuiRect.width = displaySize.width;
    subGuiRect.x = (maxDrawDim - displaySize.width) / 2;
    subGuiRect.y = (maxDrawDim - displaySize.height) / 2;
    cvGetSubRect(squareDisplayIm, &subGui, subGuiRect);
    cvCopy(displayIm, &subGui);

    sm_opencvDisplayWrapper::display((char *) "contours", squareDisplayIm);
    //    cvNamedWindow("contours", CV_WINDOW_AUTOSIZE);
    //    cvShowImage("contours", squareDisplayIm);
    return displayIm;
}
