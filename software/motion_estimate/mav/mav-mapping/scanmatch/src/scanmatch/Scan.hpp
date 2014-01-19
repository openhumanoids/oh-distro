/**
 * SECTION:scanmatch
 * @title: Scan
 * @short_description: Storage class for keeping track of relevant info associated with a scan.
 * @include: bot/scanmatch/Scan.hpp
 *
 * These parameters work pretty well with a hokuyo UTM, but others might be better.
 *
 * Linking: -lscanmatch
 * namespace: scanmatch
 */

#ifndef SCAN_H_
#define SCAN_H_

#include "Contour.hpp"
#include <vector>
#include <list>
#include "ScanMatchingUtils.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pthread.h>
#include <float.h>

namespace scanmatch
{

class Scan
{
public:

    Scan(int numP, smPoint * points_, ScanTransform T_,
            sm_laser_type_t laser_type_, int64_t utime_, bool buildContours =
                    false);
    Scan();
    ~Scan();
    /**
     * number of points that make up this scan.
     */
    unsigned numPoints;
    /**
     * ScanTransform (x,y,theta) of scan origin, presumably to the local frame
     */
    ScanTransform T;
    /**
     * the raw points in the laser's coordinate frame
     */
    smPoint * points;
    /**
     * the projected points aftering being transformed by T.
     */
    smPoint * ppoints;
    /**
     * timestamp for laser scan.
     */
    int64_t utime;

    /**
     * The type of laser that was used to create this scan. (needed for finding contours)
     */
    sm_laser_type_t laser_type;
    /**
     *  The set of piecewise linear contours, (projected by T)
     */
    std::vector<Contour*> contours;

    /**
     * applyTransform:
     * @T_: ScanTransform to be applied to the set of points
     *
     * apply the ScanTransform T_ to the raw points in this scan and update
     *  its internal notion of T.
     *
     */
    inline void
    applyTransform(ScanTransform &T_)
    {
        sm_transformPoints(&T_, points, numPoints, ppoints);
        if (!contours.empty()) {
            double ct_diff = cos(T_.theta - T.theta), st_diff = sin(T_.theta
                    - T.theta);
            for (unsigned int i = 0; i < contours.size(); i++) {
                for (unsigned int j = 0; j < contours[i]->points.size(); j++) {
                    smPoint p = contours[i]->points[j];
                    p.x -= T.x;
                    p.y -= T.y;
                    contours[i]->points[j].x = T_.x + ct_diff * p.x - st_diff
                            * p.y;
                    contours[i]->points[j].y = T_.y + st_diff * p.x + ct_diff
                            * p.y;
                }
            }
        }
        T = T_;

    }

    /**
     * drawContours:
     * Convenience function for drawing the set of extracted contours
     */
    CvMat*
    drawContours(int maxDrawDim = 1000, double pixPerMeter = 100,
            smPoint * point1 = NULL, smPoint* point2 = NULL,
            double arrowScalFactor = 1);

    //for drawing contours
private:
    CvMat * drawIm;
    CvMat * displayIm;
    CvMat * squareDisplayIm;

};

}

#endif /* SCAN_H_ */
