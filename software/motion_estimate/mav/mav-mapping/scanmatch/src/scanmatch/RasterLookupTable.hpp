/**
 * SECTION:scanmatch
 * @title: RasterLookupTable
 * @short_description: Local occupancy gridmap for scan matching
 * @include: bot/scanmatch/RasterLookupTable.hpp
 *
 * Maintain an occupancy gridmap for scanmatching.
 *
 * The internal functions for matching a scan are contained in this class.
 *
 *
 * Linking: -lscanmatch
 * namespace: scanmatch
 */

#ifndef RASTERLOOKUPTABLE_H_
#define RASTERLOOKUPTABLE_H_

#include "ScanMatchingUtils.hpp"
#include <vector>
#include <opencv/cv.h>

namespace scanmatch
{

class RasterLookupTable
{
public:
    /**
     * RasterLookupTable:
     * Constructor for a normal table.
     * @x0i: min x position in table in meters
     * @y0i: min y position in table in meters
     * @x1i: max x position in table in meters
     * @y1i: max y position in table in meters
     * @mPP: resolution of the table (meters per pixel)
     * @pixelDivisor: round size of table to ensure that the height/width are a factor of pixelDivisor
     * @initialValue: initialize entries in the table to this value
     *
     */
    RasterLookupTable(double x0i, double y0i, double x1i, double y1i,
            double mPP, int pixelDivisor = 1, unsigned char initialValue = 0);

    /**
     * RasterLookupTable:
     * Constructor for the low resolution table, corresponding to the hi_res table.
     * @hi_res: pointer to the corresponding high resolution table
     * @downsampleFactor: downsample the hi_res table by this factor
     */
    RasterLookupTable(RasterLookupTable * hi_res, int downsampleFactor);
    virtual
    ~RasterLookupTable();

    /**
     * evaluate2D:
     * Perform an exhaustive search over a grid of possible translational transforms of
     *  the scan to find the best one.(searches X,Y)
     * @points: laser points
     * @numPoints: number of laser points
     * @XYT0: center of the search window
     * @prior: initial guess for the aligning position of the scan (normally XTY0==prior, except when
     * performing a multi-resolution search)
     * @ixrange: x-range to be searched in pixels
     * @iyrange: y-range to be searched in pixels
     * @ixdim: x-size of the scores array
     * @iydim: y-size of the scores array
     * @bestScoreIndX: return value for x-index of the best score in the scores array
     * @bestScoreIndY: return value for y-index of the best score in the scores array
     * Returns: the ScanTransform corresponding to the best alignment
     *
     */
    ScanTransform
    evaluate2D(const smPoint * points, const unsigned numPoints,
            const ScanTransform * XYT0, const ScanTransform * prior,
            int ixrange, int iyrange, int ixdim, int iydim, float * scores,
            int * bestScoreIndX, int *bestScoreIndY);

    /**
     * evaluate3D:
     * Perform an exhaustive search over a grid of possible translational and rotational transforms of the
     *  scan to find the best one. (searches X,Y,theta)
     * @points: laser points
     * @numPoints: number of laser points
     * @prior: initial guess for the aligning position of the scan. Used as the center of the window
     * @xrange: x-range to be searched in meters
     * @yrange: y-range to be searched in meters
     * @thetarange: theta-range to be searched in radians
     * @dthetastep: step size to use in theta
     * @xSat: return value for whether the best transform was on the x-edge of the search window
     * @ySat: return value for whether the best transform was on the y-edge of the search window
     * @thetaSat: return value for whether the best transform was on the theta-edge of the search window
     * Returns: the ScanTransform corresponding to the best alignment
     *
     */
    ScanTransform
    evaluate3D(const smPoint * points, const unsigned numPoints,
            const ScanTransform * prior, double xrange, double yrange,
            double thetarange, double dthetastep, int * xSat, int *ySat,
            int * thetaSat);

    /**
     * evaluate3D_multiRes:
     * Perform a multi-resolution search over the grid of possible translational and rotational transforms of the
     *  scan to find the best one. (searches X,Y,theta)
     * @points: laser points
     * @numPoints: number of laser points
     * @prior: initial guess for the aligning position of the scan. Used as the center of the window
     * @xrange: x-range to be searched in meters
     * @yrange: y-range to be searched in meters
     * @thetarange: theta-range to be searched in radians
     * @dthetastep: step size to use in theta
     * @xSat: return value for whether the best transform was on the x-edge of the search window
     * @ySat: return value for whether the best transform was on the y-edge of the search window
     * @thetaSat: return value for whether the best transform was on the theta-edge of the search window
     * Returns: the ScanTransform corresponding to the best alignment
     *
     */
    ScanTransform
    evaluate3D_multiRes(RasterLookupTable * hi_res, const smPoint * points,
            const unsigned numPoints, const ScanTransform * prior,
            double xrange, double yrange, double thetarange, double thetastep,
            int * xSat, int *ySat, int * thetaSat);

    /**
     * getScore:
     * evaluate the score of the transform XYT0
     */
    float
    getScore(const smPoint * points, const unsigned numPoints,
            const ScanTransform * XYT0);
    /**
     * getScoreDump:
     * evaluate the score of the transform XYT0, and dump individual point scores to a file
     */
    float
    getScoreDump(const smPoint * points, const unsigned numPoints,
            const ScanTransform * XYT0, const char * name);

    /**
     * getNumHits:
     * get number of points that are given a likelihood above the "hit" threshold
     */
    int
    getNumHits(const smPoint * points, const unsigned numPoints,
            const ScanTransform * XYT);

    /**
     * drawRectangle:
     * function used by Ed's method for rendering the lookup table.
     * renders the cost function for a line segment in the rectangular region around the segment
     */
    void
    drawRectangle(double cx, double cy, double x_size, double y_size,
            double theta, unsigned char *lutSq, int lutSq_size,
            int lutSq_first_zero, double lutSqRange);

    /**
     * makeLut:
     * create the lookup table used by the drawRectangle function
     */
    static unsigned char *
    makeLut(int sz, double maxChiSq, double weight, int *lutSq_first_zero);

    /**
     * dumpTable:
     * dump the contents of this table to an image file (.bmp)
     */
    void
    dumpTable(const char * varName);

    /**
     * drawTable:
     * render the table using openCV for visualization.
     * returns a pointer to the rendered CvMat image
     */
    CvMat *
    drawTable();

    /**
     * drawQuad:
     * draw a symbol for the robot in the drawImg
     */
    void
    drawRobot(CvMat * drawImg, ScanTransform transf, CvScalar color);
    /**
     * drawScan:
     * draw the laser scan
     */
    void
    drawScan(CvMat * drawImg, smPoint * points, unsigned numPoints,
            ScanTransform transf, CvScalar color);

    /**
     * worldToTable:
     * accessor function that converts from world coordinates in meters to the table coordinates in pixels
     */
    inline void
    worldToTable(double x, double y, int * ix, int * iy)
    {
        *ix = sm_clamp(round((x - x0) * pixelsPerMeter), 0, width - 1);
        *iy = sm_clamp(round((y - y0) * pixelsPerMeter), 0, height - 1);
    }

    /**
     * worldToTable:
     * accessor function that converts from table coordinates in pixels to world coordinates in meters
     */
    inline void
    tableToWorld(int ix, int iy, double * x, double * y)
    {
        //    *x = ((double)ix+0.5) * metersPerPixel + x0; //+.5 puts it in the center of the cell
        //    *y = ((double)iy+0.5) * metersPerPixel + y0;
        *x = ((double) ix) * metersPerPixel + x0;
        *y = ((double) iy) * metersPerPixel + y0;

    }

    /**
     * scoresToWorld:
     * convert from the scoring matrix indices back to world coordinates in meters
     */
    inline void
    scoresToWorld(const ScanTransform * XYT0, int ixrange, int iyrange, int sx,
            int sy, double * x, double * y)
    {
        int ixXYT0, iyXYT0;
        worldToTable(XYT0->x, XYT0->y, &ixXYT0, &iyXYT0);
        int ix = ixXYT0 - ixrange + sx;
        int iy = iyXYT0 - iyrange + sy;
        tableToWorld(ix, iy, x, y);
    }

    /**
     * readTable:
     * returns the value of the table at pixel ix,iy
     */
    inline int
    readTable(int ix, int iy)
    {
        return distdata[iy * width + ix];
    }

    /**
     * readTable:
     * returns the value of the table at position x,y (in meters)
     */
    inline int
    readTable(double x, double y)
    {
        int ix, iy;
        worldToTable(x, y, &ix, &iy);
        return readTable(ix, iy);
    }

    /**
     * writeTable:
     * explicitly set the value of pixel ix,iy to v
     */
    inline int
    writeTable(int ix, int iy, unsigned char v)
    {
        return distdata[iy * width + ix] = v;
    }

    /**
     * writeTable:
     * explicitly set the value at position x,y (in meters)to v
     */
    inline int
    writeTable(double x, double y, unsigned char v)
    {
        int ix, iy;
        worldToTable(x, y, &ix, &iy);
        return writeTable(ix, iy, v);
    }

    /**
     * worldToDisplay:
     * convert from world coordinates to the rendered display image coordinates
     */
    inline void
    worldToDisplay(double x, double y, int * ix, int * iy)
    {
        int ixt, iyt;
        worldToTable(x, y, &ixt, &iyt);
        double displayRatio = (double) maxDrawDim / (double) sm_imax(width,
                height);
        *ix = (int) (ixt * displayRatio);
        *iy = (int) (iyt * displayRatio);
    }

    //extremum of the table in meters
    double x0, y0, x1, y1;
    //resolution of the table
    double pixelsPerMeter, metersPerPixel;
    //pixel dimensions
    int width, height;

    //the actual occupancy grid data
    unsigned char * distdata;
    //opencv header for the occupancy grid
    CvMat distim;

    //internal structures for drawing
    CvMat * drawim;
    CvMat * drawim_color;
    int maxDrawDim;

    //hardcoded parameter
    static const int hitThresh = 100; // a hit is counted when the rlt score is >=
    // hitThresh


};
}
#endif /*RASTERLOOKUPTABLE_H_*/
