/**
 * SECTION:scanmatch
 * @title: ScanMatcher
 * @short_description: Class for peforming scan matching of 2D laser scans
 * @include: bot/scanmatch/RasterLookupTable.hpp
 *
 * A fast and accurate scan matching module for aligning 2D laser scans.
 *
 * The scan matcher maintains a sliding window local occupancy gridmap for scanmatching.
 *
 *
 * Linking: -lscanmatch
 * namespace: scanmatch
 */

#ifndef SCANMATCHER_H_
#define SCANMATCHER_H_

#include "RasterLookupTable.hpp"
#include "Contour.hpp"
#include "Scan.hpp"
#include <vector>
#include <list>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pthread.h>
#include <float.h>

namespace scanmatch
{

  /*
   * matchingMode indicates which type of matching should be applied
   * SM_GRID_ONLY   - don't run coordinate ascent
   * SM_GRID_COORD  - run coordinate ascent after grid search (a safe default)
   * SM_Y_GRID_COORD- run coordinate ascent on y-theta for height matching after grid
   * SM_RUN_GRID    - dummy enum for separating the mode types
   * SM_COORD_ONLY  - only run the coordinate ascent
   * SM_Y_COORD_ONLY- only run the coordinate ascent on y-theta for height matching
   */
  typedef enum{
    SM_GRID_ONLY=0,
    SM_GRID_COORD =1,
    SM_Y_GRID_COORD=2,
    SM_RUN_GRID  = 3, //dummy enum... don't use
    SM_COORD_ONLY =4,
    SM_Y_COORD_ONLY =5 //don't search in x (for height matching)
  }sm_incremental_matching_modes_t;

class ScanMatcher
{
public:
    /*
     * ScanMatcher:
     * Constructor for the scan matching object.
     * Pass in a the matching parameters.
     * @metersPerPixel: Resolution for the local gridmap, and the xy resolution for the grid search
     * @thetaResolution_: theta resolution for the grid search
     * @useMultires_: perform multi resolution matching, with the specified downsampling factor.
     *          useMultires_=0, no downsampling
     *          useMultires_=1, downsample low res by factor of 2
     *          useMultires_=k, downsample low res by factor of 2^k
     *          normally, ~3 is a good value
     * @useThreads: using threading to make the occupancy map rebuilds occur in a background thread
     * @verbose: print out some status info
     */
    ScanMatcher(double metersPerPixel_, double thetaResolution_,
            int useMultires_, bool useThreads_, bool verbose_);

    /**
     * initSuccessiveMatching:
     * set the parameters for successive matching to other than the default:
     * @initialSearchRangeXY_: nominal search region in meters for the grid search in XY.
     *          If the best match is on the edge of the region,
     *          the window may be expanded up to the maxSearchRangeXY
     * @maxSearchRangeXY_: max XY window size
     * @initialSearchRangeTheta_: same, but for theta/radians
     * @maxSearchRangeTheta_: same, but for theta/radians
     * @matchingMode_: which matching mode to use
     * @addScanHitThresh_: add a scan to the local map if the percentage of points that are "hits" is below this threshold.
     * @stationaryMotionModel_: use a "stationary" motion model instead of the normal constant velocity one
     * @motionModelPriorWeight_: indicates the standard deviation of the motion model estimate.
     *                          If its less than 0.1, we don't use the priors for anything other
     *                          than centering the search window
     * @startPose: initialize pose to something other than (0,0)-0
     */
    void
    initSuccessiveMatchingParams(unsigned int maxNumScans_,
            double initialSearchRangeXY_, double maxSearchRangeXY_,
            double initialSearchRangeTheta_, double maxSearchRangeTheta_,
            sm_incremental_matching_modes_t matchingMode_, double addScanHitThresh_,
            bool stationaryMotionModel_,double motionModelPriorWeight_,
            ScanTransform * startPose);

    virtual
    ~ScanMatcher();

    /**
     * matchSuccessive:
     * Match consecutive laser scans with each other.
     * Assumes the vehicle moves with a constant velocity between scans.
     * This method that should be called on successive incoming laser scans to perform "incremental" scan matching.
     *
     * @points: laser points (in body frame)
     * @numPoints: number of laser points
     * @laser_type: the type of laser that generated the points
     * @utime: timestamp for the laser reading
     * @preventAddScan: perform the matching, but don't allow the scan to be added to the map
     *          (used in case the vehicle is pitching/rolling too much)
     * @prior: If you have a better guess than constant velocity propogation (ie: from odometry)
     *          pass in an initial guess for the aligning position of the scan to be used as the center of the window.
     *          otherwise leave prior=NULL.
     *          The prior.score field indicates the standard deviation of the motion model estimate.
     *          If its less than 0.1, we don't use the prior for anything other than centering the search window,
     *          otherwise a gaussian prior with that standard deviation is used.
     * @xrange: x-range to be searched in meters
     * @yrange: y-range to be searched in meters
     * @thetarange: theta-range to be searched in radians
     *
     * Returns: a ScanTransform containing the best alignment for the input set of point
     */
    ScanTransform
    matchSuccessive(smPoint * points, unsigned numPoints,
            sm_laser_type_t laser_type, int64_t utime, bool preventAddScan =
                    false, ScanTransform * prior = NULL);
    /*
     * gridMatch:
     * match the set of points to the current map (perform Scan Matching)
     * this would be the interface to use for "loop-closing"
     *
     * @points: laser points (in body frame)
     * @numPoints: number of laser points
     * @prior: initial guess for the aligning position of the scan. Used as the center of the window
     *          The prior.score field indicates the standard deviation of the motion model estimate.
     *          If its less than 0.1, we don't use the prior for anything other than centering the search window,
     *          otherwise a gaussian prior with that standard deviation is used.
     * @xrange: x-range to be searched in meters
     * @yrange: y-range to be searched in meters
     * @thetarange: theta-range to be searched in radians
     * @xSat: return value for whether the best transform was on the x-edge of the search window
     * @ySat: return value for whether the best transform was on the y-edge of the search window
     * @thetaSat: return value for whether the best transform was on the theta-edge of the search window
     *
     * Returns: a ScanTransform containing the best alignment for the input set of point
     */
    ScanTransform
    gridMatch(smPoint * points, unsigned numPoints, ScanTransform * prior,
            double xRange, double yRange, double thetaRange, int * xSat = NULL,
            int *ySat = NULL, int * thetaSat = NULL);

    /*
     * coordAscentMatch:
     * Use gradient ascent to refine the resulting match...
     * should normally be done on the result from calling ScanMatcher::match()
     * to get a reasonable startPoint
     * @points: the set of points that make up the scan to be matched
     * @numPoints: number of points
     * @startPoint: initial guess of the alignment, needs to be close for gradient ascent to work
     *
     */
    ScanTransform
    coordAscentMatch(smPoint * points, unsigned numPoints,
            ScanTransform * startPoint);

    /*
     * addScanToBeProcessed:
     * add this scan to the queue to get processed in a background thread. This ensures
     * we can rebuild the map without disturbing the realtime processing thread
     * if useThreads is false, this just calls the regular addScan
     *
     * @points: the set of points that make up the scan
     * @numPoints: number of points
     * @T: the transform to project the points to the world coordinate frame
     * @laser_type: the type of laser that generated the points
     * @utime: timestamp for the laser reading
     *
     */
    void
    addScanToBeProcessed(smPoint * points, unsigned numPoints,
            ScanTransform * T, sm_laser_type_t laser_type, int64_t utime);

    /*
     * addScan:
     * add this scan to the map immediately
     *
     * @points: the set of points that make up the scan
     * @numPoints: number of points
     * @T: the transform to project the points to the world coordinate frame
     * @laser_type: the type of laser that generated the points
     * @utime: timestamp for the laser reading
     * @rebuildNow: if true, rlt will be rebuilt, otherwise,
     *     the scan will just be added to the list without touching the occupancy map
     */
    void
    addScan(smPoint * points, unsigned numPoints, ScanTransform *T,
            sm_laser_type_t laser_type, int64_t utime, bool rebuildNow = true);

    /*
     * addScan:
     * add this scan to the map immediately
     * @s: the Scan to be added
     * @rebuildNow: if true, rlt will be rebuilt, otherwise,
     *     the scan will just be added to the list without touching the occupancy map
     */
    void
    addScan(Scan *s, bool rebuildNow);

    /*numScans:
     * get the number of scans in currently being used
     */
    inline int
    numScans()
    {
        return scans.size();
    }

    /*
     * clearScans:
     * remove all the scans from the current local map
     * @deleteScans: also delete the contained Scan objects
     */
    void
    clearScans(bool deleteScans);

    /*
     * drawGUI:
     * draw a redimentary visualization of whats goin on with the scan matcher using openCV
     */
    void
    drawGUI(smPoint * points, unsigned numPoints, ScanTransform transf,
            CvMat * heightGui = NULL, const char * guiname = "ScanMatcher",
            CvScalar scanColor = CV_RGB(255, 0, 0));

    /*
     * isUsingIPP:
     * check whether the scan matcher was compiled with IPP
     */
    int isUsingIPP();
private:
    /*
     * rebuildRaster:
     * rebuild rasterTable using the current set of scans
     */
    void
    rebuildRaster(RasterLookupTable ** rasterTable);

    void
    rebuildRaster_olson(RasterLookupTable ** rasterTable);
    void
    rebuildRaster_blur(RasterLookupTable ** rasterTable);
    void
    drawBlurredScan(RasterLookupTable ** rasterTable, Scan * s);
    void
    rebuildRaster_blurLine(RasterLookupTable ** rasterTable);
    /**
     * computeBouds:
     * compute the bounds of the currently stored set of scans
     */
    void
    computeBounds(double *minx, double *miny, double *maxx, double *maxy);

public:
    /*
     * General scan matching params:
     */
    /*
     * Resolution for the local gridmap, and the xy resolution for the grid search
     */
    double metersPerPixel;

    /*
     * theta resolution for the grid search
     */
    double thetaResolution;

    /*
     * Max number of scans to keep in the local sliding window map
     */
    unsigned int maxNumScans;

    /*
     * use the mutli resolution search, should be faster for large search windows (and small, but not as big of a difference)
     */
    bool useMultiRes;
    /*
     * if using the multi-res search, low res table should be downsampled by this factor
     */
    int downsampleFactor;

    /*
     * rebuild the local occupancy map in a background thread.
     */
    bool useThreads;

    /*
     * successive matching params:
     * These only have an effect if using the matchSuccessive Interface
     */

    /*
     * add a scan to the local map if the percentage of points that are "hits" is below this threshold.
     */
    double addScanHitThresh;
    /**
     * default search range to be search over in xy
     */
    double initialSearchRangeXY;
    /**
     * If the best transform is on the edge of the window, the search range may be expanded up to this much
     * for the search over in xy
     */
    double maxSearchRangeXY;
    /**
     * default search range to be search over in theta
     */
    double initialSearchRangeTheta;
    /**
     * If the best transform is on the edge of the window, the search range may be expanded up to this much
     * for the search over in theta
     */
    double maxSearchRangeTheta;

    /*
     * indicates which type of matching should be applied
     */
    sm_incremental_matching_modes_t matchingMode;

    /*
     * indicates weather we want to use the stationary motion model instead of the usual constant velocity motion model.
     */
    bool stationaryMotionModel;

    /*
     * indicates the standard deviation of the motion model estimate.
     * If we're using one of the above motion models.
     * If its less than 0.1, we don't use the prior for anything other than centering the search window
     */
    double motionModelPriorWeight;

    bool verbose;

    /*
     * current pose from successive matching
     */
    ScanTransform currentPose;
    /*
     * previous pose from successive matching
     */
    ScanTransform prevPose;

    /*
     * list of scans that are used to create the Raster Lookup table (occupancy map)
     */
    std::list<Scan *> scans;
    /*
     * The internal occupancy map
     */
    RasterLookupTable *rlt;

private:
    RasterLookupTable *rltTmp;

    RasterLookupTable *rlt_low_res;
    RasterLookupTable *rltTmp_low_res;

    //stuff for creating raster table using line drawing primitive
    unsigned int kernelSize;
    unsigned char * squareKernel;
    unsigned char * lineKernel;
    unsigned char * diagLineKernel;

    CvMat * fullGui;

    //threading stuff
    int killThread;
    int cancelAdd;
    pthread_t rebuilder;
    static void *
    rebuildThreadFunc(ScanMatcher*parent);
    std::list<Scan *> scansToBeProcessed;

    pthread_mutex_t scans_mutex; //this ordering must be obeyed... lock scans first!
    pthread_mutex_t rlt_mutex;
    pthread_mutex_t toBeProcessed_mutex; //shouldn't need to be locked alongside anything else...
    pthread_cond_t toBeProcessed_cv;

};

}


#endif /*SCANMATCHER_H_*/
