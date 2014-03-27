/**
 * SECTION:scanmatch
 * @title: Contour
 * @short_description: Extract piecewise linear contours from a laser scan
 * @include: bot/scanmatch/ScanMatcher.hpp
 *
 * Iteratively connect nearby points to extract a set of piecewise linear
 * contours from a laser scan.
 *
 * These parameters work pretty well with a hokuyo UTM, but others might be better.
 *
 * Linking: -lscanmatch
 * namespace: scanmatch
 */

#ifndef CONTOUR_H_
#define CONTOUR_H_
#include <vector>
#include "ScanMatchingUtils.hpp"

namespace scanmatch
{

/*
 * Utility class for keeping track of which points are part of which contour
 */
class PointRecord
{
public:
    PointRecord() :
        left(-1), right(-1), contour(-1), leftDistance(-1), rightDistance(-1)
    {

    }
    smPoint point;
    int left; // index of our left neighbor (whose idx is smaller)
    int right; // index of our right neighbor (whose idx is bigger)
    int contour; // which contour # we are part of.

    // distance to the left and right points
    double leftDistance;
    double rightDistance;

};

/*
 * Utility class for maintaing the score of candidate contour merges
 */
class Join
{
public:
    int parent;
    int victim;
    double cost;

    Join() :
        parent(-1), victim(-1), cost(-1)
    {
    }
    Join(int p, int v, double c)
    {
        parent = p;
        victim = v;
        cost = c;
    }
};

/*
 * comparator class that allows us to put the joins into a priority_queue
 */
class JoinCompare
{
public:
    bool
    operator()(const Join * j1, const Join * j2)
    {
        double c = j1->cost - j2->cost;
        if (fabs(c) > 1e-4)
            return c > 0;
        //      printf("got equal costs, j1->cost=%.3f, j2->cost = %.3f,j1->parent=%d, j1->victim=%d, j2->parent=%d, j2->victim=%d\n",
        //          j1->cost,j2->cost,j1->parent,j1->victim,j2->parent,j2->victim);
        c = j1->parent - j2->parent;
        if (fabs(c) > 1e-4)
            return c > 0;

        c = j1->victim - j2->victim;
        return c > 0;
    }
};

/**
 * class for storing a contour... for now its just a vector of points
 */
class Contour
{
public:
    Contour()
    {

    }
    ~Contour()
    {
        points.clear();
    }
    /**
     * The list of points that make up this contour
     */
    std::vector<smPoint> points;
    std::vector<smPoint> allPoints;
    /**
     * simplify the points that make up the contour
     */
    void simplify(float tol);
private:
  static void simplifyDP(float tol, std::vector<smPoint> &v, int j, int k, std::vector<bool> &mk);

};

class ContourExtractor
{
public:
    ContourExtractor(sm_laser_type_t laser_type = SM_HOKUYO_UTM);
    virtual
    ~ContourExtractor();

    /**
     * findContours: Function to extract a set of piecewise linear contours
     * from the set of points from a laser scan.
     *
     * Makes some assumptions about the ordering of points.
     *
     * @points: the set of points
     * @numPoints: number of points
     *
     * @countours: Output parameter. The set of connected piecewise linear
     * contours extracted from the laser scan.
     */
    void
    findContours(smPoint * points, unsigned numPoints,
            std::vector<Contour*> &contours);

private:
    /**
     * findClosestPoint:
     *
     * Internal function, that returns the join candidate for the parent
     *  between points a and b.
     *
     * Returns: a Join candidate, or NULL if there are no suitable candidates
     */
    Join *
    findClosestPoint(std::vector<PointRecord> &pointrecs, int parent, int a,
            int b);
public:
    /**
     * Parameters used for finding contours!
     */

    /**
     * When adding a point to a contour, it's never okay to add a point farther
     * than this away.
     */
    double maxAdjacentDistance;
    /**
     * When adding a point to a contour, it's never okay to add a point which
     * was this many degrees away (as measured by laser scanning angle).
     */
    double maxAdjacentAngularDistance; // radians
    /** Discard contours that have fewer than this many points. * */
    unsigned int minPointsPerContour;
    /**
     * It's always okay to add a point to a contour if it's less than this far
     * away. This rule overrides any other conditions.
     */
    double alwaysOkayDistance; // 0.20;
    /**
     * It's not okay to suddenly increase the distance between consecutive
     * points.
     */
    double maxDistanceRatio;
    /**
     * Don't start a new contour between two points if they're more than this
     * far apart.
     */
    double maxFirstDistance;
    /**
     * When looking for the next nearest point in the contour, if we find a
     * neighbor that's this close, we stop looking for better points. This helps
     * us avoid segmenting contours that are actually just the result of
     * quantization noise in the sensor. Only affects adjacent scan points.
     */
    double alwaysAcceptDistance;
    /**
     * When looking for possible points to add to a contour, how many points
     * away do we look?
     */
    int searchRange;
    /**
     * make sure that the previous points are at least this far away for
     * computing the distance ratio test.
     */
    double minDistForDistRatio;

    /**
     * if we're skipping the distance ratio test, make sure that points are less
     * than this far apart.
     */
    double maxDistForSkippingDistRatio;

    /**
     * Simplify the extracted contours to by removing redundant points within this far of a straight line
     */
    double simplifyContourThresh;

};

}

#endif /*CONTOUR_H_*/
