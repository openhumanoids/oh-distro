#include "Contour.hpp"
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <queue>
#include <string>
#include <math.h>
#include <float.h>
#include <assert.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>

using namespace std;
using namespace scanmatch;

ContourExtractor::ContourExtractor(sm_laser_type_t laser_type)
{
    switch (laser_type)
        {
    case SM_HOKUYO_UTM:
        maxAdjacentDistance = 6;
        maxAdjacentAngularDistance = 3.1 * PI / 180; // radians
        minPointsPerContour = 2;
        alwaysOkayDistance = 0.33; // 0.20;
        maxDistanceRatio = 2;
        maxFirstDistance = 1;
        alwaysAcceptDistance = 0.1;
        searchRange = 10;
        minDistForDistRatio = .08;
        maxDistForSkippingDistRatio = .4;
        simplifyContourThresh = .025;
        break;

    case SM_HOKUYO_URG:
        //These used to work well for a URG, haven't tested in a while.
        maxAdjacentDistance = 0.3;
        maxAdjacentAngularDistance = 10.0 * PI / 180; // radians
        minPointsPerContour = 5;
        alwaysOkayDistance = 0.15; // 0.20;
        maxDistanceRatio = 100;
        maxFirstDistance = .2;
        alwaysAcceptDistance = 0.05;
        searchRange = 8;
        minDistForDistRatio = .08;
        maxDistForSkippingDistRatio = .4;
        simplifyContourThresh = 0;
        break;

    case SM_SICK_LMS:
        //THESE ARE WHAT ED USED... PRESUMABLY WITH A SICK
        maxAdjacentDistance = 5;
        maxAdjacentAngularDistance = 3.1 * PI / 180; // radians
        minPointsPerContour = 1;
        alwaysOkayDistance = 0.33; // 0.20;
        maxDistanceRatio = 1.8;
        maxFirstDistance = 1;
        alwaysAcceptDistance = 0.1;
        searchRange = 4;
        minDistForDistRatio = .08;
        maxDistForSkippingDistRatio = .4;
        simplifyContourThresh = 0;
        break;

    default:
        fprintf(stderr,
                "ERROR: unknown laser type (%d) for ContourExtraction!\n",
                laser_type);
        exit(1);

        }
}

ContourExtractor::~ContourExtractor()
{

}

void
ContourExtractor::findContours(smPoint * points, unsigned numValidPoints,
        std::vector<Contour*> &contours)
{

    priority_queue<Join*, std::vector<Join*>, JoinCompare> joins;

    std::vector<PointRecord> pointrecs;

    int range = searchRange;

    pointrecs.resize(numValidPoints);
    //  int i = 0;
    // create the point record table, one for every point.
    for (unsigned int parent = 0; parent < numValidPoints; parent++) {
        pointrecs[parent].point = points[parent];
    }

    // build the joins...
    // everybody gets their first pick to begin with.
    int numJoins = 0;
    for (unsigned int parent = 0; parent < pointrecs.size(); parent++) {
        Join *j;
        j = findClosestPoint(pointrecs, parent, parent - range, parent - 1);
        if (j != NULL) {
            joins.push(j);
            numJoins++;
        }

        j = findClosestPoint(pointrecs, parent, parent + 1, parent + range);
        if (j != NULL) {
            joins.push(j);
            numJoins++;
        }
    }

    // now we start plucking the best joins. If someone's best choice is
    // taken, we let them
    // pick a new partner and reinsert them into the queue.
    Join j;
    while (joins.size() > 0) {
        Join *jtmp = joins.top();
        joins.pop();
        j = *jtmp;
        delete jtmp;

        //    printf("JOIN cost=%f, parent=%d, victim=%d\n",j.cost,j.parent,j.victim);
        //    continue;

        // victim <--> parent
        if (j.parent > j.victim) {
            // if this parent has already been joined, we're done.
            if (pointrecs[j.parent].left >= 0)
                continue;

            // is the victim still available?
            if (pointrecs[j.victim].right < 0) {
                if (j.cost > alwaysOkayDistance) {
                    if (pointrecs[j.victim].leftDistance > minDistForDistRatio
                            && j.cost / pointrecs[j.victim].leftDistance
                                    > maxDistanceRatio)
                        continue;
                    else if (pointrecs[j.victim].leftDistance
                            < minDistForDistRatio && j.cost
                            > maxDistForSkippingDistRatio)
                        continue;
                    if (pointrecs[j.parent].rightDistance > minDistForDistRatio
                            && j.cost / pointrecs[j.parent].rightDistance
                                    > maxDistanceRatio)
                        continue;
                    else if (pointrecs[j.parent].rightDistance
                            < minDistForDistRatio && j.cost
                            > maxDistForSkippingDistRatio)
                        continue;

                    if (pointrecs[j.victim].leftDistance < 0
                            && pointrecs[j.parent].rightDistance < 0 && j.cost
                            > maxFirstDistance)
                        continue;
                }

                // yes, join.
                pointrecs[j.victim].right = j.parent;
                pointrecs[j.parent].left = j.victim;
                pointrecs[j.parent].leftDistance = j.cost;
                pointrecs[j.victim].rightDistance = j.cost;
            } else {
                // search for a new point.
                jtmp = findClosestPoint(pointrecs, j.parent, j.parent - range,
                        j.parent - 1);
                if (jtmp != NULL)
                    joins.push(jtmp);
            }

            continue;
        }

        // parent <--> victim. Same as above, roles reversed.
        if (j.parent < j.victim) {
            if (pointrecs[j.parent].right >= 0)
                continue;

            if (pointrecs[j.victim].left < 0) {
                if (j.cost > alwaysOkayDistance) {

                    if (pointrecs[j.parent].leftDistance > minDistForDistRatio
                            && j.cost / pointrecs[j.parent].leftDistance
                                    > maxDistanceRatio)
                        continue;
                    else if (pointrecs[j.parent].leftDistance
                            < minDistForDistRatio && j.cost
                            > maxDistForSkippingDistRatio)
                        continue;
                    if (pointrecs[j.victim].rightDistance > minDistForDistRatio
                            && j.cost / pointrecs[j.victim].rightDistance
                                    > maxDistanceRatio)
                        continue;
                    else if (pointrecs[j.victim].rightDistance
                            < minDistForDistRatio && j.cost
                            > maxDistForSkippingDistRatio)
                        continue;

                    if (pointrecs[j.parent].leftDistance < 0
                            && pointrecs[j.victim].rightDistance < 0 && j.cost
                            > maxFirstDistance)
                        continue;
                }

                pointrecs[j.victim].left = j.parent;
                pointrecs[j.parent].right = j.victim;
                pointrecs[j.parent].rightDistance = j.cost;
                pointrecs[j.victim].leftDistance = j.cost;

            } else {
                jtmp = findClosestPoint(pointrecs, j.parent, j.parent + 1,
                        j.parent + range);
                if (jtmp != NULL)
                    joins.push(jtmp);
            }
            continue;
        }

    }

    int contour = 0;

    // pull out the contours.
    for (unsigned int i = 0; i < pointrecs.size(); i++) {
        // we have a new contour.
        if (pointrecs[i].contour < 0) {
            //      if (debug)
            //        System.out.print("contour " + contour + " " + pointrecs[i].left + ": ");

            assert(pointrecs[i].left == -1);

            Contour * c = new Contour();
            int p = i;
            while (p >= 0) {
                //        if (debug)
                //          System.out.print(p + " ");
                c->points.push_back(pointrecs[p].point);
                pointrecs[p].contour = contour;
                p = pointrecs[p].right;
            }

            //      if (debug)
            //        System.out.println("");
            contour++;

            if (c->points.size() == 0) {
                printf("*Adding empty contour!?\n");
            }

            if (c->points.size() >= minPointsPerContour) {
                if (simplifyContourThresh > 0 && c->points.size() > 2) {
                  c->simplify(simplifyContourThresh);
                      }
		  contours.push_back(c);
            } else
                delete c;
        }
    }

    pointrecs.clear();

}

Join *
ContourExtractor::findClosestPoint(std::vector<PointRecord> &pointrecs,
        int parent, int a, int b)
{
    if (a < 0)
        a = 0;
    if (a >= (int) pointrecs.size())
        a = pointrecs.size() - 1;

    if (b < 0)
        b = 0;
    if (b >= (int) pointrecs.size())
        b = pointrecs.size() - 1;

    if (a == parent || b == parent)
        return NULL;

    int bestvictim = -1;
    double bestcost = DBL_MAX;

    // how far a span was there in this contour between the last two points?

    // parent better not already have children on both left & right.
    assert(!(pointrecs[parent].left >= 0 && pointrecs[parent].right >= 0));

    smPoint parentp = pointrecs[parent].point;
    for (int i = a; i <= b; i++) {
        PointRecord victim = pointrecs[i];
        if (i == parent)
            continue; //shouldn't happen
        if (victim.left >= 0 && parent < i)
            continue;
        if (victim.right >= 0 && parent > i)
            continue;

        double cost = sm_dist(&parentp, &victim.point);

        if (cost <= alwaysAcceptDistance) {// && i == parent + 1) {
            // stop looking.
            //      bestcost = cost / 4;
            bestcost = cost; //ed had this division by 4... not sure  why.
            bestvictim = i;
            break;
        }

        if (cost < bestcost && cost < maxAdjacentDistance) {
            //      double angularDistance = fabs(sm_angle_subtract(atan2(parentp.y, parentp.x), atan2(victim.point.y, victim.point.x)));
            //
            //      if (angularDistance > maxAdjacentAngularDistance) {
            //        printf("rejected due to Angular distance\n");
            //        continue;
            //      }

            bestcost = cost;
            bestvictim = i;
        }
    }

    // ///////////////////////////////////////////////////
    if (bestvictim < 0)
        return NULL;

    return new Join(parent, bestvictim, bestcost);
}



// Copyright 2002, softSurfer (www.softsurfer.com)
// This code may be freely used and modified for any purpose
// providing that this copyright notice is included with it.
// SoftSurfer makes no warranty for this code, and cannot be held
// liable for any real or imagined damage resulting from its use.
// Users of this code must verify correctness for their application.

//  simplify():
//    Input:  tol = approximation tolerance
//      allPoints should contain the complete set of points
//    Output:
//      points will contain the simplified set of points

void Contour::simplify(float tol)
{
  int n = points.size();
  allPoints = points;

  // STAGE 1.  Vertex Reduction within tolerance of prior vertex cluster
  int m = 1;// first point always kept
  int pv=0; //previously added ind
  for (int i = 1; i < n - 1; i++) {
    if (sm_dist(&points[i], &points[pv]) < tol)
      continue;
    points[m] = points[i];
    pv = m++;
  }
  points[m++] = points[n - 1]; //make sure end is added
  //m vertices in vertex reduced polyline
  points.resize(m);

  // STAGE 2.  Douglas-Peucker polyline simplification
  vector<bool> mk(points.size(), false);
  mk[0] = mk.back() = 1; // mark the first and last vertices
  simplifyDP(tol, points, 0, points.size() - 1, mk);

  // copy marked vertices to the output simplified polyline
  m = 0;
  for (int i = 0; i < points.size(); i++) {
    if (mk[i])
      points[m++] = points[i]; //m<=i;
  }
  //m vertices in simplified polyline
  points.resize(m);

}

// simplifyDP():
//  This is the Douglas-Peucker recursive simplification routine
//  It just marks vertices that are part of the simplified polyline
//  for approximating the polyline subchain v[j] to v[k].
//    Input:  tol = approximation tolerance
//            v[] = polyline array of vertex points
//            j,k = indices for the subchain v[j] to v[k]
//    Output: mk[] = array of markers matching vertex array v[]
void Contour::simplifyDP(float tol, std::vector<smPoint> &v, int j, int k, std::vector<bool> &mk)
{
  if (k <= j + 1) // there is nothing to simplify
    return;

  // check for adequate approximation by segment S from v[j] to v[k]
  int maxi = j; // index of vertex farthest from segment
  float max_dist_to_seg = 0;
  // test each vertex v[i] for max distance from segment v[j]-v[k]
  for (int i = j + 1; i < k; i++) {
    double dist_to_seg = sm_dist_to_segment(&v[i], &v[j], &v[k]);
    if (dist_to_seg <= max_dist_to_seg)
      continue;
    else {
      // v[i] is a new max vertex
      maxi = i;
      max_dist_to_seg = dist_to_seg;
    }
  }
  if (max_dist_to_seg > tol) // error is worse than the tolerance
  {
    // split the polyline at the farthest vertex
    mk[maxi] = true; // mark v[maxi] for the simplified polyline
    // recursively simplify the two subpolylines at v[maxi]
    simplifyDP(tol, v, j, maxi, mk); // polyline v[j] to v[maxi]
    simplifyDP(tol, v, maxi, k, mk); // polyline v[maxi] to v[k]
  }
  // else the approximation is OK, so ignore intermediate vertices
  return;
}
//===================================================================
