#ifdef USE_IPP
#include "ipp.h"
#endif

#include "RasterLookupTable.hpp"
#include "ScanMatchingUtils.hpp"
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "ScanMatchingOpencvUtils.hpp"
#include "ScanMatchingUtils.hpp"
#include <vector>
#include <algorithm>


#define DRAW_COST_SURFACE 0
#if DRAW_COST_SURFACE
//for debugging/visualization
#include <bot/bot_core.h>
#include <GL/gl.h>
#endif

#define SCALE_COVARIANCE 1 //TODO: this should probably be a parameter and be done elsewhere
#define VAR_COMP_THRESH .80

using namespace std;
using namespace scanmatch;

typedef struct
{
    double x[3];
    double p;
} cov_sample_t;

RasterLookupTable::~RasterLookupTable()
{
    free(distdata); //distim is just the header, so doesn't need to  be freed
    cvReleaseMat(&drawim);
    cvReleaseMat(&drawim_color);

}

RasterLookupTable::RasterLookupTable(double x0i, double y0i, double x1i,
        double y1i, double mPP, int pixelDivisor, unsigned char initialValue)
{
    x0 = x0i;
    y0 = y0i;
    x1 = x1i;
    y1 = y1i;
    metersPerPixel = mPP;
    pixelsPerMeter = 1.0 / metersPerPixel;

    width = ceil((pixelsPerMeter) * (x1 - x0));
    height = ceil((pixelsPerMeter) * (y1 - y0));

    //make it divisible by pixelDivisor
    width = ceil(width / (double) pixelDivisor) * pixelDivisor;
    height = ceil(height / (double) pixelDivisor) * pixelDivisor;

    //  printf("x0=%.3f, x1=%.3f, x0=%.3f, x1=%.3f, width=%d, height=%d\n", x0, x1, y0, y1, width, height);

    if (width > 8192 || height > 8192)
        fprintf(stderr, "RasterLookupTable: Enormous dimensions: %dx%d\n",
                width, height);
    if (width <= 0 || height <= 0) {
        fprintf(stderr, "ERROR:width %d or height %d is less than 0\n",width,height);
        exit(1);
    }

    int distdata_size = width * height * sizeof(unsigned char);
    distdata = (unsigned char *) malloc(distdata_size);
    memset(distdata, initialValue, distdata_size);
    distim = cvMat(height, width, CV_8UC1, distdata);
    drawim = NULL;
    drawim_color = NULL;
    maxDrawDim = 640;

}

RasterLookupTable::RasterLookupTable(RasterLookupTable * hi_res,
        int downsampleFactor)
{
    x0 = hi_res->x0;
    y0 = hi_res->y0;
    x1 = hi_res->x1;
    y1 = hi_res->y1;
    metersPerPixel = hi_res->metersPerPixel * downsampleFactor;
    pixelsPerMeter = 1.0 / metersPerPixel;

    width = ceil((pixelsPerMeter) * (x1 - x0));
    height = ceil((pixelsPerMeter) * (y1 - y0));
    assert(width==hi_res->width/downsampleFactor);
    assert(height==hi_res->height/downsampleFactor);

    int distdata_size = width * height * sizeof(unsigned char);
    distdata = (unsigned char *) malloc(distdata_size);
    memset(distdata, 0, distdata_size);
    distim = cvMat(height, width, CV_8UC1, distdata);
    drawim = NULL;
    drawim_color = NULL;
    maxDrawDim = 1000;

    //    sm_tictoc("downsample_window");
    //    //downsample the high res table
    //    //each pixel is set to the max of all its high-res counterparts
    //    CvRect hres_rect;
    //    CvMat hres_window;
    //    hres_rect.width = 2* downsampleFactor + 1;
    //    hres_rect.height = 2* downsampleFactor + 1;
    //    for (int iy = 1; iy < height - 1; iy++) { //border should be 0, so ignore it for simplicity
    //        for (int ix = 1; ix < width - 1; ix++) {
    //          sm_tictoc("getInd");
    //            double x, y;
    //            tableToWorld(ix, iy, &x, &y);
    //            int hix, hiy;
    //            hi_res->worldToTable(x, y, &hix, &hiy);
    //            hres_rect.x = hix - downsampleFactor;
    //            hres_rect.y = hiy - downsampleFactor;
    //            sm_tictoc("getInd");
    //            sm_tictoc("subrect");
    //            cvGetSubRect(&hi_res->distim, &hres_window, hres_rect);
    //            sm_tictoc("subrect");
    //
    //            double min_val, max_val;
    //            sm_tictoc("minmax");
    //            cvMinMaxLoc(&hres_window, &min_val, &max_val);
    //            sm_tictoc("minmax");
    //            distdata[iy * width + ix] = max_val;
    //        }
    //    }
    //    sm_tictoc("downsample_window");
    ////    cvSaveImage("windowed.bmp",&distim);
    //
    //  //check that each pixel is set to the max of all its high-res counterparts
    //    for (int i = 0; i < hi_res->height; i++) {
    //      for (int j = 0; j < hi_res->width; j++) {
    //        int lind = i / downsampleFactor * width + j / downsampleFactor;
    //        int hind = i * hi_res->width + j;
    //        assert(distdata[lind] >=  hi_res->distdata[hind]);
    //      }
    //    }

    //    sm_tictoc("downsample_exp");
    //      //downsample the high res table
    //      //each pixel is set to the max of all its high-res counterparts
    //      for (int i = 0; i < hi_res->height; i++) {
    //        for (int j = 0; j < hi_res->width; j++) {
    //          int lind = i / downsampleFactor * width + j / downsampleFactor;
    //          int hind = i * hi_res->width + j;
    //          distdata[lind] = sm_ucmax(distdata[lind], hi_res->distdata[hind]);
    //        }
    //      }
    //    sm_tictoc("downsample_exp");

    sm_tictoc("downsample_dilate");
    //downsample the high res table
    CvMat * tmp = cvCreateMat(hi_res->height, hi_res->width, CV_8UC1);
    int sz = 1.5 * downsampleFactor + 1;
    IplConvKernel* element = cvCreateStructuringElementEx(sz, sz, sz / 2, sz
            / 2, CV_SHAPE_RECT);

    cvDilate(&hi_res->distim, tmp, element, 1);
    cvResize(tmp, &distim, CV_INTER_NN);
    //    cvSaveImage("dilated.bmp",&distim);

    cvReleaseStructuringElement(&element);
    cvReleaseMat(&tmp);
    sm_tictoc("downsample_dilate");
}

void
RasterLookupTable::drawRectangle(double cx, double cy, double x_size,
        double y_size, double theta, unsigned char * lutSq, int lutSq_size,
        int lutSq_first_zero, double lutSqRange)
{
    double ux = cos(theta), uy = sin(theta);

    double lutRange = sqrt(lutSqRange);
    double invLutSqRange = 1.0 / lutSqRange;

    double x_bound = (x_size / 2.0 * fabs(ux) + y_size / 2.0 * fabs(uy))
            + lutRange;
    double y_bound = (x_size / 2.0 * fabs(uy) + y_size / 2.0 * fabs(ux))
            + lutRange;

    int ix0 = sm_clamp((int) ((cx - x_bound - x0) * pixelsPerMeter), 0, width
            - 1);
    int ix1 = sm_clamp((int) ((cx + x_bound - x0) * pixelsPerMeter), 0, width
            - 1);

    int iy0 = sm_clamp((int) ((cy - y_bound - y0) * pixelsPerMeter), 0, height
            - 1);
    int iy1 = sm_clamp((int) ((cy + y_bound - y0) * pixelsPerMeter), 0, height
            - 1);

    double y = y0 + (iy0 + .5) * metersPerPixel;

    for (int iy = iy0; iy <= iy1; iy++) {

        double x = x0 + (ix0 + .5) * metersPerPixel;

        for (int ix = ix0; ix <= ix1; ix++) {

            // distances from query point to center of rectangle
            double dx = x - cx, dy = y - cy;

            // how long are the projections of the vector (dx,dy) onto the
            // two principle
            // components of the rectangle? How much longer are they than
            // the dimensions
            // of the rectangle?
            double c1 = fabs(dx * ux + dy * uy) - (x_size / 2);
            double c2 = fabs(-dx * uy + dy * ux) - (y_size / 2);

            // if the projection length is < 0, we're *inside* the
            // rectangle.
            c1 = fmax(0, c1);
            c2 = fmax(0, c2);

            double distSq = sm_sq(c1) + sm_sq(c2);

            int lutSqIdx = (int) (lutSq_size * distSq * invLutSqRange + .5);
            //      printf("dist = %f, lutSqIdx=%d, lutSq_first_zero=%d\n",distSq,lutSqIdx,lutSq_first_zero);

            if (lutSqIdx < lutSq_first_zero) {
                int idx = iy * width + ix;
                distdata[idx] = sm_ucmax(distdata[idx], lutSq[lutSqIdx]);
            }

            x += metersPerPixel;
        }

        y += metersPerPixel;
    }

}

/**
 * Make a lookup table with 'sz' entries. The lookup table will have an
 * exponential fall-off with maximum value 255.
 *
 * lut[0] = weight*255 ... lut[i] = weight*255*e^{-i*alpha} ... lut[sz] =
 * weight*255*e^{-maxChiSq}
 *
 * Weight should be [0,1].
 */
unsigned char *
RasterLookupTable::makeLut(int sz, double maxChiSq, double weight,
        int *lutSq_first_zero)
{
    unsigned char * lutSq = (unsigned char *) malloc(sz * sizeof(char));
    bool set_lutSq_first_zero = false;
    for (int i = 0; i < sz; i++) {
        int v = (int) round(255.0 * weight * exp(-maxChiSq * i / (sz - 1)));
        if (v == 0 && !set_lutSq_first_zero) {
            set_lutSq_first_zero = true;
            *lutSq_first_zero = i;
        }
        assert(v >= 0);
        assert(v <= 255);
        lutSq[i] = (unsigned char) v;
    }
    if (!set_lutSq_first_zero) {
        set_lutSq_first_zero = true;
        *lutSq_first_zero = sz;
    }

    return lutSq;
}
/*
 * compute the number of hits seen with this scanTransform
 */
int
RasterLookupTable::getNumHits(const smPoint * points, const unsigned numPoints,
        const ScanTransform * XYT0)
{
    int hits = 0;
    double ct = cos(XYT0->theta), st = sin(XYT0->theta);
    // Evaluate each point for a fixed transform
    for (unsigned pidx = 0; pidx < numPoints; pidx++) {

        // project the point
        smPoint p = points[pidx];
        double x = p.x * ct - p.y * st + XYT0->x;
        double y = p.x * st + p.y * ct + XYT0->y;
        int ix, iy;
        worldToTable(x, y, &ix, &iy);
        if (distdata[iy * width + ix] >= hitThresh)
            hits++;
    }
    return hits;
}

/**
 * Take the provided 'points', Transform them by XYT-> 'points' should be in
 * the body frame of the robot. Compute scanmatch for a single transform.
 */

float
RasterLookupTable::getScore(const smPoint * points, const unsigned numPoints,
        const ScanTransform * XYT0)
{
    float score = 0.0;
    double ct = cos(XYT0->theta), st = sin(XYT0->theta);
    // Evaluate each point for a fixed transform
    for (unsigned pidx = 0; pidx < numPoints; pidx++) {
        // project the point
        smPoint p = points[pidx];
        double x = p.x * ct - p.y * st + XYT0->x;
        double y = p.x * st + p.y * ct + XYT0->y;
        int ix, iy;
        worldToTable(x, y, &ix, &iy);
        score += distdata[iy * width + ix];
    }
    return score;
}

/**
 * Take the provided 'points', Transform them by XYT-> 'points' should be in
 * the body frame of the robot. Compute scanmatch for a single transform.
 */

float
RasterLookupTable::getScoreDump(const smPoint * points,
        const unsigned numPoints, const ScanTransform * XYT0, const char * name)
{
    FILE * f = fopen(name, "w");
    float score = 0.0;
    double ct = cos(XYT0->theta), st = sin(XYT0->theta);
    // Evaluate each point for a fixed transform
    for (unsigned pidx = 0; pidx < numPoints; pidx++) {
        // project the point
        smPoint p = points[pidx];
        double x = p.x * ct - p.y * st + XYT0->x;
        double y = p.x * st + p.y * ct + XYT0->y;
        int ix, iy;
        worldToTable(x, y, &ix, &iy);
        score += distdata[iy * width + ix];
        fprintf(f, "blah %d %d %d %f %f %d %f \n", pidx, ix, iy, x, y,
                distdata[iy * width + ix], score);
    }
    fclose(f);
    return score;
}

/**
 * Take the provided 'points', Transform them by XYT0-> 'points' should be in
 * the body frame of the robot. Compute scanmatch results around that
 * coordinate (in both directions, e.g. [-ixrange,+ixrange].) We count the
 * score for each pixel and whether it was a "hit", based on a closeness
 * threshold.
 *
 */
ScanTransform
RasterLookupTable::evaluate2D(const smPoint * points, const unsigned numPoints,
        const ScanTransform * XYT0, const ScanTransform * prior, int ixrange,
        int iyrange, int ixdim, int iydim, float * scores, int * bestScoreIndX,
        int *bestScoreIndY)
{

#ifdef USE_IPP
    int scoresStep = ixdim * sizeof(float);
    int dataStep = width * sizeof(unsigned char);
#endif
    //zero out the scores array!
    memset(scores, 0, ixdim * iydim * sizeof(float));

    double ct = cos(XYT0->theta), st = sin(XYT0->theta);

    // Evaluate each point for a fixed rotation but variable
    // translation
    sm_tictoc("forloop_acum");
    for (unsigned pidx = 0; pidx < numPoints; pidx++) {

        // project the point
        smPoint p = points[pidx];
        double x = p.x * ct - p.y * st + XYT0->x;
        double y = p.x * st + p.y * ct + XYT0->y;

        // (ix0, iy0) are the coordinates in distdata that
        // correspond to scores.x. It's the (nominal) upper-left
        // corner of our search window.
        int ix, iy;
        worldToTable(x, y, &ix, &iy);
        int ix0 = ix - ixrange;
        int iy0 = iy - iyrange;

        // compute the intersection of the box
        // (ix0,iy0)->(ix0+ixdim-1,iy0+iydim-1) and the box
        // (0,0)->(width-1, height-1). This will be our actual
        // search window.
        int bx0 = sm_imax(ix0, 0);
        int by0 = sm_imax(iy0, 0);

        int bx1 = sm_imin(ix0 + ixdim - 1, width - 1);
        int by1 = sm_imin(iy0 + iydim - 1, height - 1);

        if (by1 < by0 || bx1 < bx0)
            continue; //point is way off map!

#ifdef USE_IPP
        //ipp
        IppiSize roiSize = {bx1 - bx0 + 1, by1 - by0 + 1}; //+1 due to <= below
        unsigned char * pSrc = distdata+ width*by0+bx0;
        float * pScoreAcum = scores + (by0 - iy0)*ixdim + (bx0 - ix0); //
        //    FILE * f = fopen("tmpDump.m","w");
        //    fprintf(f,"pidx=%d\n",pidx);
        //    fprintf(f,"width=%d\n",width);
        //    fprintf(f,"height=%d\n",height);
        //
        //    fprintf(f,"ixdim=%d\n",ixdim);
        //    fprintf(f,"iydim=%d\n",iydim);
        //
        //    fprintf(f,"ix0=%d\n",ix0);
        //    fprintf(f,"iy0=%d\n",iy0);
        //    fprintf(f,"bx0=%d\n",bx0);
        //    fprintf(f,"by0=%d\n",by0);
        //    fprintf(f,"bx1=%d\n",bx1);
        //    fprintf(f,"by1=%d\n",by1);
        //
        //
        //
        //    fprintf(f,"pSrc=0x%p\n",pSrc);
        //    fprintf(f,"dataStep=%d\n",dataStep);
        //    fprintf(f,"pScoreAcum=0x%p\n",pScoreAcum);
        //    fprintf(f,"scoresStep=%d\n",scoresStep);
        //    fprintf(f,"roiSize.width=%d\n",roiSize.width);
        //    fprintf(f,"roiSize.height=%d\n",roiSize.height);
        //    fprintf(f,"src = [\n");
        //    for (int i = 0;i<roiSize.height;i++) {
        //      unsigned char * p = pSrc+ i*dataStep;
        //      for (int j=0;j<roiSize.width;j++) {
        //        fprintf(f,"%d ",*p++);
        //      }
        //      fprintf(f,"\n");
        //    }
        //    fprintf(f,"];\n");
        //
        //    fprintf(f,"scoreAcum = [\n");
        //    for (int i = 0;i<roiSize.height;i++) {
        //      float * p = pScoreAcum+ i*ixdim;
        //      for (int j=0;j<roiSize.width;j++) {
        //        fprintf(f,"%f ",*p++);
        //      }
        //      fprintf(f,"\n");
        //    }
        //    fprintf(f,"];\n");
        //    fclose(f);

        ippiAdd_8u32f_C1IR(pSrc, dataStep, pScoreAcum, scoresStep, roiSize);
#else

        for (int iy = by0; iy <= by1; iy++) {

            int sy = iy - iy0; // y coordinate in scores[]

            for (int ix = bx0; ix <= bx1; ix++) {

                int lutval = distdata[iy * width + ix];
                int sx = ix - ix0;

                int sidx = sy * ixdim + sx;

                if (lutval > 0) {
                    scores[sidx] += lutval;
                }
            }
        }
#endif

    }
    sm_tictoc("forloop_acum");

    //factor in wide gaussian prior
    if (prior->score > .1) {
        double x, y;
        for (int sy = 0; sy < iydim; sy++) {
            for (int sx = 0; sx < ixdim; sx++) {
                int sidx = sy * ixdim + sx;
                scoresToWorld(XYT0, ixrange, iyrange, sx, sy, &x, &y);
                //assuming diag covariance
                scores[sidx] *= exp(-1.0 / prior->score * (sm_sq(x - prior->x)
                        + sm_sq(y - prior->y)));
            }
        }
    }

    // Look for the best score
    float bestscore = -1;
    int bestsx = 0, bestsy = 0;
    for (int sy = 0; sy < iydim; sy++) {
        for (int sx = 0; sx < ixdim; sx++) {
            int sidx = sy * ixdim + sx;
            if (scores[sidx] > bestscore) {
                bestscore = scores[sidx];
                bestsx = sx;
                bestsy = sy;
            }
        }
    }
    *bestScoreIndX = bestsx;
    *bestScoreIndY = bestsy;

    int bsidx = bestsy * ixdim + bestsx;

    ScanTransform result;
    scoresToWorld(XYT0, ixrange, iyrange, bestsx, bestsy, &result.x, &result.y);
    result.theta = XYT0->theta;

    result.score = scores[bsidx];

    return result;
}

class score_entry
{
public:
    score_entry(float score_, int it_, int ix_, int iy_) :
        score(score_), it(it_), ix(ix_), iy(iy_)
    {
    }
    float score;
    int it;
    int ix;
    int iy;
};

bool
score_entry_comp(const score_entry & i, const score_entry & j)
{
    return i.score > j.score;
}


static void scaleCovariance(const double cov[9], double scaledCov[9]){
  //scale and square xy-covariance, since that seems to gives a better approximation of the uncertainty...
      double sigma[9];
      memcpy(sigma, cov, 9 * sizeof(double));
      double evals[3] = { 0 };
      double evals_sq[9] =  { 0 };
      double evecs[9] ={ 0 };
      double evecsT[9] ={ 0 };

      memset(scaledCov,0,9*sizeof(double));

      CvMat cv_sigma = cvMat(3, 3, CV_64FC1, sigma);
      CvMat cv_scaledCov = cvMat(3, 3, CV_64FC1, scaledCov);
      CvMat cv_evals = cvMat(3, 1, CV_64FC1, evals);
      CvMat cv_evals_sq = cvMat(3, 3, CV_64FC1, evals_sq);
      CvMat cv_evecs = cvMat(3, 3, CV_64FC1, evecs);
      CvMat cv_evecsT = cvMat(3, 3, CV_64FC1, evecsT);
      cvEigenVV(&cv_sigma, &cv_evecs, &cv_evals);
      if (evals[0] <= 0 || evals[1] <= 0 || evals[2] <= 0)
          fprintf(stderr, "ERROR: pre-scaled cov is not P.S.D!\n");

      cvPow(&cv_evals, &cv_evals, 2); //square the eigan values
      cvScale(&cv_evals, &cv_evals, 50); //multiply by 50

      if (evals[0] > .015) {
          fprintf(stderr, "Eigenvalue 1 is %f, discarding this direction\n",
                  evals[0]);
          evals[0] *= 10; //estimate is total garbage in this direction
      }
      if (evals[1] > .015) {
          fprintf(stderr, "Eigenvalue 1 is %f, discarding this direction\n",
                  evals[1]);
          evals[1] *= 10; //estimate is total garbage in this direction
      }
      CvMat cv_diag;
      cvGetDiag(&cv_evals_sq, &cv_diag);
      cvCopy(&cv_evals, &cv_diag);

      cvTranspose(&cv_evecs, &cv_evecsT);
      cvMatMul(&cv_evecs,&cv_evals_sq,&cv_sigma);
      cvMatMul(&cv_sigma,&cv_evecsT,&cv_scaledCov);

      cvEigenVV(&cv_scaledCov, &cv_evecs, &cv_evals);
      if (evals[0] <= 0 || evals[1] <= 0 || evals[2] <= 0)
          fprintf(stderr, "ERROR: scaled cov is not P.S.D!\n");
}


/** Perform a brute-force search in 3DOFs. * */
ScanTransform
RasterLookupTable::evaluate3D_multiRes(RasterLookupTable * rlt_high_res,
        const smPoint * points, const unsigned numPoints,
        const ScanTransform * prior, double xrange, double yrange,
        double thetarange, double thetastep, int * xSat, int *ySat,
        int * thetaSat)
{
    ScanTransform XYT0;

    //round the prior to a cell
    int ix, iy;
    worldToTable(prior->x, prior->y, &ix, &iy);
    tableToWorld(ix, iy, &XYT0.x, &XYT0.y);

    XYT0.theta = prior->theta;

    ScanTransform bestResult;
    bestResult.score = -1;

    // search range in pixels
    int ixrange = (int) (xrange * pixelsPerMeter) + 1; // always round up,
    // make sure > 0
    int iyrange = (int) (yrange * pixelsPerMeter) + 1; // ditto...

    // allocate our scoring arrays.
    int ixdim = (2 * ixrange + 1);
    int iydim = (2 * iyrange + 1);

    int itdim = (int) (2 * thetarange / thetastep) + 1;

    float * allScores;
    int numScores = itdim * ixdim * iydim;
    allScores = (float *) malloc(numScores * sizeof(float));
    int allScores_step = ixdim * iydim;

    //compute the scores for the full 3D voxel grid at the low resolution
    ScanTransform xyt;
    xyt.x = XYT0.x;
    xyt.y = XYT0.y;
    double dtheta = -thetarange;
    for (int it = 0; it < itdim; it++) {
        xyt.theta = XYT0.theta + dtheta;
        int xInd = 0, yInd = 0;
        ScanTransform thisResult = evaluate2D(points, numPoints, &xyt, prior,
                ixrange, iyrange, ixdim, iydim, allScores + (it
                        * allScores_step), &xInd, &yInd);

        //    //sanity check
        //    float sc = getScore(points, numPoints, &xyt);
        //    float sc2;
        //    ScanTransform tc = evaluate2D(points, numPoints, &xyt, 0, 0, 1, 1, &sc2, &xInd, &yInd);
        //    fprintf(stderr, "e2dscore=%f, scorecheck=%f\n", sc2, sc);
        //
        //    for (int iy = 0; iy < iydim; iy++) {
        //      for (int ix = 0; ix < ixdim; ix++) {
        //
        //        ScanTransform t;
        //        scoresToWorld(&xyt, ixrange, iyrange, ix, iy, &t.x, &t.y);
        //        t.theta = xyt.theta;
        //        float sc = getScore(points, numPoints, &t);
        //        fprintf(stderr, "(%f,%f,%f) (%d,%d,%d) score=%f, scorecheck=%f\n", t.x, t.y,t.theta,ix,iy,it,
        //            allScores[it * allScores_step + iy * ixdim + ix], sc);
        //        assert(fabs(sc-allScores[it * allScores_step + iy * ixdim + ix])<1e-6);
        //      }
        //    }

        dtheta += thetastep;
    }

    //have upper bounds on scores from low res, now find best
    std::vector<score_entry> sorted_scores;
    sorted_scores.reserve(numScores);
    for (int it = 0; it < itdim; it++) {
        for (int iy = 0; iy < iydim; iy++) {
            for (int ix = 0; ix < ixdim; ix++) {
                float score = allScores[it * allScores_step + iy * ixdim + ix];
                sorted_scores.push_back(score_entry(score, it, ix, iy));
            }
        }
    }
    std::sort(sorted_scores.begin(), sorted_scores.end(), score_entry_comp);

    // high_res search range in pixels
    int ixrange_high_res = (int) (metersPerPixel / 2.0
            * rlt_high_res->pixelsPerMeter);
    // make sure > 0
    int iyrange_high_res = (int) (metersPerPixel / 2.0
            * rlt_high_res->pixelsPerMeter);

    // allocate our scoring arrays.
    int ixdim_high_res = (2 * ixrange_high_res + 1);
    int iydim_high_res = (2 * iyrange_high_res + 1);

    float * scores_high_res;
    scores_high_res = (float *) calloc(ixdim_high_res * iydim_high_res,
            sizeof(float));

    //initialize the variables for the covariance computation
    vector<cov_sample_t> cov_samples;

    int i;
    int canBeDone = 0;
    int bestIx = -1, bestIy = -1, bestIt = -1;
    int cov_samples_ind = 0;
    double maxProb = 0;
    for (i = 0; i < numScores; i++) {
        //make sure we do at least the best 6 voxels for covariance and robustness...
        if (sorted_scores[i].score < bestResult.score && i > 6)
            canBeDone = 1; //scores from low_res are an upper bound, so we're done...
        //make sure we have checked the voxels around the best
        if (canBeDone && (abs(sorted_scores[i].ix - bestIx) + abs(
                sorted_scores[i].iy - bestIy) + abs(sorted_scores[i].it
                - bestIt)) != 1)
            continue;

        ScanTransform t;
        //convert from the scores index back to world...
        scoresToWorld(&XYT0, ixrange, iyrange, sorted_scores[i].ix,
                sorted_scores[i].iy, &t.x, &t.y);
        t.theta = XYT0.theta - thetarange + sorted_scores[i].it * thetastep;

        int xInd, yInd;
        ScanTransform result = rlt_high_res->evaluate2D(points, numPoints, &t,
                prior, ixrange_high_res, iyrange_high_res, ixdim_high_res,
                iydim_high_res, scores_high_res, &xInd, &yInd);

        //add these "samples" to the covariance computation
        assert(cov_samples_ind== cov_samples.size());
        cov_samples.resize(cov_samples.size() + ixdim_high_res * iydim_high_res);
        double wx, wy, p, score;
        for (int sy = 0; sy < iydim_high_res; sy++) {
            for (int sx = 0; sx < ixdim_high_res; sx++) {
                score = scores_high_res[sy * ixdim_high_res + sx];
                p = score / ((double) numPoints * 255.0);
                p = exp(p) / 2.71828183;
                cov_samples[cov_samples_ind].p = p;
                maxProb = fmax(maxProb, p);

                rlt_high_res->scoresToWorld(&t, ixrange_high_res,
                        iyrange_high_res, sx, sy, &wx, &wy);
                cov_samples[cov_samples_ind].x[0] = wx;
                cov_samples[cov_samples_ind].x[1] = wy;
                cov_samples[cov_samples_ind].x[2] = t.theta;
                cov_samples_ind++;
            }
        }
        if (result.score > bestResult.score) {
            bestResult = result;
            bestIx = sorted_scores[i].ix;
            bestIy = sorted_scores[i].iy;
            bestIt = sorted_scores[i].it;
        }

        //    fprintf(stderr, "low_res_score=%f, hi_res_score=%f, it=%d iy=%d ix=%d\n", sorted_scores[i].score, bestResult.score,
        //        sorted_scores[i].it, sorted_scores[i].iy, sorted_scores[i].ix);
        if (result.score > sorted_scores[i].score) { //it should be an upper bound!
            //    TODO:Not sure why this is happening sometimes!
            //            fprintf(
            //                    stderr,
            //                    "WARNING! score num %d (%d,%d,%d) hi_res score of %f higher than low resolution of %f\n",
            //                    i, sorted_scores[i].ix, sorted_scores[i].iy,
            //                    sorted_scores[i].it, result.score, sorted_scores[i].score);
            //      dumpTable("low_res");
            //      rlt_high_res->dumpTable("hi_res");
            //      ScanTransform tc;
            //      scoresToWorld(&XYT0, ixrange, iyrange, sorted_scores[i].ix, sorted_scores[i].iy, &tc.x, &tc.y);
            //      tc.theta = XYT0.theta - thetarange + sorted_scores[i].it * thetastep;
            //      float lrs = getScoreDump(points, numPoints, &tc, "low_res_dump");
            //      float lrs2 = getScoreDump(points, numPoints, &result, "low_res_dump2");
            //      float lrs3 = getScoreDump(points, numPoints, &t, "low_res_dump3");
            //
            //      int ix, iy;
            //      int ix2, iy2;
            //      int ix3, iy3;
            //      worldToTable(tc.x, tc.y, &ix, &iy);
            //      worldToTable(result.x, result.y, &ix2, &iy2);
            //      worldToTable(t.x, t.y, &ix3, &iy3);
            //      fprintf(stderr, "t=(%f,%f),tc=(%f,%f),r=(%f,%f) t_i=(%d,%d),tc_i=(%d,%d), r_i=(%d,%d)\n", t.x, t.y, tc.x, tc.y,
            //          result.x, result.y, ix3, iy3, ix, iy, ix2, iy2);
            //
            //      float hrs = rlt_high_res->getScoreDump(points, numPoints, &tc, "high_res_dump");
            //      float hrs2 = rlt_high_res->getScoreDump(points, numPoints, &result, "high_res_dump2");
            //      float hrs3 = rlt_high_res->getScoreDump(points, numPoints, &t, "high_res_dump3");
            //      fprintf(stderr, "dump done, lrs=%f,lrs2=%f,lrs3=%f  hrs=%f,hrs2=%f\n", lrs, lrs2, lrs3, hrs, hrs2, hrs3);
            //      exit(1);
        }
    }

    //  fprintf(stderr, "only checked %d of %d voxels at high_res\n",i,ixdim*iydim*itdim);
    //get the number of hits for this transform in the high res table
    bestResult.hits = rlt_high_res->getNumHits(points, numPoints, &bestResult);

    //actually compute the covariance
    double K[9] =
        { 0 };
    double u[3] =
        { 0 };
    double s = 0;

    double x[3];
    double Ktmp[9];
    double prob;
    //    FILE * f = fopen("vardump.log","w");
    for (unsigned int i = 0; i < cov_samples.size(); i++) {
        memcpy(x, cov_samples[i].x, 3 * sizeof(double));
        prob = cov_samples[i].p;
        //      fprintf(f,"p %f %f %f %f\n",x[0],x[1],x[2],prob);
        if (prob / maxProb < VAR_COMP_THRESH)
            continue;

        //        K = K+ x*x'*p;
        sm_vector_vector_outer_product_3d(x, x, Ktmp);
        sm_vector_scale_nd(Ktmp, 9, prob);
        sm_vector_add_nd(K, Ktmp, 9, K);
        //        u = u+x*p;
        sm_vector_scale_3d(x, prob);
        sm_vector_add_3d(u, x, u);
        //        s = s+p;
        s += prob;
    }
    //    fclose(f);

    //sig_hat = 1/s*K-1/s^2*u*u'
    sm_vector_scale_nd(K, 9, 1.0 / s);

    double u_outer[9];
    sm_vector_vector_outer_product_3d(u, u, u_outer);
    sm_vector_scale_nd(u_outer, 9, 1.0 / (s * s));

    double cov[9];
    sm_vector_sub_nd(K, u_outer, 9, cov);
    cov[8] = fmax(cov[8], 1e-3); //often times the top 85% will all be the same theta

#ifdef SCALE_COVARIANCE
    scaleCovariance(cov,bestResult.sigma);
#else
      memcpy(bestResult.sigma, cov, 9 * sizeof(double));
#endif

#if DRAW_COST_SURFACE
    //draw score cloud with lcmgl
    static lcm_t * lcm = lcm_create(NULL);
    static bot_lcmgl_t *lcmgl = bot_lcmgl_init(lcm, "sm_scores");
    bot_lcmgl_push_attrib(lcmgl,GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
    bot_lcmgl_enable(lcmgl,GL_DEPTH_TEST);
    bot_lcmgl_depth_func(lcmgl,GL_LESS);

    bot_lcmgl_point_size(lcmgl, 6);
    bot_lcmgl_begin(lcmgl, GL_POINTS);
    for (unsigned int i = 0; i < cov_samples.size(); i++) {
        if (fabs(sm_angle_subtract(cov_samples[i].x[2],bestResult.theta))>.01 ||cov_samples[i].p/maxProb <VAR_COMP_THRESH)
        continue;
        float * color3fv = sm_color_util_jet(cov_samples[i].p/maxProb);
        bot_lcmgl_color3f(lcmgl, color3fv[0], color3fv[1], color3fv[2]);
        //      double x = cov_samples[i].x[0]-bestResult.x;
        //      double y = cov_samples[i].x[1]-bestResult.y;
        double x = cov_samples[i].x[0];
        double y = cov_samples[i].x[1];
        bot_lcmgl_vertex3f(lcmgl, x,y ,cov_samples[i].p/maxProb);
    }

    for (unsigned int i = 0; i < cov_samples.size(); i++) {
        if (fabs(sm_angle_subtract(cov_samples[i].x[2],bestResult.theta))>.01)
        continue;
        float * color3fv = sm_color_util_jet(cov_samples[i].p/maxProb);
        bot_lcmgl_color3f(lcmgl, color3fv[0], color3fv[1], color3fv[2]);
        //      double x = cov_samples[i].x[0]-bestResult.x;
        //      double y = cov_samples[i].x[1]-bestResult.y;
        double x = cov_samples[i].x[0];
        double y = cov_samples[i].x[1];
        bot_lcmgl_vertex3f(lcmgl, x+1,y ,cov_samples[i].p/maxProb);
    }

    bot_lcmgl_end(lcmgl);
    bot_lcmgl_pop_attrib(lcmgl);
    bot_lcmgl_switch_buffer(lcmgl);

#endif

    //set the sat variables to zero, cuz we're ignoring it for now...
    if (xSat != NULL)
        *xSat = 0;
    if (ySat != NULL)
        *ySat = 0;
    if (thetaSat != NULL)
        *thetaSat = 0;

    //cleanup
    free(scores_high_res);
    free(allScores);

    return bestResult;
}

/** Perform a brute-force search in 3DOFs. * */
ScanTransform
RasterLookupTable::evaluate3D(const smPoint * points, const unsigned numPoints,
        const ScanTransform * prior, double xrange, double yrange,
        double thetarange, double thetastep, int * xSat, int *ySat,
        int * thetaSat)
{
    ScanTransform xyt = *prior;
    ScanTransform bestResult;
    bestResult.score = -1;

    // search range in pixels
    int ixrange = (int) (xrange * pixelsPerMeter) + 1; // always round up,
    // make sure > 0
    int iyrange = (int) (yrange * pixelsPerMeter) + 1; // ditto...

    // allocate our scoring arrays.
    int ixdim = (2 * ixrange + 1);
    int iydim = (2 * iyrange + 1);

    int itdim = (int) (2 * thetarange / thetastep) + 1;

    float ** allScores;
    allScores = (float **) calloc(itdim, sizeof(float *));
    float * bestThetaScores;
    bestThetaScores = (float *) calloc(itdim, sizeof(float));

    for (int i = 0; i < itdim; i++) {
        allScores[i] = (float *) calloc(ixdim * iydim, sizeof(float));
    }

    int bestThetaInd = 0, bestXInd = 0, bestYInd = 0;
    double dtheta = -thetarange;
    //TODO: this could/should be threaded...
    for (int it = 0; it < itdim; it++) {
        xyt.theta = prior->theta + dtheta;

        int xInd = 0, yInd = 0;
        ScanTransform thisResult = evaluate2D(points, numPoints, &xyt, prior,
                ixrange, iyrange, ixdim, iydim, allScores[it], &xInd, &yInd);
        bestThetaScores[it] = thisResult.score;
        if (bestResult.score == -1 || thisResult.score > bestResult.score) {
            bestResult = thisResult;
            bestThetaInd = it;
            bestXInd = xInd;
            bestYInd = yInd;
        }

        dtheta += thetastep;
    }

    sm_tictoc("compute_Variance");
    //compute covariance by fitting multivariate gaussian directly
    //initialize the variables for the covariance computation
    double K[9] =
        { 0 };
    double u[3] =
        { 0 };
    double s = 0;

    double x[3];
    double Ktmp[9];
    double prob;
    double score;
    double maxProb = exp(bestResult.score / ((double) numPoints * 255.0))
            / 2.71828183;
    //    FILE * f = fopen("vardump.log","w");
    for (int st = 0; st < itdim; st++) {
        for (int sy = 0; sy < iydim; sy++) {
            for (int sx = 0; sx < ixdim; sx++) {
                score = allScores[st][sy * ixdim + sx];
                prob = score / ((double) numPoints * 255.0);
                prob = exp(prob) / 2.71828183;
                if (prob / maxProb < VAR_COMP_THRESH)
                    continue;

                scoresToWorld(prior, ixrange, iyrange, sx, sy, &x[0], &x[1]);
                x[2] = prior->theta - thetarange + st * thetastep;
                //                fprintf(f,"p %f %f %f %f\n",x[0],x[1],x[2],prob);

                //        K = K+ x*x'*p;
                sm_vector_vector_outer_product_3d(x, x, Ktmp);
                sm_vector_scale_nd(Ktmp, 9, prob);
                sm_vector_add_nd(K, Ktmp, 9, K);
                //        u = u+x*p;
                sm_vector_scale_3d(x, prob);
                sm_vector_add_3d(u, x, u);
                //        s = s+p;
                s += prob;
            }
        }
    }
    //    fclose(f);

    //sig_hat = 1/s*K-1/s^2*u*u'
    sm_vector_scale_nd(K, 9, 1.0 / s);

    double u_outer[9];
    sm_vector_vector_outer_product_3d(u, u, u_outer);
    sm_vector_scale_nd(u_outer, 9, 1.0 / (s * s));

    double cov[9];
    sm_vector_sub_nd(K, u_outer, 9, cov);
    cov[8] = fmax(cov[8], 1e-3); //often times the top 85% will all be the same theta

#ifdef SCALE_COVARIANCE
    scaleCovariance(cov,bestResult.sigma);
#else
      memcpy(bestResult.sigma, cov, 9 * sizeof(double));
#endif


    sm_tictoc("compute_Variance");

#if DRAW_COST_SURFACE
    //draw score cloud with lcmgl
    static lcm_t * lcm = lcm_create(NULL);
    static bot_lcmgl_t *lcmgl = bot_lcmgl_init(lcm, "sm_scores2");
    bot_lcmgl_push_attrib(lcmgl,GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
    bot_lcmgl_enable(lcmgl,GL_DEPTH_TEST);
    bot_lcmgl_depth_func(lcmgl,GL_LESS);

    bot_lcmgl_point_size(lcmgl, 6);
    bot_lcmgl_begin(lcmgl, GL_POINTS);

    for (int st = 0; st < itdim; st++) {
        for (int sy = 0; sy < iydim; sy++) {
            for (int sx = 0; sx < ixdim; sx++) {
                score = allScores[st][sy * ixdim + sx];
                prob = score / ((double) numPoints * 255.0);
                prob = exp(prob)/2.71828183;
                scoresToWorld(prior, ixrange, iyrange, sx, sy, &x[0], &x[1]);
                x[2] = prior->theta - thetarange + st * thetastep;
                if (fabs(sm_angle_subtract(x[2],bestResult.theta))>.01)
                continue;
                float * color3fv = sm_color_util_jet(prob/maxProb);
                bot_lcmgl_color3f(lcmgl, color3fv[0], color3fv[1], color3fv[2]);
                bot_lcmgl_vertex3f(lcmgl, x[0]+1,x[1] ,prob/maxProb);

            }
        }
    }

    bot_lcmgl_end(lcmgl);
    bot_lcmgl_pop_attrib(lcmgl);
    bot_lcmgl_switch_buffer(lcmgl);

#endif

    //      sm_tictoc("compute_Variance");
    //
    //      //compute cost map statistics so we can extract a covariance matrix
    //
    //      //xy covariance
    //      unsigned char * costMap = (unsigned char *) calloc(ixdim * iydim, sizeof(unsigned char));
    //      for (int i = 0; i < iydim; i++) {
    //        for (int j = 0; j < ixdim; j++) {
    //          if (allScores[bestThetaInd][i * ixdim + j] > .97 * bestResult.score)
    //            costMap[i * ixdim + j] = (unsigned char) ((255 * allScores[bestThetaInd][i * ixdim + j]) / bestResult.score); //normalize this to be 0-255
    //        }
    //      }
    //      CvMoments moments;
    //      CvMat cv_costMap = cvMat(iydim, ixdim, CV_8UC1, costMap);
    //      cvMoments(&cv_costMap, &moments, 0);
    //      double u00 = cvGetCentralMoment(&moments, 0, 0);
    //      double u20 = cvGetCentralMoment(&moments, 2, 0);
    //      double u02 = cvGetCentralMoment(&moments, 0, 2);
    //      double u11 = cvGetCentralMoment(&moments, 1, 1);
    //
    //      bestResult.sigma[0] = u20 / u00 * .001;
    //      bestResult.sigma[1] = u11 / u00* .001;
    //      bestResult.sigma[3] = u11 / u00* .001;
    //      bestResult.sigma[4] = u02 / u00* .001;
    //
    //      //  printf("moments are :\n");
    //      //  printf("u00=%f, u20=%f, u02=%f, u11=%f \n", u00, u20, u02, u11);
    //      //  printf("that would indicate the cov matrix is:\n");
    //      //  printf("[ %f %f\n",u20/u00,u11/u00);
    //      //  printf("  %f %f]\n",u11/u00,u02/u00);
    //
    //      //  CvMat * cv_costMap_big = cvCreateMat(iydim*10, ixdim*10, CV_8UC1);
    //      //  cvResize(&cv_costMap,cv_costMap_big);
    //      //  cvFlip(cv_costMap_big,cv_costMap_big);
    //      //  cvNamedWindow("varComp",CV_WINDOW_AUTOSIZE);
    //      //  cvShowImage("varComp", cv_costMap_big);
    //
    //      //yaw variance
    //      int l = sm_imax(bestThetaInd - 3, 0);
    //      int r = sm_imin(bestThetaInd + 3, itdim - 1);
    //      float slope = (bestThetaScores[bestThetaInd] - bestThetaScores[l] + bestThetaScores[bestThetaInd]
    //          - bestThetaScores[r]) * 1.0 / (r - l);
    //      bestResult.sigma[8] = 100.0 / slope* .0001;
    //
    //      //  unsigned char * tmap = (unsigned char *) calloc(itdim, sizeof(unsigned char));
    //      //  for (int j = 0; j < itdim; j++) {
    //      //    if (bestThetaScores[j] > .97 * bestResult.score)
    //      //      tmap[j] = (255 * bestThetaScores[j]) / bestResult.score; //normalize this to be 0-255
    //      //  }
    //      //  CvMoments thetamoments;
    //      //  CvMat cv_tMap = cvMat(itdim, 1, CV_8UC1, tmap);
    //      //  cvMoments(&cv_tMap, &thetamoments, 0);
    //      //  double u00t = cvGetCentralMoment(&thetamoments, 0, 0);
    //      //  double u02t = cvGetCentralMoment(&thetamoments, 0, 2);
    //      //
    //      //  //  printf("theta moments are :\n");
    //      //  //  printf("u00t=%f, u02t=%f\n", u00t, u02t);
    //      //  //  printf("that would indicate the Variance is:\n");
    //      //  //  printf(" [%f]\n",u02t / u00t);
    //      //
    //      //  bestResult.sigmaT = u02t / u00t;
    //
    //      sm_tictoc("compute_Variance");

    //check whether we hit the edge of our ranges...

    if (xSat != NULL) {
      *xSat = 0;
      if (bestXInd == 0)
        *xSat = -1;
      else if (bestXInd == ixdim - 1)
        *xSat = 1;
    }

    if (ySat != NULL) {
      *ySat = 0;
      if (bestYInd == 0)
        *ySat = -1;
      else if (bestYInd == iydim - 1)
        *ySat = 1;
    }

    if (thetaSat != NULL) {
      *thetaSat = 0;
      if (bestThetaInd == 0)
        *thetaSat = -1;
      else if (bestThetaInd == itdim - 1)
        *thetaSat = 1;
    }

    //  FILE * f = fopen("scoreDump.m", "w");
    //  fprintf(f, "bestThetaInd =%d\n", bestThetaInd + 1);
    //  fprintf(f, "bestXInd =%d\n", bestXInd + 1);
    //  fprintf(f, "bestYInd =%d\n", bestYInd + 1);
    //  for (int t = 0; t < itdim; t++) {
    //    fprintf(f, "scores{%d} = [\n", t + 1);
    //    for (int i = 0; i < iydim; i++) {
    //      for (int j = 0; j < ixdim; j++) {
    //        fprintf(f, "%f,", allScores[t][i * ixdim + j]);
    //      }
    //      fprintf(f, ";\n");
    //    }
    //    fprintf(f, "];\n");
    //  }
    //
    //  fprintf(f, "origpoints=[\n");
    //  double ct = cos(XYT0->theta), st = sin(XYT0->theta);
    //  // Evaluate each point for a fixed transform
    //  for (unsigned pidx = 0; pidx < numPoints; pidx++) {
    //    // project the point
    //    smPoint p = points[pidx];
    //    double x = p.x * ct - p.y * st + XYT0->x;
    //    double y = p.x * st + p.y * ct + XYT0->y;
    //    int ix, iy;
    //    worldToTable(x, y, &ix, &iy);
    //    fprintf(f, "%d %d\n", ix, iy);
    //  }
    //  fprintf(f, "];\n");
    //
    //  fprintf(f, "bestpoints=[\n");
    //  // Evaluate each point for a fixed transform
    //  for (unsigned pidx = 0; pidx < numPoints; pidx++) {
    //    // project the point
    //    smPoint p = points[pidx];
    //    double x = p.x * ct - p.y * st + bestResult.x;
    //    double y = p.x * st + p.y * ct + bestResult.y;
    //    int ix, iy;
    //    worldToTable(x, y, &ix, &iy);
    //    fprintf(f, "%d %d\n", ix, iy);
    //  }
    //  fprintf(f, "];\n");
    //
    //  fclose(f);
    //  dumpTable("table");
    //

    //  //sanity check:
    //  float priorScore = getScore(points,numPoints,XYT0);
    //  int priorHits = getNumHits(points, numPoints, XYT0);
    //  float bestScore = getScore(points,numPoints,&bestResult);
    //  int bestHits = getNumHits(points, numPoints, &bestResult);
    //  assert(bestScore>=priorScore);


    //get the number of hits for this transform
    bestResult.hits = getNumHits(points, numPoints, &bestResult);

    //cleanup
    for (int i = 0; i < itdim; i++) {
        free(allScores[i]);
    }
    free(allScores);
    free(bestThetaScores);
    //  free(costMap);
    //  free(tmap);

    //  printf("br.x=%f \t br.y=%f \t br.t=%f\t br.score=%f\n", bestResult.x, bestResult.y, bestResult.theta, bestResult.score);
    return bestResult;
}

void
RasterLookupTable::dumpTable(const char * varName)
{
    char buf[256];
    sprintf(buf, "%s.bmp", varName);
    cvSaveImage(buf, &distim);
    //  char buf[256];
    //  sprintf(buf, "%s.m", varName);
    //  FILE * f = fopen(buf, "w");
    //  fprintf(f, "%s_data = [\n", varName);
    //  for (int iy = 0; iy < height; iy++) {
    //    for (int ix = 0; ix < width; ix++) {
    //      int idx = iy * width + ix;
    //      fprintf(f, "%d, ", distdata[idx]);
    //    }
    //    fprintf(f, ";\n ");
    //  }
    //
    //  fprintf(f, "];\n");
    //  fclose(f);
}

CvMat *
RasterLookupTable::drawTable()
{

    double AR = (double) height / (double) width;
    int drawHeight, drawWidth;
    if (AR <= 1) {
        drawWidth = maxDrawDim;
        drawHeight = (int) (((double) maxDrawDim) * AR);
    } else {
        drawWidth = (int) (((double) maxDrawDim) * 1 / AR);
        drawHeight = maxDrawDim;
    }
    sm_allocateOrResizeMat(&drawim, drawHeight, drawWidth, CV_8UC1);
    cvResize(&distim, drawim);
    sm_allocateOrResizeMat(&drawim_color, drawHeight, drawWidth, CV_8UC3);
    cvCvtColor(drawim, drawim_color, CV_GRAY2BGR);
    cvSubRS(drawim_color, CV_RGB(255, 255, 255), drawim_color);

    return drawim_color;
}

void
RasterLookupTable::drawRobot(CvMat * drawImg, ScanTransform transf,
        CvScalar color)
{

    //  double displayRatio = (double) maxDrawDim / (double) width;
    CvPoint p1, p2;
    worldToDisplay(transf.x, transf.y, &p1.x, &p1.y);

    cvCircle(drawImg, p1, 3, color, 2);

    double hx, hy;
    hx = transf.x + .3 * cos(transf.theta);
    hy = transf.y + .3 * sin(transf.theta);
    worldToDisplay(hx, hy, &p2.x, &p2.y);
    cvLine(drawImg, p1, p2, color, 2);

}

void
RasterLookupTable::drawScan(CvMat * drawImg, smPoint * points,
        unsigned numPoints, ScanTransform transf, CvScalar color)
{

    static smPoint * pp = NULL;
    pp = (smPoint *) realloc((void *) pp, numPoints * sizeof(smPoint));
    sm_transformPoints(&transf, points, numPoints, pp);

    CvPoint p;
    for (unsigned i = 0; i < numPoints; i++) {
        worldToDisplay(pp[i].x, pp[i].y, &p.x, &p.y);
        cvCircle(drawImg, p, 2, color, -1);
    }
    drawRobot(drawImg, transf, color);

    worldToDisplay(transf.x, transf.y, &p.x, &p.y);

    sm_drawCov(drawImg, p, transf.sigma[0], transf.sigma[1], transf.sigma[4],
            transf.sigma[8], 500, 250, 2, CV_RGB(0,255,0), CV_RGB(255,0,0));

    //this doesn't actually work
    //  extern carmen3d_state_correction_message state_correction;
    //  ScanTransform correctedTransf = *transf;
    //  correctedTransf.x = transf.x + state_correction.correction.x;
    //  correctedTransf.y = transf.y + state_correction.correction.y;
    //  correctedTransf.theta = transf.theta + state_correction.correction.yaw;
    //  drawRobot(drawImg,&correctedTransf,CV_RGB(0,0,255));

}
