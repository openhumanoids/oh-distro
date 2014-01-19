#include "ScanMatcher.hpp"
#include <stdio.h>
#include <unistd.h>
#include "RasterLookupTable.hpp"
#include "Contour.hpp"
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "ScanMatchingOpencvUtils.hpp"
#include "ScanMatchingUtils.hpp"
#include <signal.h>

using namespace std;
using namespace scanmatch;

ScanMatcher::ScanMatcher(double metersPerPixel_, double thetaResolution_,
        int useMultires_, bool useThreads_, bool verbose_) :

    metersPerPixel(metersPerPixel_), thetaResolution(thetaResolution_),
            useMultiRes(useMultires_), verbose(verbose_), rlt(NULL), rltTmp(
                    NULL), rlt_low_res(NULL), rltTmp_low_res(NULL), fullGui(
                    NULL), useThreads(useThreads_), killThread(1)
{

    downsampleFactor = (1 << useMultiRes);

    //line drawing kernel stuff
    double sigma = .0675 / metersPerPixel; //sigma is in pixels
    //  sigma = 21;
    kernelSize = sm_createGaussianKernels(sigma, &squareKernel, &lineKernel);
    diagLineKernel = (unsigned char *) calloc(kernelSize * 2,
            sizeof(unsigned char)); //space for tmp kernel (double the line kernel size

    //set reasonable defaults for successive matching
    memset(&currentPose, 0, sizeof(currentPose));
    memset(&prevPose, 0, sizeof(prevPose));
    maxNumScans = 30;
    initialSearchRangeXY = .13;
    maxSearchRangeXY = .25;
    initialSearchRangeTheta = .1;
    maxSearchRangeTheta = .2;
    matchingMode = SM_GRID_COORD;
    addScanHitThresh = .8;

    //threading stuff
    if (useThreads) {
        //remember to make sure that sm_tictoc gets initialized
        killThread = 0;

        /* Initialize mutex and condition variable objects */
        pthread_mutex_init(&scans_mutex, NULL);
        pthread_mutex_init(&rlt_mutex, NULL);
        pthread_mutex_init(&toBeProcessed_mutex, NULL);
        pthread_cond_init(&toBeProcessed_cv, NULL);

        //create rebuilder thread
        pthread_create(&rebuilder, 0, (void *
        (*)(void *)) rebuildThreadFunc, (void *) this);
    }
}

void
ScanMatcher::initSuccessiveMatchingParams(unsigned int maxNumScans_,
        double initialSearchRangeXY_, double maxSearchRangeXY_,
        double initialSearchRangeTheta_, double maxSearchRangeTheta_,
        sm_incremental_matching_modes_t matchingMode_, double addScanHitThresh_,
        bool stationaryMotionModel_,double motionModelPriorWeight_,
        ScanTransform * startPose)
{

    maxNumScans = maxNumScans_;
    initialSearchRangeXY = initialSearchRangeXY_;
    maxSearchRangeXY = maxSearchRangeXY_;
    initialSearchRangeTheta = initialSearchRangeTheta_;
    maxSearchRangeTheta = maxSearchRangeTheta_;
    matchingMode = matchingMode_ ;
    addScanHitThresh = addScanHitThresh_;
    stationaryMotionModel =stationaryMotionModel_;
    motionModelPriorWeight =motionModelPriorWeight_;

    if (maxSearchRangeXY < initialSearchRangeXY) {
        fprintf(
                stderr,
                "WARNING: maxSearchRangeXY of %f is less than initialSearchRangeXY of %f, setting max=initial\n",
                maxSearchRangeXY, initialSearchRangeXY);
        maxSearchRangeXY = initialSearchRangeXY;
    }
    if (maxSearchRangeTheta < initialSearchRangeTheta) {
        fprintf(
                stderr,
                "WARNING: maxSearchRangeTheta of %f is less than initialSearchRangeTheta of %f, setting max=initial\n",
                maxSearchRangeTheta, initialSearchRangeTheta);
        maxSearchRangeTheta = initialSearchRangeTheta;
    }
    if (startPose!=NULL){
      memcpy(&currentPose, startPose, sizeof(currentPose));
      memcpy(&prevPose, startPose, sizeof(prevPose));
    }
    else{
      memset(&currentPose, 0, sizeof(currentPose));
      memset(&prevPose, 0, sizeof(prevPose));
    }

}

ScanMatcher::~ScanMatcher()
{
    clearScans(true);
    if (useThreads) {
        //aquire all locks so we can destroy them
        pthread_mutex_lock(&scans_mutex);
        pthread_mutex_lock(&rlt_mutex);
        //kill the rebuilder thread
        while (killThread != -1) {
            killThread = 1;
            pthread_cond_broadcast(&toBeProcessed_cv);
            usleep(10000);

        }

        pthread_mutex_lock(&toBeProcessed_mutex);
        // destroy mutex and condition variable objects
        pthread_mutex_destroy(&scans_mutex);
        pthread_mutex_destroy(&rlt_mutex);
        pthread_mutex_destroy(&toBeProcessed_mutex);
        pthread_cond_destroy(&toBeProcessed_cv);

    }

    cvReleaseMat(&fullGui);

}

void
ScanMatcher::clearScans(bool deleteScans)
{
    if (useThreads) {
        pthread_mutex_lock(&scans_mutex);
        pthread_mutex_lock(&rlt_mutex);
        pthread_mutex_lock(&toBeProcessed_mutex);
    }
    if (deleteScans) {
        while (scans.size() > 1) {
            delete scans.front();
            scans.pop_front();
        }
        while (scansToBeProcessed.size() > 1) {
            delete scansToBeProcessed.front();
            scansToBeProcessed.pop_front();
        }
    } else {
        scans.clear();
        scansToBeProcessed.clear();
    }
    //rlt is no longer valid
    if (rlt != NULL)
        delete rlt;
    if (rltTmp != NULL)
        delete rltTmp;

    if (rlt_low_res != NULL)
        delete rlt_low_res;
    if (rltTmp_low_res != NULL)
        delete rltTmp_low_res;

    rlt = NULL;
    rltTmp = NULL;
    rlt_low_res = NULL;
    rltTmp_low_res = NULL;

    if (useThreads) {
        pthread_mutex_unlock(&scans_mutex);
        pthread_mutex_unlock(&rlt_mutex);
        pthread_mutex_unlock(&toBeProcessed_mutex);
    }
}

void *
ScanMatcher::rebuildThreadFunc(ScanMatcher*parent)
{
    if (!parent->useThreads) {
        fprintf(stderr,
                "ERROR, threading is disabled! this shouldn't be being called\n");
        exit(1);
    }

    while (true) {
        pthread_mutex_lock(&parent->toBeProcessed_mutex);
        while (parent->scansToBeProcessed.empty()) {
            pthread_cond_wait(&parent->toBeProcessed_cv,
                    &parent->toBeProcessed_mutex);
            if (parent->killThread) {
                pthread_mutex_unlock(&parent->toBeProcessed_mutex);
                parent->killThread = -1;
                fprintf(stderr, "rebuild thread exiting\n");
                return 0;
            }
        }
        std::list<Scan *> scansBeingProcessed;
        //there are scans to be processed... lets take care of business
        //first swap out the lists so we can let go of the lock...
        scansBeingProcessed = parent->scansToBeProcessed;
        parent->scansToBeProcessed.clear();
        pthread_mutex_unlock(&parent->toBeProcessed_mutex);

        if (scansBeingProcessed.size() > 1) {
            if (parent->verbose) {
                fprintf(
                        stderr,
                        "there are %zu scans to be processed, discarding all but most recent\n",
                        scansBeingProcessed.size());
            }
            while (scansBeingProcessed.size() > 1) {
                delete scansBeingProcessed.front();
                scansBeingProcessed.pop_front();
            }
        }

        if (scansBeingProcessed.size() > parent->maxNumScans) {
            fprintf(
                    stderr,
                    "Scan Matcher is way behind!, there are %zu scans to be added\n",
                    scansBeingProcessed.size());
            while (scansBeingProcessed.size() > parent->maxNumScans) {
                delete scansBeingProcessed.front();
                scansBeingProcessed.pop_front();
            }
        }

        pthread_mutex_lock(&parent->scans_mutex);
        //make space for new scans
        while (parent->numScans() + scansBeingProcessed.size()
                > parent->maxNumScans) {
            delete parent->scans.front();
            parent->scans.pop_front();
        }

        list<Scan *>::iterator it;
        it = parent->scans.end(); //save the current end for drawing the new scans
        while (!scansBeingProcessed.empty()) {
            Scan * s = scansBeingProcessed.front();
            sm_tictoc("findContours");
            ContourExtractor * cextractor = new ContourExtractor(s->laser_type);
            cextractor->findContours(s->ppoints, s->numPoints, s->contours);
            delete cextractor; //TODO: probably a cleaner way
            sm_tictoc("findContours");
            //            s->drawContours(1000,100);

            parent->scans.push_back(s);
            scansBeingProcessed.pop_front();
        }

        //    //draw in the new scans onto the old rlt while we're waiting for the full rebuild.
        //    pthread_mutex_lock(&parent->rlt_mutex);
        //    sm_tictoc("tempDraw");
        //    while (++it != parent->scans.end()) {
        //      Scan * s = *it;
        //      //draw the new scans on the old raster table using the blurred lines for now.
        //      parent->drawBlurredScan(&parent->rlt, s);
        //    }
        //    sm_tictoc("tempDraw");
        //    pthread_mutex_unlock(&parent->rlt_mutex);


        //    sm_tictoc("rebuildRaster_olson");
        //    parent->rebuildRaster_olson(&parent->rltTmp); //rebuild the raster in the tmp
        //    sm_tictoc("rebuildRaster_olson");

        //    sm_tictoc("rebuildRaster_blur");
        //    parent->rebuildRaster_blur(&parent->rltTmp); //rebuild the raster in the tmp
        //    sm_tictoc("rebuildRaster_blur");

        //        //this is works really well now :-)
        //        sm_tictoc("rebuildRaster_blurLine");
        //        parent->rebuildRaster_blurLine(&parent->rltTmp); //rebuild the raster in the tmp
        //        sm_tictoc("rebuildRaster_blurLine");

        sm_tictoc("rebuildRaster");
        parent->rebuildRaster(&parent->rltTmp); //rebuild the raster in the tmp
        sm_tictoc("rebuildRaster");

        if (parent->useMultiRes > 0) {
            sm_tictoc("rebuild_lowRes");
            parent->rltTmp_low_res = new RasterLookupTable(parent->rltTmp,
                    parent->downsampleFactor);
            sm_tictoc("rebuild_lowRes");
        }

        //swap rltTmp with rlt to put it in use
        pthread_mutex_lock(&parent->rlt_mutex);
        if (parent->cancelAdd) {
            if (parent->verbose)
                fprintf(stderr, "Scan add was canceled!\n");
            delete parent->rltTmp;
            parent->rltTmp = NULL;
            if (parent->useMultiRes > 0) {
                delete parent->rltTmp_low_res;
                parent->rltTmp_low_res = NULL;
            }
        } else {
            delete parent->rlt;
            parent->rlt = parent->rltTmp;
            parent->rltTmp = NULL;

            if (parent->useMultiRes > 0) {
                delete parent->rlt_low_res;
                parent->rlt_low_res = parent->rltTmp_low_res;
                parent->rltTmp_low_res = NULL;
            }
            if (parent->verbose)
                fprintf(stderr, "rlt swapped!\n");
        }
        pthread_mutex_unlock(&parent->rlt_mutex);
        pthread_mutex_unlock(&parent->scans_mutex);

        //    //clear out scans that got added in the interim
        //    pthread_mutex_lock(&parent->toBeProcessed_mutex);
        //    parent->scansToBeProcessed.clear();
        //    pthread_mutex_unlock(&parent->toBeProcessed_mutex);

    }

    return NULL;

}

void
ScanMatcher::computeBounds(double *minx, double *miny, double *maxx,
        double *maxy)
{
    *minx = DBL_MAX;
    *maxx = -DBL_MAX;
    *miny = DBL_MAX;
    *maxy = -DBL_MAX;

    sm_tictoc("rebuild_bounds");
    // Compute bounds of the scans.
    list<Scan *>::iterator it;
    for (it = scans.begin(); it != scans.end(); ++it) {
        Scan * s = *it;
        for (unsigned cidx = 0; cidx < s->contours.size(); cidx++) {
            for (unsigned i = 0; i < s->contours[cidx]->points.size(); i++) {
                smPoint p = s->contours[cidx]->points[i];

                *minx = fmin(*minx, p.x);
                *maxx = fmax(*maxx, p.x);
                *miny = fmin(*miny, p.y);
                *maxy = fmax(*maxy, p.y);
            }
        }
        //make sure the scan origin is in the map
        *minx = fmin(*minx, s->T.x);
        *maxx = fmax(*maxx, s->T.x);
        *miny = fmin(*miny, s->T.y);
        *maxy = fmax(*maxy, s->T.y);
    }
    sm_tictoc("rebuild_bounds");

}

void
ScanMatcher::rebuildRaster(RasterLookupTable ** rasterTable)
{
    //dispatcher for which rebuildRaster version to use
    rebuildRaster_blurLine(rasterTable);
}



void
ScanMatcher::rebuildRaster_olson(RasterLookupTable ** rasterTable)
{
    double minx, maxx;
    double miny, maxy;
    computeBounds(&minx, &miny, &maxx, &maxy);

    // create the LUT, with a bit more space than we currently
    // think we'll need (the thought being that we could add new
    // scans without having to rebuild the whole damn thing
    double margin = fmax(1, 0.1 * fmax(maxx - minx, maxy - miny));

    RasterLookupTable * rt = new RasterLookupTable(minx - margin,
            miny - margin, maxx + margin, maxy + margin, metersPerPixel,
            downsampleFactor);

    // For a long time, I experimented with drawing older scans
    // with greater "weight" than newer scans. I now think this
    // isn't a good idea. It tends to discount new (and often
    // useful) feature observations. Arbitrarilly rendering the
    // oldest scan with greater weight magnifies the error in that
    // one particular scan and can induce oscillatory behavior
    // (where we jump back and forth between two slightly
    // misalgined scans.)

    static int lutSq_first_zero = -1;
    static unsigned char * lutSq = RasterLookupTable::makeLut(512, 10, 1.0,
            &lutSq_first_zero);
    double lutSqRange = pow(0.25, 2);

    // draw each scan.
    list<Scan *>::iterator it;
    for (it = scans.begin(); it != scans.end(); ++it) {
        Scan * s = *it;
        for (unsigned cidx = 0; cidx < s->contours.size(); cidx++) {
            for (unsigned i = 0; i + 1 < s->contours[cidx]->points.size(); i++) {
                smPoint p0 = s->contours[cidx]->points[i];
                smPoint p1 = s->contours[cidx]->points[i + 1];

                double length = sm_dist(&p0, &p1);

                rt->drawRectangle((p0.x + p1.x) / 2, (p0.y + p1.y) / 2, length,
                        0, atan2(p1.y - p0.y, p1.x - p0.x), lutSq, 512,
                        lutSq_first_zero, lutSqRange);

            }
        }
    }

    //  free(lutSq);

    //      rt->dumpTable("rt");
    //    rt->drawTable();
    if (*rasterTable != NULL)
        delete *rasterTable;
    *rasterTable = rt;
    return;
}

void
ScanMatcher::rebuildRaster_blur(RasterLookupTable ** rasterTable)
{
    double minx, maxx;
    double miny, maxy;
    computeBounds(&minx, &miny, &maxx, &maxy);

    // create the LUT, with a bit more space than we currently
    // think we'll need (the thought being that we could add new
    // scans without having to rebuild the whole damn thing
    double margin = fmax(1, 0.1 * fmax(maxx - minx, maxy - miny));

    sm_tictoc("rebuild_alloc");
    RasterLookupTable * rt = new RasterLookupTable(minx - margin,
            miny - margin, maxx + margin, maxy + margin, metersPerPixel,
            downsampleFactor);
    sm_tictoc("rebuild_alloc");

    sm_tictoc("blured");
    sm_tictoc("rebuild_drawLines");
    // draw each scan.
    list<Scan *>::iterator it;
    for (it = scans.begin(); it != scans.end(); ++it) {
        Scan * s = *it;
        for (unsigned cidx = 0; cidx < s->contours.size(); cidx++) {
            for (unsigned i = 0; i + 1 < s->contours[cidx]->points.size(); i++) {
                smPoint p0 = s->contours[cidx]->points[i];
                smPoint p1 = s->contours[cidx]->points[i + 1];

                CvPoint cv_p0;
                CvPoint cv_p1;
                rt->worldToTable(p0.x, p0.y, &cv_p0.x, &cv_p0.y);
                rt->worldToTable(p1.x, p1.y, &cv_p1.x, &cv_p1.y);
                cvLine(&rt->distim, cv_p0, cv_p1, CV_RGB(255, 255, 255), 10);

            }
        }
    }
    sm_tictoc("rebuild_drawLines");

    sm_tictoc("gaussianSmooth");
    cvSmooth(&rt->distim, &rt->distim, CV_GAUSSIAN, 31);
    sm_tictoc("gaussianSmooth");
    sm_tictoc("blured");

    //  rt->dumpTable("rt");
    //  rt->drawTable();

    if (*rasterTable != NULL)
        delete *rasterTable;
    *rasterTable = rt;
    return;
}

void
ScanMatcher::drawBlurredScan(RasterLookupTable ** rasterTable, Scan * s)
{
    RasterLookupTable * rt = *rasterTable;
    for (unsigned cidx = 0; cidx < s->contours.size(); cidx++) {
        for (unsigned i = 0; i + 1 < s->contours[cidx]->points.size(); i++) {
            smPoint p0 = s->contours[cidx]->points[i];
            smPoint p1 = s->contours[cidx]->points[i + 1];

            CvPoint cv_p0;
            CvPoint cv_p1;
            rt->worldToTable(p0.x, p0.y, &cv_p0.x, &cv_p0.y);
            rt->worldToTable(p1.x, p1.y, &cv_p1.x, &cv_p1.y);
            sm_drawLine_lineKernel(&rt->distim, cv_p0, cv_p1, kernelSize,
                    squareKernel, lineKernel, diagLineKernel);
        }
    }
}

void
ScanMatcher::rebuildRaster_blurLine(RasterLookupTable ** rasterTable)
{
    double minx, maxx;
    double miny, maxy;
    computeBounds(&minx, &miny, &maxx, &maxy);

    // create the LUT, with a bit more space than we currently
    // think we'll need (the thought being that we could add new
    // scans without having to rebuild the whole damn thing
    double margin = fmax(.5, 0.1 * fmax(maxx - minx, maxy - miny));

    //  fprintf(stderr,"table minx=%.3f, maxx=%.3f, miny=%.3f, maxy=%.3f, margin=%f\n",minx,maxx,miny,maxy,margin);

    sm_tictoc("rebuild_alloc");
    RasterLookupTable * rt = new RasterLookupTable(minx - margin,
            miny - margin, maxx + margin, maxy + margin, metersPerPixel,
            downsampleFactor);
    sm_tictoc("rebuild_alloc");

    sm_tictoc("drawBlurLines");
    // draw each scan.
    //  fprintf(stderr,"drawing lines....  ");

    //  FILE * f = fopen("scanToDraw.m","w");
    //  fprintf(f, "s =[\n");
    list<Scan *>::iterator it;
    for (it = scans.begin(); it != scans.end(); ++it) {
        Scan * s = *it;
        for (unsigned cidx = 0; cidx < s->contours.size(); cidx++) {
            for (unsigned i = 0; i + 1 < s->contours[cidx]->points.size(); i++) {
                smPoint p0 = s->contours[cidx]->points[i];
                smPoint p1 = s->contours[cidx]->points[i + 1];

                CvPoint cv_p0;
                CvPoint cv_p1;
                rt->worldToTable(p0.x, p0.y, &cv_p0.x, &cv_p0.y);
                rt->worldToTable(p1.x, p1.y, &cv_p1.x, &cv_p1.y);
                //        fprintf(f, "%d %d %d %d \n", cv_p0.x+1, cv_p0.y+1, cv_p1.x+1, cv_p1.y+1); //+1 for matlab indexing
                sm_drawLine_lineKernel(&rt->distim, cv_p0, cv_p1, kernelSize,
                        squareKernel, lineKernel, diagLineKernel);
            }
        }
    }
    //  fprintf(f, "];\n");
    //  fclose(f);
    sm_tictoc("drawBlurLines");
    //  rt->dumpTable("rt");
    //  fprintf(stderr,"done! \n");


    //  rt->drawTable();

    if (*rasterTable != NULL)
        delete *rasterTable;
    *rasterTable = rt;
    return;
}

ScanTransform
ScanMatcher::gridMatch(smPoint * points, unsigned numPoints, ScanTransform * prior,
        double xRange, double yRange, double thetaRange, int * xSat, int *ySat,
        int * thetaSat)
{
    if (useThreads)
        pthread_mutex_lock(&rlt_mutex);

    //  fprintf(stderr,"matching has rlt lock\n");

    if (rlt == NULL) {
        fprintf(stderr, "ERROR: raster lookup table is NULL\n");
        exit(1);
    }

    if (useMultiRes > 0 && rlt_low_res == NULL) {
        fprintf(
                stderr,
                "ERROR: low res raster lookup table is NULL, and multiRes is %d\n",
                useMultiRes);
        exit(1);
    }

    //////////////////////////////////////////////
    // Here's where we actually do a scan match!
    ScanTransform r;
    if (useMultiRes <= 0) {
        sm_tictoc("evaluate3D");
        r = rlt->evaluate3D(points, numPoints, prior, xRange, yRange,
                thetaRange, thetaResolution, xSat, ySat, thetaSat);
        sm_tictoc("evaluate3D");
    } else {
        sm_tictoc("evaluate3D_multiRes");
        r = rlt_low_res->evaluate3D_multiRes(rlt, points, numPoints, prior,
                xRange, yRange, thetaRange, thetaResolution, xSat, ySat,
                thetaSat);
        sm_tictoc("evaluate3D_multiRes");
    }
    r.theta = sm_normalize_theta(r.theta);

    //    fprintf(stderr,"r.x=%f \t r.y=%f \t r.t=%f\t r.score=%f\n", r.x, r.y, r.theta, r.score);
    //  fprintf(stderr,"r1.x=%f \t r1.y=%f \t r1.t=%f\t r1.score=%f\n",r1.x,r1.y,r1.theta,r1.score);
    //  fprintf(stderr,"r2.x=%f \t r2.y=%f \t r2.t=%f\t r2.score=%f\n",r2.x,r2.y,r2.theta,r2.score);

    if (useThreads)
        pthread_mutex_unlock(&rlt_mutex);
    //  fprintf(stderr,"matching released rlt lock\n");
    return r;

}

ScanTransform
ScanMatcher::coordAscentMatch(smPoint * points, unsigned numPoints,
        ScanTransform * startTrans)
{
    typedef enum
    {
        Front = 0,
        Back = 1,
        Right = 2,
        Left = 3,
        TurnLeft = 4,
        TurnRight = 5,
        Done = 6
    }move_type_t;
    if (useThreads)
        pthread_mutex_lock(&rlt_mutex);

    //halve the step size this many times.
    int numRounds = 5;
    //at each round, try this many steps
    int max_steps=100;
    if (matchingMode>SM_RUN_GRID){
      max_steps=1000;
    }
    //start with a step size that is half the brute force search's resolution
    float ldelta = metersPerPixel / 2.0;
    //  float ldelta_diag = sqrt(2) / 2* metersPerPixel / 2.0;
    float adelta = thetaResolution / 2.0;
    //  float adelta_diag = sqrt(2) / 2* thetaResolution / 2.0;

    //if matching mode is Y only, start with the "Right" step
    move_type_t startMove = Front;
    if (matchingMode==SM_Y_COORD_ONLY || matchingMode == SM_Y_GRID_COORD)
      startMove = Right;

    ScanTransform currentTrans = *startTrans;
    int totalNumSteps = 0; //keep track of total number of tests
    int numSteps = 0;
    for (int i = 0; i < numRounds; i++) {
        bool hadImprovement = true;
        numSteps = 0;
        while (hadImprovement) {
            if (numSteps > max_steps) {
                fprintf(stderr,"gradient ascent didn't converge on round %d\n",i);
                break;
            }
            numSteps++;
            totalNumSteps++;

            hadImprovement = false; //at least one direction should improve
            //try each direction
            for (int move = startMove; move < Done; move++) {
                ScanTransform testTrans = currentTrans;
                switch (move)
                    {
                case Front:
                    testTrans.x += ldelta;
                    break;
                case Back:
                    testTrans.x -= ldelta;
                    break;
                case Right:
                    testTrans.y += ldelta;
                    break;
                case Left:
                    testTrans.y -= ldelta;
                    break;
                case TurnLeft:
                    testTrans.theta += adelta;
                    break;
                case TurnRight:
                    testTrans.theta -= adelta;
                    break;
                default:
                    assert(false);
                    }
                testTrans.score = rlt->getScore(points, numPoints, &testTrans);
                if (testTrans.score > currentTrans.score) {
                    //we got an improvement, keep it
                    currentTrans = testTrans;
                    hadImprovement = true;
                }
            }
        }
        //    fprintf(stderr,"didn't have improvment after %d steps, total steps is %d\n",numSteps,totalNumSteps);
        //half the step size again
        ldelta /= 2.0;
        adelta /= 2.0;
        //    ldelta_diag /= 2.0;
        //    adelta_diag /= 2.0;

    }
    //  fprintf(stderr, "totalNumSteps= %d, numSteps at lowest level=%d\n", totalNumSteps, numSteps);
    //  if (fabs(currentTrans.x - startTrans->x) > metersPerPixel || fabs(currentTrans.y - startTrans->y) > metersPerPixel
    //      || fabs(currentTrans.theta - startTrans->theta) > thetaResolution) {
    //    fprintf(stderr,
    //        "WARNING:gradient descent moved more than 1 cell away!, totalNumSteps= %d, numSteps at lowest level=%d\n",
    //        totalNumSteps, numSteps);
    //    fprintf(
    //        stderr,
    //        "polished: score=%f  (%.6f,%.6f,%.6f),  \t normal: score=%f  (%.6f,%.6f,%.6f) \t delta: score=%f  (%.6f,%.6f,%.6f)\n",
    //        currentTrans.score, currentTrans.x, currentTrans.y, currentTrans.theta, startTrans->score, startTrans->x,
    //        startTrans->y, startTrans->theta, currentTrans.score - startTrans->score, currentTrans.x - startTrans->x,
    //        currentTrans.y - startTrans->y, currentTrans.theta - startTrans->theta);
    //  }

    //get the number of hits
    currentTrans.hits = rlt->getNumHits(points, numPoints, &currentTrans);

    if (useThreads)
        pthread_mutex_unlock(&rlt_mutex);

    return currentTrans;
}

void
ScanMatcher::addScanToBeProcessed(smPoint * points, unsigned numPoints,
        ScanTransform * T, sm_laser_type_t laser_type, int64_t utime)
{
    if (!useThreads) {
        addScan(points, numPoints, T, laser_type, utime);
        if (verbose)
            fprintf(stderr, "done\n");
        return;
    } else {
        Scan * s = new Scan(numPoints, points, *T, laser_type, utime);
        pthread_mutex_lock(&toBeProcessed_mutex);
        scansToBeProcessed.push_back(s);
        cancelAdd = false;
        pthread_mutex_unlock(&toBeProcessed_mutex);
        pthread_cond_broadcast(&toBeProcessed_cv);
    }
}

void
ScanMatcher::addScan(smPoint * points, unsigned numPoints, ScanTransform * T,
        sm_laser_type_t laser_type, int64_t utime, bool rebuildNow)
{
    Scan * s = new Scan(numPoints, points, *T, laser_type, utime, true);
    addScan(s, rebuildNow);
}

void
ScanMatcher::addScan(Scan *s, bool rebuildNow)
{

    if (useThreads)
        pthread_mutex_lock(&scans_mutex);
    if (s != NULL) {
        scans.push_back(s);
        if (scans.size() > maxNumScans) {
            delete scans.front();
            scans.pop_front();
        }
    }

    if (rebuildNow) {
        list<Scan *>::iterator it;
        for (it = scans.begin(); it != scans.end(); ++it) {
            Scan * scan = *it;
            if (scan->contours.size() == 0) {
                sm_tictoc("findContours");
                ContourExtractor * cextractor = new ContourExtractor(
                        scan->laser_type);
                cextractor->findContours(scan->ppoints, scan->numPoints,
                        scan->contours);
                delete cextractor; //TODO: probably a cleaner way
                sm_tictoc("findContours"); //      s->drawContours();
            }
        }

        if (useThreads)
            pthread_mutex_lock(&rlt_mutex);

        //rebuild now
        //    sm_tictoc("rebuildRaster_olson");
        //    rebuildRaster_olson(&rlt);
        //    sm_tictoc("rebuildRaster_olson");


        //  sm_tictoc("rebuildRaster_blur");
        //  rebuildRaster_blur(&rlt);
        //  sm_tictoc("rebuildRaster_blur");

        //        sm_tictoc("rebuildRaster_blurLine");
        //        rebuildRaster_blurLine(&rlt);
        //        sm_tictoc("rebuildRaster_blurLine");


        sm_tictoc("rebuildRaster");
        rebuildRaster(&rlt);
        sm_tictoc("rebuildRaster");

        //  CvMat * tab = rlt->drawTable();
        //  cvFlip(tab, tab);
        //  cvSaveImage("table.bmp", tab);
        //  smPoint tpoint = { T->x, T->y };
        //  CvMat * cont = scans.back()->drawContours(rlt->maxDrawDim,rlt->pixelsPerMeter, &tpoint);
        //  cvSaveImage("contours.bmp", cont);

        if (useMultiRes > 0) {
            sm_tictoc("rebuild_lowRes");
            if (rlt_low_res != NULL)
                delete rlt_low_res;
            rlt_low_res = new RasterLookupTable(rlt, downsampleFactor);
            sm_tictoc("rebuild_lowRes");
        }

        //  rlt->dumpTable("High_res");
        //  rlt_low_res->dumpTable("Low_res");

        if (useThreads) {
            pthread_mutex_unlock(&rlt_mutex);
        }
    }
    if (useThreads) {
        pthread_mutex_unlock(&scans_mutex);
    }

}

void
ScanMatcher::drawGUI(smPoint * points, unsigned numPoints,
        ScanTransform transf, CvMat * heightGui, const char * guiName,
        CvScalar scanColor)
{

    if (useThreads) {
        pthread_mutex_lock(&scans_mutex);
        pthread_mutex_lock(&rlt_mutex);
    }
    //  fprintf(stderr,"gui has rlt lock\n");

    CvMat * gui = rlt->drawTable();
    list<Scan *>::iterator it;
    for (it = scans.begin(); it != scans.end(); ++it) {
        Scan * s = *it;
        rlt->drawRobot(gui, s->T, CV_RGB(0, 255, 0));
    }
    rlt->drawScan(gui, points, numPoints, transf, scanColor);

    cvFlip(gui, gui);

    CvSize guiSize = cvGetSize(gui);
    CvSize heightSize;
    if (heightGui != NULL)
        heightSize = cvGetSize(heightGui);
    else {
        heightSize.width = 0;
        heightSize.height = 0;
    }

    int squareSize = rlt->maxDrawDim;

    sm_allocateOrResizeMat(&fullGui, squareSize, squareSize + heightSize.width,
            CV_8UC3);
    cvSet(fullGui, CV_RGB(255, 255, 255));
    //put the map etc in the fullGui
    CvMat subGui;
    CvRect subGuiRect;
    subGuiRect.height = guiSize.height;
    subGuiRect.width = guiSize.width;
    subGuiRect.x = (squareSize - guiSize.width) / 2;
    subGuiRect.y = (squareSize - guiSize.height) / 2;
    cvGetSubRect(fullGui, &subGui, subGuiRect);
    cvCopy(gui, &subGui);

    if (heightGui != NULL) {
        //put the height Drawing in the fullGui
        subGuiRect.height = heightSize.height;
        subGuiRect.width = heightSize.width;
        subGuiRect.x = squareSize;
        subGuiRect.y = 0;
        cvGetSubRect(fullGui, &subGui, subGuiRect);
        cvCopy(heightGui, &subGui);

        cvLine(fullGui, cvPoint(squareSize, 0),
                cvPoint(squareSize, squareSize), CV_RGB(0, 0, 0), 1);
    }
    sm_opencvDisplayWrapper::display(guiName, fullGui);
    //  cvNamedWindow("ScanMatcher", CV_WINDOW_AUTOSIZE);
    //  cvShowImage("ScanMatcher", gui);
    //  cvShowImage("ScanMatcher", fullGui);

    if (useThreads) {
        pthread_mutex_unlock(&rlt_mutex);
        pthread_mutex_unlock(&scans_mutex);
    }
    //  fprintf(stderr,"gui released rlt lock\n");

    //  static int frameCount=0;
    //  char fname[256];
    //  sprintf(fname,"frame_%06d.png",frameCount++);
    //  cvSaveImage(fname,squareGui);

}

ScanTransform
ScanMatcher::matchSuccessive(smPoint * points, unsigned numPoints,
        sm_laser_type_t laser_type, int64_t utime, bool preventAddScan,
        ScanTransform * prior)
{
    int xSat = 0, ySat = 0, thetaSat = 0;
    if (numScans() > 0) {
        sm_tictoc("scanMatch");

        double xRange1 = initialSearchRangeXY;
        double yRange1 = initialSearchRangeXY;
        double thetaRange1 = initialSearchRangeTheta;

        ScanTransform poseGuess;
        if (prior != NULL) {
            poseGuess = *prior;
        } else {
            if (stationaryMotionModel){
            //guess Velocity is 0
              poseGuess.x = currentPose.x;
              poseGuess.y = currentPose.y;
              poseGuess.theta = currentPose.theta;
            }
            else{
              //guess that velocity is constant, but capped at the maxSearchRange...
              double dx = (currentPose.x - prevPose.x);
              if (fabs(dx) > maxSearchRangeXY)
                dx = sm_fsign(dx) * maxSearchRangeXY;
              poseGuess.x = currentPose.x + dx;

              double dy = (currentPose.y - prevPose.y);
              if (fabs(dy) > maxSearchRangeXY)
                dy = sm_fsign(dy) * maxSearchRangeXY;
              poseGuess.y = currentPose.y + dy;

              double dt = sm_normalize_theta(currentPose.theta - prevPose.theta);
              if (fabs(dt) > maxSearchRangeTheta)
                dt = sm_fsign(dt) * maxSearchRangeTheta;
              poseGuess.theta = sm_normalize_theta(currentPose.theta + dt);
            }
            poseGuess.score = motionModelPriorWeight;
        }
        //make sure that the search range is large enough for the current velocity!
        xRange1 = fmax(xRange1, fabs(1.5 * (currentPose.x - prevPose.x)));
        yRange1 = fmax(yRange1, fabs(1.5 * (currentPose.y - prevPose.y)));
        thetaRange1 = fmax(thetaRange1, fabs(1.5 * sm_normalize_theta(
                currentPose.theta - prevPose.theta)));

        prevPose = currentPose;

        int numTries = 3;
        int tr = 0;
        if (matchingMode==SM_Y_GRID_COORD){
          xRange1 = 0; //gets rounded up to 1 pixel
          numTries=1;
        }
        if (matchingMode <SM_RUN_GRID) {
            for (tr = 0; tr < numTries; tr++) {
                xRange1 = fmin(xRange1, maxSearchRangeXY);
                yRange1 = fmin(yRange1, maxSearchRangeXY);
                thetaRange1 = fmin(thetaRange1, maxSearchRangeTheta);
                if (tr > 0)
                    fprintf(stderr, "xRange=%f, yRange=%f, thetaRange=%f\n",
                            xRange1, yRange1, thetaRange1);
                currentPose = gridMatch(points, numPoints, &poseGuess, xRange1,
                        yRange1, thetaRange1, &xSat, &ySat, &thetaSat);
                if (matchingMode==SM_Y_GRID_COORD)
                  xSat=0; //x doesn't matter
                if (!(xSat || ySat || thetaSat))
                    break;
                else {
                    if (thetaSat) {
                        fprintf(stderr, "Hit Theta!");
                        thetaRange1 *= 2;
                    } else {
                        poseGuess = currentPose; //hopefully all are close
                        //          poseGuess.theta = currentPose.theta; //other ones are probably crap
                        thetaRange1 = thetaResolution * 6; //theta should be almost correct!
                        if (xSat && ySat) {
                            fprintf(stderr, "hit BOTH x and y!");
                            xRange1 *= 2;
                            yRange1 *= 2;
                        } else if (xSat) {
                            fprintf(stderr, "just hit X");
                            xRange1 *= 3;
                            yRange1 /= 1.5;
                            poseGuess.y = currentPose.y;
                        } else if (ySat) {
                            fprintf(stderr, "just hit Y");
                            xRange1 /= 3;
                            yRange1 *= 1.5;
                            poseGuess.x = currentPose.x;
                        }
                        fprintf(stderr, " ... trying again... ");

                    }
                }

            }

            if (matchingMode == SM_GRID_COORD) {
                sm_tictoc("GradientAscent Polish");
                double sigma[4] =
                    { currentPose.sigma[0], currentPose.sigma[1],
                            currentPose.sigma[3], currentPose.sigma[4] };
                double evals[2];
                double evecs[4];
                CvMat cv_sigma = cvMat(2, 2, CV_64FC1, sigma);
                CvMat cv_evals = cvMat(2, 1, CV_64FC1, evals);
                CvMat cv_evecs = cvMat(2, 2, CV_64FC1, evecs);
                cvEigenVV(&cv_sigma, &cv_evecs, &cv_evals);
                if (evals[0] <= 0 || evals[1] <= 0)
                    fprintf(stderr, "ERROR: cov is not P.S.D!\n");
                if (evals[0] < .0075 && evals[1] < .0075) { //only polish if match already confident
                    ScanTransform polished = coordAscentMatch(points,
                            numPoints, &currentPose);
                    currentPose = polished;
                }
                sm_tictoc("GradientAscent Polish");
            }
        } else {
            sm_tictoc("GradientAscent Match");
            ScanTransform polished = coordAscentMatch(points,
                    numPoints, &poseGuess);
            memset(polished.sigma, 0, 9 * sizeof(double));
            polished.sigma[0] = polished.sigma[4] = polished.sigma[8] = .0001;
            currentPose = polished;
            sm_tictoc("GradientAscent Match");

        }
        //make sure that the variances are nonzero
        currentPose.sigma[0] = fmax(1e-12, currentPose.sigma[0]);
        currentPose.sigma[4] = fmax(1e-12, currentPose.sigma[4]);
        currentPose.sigma[8] = fmax(1e-12, currentPose.sigma[8]);

        //increase variances if we hit a rail and had to try more than once...
        if (tr > 1) {
            currentPose.sigma[0] *= tr;
            currentPose.sigma[1] *= tr;
            currentPose.sigma[4] *= tr;
            if (xSat || ySat || thetaSat)
                fprintf(stderr, "Warning!hit a rail in the end!\n");
            else
                fprintf(stderr, "Found large enough search region\n");
        }
        if (xSat) {
            fprintf(stderr, "hit rail in X direction %d\n", xSat);
            currentPose.sigma[0] *= 3;
            currentPose.sigma[1] *= 2;
            currentPose.sigma[4] *= 1.5;
        }
        if (ySat) {
            fprintf(stderr, "hit rail in Y direction %d\n", ySat);
            currentPose.sigma[0] *= 1.5;
            currentPose.sigma[1] *= 2;
            currentPose.sigma[4] *= 3;
        }
        if (thetaSat) {
            fprintf(stderr, "hit rail in theta direction %d\n", thetaSat);
            currentPose.sigma[0] *= 3;
            currentPose.sigma[1] *= 3;
            currentPose.sigma[4] *= 3;
        }

        sm_tictoc("scanMatch");

    }
    if (matchingMode==SM_Y_COORD_ONLY || matchingMode==SM_Y_GRID_COORD){
      currentPose.x = prevPose.x; //don't let things move in x...
    }


    if (numScans() > 0) {
        bool addScan = false;
        if ((double) currentPose.hits / (double) numPoints < addScanHitThresh) {
            if (verbose)
                fprintf(stderr, "hits = %.1f%% should add scan...  ",
                        (double) currentPose.hits / (double) numPoints * 100);
            addScan = true;
        } else {
            if (useThreads) { //dip in hits was temporary... don't add the map if we haven't yet
                pthread_mutex_lock(&rlt_mutex);
                cancelAdd = true;
                pthread_mutex_unlock(&rlt_mutex);
            }
        }

        if (addScan && preventAddScan) {
            if (verbose)
                fprintf(stderr, "NOT adding scan due to preventAddScan! \n");
            addScan = false;
        }
        if (addScan && (xSat || ySat || thetaSat)) {
            if (verbose)
                fprintf(stderr, "NOT adding scan due to hitting a rail! \n");
            addScan = false;
        }

        if (addScan) {
            sm_tictoc("addScan");
            addScanToBeProcessed(points, numPoints, &currentPose, laser_type,
                    utime);
            sm_tictoc("addScan");
        }
    } else {
        if (verbose)
            fprintf(stderr, "adding first scan\n");
        if (prior!=NULL)
          memcpy(&currentPose,prior,sizeof(currentPose));
        addScan(points, numPoints, &currentPose, laser_type, utime); //do a blocking add...

        if (verbose)
            fprintf(stderr, "done adding first scan\n");

        //set the covariance to be nonzero so it doesn't crash isam
        currentPose.sigma[0] = .1;
        currentPose.sigma[4] = .1;
        currentPose.sigma[8] = .1;
    }

    sm_tictoc("laser_handler");

    return currentPose;

}

int ScanMatcher::isUsingIPP()
{
#ifdef USE_IPP
    return 1;
#else
    return 0;
#endif
}
