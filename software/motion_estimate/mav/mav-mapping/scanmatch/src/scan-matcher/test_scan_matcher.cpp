#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <signal.h>
#include <opencv/highgui.h>
#include <scanmatch/ScanMatcher.hpp>
#include <scanmatch/ScanMatchingOpencvUtils.hpp>

using namespace std;
using namespace scanmatch;

////////////////////////////////////////////////////////////////////
//hardcoded global parameters
////////////////////////////////////////////////////////////////////
static int beam_skip = 3; //downsample ranges by only taking 1 out of every beam_skip points
static double spatialDecimationThresh = .5; //don't discard a point if its range is this far from the last added neighbor
static double metersPerPixel = .01; //translational resolution for the brute force search
static double thetaResolution = .05; //angular step size for the brute force search
static sm_incremental_matching_modes_t  matchingMode= SM_GRID_COORD; //use gradient descent to improve estimate after brute force search
static int useMultires = 3; // low resolution will have resolution metersPerPixel * 2^useMultiRes
static int useThreads = 1; //using threading to make the occupancy map rebuilds occur in a background thread

static double initialSearchRangeXY = .15; //nominal range that will be searched over
static double initialSearchRangeTheta = .1;

//SHOULD be set greater than the initialSearchRange
static double maxSearchRangeXY = .3; //if a good match isn't found I'll expand and try again up to this size...
static double maxSearchRangeTheta = .4; //if a good match isn't found I'll expand and try again up to this size...


static bool stationaryMotionModel = false; //use constant velocity model
static double motionModelPriorWeight =0; //don't use the prior for anything other than centering the window


static double maxRange = 25; //discard beams with reading further than this value
static float validBeamAngles[2] =
    { -2.1, 2.1 }; //valid part of the field of view of the laser in radians, 0 is the center beam

static int maxNumScans = 30; //keep around this many scans in the history
static double addScanHitThresh = .80; //add a new scan to the map when the number of "hits" drops below this


////////////////////////////////////////////////////////////////////
//where all the work is done
////////////////////////////////////////////////////////////////////
void
laser_handler(ScanMatcher * sm, float *ranges, int numRanges,
        double angularResolution, double startAngle)
{
    sm_tictoc("laser_handler");
    sm_tictoc("recToSend");

    ////////////////////////////////////////////////////////////////////
    //Project ranges into points, and decimate points so we don't have too many
    ////////////////////////////////////////////////////////////////////
    smPoint * points = (smPoint *) calloc(numRanges, sizeof(smPoint));
    int numValidPoints = sm_projectRangesAndDecimate(beam_skip,
            spatialDecimationThresh, ranges, numRanges, startAngle,
            angularResolution, points, maxRange, validBeamAngles[0],
            validBeamAngles[1]);
    if (numValidPoints < 30) {
        fprintf(stderr, "WARNING! NOT ENOUGH VALID POINTS! numValid=%d\n",
                numValidPoints);
        return;
    }

    ////////////////////////////////////////////////////////////////////
    //Actually do the matching
    ////////////////////////////////////////////////////////////////////
    ScanTransform r = sm->matchSuccessive(points, numValidPoints,
            SM_HOKUYO_UTM, sm_get_utime(), NULL); //don't have a better estimate than prev, so just set prior to NULL

    ////////////////////////////////////////////////////////////////////
    //Print current position periodically!
    ////////////////////////////////////////////////////////////////////
    static double lastPrintTime = 0;
    if (sm_get_time() - lastPrintTime > 2.0) {
        lastPrintTime = sm_get_time();
        //print out current state
        fprintf(
                stderr,
                "x=%+7.3f y=%+7.3f t=%+7.3f\t score=%f hits=%.2f sx=%.2f sxy=%.2f sy=%.2f st=%.2f, numValid=%d\n",
                r.x, r.y, r.theta, r.score, (double) r.hits
                        / (double) numValidPoints, r.sigma[0], r.sigma[1],
                r.sigma[4], r.sigma[8], numValidPoints);

    }

    ////////////////////////////////////////////////////////////////////
    //Do drawing periodically!
    ////////////////////////////////////////////////////////////////////
    static double lastDrawTime = 0;
    if (sm_get_time() - lastDrawTime > .2) {
        lastDrawTime = sm_get_time();
        sm_tictoc("drawing");
        sm->drawGUI(points, numValidPoints, r, NULL);
        sm_tictoc("drawing");
    }

    ////////////////////////////////////////////////////////////////////
    //cleanup!
    ////////////////////////////////////////////////////////////////////

    free(points);
    sm_tictoc("laser_handler");

}

void
shutdown_handler(int unused __attribute__((unused)))
{
    printf("Shutting Down \n");
    sm_tictoc(NULL);
    exit(1);
}

int
main(int argc __attribute__((unused)), char *argv[] __attribute__((unused)))
{
    sm_setupOpencvErrorHandler();
    signal(SIGINT, shutdown_handler);
    //initialize tictoc for threading
    sm_tictoc_init();

    //create the actual scan matcher object
    ScanMatcher * sm = new ScanMatcher(metersPerPixel, thetaResolution,
            useMultires, useThreads,true);

    if (sm->isUsingIPP())
      fprintf(stderr, "Using IPP\n");
    else
      fprintf(stderr, "NOT using IPP\n");

    ScanTransform startPose;
    memset(&startPose, 0, sizeof(startPose));
    startPose.theta = M_PI / 2; //set the scan matcher to start at pi/2... cuz it looks better
    sm->initSuccessiveMatchingParams(maxNumScans, initialSearchRangeXY,
            maxSearchRangeXY, initialSearchRangeTheta, maxSearchRangeTheta,
            matchingMode, addScanHitThresh,
            stationaryMotionModel,motionModelPriorWeight,&startPose);

    //set the scan matcher to start at pi/2... cuz it looks better
    memset(&sm->currentPose, 0, sizeof(sm->currentPose));
    sm->currentPose.theta = M_PI / 2;
    sm->prevPose = sm->currentPose;

    //hardcode stuff that would normally be in the laser_msgs
    //settings for a hokuyo UTM laser scanner
    maxRange = 29.7;
    int numreadings = 1080;
    double angularResolution = 0.0043629999999999997;
    double startAngle = -2.3561939999999999;

    FILE * f = fopen("laserdata.txt", "r");
    if (f == NULL) {
        printf("ERROR: Couldn't open file\n");
        exit(1);
    }
    vector<float*> scans;

    bool stop = false;
    while (true) {

        float * r = (float *) malloc(numreadings * sizeof(float));
        for (int i = 0; i < numreadings; i++) {
            int b = fscanf(f, "%f, ", &r[i]);
            if (b <= 0)
                stop = true;
        }
        if (stop) {
            free(r);
            break;
        }
        scans.push_back(r);
    }

    for (unsigned int i = 0; i < scans.size(); i++)
        laser_handler(sm, scans[i], numreadings, angularResolution, startAngle);

    delete sm;

    sm_tictoc(NULL);
    printf("done with last scan\n");

}

