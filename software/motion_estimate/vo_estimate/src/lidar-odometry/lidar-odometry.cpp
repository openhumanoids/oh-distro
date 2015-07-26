#include "lidar-odometry.hpp"

using namespace std;

LidarOdom::LidarOdom(boost::shared_ptr<lcm::LCM> &lcm_):
  lcm_(lcm_)
{


  // Set up frames and config:
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);


  laserType_ = SM_HOKUYO_UTM;
  //parameters for a hokuyo with the helicopters mirror's attached
  validBeamAngles_[0] = -2.1;
  validBeamAngles_[1] = 2.1;
  beamSkip_ = 3;
  spatialDecimationThresh_ = .2;
  maxRange_ = 29.7;


  publishRelative_ = FALSE;
  publishPose_ = TRUE;
  doDrawing_ = TRUE;

    //hardcoded scan matcher params
    double metersPerPixel = .02; //translational resolution for the brute force search
    double thetaResolution = .01; //angular step size for the brute force search
    sm_incremental_matching_modes_t  matchingMode= SM_COORD_ONLY; //use gradient descent to improve estimate after brute force search
    int useMultires = 3; // low resolution will have resolution metersPerPixel * 2^useMultiRes

    double initialSearchRangeXY = .15; //nominal range that will be searched over
    double initialSearchRangeTheta = .1;

    //SHOULD be set greater than the initialSearchRange
    double maxSearchRangeXY = .3; //if a good match isn't found I'll expand and try again up to this size...
    double maxSearchRangeTheta = .2; //if a good match isn't found I'll expand and try again up to this size...

    int maxNumScans = 30; //keep around this many scans in the history
    double addScanHitThresh = .80; //add a new scan to the map when the number of "hits" drops below this

    bool stationaryMotionModel = false; //use constant velocity model
    double motionModelPriorWeight =0; //don't use the prior for anything other than centering the window

    int useThreads = 1;

    //create the actual scan matcher object
    sm_ = new ScanMatcher(metersPerPixel, thetaResolution, useMultires,
            useThreads,true);

    if (sm_->isUsingIPP())
        fprintf(stderr, "Using IPP\n");
    else
        fprintf(stderr, "NOT using IPP\n");


    ScanTransform startPose;
    memset(&startPose, 0, sizeof(startPose));
    startPose.theta = 0; //set the scan matcher to start at 0 heading... cuz pi/2 would be rediculous
  sm_->initSuccessiveMatchingParams(maxNumScans, initialSearchRangeXY,
            maxSearchRangeXY, initialSearchRangeTheta, maxSearchRangeTheta,
            matchingMode, addScanHitThresh,
            stationaryMotionModel,motionModelPriorWeight,&startPose);



  cout <<"LidarOdom Constructed\n";

}


LidarOdom::~LidarOdom()
{
}


void 
sm_roll_pitch_yaw_to_quat (const double rpy[3], double q[4])
{
    double roll = rpy[0], pitch = rpy[1], yaw = rpy[2];

    double halfroll = roll / 2;
    double halfpitch = pitch / 2;
    double halfyaw = yaw / 2;

    double sin_r2 = sin (halfroll);
    double sin_p2 = sin (halfpitch);
    double sin_y2 = sin (halfyaw);

    double cos_r2 = cos (halfroll);
    double cos_p2 = cos (halfpitch);
    double cos_y2 = cos (halfyaw);

    q[0] = cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2;
    q[1] = sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2;
    q[2] = cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2;
    q[3] = cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2;
}


void LidarOdom::doOdometry(const float* ranges, int nranges, float rad0, float radstep, int64_t utime){


    ////////////////////////////////////////////////////////////////////
    //Project ranges into points, and decimate points so we don't have too many
    ////////////////////////////////////////////////////////////////////
    smPoint * points = (smPoint *) calloc(nranges, sizeof(smPoint));
    int numValidPoints = sm_projectRangesAndDecimate(beamSkip_,
            spatialDecimationThresh_, ranges, nranges, rad0,
            radstep, points, maxRange_, validBeamAngles_[0],
            validBeamAngles_[1]);
    if (numValidPoints < 30) {
        fprintf(stderr, "WARNING! NOT ENOUGH VALID POINTS! numValid=%d\n",
                numValidPoints);
        free(points);
        return;
    }

    ////////////////////////////////////////////////////////////////////
    //Actually do the matching
    ////////////////////////////////////////////////////////////////////
    ScanTransform r = sm_->matchSuccessive(points, numValidPoints,
            laserType_, utime, NULL); //don't have a better estimate than prev, so just set prior to NULL
                                                //utime is ONLY used to tag the scans that get added to the map, doesn't actually matter

    ////////////////////////////////////////////////////////////////////
    //Publish
    ////////////////////////////////////////////////////////////////////
    //publish integrated absolute position instead of delta to last scan
    sm::rigid_transform_2d_t odom;
    odom.utime = utime;
    memset(&odom, 0, sizeof(odom));
    odom.pos[0] = r.x;
    odom.pos[1] = r.y;
    odom.theta = r.theta;

    //publish integrated absolute position instead of delta to last scan
    sm::rigid_transform_2d_t cur_odom;
    memset(&cur_odom, 0, sizeof(cur_odom));
    cur_odom.utime = utime;
    cur_odom.pos[0] = r.x;
    cur_odom.pos[1] = r.y;
    cur_odom.theta = r.theta;
    memcpy(cur_odom.cov, r.sigma, 9 * sizeof(double));

    if (!publishRelative_)
        lcm_->publish("ODOM_CHANNEL", &cur_odom);
        //sm::rigid_transform_2d_t_publish(app->lcm, app->odom_chan, &cur_odom);
    else {
        //compute the relative odometry
        if (prevOdom_.utime > 0) {
            sm::rigid_transform_2d_t rel_odom;
            rel_odom.utime = cur_odom.utime;
            rel_odom.utime_prev = prevOdom_.utime;
            double delta[2];
            sm_vector_sub_2d(cur_odom.pos, prevOdom_.pos, delta);

            sm_rotate2D(delta, -cur_odom.theta, rel_odom.pos);
            rel_odom.theta = sm_angle_subtract(cur_odom.theta,
                    prevOdom_.theta);
            //rotate the covariance estimate to body frame
            sm_rotateCov2D(cur_odom.cov, -cur_odom.theta, rel_odom.cov);
            lcm_->publish("ODOM_CHANNEL", &rel_odom);
            //sm::rigid_transform_2d_t_publish(app->lcm, app->odom_chan,
            //        &rel_odom);
        }

    }

    if (publishPose_) {
        bot_core::pose_t pose;
        memset(&pose, 0, sizeof(pose));
        pose.utime = cur_odom.utime;

        memcpy(pose.pos, cur_odom.pos, 2 * sizeof(double));

        double rpy[3] =
            { 0, 0, cur_odom.theta };
        sm_roll_pitch_yaw_to_quat(rpy, pose.orientation);

        //sm_pose_t_publish(app->lcm, app->pose_chan, &pose);
        lcm_->publish("POSE_BODY", &pose);
    }
    sm_tictoc("recToSend");

    prevOdom_ = cur_odom;

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
    if (doDrawing_ && sm_get_time() - lastDrawTime > .2) {
        lastDrawTime = sm_get_time();
        sm_tictoc("drawing");
        sm_->drawGUI(points, numValidPoints, r, NULL);
        sm_tictoc("drawing");
    }

    ////////////////////////////////////////////////////////////////////
    //cleanup!
    ////////////////////////////////////////////////////////////////////

    free(points);
    sm_tictoc("process_laser");
}
