#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <opencv/highgui.h>
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <getopt.h>

#include <lcm/lcm.h>
#include <scanmatch/ScanMatcher.hpp>
#include <lcmtypes/sm_rigid_transform_2d_t.h>
#include <lcmtypes/sm_planar_lidar_t.h>
#include <lcmtypes/sm_pose_t.h>
#include <lcmtypes/drc_lcmtypes.h>
#include "rotations.h"
#include <deque>
#include <pthread.h>

#include <boost/assign/std/vector.hpp>
#include <lcmtypes/visualization.h>
#include <pointcloud_tools/pointcloud_vis.hpp>

using namespace boost::assign; // bring 'operator+=()' into scope
using namespace Eigen;
using namespace std;
using namespace scanmatch;

typedef struct
{
    lcm_t * lcm;
    ScanMatcher * sm;
    int do_drawing;
    int publish_relative;
    int publish_pose;
    char * lidar_chan;
    char * odom_chan;
    char * pose_chan;
    sm_laser_type_t laser_type;
    int beam_skip; //downsample ranges by only taking 1 out of every beam_skip points
    double spatialDecimationThresh; //don't discard a point if its range is more than this many std devs from the mean range (end of hallway)
    double maxRange; //discard beams with reading further than this value
    float validBeamAngles[2]; //valid part of the field of view of the laser in radians, 0 is the center beam
    sm_rigid_transform_2d_t prev_odom;
    int verbose;

    //lcm reading thread stuff
    pthread_t processor_thread;
    pthread_mutex_t lcm_data_mutex;
    pthread_cond_t newLcmData_cv;
    deque<sm_planar_lidar_t *> * laser_queue;
    int noDrop;

    // IMU:
    drc_imu_t* last_head_imu;
    char * head_imu_chan;
    double last_head_orient[4];
    
    // Righted Laser Pose:
    char * righted_laser_pose_chan;
    bot_core_pose_t* last_righted_pose;
  
    pointcloud_vis* pc_vis_;   
    
} app_t;

////////////////////////////////////////////////////////////////////
//where all the work is done
////////////////////////////////////////////////////////////////////

static void
laser_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)), const char * channel __attribute__((unused)),
        const sm_planar_lidar_t * msg, void * user  __attribute__((unused)))
{
    app_t * app = (app_t *) user;
    pthread_mutex_lock(&app->lcm_data_mutex);
    app->laser_queue->push_back(sm_planar_lidar_t_copy(msg));
    pthread_mutex_unlock(&app->lcm_data_mutex);
    pthread_cond_broadcast(&app->newLcmData_cv);
}


// q (a quaternion) should be wxyz ordered
void quat_to_euler(double q[], double& yaw, double& pitch, double& roll) {
  roll = atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
  pitch = asin(2*(q[0]*q[2]-q[3]*q[1]));
  yaw = atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
}

void quat_to_euler(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}


Eigen::Quaterniond euler_to_quat(double yaw, double pitch, double roll) {
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}


static void
head_imu_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)), const char * channel __attribute__((unused)),
        const drc_imu_t * msg, void * user  __attribute__((unused)))
{
    app_t * app = (app_t *) user;
    pthread_mutex_lock(&app->lcm_data_mutex);
    app->last_head_imu =  drc_imu_t_copy(msg);
    memcpy(app->last_head_orient, msg->orientation,sizeof(double)*4) ;
    
    //std::cout << "got head imu: " << msg->utime << "\n";
    pthread_mutex_unlock(&app->lcm_data_mutex);
    //pthread_cond_broadcast(&app->newLcmData_cv);
}

static void
righted_pose_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)), const char * channel __attribute__((unused)),
        const bot_core_pose_t * msg, void * user  __attribute__((unused)))
{
    app_t * app = (app_t *) user;
    pthread_mutex_lock(&app->lcm_data_mutex);
    app->last_righted_pose =  bot_core_pose_t_copy(msg);
    
    pthread_mutex_unlock(&app->lcm_data_mutex);
    //pthread_cond_broadcast(&app->newLcmData_cv);
}


/*
void LCMLidarHandler::onLidarData(const lcm_recv_buf_t *rbuf, const char *channel,
                                  const bot_core_planar_lidar_t *msg, void *user_data )
{
        LCMAdapter* adapter = (LCMAdapter*)user_data;
        
        // convert to point cloud
        PointCloudPtr p = convertToPointCloud(msg);
        int64_t msg_utime =msg->utime;
        if (adapter->getMapReset()){
         //msg_utime = -1; 
         // setting the to -1 will reset the map
         // in LaserOdometry::onData by producing a large shift
         adapter->setMapReset(false);
        }
        
        //mfallon added these to support multifloor/submap ops:
        int temp_submap =0;
        int temp_floor =0;
        adapter->onData(msg_utime,temp_floor,temp_submap, std::string(channel), p);     
}*/


void convertToPointCloud(const sm_planar_lidar_t *msg,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, 
  app_t*  app)
{
  
    // @todo add parameters to point cloud
    // @todo add intensities
    smPoint * points = (smPoint *) calloc(msg->nranges, sizeof(smPoint));
    int numValidPoints = sm_projectRangesAndDecimate(app->beam_skip,
        app->spatialDecimationThresh, msg->ranges, msg->nranges, msg->rad0,
        msg->radstep, points, app->maxRange, app->validBeamAngles[0],
        app->validBeamAngles[1]);
    for (int i=0; i<numValidPoints; ++i) {
      pcl::PointXYZRGB pt;
      pt.x =points[i].x;
      pt.y =points[i].y;
      pt.z =0;
      pt.r =0;
      pt.g =0;
      pt.b =255;
      cloud->points.push_back(pt);
    }
    
    free(points);

}

 


static void
process_laser(const sm_planar_lidar_t * msg, void * user  __attribute__((unused)))
{
    app_t * app = (app_t *) user;
    sm_tictoc("process_laser");
    sm_tictoc("recToSend");

  bool VERBOSE_LCM =false;
    
  // 1. Extract pitch and roll of the righted laser:
  Eigen::Quaterniond m;
  m = Eigen::Quaterniond(app->last_righted_pose->orientation[0], app->last_righted_pose->orientation[1],
                        app->last_righted_pose->orientation[2], app->last_righted_pose->orientation[3]); // = euler_to_quat(angle , 0,0);
  double righted_ypr[3];
  quat_to_euler(m, righted_ypr[0], righted_ypr[1], righted_ypr[2] );
  //std::cout << "righted_ypr [d]: " << righted_ypr[0]*180/M_PI << ", "
  //          << righted_ypr[1]*180/M_PI << ", " << righted_ypr[2]*180/M_PI << "\n";
  Eigen::Quaterniond m2 = euler_to_quat(0 ,righted_ypr[1], righted_ypr[2] ) ;
                        
  // tiles[i].utime dosent give correct utime for some
  Isometry3dTime lidarframe_poseT(  msg->utime , Eigen::Isometry3d::Identity());
  lidarframe_poseT.pose.translation() << 0,0, 0; // at null
  lidarframe_poseT.pose.rotate(m2);

  // 2. Convert the Lidar into a point cloud:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  convertToPointCloud (msg, cloud, app);
  //cout << "sending: " << cloud->points.size() << " points\n";
  if (VERBOSE_LCM) app->pc_vis_->pose_to_lcm_from_list(1000, lidarframe_poseT);      
  if (VERBOSE_LCM) app->pc_vis_->ptcld_to_lcm_from_list(1001, *cloud, lidarframe_poseT.utime, lidarframe_poseT.utime); 

  // 3. Project the points into local:
  Isometry3dTime null_poseT(  msg->utime , Eigen::Isometry3d::Identity());
  app->pc_vis_->pose_to_lcm_from_list(1010, null_poseT);      
  Eigen::Isometry3f pose_f = Isometry_d2f(lidarframe_poseT.pose);//.inverse());
  Eigen::Quaternionf pose_quat_f(pose_f.rotation());
  pcl::transformPointCloud (*cloud, *cloud, 
              pose_f.translation(), pose_quat_f); // !! modifies cloud
  if (VERBOSE_LCM) app->pc_vis_->ptcld_to_lcm_from_list(1011, *cloud, null_poseT.utime, null_poseT.utime); 

  // 4. Project valid points onto horizontal
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2d (new pcl::PointCloud<pcl::PointXYZRGB> ());
  // distance off horizontal we are willing to accept non planar points
  double height_threshold = 0.2; // 0.2;  0.7
  for (size_t i=0; i< cloud->points.size(); i++) {
    if ( fabs(cloud->points[i].z)  < height_threshold){ 
      pcl::PointXYZRGB pt = cloud->points[i];
      pt.z =0;
      pt.x = -pt.x; // this is required because we are using a lidar pointing backways
      pt.y = -pt.y;
      pt.z =0;
      cloud2d->points.push_back( pt );
    }
  }
  if (VERBOSE_LCM) app->pc_vis_->ptcld_to_lcm_from_list(1012, *cloud2d, null_poseT.utime, null_poseT.utime); 
      
  // 5. Use these points in scan matching:
  smPoint * points = (smPoint *) calloc( cloud2d->points.size() , sizeof(smPoint));
  for (size_t i=0; i< cloud2d->points.size(); i++) {
    points[i].x = cloud2d->points[i].x;
    points[i].y = cloud2d->points[i].y;
  }
  int numValidPoints = cloud2d->points.size();

    /*   
     * Default code:
    ////////////////////////////////////////////////////////////////////
    //Project ranges into points, and decimate points so we don't have too many
    ////////////////////////////////////////////////////////////////////
    smPoint * points = (smPoint *) calloc(msg->nranges, sizeof(smPoint));
    int numValidPoints = sm_projectRangesAndDecimate(app->beam_skip,
            app->spatialDecimationThresh, msg->ranges, msg->nranges, msg->rad0,
            msg->radstep, points, app->maxRange, app->validBeamAngles[0],
            app->validBeamAngles[1]);
    if (numValidPoints < 30) {
        fprintf(stderr, "WARNING! NOT ENOUGH VALID POINTS! numValid=%d\n",
                numValidPoints);
        free(points);
        return;
    }
    */

    ////////////////////////////////////////////////////////////////////
    //Actually do the matching
    ////////////////////////////////////////////////////////////////////
    ScanTransform r = app->sm->matchSuccessive(points, numValidPoints,
            app->laser_type, msg->utime, NULL); //don't have a better estimate than prev, so just set prior to NULL
                                                //utime is ONLY used to tag the scans that get added to the map, doesn't actually matter

    ////////////////////////////////////////////////////////////////////
    //Publish
    ////////////////////////////////////////////////////////////////////
    //publish integrated absolute position instead of delta to last scan
    sm_rigid_transform_2d_t odom;
    odom.utime = msg->utime;
    memset(&odom, 0, sizeof(odom));
    odom.pos[0] = r.x;
    odom.pos[1] = r.y;
    odom.theta = r.theta;

    //publish integrated absolute position instead of delta to last scan
    sm_rigid_transform_2d_t cur_odom;
    memset(&cur_odom, 0, sizeof(cur_odom));
    cur_odom.utime = msg->utime;
    cur_odom.pos[0] = r.x;
    cur_odom.pos[1] = r.y;
    cur_odom.theta = r.theta;
    memcpy(cur_odom.cov, r.sigma, 9 * sizeof(double));

    if (!app->publish_relative)
        sm_rigid_transform_2d_t_publish(app->lcm, app->odom_chan, &cur_odom);
    else {
        //compute the relative odometry
        if (app->prev_odom.utime > 0) {
            sm_rigid_transform_2d_t rel_odom;
            rel_odom.utime = cur_odom.utime;
            rel_odom.utime_prev = app->prev_odom.utime;
            double delta[2];
            sm_vector_sub_2d(cur_odom.pos, app->prev_odom.pos, delta);

            sm_rotate2D(delta, -cur_odom.theta, rel_odom.pos);
            rel_odom.theta = sm_angle_subtract(cur_odom.theta,
                    app->prev_odom.theta);
            //rotate the covariance estimate to body frame
            sm_rotateCov2D(cur_odom.cov, -cur_odom.theta, rel_odom.cov);
            sm_rigid_transform_2d_t_publish(app->lcm, app->odom_chan,
                    &rel_odom);
        }

    }

    if (app->publish_pose) {
      
      
      
      

      
      
      
      
      
      
      
      
      
      
      
        sm_pose_t pose;
        memset(&pose, 0, sizeof(pose));
        pose.utime = cur_odom.utime;

        memcpy(pose.pos, cur_odom.pos, 2 * sizeof(double));


        // need to apply laser to camera transform to this pose

        double imu_rpy[3];
        
        
        quat_to_euler(app->last_head_orient, imu_rpy[2], imu_rpy[1], imu_rpy[0] );
        std::cout << "imu rpy [d]: " << imu_rpy[0]*180/M_PI << ", "
                     << imu_rpy[1]*180/M_PI << ", "
                     << imu_rpy[2]*180/M_PI << "\n";

        double rpy[3] = { imu_rpy[0], imu_rpy[1], cur_odom.theta }; // pitch and roll comes from imu, yaw from laser
        //double rpy[3] = { 0,0, cur_odom.theta }; // previously
        sm_roll_pitch_yaw_to_quat(rpy, pose.orientation);
                     
                     
        sm_pose_t_publish(app->lcm, "POSE_HEAD", &pose); // mfallon...

        

    }
    sm_tictoc("recToSend");

    app->prev_odom = cur_odom;

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
    if (app->do_drawing && sm_get_time() - lastDrawTime > .2) {
        lastDrawTime = sm_get_time();
        sm_tictoc("drawing");
        app->sm->drawGUI(points, numValidPoints, r, NULL);
        sm_tictoc("drawing");
    }

    ////////////////////////////////////////////////////////////////////
    //cleanup!
    ////////////////////////////////////////////////////////////////////

    free(points);
    sm_tictoc("process_laser");

}

void
app_destroy(app_t *app)
{
    // dump timing stats
    sm_tictoc(NULL);

    if (app) {
        if (app->lidar_chan)
            free(app->lidar_chan);
        if (app->odom_chan)
            free(app->odom_chan);
        if (app->pose_chan)
            free(app->pose_chan);

        lcm_destroy(app->lcm);
        //TODO: kill thread

        free(app);
    }

}

static void
usage(const char *name)
{
    fprintf(
            stderr,
            "usage: %s [options]\n"
                "\n"
                "  -h, --help                      Shows this help text and exits\n"
                "  -l, --lidar <LCM CHANNEL>       Input lcm channel default:\"LASER\"\n"
                "  -o, --odometry <LCM CHANNEL>    Output odometry channel default:\"<lidar chan>_ODOMETRY\"\n"
                "  -r, --relative                  Publish relative\n"
                "  -p, --pose   <LCM CHANNEL>      Publish pose message for easy viewing in viewer default:\"POSE\"\n"
                "  -d, --draw                      Show window with scan matches \n"
                "  -n, --nodrop                    don't drop laser messages if we're getting behind \n"
                "  -v, --verbose                   Be verbose\n"
                "  -m, --mode  \"HOKUYO_UTM\"|\"SICK\" configures low-level options.\n"
                "\n"
                "Low-level options:\n"
                "  -M, --mask <min,max>            Mask min max angles in (radians)\n"
                "  -B, --beamskip <n>              Skip every n beams \n"
                "  -D, --decimation <value>        Spatial decimation threshold (meters?)\n"
                "  -R, --range <range>             Maximum range (meters)\n",
            name);
}

sig_atomic_t still_groovy = 1;

static void
sig_action(int signal, siginfo_t *s, void *user)
{
    still_groovy = 0;
}

//dispatcher for new laser data
static void *
processingFunc(void * user)
{
    app_t * app = (app_t *) user;
    sm_planar_lidar_t * local_laser = NULL;
    pthread_mutex_lock(&app->lcm_data_mutex);
    while (1) {
        if (app->laser_queue->empty()) {
            pthread_cond_wait(&app->newLcmData_cv, &app->lcm_data_mutex);
            continue;
        }

        //there is new data to be processed
        //copy shared data to local storage
        sm_planar_lidar_t * laser_msg;
        if (app->noDrop)
            laser_msg = app->laser_queue->front();
        else
            laser_msg = app->laser_queue->back();

        if (local_laser != NULL)
            sm_planar_lidar_t_destroy(local_laser);
        local_laser = sm_planar_lidar_t_copy(laser_msg);

        //process the data
        pthread_mutex_unlock(&app->lcm_data_mutex);
        process_laser(local_laser, (void *) app);
        pthread_mutex_lock(&app->lcm_data_mutex);

        //remove data from the queue
        int numRemoved = 0;
        while (!app->laser_queue->empty() && app->laser_queue->front()->utime
                <= local_laser->utime) {
            sm_planar_lidar_t_destroy(app->laser_queue->front());
            app->laser_queue->pop_front();
            numRemoved++;
        }
        if (numRemoved > 1) {
            fprintf(stderr, "dropped %d laser messages\n", numRemoved - 1);
        }
    }
    return NULL;
}

int
main(int argc, char *argv[])
{
    setlinebuf(stdout);

    app_t *app = (app_t *) calloc(1, sizeof(app_t));

    app->lidar_chan = strdup("LASER");
    app->pose_chan = strdup("POSE");
    app->head_imu_chan = strdup("HEAD_IMU"); // mfallon
    app->righted_laser_pose_chan = strdup("POSE_RIGHTED"); // mfallon
    app->verbose = 0;
    app->do_drawing = 0;
    app->publish_relative = 0;
    app->publish_pose = 0;

    // set to default values
    app->laser_type = SM_HOKUYO_UTM;
    //TODO:these should be read from command line or something...
    //parameters for a hokuyo with the helicopters mirror's attached
    app->validBeamAngles[0] = -2.1;
    app->validBeamAngles[1] = 2.1;
    app->beam_skip = 3;
    app->spatialDecimationThresh = .2;
    app->maxRange = 29.7;
    app->laser_queue = new deque<sm_planar_lidar_t *> ();

    
    
    const char *optstring = "hl:o:drvm:R:B:D:M:p::n:";
    char c;
    struct option long_opts[] =
        {
            { "help", no_argument, 0, 'h' },
            { "lidar", required_argument, 0, 'l' },
            { "odometry", required_argument, 0, 'o' },
            { "draw", no_argument, 0, 'd' },
            { "relative", no_argument, 0, 'r' },
            { "nodrop", no_argument, 0, 'n' },
            { "mode", required_argument, 0, 'm' },
            { "range", required_argument, 0, 'R' },
            { "beamskip", required_argument, 0, 'B' },
            { "decimation", required_argument, 0, 'D' },
            { "mask", required_argument, 0, 'M' },
            { "pose", optional_argument, 0, 'p' },
            { "verbose", no_argument, 0, 'v' },
            { 0, 0, 0, 0 } };

    while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
        switch (c)
            {
        case 'l':
            free(app->lidar_chan);
            app->lidar_chan = strdup(optarg);
            break;
        case 'o':
            app->odom_chan = strdup(optarg);
            break;
        case 'r':
            app->publish_relative = 1;
            printf("INFO: Publish relative enabled\n");
            break;
        case 'p':
            app->publish_pose = 1;
            printf("INFO: Publish pose message enabled\n");
            if (optarg != NULL) {
                app->pose_chan = strdup(optarg);
            }
            break;
        case 'n':
            app->noDrop = 1;
            break;
        case 'v':
            app->verbose = 1;
            break;
        case 'd':
            app->do_drawing = 1;
            printf("INFO: Drawing enabled\n");

            break;
        case 'm':
            if (optarg && optarg[0] == 'S') {
                // assume we want sick mode:
                app->laser_type = SM_SICK_LMS;
                app->maxRange = 79.0;
                app->beam_skip = 0;
                //                app->spatialDecimationThresh =0;
                // anything else required?
            }
            break;
        case 'R':
            app->maxRange = strtod(optarg, 0);
            break;
        case 'B':
            app->beam_skip = atoi(optarg);
            break;
        case 'D':
            app->spatialDecimationThresh = strtod(optarg, 0);
            break;
        case 'M':
            sscanf(optarg, "%f,%f", &app->validBeamAngles[0],
                    &app->validBeamAngles[1]);
            break;
        case 'h':
        default:
            usage(argv[0]);
            return 1;
            }
    }

    if (!app->odom_chan) {
        int max_len = MIN(256,strlen(app->lidar_chan)+16);
        //printf("max_len:%d\n",max_len);
        app->odom_chan = (char *) calloc(1, max_len * sizeof(char));
        snprintf(app->odom_chan, max_len, "%s_ODOMETRY", app->lidar_chan);
    }

    if (app->verbose) {
        printf("INFO: Listening to:%s\n", app->lidar_chan);
        printf("INFO: Publishing on:%s\n", app->odom_chan);
        printf("INFO: Do Draw:%d\n", app->do_drawing);
        printf("INFO: Publish relative:%d\n", app->publish_relative);
        printf("INFO: Max Range:%lf\n", app->maxRange);
        printf("INFO: SpatialDecimationThresh:%lf\n",
                app->spatialDecimationThresh);
        printf("INFO: Beam Skip:%d\n", app->beam_skip);
        printf("INFO: validRange:%f,%f\n", app->validBeamAngles[0],
                app->validBeamAngles[1]);
    }

    //initialize tictoc for threading
    sm_tictoc_init();

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
    app->sm = new ScanMatcher(metersPerPixel, thetaResolution, useMultires,
            useThreads,true);

    if (app->sm->isUsingIPP())
        fprintf(stderr, "Using IPP\n");
    else
        fprintf(stderr, "NOT using IPP\n");

    
    ScanTransform startPose;
    memset(&startPose, 0, sizeof(startPose));
    startPose.theta = M_PI / 2; //set the scan matcher to start at pi/2... cuz it looks better
    app->sm->initSuccessiveMatchingParams(maxNumScans, initialSearchRangeXY,
            maxSearchRangeXY, initialSearchRangeTheta, maxSearchRangeTheta,
            matchingMode, addScanHitThresh,
            stationaryMotionModel,motionModelPriorWeight,&startPose);

    //setup lcm reading thread
    pthread_mutex_init(&app->lcm_data_mutex, NULL);
    pthread_cond_init(&app->newLcmData_cv, NULL);
    //create processing thread
    pthread_create(&app->processor_thread, 0, (void *
    (*)(void *)) processingFunc, (void *) app);

    /* LCM */
    app->lcm = lcm_create(NULL);
    if (!app->lcm) {
        fprintf(stderr, "ERROR: lcm_create() failed\n");
        return 1;
    }
    

    app->pc_vis_ = new pointcloud_vis(app->lcm);
    app->pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Sensor Pose",5,1) );
    app->pc_vis_->obj_cfg_list.push_back( obj_cfg(1010,"Null Pose",5,1) );
    
  vector <float> colors_r;
  colors_r += 255,0,0;
  app->pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Raw Projected Points"     ,1,1, 1000,1,colors_r));
  vector <float> colors_b;
  colors_b += 0,0,255;
  app->pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1011,"Local Projected Points"     ,1,1, 1010,1,colors_b));
  vector <float> colors_g;
  colors_g += 0,255,0;
  app->pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1012,"Valid Projected Points"     ,1,1, 1010,1,colors_g));
    
    sm_planar_lidar_t_subscribe(app->lcm, app->lidar_chan, laser_handler,
            app);

    bot_core_pose_t_subscribe(app->lcm, app->righted_laser_pose_chan, righted_pose_handler,
            app);
    drc_imu_t_subscribe(app->lcm, app->head_imu_chan, head_imu_handler,
            app);

    // setup sigaction();
    struct sigaction new_action;
    new_action.sa_sigaction = sig_action;
    sigemptyset(&new_action.sa_mask);
    new_action.sa_flags = 0;

    sigaction(SIGINT, &new_action, NULL);
    sigaction(SIGTERM, &new_action, NULL);
    sigaction(SIGKILL, &new_action, NULL);
    sigaction(SIGHUP, &new_action, NULL);

    /* sit and wait for messages */
    while (still_groovy)
        lcm_handle(app->lcm);

    app_destroy(app);

    return 0;
}

