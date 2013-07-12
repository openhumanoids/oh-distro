/**
 * @file LidarCalUtility/LidarCalUtility.cc
 *
 * Copyright 2013
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * This software is free: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation,
 * version 3 of the License.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Significant history (date, user, job code, action):
 *   2013-05-23, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <opencv/cv.h>

#include <LibMultiSense/MultiSenseChannel.hh>

namespace {  // anonymous

void usage(const char *programNameP) 
{
    fprintf(stderr, 
            "USAGE: %s -f <calibration_file> [<options>]\n", 
            programNameP);
    fprintf(stderr, "Where <options> are:\n");
    fprintf(stderr, "\t-a <ip_address>      : ip address (default=10.66.171.21)\n");
    fprintf(stderr, "\t-s                   : set the calibration (default is query)\n");
    fprintf(stderr, "\t-y                   : disable confirmation prompts\n");
    
    exit(-1);
}

bool fileExists(const std::string& name)
{
    struct stat sbuf;
    return (0 == stat(name.c_str(), &sbuf));
}

const char *laserToSpindleNameP       = "laser_T_spindle";
const char *cameraToSpindleFixedNameP = "camera_T_spindle_fixed";

}; // anonymous

using namespace crl::multisense;

int main(int    argc, 
         char **argvPP)
{
    std::string        ipAddress = "10.66.171.21";
    std::string        calFile;
    bool               setCal=false;
    bool               prompt=true;

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:f:sy")))
        switch(c) {
        case 'a': ipAddress = std::string(optarg);    break;
        case 'f': calFile   = std::string(optarg);    break;
        case 's': setCal    = true;                   break;
        case 'y': prompt    = false;                  break;
        default: usage(*argvPP);                      break;
        }

    //
    // Verify options

    if (calFile.empty()) {
        fprintf(stderr, "Must provide a file argument\n");
        usage(*argvPP);
    }

    if (true == setCal && false == fileExists(calFile)) {
        
        fprintf(stderr, "file not found: \"%s\"\n", calFile.c_str());
        usage(*argvPP);
    }
    
    if (false == setCal && true == prompt &&
        true == fileExists(calFile)) {
        
        fprintf(stdout, 
                "\"%s\" already exists.\n\n"
                "Really overwrite this file? (y/n): ",
                calFile.c_str());
        fflush(stdout);

        int c = getchar();
        if ('Y' != c && 'y' != c) {
            fprintf(stdout, "Aborting\n");
            return 0;
        }
    }

    //
    // Initialize communications.

    Channel *channelP = Channel::Create(ipAddress);
    if (NULL == channelP) {
	fprintf(stderr, "Failed to establish communications with \"%s\"\n",
		ipAddress.c_str());
	return -1;
    }

    //
    // Query version

    Status status;
    VersionType version;

    status = channelP->getSensorVersion(version);
    if (Status_Ok != status) {
        fprintf(stderr, "failed to query sensor version: %d\n", status);
        goto clean_out;
    }
    
    //
    // Query

    if (false == setCal) {

        lidar::Calibration c;

        status = channelP->getLidarCalibration(c);
        if (Status_Ok != status) {
            fprintf(stderr, "failed to query lidar calibration: %d\n", status);
            goto clean_out;
        }

        CvFileStorage *cvFile = cvOpenFileStorage(calFile.c_str(), NULL, CV_STORAGE_WRITE);

        if (NULL == cvFile) {
            fprintf(stderr, "failed to cvOpenFileStorage(%s) for writing\n", 
                    calFile.c_str());
            goto clean_out;
        }

        CvMat *laserToSpindle       = cvCreateMat(4, 4, CV_64FC1);
        CvMat *cameraToSpindleFixed = cvCreateMat(4, 4, CV_64FC1);

#define CPY_ARR_2(t_,a_,n_,m_)                                          \
        for(int i_=0; i_<(n_); i_++)                                    \
            for(int j_=0; j_<(m_); j_++)                                \
                CV_MAT_ELEM(*(t_), double, i_, j_) = (a_)[i_][j_];      \
        

        CPY_ARR_2(laserToSpindle, c.laserToSpindle, 4, 4);
        CPY_ARR_2(cameraToSpindleFixed, c.cameraToSpindleFixed, 4, 4);
        
        cvWrite(cvFile, laserToSpindleNameP, laserToSpindle, cvAttrList(0,0));
        cvWrite(cvFile, cameraToSpindleFixedNameP, cameraToSpindleFixed, cvAttrList(0,0));

        cvReleaseFileStorage(&cvFile);

        cvReleaseMat(&laserToSpindle);
        cvReleaseMat(&cameraToSpindleFixed);

    } else {

        CvFileStorage *cvFile = cvOpenFileStorage(calFile.c_str(), NULL, CV_STORAGE_READ);
 
        if (NULL == cvFile) {
            fprintf(stderr, "failed to cvOpenFileStorage(%s) for reading\n", 
                    calFile.c_str());
            goto clean_out;
        }

        CvMat *laserToSpindle       = (CvMat *) cvReadByName(cvFile, NULL, laserToSpindleNameP, NULL);
        CvMat *cameraToSpindleFixed = (CvMat *) cvReadByName(cvFile, NULL, cameraToSpindleFixedNameP, NULL);
        
        cvReleaseFileStorage(&cvFile);

        if (NULL == laserToSpindle          || NULL == cameraToSpindleFixed    ||
            laserToSpindle->rows       != 4 || laserToSpindle->cols       != 4 ||
            cameraToSpindleFixed->rows != 4 || cameraToSpindleFixed->cols != 4) {

            fprintf(stderr, "calibration matrices incomplete in %s, "
                    "expecting two 4x4 matrices\n", calFile.c_str());
            goto clean_out;
        }

        lidar::Calibration c;

#define CPY_MAT_2(a_,t_,n_,m_)                                          \
        for(int i_=0; i_<(n_); i_++)                                    \
            for(int j_=0; j_<(m_); j_++)                                \
                (a_)[i_][j_] = CV_MAT_ELEM(*(t_), double, i_, j_);      \

        CPY_MAT_2(c.laserToSpindle, laserToSpindle, 4, 4);
        CPY_MAT_2(c.cameraToSpindleFixed, cameraToSpindleFixed, 4, 4);

        cvReleaseMat(&laserToSpindle);
        cvReleaseMat(&cameraToSpindleFixed);
        
        status = channelP->setLidarCalibration(c);
        if (Status_Ok != status) {
            fprintf(stderr, "failed to set lidar calibration: %d\n", status);
            goto clean_out;
        }

        fprintf(stdout, "Lidar calibration successfully updated\n");
    }

clean_out:

    Channel::Destroy(channelP);
    return 0;
}
