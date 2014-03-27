/**
 * @file ImageCalUtility/ImageCalUtility.cc
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

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <opencv/cv.h>

#include <LibMultiSense/MultiSenseChannel.hh>

namespace {  // anonymous

void usage(const char *programNameP) 
{
    fprintf(stderr, 
            "USAGE: %s -e <extrinisics_file> -i <intrinsics_file> [<options>]\n", 
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

}; // anonymous

using namespace crl::multisense;

int main(int    argc, 
         char **argvPP)
{
    std::string ipAddress = "10.66.171.21";
    std::string intrinsicsFile;
    std::string extrinsicsFile;
    bool        setCal=false;
    bool        prompt=true;

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:e:i:sy")))
        switch(c) {
        case 'a': ipAddress      = std::string(optarg);    break;
        case 'i': intrinsicsFile = std::string(optarg);    break;
        case 'e': extrinsicsFile = std::string(optarg);    break;
        case 's': setCal         = true;                   break;
        case 'y': prompt         = false;                  break;
        default: usage(*argvPP);                           break;
        }

    //
    // Verify options

    if (intrinsicsFile.empty() || extrinsicsFile.empty()) {
        fprintf(stderr, "Both intrinsics and extrinsics files must be set\n");
        usage(*argvPP);
    }

    if (true == setCal &&
        (false == fileExists(intrinsicsFile) ||
         false == fileExists(extrinsicsFile))) {
        
        fprintf(stderr, "intrinsics or extrinsics file not found\n");
        usage(*argvPP);
    }

    if (false == setCal && true == prompt &&
        (true == fileExists(intrinsicsFile) ||
         true == fileExists(extrinsicsFile))) {
        
        fprintf(stdout, 
                "One or both of \"%s\" and \"%s\" already exists.\n\n"
                "Really overwrite these files? (y/n): ",
                intrinsicsFile.c_str(),
                extrinsicsFile.c_str());
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
        fprintf(stderr, "failed to query sensor version: %s\n", 
                Channel::statusString(status));
        goto clean_out;
    }
    
    //
    // Query

    if (false == setCal) {

        image::Calibration c;

        status = channelP->getImageCalibration(c);
        if (Status_Ok != status) {
            fprintf(stderr, "failed to query image calibration: %s\n", 
                    Channel::statusString(status));
            goto clean_out;
        }

        CvFileStorage *inFile, *exFile;

        inFile = cvOpenFileStorage(intrinsicsFile.c_str(), NULL, CV_STORAGE_WRITE);

        if (NULL == inFile) {
            fprintf(stderr, "failed to cvOpenFileStorage(%s) for writing\n", 
                    intrinsicsFile.c_str());
            goto clean_out;
        }

        exFile = cvOpenFileStorage(extrinsicsFile.c_str(), NULL, CV_STORAGE_WRITE);

        if (NULL == exFile) {
            fprintf(stderr, "failed to cvOpenFileStorage(%s) for writing\n", 
                    extrinsicsFile.c_str());
            goto clean_out;
        }

        CvMat *M1, *D1, *M2, *D2, *R1, *R2, *P1, *P2;

        M1 = cvCreateMat(3, 3, CV_64FC1);
        D1 = cvCreateMat(1, 8, CV_64FC1);
        R1 = cvCreateMat(3, 3, CV_64FC1);
        P1 = cvCreateMat(3, 4, CV_64FC1);
        M2 = cvCreateMat(3, 3, CV_64FC1);
        D2 = cvCreateMat(1, 8, CV_64FC1);
        R2 = cvCreateMat(3, 3, CV_64FC1);
        P2 = cvCreateMat(3, 4, CV_64FC1);

#define CPY_ARR_1(t_,a_,n_)                                             \
        for(int i_=0; i_<(n_); i_++)                                    \
            CV_MAT_ELEM(*(t_), double, 0, i_) = (a_)[i_];               \

#define CPY_ARR_2(t_,a_,n_,m_)                                          \
        for(int i_=0; i_<(n_); i_++)                                    \
            for(int j_=0; j_<(m_); j_++)                                \
                CV_MAT_ELEM(*(t_), double, i_, j_) = (a_)[i_][j_];      \
        
        CPY_ARR_2(M1, c.left.M, 3, 3);
        CPY_ARR_1(D1, c.left.D, 8);
        CPY_ARR_2(R1, c.left.R, 3, 3);
        CPY_ARR_2(P1, c.left.P, 3, 4);

        CPY_ARR_2(M2, c.right.M, 3, 3);
        CPY_ARR_1(D2, c.right.D, 8);
        CPY_ARR_2(R2, c.right.R, 3, 3);
        CPY_ARR_2(P2, c.right.P, 3, 4);
        
        cvWrite(inFile, "M1", M1, cvAttrList(0,0));
        cvWrite(inFile, "D1", D1, cvAttrList(0,0));
        cvWrite(inFile, "M2", M2, cvAttrList(0,0));
        cvWrite(inFile, "D2", D2, cvAttrList(0,0));
        
        cvWrite(exFile, "R1", R1, cvAttrList(0,0));
        cvWrite(exFile, "P1", P1, cvAttrList(0,0));
        cvWrite(exFile, "R2", R2, cvAttrList(0,0));
        cvWrite(exFile, "P2", P2, cvAttrList(0,0));

        cvReleaseFileStorage(&inFile);
        cvReleaseFileStorage(&exFile);

        cvReleaseMat(&M1);
        cvReleaseMat(&D1);
        cvReleaseMat(&R1);
        cvReleaseMat(&P1);
        cvReleaseMat(&M2);
        cvReleaseMat(&D2);
        cvReleaseMat(&R2);
        cvReleaseMat(&P2);

    } else {

        CvFileStorage *inFile, *exFile;

        inFile = cvOpenFileStorage(intrinsicsFile.c_str(), NULL, CV_STORAGE_READ);
        
        if (NULL == inFile) {
            fprintf(stderr, "failed to cvOpenFileStorage(%s) for reading\n", 
                    intrinsicsFile.c_str());
            goto clean_out;
        }

        CvMat *M1, *D1, *M2, *D2, *R1, *R2, *P1, *P2;

        M1 = (CvMat *) cvReadByName(inFile, NULL, "M1", NULL);
        D1 = (CvMat *) cvReadByName(inFile, NULL, "D1", NULL);
        M2 = (CvMat *) cvReadByName(inFile, NULL, "M2", NULL);
        D2 = (CvMat *) cvReadByName(inFile, NULL, "D2", NULL);
        
        cvReleaseFileStorage(&inFile);

        if(!M1 || !D1 || !M2 || !D2 ||
           M1->rows != 3 || M1->cols != 3 ||
           D1->rows != 1 || D1->cols < 5  ||
           M2->rows != 3 || M2->cols != 3 ||
           D2->rows != 1 || D2->cols < 5) {
            fprintf(stderr, "intrinsic matrices incomplete in %s\n",
                    intrinsicsFile.c_str());
            goto clean_out;
        }

        exFile = cvOpenFileStorage(extrinsicsFile.c_str(), NULL, CV_STORAGE_READ);

        if (NULL == exFile) {
            fprintf(stderr, "failed to cvOpenFileStorage(%s) for reading\n", 
                    extrinsicsFile.c_str());
            goto clean_out;
        }

        R1 = (CvMat *) cvReadByName(exFile, NULL, "R1", NULL);
        P1 = (CvMat *) cvReadByName(exFile, NULL, "P1", NULL);
        R2 = (CvMat *) cvReadByName(exFile, NULL, "R2", NULL);
        P2 = (CvMat *) cvReadByName(exFile, NULL, "P2", NULL);
        
        cvReleaseFileStorage(&exFile);

        if (!R1 || !P1 || !R2 || !P2 ||
            R1->rows != 3 || R1->cols != 3 ||
            P1->rows != 3 || P1->cols != 4 ||
            R2->rows != 3 || R2->cols != 3 ||
            P2->rows != 3 || P2->cols != 4) {
            fprintf(stderr, "extrinsic matrices incomplete in %s\n",
                    extrinsicsFile.c_str());
            goto clean_out;
        }

        image::Calibration c;

#define CPY_MAT_1(a_,t_,n_)                                             \
        for(int i_=0; i_<(n_); i_++)                                    \
            (a_)[i_] = CV_MAT_ELEM(*(t_), double, 0, i_);               \

#define CPY_MAT_2(a_,t_,n_,m_)                                          \
        for(int i_=0; i_<(n_); i_++)                                    \
            for(int j_=0; j_<(m_); j_++)                                \
                (a_)[i_][j_] = CV_MAT_ELEM(*(t_), double, i_, j_);      \

        CPY_MAT_2(c.left.M, M1, 3, 3);
        memset(&(c.left.D[0]), 0, sizeof(c.left.D));
        CPY_MAT_1(c.left.D, D1, 5);
        CPY_MAT_2(c.left.R, R1, 3, 3);
        CPY_MAT_2(c.left.P, P1, 3, 4);

        CPY_MAT_2(c.right.M, M2, 3, 3);
        memset(&(c.right.D[0]), 0, sizeof(c.right.D));
        CPY_MAT_1(c.right.D, D2, 5);
        CPY_MAT_2(c.right.R, R2, 3, 3);
        CPY_MAT_2(c.right.P, P2, 3, 4);

        cvReleaseMat(&M1);
        cvReleaseMat(&D1);
        cvReleaseMat(&R1);
        cvReleaseMat(&P1);
        cvReleaseMat(&M2);
        cvReleaseMat(&D2);
        cvReleaseMat(&R2);
        cvReleaseMat(&P2);
               
        status = channelP->setImageCalibration(c);
        if (Status_Ok != status) {
            fprintf(stderr, "failed to set image calibration: %s\n", 
                    Channel::statusString(status));
            goto clean_out;
        }

        fprintf(stdout, "Image calibration successfully updated\n");
    }

clean_out:

    Channel::Destroy(channelP);
    return 0;
}
