/**
 * @file ImuTestUtility/ImuTestUtility.cc
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
 *   2013-11-12, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <errno.h>

#include <LibMultiSense/MultiSenseChannel.hh>

using namespace crl::multisense;

namespace {  // anonymous

volatile bool doneG         = false;
FILE         *logFileP      = stdout;
uint32_t      accel_samples = 0;
uint32_t      gyro_samples  = 0;
uint32_t      mag_samples   = 0;
int64_t       sequence      = -1;
uint32_t      messages      = 0;
uint32_t      dropped       = 0;

void usage(const char *programNameP) 
{
    fprintf(stderr, "USAGE: %s [<options>]\n", programNameP);
    fprintf(stderr, "Where <options> are:\n");
    fprintf(stderr, "\t-a <ip_address>    : IPV4 address (default=10.66.171.21)\n");
    fprintf(stderr, "\t-m <mtu>           : default=7200\n");
    fprintf(stderr, "\t-f <log_file>      : FILE to log IMU data (stdout by default)\n");
    
    exit(-1);
}

void signalHandler(int sig)
{
    fprintf(stderr, "Shutting down on signal: %s\n",
            strsignal(sig));
    doneG = true;
}

void imuCallback(const imu::Header& header,
                 void              *userDataP)
{
    std::vector<imu::Sample>::const_iterator it = header.samples.begin();

    for(; it!=header.samples.end(); ++it) {

        const imu::Sample& s = *it;

        switch(s.type) {
        case imu::Sample::Type_Accelerometer: accel_samples ++; break;
        case imu::Sample::Type_Gyroscope:     gyro_samples ++;  break;
        case imu::Sample::Type_Magnetometer:  mag_samples ++;   break;
        }

        if (logFileP)
            fprintf(logFileP, "%d %.6f %.6f %.6f %.6f\n",
                    s.type, 
                    s.timeSeconds + 1e-6 * s.timeMicroSeconds,
                    s.x, s.y, s.z);
    }

    if (-1 == sequence)
        sequence = header.sequence;
    else if ((sequence + 1) != header.sequence) {
        const int32_t d = (header.sequence - (sequence + 1));
        dropped += d;
    }

    sequence = header.sequence;
    messages ++;
}

}; // anonymous

int main(int    argc, 
         char **argvPP)
{
    std::string currentAddress = "10.66.171.21";
    const char *logFileNameP   = NULL;
    uint32_t    mtu            = 7200;

    signal(SIGINT, signalHandler);

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:f:m:v")))
        switch(c) {
        case 'a': currentAddress = std::string(optarg);    break;
        case 'f': logFileNameP   = optarg;                 break;
        case 'm': mtu            = atoi(optarg);           break;
        default: usage(*argvPP);                           break;
        }

    //
    // Initialize communications.

    Channel *channelP = Channel::Create(currentAddress);
    if (NULL == channelP) {
	fprintf(stderr, "Failed to establish communications with \"%s\"\n",
		currentAddress.c_str());
	return -1;
    }

    //
    // Query firmware version

    system::VersionInfo v;

    Status status = channelP->getVersionInfo(v);
    if (Status_Ok != status) {
        fprintf(stderr, "failed to query sensor version: %s\n", 
                Channel::statusString(status));
        goto clean_out;
    }

    //
    // Make sure firmware supports IMU

    if (v.sensorFirmwareVersion <= 0x0202) {
        fprintf(stderr, "IMU support requires sensor firmware version v2.3 or greater, sensor is "
                "running v%d.%d\n",
                v.sensorFirmwareVersion >> 8,
                v.sensorFirmwareVersion & 0xff);
            goto clean_out;
    }

    //
    // Turn off all streams by default

    status = channelP->stopStreams(Source_All);
    if (Status_Ok != status) {
        fprintf(stderr, "failed to stop streams: %s\n", 
                Channel::statusString(status));
        goto clean_out;
    }

    //
    // Was logging requested ?

    if (NULL != logFileNameP) {
        
        //
        // Open the log file

        logFileP = fopen(logFileNameP, "w+");
        if (NULL == logFileP) {
            fprintf(stderr, "failed to open \"%s\" for writing: %s\n", logFileNameP, strerror(errno));
            goto clean_out;
        }

    }

    //
    // Change MTU

    status = channelP->setMtu(mtu);
    if (Status_Ok != status) {
        fprintf(stderr, "failed to set MTU to %d: %s\n", mtu,
                Channel::statusString(status));
        goto clean_out;
    }

    //
    // Add callbacks

    channelP->addIsolatedCallback(imuCallback);

    //
    // Start streaming

    status = channelP->startStreams(Source_Imu);
    if (Status_Ok != status) {
        fprintf(stderr, "failed to start streams: %s\n", 
                Channel::statusString(status));
        goto clean_out;
    }

    while(!doneG)
        usleep(100000);
        
    //
    // Stop streaming

    status = channelP->stopStreams(Source_All);
    if (Status_Ok != status) {
        fprintf(stderr, "failed to stop streams: %s\n", 
                Channel::statusString(status));
    }

    //
    // Report simple stats

    {
        int64_t imu_total = accel_samples + gyro_samples + mag_samples;
        if (imu_total > 0) {
            fprintf(stderr, "IMU samples : total: %ld, accel: %.1f%%, gyro: %.1f%%, mag: %.1f%%\n",
                    imu_total,
                    100.0 * static_cast<double>(accel_samples) / static_cast<double>(imu_total),
                    100.0 * static_cast<double>(gyro_samples) / static_cast<double>(imu_total),
                    100.0 * static_cast<double>(mag_samples) / static_cast<double>(imu_total));
        }
        
        if (messages > 0) 
            fprintf(stderr, "IMU messages: total: %u, dropped: %u (%.6f%%)\n",
                    messages, dropped, 
                    100* static_cast<double>(dropped) / static_cast<double>(messages+dropped));
    }

clean_out:
        
    if (logFileNameP)
        fclose(logFileP);

    Channel::Destroy(channelP);
    return 0;
}
