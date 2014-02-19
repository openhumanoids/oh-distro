/**
 * @file ImuConfigUtility/ImuConfigUtility.cc
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
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <errno.h>

#include <LibMultiSense/MultiSenseChannel.hh>

using namespace crl::multisense;

namespace {  // anonymous

std::vector<imu::Info>   sensor_infos;
std::vector<imu::Config> sensor_configs;
uint32_t                 sensor_samplesPerMessage    = 0;
uint32_t                 sensor_maxSamplesPerMessage = 0;

void usage(const char *programNameP) 
{
    fprintf(stderr, "USAGE: %s [<options>]\n", programNameP);
    fprintf(stderr, "Where <options> are:\n");
    fprintf(stderr, "\t-a <ip_address>        : IPV4 address (default=10.66.171.21)\n");
    fprintf(stderr, "\t-q                     : query and report IMU configuration\n");
    fprintf(stderr, "\t-f                     : store IMU configuration in non-volatile flash\n");
    fprintf(stderr, "\t-s <samples>           : set IMU samples-per-message\n");
    fprintf(stderr, "\t-c \"<sensor_config>\"   : IMU sensor configuration string\n\n");
    fprintf(stderr, "And \"<sensor_config>\" is of the following form:\n");
    fprintf(stderr, "\t\"<sensor_name> <enabled> <rate_table_index> <range_table_index>\"\n\n");
    fprintf(stderr, "For example, to enable the accelerometer, and have it use rate index 1 and range index 2:\n\n");
    fprintf(stderr, "\t-c \"accelerometer true 1 2\"\n\n");
    fprintf(stderr, "Multiple \"-c\" options may be specified to configure more than 1 sensor\n\n");
    fprintf(stderr, "Please note that small values for samples-per-message combined with high IMU sensor rates\n");
    fprintf(stderr, "may interfere with the acquisition and transmission of image and lidar data, if applicable\n");

    exit(-1);
}

bool imuInfoByName(const std::string& name, imu::Info& info)
{
    std::vector<imu::Info>::const_iterator it = sensor_infos.begin();

    for(; it != sensor_infos.end(); ++it) 
        if (name == (*it).name) {
            info = (*it);
            return true;
        }

    return false;
}

}; // anonymous

int main(int    argc, 
         char **argvPP)
{
    std::string              currentAddress         = "10.66.171.21";
    int32_t                  user_samplesPerMessage = 0;
    bool                     query                  = false;
    bool                     storeInFlash           = false;
    std::vector<std::string> cli_configs;
    Status                   status;

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:s:c:qf")))
        switch(c) {
        case 'a': currentAddress         = std::string(optarg);    break;
        case 's': user_samplesPerMessage = atoi(optarg);           break;
        case 'q': query                  = true;                   break;
        case 'f': storeInFlash           = true;                   break;
        case 'c': cli_configs.push_back(optarg);                   break;
        default: usage(*argvPP);                                   break;
        }

    //
    // A setting of zero here tells the firmware to leave it alone, useful
    // if you do not want to change the default.

    if (user_samplesPerMessage < 0) {
        fprintf(stderr, "invalid samples-per-message: %d\n", user_samplesPerMessage);
        exit(-2);
    }

    //
    // If no setting changes are requested, perform a query

    if (0 == cli_configs.size() && 0 == user_samplesPerMessage)
        query = true;

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

    status = channelP->getVersionInfo(v);
    if (Status_Ok != status) {
        fprintf(stderr, "failed to query sensor version: %s\n", 
                Channel::statusString(status));
        goto clean_out;
    }

    if (query) {
        fprintf(stdout, "Version information:\n");
        fprintf(stdout, "\tAPI build date      :  %s\n", v.apiBuildDate.c_str());
        fprintf(stdout, "\tAPI version         :  0x%04x\n", v.apiVersion);
        fprintf(stdout, "\tFirmware build date :  %s\n", v.sensorFirmwareBuildDate.c_str());
        fprintf(stdout, "\tFirmware version    :  0x%04x\n", v.sensorFirmwareVersion);
        fprintf(stdout, "\tHardware version    :  0x%lx\n", v.sensorHardwareVersion);
        fprintf(stdout, "\tHardware magic      :  0x%lx\n", v.sensorHardwareMagic);
        fprintf(stdout, "\tFPGA DNA            :  0x%lx\n", v.sensorFpgaDna);
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
    // Query IMU info / configuration

    status = channelP->getImuInfo(sensor_maxSamplesPerMessage,
                                  sensor_infos);
    if (Status_Ok != status) {
        fprintf(stderr, "failed to query imu info: %s\n", 
                Channel::statusString(status));
        goto clean_out;
    }
    status = channelP->getImuConfig(sensor_samplesPerMessage, sensor_configs);
    if (Status_Ok != status) {
        fprintf(stderr, "failed to query imu config: %s\n", 
                Channel::statusString(status));
        goto clean_out;
    }

    if (query) {
        fprintf(stdout, "\nMax IMU samples-per-message: %d\n", sensor_maxSamplesPerMessage);
        fprintf(stdout, "%ld IMU sensors:\n", sensor_infos.size());
        for(uint32_t i=0; i<sensor_infos.size(); i++) {

        const imu::Info& m = sensor_infos[i];

        fprintf(stdout, "\t%s:\n", m.name.c_str());
        fprintf(stdout, "\t\tdevice:   %s\n", m.device.c_str());
        fprintf(stdout, "\t\tunits :   %s\n", m.units.c_str());
        fprintf(stdout, "\t\trates :   %ld: rate (Hz), bandwidthCutoff (Hz)\n", 
                m.rates.size());
        for(int j=0; j<m.rates.size(); j++)
            fprintf(stdout, "\t\t\t\t%d: %.1f, %.3f\n", j,
                    m.rates[j].sampleRate,
                    m.rates[j].bandwidthCutoff);
        fprintf(stdout, "\t\tranges:   %ld: range (+/- %s), resolution (%s)\n", 
                    m.ranges.size(),
                    m.units.c_str(),
                    m.units.c_str());
        for(int j=0; j<m.ranges.size(); j++)
            fprintf(stdout, "\t\t\t\t%d: %.1f, %.6f\n", j,
                    m.ranges[j].range,
                    m.ranges[j].resolution);
        }

        fprintf(stdout, "\nCurrent IMU configuration:\n");
        fprintf(stdout, "\t-s %d ", sensor_samplesPerMessage);
        for(uint32_t i=0; i<sensor_configs.size(); i++) {
            
            const imu::Config& c = sensor_configs[i];
            
            fprintf(stdout, "-c \"%s %s %d %d\" ", c.name.c_str(), c.enabled ? "true" : "false",
                    c.rateTableIndex, c.rangeTableIndex);
        }
        fprintf(stdout, "\n");
    }

    //
    // Send current configuration

    if (user_samplesPerMessage > 0 || cli_configs.size() > 0) {

        std::vector<imu::Config> user_configs;
        bool                     configValid = true;

        if (user_samplesPerMessage > sensor_maxSamplesPerMessage) {
            fprintf(stderr, "invalid samples-per-message %d, valid values are in [1,%d]\n",
                    user_samplesPerMessage, sensor_maxSamplesPerMessage);
            configValid = false;
        }

        //
        // Validate each command line config option against IMU information from the head

        for(uint32_t i=0; i<cli_configs.size(); i++) {
        
            char nameP[32]    = {0};
            char enabledP[32] = {0};
            int  rate         = -1;
            int range         = -1;

            if (4 != sscanf(cli_configs[i].c_str(), "%31s %31s %d %d",
                            nameP, enabledP, &rate, &range)) {
                fprintf(stderr, "malformed IMU config: \"%s\"\n", 
                        cli_configs[i].c_str());
                configValid = false;
                continue;  // keep parsing for maximum feedback
            }

            if (0 != strcasecmp(enabledP, "true") &&
                0 != strcasecmp(enabledP, "false")) {
                fprintf(stderr, 
                        "malformed <enabled> string \"%s\", must be one of \"true\" or \"false\"\n",
                        enabledP);
                configValid = false;
                continue;
            }

            //
            // Find the IMU information for this name

            imu::Info info;
            if (false == imuInfoByName(nameP, info)) {
                fprintf(stderr, 
                        "unknown <sensor_name> \"%s\", query config for a list of valid names\n",
                        nameP);
                configValid = false;
                continue;
            }

            //
            // Validate the rate/range indices

            if (rate < 0 || rate >= info.rates.size()) {
                fprintf(stderr, 
                        "invalid rate table index %d for \"%s\", valid indices are in [0,%lu]\n",
                        rate, nameP, info.rates.size() - 1);
                configValid = false;
            }
            if (range < 0 || range >= info.ranges.size()) {
                fprintf(stderr,
                        "invalid range table index %d for \"%s\", valid indices are in [0,%lu]\n",
                        range, nameP, info.ranges.size() - 1);
                configValid = false;
            }

            if (false == configValid)
                continue;

            //
            // We have a valid config, store it

            imu::Config c;
            c.name            = std::string(nameP);
            c.enabled         = (0 == strcasecmp(enabledP, "true"));
            c.rateTableIndex  = rate;
            c.rangeTableIndex = range;

            user_configs.push_back(c);
        }

        if (false == configValid)
            fprintf(stderr, "errors exist in configuration, aborting\n");
        else {
            status = channelP->setImuConfig(storeInFlash,
                                            user_samplesPerMessage,
                                            user_configs); // can be empty
            if (Status_Ok != status)
                fprintf(stderr, "failed to set IMU configuration: %s\n",
                        Channel::statusString(status));
            else
                fprintf(stdout, "IMU configuration updated successfully\n");
        }
    }

clean_out:
        
    Channel::Destroy(channelP);
    return 0;
}
