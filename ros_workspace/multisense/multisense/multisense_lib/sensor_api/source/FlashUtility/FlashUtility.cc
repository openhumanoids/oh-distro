/**
 * @file FlashUtility/FlashUtility.cc
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
 *   2013-05-15, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <LibMultiSense/MultiSenseChannel.hh>

namespace {  // anonymous

void usage(const char *programNameP) 
{
    fprintf(stderr, "USAGE: %s <options>\n", programNameP);
    fprintf(stderr, "Where <options> are:\n");
    fprintf(stderr, "\t-a <ip_address>        : IP address of device (default=10.66.171.21)\n");
    fprintf(stderr, "\t-p                     : Perform flash operation\n");
    fprintf(stderr, "\t-v                     : Perform verify operation\n");
    fprintf(stderr, "\t-b <bitstream_file>    : The bitstream (.bin) file\n");
    fprintf(stderr, "\t-f <firmware_file>     : The firmware (.srec) file\n");

    exit(-1);
}

}; // anonymous

using namespace crl::multisense;

int main(int    argc, 
         char **argvPP)
{
    std::string ipAddress = "10.66.171.21";
    bool        programOp = false;
    bool        verifyOp  = false;
    std::string bitstreamFile;
    std::string firmwareFile;

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:epvb:f:")))
        switch(c) {
        case 'a': ipAddress     = std::string(optarg);    break;
        case 'p': programOp     = true;                   break;
        case 'v': verifyOp      = true;                   break;
        case 'b': bitstreamFile = std::string(optarg);    break;
        case 'f': firmwareFile  = std::string(optarg);    break;
        default: usage(*argvPP);                          break;
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

    VersionType version;
        
    if (Status_Ok != channelP->getSensorVersion(version)) {
        fprintf(stderr, "failed to query sensor version\n");
        goto clean_out;
    }
    
    //
    // Perform any programming operations first

    if (programOp) {

        if (!bitstreamFile.empty()) {
            
            fprintf(stderr, "Programming bitstream: %s\n",
                    bitstreamFile.c_str());

            if (Status_Ok != channelP->flashBitstream(bitstreamFile)) {
                fprintf(stderr, "Programming bitstream failed\n");
                goto clean_out;
            }
        }

        if (!firmwareFile.empty()) {
            
            fprintf(stderr, "Programming firmware: %s\n",
                    firmwareFile.c_str());

            if (Status_Ok != channelP->flashFirmware(firmwareFile)) {
                fprintf(stderr, "Programming firmware failed\n");
                goto clean_out;
            }
        }
    }

    //
    // Perform any verification operations

    if (verifyOp) {

        if (!bitstreamFile.empty()) {
            
            fprintf(stderr, "Verifying bitstream: %s\n",
                    bitstreamFile.c_str());

            if (Status_Ok != channelP->verifyBitstream(bitstreamFile)) {
                fprintf(stderr, "Verify bitstream failed\n");
                goto clean_out;
            }
        }

        if (!firmwareFile.empty()) {
            
            fprintf(stderr, "Verifying firmware: %s\n",
                    firmwareFile.c_str());

            if (Status_Ok != channelP->verifyFirmware(firmwareFile)) {
                fprintf(stderr, "Verify firmware failed\n");
                goto clean_out;
            }
        }
    }

clean_out:

    Channel::Destroy(channelP);
    return 0;
}
