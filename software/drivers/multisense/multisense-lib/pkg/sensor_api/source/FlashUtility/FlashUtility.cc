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

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <getopt.h>

#include <LibMultiSense/MultiSenseChannel.hh>

#include <fstream>

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
}

bool verifyFileWithExtension(const std::string& description,
                             const std::string& fileName,
                             const std::string& extension)
{
    try {
        std::ifstream file(fileName.c_str(),
                           std::ios::in | std::ios::binary);

        if (false == file.good()) {
            fprintf(stderr, "Cannot open %s file \"%s\" for reading, aborting.\n",
                    description.c_str(), fileName.c_str());
            return false;
        }

    } catch (const std::exception& e) {
        fprintf(stderr, "Exception accessing %s file \"%s\" for reading: %s.\n",
                description.c_str(), fileName.c_str(), e.what());
        return false;
    }

    std::string::size_type idx = fileName.rfind('.');

    if (std::string::npos != idx &&
        fileName.substr(idx+1) == extension)
        return true;

    fprintf(stderr, "%s file \"%s\" is not a \".%s\" file, aborting.\n",
            description.c_str(), fileName.c_str(), extension.c_str());

    return false;
}

}; // anonymous

using namespace crl::multisense;

int main(int    argc, 
         char **argvPP)
{
    std::string ipAddress  = "10.66.171.21";
    bool        programOp  = false;
    bool        verifyOp   = false;
    int         returnCode = 0;
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
        default: usage(*argvPP); exit(-1);                break;
        }

    //
    // Verify that the bitstream/firmware filenames are sane, and that the files exist

    if (false == bitstreamFile.empty() && 
        false == verifyFileWithExtension("Bitstream", bitstreamFile, "bin"))
        exit(-2);
    if (false == firmwareFile.empty() && 
        false == verifyFileWithExtension("Firmware", firmwareFile, "srec"))
        exit(-3);

    //
    // Initialize communications.

    Channel *channelP = Channel::Create(ipAddress);
    if (NULL == channelP) {
	fprintf(stderr, "Failed to establish communications with \"%s\"\n",
		ipAddress.c_str());
        exit(-4);
    }
    
    //
    // Perform any programming operations first

    Status status=Status_Ok;

    if (programOp) {

        if (!bitstreamFile.empty()) {
            
            fprintf(stderr, "Programming bitstream: %s\n",
                    bitstreamFile.c_str());

            status = channelP->flashBitstream(bitstreamFile); 
            if (Status_Ok != status) {
                fprintf(stderr, "Programming bitstream failed: %s\n",
                        Channel::statusString(status));
                returnCode = -6; 
                goto clean_out;
            }
        }

        if (!firmwareFile.empty()) {
            
            fprintf(stderr, "Programming firmware: %s\n",
                    firmwareFile.c_str());

            status = channelP->flashFirmware(firmwareFile);
            if (Status_Ok != status) {
                fprintf(stderr, "Programming firmware failed: %s\n",
                        Channel::statusString(status));
                returnCode = -7;
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

            status = channelP->verifyBitstream(bitstreamFile);
            if (Status_Ok != status) {
                fprintf(stderr, "Verify bitstream failed: %s\n",
                        Channel::statusString(status));
                returnCode = -8;
                goto clean_out;
            }
        }

        if (!firmwareFile.empty()) {
            
            fprintf(stderr, "Verifying firmware: %s\n",
                    firmwareFile.c_str());

            status = channelP->verifyFirmware(firmwareFile);
            if (Status_Ok != status) {
                fprintf(stderr, "Verify firmware failed: %s\n",
                        Channel::statusString(status));
                returnCode = -9;
                goto clean_out;
            }
        }
    }

clean_out:

    Channel::Destroy(channelP);
    return returnCode;
}
