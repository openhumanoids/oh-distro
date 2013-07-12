/**
 * @file SaveImageUtility/SaveImageUtility.cc
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
 *   2013-06-14, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <string>
#include <fstream>

#include <LibMultiSense/MultiSenseChannel.hh>

using namespace crl::multisense;

namespace {  // anonymous

volatile bool doneG = false;

void usage(const char *programNameP) 
{
    fprintf(stderr, "USAGE: %s [<options>]\n", programNameP);
    fprintf(stderr, "Where <options> are:\n");
    fprintf(stderr, "\t-a <current_address>    : CURRENT IPV4 address (default=10.66.171.21)\n");
    
    exit(-1);
}

void signalHandler(int sig)
{
    fprintf(stderr, "Shutting down on signal: %s\n",
            strsignal(sig));
    doneG = true;
}

bool savePgm(const std::string& fileName,
             uint32_t           width,
             uint32_t           height,
             uint32_t           bitsPerPixel,
             const void        *dataP)
{
    std::ofstream outputStream(fileName.c_str(), std::ios::binary | std::ios::out);

    switch(bitsPerPixel) {
    case 8:
    {   
        outputStream << "P5\n"
                     << width << " " << height << "\n"
                     << 0xFF << "\n";
        
        const uint8_t *imageP = (const uint8_t *) dataP;

        for (int i=0; i<height; i++)
            for (int j=0; j<width; j++) {
                uint8_t o = imageP[(i * width) + j];
                outputStream.write((const char *)&o, sizeof(o));
            }

        break;
    }
    case 16:
    {
        outputStream << "P5\n"
                     << width << " " << height << "\n"
                     << 0xFFFF << "\n";

#define swapshort(x) (((x & 0xFF) << 8) | (x >> 8))

        const uint16_t *imageP = (const uint16_t *) dataP;

        for (int i=0; i<height; i++)
            for (int j=0; j<width; j++) {
                uint16_t o = swapshort(imageP[(i * width) + j]);
                outputStream.write((const char *) &o, sizeof(o));
            }

        break;
    }
    }
        
    outputStream.close();
    return true;
}

void laserCallback(const lidar::Header&        header,
                   const lidar::RangeType     *rangesP,
                   const lidar::IntensityType *intensitiesP,
                   void                       *userP)
{
//    fprintf(stderr, "lidar: %d\n", header.pointCount);
}

void imageCallback(const image::Header& header,
                   const void          *dataP,
                   void                *userP)
{
    Channel *channelP = reinterpret_cast<Channel*>(userP);

    /*
    fprintf(stderr, "image: type=0x%x, bpp=%d, w=%d, h=%d frame=%d\n", 
            header.source,
            header.bitsPerPixel,
            header.width,
            header.height,
            header.frameId);
    */

    /*
    savePgm("test.pgm",
            header.width,
            header.height,
            header.bitsPerPixel,
            dataP);
    */

    uint32_t channels, bins;

    const uint32_t *histogramP = channelP->getHistogram(header.frameId, channels, bins);
    if (NULL == histogramP)
        fprintf(stderr, "failed to get histogram for frame %lld\n",
                header.frameId);
    else
        channelP->releaseHistogram(header.frameId);
}

}; // anonymous

using namespace crl::multisense;

int main(int    argc, 
         char **argvPP)
{
    std::string currentAddress = "10.66.171.21";

    signal(SIGINT, signalHandler);

    //
    // Parse args

    int c;

    while(-1 != (c = getopt(argc, argvPP, "a:")))
        switch(c) {
        case 'a': currentAddress = std::string(optarg);    break;
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
    // Query version

    Status status;
    system::VersionInfo v;

    status = channelP->getVersionInfo(v);
    if (Status_Ok != status) {
        fprintf(stderr, "failed to query sensor version: %d\n", status);
        goto clean_out;
    }

    fprintf(stdout, "API build date      :  %s\n", v.apiBuildDate.c_str());
    fprintf(stdout, "API version         :  0x%04x\n", v.apiVersion);
    fprintf(stdout, "Firmware build date :  %s\n", v.sensorFirmwareBuildDate.c_str());
    fprintf(stdout, "Firmware version    :  0x%04x\n", v.sensorFirmwareVersion);
    fprintf(stdout, "Hardware version    :  0x%llx\n", v.sensorHardwareVersion);
    fprintf(stdout, "Hardware magic      :  0x%llx\n", v.sensorHardwareMagic);
    fprintf(stdout, "FPGA DNA            :  0x%llx\n", v.sensorFpgaDna);

    //
    // Change framerate

    {
        image::Config cfg;

        const float FPS = 10.0;

        status = channelP->getImageConfig(cfg);
        if (Status_Ok == status) {

            cfg.setResolution(2048, 1088);

            fprintf(stderr, "Setting framerate to %f FPS (from %f)\n",
                    FPS, cfg.fps());
            cfg.setFps(FPS);
            status = channelP->setImageConfig(cfg);
        }
    }

    //
    // Change MTU

    if (Status_Ok != channelP->setMtu(9000))
        fprintf(stderr, "failed to set MTU to 9000\n");

    //
    // Add image callback

    channelP->addIsolatedCallback(imageCallback, Source_All, channelP);

    channelP->addIsolatedCallback(laserCallback, channelP);

    //
    // Start streaming

//    channelP->startStreams(Source_Disparity);
//    channelP->startStreams(Source_Luma_Left);
//    channelP->startStreams(Source_Chroma_Left);
    channelP->startStreams(Source_All);

    while(!doneG)
        usleep(100000);

    channelP->stopStreams(Source_All);

clean_out:

    Channel::Destroy(channelP);
    return 0;
}
