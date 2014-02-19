/**
 * @file LibMultiSense/details/flash.cc
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
 * License along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Significant history (date, user, job code, action):
 *   2013-05-15, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#include "details/channel.hh"
#include "details/query.hh"

#include "details/wire/AckMessage.h"
#include "details/wire/SysFlashOpMessage.h"
#include "details/wire/SysFlashResponseMessage.h"

namespace crl {
namespace multisense {
namespace details {

//
// Erase a flash region

void impl::eraseFlashRegion(uint32_t region)
{
    wire::SysFlashResponse response;

    //
    // Start the erase operation

    Status status = waitData(wire::SysFlashOp(wire::SysFlashOp::OP_ERASE, region),
                             response);
    if (Status_Ok != status)
        CRL_EXCEPTION("OP_ERASE failed: %d", status);

    //
    // Check for success, or flash in progress

    switch(response.status) {
    case wire::SysFlashResponse::STATUS_SUCCESS:
    case wire::SysFlashResponse::STATUS_ERASE_IN_PROGRESS:
        break; // ok, erase is happening
    default:
        CRL_EXCEPTION("OP_ERASE ack'd, but failed: %d\n", response.status);
    }

    //
    // Wait for the erase to complete

    const double ERASE_TIMEOUT = 210.0; // seconds

    utility::TimeStamp start = utility::TimeStamp::getCurrentTime();

    int prevProgress = -1;

    while((utility::TimeStamp::getCurrentTime() - start) < ERASE_TIMEOUT) {

        //
        // Request current progress

        status = waitData(wire::SysFlashOp(), response);
        if (Status_Ok != status)
            CRL_EXCEPTION("failed to request flash erase status");

        //
        // IDLE means the flash has been erased

        if (wire::SysFlashResponse::STATUS_IDLE == response.status)
            return; // success

        //
        // Prompt and delay a bit

        if (response.erase_progress != prevProgress &&
            0 == (response.erase_progress % 5))
            CRL_DEBUG("erasing... %3d%%\n", response.erase_progress);
        usleep(100000);

        prevProgress = response.erase_progress;
    }

    CRL_EXCEPTION("erase op timed out after %.0f seconds", ERASE_TIMEOUT);
}

//
// Program or verify a flash region from a file

void impl::programOrVerifyFlashRegion(std::ifstream& file,
                                      uint32_t       operation,
                                      uint32_t       region)
{
    //
    // Get file size

    file.seekg(0, file.end);
    int fileLength = file.tellg();
    file.seekg(0, file.beg);

    wire::SysFlashOp op(operation, region, 0,
                        wire::SysFlashOp::MAX_LENGTH);

    int prevProgress = -1;

    const char *opNameP;

    switch(operation) {
    case wire::SysFlashOp::OP_PROGRAM: opNameP = "programming"; break;
    case wire::SysFlashOp::OP_VERIFY:  opNameP = "verifying";   break;
    default: 
        CRL_EXCEPTION("unknown operation type: %d", operation);
    }

    do {

        //
        // Initialize data and read next chunk

        memset(op.data, 0xFF, op.length);
        file.read((char *) op.data, op.length);

        //
        // Send command, await response

        wire::SysFlashResponse rsp;

        Status status = waitData(op, rsp, 0.5, 4);
        if (Status_Ok != status)
            CRL_EXCEPTION("SysFlashOp (%s) failed: %d", opNameP, status);
        else if (wire::SysFlashResponse::STATUS_SUCCESS != rsp.status)
            CRL_EXCEPTION("%s failed @ %d/%d bytes", opNameP,
                          file.tellg(), fileLength);

        //
        // Print out progress

        int progress = (100 * op.start_address) / fileLength;
        if (progress != prevProgress && 0 == (progress % 5))
            CRL_DEBUG("%s... %3d%%\n", opNameP, progress);

        //
        // Update state

        prevProgress = progress;
        op.start_address += op.length;

    } while (!file.eof());

    if ((int) op.start_address < fileLength)
        CRL_EXCEPTION("unexpected EOF while %s", opNameP);

    CRL_DEBUG("%s complete\n", opNameP);
}

//
// Wrapper for all flash operations

Status impl::doFlashOp(const std::string& filename,
                       uint32_t           operation,
                       uint32_t           region)
{
    try {
        std::ifstream file(filename.c_str(), 
                           std::ios::in | std::ios::binary);

        if (!file.good()) 
            CRL_EXCEPTION("unable to open file: \"%s\"", 
                          filename.c_str());

        if (wire::SysFlashOp::OP_PROGRAM == operation)
            eraseFlashRegion(region);

        programOrVerifyFlashRegion(file, operation, region);

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }

    return Status_Ok;
}    

}}}; // namespaces
