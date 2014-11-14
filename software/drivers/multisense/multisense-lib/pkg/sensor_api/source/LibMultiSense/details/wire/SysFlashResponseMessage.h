/**
 * @file LibMultiSense/SysFlashResponseMessage.h
 *
 * This message contains status information about requested or ongoing flash
 * operations.
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
 *   2012-08-03, dstrother@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_SysFlashResponseMessage
#define LibMultiSense_SysFlashResponseMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class SysFlashResponse {
public:
    static const IdType      ID      = ID_DATA_SYS_FLASH_RESPONSE;
    static const VersionType VERSION = 1; 

    //
    // Parameters representing the desired flash operation

    static const uint32_t STATUS_IDLE              = 0;// no operation in progress or requested
    static const uint32_t STATUS_SUCCESS           = 1;// requested operation succeeded/started
    static const uint32_t STATUS_FAILURE           = 2;// requested operation failed
    static const uint32_t STATUS_ERASE_IN_PROGRESS = 3;// operation not possible

    uint32_t status;

    //
    // Only for ERASE_IN_PROGRESS status

    int32_t erase_progress;

     //
    // Constructors

    SysFlashResponse(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysFlashResponse() : status(STATUS_IDLE), erase_progress(0) {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & status;
        message & erase_progress;

        switch(status) {
        case STATUS_IDLE:
        case STATUS_SUCCESS:
        case STATUS_FAILURE:
        case STATUS_ERASE_IN_PROGRESS:
            break;
        default:
            CRL_EXCEPTION("unknown status (%d)", (int)status);
        }
    }
};

}}}}; // namespaces

#endif
