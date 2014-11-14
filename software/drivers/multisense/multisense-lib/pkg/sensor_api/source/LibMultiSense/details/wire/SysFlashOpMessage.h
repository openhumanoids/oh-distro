/**
 * @file LibMultiSense/SysFlashOpMessage.h
 *
 * This message contains information on the number and layout of non-volatile
 * memory devices in the system.
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
 *   2012-08-02, dstrother@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_SysFlashOpMessage
#define LibMultiSense_SysFlashOpMessage

#include <typeinfo>

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class SysFlashOp {
public:
    static const IdType      ID      = ID_CMD_SYS_FLASH_OP;
    static const VersionType VERSION = 1; 

    //
    // Maximum payload length per operation

    static const uint32_t MAX_LENGTH = 1024;

    //
    // Parameters representing the desired flash operation

    static const uint32_t OP_STATUS  = 0; // just check status
    static const uint32_t OP_ERASE   = 1; // erase entire region
    static const uint32_t OP_PROGRAM = 2; // program/verify chunk within region
    static const uint32_t OP_VERIFY  = 3; // just verify chunk within region

    uint32_t operation;

    //
    // Parameters representing the desired flash region

    static const uint32_t RGN_BITSTREAM   = 0; // FPGA configuration bitstream
    static const uint32_t RGN_FIRMWARE    = 1; // Microblaze firmware

    uint32_t region;

    //
    // Remaining fields are only used for OP_PROGRAM and OP_VERIFY:

    uint32_t start_address;  // start address of chunk to program or verify
    uint32_t length;         // size of chunk to program or verify (power-of-2)

    uint8_t data[MAX_LENGTH];

    //
    // Constructors

    SysFlashOp(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysFlashOp(uint32_t op=OP_STATUS, 
               uint32_t r=RGN_BITSTREAM,
               uint32_t s=0,
               uint32_t l=0) : operation(op), 
                               region(r),
                               start_address(s),
                               length(l) {};
    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & operation;
        message & region;

        switch(operation) {
            case OP_PROGRAM:
            case OP_VERIFY:

                message & start_address;
                message & length;

                if(length > MAX_LENGTH)
                    CRL_EXCEPTION("length (%u) exceeds MAX_LENGTH (%u)", 
                                  length, MAX_LENGTH);

                if (typeid(Archive) == typeid(utility::BufferStreamWriter))
                    message.write(data, length);
                else
                    message.read(data, length);

                break;
            case OP_STATUS:
            case OP_ERASE:
                // start/length/data not required
                break;
            default:
                CRL_EXCEPTION("unknown operation (%d)", (int)operation);
        }

        switch(region) {
            case RGN_BITSTREAM:
            case RGN_FIRMWARE:
                break;
            default:
                CRL_EXCEPTION("unknown region (%d)", (int)region);
        }
    }
};

}}}}; // namespaces

#endif
