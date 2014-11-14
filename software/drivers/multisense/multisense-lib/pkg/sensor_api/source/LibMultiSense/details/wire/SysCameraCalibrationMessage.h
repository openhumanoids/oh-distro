/**
 * @file LibMultiSense/SysCameraCalibrationMessage.h
 *
 * This message contains camera calibration
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
 *   2013-05-23, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_SysCameraCalibrationMessage
#define LibMultiSense_SysCameraCalibrationMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {
    
class CameraCalData {
public:
    static const VersionType VERSION = 1;

    float M[3][3];
    float D[8];
    float R[3][3];
    float P[3][4];

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        SER_ARRAY_2(M, 3, 3);
        SER_ARRAY_1(D, 8);
        SER_ARRAY_2(R, 3, 3);
        SER_ARRAY_2(P, 3, 4);
    };
};

class SysCameraCalibration {
public:
    static const IdType      ID      = ID_DATA_SYS_CAMERA_CAL;
    static const VersionType VERSION = 1;

    //
    // 2 MPix 

    CameraCalData left;
    CameraCalData right;

    //
    // Constructors

    SysCameraCalibration(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysCameraCalibration() {};

    //
    // Serialization routine
    
    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        left.serialize(message, version);
        right.serialize(message, version);
    }
};

}}}}; // namespaces

#endif
