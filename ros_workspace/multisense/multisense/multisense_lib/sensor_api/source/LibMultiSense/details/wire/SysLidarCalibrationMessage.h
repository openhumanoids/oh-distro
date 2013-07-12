/**
 * @file LibMultiSense/SysLidarCalibrationMessage.h
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

#ifndef LibMultiSense_SysLidarCalibrationMessage
#define LibMultiSense_SysLidarCalibrationMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {
    
class SysLidarCalibration {
public:
    static const IdType      ID      = ID_DATA_SYS_LIDAR_CAL;
    static const VersionType VERSION = 1;

    float laserToSpindle[4][4];
    float cameraToSpindleFixed[4][4];

    //
    // Constructors

    SysLidarCalibration(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysLidarCalibration() {};

    //
    // Serialization routine
    
    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        SER_ARRAY_2(laserToSpindle, 4, 4);
        SER_ARRAY_2(cameraToSpindleFixed, 4, 4);
    }
};

}}}}; // namespaces

#endif
