/**
 * @file LibMultiSense/MultiSenseTypes.hh
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
 *   2013-05-06, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#ifndef LibMultiSense_MultiSenseTypes_hh
#define LibMultiSense_MultiSenseTypes_hh

#include <stdint.h>

#include <string>
#include <vector>

namespace crl {
namespace multisense {

//
// General types

typedef uint32_t VersionType;
typedef int32_t  Status;

//
// General status codes

static const Status Status_Ok          =  0;
static const Status Status_TimedOut    = -1;
static const Status Status_Error       = -2;
static const Status Status_Failed      = -3;
static const Status Status_Unsupported = -4;
static const Status Status_Unknown     = -5;

//
// Data sources

typedef uint32_t DataSource;

static const DataSource Source_Unknown                = 0;
static const DataSource Source_All                    = 0xffffffff;
static const DataSource Source_Raw_Left               = (1<<0);
static const DataSource Source_Raw_Right              = (1<<1);
static const DataSource Source_Luma_Left              = (1<<2);
static const DataSource Source_Luma_Right             = (1<<3);
static const DataSource Source_Luma_Rectified_Left    = (1<<4);
static const DataSource Source_Luma_Rectified_Right   = (1<<5);
static const DataSource Source_Chroma_Left            = (1<<6);
static const DataSource Source_Chroma_Right           = (1<<7);
static const DataSource Source_Disparity              = (1<<10);
static const DataSource Source_Lidar_Scan             = (1<<24);

namespace image {

//
// Header information common to all image types

class Header {
public:

    DataSource source;
    uint32_t   bitsPerPixel;
    uint32_t   width;
    uint32_t   height;
    int64_t    frameId;
    uint32_t   timeSeconds;        // relative, from device
    uint32_t   timeMicroSeconds;
    
    uint32_t   exposure;  // microseconds
    float      gain;
    float      framesPerSecond;

    Header() 
        : source(Source_Unknown) {};
};

//
// Function pointer for receiving callbacks of image data. Pointers
// to data are no longer valid after the callback returns.

typedef void (*Callback)(const Header& header,
                         const void   *imageDataP,
                         void         *userDataP);

//
// For query/setting camera configuration

class Config {
public:

    //
    // User configurable

    void setResolution        (uint32_t w,
                               uint32_t h) { m_width=w;m_height=h; };
    void setWidth             (uint32_t w) { m_width     = w;      };
    void setHeight            (uint32_t h) { m_height    = h;      };
    void setFps               (float f)    { m_fps       = f;      };

    void setGain              (float g)    { m_gain      = g; };
    void setExposure          (uint32_t e) { m_exposure  = e; }; // microseconds, [10, 5000000]
    void setAutoExposure      (bool e)     { m_aeEnabled = e; };
    void setAutoExposureMax   (uint32_t m) { m_aeMax     = m; }; // microseconds
    void setAutoExposureDecay (uint32_t d) { m_aeDecay   = d; }; // [0, ]
    void setAutoExposureThresh(float t)    { m_aeThresh  = t; }; // [0.0, 1.0]

    void setWhiteBalance          (float r,
                                   float b)    { m_wbRed=r;m_wbBlue=b; }; // [0.25, 4.0]
    void setAutoWhiteBalance      (bool e)     { m_wbEnabled = e;      };
    void setAutoWhiteBalanceDecay (uint32_t d) { m_wbDecay   = d;      }; // [0, ]
    void setAutoWhiteBalanceThresh(float t)    { m_wbThresh  = t;      }; // [0.0, 1.0]

    //
    // Query

    uint32_t width () const { return m_width;  };
    uint32_t height() const { return m_height; };
    float    fps   () const { return m_fps;    };
        
    float    gain              () const { return m_gain;      };
    uint32_t exposure          () const { return m_exposure;  };
    bool     autoExposure      () const { return m_aeEnabled; };
    uint32_t autoExposureMax   () const { return m_aeMax;     };
    uint32_t autoExposureDecay () const { return m_aeDecay;   };
    float    autoExposureThresh() const { return m_aeThresh;  };

    float    whiteBalanceRed       () const { return m_wbRed;     };
    float    whiteBalanceBlue      () const { return m_wbBlue;    };
    bool     autoWhiteBalance      () const { return m_wbEnabled; };
    uint32_t autoWhiteBalanceDecay () const { return m_wbDecay;   };
    float    autoWhiteBalanceThresh() const { return m_wbThresh;  };

    //
    // Query camera calibration (read-only)
    
    float fx()    const { return m_fx;    }; 
    float fy()    const { return m_fy;    };
    float cx()    const { return m_cx;    }; 
    float cy()    const { return m_cy;    };
    float tx()    const { return m_tx;    };
    float ty()    const { return m_ty;    };
    float tz()    const { return m_tz;    };
    float roll()  const { return m_roll;  };
    float pitch() const { return m_pitch; };
    float yaw()   const { return m_yaw;   };

    Config() : m_fps(5.0f), m_gain(1.0f),
               m_exposure(10000), m_aeEnabled(true), m_aeMax(5000000), m_aeDecay(7), m_aeThresh(0.75f),
               m_wbBlue(1.0f), m_wbRed(1.0f), m_wbEnabled(true), m_wbDecay(3), m_wbThresh(0.5f),
               m_width(1024), m_height(544), 
               m_fx(0), m_fy(0), m_cx(0), m_cy(0),
               m_tx(0), m_ty(0), m_tz(0), m_roll(0), m_pitch(0), m_yaw(0) {};
protected:

    float    m_fps, m_gain;
    uint32_t m_exposure;
    bool     m_aeEnabled;
    uint32_t m_aeMax;
    uint32_t m_aeDecay;
    float    m_aeThresh;
    float    m_wbBlue;
    float    m_wbRed;
    bool     m_wbEnabled;
    uint32_t m_wbDecay;
    float    m_wbThresh;
    uint32_t m_width, m_height;
    float    m_fx, m_fy, m_cx, m_cy;
    float    m_tx, m_ty, m_tz;
    float    m_roll, m_pitch, m_yaw;
};

//
// For querying/setting camera calibration

class Calibration {
public:
    
    class Data {
    public:
        
        float M[3][3];
        float D[8];
        float R[3][3];
        float P[3][4];
    };

    Data left;
    Data right;
};

}; // namespace image

namespace lidar {

typedef uint32_t RangeType;
typedef uint32_t IntensityType;

//
// Header information for a lidar scan

class Header {
public:
    
    uint32_t scanId;
    uint32_t timeStartSeconds;
    uint32_t timeStartMicroSeconds;
    uint32_t timeEndSeconds;
    uint32_t timeEndMicroSeconds;
    int32_t  spindleAngleStart; // microradians
    int32_t  spindleAngleEnd;
    int32_t  scanArc;           // microradians
    uint32_t maxRange;          // millimeters
    uint32_t pointCount;

    Header()
        : pointCount(0) {};
};

//
// Function pointer for receiving callbacks of lidar data. Pointers
// to data are no longer valid after the callback returns.

typedef void (*Callback)(const Header&        header,
                         const RangeType     *rangesP,
                         const IntensityType *intensitiesP,
                         void                *userDataP);

class Calibration {
public:

    float laserToSpindle[4][4];
    float cameraToSpindleFixed[4][4];
};

}; // namespace lidar

namespace lighting {

static const uint32_t MAX_LIGHTS     = 8;
static const float    MAX_DUTY_CYCLE = 100.0;

//
// For query/setting lighting configuration

class Config {
public:
    
    //
    // Turn on/off flashing (lights on only when sensor is
    // exposing)

    void setFlash(bool onOff) { m_flashEnabled = onOff; };
    bool getFlash()     const { return m_flashEnabled;  };

    //
    // Set/get a channel's duty cycle in percent

    bool setDutyCycle(float percent) {
        if (percent < 0.0 || percent > MAX_DUTY_CYCLE) 
            return false;

        std::fill(m_dutyCycle.begin(),
                  m_dutyCycle.end(),
                  percent);
        return true;
    };

    bool setDutyCycle(uint32_t i,
                      float    percent) {
        if (i >= MAX_LIGHTS ||
            percent < 0.0 || percent > MAX_DUTY_CYCLE)
            return false;

        m_dutyCycle[i] = percent;
        return true;
    };

    float getDutyCycle(uint32_t i) const { 
        if (i >= MAX_LIGHTS)
            return 0.0f;
        return m_dutyCycle[i];
    };

    //
    // Constructor

    Config() : m_flashEnabled(false), m_dutyCycle(MAX_LIGHTS, -1.0f) {};

private:
        
    bool               m_flashEnabled;
    std::vector<float> m_dutyCycle;
};    

}; // namespace lighting

namespace system {

//
// Device mode

class DeviceMode {
public:

    uint32_t   width;
    uint32_t   height;
    DataSource supportedDataSources;
    uint32_t   flags; //TBD

    DeviceMode(uint32_t   w=0, 
               uint32_t   h=0, 
               DataSource d=0, 
               uint32_t   f=0) :
        width(w),
        height(h),
        supportedDataSources(d),
        flags(f) {};
};

class VersionInfo {
public:    

    std::string apiBuildDate;
    VersionType apiVersion;

    std::string sensorFirmwareBuildDate;
    VersionType sensorFirmwareVersion;
    
    uint64_t    sensorHardwareVersion;
    uint64_t    sensorHardwareMagic;
    uint64_t    sensorFpgaDna;

    VersionInfo() :
        apiVersion(0),
        sensorFirmwareVersion(0),
        sensorHardwareVersion(0),
        sensorHardwareMagic(0),
        sensorFpgaDna(0) {};
};

class PcbInfo {
public:

    std::string name;
    uint32_t    revision;
    
    PcbInfo() : revision(0) {};
};

class DeviceInfo {
public:

    static const uint32_t MAX_PCBS = 8;

    std::string name;
    std::string buildDate;
    std::string serialNumber;
    uint32_t    hardwareRevision;
  
    std::vector<PcbInfo> pcbs;
  
    std::string imagerName;
    uint32_t    imagerType;
    uint32_t    imagerWidth;
    uint32_t    imagerHeight;
    
    std::string lensName;
    uint32_t    lensType;
    float       nominalBaseline;          // meters
    float       nominalFocalLength;       // meters
    float       nominalRelativeAperture;  // f-stop

    uint32_t    lightingType;
    uint32_t    numberOfLights;

    std::string laserName;
    uint32_t    laserType;

    std::string motorName;
    uint32_t    motorType;
    float       motorGearReduction;

    DeviceInfo() :
        hardwareRevision(0),
        imagerType(0),
        imagerWidth(0),
        imagerHeight(0),
        lensType(0),
        nominalBaseline(0.0),
        nominalFocalLength(0.0),
        nominalRelativeAperture(0.0),
        lightingType(0),
        numberOfLights(0),
        laserType(0),
        motorType(0),
        motorGearReduction(0.0) {};
};

class NetworkConfig {
public:

    std::string ipv4Address;
    std::string ipv4Gateway;
    std::string ipv4Netmask;

    NetworkConfig() :
        ipv4Address("10.66.171.21"),
        ipv4Gateway("10.66.171.1"),
        ipv4Netmask("255.255.240.0") {};

    NetworkConfig(const std::string& a,
                  const std::string& g,
                  const std::string& n) :
        ipv4Address(a),
        ipv4Gateway(g),
        ipv4Netmask(n) {};
};


}; // namespace system
}; // namespace multisense
}; // namespace crl

#endif // LibMultiSense_MultiSenseTypes_hh
