///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) OMG Plc 2009.
// All rights reserved.  This software is protected by copyright
// law and international treaties.  No part of this software / document
// may be reproduced or distributed in any form or by any means,
// whether transiently or incidentally to some other use of this software,
// without the written permission of the copyright owner.
//
///////////////////////////////////////////////////////////////////////////////
#pragma once

#ifdef WIN32

#ifdef _EXPORTING
   #define CLASS_DECLSPEC    __declspec(dllexport)
#else
   #define CLASS_DECLSPEC    __declspec(dllimport)
#endif // _EXPORTING

#elif defined( __GNUC__ )

#if __GNUC__ < 4
    #error gcc 4 is required.
  #endif
  #define CLASS_DECLSPEC     __attribute__((visibility("default")))

#else

#define CLASS_DECLSPEC

#endif

#include <string>

namespace ViconDataStreamSDK
{
namespace CPP
{

class IStringFactory
{
public:
  virtual char * AllocAndCopyString( const char * i_pSource ) = 0;
  virtual void FreeString( char * i_pString ) = 0;
protected:
  virtual ~IStringFactory() {}
};

class String
{
public:
  // A string which we are not responsible for deallocating
  inline
  String( const char * i_pString = 0 )
  : m_pString( 0 )
  , m_pConstString( i_pString )
  , m_pStringFactory( 0 )
  {
  }

  // A string which we are not responsible for deallocating
  String( const std::string & i_rString )
  : m_pString( 0 )
  , m_pConstString( i_rString.c_str() )
  , m_pStringFactory( 0 )
  {
  }

  // Copy constructor
  inline
  String( const String & i_rString )
  {
    m_pConstString = i_rString.m_pConstString;
    m_pStringFactory = i_rString.m_pStringFactory;
    if( m_pStringFactory )
    {
      m_pString = m_pStringFactory->AllocAndCopyString( i_rString.m_pString );
    }
    else
    {
      m_pString = 0;
    }
  }

  inline
  ~String()
  {
    if( m_pStringFactory )
    {
      m_pStringFactory->FreeString( m_pString );
    }
  }

  // A string which we are responsible for deallocating
  inline
  void Set( const char * i_pString, IStringFactory & i_rStringFactory )
  {
    m_pString = i_rStringFactory.AllocAndCopyString( i_pString );
    m_pStringFactory = &i_rStringFactory;
    m_pConstString = 0;
  }

  inline
  operator std::string() const
  {
    if( m_pStringFactory )
    {
      return std::string( m_pString );
    }
    else
    {
      return std::string( m_pConstString );
    }
  }

private:
        char     * m_pString;
  const char     * m_pConstString;
  IStringFactory * m_pStringFactory;
};

// Streaming operator for String
inline std::ostream & operator<<( std::ostream & io_rStream, const String & i_rString )
{
  io_rStream << std::string( i_rString );
  return io_rStream;
}

namespace Direction
{
  enum Enum
  {
    Up,
    Down,
    Left,
    Right,
    Forward,
    Backward
  };
}

namespace StreamMode
{
  enum Enum
  {
    ClientPull,
    ClientPullPreFetch,
    ServerPush
  };
}

namespace TimecodeStandard
{
  enum Enum
  {
    None,
    PAL,
    NTSC,
    NTSCDrop,
    Film
  };
}

namespace DeviceType
{
  enum Enum
  {
    Unknown,
    ForcePlate
  };
}

namespace Unit
{
  enum Enum
  {
    Unknown,
    Volt,
    Newton,
    NewtonMeter,
    Meter
  };
}

namespace Result
{
  enum Enum
  {
    Unknown,
    NotImplemented,
    Success,
    InvalidHostName,
    InvalidMulticastIP,
    ClientAlreadyConnected,
    ClientConnectionFailed,
    ServerAlreadyTransmittingMulticast,
    ServerNotTransmittingMulticast,
    NotConnected,
    NoFrame,
    InvalidIndex,
    InvalidSubjectName,
    InvalidSegmentName,
    InvalidMarkerName,
    InvalidDeviceName,
    InvalidDeviceOutputName,
    InvalidLatencySampleName,
    CoLinearAxes,
    LeftHandedAxes
  };
}

  class Output_GetVersion
  {
  public:
    unsigned int Major;
    unsigned int Minor;
    unsigned int Point;
  };

  class Output_Connect
  {
  public:
    Result::Enum Result;
  };

  class Output_ConnectToMulticast
  {
  public:
    Result::Enum Result;
  };

  class Output_Disconnect
  {
  public:
    Result::Enum Result;
  };

  class Output_IsConnected
  {
  public:
    bool Connected;
  };

  class Output_StartTransmittingMulticast
  {
  public:
    Result::Enum Result;
  };

  class Output_StopTransmittingMulticast
  {
  public:
    Result::Enum Result;
  };

  class Output_EnableSegmentData
  {
  public:
    Result::Enum Result;
  };

  class Output_EnableMarkerData
  {
  public:
    Result::Enum Result;
  };

  class Output_EnableUnlabeledMarkerData
  {
  public:
    Result::Enum Result;
  };

  class Output_EnableDeviceData
  {
  public:
    Result::Enum Result;
  };

  class Output_DisableSegmentData
  {
  public:
    Result::Enum Result;
  };

  class Output_DisableMarkerData
  {
  public:
    Result::Enum Result;
  };

  class Output_DisableUnlabeledMarkerData
  {
  public:
    Result::Enum Result;
  };

  class Output_DisableDeviceData
  {
  public:
    Result::Enum Result;
  };

  class Output_IsSegmentDataEnabled
  {
  public:
    bool Enabled;
  };

  class Output_IsMarkerDataEnabled
  {
  public:
    bool Enabled;
  };

  class Output_IsUnlabeledMarkerDataEnabled
  {
  public:
    bool Enabled;
  };

  class Output_IsDeviceDataEnabled
  {
  public:
    bool Enabled;
  };

  class Output_SetStreamMode
  {
  public:
    Result::Enum Result;
  };

  class Output_SetAxisMapping
  {
  public:
    Result::Enum Result;
  };

  class Output_GetAxisMapping
  {
  public:
    Direction::Enum XAxis;
    Direction::Enum YAxis;
    Direction::Enum ZAxis;
  };

  class Output_GetFrame
  {
  public:
    Result::Enum Result;
  };

  class Output_GetFrameNumber
  {
  public:
    Result::Enum Result;
    unsigned int FrameNumber;
  };

  class Output_GetTimecode
  {
  public:
    Result::Enum           Result;
    unsigned int           Hours;
    unsigned int           Minutes;
    unsigned int           Seconds;
    unsigned int           Frames;
    unsigned int           SubFrame;
    bool                   FieldFlag;
    TimecodeStandard::Enum Standard;
    unsigned int           SubFramesPerFrame;
    unsigned int           UserBits;
  };

  class Output_GetLatencySampleCount
  {
  public:
    Result::Enum Result;
    unsigned int Count;
  };

  class Output_GetLatencySampleName
  {
  public:
    Result::Enum Result;
    String       Name;
  };

  class Output_GetLatencySampleValue
  {
  public:
    Result::Enum Result;
    double       Value;
  };

  class Output_GetLatencyTotal
  {
  public:
    Result::Enum Result;
    double       Total;
  };

  class Output_GetSubjectCount
  {
  public:
    Result::Enum Result;
    unsigned int SubjectCount;
  };

  class Output_GetSubjectName
  {
  public:
    Result::Enum Result;
    String       SubjectName;
  };

  class Output_GetSubjectRootSegmentName
  {
  public:
    Result::Enum Result;
    String       SegmentName;
  };

  class Output_GetSegmentChildCount
  {
  public:
    Result::Enum Result;
    unsigned int SegmentCount;
  };

  class Output_GetSegmentChildName
  {
  public:
    Result::Enum Result;
    String       SegmentName;
  };

  class Output_GetSegmentParentName
  {
  public:
    Result::Enum Result;
    String       SegmentName;
  };

  class Output_GetSegmentCount
  {
  public:
    Result::Enum Result;
    unsigned int SegmentCount;
  };

  class Output_GetSegmentName
  {
  public:
    Result::Enum Result;
    String       SegmentName;
  };

  class Output_GetSegmentStaticTranslation
  {
  public:
    Result::Enum Result;
    double       Translation[ 3 ];
  };

  class Output_GetSegmentStaticRotationHelical
  {
  public:
    Result::Enum Result;
    double       Rotation[ 3 ];
  };

  class Output_GetSegmentStaticRotationMatrix
  {
  public:
    Result::Enum Result;
    double       Rotation[ 9 ];
  };

  class Output_GetSegmentStaticRotationQuaternion
  {
  public:
    Result::Enum Result;
    double       Rotation[ 4 ];
  };

  class Output_GetSegmentStaticRotationEulerXYZ
  {
  public:
    Result::Enum Result;
    double       Rotation[ 3 ];
  };

  class Output_GetSegmentGlobalTranslation
  {
  public:
    Result::Enum Result;
    double       Translation[ 3 ];
    bool         Occluded;
  };

  class Output_GetSegmentGlobalRotationHelical
  {
  public:
    Result::Enum Result;
    double       Rotation[ 3 ];
    bool         Occluded;
  };

  class Output_GetSegmentGlobalRotationMatrix
  {
  public:
    Result::Enum Result;
    double       Rotation[ 9 ];
    bool         Occluded;
  };

  class Output_GetSegmentGlobalRotationQuaternion
  {
  public:
    Result::Enum Result;
    double       Rotation[ 4 ];
    bool         Occluded;
  };

  class Output_GetSegmentGlobalRotationEulerXYZ
  {
  public:
    Result::Enum Result;
    double       Rotation[ 3 ];
    bool         Occluded;
  };

  class Output_GetSegmentLocalTranslation
  {
  public:
    Result::Enum Result;
    double       Translation[ 3 ];
    bool         Occluded;
  };

  class Output_GetSegmentLocalRotationHelical
  {
  public:
    Result::Enum Result;
    double       Rotation[ 3 ];
    bool         Occluded;
  };

  class Output_GetSegmentLocalRotationMatrix
  {
  public:
    Result::Enum Result;
    double       Rotation[ 9 ];
    bool         Occluded;
  };

  class Output_GetSegmentLocalRotationQuaternion
  {
  public:
    Result::Enum Result;
    double       Rotation[ 4 ];
    bool         Occluded;
  };

  class Output_GetSegmentLocalRotationEulerXYZ
  {
  public:
    Result::Enum Result;
    double       Rotation[ 3 ];
    bool         Occluded;
  };

  class Output_GetMarkerCount
  {
  public:
    Result::Enum Result;
    unsigned int MarkerCount;
  };

  class Output_GetMarkerName
  {
  public:
    Result::Enum Result;
    String       MarkerName;
  };

  class Output_GetMarkerParentName
  {
  public:
    Result::Enum Result;
    String       SegmentName;
  };

  class Output_GetMarkerGlobalTranslation
  {
  public:
    Result::Enum Result;
    double       Translation[ 3 ];
    bool         Occluded;
  };

  class Output_GetUnlabeledMarkerCount
  {
  public:
    Result::Enum Result;
    unsigned int MarkerCount;
  };

  class Output_GetUnlabeledMarkerGlobalTranslation
  {
  public:
    Result::Enum Result;
    double       Translation[ 3 ];
  };

  class Output_GetDeviceCount
  {
  public:
    Result::Enum Result;
    unsigned int DeviceCount;
  };

  class Output_GetDeviceName
  {
  public:
    Result::Enum     Result;
    String           DeviceName;
    DeviceType::Enum DeviceType;
  };

  class Output_GetDeviceOutputCount
  {
  public:
    Result::Enum Result;
    unsigned int DeviceOutputCount;
  };

  class Output_GetDeviceOutputName
  {
  public:
    Result::Enum Result;
    String       DeviceOutputName;
    Unit::Enum   DeviceOutputUnit;
  };

  class Output_GetDeviceOutputValue
  {
  public:
    Result::Enum Result;
    double       Value;
    bool         Occluded;
  };

  class Output_GetForcePlateCount
  {
  public:
    Result::Enum Result;
    unsigned int ForcePlateCount;
  };

  class Output_GetGlobalForceVector
  {
  public:
    Result::Enum Result;
    double       ForceVector[ 3 ];
  };

  class Output_GetGlobalMomentVector
  {
  public:
    Result::Enum Result;
    double       MomentVector[ 3 ];
  };

  class Output_GetGlobalCentreOfPressure
  {
  public:
    Result::Enum Result;
    double       CentreOfPressure[ 3 ];
  };

  class ClientImpl;

  class CLASS_DECLSPEC Client
  {
  public:
    Client();
    ~Client();

    Output_GetVersion  GetVersion() const;

    Output_Connect     Connect( const String & HostName );
    Output_ConnectToMulticast ConnectToMulticast( const String & HostName, const String & MulticastIP );
    Output_Disconnect  Disconnect();
    Output_IsConnected IsConnected() const;
    Output_StartTransmittingMulticast StartTransmittingMulticast( const String & ServerIP,
                                                                  const String & MulticastIP );

    Output_StopTransmittingMulticast StopTransmittingMulticast();

    Output_EnableSegmentData         EnableSegmentData();
    Output_EnableMarkerData          EnableMarkerData();
    Output_EnableUnlabeledMarkerData EnableUnlabeledMarkerData();
    Output_EnableDeviceData          EnableDeviceData();

    Output_DisableSegmentData         DisableSegmentData();
    Output_DisableMarkerData          DisableMarkerData();
    Output_DisableUnlabeledMarkerData DisableUnlabeledMarkerData();
    Output_DisableDeviceData          DisableDeviceData();

    Output_IsSegmentDataEnabled         IsSegmentDataEnabled() const;
    Output_IsMarkerDataEnabled          IsMarkerDataEnabled() const;
    Output_IsUnlabeledMarkerDataEnabled IsUnlabeledMarkerDataEnabled() const;
    Output_IsDeviceDataEnabled          IsDeviceDataEnabled() const;
    
    Output_SetStreamMode SetStreamMode( const StreamMode::Enum Mode );

    Output_SetAxisMapping SetAxisMapping( const Direction::Enum XAxis, const Direction::Enum YAxis, const Direction::Enum ZAxis );
    Output_GetAxisMapping GetAxisMapping() const;

    Output_GetFrame GetFrame();
    Output_GetFrameNumber GetFrameNumber() const;

    Output_GetTimecode GetTimecode() const;

    Output_GetLatencySampleCount GetLatencySampleCount() const;
    Output_GetLatencySampleName  GetLatencySampleName( const unsigned int LatencySampleIndex ) const;
    Output_GetLatencySampleValue GetLatencySampleValue( const String & LatencySampleName ) const;
    Output_GetLatencyTotal       GetLatencyTotal() const;

    Output_GetSubjectCount GetSubjectCount() const;
    Output_GetSubjectName GetSubjectName( const unsigned int SubjectIndex ) const;

    Output_GetSubjectRootSegmentName GetSubjectRootSegmentName( const String & SubjectName ) const;

    Output_GetSegmentCount GetSegmentCount( const String  & SubjectName ) const;
    
    Output_GetSegmentName GetSegmentName( const String       & SubjectName,
                                          const unsigned int   SegmentIndex ) const;

    Output_GetSegmentChildCount GetSegmentChildCount( const String & SubjectName,
                                                      const String & SegmentName ) const;

    Output_GetSegmentChildName GetSegmentChildName( const String       & SubjectName,
                                                    const String       & SegmentName,
                                                    const unsigned int   SegmentIndex ) const;

    Output_GetSegmentParentName GetSegmentParentName( const String & SubjectName,
                                                      const String & SegmentName ) const;

    Output_GetSegmentStaticTranslation GetSegmentStaticTranslation( const String & SubjectName,
                                                                    const String & SegmentName ) const;

    Output_GetSegmentStaticRotationHelical GetSegmentStaticRotationHelical( const String & SubjectName,
                                                                            const String & SegmentName ) const;

    Output_GetSegmentStaticRotationMatrix GetSegmentStaticRotationMatrix( const String & SubjectName,
                                                                          const String & SegmentName ) const;

    Output_GetSegmentStaticRotationQuaternion GetSegmentStaticRotationQuaternion( const String & SubjectName,
                                                                                  const String & SegmentName ) const;

    Output_GetSegmentStaticRotationEulerXYZ GetSegmentStaticRotationEulerXYZ( const String & SubjectName,
                                                                              const String & SegmentName ) const;

    Output_GetSegmentGlobalTranslation GetSegmentGlobalTranslation( const String & SubjectName,
                                                                    const String & SegmentName ) const;

    Output_GetSegmentGlobalRotationHelical GetSegmentGlobalRotationHelical( const String & SubjectName,
                                                                            const String & SegmentName ) const;

    Output_GetSegmentGlobalRotationMatrix GetSegmentGlobalRotationMatrix( const String & SubjectName,
                                                                          const String & SegmentName ) const;

    Output_GetSegmentGlobalRotationQuaternion GetSegmentGlobalRotationQuaternion( const String & SubjectName,
                                                                                  const String & SegmentName ) const;

    Output_GetSegmentGlobalRotationEulerXYZ GetSegmentGlobalRotationEulerXYZ( const String & SubjectName,
                                                                              const String & SegmentName ) const;

    Output_GetSegmentLocalTranslation GetSegmentLocalTranslation( const String & SubjectName,
                                                                  const String & SegmentName ) const;

    Output_GetSegmentLocalRotationHelical GetSegmentLocalRotationHelical( const String & SubjectName,
                                                                          const String & SegmentName ) const;

    Output_GetSegmentLocalRotationMatrix GetSegmentLocalRotationMatrix( const String & SubjectName,
                                                                        const String & SegmentName ) const;

    Output_GetSegmentLocalRotationQuaternion GetSegmentLocalRotationQuaternion( const String & SubjectName,
                                                                                const String & SegmentName ) const;

    Output_GetSegmentLocalRotationEulerXYZ GetSegmentLocalRotationEulerXYZ( const String & SubjectName,
                                                                            const String & SegmentName ) const;

    Output_GetMarkerCount GetMarkerCount( const String  & SubjectName ) const;

    Output_GetMarkerName GetMarkerName( const String & SubjectName,
                                        const unsigned int  MarkerIndex ) const;

    Output_GetMarkerParentName GetMarkerParentName( const String & SubjectName,
                                                    const String & MarkerName ) const;

    Output_GetMarkerGlobalTranslation GetMarkerGlobalTranslation( const String & SubjectName,
                                                                  const String & MarkerName ) const;


    Output_GetUnlabeledMarkerCount GetUnlabeledMarkerCount() const;
    
    Output_GetUnlabeledMarkerGlobalTranslation GetUnlabeledMarkerGlobalTranslation( const unsigned int MarkerIndex ) const;

    Output_GetDeviceCount GetDeviceCount() const;
    Output_GetDeviceName  GetDeviceName( const unsigned int DeviceIndex ) const;

    Output_GetDeviceOutputCount GetDeviceOutputCount( const String  & DeviceName ) const;

    Output_GetDeviceOutputName GetDeviceOutputName( const String  & DeviceName,
                                                    const unsigned int   DeviceOutputIndex ) const;
    
    Output_GetDeviceOutputValue GetDeviceOutputValue( const String & DeviceName,
                                                      const String & DeviceOutputName ) const;

    Output_GetForcePlateCount GetForcePlateCount() const;

    Output_GetGlobalForceVector GetGlobalForceVector( const unsigned int ForcePlateIndex ) const;
    Output_GetGlobalMomentVector GetGlobalMomentVector( const unsigned int ForcePlateIndex ) const;
    Output_GetGlobalCentreOfPressure GetGlobalCentreOfPressure( const unsigned int ForcePlateIndex ) const;

  private:
    ClientImpl * m_pClientImpl;
  };
} // End of namespace CPP
} // End of namespace ViconDataStreamSDK
