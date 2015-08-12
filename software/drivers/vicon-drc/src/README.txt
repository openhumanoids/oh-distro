Vicon DataStream SDK
Copyright 2009 OMG Plc
www.vicon.com/support

Change history:

v1.2.0 - September 2010
What’s New in Version 1.2.0
* New function calls
   GetForcePlateCount()
   GetGlobalForceVector()
   GetGlobalCentreOfPressureVector()

v1.1.0 - December 2009
* Release of C++ and .NET SDKs on Windows x64
* Release of C++ SDK on Linux x86
* New function calls
   DisableSegmentData()
   DisableMarkerData()
   DisableUnlabeledMarkerData()
   DisableDeviceData()
   GetMarkerParentName()
   GetSubjectRootSegmentName()
   GetSegmentParentName()
   GetSegmentChildCount()
   GetSegmentChildName()
   GetSegmentStaticTranslation()
   GetSegmentStaticRotationHelical()
   GetSegmentStaticRotationMatrix()
   GetSegmentStaticRotationQuaternion()
   GetSegmentStaticRotationEulerXYZ()
   
* Corrected some units. The values given by the SDK have not changed - they were incorrectly labeled
   "NewtonMillimetre" has become "NewtonMeter"
   "Millimetre" has become "Meter"

* Corrected segment rotations following calls to SetAxisMapping()   

* Added command-line options for the Test programs to specify a host to connect to  
   
v1.0.1 - July 2009
* Introduced a new String class into the C++ SDK. The previous use of std::string was not safe when building Debug STL-based programs agaisnt the Release STL-based SDK.
* Included documentation in the main installer.
* Installer now upgrades old versions automatically if present.

v1.0.0 - June 2009
* Initial release of C++, .NET, and MATLAB SDKs on Windows x86
