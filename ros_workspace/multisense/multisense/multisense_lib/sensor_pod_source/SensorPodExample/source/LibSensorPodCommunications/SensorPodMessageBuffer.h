/**
 * @file LibSensorPodCommunications/SensorPodMessageBuffer.h
 *
 * A message object is read from or transmitted over the sensor pod
 * communications channel.
 *
 * Copyright 2011
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-10, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_SensorPodMessageBuffer
#define LibSensorPodCommunications_SensorPodMessageBuffer

#include <sstream>
#include <stdint.h>
#include <LibPlatform/TimeStamp.hh>
//
// The message is the object that we transmit around on the network.
//
// Note(dlr): Perhaps we need a different name for this class?
// Classes CamConfigMessage, StatusResponseMessage, etc. already give
// us a clear meaning for "Message."  This is more like a
// MessageBuffer.
//

class SensorPodMessageBuffer
{
private:

    //
    // The sender information (if any). Will be zero
    // if invalid.
    //

    int senderAddress;
    int senderPort;

    //
    // The type of this packet (defined mostly in MessageIdentifiers.h).
    //

    uint8_t type;

    //
    // The raw packet data.
    //

    std::stringstream data;

public:

    //
    // Public API.
    //

    SensorPodMessageBuffer();
    SensorPodMessageBuffer(int senderAddress, int senderPort);

    //
    // Routines to get private variables.
    //

    int getSenderAddress();
    int getSenderPort();

    //
    // Type management.
    //

    void setType(uint8_t type);
    uint8_t getType();
    void confirmType(uint8_t type);

    //
    // Get a reference to the raw data buffer.
    //

    std::stringstream& getDataStream();
    int getSize();

    //
    // Seeking and stream manipulation routines.
    //

    void write(const unsigned char* outputData, unsigned int dataSize);
    void read(unsigned char* inputData, unsigned int dataSize);
    void seekg(int position);

    //
    // Overloads for writing data out.
    //

    template <typename T> SensorPodMessageBuffer& operator<<(const T &value)
    {
        this->write((const unsigned char*)&value, sizeof(T));
        return *this;
    }

    SensorPodMessageBuffer& operator<<(crl::TimeStamp value);

    //
    // Overloads for reading data in.
    //

    template <typename T> SensorPodMessageBuffer& operator>>(T &value)
    {
        this->read((unsigned char*)&value, sizeof(T));
        return *this;
    }

    SensorPodMessageBuffer& operator>>(crl::TimeStamp& value);

};

#endif
