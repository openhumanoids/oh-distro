/**
 * @file LibSensorPodCommunications/SensorPodMessageBuffer.cc
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

#include <LibPlatform/StandardException.hh>

#include "SensorPodMessageBuffer.h"


/**
 * Constructor. Prepares a new message for transmission.
 */
SensorPodMessageBuffer::SensorPodMessageBuffer() : senderAddress(0),
                                       senderPort(0),
                                       type(0)
{
}

/**
 * Constructor. Prepares a new message for reading, assuming that it
 * originally came from the given sender.
 */
SensorPodMessageBuffer::SensorPodMessageBuffer(int senderAddress, int senderPort) : senderAddress(senderAddress),
                                                                        senderPort(senderPort),
                                                                        type(0)
{
}

/**
 * Returns the address of the client that sent this message. May
 * be zero if no client sent it.
 */
int SensorPodMessageBuffer::getSenderAddress()
{
    return this->senderAddress;
}

/**
 * Returns the port of the client that sent this message. May
 * be zero if no client sent it.
 */
int SensorPodMessageBuffer::getSenderPort()
{
    return this->senderPort;
}

/**
 * Sets the type of this message.
 */
void SensorPodMessageBuffer::setType(uint8_t type)
{
    this->type = type;
}

/**
 * Gets the type of this message.
 */
uint8_t SensorPodMessageBuffer::getType()
{
    return this->type;
}

/**
 * Confirms the type of this message. If it doesn't match
 * the parameter given, then this throws an exception.
 */
void SensorPodMessageBuffer::confirmType(uint8_t type)
{
    if(this->type != type)
        throw crl::StandardException("SensorPod message type mismatch (received 0x%08x, wanted 0x%08x).", this->type, type);
}

/**
 * Returns a reference to our internal data buffer. Only of
 * real use to networking code. You should not need to access
 * it directly.
 */
std::stringstream& SensorPodMessageBuffer::getDataStream()
{
    return this->data;
}

/**
 * Returns the size of this message, in bytes.
 */
int SensorPodMessageBuffer::getSize()
{
    return this->data.str().length();
}

/**
 * Writes the given data blob out (literally) to this message.
 */
void SensorPodMessageBuffer::write(const unsigned char* outputData, unsigned int dataSize)
{
    this->data.write((const char*) outputData, dataSize);
}

/**
 * Reads the given data blob out (literally) from this message.
 */
void SensorPodMessageBuffer::read(unsigned char* inputData, unsigned int dataSize)
{
    this->data.read((char*) inputData, dataSize);
}

/**
 * Sets the position that we will read from next, within this message.
 * Does not adjust the write position.
 */
void SensorPodMessageBuffer::seekg(int position)
{
    this->data.seekg(position);
}

/**
 * Writes a timestamp to the message.
 */
SensorPodMessageBuffer& SensorPodMessageBuffer::operator<<(crl::TimeStamp value)
{
    uint32_t seconds = value.getSeconds();
    uint32_t microseconds = value.getMicroSeconds();

    this->data.write((const char*) &seconds, sizeof(seconds));
    this->data.write((const char*) &microseconds, sizeof(microseconds));

    return *this;
}

/**
 * Reads a timestamp from the message.
 */
SensorPodMessageBuffer& SensorPodMessageBuffer::operator>>(crl::TimeStamp& value)
{
    //
    // Read in seconds and microseconds.
    //

    uint32_t seconds;
    uint32_t microseconds;

    this->data.read((char*) &seconds, sizeof(seconds));
    this->data.read((char*) &microseconds, sizeof(microseconds));

    //
    // Convert to a timeval and set it.
    //

    struct timeval t;

    t.tv_sec = seconds;
    t.tv_usec = microseconds;

    value.set(t);

    return *this;
}
