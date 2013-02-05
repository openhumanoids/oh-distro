/**
 * @file LibSensorPodCommunications/SensorPodCommunications.h
 *
 * Public communications server for the Sensor Pod. This is a simple
 * UDP server for sending/receiving messages.
 *
 * Copyright 2011
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-10, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_SensorPodCommunications
#define LibSensorPodCommunications_SensorPodCommunications

#include "SensorPodMessageBuffer.h"

#include <vector>
#include <fcntl.h>
#include <stdint.h>

//
// Defines the default MTU size for communications channels. This is
// the threshold that we fragment packets on automatically. You can
// change this number using the SensorPodCommunications constructor
// argument (this is useful if you have jumbo frames).
//

// #define SENSOR_POD_MTU_DEFAULT_SIZE                    (7160)
#define SENSOR_POD_MTU_DEFAULT_SIZE                    (1460)

//
// To help debug network issues with the customer, we keep track of
// the frame numbers of the images we send.  This constant determines
// how much history we should remember.
//

#define SENSOR_POD_PUBLICATION_HISTORY_LENGTH  (50)

//
// Defines a function pointer for receiving callbacks.
//

typedef void (*SensorPodCommunicationsCallback)(SensorPodMessageBuffer& message, const void* userData);

//
// Represents a bound listener to this channel.
//

typedef struct _SensorPodCommunicationsListener
{
    //
    // The function pointer for the listener.
    //

    SensorPodCommunicationsCallback callback;

    //
    // The user data associated with the callback.
    //

    const void* userData;

} SensorPodCommunicationsListener;

//
// The defined packet structure header, packed for
// easy transmission.
//

#pragma pack(push,1)

struct SensorPodPacketHeader
{
    //
    // The magic number.
    //

    uint8_t magic;

    //
    // The type of packet (name of packet).
    //

    uint8_t type;

    //
    // The sequence identifier. A unique, incrementing value
    // for all UDP messages.
    //

    uint32_t sequenceIdentifier;

    //
    // Length of this packet (just this packet, not the group
    // of packets).
    //

    uint32_t packetLength;

    //
    // The total number of packets.
    //

    uint16_t packetCount;

    //
    // The packet identifier for this group.
    //

    uint16_t packetIdentifier;

};

#pragma pack(pop)

//
// The main instance of a communications channel.
//

class SensorPodCommunications
{
private:

    //
    // The socket identifier, created once we initialize the server.
    //

    int serverSocket;

    //
    // The port we bind to and connect over.
    //

    int port;

    //
    // This value is increased with every packet we send out. It
    // can be used for clients to reorder packets.
    //

    uint8_t transmitSequenceIdentifier;

    //
    // For receiving messages, we keep this metadata around.
    //

    unsigned int receivingSequenceIdentifier;
    int fragmentsReceived;

    std::stringstream packetData;

    //
    // Mutexes that are held for interacting with the channel.
    //

    pthread_mutex_t lock;
    pthread_mutex_t publishLock;

    //
    // Identifier/class for the dispatcher.
    //

    void *dispatchIdentifier;

    //
    // The list of listeners.
    //

    std::vector<SensorPodCommunicationsListener> listeners;

    //
    // The packet size at which to fragment.
    //

    unsigned int const mtuSize;

    //
    // Temporary buffer for use when reading data from the network.
    //

    char* incomingBuffer;
  
    //
    // Temporary buffer for use when writing data to the network.
    //

    char* outgoingBuffer;
  
    // 
    // Private procedures.
    //

    void configureSocket(int socket);

    //
    // An internal thread (optional).
    //

    static void* dispatchThread(void* userData);

public:

    //
    // Public procedures.
    //

    SensorPodCommunications(
      int port,
      unsigned int mtuSizeArg = SENSOR_POD_MTU_DEFAULT_SIZE);

    ~SensorPodCommunications();

    void bind();
    void createDispatchThread();

    void addDescriptorsToSets(fd_set& readSet, fd_set& errorSet);
    void handleSets(fd_set& readSet, fd_set& errorSet);

    void addListener(SensorPodCommunicationsCallback callback, const void* userData);

    void publish(SensorPodMessageBuffer& message);

};

#endif
