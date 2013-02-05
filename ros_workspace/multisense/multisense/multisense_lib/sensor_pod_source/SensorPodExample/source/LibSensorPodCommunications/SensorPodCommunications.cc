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

#include <LibThread/StandardThread.hh>

#include "SensorPodCommunications.h"


#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <netinet/ip.h>
#include <sys/socket.h>
#include <memory.h>
#include <stdio.h>
#include <errno.h>

/**
 * Constructor. Initializes a communications channel. You still must
 * call bind() to actually begin listening for connections.
 *
 * \param port The port to connect to.
 */
SensorPodCommunications::SensorPodCommunications(
    int port,
    unsigned int mtuSizeArg)
    : serverSocket(-1),
      port(port),
      transmitSequenceIdentifier(1),
      receivingSequenceIdentifier(0),
      fragmentsReceived(0),
      lock(),
      dispatchIdentifier(NULL),
      listeners(),
      mtuSize(mtuSizeArg),
      incomingBuffer(new char[mtuSizeArg]),
      outgoingBuffer(new char[mtuSizeArg])
{
    //
    // Create the locking mutex.
    //

    int result = pthread_mutex_init(&this->lock, NULL);

    if(result)
        CRL_STANDARD_EXCEPTION("Failed to create the channel lock.");

    result = pthread_mutex_init(&this->publishLock, NULL);

    if(result)
        CRL_STANDARD_EXCEPTION("Failed to create the publish lock.");

}

/**
 * Destructor.
 */
SensorPodCommunications::~SensorPodCommunications()
{
    if(this->serverSocket >= 0)
        close(this->serverSocket);
    if(this->incomingBuffer) {
        delete[] this->incomingBuffer;
    }
    if(this->outgoingBuffer) {
        delete[] this->outgoingBuffer;
    }
}

/**
 * Adds a new listener to the system. This listener will be invoked whenever
 * a new message is received from a remote client.
 */
void SensorPodCommunications::addListener(SensorPodCommunicationsCallback callback, const void* userData)
{
    SensorPodCommunicationsListener listener;

    listener.callback = callback;
    listener.userData = userData;

    pthread_mutex_lock(&this->lock);

    this->listeners.push_back(listener);

    pthread_mutex_unlock(&this->lock);
}

/**
 * Binds the communications channel, preparing it to send/receive data
 * over the network.
 */
void SensorPodCommunications::bind()
{
    //
    // Create the socket.
    //

    this->serverSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if(this->serverSocket < 0)
        CRL_STANDARD_EXCEPTION("Failed to create the UDP socket.");

    //
    // Configure the basic socket settings.
    //

    this->configureSocket(this->serverSocket);

    //
    // Bind the connection to the port.
    //

    struct sockaddr_in address;

    address.sin_family = AF_INET;
    address.sin_port = htons(this->port);
    address.sin_addr.s_addr = htonl(INADDR_ANY);

    int result = ::bind(this->serverSocket, (struct sockaddr*) &address, sizeof(address));

    if(result != 0)
    {
        CRL_STANDARD_EXCEPTION(
          "Failed to bind a sensor pod server socket to port %d.", this->port);
    }
}

/**
 * Creates the standard dispatch thread. This thread will handle incoming messages
 * and redirect them to appropriate listeners.
 */
void SensorPodCommunications::createDispatchThread()
{
    //
    // Sanity check.
    //

    if(this->dispatchIdentifier)
        CRL_STANDARD_EXCEPTION("Tried to create two dispatch threads on the same communications interface.");

    //
    // Create the new thread.
    //

    crl::StandardThread *tP = new crl::StandardThread(SensorPodCommunications::dispatchThread, this);

    this->dispatchIdentifier = (void *) tP;
}

/**
 * Configures the given socket, turning on non-blocking and setting increased
 * buffer sizes.
 */
void SensorPodCommunications::configureSocket(int socket)
{
    //
    // Turn non-blocking on.
    //

    int flags = fcntl(socket, F_GETFL, 0);

    int result = fcntl(socket, F_SETFL, flags | O_NONBLOCK);

    if(result != 0)
        CRL_STANDARD_EXCEPTION("Failed to turn non-blocking on a socket.");

    //
    // Allow reusing sockets.
    //

    int reuseSocket = 1;

    result = setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, (void*) &reuseSocket, sizeof(reuseSocket));

    if(result != 0)
        CRL_STANDARD_EXCEPTION("Failed to turn on socket reuse.");

    //
    // Set decent buffer sizes. We'll use 4MB for all of them.
    //

    int bufferSize = 48 * 1024 * 1024;

    result = setsockopt(socket, SOL_SOCKET, SO_RCVBUF, (void*) &bufferSize, sizeof(bufferSize));

    if(result == 0)
        result = setsockopt(socket, SOL_SOCKET, SO_SNDBUF, (void*) &bufferSize, sizeof(bufferSize));

    if(result != 0)
        CRL_STANDARD_EXCEPTION("Failed to adjust socket buffer sizes.");
}

/**
 * Adds the descriptors of this channel and all of its children to the given
 * sets.
 */
void SensorPodCommunications::addDescriptorsToSets(fd_set& readSet, fd_set& errorSet)
{
    pthread_mutex_lock(&this->lock);

    //
    // Add the server socket to the read set.
    //

    FD_SET(this->serverSocket, &readSet);

    pthread_mutex_unlock(&this->lock);
}

/**
 * Handles the given sets. Any descriptor of ours that is set in the read set
 * will be handled.
 */
void SensorPodCommunications::handleSets(fd_set& readSet, fd_set& errorSet)
{
    //
    // Check to see if the read bit is set. If so, we know we have data that
    // we need to read. We read packets until we get an error here.
    //

    pthread_mutex_lock(&this->lock);

    try {
        if(FD_ISSET(this->serverSocket, &readSet))
        {
            for(;;)
            {
                struct sockaddr_in senderAddress;
                socklen_t senderAddressLength = sizeof(senderAddress);

                //
                // Receive the packet. We sized things so that we can receive
                // up to Jumbo frames.
                //

                int bytesRead = recvfrom (
                    this->serverSocket,
                    this->incomingBuffer,
                    this->mtuSize,
                    0,
                    (struct sockaddr*) &senderAddress,
                    &senderAddressLength
                    );

                //
                // As soon as we get an error, break out of the for() loop.
                //

                if(bytesRead < 0)
                    break;

                //
                // Let's pull out the header and make sense of it. We do a quick sanity
                // check on packet length.
                //

                SensorPodPacketHeader *header =
                    (SensorPodPacketHeader*) this->incomingBuffer;

                if(bytesRead < (int) sizeof(SensorPodPacketHeader))
                    continue;

                //
                // See if this is part of a new incoming packet group. If so, reset
                // some of our internal metrics.
                //

                if(header->sequenceIdentifier != this->receivingSequenceIdentifier)
                {
                    this->receivingSequenceIdentifier = header->sequenceIdentifier;
                    this->fragmentsReceived = 0;

                    //
                    // We need to make sure the string stream object can contain at least
                    // the packet length. Write empty data into it until we can fit the
                    // entire packet. This is really terrible.
                    //

                    while(this->packetData.str().size() < header->packetLength)
                    {
                        this->packetData.write(this->incomingBuffer, this->mtuSize);
                    }

                    this->packetData.clear();
                }

                //
                // Copy this fragment into the array at an appropriate location.
                // We are very dependant on MTU size here, otherwise, all of our
                // assumptions are violated and this will probably error out.
                //

                this->packetData.seekp(header->packetIdentifier * (this->mtuSize - sizeof(SensorPodPacketHeader)));
                this->packetData.write((const char*) &(this->incomingBuffer)[sizeof(SensorPodPacketHeader)], bytesRead - sizeof(SensorPodPacketHeader));

                this->fragmentsReceived++;

                //
                // Have we read all of the packets in this sequence yet?
                //

                if (
                    this->fragmentsReceived == header->packetCount &&
                    this->packetData.tellp() == header->packetLength
                    )
                {
                    //
                    // Let's form a message. Form it with the sender's IP
                    // address and port, so any clients will know how to reply.
                    //

                    SensorPodMessageBuffer message(senderAddress.sin_addr.s_addr, ntohs(senderAddress.sin_port));

                    message.setType(header->type);

                    message.write((const unsigned char*) this->packetData.str().c_str(), header->packetLength);

                    //
                    // Send it to all of our children.
                    //

                    for (
                        std::vector<SensorPodCommunicationsListener>::iterator listenerIterator = this->listeners.begin();
                        listenerIterator != this->listeners.end();
                        ++listenerIterator
                        )
                    {
                        //
                        // Need to rewind the message to zero, in case they want to
                        // actually read from it.
                        //

                        message.seekg(0);

                        SensorPodCommunicationsListener& listener = *listenerIterator;

                        //
                        // Invoke the callback; but protect ourself in an exception
                        // handler in case they screw up.
                        //

                        try
                        {
                            listener.callback(message, listener.userData);
                        }
                        catch(crl::StandardException exception)
                        {
                            fprintf(stderr, "Exception while invoking a communications callback: %s.\n", exception.what());
                        }
                    }
                }
            }
        
        }

    } catch(...) {
        pthread_mutex_unlock(&this->lock);
        throw;
    }

    pthread_mutex_unlock(&this->lock);
}

/**
 * Publishes the given message out.
 *
 * \param message The message to publish.
 */
void SensorPodCommunications::publish(SensorPodMessageBuffer& message)
{
    //
    // Form the socket address we send on.
    //

    struct sockaddr_in address;

    memset(&address, 0, sizeof(struct sockaddr_in));

    address.sin_family = AF_INET;
    address.sin_port = htons((short) message.getSenderPort());
    address.sin_addr.s_addr = message.getSenderAddress();

    //
    // Get the next transmit sequence, atomically.
    //

    int localTransmitSequenceIdentifier = __sync_fetch_and_add(&this->transmitSequenceIdentifier, 1);

    //
    // Create an empty packet. We split on jumbo frame boundaries, in case
    // they actually have jumbo frames turned on.
    //

    std::string dataStream = message.getDataStream().str();

    const char* rawData = dataStream.c_str();
    int rawDataLength = dataStream.length();

    int packetIndex = 0;
    int offset = 0;

    // 
    // We need to lock here to avoid having another thread interrupt multi-fragment
    // messages.
    // 

    pthread_mutex_lock(&this->publishLock);
    try {

        while(offset < rawDataLength || rawDataLength == 0) {
            //
            // Pack objects into the 'packet', maxed out to the size
            // given.  We just walk through the size of the buffer
            // below -- so if you want to increase/decrease UDP packet
            // sizes (to, say, let IP fragmentation take more of the
            // load) just increase/decrease that.
            //

            SensorPodPacketHeader* header =
              (SensorPodPacketHeader*) this->outgoingBuffer;

            header->magic = 0xda;
            header->type = message.getType();

            //
            // How many bytes should we write? Don't go over our packet size!
            //

            int maximumPayloadSize =
              this->mtuSize - sizeof(SensorPodPacketHeader);

            int bytesToWrite = rawDataLength - offset;

            if(bytesToWrite > maximumPayloadSize)
                bytesToWrite = maximumPayloadSize;

            //
            // Fill in the rest of the header now.
            //

            header->sequenceIdentifier = localTransmitSequenceIdentifier;
            header->packetLength = rawDataLength;
            header->packetIdentifier = packetIndex;
            header->packetCount = (rawDataLength / maximumPayloadSize) + 1;

            //
            // Copy the data.
            //

            memcpy(&(this->outgoingBuffer)[sizeof(SensorPodPacketHeader)],
                   &rawData[offset], bytesToWrite);

            //
            // Advance pointers.
            //

            packetIndex++;
            offset += bytesToWrite;

            //
            // Send the result off!
            //

            int bytesWritten = sendto (
                                       this->serverSocket,
                                       this->outgoingBuffer,
                                       bytesToWrite + sizeof(SensorPodPacketHeader),
                                       0,
                                       (struct sockaddr *) &address,
                                       sizeof(address)
                                       );

            //
            // Sleep 1ms every 5 packets. This can add a lot of
            // latency for larger packet groups, but keeps the
            // switches and OS from getting overwhelmed by UDP data.
            // If we had Jumbo frames, this could be removed.
            //

            if((packetIndex % 5) == 0)
                usleep(100);

            //
            // If we had a problem writing, throw an exception.
            //

            if(bytesWritten !=
               (int) (bytesToWrite + sizeof(SensorPodPacketHeader))) {
                CRL_STANDARD_EXCEPTION(
	            "Error sending data to a remote host, wrote %d of %d bytes (%s).",
		    bytesWritten, bytesToWrite + sizeof(SensorPodPacketHeader),
		    strerror(errno));
	    }

            //
            // If raw data length was zero, bail out now. This is a special
            // case, when we have no data bytes to write.
            //

            if(rawDataLength == 0)
                break;
        }

    } catch(...) {
	pthread_mutex_unlock(&this->publishLock);
	throw;
    }
    pthread_mutex_unlock(&this->publishLock);
}

/**
 * This thread spins and waits for UDP packets, responding to them
 * as appropriate.
 */
void *SensorPodCommunications::dispatchThread(void *userData)
{
    SensorPodCommunications *self = (SensorPodCommunications *) userData;

    //
    // Loop forever, waiting for (and dispatching) messages.
    //

    fd_set errorSet;
    fd_set readSet;

    for(;;)
    {
        //
        // Add all of the appropriate sockets to their sets.
        //

        FD_ZERO(&errorSet);
        FD_ZERO(&readSet);

        self->addDescriptorsToSets(readSet, errorSet);

        //
        // Wait for a new packet to arrive.
        //

        int result = select(1024, &readSet, NULL, &errorSet, NULL);

        if(result < 0)
            continue;

        //
        // Let the communications object handle things.
        //

        self->handleSets(readSet, errorSet);
    }

    //
    // We'll never get here.
    //

    return NULL;
}
