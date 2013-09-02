/**
 * @file LibMultiSense/details/query.hh
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
 *   2013-05-15, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#ifndef _LibMultiSense_details_query_hh
#define _LibMultiSense_details_query_hh

#include "details/channel.hh"

namespace crl {
namespace multisense {
namespace details {

//
// Publishes the given message to the sensor (message must
// serialize into a single MTU)

template<class T> void impl::publish(const T& message)
{
    //
    // An output stream to serialize the data

    utility::BufferStreamWriter stream(m_sensorMtu - 
                                       wire::COMBINED_HEADER_LENGTH);

    //
    // Hide the header area

    stream.seek(sizeof(wire::Header));

    //
    // Set the ID and version
    
    stream & MSG_ID(T::ID);
    stream & MSG_VER(T::VERSION);
    
    //
    // Add the message payload. We cast away const here because
    // we have a single serialize() for both directions, and cannot
    // mark it const.

    const_cast<T*>(&message)->serialize(stream, 0);

    //
    // Publish the stream

    publish(stream);
}

//
// Send a message, wait for a particular repsonse, re-trying if
// necessary

template <class T> Status impl::waitAck(const T&      msg, 
                                        wire::IdType  ackId,
                                        const double& timeout,
                                        int32_t       attempts)
{
    try {
        ScopedWatch ack(ackId, m_watch);

        while(attempts-- > 0) {

            publish(msg);
            
            Status status;
            if (false == ack.wait(status, timeout))
                continue;
            else
                return status;
        }

        return Status_TimedOut;

    } catch (const utility::Exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Error;
    }
}

//
// Send a message (with retry), expecting data message, 
// extract data payload for user

template <class T, class U> Status impl::waitData(const T&      command,
                                                  U&            data,
                                                  const double& timeout,
                                                  int32_t       attempts)
{
    Status status = waitAck(command, MSG_ID(U::ID), timeout, attempts);
    if (Status_Ok != status)
        return status;

    try {
        
        return m_messages.extract(data);
        
    } catch (const utility::Exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Error;
    }
}

}}}; // namespaces

#endif // _LibMultiSense_details_publish_hh
