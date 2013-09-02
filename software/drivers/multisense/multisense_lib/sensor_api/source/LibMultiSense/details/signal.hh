/**
 * @file LibMultiSense/details/signal.hh
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
 *   2013-05-07, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#ifndef LibMultiSense_details_signal_hh
#define LibMultiSense_details_signal_hh

#include "details/utility/Thread.hh"
#include "details/wire/Protocol.h"
#include "details/wire/AckMessage.h"

#include <map>

namespace crl {
namespace multisense {
namespace details {

//
// Here we provide a thread-safe, blocking, signaling
// interface for sensor message RX.

class MessageWatch {
public:

    void signal(wire::IdType id,
		Status       status=Status_Ok) {
        utility::ScopedLock lock(m_lock);

        Map::iterator it = m_map.find(id);

        if (m_map.end() != it)
            it->second->post(status);
    };

    void signal(const wire::Ack& ack) {
	signal(ack.command, ack.status);
    };

private:

    friend class ScopedWatch;

    typedef utility::WaitVar<Status>        Signal;
    typedef std::map<wire::IdType, Signal*> Map;

    void insert(wire::IdType type, 
		Signal      *signalP) {
        utility::ScopedLock lock(m_lock);

        Map::const_iterator it = m_map.find(type);

        //
        // Hmm.. this will prohibit multiple threads
        // simultaneously commanding the sensor with this 
	// message ID.

        if (m_map.end() != it)
            CRL_EXCEPTION("ack signal already set for id=%d", type);

        m_map[type] = signalP;
    };

    void remove(wire::IdType type) {
        utility::ScopedLock lock(m_lock);

        Map::iterator it = m_map.find(type);

        if (m_map.end() == it)
            CRL_EXCEPTION("ack signal not found for id=%d\n", type);

        m_map.erase(it);
    };

    utility::Mutex m_lock;
    Map            m_map;
};

 //
 // Exception-safe [de]registration of signal handlers

class ScopedWatch {
public:

    ScopedWatch(wire::IdType  t,
                MessageWatch& m) : m_id(t), m_map(m) {
	m_map.insert(m_id, &m_signal);
    };

    ~ScopedWatch() {
	m_map.remove(m_id);
    };

    bool wait(Status&       status, 
	      const double& timeout) {
	return m_signal.timedWait(status, timeout);
    };

private:

    wire::IdType         m_id;
    MessageWatch&        m_map;
    MessageWatch::Signal m_signal;
};

}; // namespace details
}; // namespace multisense
}; // namespace crl


#endif //  LibMultiSense_details_signal_hh
