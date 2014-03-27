/**
 * @file LibMultiSense/details/listeners.hh
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

#ifndef LibMultiSense_impl_listeners
#define LibMultiSense_impl_listeners

#include "MultiSenseTypes.hh"

#include "details/utility/Thread.hh"
#include "details/utility/BufferStream.hh"

namespace crl {
namespace multisense {
namespace details {

//
// For access to a buffer back-end in a dispatch thread

extern __thread utility::BufferStream *dispatchBufferReferenceTP;

//
// The dispatch mechanism. Each instance represents a bound
// listener to a datum stream.

template<class HEADER, class CALLBACK>
class Listener {
public:
    
    Listener(CALLBACK   c,
             DataSource s,
             void      *d,
             uint32_t   m=0)
        : m_callback(c),
          m_sourceMask(s),
          m_userDataP(d),
          m_running(false),
          m_queue(m),
          m_dispatchThreadP(NULL) {
        
        m_running         = true;
        m_dispatchThreadP = new utility::Thread(dispatchThread, this);
    };

    Listener() :
        m_callback(NULL),
        m_sourceMask(0),
        m_userDataP(NULL),
        m_running(false),
        m_queue(),
        m_dispatchThreadP(NULL) {};

    ~Listener() {
        if (m_running) {
            m_running = false;
            m_queue.kick();
            delete m_dispatchThreadP;
        }
    };

    void dispatch(HEADER& header) {

        if (header.inMask(m_sourceMask))
            m_queue.post(Dispatch(m_callback,
                                  header,
                                  m_userDataP));
    };

    void dispatch(utility::BufferStream& buffer,
                  HEADER&                header) {

        if (header.inMask(m_sourceMask))
            m_queue.post(Dispatch(m_callback,
                                  buffer,
                                  header,
                                  m_userDataP));
    };

    CALLBACK callback() { return m_callback; };

private:

    //
    // For thread-safe dispatching

    class Dispatch {
    public:

        Dispatch(CALLBACK  c,
                 HEADER&   h,
                 void     *d) :
            m_callback(c),
            m_exposeBuffer(false),
            m_header(h),
            m_userDataP(d) {};

        Dispatch(CALLBACK               c,
                 utility::BufferStream& b,
                 HEADER&                h,
                 void                  *d) :
            m_callback(c),
            m_buffer(b),
            m_exposeBuffer(true),
            m_header(h),
            m_userDataP(d) {};

        Dispatch() :
            m_callback(NULL),
            m_buffer(),
            m_exposeBuffer(false),
            m_header(),
            m_userDataP(NULL) {};

        void operator() (void) {

            if (m_callback) {
                if (m_exposeBuffer)
                    dispatchBufferReferenceTP = &m_buffer;
                m_callback(m_header, m_userDataP);
            }
        };

    private:

        CALLBACK              m_callback;
        utility::BufferStream m_buffer;
        bool                  m_exposeBuffer;
        HEADER                m_header;
        void                 *m_userDataP;
    };

    //
    // The dispatch thread
    //
    // We are penalized with two memory copies of
    // HEADER by std::deque, but the image/lidar data
    // is zero-copy (reference-counted by BufferStream)

    static void *dispatchThread(void *argumentP) {
        
        Listener<HEADER,CALLBACK> *selfP = reinterpret_cast< Listener<HEADER,CALLBACK> * >(argumentP);
    
        while(selfP->m_running) {
            try {
                Dispatch d;
                if (false == selfP->m_queue.wait(d))
                    break;
                d();
            } catch (const std::exception& e) {
                CRL_DEBUG("exception invoking image callback: %s\n",
                          e.what());
            } catch ( ... ) {
                CRL_DEBUG("unknown exception invoking image callback\n");
            }
        };

        return NULL;
    }

    //
    // Set by user

    CALLBACK   m_callback;
    DataSource m_sourceMask;
    void      *m_userDataP;

    //
    // Dispatch mechanism
    
    volatile bool                m_running;
    utility::WaitQueue<Dispatch> m_queue;
    utility::Thread             *m_dispatchThreadP;
};

typedef Listener<image::Header, image::Callback> ImageListener;
typedef Listener<lidar::Header, lidar::Callback> LidarListener;
typedef Listener<pps::Header,   pps::Callback>   PpsListener;
typedef Listener<imu::Header,   imu::Callback>   ImuListener;

}; // namespace details
}; // namespace multisense
}; // namespace crl


#endif //  LibMultiSense_impl_listeners
