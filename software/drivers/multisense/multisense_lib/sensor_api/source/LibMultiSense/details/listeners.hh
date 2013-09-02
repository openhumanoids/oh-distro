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

extern __thread utility::BufferStream *dispatchBufferReferenceTP;

//
// TODO: combine ImageListener and LaserListener via templates or inheritance, 
//       much of the code is shared between them.

//
// Represents a bound listener to an image stream

class ImageListener {
public:

    ImageListener(image::Callback c,
                  DataSource      s,
                  void           *d,
                  uint32_t        m=0) // max queue entries (0 == infinite)
        : m_callback(c),
          m_sourceMask(s),
          m_userDataP(d),
          m_running(false),
          m_queue(m),
          m_dispatchThreadP(NULL) {
        
        m_running         = true;
        m_dispatchThreadP = new utility::Thread(dispatchThread, this);
    };

    ImageListener() : 
        m_callback(NULL),
        m_sourceMask(0),
        m_userDataP(NULL),
        m_running(false),
        m_queue(),
        m_dispatchThreadP(NULL) {};

    ~ImageListener() {
        if (m_running) {
            m_running = false;
            m_queue.kick();
            delete m_dispatchThreadP;
        }
    };

    void dispatch(utility::BufferStream& buffer,
                  image::Header&         header,
                  void                  *imageP) {

        if (header.source & m_sourceMask)
            m_queue.post(Dispatch(m_callback,
                                  buffer,
                                  header,
                                  imageP,
                                  m_userDataP));
    };

    image::Callback callback() { return m_callback; };

private:

    //
    // For thread-safe dispatching

    class Dispatch {
    public:

        Dispatch(image::Callback        c,
                 utility::BufferStream& b,
                 image::Header&         h,
                 void                  *i,
                 void                  *d) :
            m_callback(c),
            m_buffer(b),
            m_header(h),
            m_imageP(i),
            m_userDataP(d) {};

        Dispatch() :
            m_callback(NULL),
            m_buffer(),
            m_header(),
            m_imageP(NULL),
            m_userDataP(NULL) {};

        void operator() (void) {

            if (m_callback) {

                dispatchBufferReferenceTP = &m_buffer;

                m_callback(m_header,
                           m_imageP,
                           m_userDataP);
            }
        };
    
    private:

        image::Callback       m_callback;
        utility::BufferStream m_buffer;
        image::Header         m_header;
        void                 *m_imageP;
        void                 *m_userDataP;
    };

    //
    // The dispatch thread
    //
    // We are penalized with two memory copies of
    // image::Header by std::deque, but the 
    // image data is zero-copy (reference-counted by
    // BufferStream)

    static void *dispatchThread(void *argumentP) {
        
        ImageListener *selfP = reinterpret_cast<ImageListener*>(argumentP);

        while(selfP->m_running) {
            try {
                Dispatch d;
                if (false == selfP->m_queue.wait(d))
                    break;
                d();
            } catch (const utility::Exception& e) {
                CRL_DEBUG("exception invoking image callback: %s\n",
                          e.what());
            } catch ( ... ) {
                CRL_DEBUG("unknown exception invoking image callback\n");
            }
        }

        return NULL;
    };

    //
    // Set by user

    image::Callback m_callback;
    DataSource      m_sourceMask;
    void           *m_userDataP;

    //
    // Dispatch mechanism
    
    volatile bool                m_running;
    utility::WaitQueue<Dispatch> m_queue;
    utility::Thread             *m_dispatchThreadP;
};

//
// Represents a bound listener to a laser stream

class LidarListener {
public:

    LidarListener(lidar::Callback c,
                  void           *d,
                  uint32_t        m=0) // max queue size (0 == infinite)
        : m_callback(c),
          m_userDataP(d),
          m_running(false),
          m_queue(m),
          m_dispatchThreadP(NULL) {

        m_running         = true;
        m_dispatchThreadP = new utility::Thread(dispatchThread, this);
    };

    LidarListener() : 
        m_callback(NULL),
        m_userDataP(NULL),
        m_running(false),
        m_queue(),
        m_dispatchThreadP(NULL) {};

    ~LidarListener() {
        if (m_running) {
            m_running = false;
            m_queue.kick();
            delete m_dispatchThreadP;
        }
    };

    void dispatch(utility::BufferStream& buffer,
                  lidar::Header&         header,
                  lidar::RangeType      *rangesP,
                  lidar::IntensityType  *intensitiesP) {

        m_queue.post(Dispatch(m_callback,
                              buffer,
                              header,
                              rangesP,
                              intensitiesP,
                              m_userDataP));
    };

    lidar::Callback callback() { return m_callback; };

private:
    
    //
    // For thread-safe dispatching

    class Dispatch {
    public:

        Dispatch(lidar::Callback        c,
                 utility::BufferStream& b,
                 lidar::Header&         h,
                 lidar::RangeType      *r,
                 lidar::IntensityType  *i,
                 void                  *d) :
            m_callback(c),
            m_buffer(b),
            m_header(h),
            m_rangesP(r),
            m_intensitiesP(i),
            m_userDataP(d) {};

        Dispatch() :
            m_callback(NULL),
            m_buffer(),
            m_header(),
            m_rangesP(NULL),
            m_intensitiesP(NULL),
            m_userDataP(NULL) {};

        void operator() (void) {

            if (m_callback) {

                dispatchBufferReferenceTP = &m_buffer;

                m_callback(m_header,
                           m_rangesP,
                           m_intensitiesP,
                           m_userDataP);
            }
        };
    
    private:

        lidar::Callback       m_callback;
        utility::BufferStream m_buffer;
        lidar::Header         m_header;
        lidar::RangeType     *m_rangesP;
        lidar::IntensityType *m_intensitiesP;
        void                 *m_userDataP;
    };

    //
    // The dispatch thread
    //
    // We are penalized with two memory copies of
    // lidar::Header by std::deque, but the 
    // laser data is zero-copy (reference-counted by
    // BufferStream)

    static void *dispatchThread(void *argumentP) {
        
        LidarListener *selfP = reinterpret_cast<LidarListener*>(argumentP);

        while(selfP->m_running) {
            try {
                Dispatch d;
                if (false == selfP->m_queue.wait(d))
                    break;
                d();
            } catch (const utility::Exception& e) {
                CRL_DEBUG("exception invoking laser callback: %s\n",
                          e.what());
            } catch ( ... ) {
                CRL_DEBUG("unknown exception invoking lidar callback\n");
            }
        }

        return NULL;
    };

    //
    // Set by user

    lidar::Callback m_callback;
    void           *m_userDataP;

    //
    // Dispatch mechanism
    
    volatile bool                m_running;
    utility::WaitQueue<Dispatch> m_queue;
    utility::Thread             *m_dispatchThreadP;
};

}; // namespace details
}; // namespace multisense
}; // namespace crl


#endif //  LibMultiSense_impl_listeners
