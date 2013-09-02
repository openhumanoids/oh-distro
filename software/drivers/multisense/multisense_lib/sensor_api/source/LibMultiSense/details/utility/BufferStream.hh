/**
 * @file LibMultiSense/details/utility/BufferStream.hh
 *
 * A generic I/O stream buffer
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
 *   2013-03-01, ekratzer@carnegierobotics.com, PR1010, Created file.
 **/

#ifndef CRL_MULTISENSE_BUFFERSTREAM_HH
#define CRL_MULTISENSE_BUFFERSTREAM_HH

#include "Exception.hh"
#include "TimeStamp.hh"
#include "ReferenceCount.hh"

#include <stdint.h>
#include <cstddef>
#include <vector>

namespace crl {
namespace multisense {
namespace details {
namespace utility {

//
// The base storage class.
//
// To read/write from the stream, use the Reader/Writer
// derivatives below.
//
// SENSORPOD_FIRMWARE: microblaze build, no shared() interface

class BufferStream {
public:

    void        clear ()       { m_tell = 0;                  };
    std::size_t tell  () const { return m_tell;               };
    std::size_t size  () const { return m_size;               };
    void       *data  () const { return m_bufferP;            };
    void       *peek  () const { return &(m_bufferP[m_tell]); };

#ifndef SENSORPOD_FIRMWARE
    bool        shared() const { return m_ref.isShared();     };
#endif // SENSORPOD_FIRMWARE

    virtual void read (void *bufferP, std::size_t length) {
        CRL_EXCEPTION("not implemented");
    };
    virtual void write(const void *bufferP, std::size_t length) {
        CRL_EXCEPTION("not implemented");
    };

    //
    // Move the r/w pointer in the buffer, checking bounds

    void seek(std::size_t idx)  { 

        if (idx > m_size)
            CRL_EXCEPTION("invalid seek location %d, [0, %d] valid\n",
                          idx, m_size);
        m_tell = idx;
    };

    //
    // Default constructor

    BufferStream() :
        m_alloced(false),
        m_size(0),
        m_tell(0),
        m_bufferP(NULL) {};

    //
    // Construction, we allocate memory

    BufferStream(std::size_t size) : 
        m_alloced(false), 
        m_size(size),
        m_tell(0),
        m_bufferP(NULL) {

        m_bufferP = new (std::nothrow) uint8_t[size];
        if (NULL == m_bufferP)
            CRL_EXCEPTION("unable to allocate %d bytes", size);
        m_alloced = true;
    };
    
    //
    // Construction, memory is already allocated

    BufferStream(uint8_t *bufP, std::size_t size) : 
        m_alloced(false), 
        m_size(size), 
        m_tell(0),
        m_bufferP(bufP) {};

    //
    // Destruction, free memory only if we allocated

    virtual ~BufferStream() { 
#ifdef SENSORPOD_FIRMWARE
        if (m_alloced) 
#else
        if (m_alloced && false == m_ref.isShared()) 
#endif // SENSORPOD_FIRMWARE
            delete[] m_bufferP;
    };

    //
    // Copy constructor

    BufferStream(const BufferStream& source) {
        m_alloced = source.m_alloced;
        m_size    = source.m_size;
        m_tell    = 0;  // reset
        m_bufferP = source.m_bufferP;

#ifndef SENSORPOD_FIRMWARE
        m_ref     = source.m_ref;
#endif // SENSORPOD_FIRMWARE
    };

protected:

    bool         m_alloced;
    std::size_t  m_size;
    std::size_t  m_tell;
    uint8_t     *m_bufferP;

#ifndef SENSORPOD_FIRMWARE
    ReferenceCount m_ref;
#endif // SENSORPOD_FIRMWARE
};

//
// The input (deserialization) implementation. Must operate on
// non-const data.

class BufferStreamReader : public BufferStream {
public:

    BufferStreamReader() : BufferStream() {};
    BufferStreamReader(BufferStream& s) : BufferStream(s) {};
    BufferStreamReader(const uint8_t *p, std::size_t s) : BufferStream(const_cast<uint8_t*>(p), s) {};
    BufferStreamReader(std::size_t s) : BufferStream(s) {};

    virtual void read (void *bufferP, std::size_t length) {

        if (length > (m_size - m_tell))
            CRL_EXCEPTION("read overflow: tell=%d, size=%d, length=%d\n",
                          m_tell, m_size, length);

        memcpy(bufferP, &(m_bufferP[m_tell]), length);
        m_tell += length;
    };

    template <typename T> BufferStreamReader& operator&(T &value) {
        this->read(&value, sizeof(T));
        return *this;
    };

    template <typename T> BufferStreamReader& operator&(std::vector<T>& v) {
        uint32_t num;
        v.clear();
        this->read(&num, sizeof(num));
        for(uint32_t i=0; i<num; i++) {
            T data;
            *this & data;
            v.push_back(data);
        }
        return *this;
    }

    BufferStreamReader& operator&(std::string& value) {
        uint16_t length;

        this->read(&length, sizeof(length));
        if (length > 512)
            CRL_EXCEPTION("unusually large string: %d bytes",
                                   length);
        else if (length > 0) {
            char buffer[length+1];
            buffer[length] = '\0';
            this->read(buffer, length);
            value = std::string(buffer);
        }
        return *this;
    };

    BufferStreamReader& operator&(TimeStamp& value) {
        uint32_t seconds;
        uint32_t microseconds;

        this->read(&seconds, sizeof(seconds));
        this->read(&microseconds, sizeof(microseconds));

        value = seconds + 1e-6 * microseconds;

        return *this;
    };
};

//
// The output (serialization) implementation. Able to operate
// purely on const data.

class BufferStreamWriter : public BufferStream {
public:

    BufferStreamWriter() : BufferStream() {};
    BufferStreamWriter(BufferStream& s) : BufferStream(s) {};
    BufferStreamWriter(uint8_t *b, std::size_t s) : BufferStream(b, s) {};
    BufferStreamWriter(std::size_t s) : BufferStream(s) {};

    virtual void write(const void *bufferP, std::size_t length) {

        if ((length + m_tell) > m_size)
            CRL_EXCEPTION("write overflow: tell=%d, size=%d, length=%d\n",
                          m_tell, m_size, length);

        memcpy(&(m_bufferP[m_tell]), bufferP, length);
        m_tell += length;
    };

    template <typename T> BufferStreamWriter& operator&(const T& value) {
        this->write(&value, sizeof(T));
        return *this;
    };

    template <typename T> BufferStreamWriter& operator&(const std::vector<T>& v) {
        uint32_t num = v.size();
        this->write(&num, sizeof(num));
        for(uint32_t i=0; i<num; i++)
            *this & v[i];
        return *this;
    }

    BufferStreamWriter& operator&(const std::string& value) {
        uint16_t length = value.size();

        if (length > 512)
            CRL_EXCEPTION("unusually large string: %d bytes", length);

        this->write(&length, sizeof(length));
        if (length > 0)
            this->write(value.c_str(), length);
        return *this;
    };

    BufferStreamWriter& operator&(const TimeStamp& value) {
        const uint32_t seconds      = value.getSeconds();
        const uint32_t microseconds = value.getMicroSeconds();

        this->write(&seconds, sizeof(seconds));
        this->write(&microseconds, sizeof(microseconds));

        return *this;
    };
};

}}}} // namespaces

#endif /* #ifndef CRL_MULTISENSE_BUFFERSTREAM_HH */
