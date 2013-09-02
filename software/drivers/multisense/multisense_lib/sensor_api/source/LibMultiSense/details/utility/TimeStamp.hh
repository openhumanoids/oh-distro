/**
 * @file LibMultiSense/details/utility/TimeStamp.hh
 *
 * The timestamp class gives some type-safety and helper routines for
 * managing time stamps.  This is derived from Dan Tascione's and Eric
 * Kratzer's TimeStamp class, which was developed under project
 * RD1034.
 *
 * Copyright 2011
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
 *   2012-08-14, dlr@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef CRL_MULTISENSE_TIMESTAMP_HH
#define CRL_MULTISENSE_TIMESTAMP_HH

#include <sys/time.h>
#include <stdint.h>

namespace crl {
namespace multisense {
namespace details {
namespace utility {

//
// This is a simple class that helps manage time for the rest of the system,
// abstracting it away into something that is more usable.
//

class TimeStamp
{
private:

    //
    // The stored time.
    //

    struct timeval time;

    //
    // The time synchronization offset. This offset is recalculated each time
    // we receive a PPS timestamp set. It is applied to all times returned by
    // calling getCurrentTime().
    //

    static double timeSynchronizationOffset;

public:

    //
    // Static routines, for the singleton version of the TimeStamp.
    //

    static TimeStamp getCurrentTime();
    static TimeStamp getMonotonicTime();

    static void setTimeAtPps(TimeStamp& local, TimeStamp& remote);
    static void setTimeAtPps(struct timeval& local, struct timeval& remote);
    static double getTimeSynchronizationOffset();

    //
    // Public constructor. Initializes from a timestamp.
    //

    TimeStamp();
    TimeStamp(struct timeval& value);
    TimeStamp(double value);

    //
    // For setting the timestamp.
    //

    void set(struct timeval& value);

    //
    // For getting precise values from the timestamp.
    //

    uint32_t getSeconds() const;
    uint32_t getMicroSeconds() const;

    //
    // Operator overloads, for working with time.
    //

    operator double() const;
    TimeStamp& operator=(double timeStamp);
    TimeStamp& operator+=(TimeStamp const& other);
    TimeStamp& operator-=(TimeStamp const& other);
    
    //
    // Make sure internal state is consistent.  Use this if you set
    // the TimeStamp using a timeval struct that had an unchecked
    // microseconds value, such as a negative number.
    //

    void normalize();
};


//
// Arithmetic operators are mostly handled by implicit conversion to
// and from double.
//

// TimeStamp operator +(TimeStamp const& arg0, TimeStamp const& arg1);
// TimeStamp operator -(TimeStamp const& arg0, TimeStamp const& arg1);
  
}}}} // namespaces

#endif /* #ifndef CRL_MULTISENSE_TIMESTAMP_HH */
