/**
 * @file LibMultiSense/details/utility/TimeStamp.cc
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

#include "TimeStamp.hh"

#include <sys/time.h>
#include <time.h>

namespace crl {
namespace multisense {
namespace details {
namespace utility {


// Initialize static routines.

double TimeStamp::timeSynchronizationOffset = 0.0;

/*
 * Constructor. Empty. We rely on the getter methods to do
 * things that are more useful.
 */
TimeStamp::TimeStamp()
{
    this->time.tv_sec = 0;
    this->time.tv_usec = 0;
}

/*
 * Constructor. Initializes with the specified timestamp value.
 */
TimeStamp::TimeStamp(struct timeval& value)
{
    this->set(value);
}

/*
 * Cosntructor. Initializes with the specified timestamp value.
 */
TimeStamp::TimeStamp(double value)
{
    this->time.tv_sec = (int) value;
    this->time.tv_usec = (int) ((value - this->time.tv_sec) * 1000000);
}

/*
 * Sets this timestamp equal to the timestamp specified.
 */
void TimeStamp::set(struct timeval& value)
{
    this->time.tv_sec = value.tv_sec;
    this->time.tv_usec = value.tv_usec;
}

/*
 * Sets the time, for time synchronization. This will report the local clock when
 * the PPS event occurred, and the remote clock when the PPS event occurred.
 *
 * When you use the routine 'getCurrentTime()', it will then return a timestamp
 * that is time-synchronized with the remote system.
 */
void TimeStamp::setTimeAtPps(TimeStamp& local, TimeStamp& remote)
{
    setTimeAtPps(local.time, remote.time);
}

/*
 * Sets the time, for time synchronization. This will report the local clock when
 * the PPS event occurred, and the remote clock when the PPS event occurred.
 *
 * When you use the routine 'getCurrentTime()', it will then return a timestamp
 * that is time-synchronized with the remote system.
 */
void TimeStamp::setTimeAtPps(struct timeval& local, struct timeval& remote)
{
    //
    // To make things atomic, we are somewhat in trouble. To make things lock-free,
    // we are going to do all of the math as doubles. Convert the above timestamps
    // to doubles.
    //

    double localTimeAtPps = local.tv_sec + (local.tv_usec / 1000000.0);
    double remoteTimeAtPps = remote.tv_sec + (remote.tv_usec / 1000000.0);

    //
    // Store the offset between the two as a static variable.
    //

    timeSynchronizationOffset = remoteTimeAtPps - localTimeAtPps;
}

/*
 * Returns the offset from the remote clock to the local clock. Add this to the
 * time returned by a standard time call to have a synchronized time. Notice that
 * this is already applied to anything using TimeStamps normally.
 */
double TimeStamp::getTimeSynchronizationOffset()
{
    return timeSynchronizationOffset;
}

#ifndef SENSORPOD_FIRMWARE

/*
 * This routine will get the current time (as gettimeofday()) and
 * store it off. It is the normal way of initializing time. Notice
 * that there may be large time skips when you call this routine, due
 * to time synchronization jumps.
 *
 * Notice that the timestamp returned by this object *is* atomic. It
 * will not change, even if time synchronization skews things.
 */
TimeStamp TimeStamp::getCurrentTime()
{
    //
    // Create a new timestamp object and fill it in with
    // the current time of day.
    //

    TimeStamp timeStamp;

    gettimeofday(&timeStamp.time, 0);

    //
    // Transform it into a double... Notice that this (quite handily)
    // removes all precision up to the 100-nanosecond mark, making carrying
    // times around as timevals fairly pointless. We do this to apply the
    // time synchronization offset, lockless-ly.
    //
    // It's probably that this is pointless, and we should add a lock
    // in the future.
    //

    double currentTime = (double) timeStamp;

    currentTime += timeSynchronizationOffset;

    timeStamp = currentTime;

    //
    // Return the final timestamp.
    //

    return timeStamp;
}

/*
 * Returns the monotonic time. This clock is not affected by time synchronization
 * and should never time-jump. It will be completely unrelated to the normal wall
 * time, though.
 */
TimeStamp TimeStamp::getMonotonicTime()
{
    //
    // Retrieve the monotonic time, in nanoseconds.
    //

    struct timespec time = { 0 };

    clock_gettime(CLOCK_MONOTONIC, &time);

    //
    // Copy into a timeval, taking care to convert nanoseconds
    // into microseconds.
    //

    TimeStamp timeStamp;

    timeStamp.time.tv_sec = time.tv_sec;
    timeStamp.time.tv_usec = time.tv_nsec / 1000;

    return timeStamp;
}

#endif // SENSORPOD_FIRMWARE

/*
 * Returns the seconds portion of the timestamp.
 */
uint32_t TimeStamp::getSeconds() const
{
    return this->time.tv_sec;
}

/*
 * Returns the microseconds portion of the timestamp.
 */
uint32_t TimeStamp::getMicroSeconds() const
{
    return this->time.tv_usec;
}

/*
 * Returns the stored time as if it was a double. This will be seconds since 1970.
 * The time will be accurate to the (roughly) 100-nanosecond mark.
 */
TimeStamp::operator double() const
{
    return this->time.tv_sec + (this->time.tv_usec / 1000000.0);
}

/*
 * Sets the stored time from the seconds value given.
 */
TimeStamp& TimeStamp::operator=(double timeStamp)
{
    this->time.tv_sec = ((unsigned long long) timeStamp);
    this->time.tv_usec = ((unsigned long long) ((timeStamp - this->time.tv_sec) * 1000000));

    // This call avoids having negative microseconds if the input
    // argument is less than zero and non-integral.
    this->normalize();
    
    return *this;
}


TimeStamp& TimeStamp::operator+=(TimeStamp const& other)
{
    this->time.tv_sec += other.time.tv_sec;
    this->time.tv_usec += other.time.tv_usec;
    this->normalize();
    return *this;
}
  

TimeStamp& TimeStamp::operator-=(TimeStamp const& other)
{
    this->time.tv_sec -= other.time.tv_sec;
    this->time.tv_usec -= other.time.tv_usec;
    this->normalize();
    return *this;
}

  
void TimeStamp::normalize()
{
    while(this->time.tv_usec < 0) {
        this->time.tv_usec += 1000000;
        this->time.tv_sec -= 1;
    }

    while(this->time.tv_usec >= 1000000) {
        this->time.tv_usec -= 1000000;
        this->time.tv_sec += 1;
    }
}


//
// Arithmetic operators are mostly handled by implicit conversion to
// and from double.
//

// TimeStamp operator +(TimeStamp const& arg0, TimeStamp const& arg1)
// {
//   TimeStamp result = arg0;
//   result += arg1;
//   return result;
// }

  
// TimeStamp operator -(TimeStamp const& arg0, TimeStamp const& arg1)
// {
//   TimeStamp result = arg0;
//   result -= arg1;
//   return result;
// }
  
}}}} // namespaces
